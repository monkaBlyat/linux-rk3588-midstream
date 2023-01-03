// SPDX-License-Identifier: GPL-2.0
/* Copyright 2018 Marty E. Plummer <hanetzer@startmail.com> */
/* Copyright 2019 Linaro, Ltd., Rob Herring <robh@kernel.org> */
/* Copyright 2019 Collabora ltd. */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "pancsf_device.h"
#include "pancsf_gpu.h"
#include "pancsf_regs.h"

#define MAX_HW_REVS 6

struct pancsf_gpu {
	int irq;
	spinlock_t reqs_lock;
	u32 pending_reqs;
	wait_queue_head_t reqs_acked;
};

struct pancsf_model {
	const char *name;
	u32 id;
};

#define GPU_MODEL(_name, _id, ...) \
{\
	.name = __stringify(_name),				\
	.id = _id,						\
}

#define GPU_MODEL_ID_MASK		0xf00f0000

static const struct pancsf_model gpu_models[] = {
	GPU_MODEL(g610, 0xa0070000),
	{},
};

static void pancsf_gpu_init_info(struct pancsf_device *pfdev)
{
	const struct pancsf_model *model;
	u32 major, minor, status;
	unsigned int i;

	pfdev->gpu_info.gpu_id = gpu_read(pfdev, GPU_ID);
	pfdev->gpu_info.csf_id = gpu_read(pfdev, GPU_CSF_ID);
	pfdev->gpu_info.gpu_rev = gpu_read(pfdev, GPU_REVID);
	pfdev->gpu_info.l2_features = gpu_read(pfdev, GPU_L2_FEATURES);
	pfdev->gpu_info.tiler_features = gpu_read(pfdev, GPU_TILER_FEATURES);
	pfdev->gpu_info.mem_features = gpu_read(pfdev, GPU_MEM_FEATURES);
	pfdev->gpu_info.mmu_features = gpu_read(pfdev, GPU_MMU_FEATURES);
	pfdev->gpu_info.thread_features = gpu_read(pfdev, GPU_THREAD_FEATURES);
	pfdev->gpu_info.max_threads = gpu_read(pfdev, GPU_THREAD_MAX_THREADS);
	pfdev->gpu_info.thread_max_workgroup_size = gpu_read(pfdev, GPU_THREAD_MAX_WORKGROUP_SIZE);
	pfdev->gpu_info.thread_max_barrier_size = gpu_read(pfdev, GPU_THREAD_MAX_BARRIER_SIZE);
	pfdev->gpu_info.coherency_features = gpu_read(pfdev, GPU_COHERENCY_FEATURES);
	for (i = 0; i < 4; i++)
		pfdev->gpu_info.texture_features[i] = gpu_read(pfdev, GPU_TEXTURE_FEATURES(i));

	pfdev->gpu_info.as_present = gpu_read(pfdev, GPU_AS_PRESENT);

	pfdev->gpu_info.shader_present = gpu_read(pfdev, GPU_SHADER_PRESENT_LO);
	pfdev->gpu_info.shader_present |= (u64)gpu_read(pfdev, GPU_SHADER_PRESENT_HI) << 32;

	pfdev->gpu_info.tiler_present = gpu_read(pfdev, GPU_TILER_PRESENT_LO);
	pfdev->gpu_info.tiler_present |= (u64)gpu_read(pfdev, GPU_TILER_PRESENT_HI) << 32;

	pfdev->gpu_info.l2_present = gpu_read(pfdev, GPU_L2_PRESENT_LO);
	pfdev->gpu_info.l2_present |= (u64)gpu_read(pfdev, GPU_L2_PRESENT_HI) << 32;
	pfdev->gpu_info.core_group_count = hweight64(pfdev->gpu_info.l2_present);

	major = (pfdev->gpu_info.gpu_id >> 12) & 0xf;
	minor = (pfdev->gpu_info.gpu_id >> 4) & 0xff;
	status = pfdev->gpu_info.gpu_id & 0xf;

	for (model = gpu_models; model->name; model++) {
		if (model->id == (pfdev->gpu_info.gpu_id & GPU_MODEL_ID_MASK))
			break;
	}

	dev_info(pfdev->dev, "mali-%s id 0x%x major 0x%x minor 0x%x status 0x%x",
		 model->name ?: "unknown", pfdev->gpu_info.gpu_id >> 16,
		 major, minor, status);

	dev_info(pfdev->dev, "Features: L2:0x%08x Tiler:0x%08x Mem:0x%0x MMU:0x%08x AS:0x%x",
		 pfdev->gpu_info.l2_features,
		 pfdev->gpu_info.tiler_features,
		 pfdev->gpu_info.mem_features,
		 pfdev->gpu_info.mmu_features,
		 pfdev->gpu_info.as_present);

	dev_info(pfdev->dev, "shader_present=0x%0llx l2_present=0x%0llx tiler_present=0x%0llx",
		 pfdev->gpu_info.shader_present, pfdev->gpu_info.l2_present,
		 pfdev->gpu_info.tiler_present);
}

static irqreturn_t pancsf_gpu_irq_handler(int irq, void *data)
{
	struct pancsf_device *pfdev = data;
	u32 state = gpu_read(pfdev, GPU_INT_STAT);

	if (!state)
		return IRQ_NONE;

	if (state & (GPU_IRQ_FAULT | GPU_IRQ_PROTM_FAULT)) {
		u32 fault_status = gpu_read(pfdev, GPU_FAULT_STATUS);
		u64 address = ((u64)gpu_read(pfdev, GPU_FAULT_ADDR_HI) << 32) |
			      gpu_read(pfdev, GPU_FAULT_ADDR_LO);

		dev_warn(pfdev->dev, "GPU Fault 0x%08x (%s) at 0x%016llx\n",
			 fault_status, pancsf_exception_name(fault_status & 0xFF),
			 address);
	}

	spin_lock(&pfdev->gpu->reqs_lock);
	if (state & pfdev->gpu->pending_reqs) {
		pfdev->gpu->pending_reqs &= ~state;
		wake_up_all(&pfdev->gpu->reqs_acked);
	}
	spin_unlock(&pfdev->gpu->reqs_lock);

	gpu_write(pfdev, GPU_INT_CLEAR, state);
	return IRQ_HANDLED;
}

void pancsf_gpu_fini(struct pancsf_device *pfdev)
{
	unsigned long flags;

	gpu_write(pfdev, GPU_INT_MASK, 0);

	if (pfdev->gpu->irq > 0)
		synchronize_irq(pfdev->gpu->irq);

	spin_lock_irqsave(&pfdev->gpu->reqs_lock, flags);
	pfdev->gpu->pending_reqs = 0;
	wake_up_all(&pfdev->gpu->reqs_acked);
	spin_unlock_irqrestore(&pfdev->gpu->reqs_lock, flags);
}

int pancsf_gpu_init(struct pancsf_device *pfdev)
{
	struct pancsf_gpu *gpu;
	u32 pa_bits;
	int ret, irq;

	gpu = devm_kzalloc(pfdev->dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	spin_lock_init(&gpu->reqs_lock);
	init_waitqueue_head(&gpu->reqs_acked);
	pfdev->gpu = gpu;
	pancsf_gpu_init_info(pfdev);

	dma_set_max_seg_size(pfdev->dev, UINT_MAX);
	pa_bits = GPU_MMU_FEATURES_PA_BITS(pfdev->gpu_info.mmu_features);
	ret = dma_set_mask_and_coherent(pfdev->dev, DMA_BIT_MASK(pa_bits));
	if (ret)
		return ret;

	gpu_write(pfdev, GPU_INT_CLEAR, ~0);
	gpu_write(pfdev, GPU_INT_MASK,
		  GPU_IRQ_FAULT |
		  GPU_IRQ_PROTM_FAULT |
		  GPU_IRQ_RESET_COMPLETED |
		  GPU_IRQ_MCU_STATUS_CHANGED |
		  GPU_IRQ_CLEAN_CACHES_COMPLETED);

	irq = platform_get_irq_byname(to_platform_device(pfdev->dev), "gpu");
	gpu->irq = irq;
	if (irq <= 0)
		return -ENODEV;

	ret = devm_request_irq(pfdev->dev, irq,
			       pancsf_gpu_irq_handler,
			       IRQF_SHARED, KBUILD_MODNAME "-gpu",
			       pfdev);
	if (ret)
		return ret;

	return 0;
}

int pancsf_gpu_block_power_off(struct pancsf_device *pfdev,
			       const char *blk_name,
			       u32 pwroff_reg, u32 pwrtrans_reg,
			       u64 mask, u32 timeout_us)
{
	u32 val, i;
	int ret;

	for (i = 0; i < 2; i++) {
		u32 mask32 = mask >> (i * 32);

		if (!mask32)
			continue;

		ret = readl_relaxed_poll_timeout(pfdev->iomem + pwrtrans_reg + (i * 4),
						 val, !(mask32 & val),
						 100, timeout_us);
		if (ret) {
			dev_err(pfdev->dev, "timeout waiting on %s:%llx power transition",
				blk_name, mask);
			return ret;
		}
	}

	if (mask & GENMASK(31, 0))
		gpu_write(pfdev, pwroff_reg, mask);

	if (mask >> 32)
		gpu_write(pfdev, pwroff_reg, mask >> 32);

	for (i = 0; i < 2; i++) {
		u32 mask32 = mask >> (i * 32);

		if (!mask32)
			continue;

		ret = readl_relaxed_poll_timeout(pfdev->iomem + pwrtrans_reg + (i * 4),
						 val, !(mask & val),
						 100, timeout_us);
		if (ret) {
			dev_err(pfdev->dev, "timeout waiting on %s:%llx power transition",
				blk_name, mask);
			return ret;
		}
	}

	return 0;
}

int pancsf_gpu_block_power_on(struct pancsf_device *pfdev,
			      const char *blk_name,
			      u32 pwron_reg, u32 pwrtrans_reg,
			      u32 rdy_reg, u64 mask, u32 timeout_us)
{
	u32 val, i;
	int ret;

	for (i = 0; i < 2; i++) {
		u32 mask32 = mask >> (i * 32);

		if (!mask32)
			continue;

		ret = readl_relaxed_poll_timeout(pfdev->iomem + pwrtrans_reg + (i * 4),
						 val, !(mask32 & val),
						 100, timeout_us);
		if (ret) {
			dev_err(pfdev->dev, "timeout waiting on %s:%llx power transition",
				blk_name, mask);
			return ret;
		}
	}

	if (mask & GENMASK(31, 0))
		gpu_write(pfdev, pwron_reg, mask);

	if (mask >> 32)
		gpu_write(pfdev, pwron_reg + 4, mask >> 32);

	for (i = 0; i < 2; i++) {
		u32 mask32 = mask >> (i * 32);

		if (!mask32)
			continue;

		ret = readl_relaxed_poll_timeout(pfdev->iomem + rdy_reg + (i * 4),
						 val, (mask32 & val) == mask32,
						 100, timeout_us);
		if (ret) {
			dev_err(pfdev->dev, "timeout waiting on %s:%llx readyness",
				blk_name, mask);
			return ret;
		}
	}

	return 0;
}

int pancsf_gpu_l2_power_on(struct pancsf_device *pfdev)
{
	u64 core_mask = U64_MAX;

	if (pfdev->gpu_info.l2_present != 1) {
		/*
		 * Only support one core group now.
		 * ~(l2_present - 1) unsets all bits in l2_present except
		 * the bottom bit. (l2_present - 2) has all the bits in
		 * the first core group set. AND them together to generate
		 * a mask of cores in the first core group.
		 */
		core_mask = ~(pfdev->gpu_info.l2_present - 1) &
			     (pfdev->gpu_info.l2_present - 2);
		dev_info_once(pfdev->dev, "using only 1st core group (%lu cores from %lu)\n",
			      hweight64(core_mask),
			      hweight64(pfdev->gpu_info.shader_present));
	}

	return pancsf_gpu_power_on(pfdev, L2,
				   pfdev->gpu_info.l2_present & core_mask,
				   20000);
}

int pancsf_gpu_flush_caches(struct pancsf_device *pfdev,
			    u32 l2, u32 lsc, u32 other)
{
	bool timedout = false;
	unsigned long flags;

	spin_lock_irqsave(&pfdev->gpu->reqs_lock, flags);
	if (!WARN_ON(pfdev->gpu->pending_reqs & GPU_IRQ_CLEAN_CACHES_COMPLETED)) {
		pfdev->gpu->pending_reqs |= GPU_IRQ_CLEAN_CACHES_COMPLETED;
		gpu_write(pfdev, GPU_CMD, GPU_FLUSH_CACHES(l2, lsc, other));
	}
	spin_unlock_irqrestore(&pfdev->gpu->reqs_lock, flags);

	if (!wait_event_timeout(pfdev->gpu->reqs_acked,
				!(pfdev->gpu->pending_reqs & GPU_IRQ_CLEAN_CACHES_COMPLETED),
				msecs_to_jiffies(100))) {
		spin_lock_irqsave(&pfdev->gpu->reqs_lock, flags);
		if ((pfdev->gpu->pending_reqs & GPU_IRQ_CLEAN_CACHES_COMPLETED) != 0 &&
		    !(gpu_read(pfdev, GPU_INT_STAT) & GPU_IRQ_CLEAN_CACHES_COMPLETED))
			timedout = true;
		spin_unlock_irqrestore(&pfdev->gpu->reqs_lock, flags);
	}

	if (timedout) {
		dev_err(pfdev->dev, "Flush caches timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

int pancsf_gpu_soft_reset(struct pancsf_device *pfdev)
{
	bool timedout = false;
	unsigned long flags;

	spin_lock_irqsave(&pfdev->gpu->reqs_lock, flags);
	if (!WARN_ON(pfdev->gpu->pending_reqs & GPU_IRQ_RESET_COMPLETED)) {
		pfdev->gpu->pending_reqs |= GPU_IRQ_RESET_COMPLETED;
		gpu_write(pfdev, GPU_CMD, GPU_SOFT_RESET);
	}
	spin_unlock_irqrestore(&pfdev->gpu->reqs_lock, flags);

	if (!wait_event_timeout(pfdev->gpu->reqs_acked,
				!(pfdev->gpu->pending_reqs & GPU_IRQ_RESET_COMPLETED),
				msecs_to_jiffies(100))) {
		spin_lock_irqsave(&pfdev->gpu->reqs_lock, flags);
		if ((pfdev->gpu->pending_reqs & GPU_IRQ_RESET_COMPLETED) != 0 &&
		    !(gpu_read(pfdev, GPU_INT_STAT) & GPU_IRQ_RESET_COMPLETED))
			timedout = true;
		spin_unlock_irqrestore(&pfdev->gpu->reqs_lock, flags);
	}

	gpu_write(pfdev, GPU_INT_MASK,
		  GPU_IRQ_FAULT |
		  GPU_IRQ_PROTM_FAULT |
		  GPU_IRQ_RESET_COMPLETED |
		  GPU_IRQ_MCU_STATUS_CHANGED |
		  GPU_IRQ_CLEAN_CACHES_COMPLETED);

	if (timedout) {
		dev_err(pfdev->dev, "Soft reset timeout");
		return -ETIMEDOUT;
	}

	return 0;
}
