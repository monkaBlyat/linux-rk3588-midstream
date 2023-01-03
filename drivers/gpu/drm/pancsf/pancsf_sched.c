// SPDX-License-Identifier: GPL-2.0
/* Copyright 2023 Collabora ltd. */

#ifdef CONFIG_ARM_ARCH_TIMER
#include <asm/arch_timer.h>
#endif

#include <drm/pancsf_drm.h>
#include <drm/drm_gem_shmem_helper.h>

#include <linux/build_bug.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/iosys-map.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-resv.h>

#include "pancsf_sched.h"
#include "pancsf_device.h"
#include "pancsf_gem.h"
#include "pancsf_heap.h"
#include "pancsf_regs.h"
#include "pancsf_gpu.h"
#include "pancsf_mcu.h"
#include "pancsf_mmu.h"

#define PANCSF_CS_FW_NAME "mali_csffw.bin"

#define CSF_JOB_TIMEOUT_MS			5000

#define MIN_CS_PER_CSG				8

#define MIN_CSGS				3
#define MAX_CSG_PRIO				0xf

struct pancsf_group;

struct pancsf_csg_slot {
	struct pancsf_group *group;
	u32 pending_reqs;
	wait_queue_head_t reqs_acked;
	u8 priority;
	bool idle;
};

enum pancsf_csg_priority {
	PANCSF_CSG_PRIORITY_LOW = 0,
	PANCSF_CSG_PRIORITY_MEDIUM,
	PANCSF_CSG_PRIORITY_HIGH,
	PANCSF_CSG_PRIORITY_RT,
	PANCSF_CSG_PRIORITY_COUNT,
};

struct pancsf_scheduler {
	struct pancsf_device *pfdev;
	struct workqueue_struct *wq;
	struct delayed_work tick_work;
	struct delayed_work ping_work;
	struct work_struct sync_upd_work;
	struct work_struct reset_work;
	bool reset_pending;
	u64 resched_target;
	u64 last_tick;
	u64 tick_period;

	struct mutex lock;
	struct list_head run_queues[PANCSF_CSG_PRIORITY_COUNT];
	struct list_head idle_queues[PANCSF_CSG_PRIORITY_COUNT];
	struct list_head wait_queue;

	struct pancsf_csg_slot csg_slots[MAX_CSGS];
	u32 csg_slot_count;
	u32 cs_slot_count;
	u32 as_slot_count;
	u32 used_csg_slot_count;
	u32 sb_slot_count;
	bool might_have_idle_groups;

	u32 pending_reqs;
	wait_queue_head_t reqs_acked;
};

struct pancsf_syncobj_32b {
	u32 seqno;
	u32 status;
};

struct pancsf_syncobj_64b {
	u64 seqno;
	u32 status;
	u32 pad;
};

#define PANCSF_CS_QUEUE_FENCE_CTX_NAME_PREFIX "pancsf-csqf-ctx-"
#define PANCSF_CS_QUEUE_FENCE_CTX_NAME_LEN (sizeof(PANCSF_CS_QUEUE_FENCE_CTX_NAME_PREFIX) + 16)

struct pancsf_queue_fence_ctx {
	struct kref refcount;
	char name[PANCSF_CS_QUEUE_FENCE_CTX_NAME_LEN];
	spinlock_t lock;
	u64 id;
	atomic64_t seqno;
};

struct pancsf_queue_fence {
	struct dma_fence base;
	struct pancsf_queue_fence_ctx *ctx;
	u64 seqno;
};

struct pancsf_queue {
	u8 priority;
	struct {
		struct pancsf_gem_object *bo;
		u64 gpu_va;
		u64 *kmap;
	} ringbuf;

	struct {
		struct pancsf_fw_mem *mem;
		struct pancsf_ringbuf_input_iface *input;
		struct pancsf_ringbuf_output_iface *output;
	} iface;

	struct {
		spinlock_t lock;
		struct list_head in_flight;
		struct list_head pending;
	} jobs;


	struct {
		u64 gpu_va;
		u64 ref;
		bool gt;
		bool sync64;
		struct pancsf_gem_object *bo;
		u64 offset;
		void *kmap;
	} syncwait;

	struct pancsf_syncobj_64b *syncobj;
	struct pancsf_queue_fence_ctx *fence_ctx;
};

struct pancsf_file_ctx;

enum pancsf_group_state {
	PANCSF_CS_GROUP_CREATED,
	PANCSF_CS_GROUP_ACTIVE,
	PANCSF_CS_GROUP_SUSPENDED,
	PANCSF_CS_GROUP_TERMINATED,
};

struct pancsf_group {
	struct kref refcount;
	struct pancsf_device *pfdev;
	struct pancsf_heap_pool *heaps;
	struct pancsf_vm *vm;
	u64 compute_core_mask;
	u64 fragment_core_mask;
	u64 tiler_core_mask;
	u8 max_compute_cores;
	u8 max_fragment_cores;
	u8 max_tiler_cores;
	u8 priority;
	u32 blocked_streams;
	u32 idle_streams;
	spinlock_t fatal_lock;
	u32 fatal_streams;
	u32 stream_count;
	struct pancsf_queue *streams[MAX_CS_PER_CSG];
	int as_id;
	int csg_id;
	bool destroyed;
	bool timedout;
	bool in_tick_ctx;

	struct {
		struct pancsf_gem_object *bo;
		u64 gpu_va;
		void *kmap;
	} syncobjs;

	enum pancsf_group_state state;
	struct pancsf_fw_mem *suspend_buf;
	struct pancsf_fw_mem *protm_suspend_buf;

	struct work_struct sync_upd_work;
	struct work_struct job_pending_work;
	struct work_struct term_work;

	struct list_head run_node;
	struct list_head wait_node;
};

#define MAX_GROUPS_PER_POOL MAX_CSGS

struct pancsf_group_pool {
	struct mutex lock;
	struct xarray xa;
};

struct pancsf_job {
	struct kref refcount;
	struct pancsf_group *group;
	u32 stream_idx;

	struct pancsf_call_info call_info;
	struct {
		u64 start, end;
	} ringbuf;
	struct delayed_work timeout_work;
	struct dma_fence_cb dep_cb;
	struct dma_fence *cur_dep;
	struct xarray dependencies;
	unsigned long last_dependency;
	struct list_head node;
	struct dma_fence *done_fence;
};

static u32 pancsf_conv_timeout(struct pancsf_device *pfdev, u32 timeout_us)
{
	bool use_cycle_counter = false;
	u32 timer_rate = 0;
	u64 cycles;

#ifdef CONFIG_ARM_ARCH_TIMER
	timer_rate = arch_timer_get_cntfrq();
#endif

	if (!timer_rate) {
		use_cycle_counter = true;
		timer_rate = clk_get_rate(pfdev->clock);
	}

	if (WARN_ON(!timer_rate)) {
		/* We couldn't get a valid clock rate, let's just pick the
		 * maximum value so the FW still handles the core
		 * power on/off requests.
		 */
		return GLB_TIMER_VAL(0x7fffffff) |
		       GLB_TIMER_SOURCE_GPU_COUNTER;
	}

	cycles = DIV_ROUND_UP_ULL((u64)timeout_us * timer_rate, 1000000);
	return GLB_TIMER_VAL(cycles >> 10) |
	       (use_cycle_counter ? GLB_TIMER_SOURCE_GPU_COUNTER : 0);
}

static void pancsf_global_init(struct pancsf_device *pfdev)
{
	const struct pancsf_fw_global_iface *glb_iface = pancsf_get_glb_iface(pfdev);
	struct pancsf_scheduler *sched = pfdev->scheduler;
	u32 req_mask = GLB_CFG_ALLOC_EN |
		       GLB_CFG_POWEROFF_TIMER |
		       GLB_CFG_PROGRESS_TIMER |
		       GLB_IDLE_EN;
	u32 req_val;

	/* Enable all cores. */
	glb_iface->input->core_en_mask = pfdev->gpu_info.shader_present;

	/* 800us power off hysteresis. */
	glb_iface->input->poweroff_timer = pancsf_conv_timeout(pfdev, 800);

	/* Progress timeout set to 2500 * 1024 cycles. */
	glb_iface->input->progress_timer = 2500;

	/* 10ms idle hysteresis */
	glb_iface->input->idle_timer = pancsf_conv_timeout(pfdev, 10000);

	req_val = pancsf_toggle_reqs(glb_iface->input->req,
				     glb_iface->output->ack,
				     GLB_CFG_ALLOC_EN |
				     GLB_CFG_POWEROFF_TIMER |
				     GLB_CFG_PROGRESS_TIMER) |
		  GLB_IDLE_EN;

	/* Update the request reg */
	glb_iface->input->req = pancsf_update_reqs(glb_iface->input->req,
						   req_val | GLB_IDLE_EN,
						   req_mask);

	/* Enable interrupts we care about. */
	glb_iface->input->ack_irq_mask = GLB_CFG_ALLOC_EN |
					 GLB_PING |
					 GLB_CFG_PROGRESS_TIMER |
					 GLB_CFG_POWEROFF_TIMER |
					 GLB_IDLE_EN |
					 GLB_IDLE;

	gpu_write(pfdev, CSF_DOORBELL(CSF_GLB_DOORBELL_ID), 1);

	/* Kick the FW watchdog. */
	mod_delayed_work(sched->wq,
			 &sched->ping_work,
			 msecs_to_jiffies(12000));
}

static void
pancsf_queue_release_syncwait_obj(struct pancsf_group *group,
				  struct pancsf_queue *stream)
{
	pancsf_gem_unmap_and_put(group->vm, stream->syncwait.bo,
				 stream->syncwait.gpu_va, stream->syncwait.kmap);
}

static void pancsf_queue_fence_ctx_release(struct kref *kref)
{
	struct pancsf_queue_fence_ctx *ctx = container_of(kref,
							  struct pancsf_queue_fence_ctx,
							  refcount);

	kfree(ctx);
	module_put(THIS_MODULE);
}

static void pancsf_free_queue(struct pancsf_group *group,
			      struct pancsf_queue *stream)
{
	if (IS_ERR_OR_NULL(stream))
		return;

	if (stream->syncwait.bo)
		pancsf_queue_release_syncwait_obj(group, stream);

	if (stream->fence_ctx)
		kref_put(&stream->fence_ctx->refcount, pancsf_queue_fence_ctx_release);

	if (!IS_ERR_OR_NULL(stream->ringbuf.bo)) {
		pancsf_gem_unmap_and_put(group->vm, stream->ringbuf.bo,
					 stream->ringbuf.gpu_va, stream->ringbuf.kmap);
	}

	pancsf_fw_mem_free(group->pfdev, stream->iface.mem);
}

static void pancsf_release_group(struct kref *kref)
{
	struct pancsf_group *group = container_of(kref,
						     struct pancsf_group,
						     refcount);
	u32 i;

	WARN_ON(group->csg_id >= 0);
	WARN_ON(!list_empty(&group->run_node));
	WARN_ON(!list_empty(&group->wait_node));

	for (i = 0; i < group->stream_count; i++)
		pancsf_free_queue(group, group->streams[i]);

	if (group->suspend_buf)
		pancsf_fw_mem_free(group->pfdev, group->suspend_buf);

	if (group->protm_suspend_buf)
		pancsf_fw_mem_free(group->pfdev, group->protm_suspend_buf);

	if (!IS_ERR_OR_NULL(group->syncobjs.bo)) {
		pancsf_gem_unmap_and_put(group->vm, group->syncobjs.bo,
					 group->syncobjs.gpu_va, group->syncobjs.kmap);
	}

	if (group->vm)
		pancsf_vm_put(group->vm);

	kfree(group);
}

static void pancsf_group_put(struct pancsf_group *group)
{
	if (group)
		kref_put(&group->refcount, pancsf_release_group);
}

static struct pancsf_group *
pancsf_group_get(struct pancsf_group *group)
{
	if (group)
		kref_get(&group->refcount);

	return group;
}

static int
pancsf_bind_group_locked(struct pancsf_group *group,
			 u32 csg_id)
{
	struct pancsf_device *pfdev = group->pfdev;
	struct pancsf_csg_slot *csg_slot;

	if (WARN_ON(group->csg_id != -1 || csg_id >= MAX_CSGS ||
		    pfdev->scheduler->csg_slots[csg_id].group))
		return -EINVAL;

	csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	pancsf_group_get(group);
	group->csg_id = csg_id;
	group->as_id = pancsf_vm_as_get(group->vm);
	csg_slot->group = group;

	return 0;
}

static int
pancsf_unbind_group_locked(struct pancsf_group *group)
{
	struct pancsf_device *pfdev = group->pfdev;
	struct pancsf_csg_slot *slot;

	if (WARN_ON(group->csg_id < 0 || group->csg_id >= MAX_CSGS))
		return -EINVAL;

	if (WARN_ON(group->state == PANCSF_CS_GROUP_ACTIVE))
		return -EINVAL;

	slot = &pfdev->scheduler->csg_slots[group->csg_id];
	pancsf_vm_as_put(group->vm);
	group->as_id = -1;
	group->csg_id = -1;
	slot->group = NULL;

	pancsf_group_put(group);
	return 0;
}

static int
pancsf_prog_stream_locked(struct pancsf_group *group, u32 stream_idx)
{
	const struct pancsf_fw_cs_iface *cs_iface;
	struct pancsf_queue *stream;

	if (stream_idx >= group->stream_count)
		return -EINVAL;

	stream = group->streams[stream_idx];
	cs_iface = pancsf_get_cs_iface(group->pfdev, group->csg_id, stream_idx);
	stream->iface.input->extract = stream->iface.output->extract;
	WARN_ON(stream->iface.input->insert < stream->iface.input->extract);

	cs_iface->input->ringbuf_base = stream->ringbuf.gpu_va;
	cs_iface->input->ringbuf_size = stream->ringbuf.bo->base.base.size;
	cs_iface->input->ringbuf_input = pancsf_fw_mem_va(stream->iface.mem);
	cs_iface->input->ringbuf_output = pancsf_fw_mem_va(stream->iface.mem) + PAGE_SIZE;
	cs_iface->input->config = CS_CONFIG_PRIORITY(stream->priority) |
				  CS_CONFIG_DOORBELL(group->csg_id + 1);
	cs_iface->input->ack_irq_mask = ~0;
	cs_iface->input->req = pancsf_update_reqs(cs_iface->input->req,
						  CS_IDLE_SYNC_WAIT |
						  CS_IDLE_EMPTY |
						  CS_STATE_START |
						  CS_EXTRACT_EVENT,
						  CS_IDLE_SYNC_WAIT |
						  CS_IDLE_EMPTY |
						  CS_STATE_MASK |
						  CS_EXTRACT_EVENT);
	return 0;
}

static int
pancsf_reset_cs_slot_locked(struct pancsf_group *group,
			    u32 stream_idx)
{
	const struct pancsf_fw_cs_iface *cs_iface;

	if (stream_idx >= group->stream_count)
		return -EINVAL;

	cs_iface = pancsf_get_cs_iface(group->pfdev, group->csg_id, stream_idx);
	cs_iface->input->req = pancsf_update_reqs(cs_iface->input->req,
						  CS_STATE_STOP,
						  CS_STATE_MASK);
	return 0;
}

static void
pancsf_sync_csg_slot_priority_locked(struct pancsf_device *pfdev,
				     u32 csg_id)
{
	struct pancsf_csg_slot *csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	const struct pancsf_fw_csg_iface *csg_iface;

	csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
	csg_slot->priority = (csg_iface->input->endpoint_req & CSG_EP_REQ_PRIORITY_MASK) >> 28;
}

static void
pancsf_sync_queue_state_locked(struct pancsf_group *group, u32 cs_id)
{
	struct pancsf_queue *stream = group->streams[cs_id];
	struct pancsf_fw_cs_iface *cs_iface;
	u32 status_wait_cond;

	cs_iface = pancsf_get_cs_iface(group->pfdev, group->csg_id, cs_id);

	switch (cs_iface->output->status_blocked_reason) {
	case CS_STATUS_BLOCKED_REASON_UNBLOCKED:
		if (stream->iface.input->insert == stream->iface.output->extract &&
		    cs_iface->output->status_scoreboards == 0)
			group->idle_streams |= BIT(cs_id);
		break;

	case CS_STATUS_BLOCKED_REASON_SYNC_WAIT:
		WARN_ON(!list_empty(&group->wait_node));
		list_move_tail(&group->wait_node, &group->pfdev->scheduler->wait_queue);
		group->blocked_streams |= BIT(cs_id);
		stream->syncwait.gpu_va = cs_iface->output->status_wait_sync_ptr;
		stream->syncwait.ref = cs_iface->output->status_wait_sync_value;
		status_wait_cond = cs_iface->output->status_wait & CS_STATUS_WAIT_SYNC_COND_MASK;
		stream->syncwait.gt = status_wait_cond == CS_STATUS_WAIT_SYNC_COND_GT;
		if (cs_iface->output->status_wait & CS_STATUS_WAIT_SYNC_64B) {
			u64 sync_val_hi = cs_iface->output->status_wait_sync_value_hi;

			stream->syncwait.sync64 = true;
			stream->syncwait.ref |= sync_val_hi << 32;
		} else {
			stream->syncwait.sync64 = false;
		}
		break;

	default:
		/* Other reasons are not blocking. Consider the stream as runnable
		 * in those cases.
		 */
		break;
	}
}

static void
pancsf_sync_csg_slot_streams_state_locked(struct pancsf_device *pfdev,
					  u32 csg_id)
{
	struct pancsf_csg_slot *csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	struct pancsf_group *group = csg_slot->group;
	u32 i;

	group->idle_streams = 0;
	group->blocked_streams = 0;

	for (i = 0; i < group->stream_count; i++) {
		if (group->streams[i])
			pancsf_sync_queue_state_locked(group, i);
	}
}

static void
pancsf_sync_csg_slot_state_locked(struct pancsf_device *pfdev, u32 csg_id)
{
	struct pancsf_csg_slot *csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	const struct pancsf_fw_csg_iface *csg_iface;
	struct pancsf_group *group;
	enum pancsf_group_state new_state, old_state;

	csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
	group = csg_slot->group;

	if (!group)
		return;

	old_state = group->state;
	switch (csg_iface->output->ack & CSG_STATE_MASK) {
	case CSG_STATE_START:
	case CSG_STATE_RESUME:
		new_state = PANCSF_CS_GROUP_ACTIVE;
		break;
	case CSG_STATE_TERMINATE:
		new_state = PANCSF_CS_GROUP_TERMINATED;
		break;
	case CSG_STATE_SUSPEND:
		new_state = PANCSF_CS_GROUP_SUSPENDED;
		break;
	}

	if (old_state == new_state)
		return;

	if (new_state == PANCSF_CS_GROUP_SUSPENDED)
		pancsf_sync_csg_slot_streams_state_locked(pfdev, csg_id);

	if (old_state == PANCSF_CS_GROUP_ACTIVE) {
		u32 i;

		/* Reset the stream slots so we start from a clean
		 * state when starting/resuming a new group on this
		 * CSG slot. No wait needed here, and no ringbell
		 * either, since the CS slot will only be re-used
		 * on the next CSG start operation.
		 */
		for (i = 0; i < group->stream_count; i++) {
			if (group->streams[i])
				pancsf_reset_cs_slot_locked(group, i);
		}
	}

	group->state = new_state;
}

static int
pancsf_prog_csg_slot_locked(struct pancsf_device *pfdev, u32 csg_id, u32 priority)
{
	const struct pancsf_fw_global_iface *glb_iface;
	const struct pancsf_fw_csg_iface *csg_iface;
	struct pancsf_csg_slot *csg_slot;
	struct pancsf_group *group;
	u32 stream_mask = 0, i;

	if (priority > MAX_CSG_PRIO)
		return -EINVAL;

	if (WARN_ON(csg_id >= MAX_CSGS))
		return -EINVAL;

	csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	group = csg_slot->group;
	if (!group || group->state == PANCSF_CS_GROUP_ACTIVE)
		return 0;

	glb_iface = pancsf_get_glb_iface(group->pfdev);
	csg_iface = pancsf_get_csg_iface(group->pfdev, group->csg_id);

	for (i = 0; i < group->stream_count; i++) {
		if (group->streams[i]) {
			pancsf_prog_stream_locked(group, i);
			stream_mask |= BIT(i);
		}
	}

	csg_iface->input->allow_compute = group->compute_core_mask;
	csg_iface->input->allow_fragment = group->fragment_core_mask;
	csg_iface->input->allow_other = group->tiler_core_mask;
	csg_iface->input->endpoint_req = CSG_EP_REQ_COMPUTE(group->max_compute_cores) |
					 CSG_EP_REQ_FRAGMENT(group->max_fragment_cores) |
					 CSG_EP_REQ_TILER(group->max_tiler_cores) |
					 CSG_EP_REQ_PRIORITY(priority);
	csg_iface->input->config = group->as_id;

	if (group->suspend_buf)
		csg_iface->input->suspend_buf = pancsf_fw_mem_va(group->suspend_buf);
	else
		csg_iface->input->suspend_buf = 0;

	if (group->protm_suspend_buf)
		csg_iface->input->protm_suspend_buf = pancsf_fw_mem_va(group->protm_suspend_buf);
	else
		csg_iface->input->protm_suspend_buf = 0;

	csg_iface->input->ack_irq_mask = ~0;
	csg_iface->input->doorbell_req = pancsf_toggle_reqs(csg_iface->input->doorbell_req,
							    csg_iface->output->doorbell_ack,
							    stream_mask);
	return 0;
}

static void pancsf_handle_cs_fatal(struct pancsf_device *pfdev,
				   unsigned int csg_id, unsigned int cs_id)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];
	struct pancsf_group *group = csg_slot->group;
	const struct pancsf_fw_cs_iface *csg_iface;
	const struct pancsf_fw_cs_iface *cs_iface;
	u32 fatal;
	u64 info;

	csg_iface = pancsf_get_cs_iface(pfdev, csg_id, cs_id);
	cs_iface = pancsf_get_cs_iface(pfdev, csg_id, cs_id);
	fatal = cs_iface->output->fatal;
	info = cs_iface->output->fatal_info;
	group->fatal_streams |= BIT(cs_id);
	mod_delayed_work(sched->wq, &sched->tick_work, 0);
	dev_warn(pfdev->dev,
		 "CSG slot %d CS slot: %d\n"
		 "CS_FATAL.EXCEPTION_TYPE: 0x%x (%s)\n"
		 "CS_FATAL.EXCEPTION_DATA: 0x%x\n"
		 "CS_FATAL_INFO.EXCEPTION_DATA: 0x%llx\n",
		 csg_id, cs_id,
		 (unsigned int)CS_EXCEPTION_TYPE(fatal),
		 pancsf_exception_name(CS_EXCEPTION_TYPE(fatal)),
		 (unsigned int)CS_EXCEPTION_DATA(fatal),
		 info);
}

static void pancsf_handle_cs_fault(struct pancsf_device *pfdev,
				   unsigned int csg_id, unsigned int cs_id)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];
	struct pancsf_group *group = csg_slot->group;
	struct pancsf_queue *stream = cs_id < group->stream_count ? group->streams[cs_id] : NULL;
	const struct pancsf_fw_cs_iface *cs_iface;
	u32 fault;
	u64 info;

	cs_iface = pancsf_get_cs_iface(pfdev, csg_id, cs_id);
	fault = cs_iface->output->fault;
	info = cs_iface->output->fault_info;

	if (stream && CS_EXCEPTION_TYPE(fault) == DRM_PANCSF_EXCEPTION_CS_INHERIT_FAULT) {
		u64 cs_extract = stream->iface.output->extract;
		struct pancsf_job *job;

		spin_lock(&stream->jobs.lock);
		list_for_each_entry(job, &stream->jobs.in_flight, node) {
			if (cs_extract >= job->ringbuf.end)
				continue;

			if (cs_extract < job->ringbuf.start)
				break;

			dma_fence_set_error(job->done_fence, -EINVAL);
		}
		spin_unlock(&stream->jobs.lock);
	}

	dev_warn(pfdev->dev,
		 "CSG slot %d CS slot: %d\n"
		 "CS_FAULT.EXCEPTION_TYPE: 0x%x (%s)\n"
		 "CS_FAULT.EXCEPTION_DATA: 0x%x\n"
		 "CS_FAULT_INFO.EXCEPTION_DATA: 0x%llx\n",
		 csg_id, cs_id,
		 (unsigned int)CS_EXCEPTION_TYPE(fault),
		 pancsf_exception_name(CS_EXCEPTION_TYPE(fault)),
		 (unsigned int)CS_EXCEPTION_DATA(fault),
		 info);
}

static void pancsf_handle_tiler_oom(struct pancsf_device *pfdev,
				    unsigned int csg_id,
				    unsigned int cs_id)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];
	struct pancsf_group *group = csg_slot->group;
	const struct pancsf_fw_cs_iface *cs_iface;
	struct pancsf_heap_pool *heaps;
	struct pancsf_queue *stream;
	u32 fault, vt_start, vt_end, frag_end;
	u32 renderpasses_in_flight, pending_frag_count;
	u64 info, heap_address, new_chunk_va;
	int ret;

	if (WARN_ON(!group))
		return;

	cs_iface = pancsf_get_cs_iface(pfdev, csg_id, cs_id);
	stream = group->streams[cs_id];
	heaps = group->heaps;
	fault = cs_iface->output->fault;
	info = cs_iface->output->fault_info;
	heap_address = cs_iface->output->heap_address;
	vt_start = cs_iface->output->heap_vt_start;
	vt_end = cs_iface->output->heap_vt_end;
	frag_end = cs_iface->output->heap_frag_end;
	renderpasses_in_flight = vt_start - frag_end;
	pending_frag_count = vt_end - frag_end;

	if (!heaps || frag_end > vt_end || vt_end >= vt_start) {
		ret = -EINVAL;
	} else {
		ret = pancsf_heap_grow(heaps, heap_address,
				       renderpasses_in_flight,
				       pending_frag_count, &new_chunk_va);
	}

	if (!ret) {
		cs_iface->input->heap_start = new_chunk_va;
		cs_iface->input->heap_end = new_chunk_va;
	} else if (ret == -EBUSY) {
		cs_iface->input->heap_start = 0;
		cs_iface->input->heap_end = 0;
	} else {
		group->fatal_streams |= BIT(csg_id);
		mod_delayed_work(sched->wq, &sched->tick_work, 0);
	}
}

static bool pancsf_handle_cs_irq(struct pancsf_device *pfdev,
				 unsigned int csg_id, unsigned int cs_id)
{
	const struct pancsf_fw_cs_iface *cs_iface;
	u32 req, ack, events;

	cs_iface = pancsf_get_cs_iface(pfdev, csg_id, cs_id);
	req = cs_iface->input->req;
	ack = cs_iface->output->ack;
	events = req ^ ack;

	if (events & CS_FATAL)
		pancsf_handle_cs_fatal(pfdev, csg_id, cs_id);

	if (events & CS_FAULT)
		pancsf_handle_cs_fault(pfdev, csg_id, cs_id);

	if (events & CS_TILER_OOM)
		pancsf_handle_tiler_oom(pfdev, csg_id, cs_id);

	cs_iface->input->req = pancsf_update_reqs(req, ack,
						  CS_FATAL | CS_FAULT | CS_TILER_OOM);

	return (events & (CS_FAULT | CS_TILER_OOM)) != 0;
}

static void pancsf_handle_csg_state_update(struct pancsf_device *pfdev, unsigned int csg_id)
{
	struct pancsf_csg_slot *csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	const struct pancsf_fw_csg_iface *csg_iface;

	csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
	csg_slot->idle = csg_iface->output->status_state & CSG_STATUS_STATE_IS_IDLE;
}

static void pancsf_handle_csg_idle(struct pancsf_device *pfdev, unsigned int csg_id)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	int prio;

	sched->might_have_idle_groups = true;

	/* Schedule a tick if there are other runnable groups waiting. */
	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		if (!list_empty(&sched->run_queues[prio])) {
			mod_delayed_work(sched->wq, &sched->tick_work, 0);
			return;
		}
	}
}

/* Macro automating the 'grab a ref, schedule the work and release ref if
 * already scheduled' sequence.
 */
#define pancsf_group_queue_work(sched, group, wname) \
	do { \
		pancsf_group_get(group); \
		if (!queue_work((sched)->wq, &(group)->wname ## _work)) \
			pancsf_group_put(group); \
	} while (0)

static void pancsf_queue_csg_sync_update_locked(struct pancsf_device *pfdev,
						unsigned int csg_id)
{
	struct pancsf_csg_slot *csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	struct pancsf_group *group = csg_slot->group;

	pancsf_group_queue_work(pfdev->scheduler, group, sync_upd);

	queue_work(pfdev->scheduler->wq, &pfdev->scheduler->sync_upd_work);
}

static void
pancsf_handle_csg_progress_timer_evt(struct pancsf_device *pfdev, unsigned int csg_id)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];
	struct pancsf_group *group = csg_slot->group;

	dev_warn(pfdev->dev, "CSG slot %d progress timeout\n", csg_id);
	group->timedout = true;
	mod_delayed_work(sched->wq, &sched->tick_work, 0);
}

static bool pancsf_sched_handle_csg_irq(struct pancsf_device *pfdev, unsigned int csg_id)
{
	struct pancsf_csg_slot *csg_slot = &pfdev->scheduler->csg_slots[csg_id];
	const struct pancsf_fw_csg_iface *csg_iface;
	struct pancsf_group *group = csg_slot->group;
	u32 req, ack, irq_req, irq_ack, cs_events, csg_events;
	u32 ring_cs_db_mask = 0;
	u32 acked_reqs;

	lockdep_assert_held(&pfdev->scheduler->lock);
	if (WARN_ON(csg_id >= pfdev->scheduler->csg_slot_count))
		return false;

	csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
	req = csg_iface->input->req;
	ack = csg_iface->output->ack;
	irq_req = csg_iface->output->irq_req;
	irq_ack = csg_iface->input->irq_ack;
	csg_events = req ^ ack;
	acked_reqs = csg_slot->pending_reqs & ~csg_events;

	if (acked_reqs & CSG_ENDPOINT_CONFIG)
		pancsf_sync_csg_slot_priority_locked(pfdev, csg_id);

	if ((acked_reqs & CSG_STATE_MASK) == CSG_STATE_MASK) {
		acked_reqs |= CSG_STATE_MASK;
		pancsf_sync_csg_slot_state_locked(pfdev, csg_id);
	}

	if ((acked_reqs & CSG_STATE_MASK) == CSG_STATE_MASK)
		pancsf_sync_csg_slot_state_locked(pfdev, csg_id);
	else
		acked_reqs &= ~CSG_STATE_MASK;

	if (acked_reqs & CSG_STATUS_UPDATE)
		pancsf_handle_csg_state_update(pfdev, csg_id);

	if (acked_reqs) {
		csg_slot->pending_reqs &= ~acked_reqs;
		wake_up_all(&csg_slot->reqs_acked);
	}

	/* There may not be any pending CSG/CS interrupts to process */
	if (req == ack && irq_req == irq_ack)
		return false;

	/* Immediately set IRQ_ACK bits to be same as the IRQ_REQ bits before
	 * examining the CS_ACK & CS_REQ bits. This would ensure that Host
	 * doesn't misses an interrupt for the CS in the race scenario where
	 * whilst Host is servicing an interrupt for the CS, firmware sends
	 * another interrupt for that CS.
	 */
	csg_iface->input->irq_ack = irq_req;

	if (WARN_ON(!group))
		return false;

	csg_iface->input->req = pancsf_update_reqs(req, ack,
						   CSG_SYNC_UPDATE |
						   CSG_IDLE |
						   CSG_PROGRESS_TIMER_EVENT);

	if (csg_events & CSG_IDLE)
		pancsf_handle_csg_idle(pfdev, csg_id);

	if (csg_events & CSG_PROGRESS_TIMER_EVENT)
		pancsf_handle_csg_progress_timer_evt(pfdev, csg_id);

	cs_events = irq_req ^ irq_ack;
	while (cs_events) {
		u32 cs_id = ffs(cs_events) - 1;

		if (pancsf_handle_cs_irq(pfdev, csg_id, cs_id))
			ring_cs_db_mask |= BIT(cs_id);

		cs_events &= ~BIT(cs_id);
	}

	if (csg_events & CSG_SYNC_UPDATE)
		pancsf_queue_csg_sync_update_locked(pfdev, csg_id);

	if (ring_cs_db_mask) {
		csg_iface->input->doorbell_req = pancsf_toggle_reqs(csg_iface->input->doorbell_req,
								    csg_iface->output->doorbell_ack,
								    ring_cs_db_mask);
	}

	return ring_cs_db_mask != 0;
}

static void pancsf_sched_handle_global_irq(struct pancsf_device *pfdev)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	const struct pancsf_fw_global_iface *glb_iface = pancsf_get_glb_iface(pfdev);
	u32 req, ack, events, acked_reqs;

	req = glb_iface->input->req;
	ack = glb_iface->output->ack;
	events = req ^ ack;
	acked_reqs = sched->pending_reqs & ~events;

	if (acked_reqs) {
		sched->pending_reqs &= ~acked_reqs;
		wake_up_all(&sched->reqs_acked);
	}

	if (events & GLB_IDLE) {
		glb_iface->input->req = pancsf_update_reqs(glb_iface->input->req,
							   glb_iface->output->ack,
							   GLB_IDLE);
	}
}

void pancsf_sched_handle_job_irqs(struct pancsf_device *pfdev, u32 status)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	u32 csg_ints = status & ~JOB_INT_GLOBAL_IF;
	u32 ring_csg_db = 0;

	mutex_lock(&sched->lock);
	if (status & JOB_INT_GLOBAL_IF)
		pancsf_sched_handle_global_irq(pfdev);

	while (csg_ints) {
		u32 csg_id = ffs(csg_ints) - 1;

		csg_ints &= ~BIT(csg_id);
		if (pancsf_sched_handle_csg_irq(pfdev, csg_id))
			ring_csg_db |= BIT(csg_id);
	}

	if (ring_csg_db) {
		const struct pancsf_fw_global_iface *glb_iface = pancsf_get_glb_iface(pfdev);

		glb_iface->input->doorbell_req = pancsf_toggle_reqs(glb_iface->input->doorbell_req,
								    glb_iface->output->doorbell_ack,
								    ring_csg_db);
		gpu_write(pfdev, CSF_DOORBELL(CSF_GLB_DOORBELL_ID), 1);
	}

	mutex_unlock(&sched->lock);
}

static struct pancsf_queue_fence *
to_pancsf_queue_fence(struct dma_fence *fence)
{
	return container_of(fence, struct pancsf_queue_fence, base);
}

static const char *pancsf_queue_fence_get_driver_name(struct dma_fence *fence)
{
	return "pancsf";
}

static const char *pancsf_queue_fence_get_timeline_name(struct dma_fence *fence)
{
	struct pancsf_queue_fence *f = to_pancsf_queue_fence(fence);

	return f->ctx->name;
}

static int
pancsf_queue_fence_ctx_create(struct pancsf_queue *queue)
{
	struct pancsf_queue_fence_ctx *ctx;

	if (WARN_ON(!try_module_get(THIS_MODULE)))
		return -ENOENT;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		module_put(THIS_MODULE);
		return -ENOMEM;
	}

	ctx->id = dma_fence_context_alloc(1);
	spin_lock_init(&ctx->lock);
	snprintf(ctx->name, sizeof(ctx->name),
		 PANCSF_CS_QUEUE_FENCE_CTX_NAME_PREFIX "%llx",
		 ctx->id);
	kref_init(&ctx->refcount);
	queue->fence_ctx = ctx;
	return 0;
}

static void pancsf_queue_fence_release(struct dma_fence *fence)
{
	struct pancsf_queue_fence *f = to_pancsf_queue_fence(fence);

	kref_put(&f->ctx->refcount, pancsf_queue_fence_ctx_release);
	dma_fence_free(fence);
}

static const struct dma_fence_ops pancsf_queue_fence_ops = {
	.get_driver_name = pancsf_queue_fence_get_driver_name,
	.get_timeline_name = pancsf_queue_fence_get_timeline_name,
	.release = pancsf_queue_fence_release,
};

static struct dma_fence *pancsf_queue_fence_create(struct pancsf_queue *queue)
{
	struct pancsf_queue_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	kref_get(&queue->fence_ctx->refcount);
	fence->ctx = queue->fence_ctx;
	fence->seqno = atomic64_inc_return(&fence->ctx->seqno);
	dma_fence_init(&fence->base, &pancsf_queue_fence_ops, &fence->ctx->lock,
		       fence->ctx->id, fence->seqno);

	return &fence->base;
}

#define CSF_MAX_QUEUE_PRIO	GENMASK(3, 0)

static struct pancsf_queue *
pancsf_create_queue(struct pancsf_group *group,
		    const struct drm_pancsf_queue_create *args)
{
	struct pancsf_queue *stream;
	int ret;

	if (args->pad[0] || args->pad[1] || args->pad[2])
		return ERR_PTR(-EINVAL);

	if (!IS_ALIGNED(args->ringbuf_size, PAGE_SIZE) || args->ringbuf_size > SZ_64K)
		return ERR_PTR(-EINVAL);

	if (args->priority > CSF_MAX_QUEUE_PRIO)
		return ERR_PTR(-EINVAL);

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return ERR_PTR(-ENOMEM);

	spin_lock_init(&stream->jobs.lock);
	INIT_LIST_HEAD(&stream->jobs.in_flight);
	INIT_LIST_HEAD(&stream->jobs.pending);
	stream->priority = args->priority;

	stream->ringbuf.bo = pancsf_gem_create_and_map(group->pfdev, group->vm,
						       args->ringbuf_size, 0,
						       PANCSF_VMA_MAP_NOEXEC |
						       PANCSF_VMA_MAP_UNCACHED |
						       PANCSF_VMA_MAP_AUTO_VA,
						       &stream->ringbuf.gpu_va,
						       (void **)&stream->ringbuf.kmap);
	if (IS_ERR(stream->ringbuf.bo)) {
		ret = PTR_ERR(stream->ringbuf.bo);
		goto out;
	}

	stream->iface.mem = pancsf_fw_alloc_queue_iface_mem(group->pfdev);
	if (IS_ERR(stream->iface.mem)) {
		ret = PTR_ERR(stream->iface.mem);
		goto out;
	}

	stream->iface.input = pancsf_fw_mem_vmap(stream->iface.mem,
						 pgprot_writecombine(PAGE_KERNEL));
	if (!stream->iface.input) {
		ret = -ENOMEM;
		goto out;
	}

	memset(stream->iface.input, 0, sizeof(*stream->iface.input));
	stream->iface.output = (void *)stream->iface.input + PAGE_SIZE;
	memset((void *)stream->iface.output, 0, sizeof(*stream->iface.output));

	ret = pancsf_queue_fence_ctx_create(stream);

out:
	if (ret)
		return ERR_PTR(ret);

	return stream;
}

static void pancsf_job_dep_cb(struct dma_fence *fence, struct dma_fence_cb *cb)
{
	struct pancsf_job *job = container_of(cb, struct pancsf_job, dep_cb);
	struct pancsf_group *group = job->group;
	struct pancsf_scheduler *sched = group->pfdev->scheduler;

	pancsf_group_queue_work(sched, group, job_pending);
}

static bool
pancsf_job_deps_done(struct pancsf_job *job)
{
	if (job->cur_dep && !dma_fence_is_signaled(job->cur_dep))
		return false;

	dma_fence_put(job->cur_dep);
	job->cur_dep = NULL;

	while (!xa_empty(&job->dependencies)) {
		struct dma_fence *next_dep;
		int ret;

		next_dep = xa_erase(&job->dependencies, job->last_dependency++);
		ret = dma_fence_add_callback(next_dep, &job->dep_cb,
					     pancsf_job_dep_cb);
		if (!ret) {
			job->cur_dep = next_dep;
			return false;
		}

		WARN_ON(ret != -ENOENT);
		dma_fence_put(next_dep);
	}

	return true;
}

#define NUM_INSTRS_PER_SLOT		16

static bool
pancsf_queue_can_take_new_jobs(struct pancsf_queue *stream)
{
	u32 ringbuf_size = stream->ringbuf.bo->base.base.size;
	u64 used_size = stream->iface.input->insert - stream->iface.output->extract;

	return used_size + (NUM_INSTRS_PER_SLOT * sizeof(u64)) <= ringbuf_size;
}

static void pancsf_queue_submit_job(struct pancsf_queue *stream, struct pancsf_job *job)
{
	struct pancsf_group *group = job->group;
	struct pancsf_device *pfdev = group->pfdev;
	struct pancsf_scheduler *sched = pfdev->scheduler;
	u32 ringbuf_size = stream->ringbuf.bo->base.base.size;
	u32 ringbuf_insert = stream->iface.input->insert % ringbuf_size;
	u64 addr_reg = pfdev->csif_info.cs_reg_count -
		       pfdev->csif_info.unpreserved_cs_reg_count;
	u64 val_reg = addr_reg + 2;
	u64 sync_addr = group->syncobjs.gpu_va +
			job->stream_idx * sizeof(struct pancsf_syncobj_64b);
	u32 waitall_mask = GENMASK(sched->sb_slot_count - 1, 0);

	u64 call_instrs[NUM_INSTRS_PER_SLOT] = {
		/* MOV32 rX+2, cs.latest_flush */
		(2ull << 56) | (val_reg << 48) | job->call_info.latest_flush,

		/* FLUSH_CACHE2.clean_inv_all.no_wait.signal(0) rX+2 */
		(36ull << 56) | (0ull << 48) | (val_reg << 40) | (0 << 16) | 0x233,

		/* MOV48 rX:rX+1, cs.start */
		(1ull << 56) | (addr_reg << 48) | job->call_info.start,

		/* MOV32 rX+2, cs.size */
		(2ull << 56) | (val_reg << 48) | job->call_info.size,

		/* WAIT(0) => waits for FLUSH_CACHE2 instruction */
		(3ull << 56) | (1 << 16),

		/* CALL rX:rX+1, rX+2 */
		(32ull << 56) | (addr_reg << 40) | (val_reg << 32),

		/* MOV48 rX:rX+1, sync_addr */
		(1ull << 56) | (addr_reg << 48) | sync_addr,

		/* MOV32 rX+2, #1 */
		(1ull << 56) | (val_reg << 48) | 1,

		/* WAIT(all) */
		(3ull << 56) | (waitall_mask << 16),

		/* SYNC_ADD64.system_scope.propage_err.nowait rX:rX+1, rX+2*/
		(51ull << 56) | (0ull << 48) | (addr_reg << 40) | (val_reg << 32) | (0 << 16) | 1,

		/* ERROR_BARRIER, so we can recover from faults at job
		 * boundaries.
		 */
		(47ull << 56),
	};

	/* Need to be cacheline aligned to please the prefetcher */
	WARN_ON(sizeof(call_instrs) % 64);

	memcpy((u8 *)stream->ringbuf.kmap + ringbuf_insert,
	       call_instrs, sizeof(call_instrs));

	spin_lock(&stream->jobs.lock);
	list_move_tail(&job->node, &stream->jobs.in_flight);
	spin_unlock(&stream->jobs.lock);

	job->ringbuf.start = stream->iface.input->insert;
	job->ringbuf.end = job->ringbuf.start + sizeof(call_instrs);

	/* Make sure the ring buffer is updated before the INSERT
	 * register.
	 */
	wmb();
	stream->iface.input->extract = stream->iface.output->extract;
	stream->iface.input->insert = job->ringbuf.end;
	kref_get(&job->refcount);
	queue_delayed_work(pfdev->scheduler->wq,
			   &job->timeout_work,
			   msecs_to_jiffies(CSF_JOB_TIMEOUT_MS));
}

struct pancsf_csg_slots_upd_ctx {
	u32 update_mask;
	u32 timedout_mask;
	struct {
		u32 value;
		u32 mask;
	} requests[MAX_CSGS];
};

static void csgs_upd_ctx_init(struct pancsf_csg_slots_upd_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}

static void csgs_upd_ctx_queue_reqs(struct pancsf_device *pfdev,
				    struct pancsf_csg_slots_upd_ctx *ctx,
				    u32 csg_id, u32 value, u32 mask)
{
	if (WARN_ON(!mask) ||
	    WARN_ON(csg_id >= pfdev->scheduler->csg_slot_count))
		return;

	ctx->requests[csg_id].value = (ctx->requests[csg_id].value & ~mask) | (value & mask);
	ctx->requests[csg_id].mask |= mask;
	ctx->update_mask |= BIT(csg_id);
}

static int csgs_upd_ctx_apply_locked(struct pancsf_device *pfdev,
				     struct pancsf_csg_slots_upd_ctx *ctx)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_fw_global_iface *glb_iface;
	u32 update_slots = ctx->update_mask;

	lockdep_assert_held(&sched->lock);

	if (!ctx->update_mask)
		return 0;

	while (update_slots) {
		const struct pancsf_fw_csg_iface *csg_iface;
		u32 csg_id = ffs(update_slots) - 1;
		struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];

		update_slots &= ~BIT(csg_id);
		csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
		csg_slot->pending_reqs |= ctx->requests[csg_id].mask;
		csg_iface->input->req = pancsf_update_reqs(csg_iface->input->req,
							   ctx->requests[csg_id].value,
							   ctx->requests[csg_id].mask);
	}

	glb_iface = pancsf_get_glb_iface(pfdev);
	glb_iface->input->doorbell_req = pancsf_toggle_reqs(glb_iface->input->doorbell_req,
							    glb_iface->output->doorbell_ack,
							    ctx->update_mask);
	gpu_write(pfdev, CSF_DOORBELL(CSF_GLB_DOORBELL_ID), 1);

	update_slots = ctx->update_mask;
	while (update_slots) {
		const struct pancsf_fw_csg_iface *csg_iface;
		u32 csg_id = ffs(update_slots) - 1;
		struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];
		u32 req_mask = ctx->requests[csg_id].mask;
		bool timedout = false;

		update_slots &= ~BIT(csg_id);
		csg_iface = pancsf_get_csg_iface(pfdev, csg_id);

		/* Release the lock while we're waiting. */
		mutex_unlock(&sched->lock);
		if (!wait_event_timeout(csg_slot->reqs_acked,
					!(csg_slot->pending_reqs & req_mask),
					msecs_to_jiffies(100))) {
			WARN_ON(gpu_read(pfdev, JOB_INT_MASK) == 0);
			timedout = true;
		}
		mutex_lock(&sched->lock);

		if (timedout &&
		    (csg_slot->pending_reqs & req_mask) != 0 &&
		    ((csg_iface->input->req ^ csg_iface->output->ack) & req_mask) != 0) {
			dev_err(pfdev->dev, "CSG %d update request timedout", csg_id);
			ctx->timedout_mask |= BIT(csg_id);
		}
	}

	if (ctx->timedout_mask)
		return -ETIMEDOUT;

	return 0;
}

struct pancsf_sched_tick_ctx {
	struct list_head old_groups[PANCSF_CSG_PRIORITY_COUNT];
	struct list_head groups[PANCSF_CSG_PRIORITY_COUNT];
	u32 idle_group_count;
	u32 group_count;
	enum pancsf_csg_priority min_priority;
	struct pancsf_vm *vms[MAX_CS_PER_CSG];
	u32 as_count;
	bool immediate_tick;
	u32 csg_upd_failed_mask;
};

static bool
tick_ctx_is_full(const struct pancsf_scheduler *sched,
		 const struct pancsf_sched_tick_ctx *ctx)
{
	return ctx->group_count == sched->csg_slot_count;
}

static bool
pancsf_group_is_idle(struct pancsf_group *group)
{
	struct pancsf_device *pfdev = group->pfdev;
	u32 inactive_streams;

	if (group->csg_id >= 0)
		return pfdev->scheduler->csg_slots[group->csg_id].idle;

	inactive_streams = group->idle_streams | group->blocked_streams;
	return hweight32(inactive_streams) == group->stream_count;
}

static bool
pancsf_group_can_run(struct pancsf_group *group)
{
	return group->state != PANCSF_CS_GROUP_TERMINATED &&
	       !group->destroyed && group->fatal_streams == 0 &&
	       !group->timedout;
}

static void
tick_ctx_pick_groups_from_queue(const struct pancsf_scheduler *sched,
				struct pancsf_sched_tick_ctx *ctx,
				struct list_head *queue,
				bool skip_idle_groups)
{
	struct pancsf_group *group, *tmp;

	if (tick_ctx_is_full(sched, ctx))
		return;

	list_for_each_entry_safe(group, tmp, queue, run_node) {
		u32 i;

		if (!pancsf_group_can_run(group))
			continue;

		if (skip_idle_groups && pancsf_group_is_idle(group))
			continue;

		for (i = 0; i < ctx->as_count; i++) {
			if (ctx->vms[i] == group->vm)
				break;
		}

		if (i == ctx->as_count && ctx->as_count == sched->as_slot_count)
			continue;

		if (!group->in_tick_ctx) {
			pancsf_group_get(group);
			group->in_tick_ctx = true;
		}

		list_move_tail(&group->run_node, &ctx->groups[group->priority]);
		ctx->group_count++;
		if (pancsf_group_is_idle(group))
			ctx->idle_group_count++;

		if (i == ctx->as_count)
			ctx->vms[ctx->as_count++] = group->vm;

		if (ctx->min_priority > group->priority)
			ctx->min_priority = group->priority;

		if (tick_ctx_is_full(sched, ctx))
			return;
	}
}

static void
tick_ctx_insert_old_group(struct pancsf_scheduler *sched,
			  struct pancsf_sched_tick_ctx *ctx,
			  struct pancsf_group *group,
			  bool full_tick)
{
	struct pancsf_csg_slot *csg_slot = &sched->csg_slots[group->csg_id];
	struct pancsf_group *other_group;

	if (!full_tick) {
		list_add_tail(&group->run_node, &ctx->old_groups[group->priority]);
		return;
	}

	/* Rotate to make sure groups with lower CSG slot
	 * priorities have a chance to get a higher CSG slot
	 * priority next time they get picked. This priority
	 * has an impact on resource request ordering, so it's
	 * important to make sure we don't let one group starve
	 * all other groups with the same group priority.
	 */
	list_for_each_entry(other_group,
			    &ctx->old_groups[csg_slot->group->priority],
			    run_node) {
		struct pancsf_csg_slot *other_csg_slot = &sched->csg_slots[other_group->csg_id];

		if (other_csg_slot->priority > csg_slot->priority) {
			list_add_tail(&csg_slot->group->run_node, &other_group->run_node);
			return;
		}
	}

	list_add_tail(&group->run_node, &ctx->old_groups[group->priority]);
}

static void pancsf_sched_queue_reset_locked(struct pancsf_device *pfdev)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;

	if (!sched->reset_pending) {
		sched->reset_pending = true;
		queue_work(sched->wq, &sched->reset_work);
	}
}

void pancsf_sched_queue_reset(struct pancsf_device *pfdev)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;

	mutex_lock(&sched->lock);
	pancsf_sched_queue_reset_locked(pfdev);
	mutex_unlock(&sched->lock);
}

static void
tick_ctx_init(struct pancsf_scheduler *sched,
	      struct pancsf_sched_tick_ctx *ctx,
	      bool full_tick)
{
	struct pancsf_fw_global_iface *glb_iface;
	struct pancsf_device *pfdev = sched->pfdev;
	struct pancsf_csg_slots_upd_ctx upd_ctx;
	int ret;
	u32 i;

	glb_iface = pancsf_get_glb_iface(pfdev);
	memset(ctx, 0, sizeof(*ctx));
	csgs_upd_ctx_init(&upd_ctx);

	ctx->min_priority = PANCSF_CSG_PRIORITY_COUNT;
	for (i = 0; i < ARRAY_SIZE(ctx->groups); i++) {
		INIT_LIST_HEAD(&ctx->groups[i]);
		INIT_LIST_HEAD(&ctx->old_groups[i]);
	}

	for (i = 0; i < sched->csg_slot_count; i++) {
		struct pancsf_csg_slot *csg_slot = &sched->csg_slots[i];
		const struct pancsf_fw_csg_iface *csg_iface;

		csg_iface = pancsf_get_csg_iface(pfdev, i);
		if (csg_slot->group) {
			pancsf_group_get(csg_slot->group);
			tick_ctx_insert_old_group(sched, ctx, csg_slot->group, full_tick);

			csg_slot->group->in_tick_ctx = true;
			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, i,
						csg_iface->output->ack ^ CSG_STATUS_UPDATE,
						CSG_STATUS_UPDATE);
		}
	}

	ret = csgs_upd_ctx_apply_locked(pfdev, &upd_ctx);
	if (ret) {
		pancsf_sched_queue_reset_locked(pfdev);
		ctx->csg_upd_failed_mask |= upd_ctx.timedout_mask;
	}
}

static void
pancsf_group_term_post_processing(struct pancsf_group *group)
{
	bool cookie;
	u32 i = 0;

	if (WARN_ON(pancsf_group_can_run(group)))
		return;

	cookie = dma_fence_begin_signalling();
	for (i = 0; i < group->stream_count; i++) {
		struct pancsf_queue *stream = group->streams[i];
		struct pancsf_job *job;
		int err;

		if (group->fatal_streams & BIT(i))
			err = -EINVAL;
		else if (group->timedout)
			err = -ETIMEDOUT;
		else
			err = -ECANCELED;

		if (!stream)
			continue;

		list_for_each_entry(job, &stream->jobs.in_flight, node) {
			dma_fence_set_error(job->done_fence, err);
			dma_fence_signal(job->done_fence);
		}

		list_for_each_entry(job, &stream->jobs.pending, node) {
			dma_fence_set_error(job->done_fence, -ECANCELED);
			dma_fence_signal(job->done_fence);
		}
	}
	dma_fence_end_signalling(cookie);

	for (i = 0; i < group->stream_count; i++) {
		struct pancsf_queue *stream = group->streams[i];
		struct pancsf_job *job, *job_tmp;

		list_for_each_entry_safe(job, job_tmp, &stream->jobs.in_flight, node) {
			list_del_init(&job->node);
			if (cancel_delayed_work(&job->timeout_work))
				pancsf_put_job(job);
			pancsf_put_job(job);
		}

		list_for_each_entry_safe(job, job_tmp, &stream->jobs.pending, node) {
			list_del_init(&job->node);
			pancsf_put_job(job);
		}
	}
}

static void pancsf_group_term_work(struct work_struct *work)
{
	struct pancsf_group *group = container_of(work,
						       struct pancsf_group,
						       term_work);
	pancsf_group_term_post_processing(group);
	pancsf_group_put(group);
}

static void
tick_ctx_cleanup(struct pancsf_scheduler *sched,
		 struct pancsf_sched_tick_ctx *ctx)
{
	struct pancsf_group *group, *tmp;
	u32 i;

	for (i = 0; i < ARRAY_SIZE(ctx->old_groups); i++) {
		list_for_each_entry_safe(group, tmp, &ctx->old_groups[i], run_node) {
			/* If everything went fine, we should only have groups
			 * to be terminated in the old_groups lists.
			 */
			WARN_ON(!ctx->csg_upd_failed_mask &&
				pancsf_group_can_run(group));

			if (!pancsf_group_can_run(group)) {
				list_del_init(&group->run_node);
				pancsf_group_queue_work(sched, group, term);
			} else if (group->csg_id >= 0) {
				list_del_init(&group->run_node);
			} else {
				list_move(&group->run_node,
					  pancsf_group_is_idle(group) ?
					  &sched->idle_queues[group->priority] :
					  &sched->run_queues[group->priority]);
			}
			pancsf_group_put(group);
			group->in_tick_ctx = false;
		}
	}

	for (i = 0; i < ARRAY_SIZE(ctx->groups); i++) {
		/* If everything went fine, the groups to schedule lists should
		 * be empty.
		 */
		WARN_ON(!ctx->csg_upd_failed_mask && !list_empty(&ctx->groups[i]));

		list_for_each_entry_safe(group, tmp, &ctx->groups[i], run_node) {
			if (group->csg_id >= 0) {
				list_del_init(&group->run_node);
			} else {
				list_move(&group->run_node,
					  pancsf_group_is_idle(group) ?
					  &sched->idle_queues[group->priority] :
					  &sched->run_queues[group->priority]);
			}
			pancsf_group_put(group);
			group->in_tick_ctx = false;
		}
	}
}

static void
tick_ctx_apply(struct pancsf_scheduler *sched, struct pancsf_sched_tick_ctx *ctx)
{
	struct pancsf_group *group, *tmp;
	struct pancsf_device *pfdev = sched->pfdev;
	struct pancsf_fw_global_iface *glb_iface;
	struct pancsf_csg_slot *csg_slot;
	int prio, new_csg_prio = MAX_CSG_PRIO, i;
	u32 csg_mod_mask = 0, free_csg_slots = 0;
	struct pancsf_csg_slots_upd_ctx upd_ctx;
	int ret;

	csgs_upd_ctx_init(&upd_ctx);
	glb_iface = pancsf_get_glb_iface(pfdev);

	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		/* Suspend or terminate evicted groups. */
		list_for_each_entry(group, &ctx->old_groups[prio], run_node) {
			const struct pancsf_fw_csg_iface *csg_iface;
			bool term = !pancsf_group_can_run(group);
			int csg_id = group->csg_id;

			if (WARN_ON(csg_id < 0))
				continue;

			csg_slot = &sched->csg_slots[csg_id];
			csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, csg_id,
						term ? CSG_STATE_TERMINATE : CSG_STATE_SUSPEND,
						CSG_STATE_MASK);
		}

		/* Update priorities on already running groups. */
		list_for_each_entry(group, &ctx->groups[prio], run_node) {
			const struct pancsf_fw_csg_iface *csg_iface;
			int csg_id = group->csg_id;
			u32 ep_req;

			if (csg_id < 0) {
				new_csg_prio--;
				continue;
			}

			csg_slot = &sched->csg_slots[csg_id];
			csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
			if (csg_slot->priority == new_csg_prio) {
				new_csg_prio--;
				continue;
			}

			ep_req = pancsf_update_reqs(csg_iface->input->endpoint_req,
						    CSG_EP_REQ_PRIORITY(new_csg_prio),
						    CSG_EP_REQ_PRIORITY_MASK);
			csg_iface->input->endpoint_req = ep_req;
			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, csg_id,
						csg_iface->output->ack ^ CSG_ENDPOINT_CONFIG,
						CSG_ENDPOINT_CONFIG);
			new_csg_prio--;
		}
	}

	ret = csgs_upd_ctx_apply_locked(pfdev, &upd_ctx);
	if (ret) {
		pancsf_sched_queue_reset_locked(pfdev);
		ctx->csg_upd_failed_mask |= upd_ctx.timedout_mask;
		return;
	}

	/* Unbind evicted groups. */
	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		list_for_each_entry(group, &ctx->old_groups[prio], run_node) {
			pancsf_unbind_group_locked(group);
		}
	}

	for (i = 0; i < sched->csg_slot_count; i++) {
		if (!sched->csg_slots[i].group)
			free_csg_slots |= BIT(i);
	}

	csgs_upd_ctx_init(&upd_ctx);
	new_csg_prio = MAX_CSG_PRIO;

	/* Start new groups. */
	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		list_for_each_entry(group, &ctx->groups[prio], run_node) {
			int csg_id = group->csg_id;
			const struct pancsf_fw_csg_iface *csg_iface;

			if (csg_id >= 0) {
				new_csg_prio--;
				continue;
			}

			csg_id = ffs(free_csg_slots) - 1;
			if (WARN_ON(csg_id < 0))
				break;

			csg_iface = pancsf_get_csg_iface(pfdev, csg_id);
			csg_slot = &sched->csg_slots[csg_id];
			csg_mod_mask |= BIT(csg_id);
			pancsf_bind_group_locked(group, csg_id);
			pancsf_prog_csg_slot_locked(pfdev, csg_id, new_csg_prio--);
			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, csg_id,
						group->state == PANCSF_CS_GROUP_SUSPENDED ?
						CSG_STATE_RESUME : CSG_STATE_START,
						CSG_STATE_MASK);
			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, csg_id,
						csg_iface->output->ack ^ CSG_ENDPOINT_CONFIG,
						CSG_ENDPOINT_CONFIG);
			free_csg_slots &= ~BIT(csg_id);
		}
	}

	ret = csgs_upd_ctx_apply_locked(pfdev, &upd_ctx);
	if (ret) {
		pancsf_sched_queue_reset_locked(pfdev);
		ctx->csg_upd_failed_mask |= upd_ctx.timedout_mask;
		return;
	}

	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		list_for_each_entry_safe(group, tmp, &ctx->groups[prio], run_node) {
			list_del_init(&group->run_node);
			group->in_tick_ctx = false;

			/* If the group has been destroyed while we were
			 * scheduling, ask for an immediate tick to
			 * re-evaluate as soon as possible and get rid of
			 * this dangling group.
			 */
			if (group->destroyed)
				ctx->immediate_tick = true;
			pancsf_group_put(group);
		}

		/* Return evicted groups to the idle or run queues. Groups
		 * that can no longer be run (because they've been destroyed
		 * or experienced an unrecoverable error) will be scheduled
		 * for destruction in tick_ctx_cleanup().
		 */
		list_for_each_entry_safe(group, tmp, &ctx->old_groups[prio], run_node) {
			if (!pancsf_group_can_run(group))
				continue;

			if (pancsf_group_is_idle(group))
				list_move_tail(&group->run_node, &sched->idle_queues[prio]);
			else
				list_move_tail(&group->run_node, &sched->run_queues[prio]);
			group->in_tick_ctx = false;
			pancsf_group_put(group);
		}
	}

	sched->used_csg_slot_count = ctx->group_count;
	sched->might_have_idle_groups = ctx->idle_group_count > 0;
}

static u64
tick_ctx_update_resched_target(struct pancsf_scheduler *sched,
			       const struct pancsf_sched_tick_ctx *ctx)
{
	/* We had space left, no need to reschedule until some external event happens. */
	if (!tick_ctx_is_full(sched, ctx))
		goto no_tick;

	/* If idle groups were scheduled, no need to wake up until some external
	 * event happens (group unblocked, new job submitted, ...).
	 */
	if (ctx->idle_group_count)
		goto no_tick;

	if (WARN_ON(ctx->min_priority >= PANCSF_CSG_PRIORITY_COUNT))
		goto no_tick;

	/* If there are groups of the same priority waiting, we need to
	 * keep the scheduler ticking, otherwise, we'll just wait for
	 * new groups with higher priority to be queued.
	 */
	if (!list_empty(&sched->run_queues[ctx->min_priority])) {
		u64 resched_target = sched->last_tick + sched->tick_period;

		if (time_before64(sched->resched_target, sched->last_tick) ||
		    time_before64(resched_target, sched->resched_target))
			sched->resched_target = resched_target;

		return sched->resched_target - sched->last_tick;
	}

no_tick:
	sched->resched_target = U64_MAX;
	return U64_MAX;
}

static void pancsf_tick_work(struct work_struct *work)
{
	struct pancsf_scheduler *sched = container_of(work, struct pancsf_scheduler,
						      tick_work.work);
	struct pancsf_sched_tick_ctx ctx;
	u64 remaining_jiffies = 0, resched_delay;
	u64 now = get_jiffies_64();
	int prio;

	if (time_before64(now, sched->resched_target))
		remaining_jiffies = sched->resched_target - now;

	mutex_lock(&sched->lock);
	if (sched->reset_pending)
		goto out_unlock;

	tick_ctx_init(sched, &ctx, remaining_jiffies != 0);
	if (ctx.csg_upd_failed_mask)
		goto out_cleanup_ctx;

	if (remaining_jiffies) {
		/* Scheduling forced in the middle of a tick. Only RT groups
		 * can preempt non-RT ones. Currently running RT groups can't be
		 * preempted.
		 */
		for (prio = PANCSF_CSG_PRIORITY_COUNT - 1;
		     prio >= 0 && !tick_ctx_is_full(sched, &ctx);
		     prio--) {
			tick_ctx_pick_groups_from_queue(sched, &ctx, &ctx.old_groups[prio], true);
			if (prio == PANCSF_CSG_PRIORITY_RT) {
				tick_ctx_pick_groups_from_queue(sched, &ctx,
								&sched->run_queues[prio], true);
			}
		}
	}

	/* First pick non-idle groups */
	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1;
	     prio >= 0 && !tick_ctx_is_full(sched, &ctx);
	     prio--) {
		tick_ctx_pick_groups_from_queue(sched, &ctx, &sched->run_queues[prio], true);
		tick_ctx_pick_groups_from_queue(sched, &ctx, &ctx.old_groups[prio], true);
	}

	/* If we have free CSG slots left, pick idle groups */
	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1;
	     prio >= 0 && !tick_ctx_is_full(sched, &ctx);
	     prio--) {
		/* Check the old_group queue first to avoid reprogramming the slots */
		tick_ctx_pick_groups_from_queue(sched, &ctx, &ctx.old_groups[prio], false);
		tick_ctx_pick_groups_from_queue(sched, &ctx, &sched->idle_queues[prio], false);
	}

	tick_ctx_apply(sched, &ctx);
	if (ctx.csg_upd_failed_mask)
		goto out_cleanup_ctx;

	sched->last_tick = now;
	resched_delay = tick_ctx_update_resched_target(sched, &ctx);
	if (ctx.immediate_tick)
		resched_delay = 0;

	if (resched_delay != U64_MAX)
		mod_delayed_work(sched->wq, &sched->tick_work, resched_delay);

out_cleanup_ctx:
	tick_ctx_cleanup(sched, &ctx);

out_unlock:
	mutex_unlock(&sched->lock);
}

static void *
pancsf_queue_get_syncwait_obj(struct pancsf_group *group, struct pancsf_queue *stream)
{
	struct iosys_map map;
	int ret;

	if (stream->syncwait.kmap)
		return stream->syncwait.kmap + stream->syncwait.offset;

	if (!stream->syncwait.bo) {
		stream->syncwait.bo = pancsf_vm_get_bo_for_vma(group->vm, stream->syncwait.gpu_va,
							       &stream->syncwait.offset);
		if (WARN_ON(IS_ERR_OR_NULL(stream->syncwait.bo)))
			return NULL;
	}

	ret = drm_gem_shmem_vmap(&stream->syncwait.bo->base, &map);
	if (WARN_ON(ret))
		return NULL;

	stream->syncwait.kmap = map.vaddr;
	if (WARN_ON(!stream->syncwait.kmap))
		return NULL;

	return stream->syncwait.kmap + stream->syncwait.offset;
}

static int pancsf_queue_eval_syncwait(struct pancsf_group *group, u8 stream_idx)
{
	struct pancsf_queue *stream = group->streams[stream_idx];
	union {
		struct pancsf_syncobj_64b sync64;
		struct pancsf_syncobj_32b sync32;
	} *syncobj;
	bool result;
	u64 value;

	syncobj = pancsf_queue_get_syncwait_obj(group, stream);
	if (!syncobj)
		return -EINVAL;

	value = stream->syncwait.sync64 ?
		syncobj->sync64.seqno :
		syncobj->sync32.seqno;

	if (stream->syncwait.gt)
		result = value > stream->syncwait.ref;
	else
		result = value <= stream->syncwait.ref;

	if (result) {
		pancsf_queue_release_syncwait_obj(group, stream);
		return 1;
	}

	return 0;
}

static void pancsf_sync_upd_work(struct work_struct *work)
{
	struct pancsf_scheduler *sched = container_of(work,
						      struct pancsf_scheduler,
						      sync_upd_work);
	struct pancsf_group *group, *tmp;
	bool immediate_tick = false;

	mutex_lock(&sched->lock);
	list_for_each_entry_safe(group, tmp, &sched->wait_queue, wait_node) {
		u32 tested_streams = group->blocked_streams;
		u32 unblocked_streams = 0;

		while (tested_streams) {
			u32 cs_id = ffs(tested_streams) - 1;
			int ret;

			ret = pancsf_queue_eval_syncwait(group, cs_id);
			WARN_ON(ret < 0);
			if (ret)
				unblocked_streams |= BIT(cs_id);

			tested_streams &= ~BIT(cs_id);
		}

		if (unblocked_streams) {
			group->blocked_streams &= ~unblocked_streams;

			if (group->csg_id < 0) {
				list_move(&group->run_node, &sched->run_queues[group->priority]);
				if (group->priority == PANCSF_CSG_PRIORITY_RT)
					immediate_tick = true;
			}
		}

		if (!group->blocked_streams)
			list_del_init(&group->wait_node);
	}
	mutex_unlock(&sched->lock);

	if (immediate_tick)
		mod_delayed_work(sched->wq, &sched->tick_work, 0);
}

static void pancsf_group_queue_locked(struct pancsf_group *group, u32 stream_mask)
{
	struct pancsf_device *pfdev = group->pfdev;
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct list_head *queue = &sched->run_queues[group->priority];
	u64 delay_jiffies = 0;
	bool was_idle;
	u64 now;

	if (!pancsf_group_can_run(group))
		return;

	/* All updated streams are blocked, no need to wake up the scheduler. */
	if ((stream_mask & group->blocked_streams) == stream_mask)
		return;

	/* Group is being evaluated by the scheduler. */
	if (group->in_tick_ctx)
		return;

	was_idle = pancsf_group_is_idle(group);
	group->idle_streams &= ~stream_mask;
	if (was_idle && !pancsf_group_is_idle(group))
		list_move_tail(&group->run_node, queue);

	/* RT groups are preemptive. */
	if (group->priority == PANCSF_CSG_PRIORITY_RT) {
		mod_delayed_work(sched->wq, &sched->tick_work, 0);
		return;
	}

	/* Some groups might be idle, force an immediate tick to
	 * re-evaluate.
	 */
	if (sched->might_have_idle_groups) {
		mod_delayed_work(sched->wq, &sched->tick_work, 0);
		return;
	}

	/* Scheduler is ticking, nothing to do. */
	if (sched->resched_target != U64_MAX) {
		/* If there are free slots, force immediating ticking. */
		if (sched->used_csg_slot_count < sched->csg_slot_count)
			mod_delayed_work(sched->wq, &sched->tick_work, 0);

		return;
	}

	/* Scheduler tick was off, recalculate the resched_target based on the
	 * last tick event, and queue the scheduler work.
	 */
	now = get_jiffies_64();
	sched->resched_target = sched->last_tick + sched->tick_period;
	if (sched->used_csg_slot_count == sched->csg_slot_count &&
	    time_before64(now, sched->resched_target))
		delay_jiffies = min_t(unsigned long, sched->resched_target - now, ULONG_MAX);

	mod_delayed_work(sched->wq, &sched->tick_work, delay_jiffies);
}

static void pancsf_group_job_pending_work(struct work_struct *work)
{
	struct pancsf_group *group = container_of(work, struct pancsf_group, job_pending_work);
	struct pancsf_device *pfdev = group->pfdev;
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_job *job;
	u32 stream_mask = 0;
	u32 i;

	for (i = 0; i < group->stream_count; i++) {
		struct pancsf_queue *stream = group->streams[i];

		if (!stream)
			continue;

		while (true) {
			spin_lock(&stream->jobs.lock);
			job = list_first_entry_or_null(&stream->jobs.pending,
						       struct pancsf_job, node);
			spin_unlock(&stream->jobs.lock);

			if (!job ||
			    !pancsf_queue_can_take_new_jobs(stream) ||
			    !pancsf_job_deps_done(job))
				break;

			pancsf_queue_submit_job(stream, job);
			stream_mask |= BIT(i);
		}
	}

	if (stream_mask) {
		mutex_lock(&sched->lock);
		if (group->csg_id < 0)
			pancsf_group_queue_locked(group, stream_mask);
		else
			gpu_write(pfdev, CSF_DOORBELL(group->csg_id + 1), 1);
		mutex_unlock(&sched->lock);
	}

	pancsf_group_put(group);
}

static void pancsf_ping_work(struct work_struct *work)
{
	struct pancsf_scheduler *sched = container_of(work,
						      struct pancsf_scheduler,
						      ping_work.work);
	struct pancsf_device *pfdev = sched->pfdev;
	const struct pancsf_fw_global_iface *glb_iface = pancsf_get_glb_iface(pfdev);
	bool reset_pending, timedout = false;

	mutex_lock(&sched->lock);
	reset_pending = sched->reset_pending;
	if (!reset_pending) {
		sched->pending_reqs |= GLB_PING;
		glb_iface->input->req = pancsf_toggle_reqs(glb_iface->input->req,
							   glb_iface->output->ack,
							   GLB_PING);
		gpu_write(pfdev, CSF_DOORBELL(CSF_GLB_DOORBELL_ID), 1);
	}
	mutex_unlock(&sched->lock);

	if (reset_pending)
		goto out;

	if (!wait_event_timeout(sched->reqs_acked,
				!(sched->pending_reqs & GLB_PING),
				msecs_to_jiffies(100))) {
		mutex_lock(&sched->lock);
		if ((sched->pending_reqs & GLB_PING) != 0 &&
		    ((glb_iface->input->req ^ glb_iface->output->ack) & GLB_PING) != 0)
			timedout = true;
		mutex_unlock(&sched->lock);
	}

	if (timedout) {
		dev_err(pfdev->dev, "FW ping timeout, scheduling a reset");
		pancsf_sched_queue_reset(pfdev);
	} else {
		/* Next ping in 12 seconds. */
		mod_delayed_work(sched->wq,
				 &sched->ping_work,
				 msecs_to_jiffies(12000));
	}

out:
}

static int pancsf_sched_pre_reset(struct pancsf_device *pfdev)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_csg_slots_upd_ctx upd_ctx;
	u64 suspended_slots, faulty_slots;
	int ret;
	u32 i;

	mutex_lock(&sched->lock);
	csgs_upd_ctx_init(&upd_ctx);
	for (i = 0; i < sched->csg_slot_count; i++) {
		struct pancsf_csg_slot *csg_slot = &sched->csg_slots[i];

		if (csg_slot->group) {
			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, i,
						CSG_STATE_SUSPEND,
						CSG_STATE_MASK);
		}
	}

	suspended_slots = upd_ctx.update_mask;

	ret = csgs_upd_ctx_apply_locked(pfdev, &upd_ctx);
	suspended_slots &= ~upd_ctx.timedout_mask;
	faulty_slots = upd_ctx.timedout_mask;

	if (faulty_slots) {
		u32 slot_mask = faulty_slots;

		dev_err(pfdev->dev, "CSG suspend failed, escalating to termination");
		csgs_upd_ctx_init(&upd_ctx);
		while (slot_mask) {
			u32 csg_id = ffs(slot_mask) - 1;

			csgs_upd_ctx_queue_reqs(pfdev, &upd_ctx, csg_id,
						CSG_STATE_TERMINATE,
						CSG_STATE_MASK);
			slot_mask &= ~BIT(csg_id);
		}

		csgs_upd_ctx_apply_locked(pfdev, &upd_ctx);

		slot_mask = faulty_slots;
		while (slot_mask) {
			u32 csg_id = ffs(slot_mask) - 1;
			struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];

			/* Terminate command timedout, but the soft-reset will
			 * automatically terminate all active groups, so let's
			 * force the state to halted here.
			 */
			if (csg_slot->group->state != PANCSF_CS_GROUP_TERMINATED)
				csg_slot->group->state = PANCSF_CS_GROUP_TERMINATED;
			slot_mask &= ~BIT(csg_id);
		}
	}

	/* Flush L2 and LSC caches to make sure suspend state is up-to-date.
	 * If the flush fails, flag all streams for termination.
	 */
	if (suspended_slots) {
		bool flush_caches_failed = false;
		u32 slot_mask = suspended_slots;

		if (pancsf_gpu_flush_caches(pfdev, CACHE_CLEAN, CACHE_CLEAN, 0))
			flush_caches_failed = true;

		while (slot_mask) {
			u32 csg_id = ffs(slot_mask) - 1;
			struct pancsf_csg_slot *csg_slot = &sched->csg_slots[csg_id];

			if (flush_caches_failed)
				csg_slot->group->state = PANCSF_CS_GROUP_TERMINATED;
			else
				pancsf_queue_csg_sync_update_locked(pfdev, csg_id);

			slot_mask &= ~BIT(csg_id);
		}

		if (flush_caches_failed)
			faulty_slots |= suspended_slots;
	}

	for (i = 0; i < sched->csg_slot_count; i++) {
		struct pancsf_csg_slot *csg_slot = &sched->csg_slots[i];
		struct pancsf_group *group = csg_slot->group;

		if (!group)
			continue;

		pancsf_group_get(group);
		pancsf_unbind_group_locked(group);

		if (pancsf_group_can_run(group)) {
			WARN_ON(!list_empty(&group->run_node));
			list_add(&group->run_node,
				 pancsf_group_is_idle(group) ?
				 &sched->idle_queues[group->priority] :
				 &sched->run_queues[group->priority]);
		} else {
			pancsf_group_queue_work(sched, group, term);
		}
		pancsf_group_put(group);
	}

	mutex_unlock(&sched->lock);

	return 0;
}

static void pancsf_reset_work(struct work_struct *work)
{
	struct pancsf_scheduler *sched = container_of(work,
						      struct pancsf_scheduler,
						      reset_work);
	struct pancsf_device *pfdev = sched->pfdev;
	bool full_reload = false;
	int ret;

	pancsf_sched_pre_reset(pfdev);

retry:
	pancsf_mcu_pre_reset(pfdev);
	pancsf_mmu_pre_reset(pfdev);
	pancsf_gpu_soft_reset(pfdev);
	pancsf_gpu_l2_power_on(pfdev);
	pancsf_mmu_reset(pfdev);
	ret = pancsf_mcu_reset(pfdev, full_reload);
	if (ret && !full_reload) {
		full_reload = true;
		goto retry;
	}

	if (WARN_ON(ret && full_reload))
		dev_err(pfdev->dev, "Failed to boot MCU after reset");

	pancsf_global_init(pfdev);

	mutex_lock(&sched->lock);
	sched->reset_pending = false;
	mutex_unlock(&sched->lock);
	mod_delayed_work(pfdev->scheduler->wq,
			 &pfdev->scheduler->tick_work,
			 0);
}

static void pancsf_group_sync_upd_work(struct work_struct *work)
{
	struct pancsf_group *group = container_of(work,
						       struct pancsf_group,
						       sync_upd_work);
	struct pancsf_job *job, *job_tmp;
	LIST_HEAD(done_jobs);
	u32 stream_idx;
	bool cookie;

	cookie = dma_fence_begin_signalling();
	for (stream_idx = 0; stream_idx < group->stream_count; stream_idx++) {
		struct pancsf_queue *stream = group->streams[stream_idx];
		struct pancsf_syncobj_64b *syncobj;

		if (!stream)
			continue;

		syncobj = group->syncobjs.kmap + (stream_idx * sizeof(*syncobj));

		spin_lock(&stream->jobs.lock);
		list_for_each_entry_safe(job, job_tmp, &stream->jobs.in_flight, node) {
			struct pancsf_queue_fence *fence;
			u64 job_seqno = (job->ringbuf.start / (NUM_INSTRS_PER_SLOT * sizeof(u64))) + 1;

			if (!job->call_info.size)
				continue;

			fence = container_of(job->done_fence, struct pancsf_queue_fence, base);
			if (syncobj->seqno < job_seqno)
				break;

			list_move_tail(&job->node, &done_jobs);
		}
		spin_unlock(&stream->jobs.lock);
	}

	list_for_each_entry(job, &done_jobs, node) {
		if (cancel_delayed_work(&job->timeout_work))
			pancsf_put_job(job);

		dma_fence_signal(job->done_fence);
	}
	dma_fence_end_signalling(cookie);

	list_for_each_entry_safe(job, job_tmp, &done_jobs, node) {
		list_del_init(&job->node);
		pancsf_put_job(job);
	}

	pancsf_group_put(group);
}

int pancsf_create_group(struct pancsf_file *pfile,
			const struct drm_pancsf_group_create *group_args,
			const struct drm_pancsf_queue_create *queue_args)
{
	struct pancsf_device *pfdev = pfile->pfdev;
	struct pancsf_group_pool *gpool = pfile->groups;
	struct pancsf_scheduler *sched = pfdev->scheduler;
	const struct pancsf_fw_csg_iface *csg_iface = pancsf_get_csg_iface(pfdev, 0);
	struct pancsf_group *group = NULL;
	u32 gid, i, suspend_size;
	int ret;

	if (group_args->priority > PANCSF_CSG_PRIORITY_HIGH)
		return -EINVAL;

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	spin_lock_init(&group->fatal_lock);
	kref_init(&group->refcount);
	group->state = PANCSF_CS_GROUP_CREATED;
	group->as_id = -1;
	group->csg_id = -1;

	group->pfdev = pfdev;
	group->max_compute_cores = group_args->max_compute_cores;
	group->compute_core_mask = group_args->compute_core_mask & pfdev->gpu_info.shader_present;
	group->max_fragment_cores = group_args->max_fragment_cores;
	group->fragment_core_mask = group_args->fragment_core_mask & pfdev->gpu_info.shader_present;
	group->max_tiler_cores = group_args->max_tiler_cores;
	group->tiler_core_mask = group_args->tiler_core_mask & pfdev->gpu_info.tiler_present;
	group->priority = group_args->priority;

	INIT_LIST_HEAD(&group->wait_node);
	INIT_LIST_HEAD(&group->run_node);
	INIT_WORK(&group->job_pending_work, pancsf_group_job_pending_work);
	INIT_WORK(&group->term_work, pancsf_group_term_work);
	INIT_WORK(&group->sync_upd_work, pancsf_group_sync_upd_work);

	if ((hweight64(group->compute_core_mask) == 0 && group_args->max_compute_cores > 0) ||
	    (hweight64(group->fragment_core_mask) == 0 && group_args->max_fragment_cores > 0) ||
	    (hweight64(group->tiler_core_mask) == 0 && group_args->max_tiler_cores > 0) ||
	    (group->tiler_core_mask == 0 && !group->heaps)) {
		ret = -EINVAL;
		goto err_put_group;
	}

	group->vm = pancsf_vm_pool_get_vm(pfile->vms, group_args->vm_id);
	if (!group->vm) {
		ret = -EINVAL;
		goto err_put_group;
	}

	/* We need to instantiate the heap pool if the group wants to use the tiler. */
	if (group->tiler_core_mask) {
		mutex_lock(&pfile->heaps_lock);
		if (IS_ERR_OR_NULL(pfile->heaps))
			pfile->heaps = pancsf_heap_pool_create(pfdev, group->vm);
		mutex_unlock(&pfile->heaps_lock);

		if (IS_ERR(pfile->heaps)) {
			ret = PTR_ERR(pfile->heaps);
			goto err_put_group;
		}

		group->heaps = pfile->heaps;
	}

	if (IS_ERR_OR_NULL(pfile->heaps))
		group->heaps = pfile->heaps;

	suspend_size = csg_iface->control->suspend_size;
	group->suspend_buf = pancsf_fw_alloc_suspend_buf_mem(pfdev, suspend_size);
	if (IS_ERR(group->suspend_buf)) {
		ret = PTR_ERR(group->suspend_buf);
		group->suspend_buf = NULL;
		goto err_put_group;
	}

	suspend_size = csg_iface->control->protm_suspend_size;
	group->protm_suspend_buf = pancsf_fw_alloc_suspend_buf_mem(pfdev, suspend_size);
	if (IS_ERR(group->protm_suspend_buf)) {
		ret = PTR_ERR(group->protm_suspend_buf);
		group->protm_suspend_buf = NULL;
		goto err_put_group;
	}

	group->syncobjs.bo = pancsf_gem_create_and_map(pfdev, group->vm,
						       group_args->queues.count *
						       sizeof(struct pancsf_syncobj_64b),
						       0,
						       PANCSF_VMA_MAP_NOEXEC |
						       PANCSF_VMA_MAP_UNCACHED |
						       PANCSF_VMA_MAP_AUTO_VA,
						       &group->syncobjs.gpu_va,
						       (void **)&group->syncobjs.kmap);
	if (IS_ERR(group->syncobjs.bo)) {
		ret = PTR_ERR(group->syncobjs.bo);
		goto err_put_group;
	}

	memset(group->syncobjs.kmap, 0, group_args->queues.count * sizeof(struct pancsf_syncobj_64b));

	for (i = 0; i < group_args->queues.count; i++) {
		group->streams[i] = pancsf_create_queue(group, &queue_args[i]);
		if (IS_ERR(group->streams[i])) {
			ret = PTR_ERR(group->streams[i]);
			group->streams[i] = NULL;
			goto err_put_group;
		}

		group->stream_count++;
	}

	group->idle_streams = GENMASK(group->stream_count - 1, 0);

	mutex_lock(&gpool->lock);
	ret = xa_alloc(&gpool->xa, &gid, group, XA_LIMIT(1, sched->csg_slot_count), GFP_KERNEL);
	mutex_unlock(&gpool->lock);

	if (ret)
		goto err_put_group;

	mutex_lock(&sched->lock);
	list_add_tail(&group->run_node, &sched->idle_queues[group->priority]);
	mutex_unlock(&sched->lock);
	return gid;

err_put_group:
	pancsf_group_put(group);
	return ret;
}

void pancsf_destroy_group(struct pancsf_file *pfile, u32 group_handle)
{
	struct pancsf_group_pool *gpool = pfile->groups;
	struct pancsf_device *pfdev = pfile->pfdev;
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_group *group;

	mutex_lock(&gpool->lock);
	group = xa_erase(&gpool->xa, group_handle);
	mutex_unlock(&gpool->lock);

	if (!group)
		return;

	mutex_lock(&sched->lock);
	group->destroyed = true;
	if (group->in_tick_ctx || group->csg_id >= 0) {
		mod_delayed_work(sched->wq, &sched->tick_work, 0);
	} else {
		/* Remove from the run queues, so the scheduler can't
		 * pick the group on the next tick.
		 */
		list_del_init(&group->run_node);
		list_del_init(&group->wait_node);

		pancsf_group_queue_work(sched, group, term);
	}
	mutex_unlock(&sched->lock);

	pancsf_group_put(group);
}

int pancsf_group_get_state(struct pancsf_file *pfile,
			   struct drm_pancsf_group_get_state *get_state)
{
	struct pancsf_group_pool *gpool = pfile->groups;
	struct pancsf_device *pfdev = pfile->pfdev;
	struct pancsf_scheduler *sched = pfdev->scheduler;
	struct pancsf_group *group;

	mutex_lock(&gpool->lock);
	group = xa_load(&gpool->xa, get_state->group_handle);
	if (group)
		pancsf_group_get(group);
	mutex_unlock(&gpool->lock);

	if (!group)
		return -EINVAL;

	memset(get_state, 0, sizeof(*get_state));

	mutex_lock(&sched->lock);
	if (group->destroyed)
		get_state->state |= DRM_PANCSF_GROUP_STATE_DESTROYED;
	if (group->timedout)
		get_state->state |= DRM_PANCSF_GROUP_STATE_TIMEDOUT;
	if (group->fatal_streams) {
		get_state->state |= DRM_PANCSF_GROUP_STATE_FATAL_FAULT;
		get_state->fatal_queues = group->fatal_streams;
	}
	mutex_unlock(&sched->lock);

	pancsf_group_put(group);
	return 0;
}

int pancsf_group_pool_create(struct pancsf_file *pfile)
{
	struct pancsf_group_pool *gpool;

	gpool = kzalloc(sizeof(*gpool), GFP_KERNEL);
	if (!gpool)
		return -ENOMEM;

	xa_init_flags(&gpool->xa, XA_FLAGS_ALLOC1);
	mutex_init(&gpool->lock);
	pfile->groups = gpool;
	return 0;
}

void pancsf_group_pool_destroy(struct pancsf_file *pfile)
{
	struct pancsf_group_pool *gpool = pfile->groups;
	struct pancsf_group *group;
	unsigned long i;

	if (IS_ERR_OR_NULL(gpool))
		return;

	xa_for_each(&gpool->xa, i, group)
		pancsf_destroy_group(pfile, i);

	mutex_destroy(&gpool->lock);
	kfree(gpool);
	pfile->groups = NULL;
}

static void pancsf_release_job(struct kref *ref)
{
	struct pancsf_job *job = container_of(ref, struct pancsf_job, refcount);
	struct dma_fence *fence;
	unsigned long index;

	WARN_ON(cancel_delayed_work(&job->timeout_work));

	if (job->cur_dep) {
		dma_fence_remove_callback(job->cur_dep, &job->dep_cb);
		dma_fence_put(job->cur_dep);
	}

	xa_for_each(&job->dependencies, index, fence) {
		dma_fence_put(fence);
	}
	xa_destroy(&job->dependencies);

	dma_fence_put(job->done_fence);

	pancsf_group_put(job->group);

	kfree(job);
}

void pancsf_put_job(struct pancsf_job *job)
{
	if (job)
		kref_put(&job->refcount, pancsf_release_job);
}

int pancsf_add_job_dep(struct pancsf_job *job, struct dma_fence *fence)
{
	struct dma_fence *entry;
	unsigned long index;
	u32 id = 0;
	int ret;

	if (!fence)
		return 0;

	/* Deduplicate if we already depend on a fence from the same context.
	 * This lets the size of the array of deps scale with the number of
	 * engines involved, rather than the number of BOs.
	 */
	xa_for_each(&job->dependencies, index, entry) {
		if (entry->context != fence->context)
			continue;

		if (dma_fence_is_later(fence, entry)) {
			dma_fence_put(entry);
			xa_store(&job->dependencies, index, fence, GFP_KERNEL);
		} else {
			dma_fence_put(fence);
		}
		return 0;
	}

	ret = xa_alloc(&job->dependencies, &id, fence, xa_limit_32b, GFP_KERNEL);
	if (ret != 0)
		dma_fence_put(fence);

	return ret;
}

struct dma_fence *pancsf_get_job_done_fence(struct pancsf_job *job)
{
	return job->done_fence;
}

int pancsf_push_job(struct pancsf_job *job)
{
	struct pancsf_group *group = job->group;
	struct pancsf_queue *stream = group->streams[job->stream_idx];
	struct pancsf_device *pfdev = group->pfdev;
	bool kick_group = false;
	int ret = 0;

	kref_get(&job->refcount);

	spin_lock(&group->fatal_lock);
	if (pancsf_group_can_run(group)) {
		spin_lock(&stream->jobs.lock);
		kick_group = list_empty(&stream->jobs.pending);
		list_add_tail(&job->node, &stream->jobs.pending);
		spin_unlock(&stream->jobs.lock);
	} else {
		ret = -EINVAL;
	}
	spin_unlock(&group->fatal_lock);

	if (kick_group)
		pancsf_group_queue_work(pfdev->scheduler, group, job_pending);

	if (ret)
		pancsf_put_job(job);

	return ret;
}

static void pancsf_job_timeout_work(struct work_struct *work)
{
	struct pancsf_job *job = container_of(work, struct pancsf_job, timeout_work.work);
	struct pancsf_group *group = job->group;
	struct pancsf_device *pfdev = group->pfdev;
	struct pancsf_scheduler *sched = pfdev->scheduler;

	mutex_lock(&sched->lock);
	group->timedout = true;
	if (group->in_tick_ctx || group->csg_id >= 0) {
		mod_delayed_work(pfdev->scheduler->wq,
				 &pfdev->scheduler->tick_work,
				 0);
	} else {
		/* Remove from the run queues, so the scheduler can't
		 * pick the group on the next tick.
		 */
		list_del_init(&group->run_node);
		list_del_init(&group->wait_node);

		pancsf_group_queue_work(pfdev->scheduler, group, term);
	}
	mutex_unlock(&sched->lock);

	pancsf_put_job(job);
}

struct pancsf_job *
pancsf_create_job(struct pancsf_file *pfile,
		  u16 group_handle, u8 stream_idx,
		  const struct pancsf_call_info *cs_call)
{
	struct pancsf_group_pool *gpool = pfile->groups;
	struct pancsf_group *group;
	struct pancsf_job *job;
	struct dma_fence *done_fence;
	int ret;

	if (!cs_call)
		return ERR_PTR(-EINVAL);

	if (!cs_call->size || !cs_call->start)
		return ERR_PTR(-EINVAL);

	/* bits 24:30 must be zero. */
	if (cs_call->latest_flush & GENMASK(30, 24))
		return ERR_PTR(-EINVAL);

	mutex_lock(&gpool->lock);
	group = xa_load(&gpool->xa, group_handle);
	if (group)
		pancsf_group_get(group);
	mutex_unlock(&gpool->lock);

	if (!group)
		return ERR_PTR(-EINVAL);

	if (stream_idx >= group->stream_count || !group->streams[stream_idx])
		return ERR_PTR(-EINVAL);

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job) {
		ret = -ENOMEM;
		goto err_put_group;
	}

	INIT_DELAYED_WORK(&job->timeout_work, pancsf_job_timeout_work);
	job->group = group;
	job->stream_idx = stream_idx;
	job->call_info = *cs_call;

	done_fence = pancsf_queue_fence_create(group->streams[stream_idx]);
	if (IS_ERR(done_fence)) {
		ret = PTR_ERR(done_fence);
		goto err_free_job;
	}

	job->done_fence = done_fence;

	xa_init_flags(&job->dependencies, XA_FLAGS_ALLOC);
	kref_init(&job->refcount);

	return job;

err_free_job:
	kfree(job);

err_put_group:
	pancsf_group_put(group);
	return ERR_PTR(ret);
}

int pancsf_sched_init(struct pancsf_device *pfdev)
{
	const struct pancsf_fw_global_iface *glb_iface = pancsf_get_glb_iface(pfdev);
	const struct pancsf_fw_csg_iface *csg_iface = pancsf_get_csg_iface(pfdev, 0);
	const struct pancsf_fw_cs_iface *cs_iface = pancsf_get_cs_iface(pfdev, 0, 0);
	u32 gpu_as_count, num_groups, i;
	struct pancsf_scheduler *sched;
	int prio;

	sched = devm_kzalloc(pfdev->dev, sizeof(*sched), GFP_KERNEL);
	if (!sched)
		return -ENOMEM;

	/* The highest bit in JOB_INT_* is reserved for globabl IRQs. That
	 * leaves 31 bits for CSG IRQs, hence the MAX_CSGS clamp here.
	 */
	num_groups = min_t(u32, MAX_CSGS, glb_iface->control->group_num);

	/* The FW-side scheduler might deadlock if two groups with the same
	 * priority try to access a set of resources that overlaps, with part
	 * of the resources being allocated to one group and the other part to
	 * the other group, both groups waiting for the remaining resources to
	 * be allocated. To avoid that, it is recommended to assign each CSG a
	 * different priority. In theory we could allow several groups to have
	 * the same CSG priority if they don't request the same resources, but
	 * that makes the scheduling logic more complicated, so let's clamp
	 * the number of CSG slots to MAX_CSG_PRIO + 1 for now.
	 */
	num_groups = min_t(u32, MAX_CSG_PRIO + 1, num_groups);

	/* We need at least one AS for the MCU and one for the GPU contexts. */
	gpu_as_count = hweight32(pfdev->gpu_info.as_present & GENMASK(31, 1));
	if (!gpu_as_count) {
		dev_err(pfdev->dev, "Not enough AS (%d, expected at least 2)",
			gpu_as_count + 1);
		return -EINVAL;
	}

	sched->pfdev = pfdev;
	sched->sb_slot_count = CS_FEATURES_SCOREBOARDS(cs_iface->control->features);
	sched->csg_slot_count = num_groups;
	sched->cs_slot_count = csg_iface->control->stream_num;
	sched->as_slot_count = gpu_as_count;
	pfdev->csif_info.csg_slot_count = sched->csg_slot_count;
	pfdev->csif_info.cs_slot_count = sched->cs_slot_count;
	pfdev->csif_info.scoreboard_slot_count = sched->sb_slot_count;

	sched->last_tick = 0;
	sched->resched_target = U64_MAX;
	sched->tick_period = msecs_to_jiffies(10);
	INIT_DELAYED_WORK(&sched->tick_work, pancsf_tick_work);
	INIT_DELAYED_WORK(&sched->ping_work, pancsf_ping_work);
	INIT_WORK(&sched->sync_upd_work, pancsf_sync_upd_work);
	INIT_WORK(&sched->reset_work, pancsf_reset_work);

	mutex_init(&sched->lock);
	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		INIT_LIST_HEAD(&sched->run_queues[prio]);
		INIT_LIST_HEAD(&sched->idle_queues[prio]);
	}
	INIT_LIST_HEAD(&sched->wait_queue);

	init_waitqueue_head(&sched->reqs_acked);
	for (i = 0; i < num_groups; i++)
		init_waitqueue_head(&sched->csg_slots[i].reqs_acked);

	pfdev->scheduler = sched;

	sched->wq = alloc_ordered_workqueue("panfrost-csf-sched", 0);
	if (!sched->wq) {
		dev_err(pfdev->dev, "Failed to allocate the scheduler workqueue");
		return -ENOMEM;
	}

	pancsf_global_init(pfdev);
	return 0;
}

void pancsf_sched_fini(struct pancsf_device *pfdev)
{
	struct pancsf_scheduler *sched = pfdev->scheduler;
	int prio;

	if (!sched || !sched->csg_slot_count)
		return;

	cancel_work_sync(&sched->reset_work);
	cancel_work_sync(&sched->sync_upd_work);
	cancel_delayed_work_sync(&sched->tick_work);
	cancel_delayed_work_sync(&sched->ping_work);

	if (sched->wq)
		destroy_workqueue(sched->wq);

	for (prio = PANCSF_CSG_PRIORITY_COUNT - 1; prio >= 0; prio--) {
		WARN_ON(!list_empty(&sched->run_queues[prio]));
		WARN_ON(!list_empty(&sched->idle_queues[prio]));
	}

	WARN_ON(!list_empty(&sched->wait_queue));
	mutex_destroy(&sched->lock);

	pfdev->scheduler = NULL;
}
