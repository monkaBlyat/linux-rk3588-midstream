// SPDX-License-Identifier: GPL-2.0
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */

#include <drm/pancsf_drm.h>
#include <drm/gpu_scheduler.h>

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/rwsem.h>
#include <linux/shmem_fs.h>
#include <linux/sizes.h>

#include "pancsf_device.h"
#include "pancsf_mmu.h"
#include "pancsf_sched.h"
#include "pancsf_gem.h"
#include "pancsf_regs.h"

#define mmu_write(dev, reg, data) writel(data, (dev)->iomem + (reg))
#define mmu_read(dev, reg) readl((dev)->iomem + (reg))

#define MM_COLOR_FRAG_SHADER		BIT(0)

#define MAX_AS_SLOTS			32

struct pancsf_vm;

struct pancsf_mmu {
	int irq;

	struct {
		struct mutex slots_lock;
		unsigned long in_use_mask;
		unsigned long alloc_mask;
		unsigned long faulty_mask;
		struct pancsf_vm *slots[MAX_AS_SLOTS];
		struct list_head lru_list;
		spinlock_t op_lock;
	} as;
};

struct pancsf_vm_pool {
	struct xarray xa;
	struct mutex lock;
};

struct pancsf_vm {
	struct pancsf_device *pfdev;
	struct kref refcount;
	u64 memattr;
	struct io_pgtable_cfg pgtbl_cfg;
	struct io_pgtable_ops *pgtbl_ops;

	/* VM reservation object. All private BOs should use this resv
	 * instead of the GEM object one.
	 */
	struct dma_resv resv;
	struct mutex lock;
	struct drm_mm mm;

	u64 min_va;
	u64 max_va;

	int as;
	atomic_t as_count;
	bool for_mcu;
	struct list_head node;
};

struct pancsf_vma {
	struct drm_mm_node vm_mm_node;
	struct rb_node bo_vma_node;
	struct pancsf_gem_object *bo;
	struct pancsf_vm *vm;
	u64 offset;
	u32 flags;
	bool mapped;
};

static int wait_ready(struct pancsf_device *pfdev, u32 as_nr)
{
	int ret;
	u32 val;

	/* Wait for the MMU status to indicate there is no active command, in
	 * case one is pending.
	 */
	ret = readl_relaxed_poll_timeout_atomic(pfdev->iomem + AS_STATUS(as_nr),
						val, !(val & AS_STATUS_AS_ACTIVE),
						10, 100000);

	if (ret) {
		/* The GPU hung, let's trigger a reset */
		pancsf_sched_queue_reset(pfdev);
		dev_err(pfdev->dev, "AS_ACTIVE bit stuck\n");
	}

	return ret;
}

static int write_cmd(struct pancsf_device *pfdev, u32 as_nr, u32 cmd)
{
	int status;

	/* write AS_COMMAND when MMU is ready to accept another command */
	status = wait_ready(pfdev, as_nr);
	if (!status)
		mmu_write(pfdev, AS_COMMAND(as_nr), cmd);

	return status;
}

static void lock_region(struct pancsf_device *pfdev, u32 as_nr,
			u64 region_start, u64 size)
{
	u8 region_width;
	u64 region;
	u64 region_end = region_start + size;

	if (!size)
		return;

	/*
	 * The locked region is a naturally aligned power of 2 block encoded as
	 * log2 minus(1).
	 * Calculate the desired start/end and look for the highest bit which
	 * differs. The smallest naturally aligned block must include this bit
	 * change, the desired region starts with this bit (and subsequent bits)
	 * zeroed and ends with the bit (and subsequent bits) set to one.
	 */
	region_width = max(fls64(region_start ^ (region_end - 1)),
			   const_ilog2(AS_LOCK_REGION_MIN_SIZE)) - 1;

	/*
	 * Mask off the low bits of region_start (which would be ignored by
	 * the hardware anyway)
	 */
	region_start &= GENMASK_ULL(63, region_width);

	region = region_width | region_start;

	/* Lock the region that needs to be updated */
	mmu_write(pfdev, AS_LOCKADDR_LO(as_nr), lower_32_bits(region));
	mmu_write(pfdev, AS_LOCKADDR_HI(as_nr), upper_32_bits(region));
	write_cmd(pfdev, as_nr, AS_COMMAND_LOCK);
}

static int mmu_hw_do_operation_locked(struct pancsf_device *pfdev, int as_nr,
				      u64 iova, u64 size, u32 op)
{
	if (as_nr < 0)
		return 0;

	if (op != AS_COMMAND_UNLOCK)
		lock_region(pfdev, as_nr, iova, size);

	/* Run the MMU operation */
	write_cmd(pfdev, as_nr, op);

	/* Wait for the flush to complete */
	return wait_ready(pfdev, as_nr);
}

static int mmu_hw_do_operation(struct pancsf_vm *vm,
			       u64 iova, u64 size, u32 op)
{
	struct pancsf_device *pfdev = vm->pfdev;
	int ret;

	spin_lock(&pfdev->mmu->as.op_lock);
	ret = mmu_hw_do_operation_locked(pfdev, vm->as, iova, size, op);
	spin_unlock(&pfdev->mmu->as.op_lock);
	return ret;
}

static void pancsf_mmu_as_enable(struct pancsf_device *pfdev, u32 as_nr,
				 u64 transtab, u64 transcfg, u64 memattr)
{
	mmu_hw_do_operation_locked(pfdev, as_nr, 0, ~0ULL, AS_COMMAND_FLUSH_MEM);

	mmu_write(pfdev, AS_TRANSTAB_LO(as_nr), lower_32_bits(transtab));
	mmu_write(pfdev, AS_TRANSTAB_HI(as_nr), upper_32_bits(transtab));

	mmu_write(pfdev, AS_MEMATTR_LO(as_nr), lower_32_bits(memattr));
	mmu_write(pfdev, AS_MEMATTR_HI(as_nr), upper_32_bits(memattr));

	mmu_write(pfdev, AS_TRANSCFG_LO(as_nr), lower_32_bits(transcfg));
	mmu_write(pfdev, AS_TRANSCFG_HI(as_nr), upper_32_bits(transcfg));

	write_cmd(pfdev, as_nr, AS_COMMAND_UPDATE);
}

static void pancsf_mmu_as_disable(struct pancsf_device *pfdev, u32 as_nr)
{
	mmu_hw_do_operation_locked(pfdev, as_nr, 0, ~0ULL, AS_COMMAND_FLUSH_MEM);

	mmu_write(pfdev, AS_TRANSTAB_LO(as_nr), 0);
	mmu_write(pfdev, AS_TRANSTAB_HI(as_nr), 0);

	mmu_write(pfdev, AS_MEMATTR_LO(as_nr), 0);
	mmu_write(pfdev, AS_MEMATTR_HI(as_nr), 0);

	mmu_write(pfdev, AS_TRANSCFG_LO(as_nr), AS_TRANSCFG_ADRMODE_UNMAPPED);
	mmu_write(pfdev, AS_TRANSCFG_HI(as_nr), 0);

	write_cmd(pfdev, as_nr, AS_COMMAND_UPDATE);
}

static bool pancsf_vm_va_is_valid(struct pancsf_vm *vm, u64 addr)
{
	return addr >= vm->min_va && addr <= vm->max_va;
}

static void pancsf_vm_enable(struct pancsf_vm *vm)
{
	struct pancsf_device *pfdev = vm->pfdev;
	struct io_pgtable_cfg *cfg = &vm->pgtbl_cfg;
	u64 transtab, transcfg;

	transtab = cfg->arm_lpae_s1_cfg.ttbr;
	transcfg = AS_TRANSCFG_PTW_MEMATTR_WB |
		   AS_TRANSCFG_PTW_RA |
		   AS_TRANSCFG_ADRMODE_AARCH64_4K;
	if (pfdev->coherent)
		transcfg |= AS_TRANSCFG_PTW_SH_OS;

	pancsf_mmu_as_enable(vm->pfdev, vm->as, transtab, transcfg, vm->memattr);
}

static void pancsf_vm_disable(struct pancsf_vm *vm)
{
	pancsf_mmu_as_disable(vm->pfdev, vm->as);
}

static u32 pancsf_mmu_fault_mask(struct pancsf_device *pfdev, u32 value)
{
	/* Bits 16 to 31 mean REQ_COMPLETE. */
	return value & GENMASK(15, 0);
}

static u32 pancsf_mmu_as_fault_mask(struct pancsf_device *pfdev, u32 as)
{
	return BIT(as);
}

int pancsf_vm_as_get(struct pancsf_vm *vm)
{
	struct pancsf_device *pfdev = vm->pfdev;
	int as;

	mutex_lock(&pfdev->mmu->as.slots_lock);

	as = vm->as;
	if (as >= 0) {
		u32 mask = pancsf_mmu_as_fault_mask(pfdev, as);

		atomic_inc(&vm->as_count);
		list_move(&vm->node, &pfdev->mmu->as.lru_list);

		if (pfdev->mmu->as.faulty_mask & mask) {
			/* Unhandled pagefault on this AS, the MMU was
			 * disabled. We need to re-enable the MMU after
			 * clearing+unmasking the AS interrupts.
			 */
			mmu_write(pfdev, MMU_INT_CLEAR, mask);
			pfdev->mmu->as.faulty_mask &= ~mask;
			mmu_write(pfdev, MMU_INT_MASK, ~pfdev->mmu->as.faulty_mask);
			pancsf_vm_enable(vm);
		}

		goto out;
	}

	/* Check for a free AS */
	if (vm->for_mcu) {
		WARN_ON(pfdev->mmu->as.alloc_mask & BIT(0));
		as = 0;
	} else {
		as = ffz(pfdev->mmu->as.alloc_mask | BIT(0));
	}

	if (!(BIT(as) & pfdev->gpu_info.as_present)) {
		struct pancsf_vm *lru_vm;

		list_for_each_entry_reverse(lru_vm, &pfdev->mmu->as.lru_list, node) {
			if (!atomic_read(&lru_vm->as_count))
				break;
		}
		WARN_ON(&lru_vm->node == &pfdev->mmu->as.lru_list);

		list_del_init(&lru_vm->node);
		as = lru_vm->as;

		WARN_ON(as < 0);
		lru_vm->as = -1;
	}

	/* Assign the free or reclaimed AS to the FD */
	vm->as = as;
	pfdev->mmu->as.slots[as] = vm;
	set_bit(as, &pfdev->mmu->as.alloc_mask);
	atomic_set(&vm->as_count, 1);
	list_add(&vm->node, &pfdev->mmu->as.lru_list);

	pancsf_vm_enable(vm);

out:
	mutex_unlock(&pfdev->mmu->as.slots_lock);
	return as;
}

void pancsf_vm_as_put(struct pancsf_vm *vm)
{
	atomic_dec(&vm->as_count);
	WARN_ON(atomic_read(&vm->as_count) < 0);
}

void pancsf_mmu_pre_reset(struct pancsf_device *pfdev)
{
	mmu_write(pfdev, MMU_INT_MASK, 0);

	mutex_lock(&pfdev->mmu->as.slots_lock);
	/* Flag all AS faulty on reset so the interrupts doesn't get re-enabled
	 * in the interrupt handler if it's running concurrently.
	 */
	pfdev->mmu->as.faulty_mask = ~0;
	mutex_unlock(&pfdev->mmu->as.slots_lock);

	synchronize_irq(pfdev->mmu->irq);
}

void pancsf_mmu_reset(struct pancsf_device *pfdev)
{
	struct pancsf_vm *vm, *vm_tmp;

	mutex_lock(&pfdev->mmu->as.slots_lock);

	pfdev->mmu->as.alloc_mask = 0;
	pfdev->mmu->as.faulty_mask = 0;

	list_for_each_entry_safe(vm, vm_tmp, &pfdev->mmu->as.lru_list, node) {
		vm->as = -1;
		atomic_set(&vm->as_count, 0);
		list_del_init(&vm->node);
	}

	memset(pfdev->mmu->as.slots, 0, sizeof(pfdev->mmu->as.slots));
	mutex_unlock(&pfdev->mmu->as.slots_lock);

	mmu_write(pfdev, MMU_INT_CLEAR, pancsf_mmu_fault_mask(pfdev, ~0));
	mmu_write(pfdev, MMU_INT_MASK, pancsf_mmu_fault_mask(pfdev, ~0));
}

static size_t get_pgsize(u64 addr, size_t size, size_t *count)
{
	/*
	 * io-pgtable only operates on multiple pages within a single table
	 * entry, so we need to split at boundaries of the table size, i.e.
	 * the next block size up. The distance from address A to the next
	 * boundary of block size B is logically B - A % B, but in unsigned
	 * two's complement where B is a power of two we get the equivalence
	 * B - A % B == (B - A) % B == (n * B - A) % B, and choose n = 0 :)
	 */
	size_t blk_offset = -addr % SZ_2M;

	if (blk_offset || size < SZ_2M) {
		*count = min_not_zero(blk_offset, size) / SZ_4K;
		return SZ_4K;
	}
	blk_offset = -addr % SZ_1G ?: SZ_1G;
	*count = min(blk_offset, size) / SZ_2M;
	return SZ_2M;
}

static int pancsf_vm_flush_range(struct pancsf_vm *vm, u64 iova, u64 size)
{
	struct pancsf_device *pfdev = vm->pfdev;
	int ret = 0;

	if (vm->as < 0)
		return 0;

	pm_runtime_get_noresume(pfdev->dev);

	/* Flush the PTs only if we're already awake */
	if (pm_runtime_active(pfdev->dev))
		ret = mmu_hw_do_operation(vm, iova, size, AS_COMMAND_FLUSH_PT);

	pm_runtime_put_sync_autosuspend(pfdev->dev);
	return ret;
}

int pancsf_vm_unmap_pages(struct pancsf_vm *vm, u64 iova, size_t size)
{
	struct pancsf_device *pfdev = vm->pfdev;
	struct io_pgtable_ops *ops = vm->pgtbl_ops;
	size_t remaining = size;
	int ret;

	dev_dbg(pfdev->dev, "unmap: as=%d, iova=%llx, len=%zx",	vm->as, iova, size);

	while (remaining) {
		size_t unmapped_sz = 0, pgcount;
		size_t pgsize = get_pgsize(iova, remaining, &pgcount);

		if (ops->iova_to_phys(ops, iova)) {
			unmapped_sz = ops->unmap_pages(ops, iova, pgsize, pgcount, NULL);
			/* Unmapping might involve splitting 2MB ptes into 4K ones,
			 * which might fail on memory allocation. Assume this is what
			 * happened when unmap_pages() returns 0.
			 */
			if (WARN_ON(!unmapped_sz)) {
				ret = -ENOMEM;
				break;
			}
		}

		/* If we couldn't unmap the whole range, skip a page, and try again. */
		if (unmapped_sz < pgsize * pgcount)
			unmapped_sz += pgsize;

		iova += unmapped_sz;
		remaining -= unmapped_sz;
	}

	/* Range might be partially unmapped, but flushing the TLB for the whole range
	 * is safe and simpler.
	 */
	return pancsf_vm_flush_range(vm, iova, size);
}

static int
pancsf_vm_map_pages(struct pancsf_vm *vm, u64 iova, int prot,
		    struct sg_table *sgt, u64 offset, ssize_t size)
{
	struct pancsf_device *pfdev = vm->pfdev;
	unsigned int count;
	struct scatterlist *sgl;
	struct io_pgtable_ops *ops = vm->pgtbl_ops;
	u64 start_iova = iova;
	int ret;

	if (!size)
		return 0;

	for_each_sgtable_dma_sg(sgt, sgl, count) {
		dma_addr_t paddr = sg_dma_address(sgl);
		size_t len = sg_dma_len(sgl);

		if (len <= offset) {
			offset -= len;
			continue;
		}

		paddr -= offset;
		len -= offset;

		if (size >= 0) {
			len = min_t(size_t, len, size);
			size -= len;
		}

		dev_dbg(pfdev->dev, "map: as=%d, iova=%llx, paddr=%llx, len=%zx",
			vm->as, iova, paddr, len);

		while (len) {
			size_t pgcount, mapped = 0;
			size_t pgsize = get_pgsize(iova | paddr, len, &pgcount);

			ret = ops->map_pages(ops, iova, paddr, pgsize, pgcount, prot,
					     GFP_KERNEL, &mapped);
			/* Don't get stuck if things have gone wrong */
			mapped = max(mapped, pgsize);
			iova += mapped;
			paddr += mapped;
			len -= mapped;

			if (ret) {
				/* If something failed, unmap what we've already mapped before
				 * returning. The unmap call is not supposed to fail.
				 */
				WARN_ON(pancsf_vm_unmap_pages(vm, start_iova, iova - start_iova));
				return ret;
			}
		}

		if (!size)
			break;
	}

	return pancsf_vm_flush_range(vm, start_iova, iova - start_iova);
}

static int
pancsf_vma_map_locked(struct pancsf_vma *vma, struct sg_table *sgt)
{
	int prot = 0;
	int ret;

	if (WARN_ON(vma->mapped))
		return 0;

	if (vma->flags & PANCSF_VMA_MAP_NOEXEC)
		prot |= IOMMU_NOEXEC;

	if (!(vma->flags & PANCSF_VMA_MAP_UNCACHED))
		prot |= IOMMU_CACHE;

	if (vma->flags & PANCSF_VMA_MAP_READONLY)
		prot |= IOMMU_READ;
	else
		prot |= IOMMU_READ | IOMMU_WRITE;

	if (vma->bo)
		sgt = drm_gem_shmem_get_pages_sgt(&vma->bo->base);

	if (IS_ERR(sgt))
		return PTR_ERR(sgt);
	else if (!sgt)
		return -EINVAL;

	ret = pancsf_vm_map_pages(vma->vm, vma->vm_mm_node.start << PAGE_SHIFT, prot,
				  sgt, vma->offset, vma->vm_mm_node.size << PAGE_SHIFT);
	if (!ret)
		vma->mapped = true;

	return ret;
}

static int
pancsf_vma_unmap_locked(struct pancsf_vma *vma)
{
	int ret;

	if (WARN_ON(!vma->mapped))
		return 0;

	ret = pancsf_vm_unmap_pages(vma->vm, vma->vm_mm_node.start << PAGE_SHIFT,
				    vma->vm_mm_node.size << PAGE_SHIFT);
	if (!ret)
		vma->mapped = false;

	return ret;
}

#define PANCSF_GPU_AUTO_VA_RANGE_START	0x800000000000ull
#define PANCSF_GPU_AUTO_VA_RANGE_END	0x1000000000000ull

static void
pancsf_vm_free_vma_locked(struct pancsf_vma *vma)
{
	struct pancsf_gem_object *bo = vma->bo;

	if (WARN_ON(vma->mapped)) {
		/* Leak BO/VMA memory if we can't unmap. That's better than
		 * letting the GPU access a region that can be re-allocated.
		 */
		if (WARN_ON(pancsf_vma_unmap_locked(vma)))
			return;
	}

	if (vma->vm)
		lockdep_assert_held(&vma->vm->lock);

	if (bo)
		drm_gem_object_put(&bo->base.base);

	if (drm_mm_node_allocated(&vma->vm_mm_node))
		drm_mm_remove_node(&vma->vm_mm_node);

	kfree(vma);
}

static struct pancsf_vma *
pancsf_vm_alloc_vma_locked(struct pancsf_vm *vm, size_t size, u64 va, u32 flags)
{
	bool is_exec = !(flags & PANCSF_VMA_MAP_NOEXEC);
	u64 range_start, range_end, align;
	struct pancsf_vma *vma;
	unsigned int color = 0;
	int ret;

	lockdep_assert_held(&vm->lock);

	if (flags & PANCSF_VMA_MAP_AUTO_VA)
		va = 0;

	if (vm->for_mcu || !size || ((size | va) & ~PAGE_MASK) != 0)
		return ERR_PTR(-EINVAL);

	if (!(flags & PANCSF_VMA_MAP_AUTO_VA)) {
		/* Explicit VA assignment, we don't add coloring. If VA is
		 * inappropriate it's the caller responsibility.
		 */
		range_start = va >> PAGE_SHIFT;
		range_end = (va + size) >> PAGE_SHIFT;
		align = 1;
	} else {
		range_start = PANCSF_GPU_AUTO_VA_RANGE_START >> PAGE_SHIFT;
		range_end = PANCSF_GPU_AUTO_VA_RANGE_END >> PAGE_SHIFT;

		if (is_exec) {
			/* JUMP instructions can't cross a 4GB boundary, but
			 * JUMP_EX ones can. We assume any executable VMA smaller
			 * than 4GB expects things to fit in a single 4GB block.
			 * Let's align on the closest power-of-2 size to guarantee
			 * that. For anything bigger, we align the mapping to 4GB
			 * and assume userspace uses JUMP_EX where appropriate.
			 */
			if (size < SZ_4G) {
				u64 aligned_size = 1 << const_ilog2(size);

				if (aligned_size != size)
					aligned_size <<= 1;

				align = aligned_size >> PAGE_SHIFT;
			} else {
				align = SZ_4G;
			}
		} else {
			/* Align to 2MB is the buffer is bigger than 2MB. */
			align = (size >= SZ_2M ? SZ_2M : PAGE_SIZE) >> PAGE_SHIFT;
		}

		/* Fragment shaders might call blend shaders which need to
		 * be in the same 4GB block. We reserve the last 16M of such
		 * VMAs to map our blend shaders. Mapping blend shaders has
		 * to be done using a VM_BIND with an explicit VA.
		 */
		if (flags & PANCSF_VMA_MAP_FRAG_SHADER)
			color |= MM_COLOR_FRAG_SHADER;
	}

	vma = kzalloc(sizeof(*vma), GFP_KERNEL);
	if (!vma)
		return ERR_PTR(-ENOMEM);

	vma->flags = flags;
	vma->vm = vm;
	ret = drm_mm_insert_node_in_range(&vm->mm, &vma->vm_mm_node,
					  size >> PAGE_SHIFT, align, color,
					  range_start, range_end,
					  DRM_MM_INSERT_BEST);

	if (ret)
		goto err_free_vma;

	return vma;

err_free_vma:
	pancsf_vm_free_vma_locked(vma);
	return ERR_PTR(ret);
}

int
pancsf_vm_map_bo_range(struct pancsf_vm *vm, struct pancsf_gem_object *bo,
		       u64 offset, size_t size, u64 *va, u32 flags)
{
	struct pancsf_vma *vma;
	int ret = 0;

	/* Make sure the VA and size are aligned and in-bounds. */
	if (size > bo->base.base.size || offset > bo->base.base.size - size)
		return -EINVAL;

	mutex_lock(&vm->lock);
	vma = pancsf_vm_alloc_vma_locked(vm, size, *va, flags);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto out_unlock;
	}

	drm_gem_object_get(&bo->base.base);
	vma->bo = bo;

	if (!(flags & PANCSF_VMA_MAP_ON_FAULT)) {
		ret = pancsf_vma_map_locked(vma, NULL);
		if (ret)
			goto out_unlock;
	}

	*va = vma->vm_mm_node.start << PAGE_SHIFT;

out_unlock:
	if (ret && !IS_ERR(vma))
		pancsf_vm_free_vma_locked(vma);

	mutex_unlock(&vm->lock);

	return ret;
}

#define drm_mm_for_each_node_in_range_safe(node__, next_node__, mm__, start__, end__) \
	for (node__ = __drm_mm_interval_first((mm__), (start__), (end__) - 1), \
	     next_node__ = list_next_entry(node__, node_list); \
	     node__->start < (end__); \
	     node__ = next_node__, next_node__ = list_next_entry(node__, node_list))

int
pancsf_vm_unmap_range(struct pancsf_vm *vm, u64 va, size_t size)
{
	struct drm_mm_node *mm_node, *tmp_mm_node;
	struct pancsf_vma *first = NULL, *last = NULL;
	size_t first_new_size = 0, first_unmap_size = 0, last_new_size = 0, last_unmap_size = 0;
	u64 first_va, first_unmap_va = 0, last_va, last_unmap_va = 0, last_new_offset = 0;
	int ret = 0;

	if ((va | size) & ~PAGE_MASK)
		return -EINVAL;

	if (va + size < va)
		return -EINVAL;

	if (!size)
		return 0;

	if (!pancsf_vm_va_is_valid(vm, va) || !pancsf_vm_va_is_valid(vm, va + size - 1))
		return -EINVAL;

	mutex_lock(&vm->lock);

	drm_mm_for_each_node_in_range_safe(mm_node, tmp_mm_node, &vm->mm, va >> PAGE_SHIFT,
					   (va + size) >> PAGE_SHIFT) {
		struct pancsf_vma *vma = container_of(mm_node, struct pancsf_vma, vm_mm_node);
		u64 vma_va = mm_node->start << PAGE_SHIFT;
		size_t vma_size = mm_node->size << PAGE_SHIFT;

		if (vma_va < va) {
			first = vma;
			first_va = vma_va;
			first_new_size = va - vma_va;
			first_unmap_size = vma_size - first_new_size;
			first_unmap_va = va;
		}

		if (vma_va + vma_size > va + size) {
			last = vma;
			last_va = va + size;
			last_new_size = vma_va + vma_size - last_va;
			last_new_offset = last->offset +
					  ((last->vm_mm_node.size << PAGE_SHIFT) - last_new_size);
			last_unmap_size = vma_size - last_new_size;
			last_unmap_va = vma_va;

			/* Partial unmap in the middle of the only VMA. We need to create a
			 * new VMA.
			 */
			if (first == last) {
				struct pancsf_vma *last = kzalloc(sizeof(*vma), GFP_KERNEL);

				if (!last) {
					ret = -ENOMEM;
					goto out_unlock;
				}

				last_unmap_va = 0;
				last_unmap_size = 0;
				first_unmap_size -= last_new_size;
				last->flags = first->flags;
				last->bo = first->bo;
				last->mapped = first->mapped;
				drm_gem_object_get(&last->bo->base.base);
			}
		}

		mm_node = list_next_entry(mm_node, node_list);

		if (vma != last && vma != first) {
			ret = pancsf_vma_unmap_locked(vma);
			if (ret)
				goto out_unlock;

			pancsf_vm_free_vma_locked(vma);
		}
	}

	/* Re-insert first and last VMAs if the unmap was partial. */
	if (first) {
		ret = pancsf_vm_unmap_pages(vm, first_unmap_va >> PAGE_SHIFT,
					    first_unmap_size >> PAGE_SHIFT);
		if (ret)
			goto out_unlock;

		drm_mm_remove_node(&first->vm_mm_node);
		drm_mm_insert_node_in_range(&vm->mm, &first->vm_mm_node,
					    first_new_size >> PAGE_SHIFT, 1, 0,
					    first_va >> PAGE_SHIFT,
					    (first_va + first_new_size) >> PAGE_SHIFT,
					    DRM_MM_INSERT_BEST);
	}

	if (last) {
		if (drm_mm_node_allocated(&last->vm_mm_node)) {
			ret = pancsf_vm_unmap_pages(vm, last_unmap_va >> PAGE_SHIFT,
						    last_unmap_size >> PAGE_SHIFT);
			if (ret)
				goto out_unlock;

			drm_mm_remove_node(&last->vm_mm_node);
		}

		last->offset = last_new_offset;
		drm_mm_insert_node_in_range(&vm->mm, &last->vm_mm_node, last_new_size, 1, 0,
					    last_va, last_va + last_new_size,
					    DRM_MM_INSERT_BEST);
	}

out_unlock:
	mutex_unlock(&vm->lock);

	return ret;
}

struct pancsf_gem_object *
pancsf_vm_get_bo_for_vma(struct pancsf_vm *vm, u64 va, u64 *bo_offset)
{
	struct pancsf_gem_object *bo = ERR_PTR(-ENOENT);
	struct pancsf_vma *vma = NULL;
	struct drm_mm_node *mm_node;
	u64 vma_va;

	if (!pancsf_vm_va_is_valid(vm, va))
		return ERR_PTR(-EINVAL);

	mutex_lock(&vm->lock);
	drm_mm_for_each_node_in_range(mm_node, &vm->mm, va >> PAGE_SHIFT, (va >> PAGE_SHIFT) + 1) {
		vma = container_of(mm_node, struct pancsf_vma, vm_mm_node);
		break;
	}

	if (vma && vma->bo) {
		bo = vma->bo;
		drm_gem_object_get(&bo->base.base);
		vma_va = mm_node->start << PAGE_SHIFT;
		*bo_offset = va - vma_va;
	}
	mutex_unlock(&vm->lock);

	return bo;
}

#define PANCSF_MAX_VMS_PER_FILE	 32

int pancsf_vm_pool_create_vm(struct pancsf_device *pfdev, struct pancsf_vm_pool *pool)
{
	struct pancsf_vm *vm;
	int ret;
	u32 id;

	vm = pancsf_vm_create(pfdev, false);
	if (IS_ERR(vm))
		return PTR_ERR(vm);

	mutex_lock(&pool->lock);
	ret = xa_alloc(&pool->xa, &id, vm,
		       XA_LIMIT(1, PANCSF_MAX_VMS_PER_FILE), GFP_KERNEL);
	mutex_unlock(&pool->lock);

	if (ret) {
		pancsf_vm_put(vm);
		return ret;
	}

	return id;
}

void pancsf_vm_pool_destroy_vm(struct pancsf_vm_pool *pool, u32 handle)
{
	struct pancsf_vm *vm;

	mutex_lock(&pool->lock);
	vm = xa_erase(&pool->xa, handle);
	mutex_unlock(&pool->lock);

	if (vm)
		pancsf_vm_put(vm);
}

struct pancsf_vm *pancsf_vm_pool_get_vm(struct pancsf_vm_pool *pool, u32 handle)
{
	struct pancsf_vm *vm;

	mutex_lock(&pool->lock);
	vm = xa_load(&pool->xa, handle);
	if (vm)
		pancsf_vm_get(vm);
	mutex_unlock(&pool->lock);

	return vm;
}

void pancsf_vm_pool_destroy(struct pancsf_file *pfile)
{
	struct pancsf_vm *vm;
	unsigned long i;

	if (!pfile->vms)
		return;

	mutex_lock(&pfile->vms->lock);
	xa_for_each(&pfile->vms->xa, i, vm)
		pancsf_vm_put(vm);
	mutex_unlock(&pfile->vms->lock);

	mutex_destroy(&pfile->vms->lock);
	xa_destroy(&pfile->vms->xa);
	kfree(pfile->vms);
}

int pancsf_vm_pool_create(struct pancsf_file *pfile)
{
	pfile->vms = kzalloc(sizeof(*pfile->vms), GFP_KERNEL);
	if (!pfile->vms)
		return -ENOMEM;

	xa_init_flags(&pfile->vms->xa, XA_FLAGS_ALLOC1);
	mutex_init(&pfile->vms->lock);
	return 0;
}

/* dummy TLB ops, the real TLB flush happens in pancsf_vm_flush_range() */
static void mmu_tlb_flush_all(void *cookie)
{
}

static void mmu_tlb_flush_walk(unsigned long iova, size_t size, size_t granule, void *cookie)
{
}

static const struct iommu_flush_ops mmu_tlb_ops = {
	.tlb_flush_all = mmu_tlb_flush_all,
	.tlb_flush_walk = mmu_tlb_flush_walk,
};

#define NUM_FAULT_PAGES (SZ_2M / PAGE_SIZE)

static int pancsf_mmu_map_fault_addr_locked(struct pancsf_device *pfdev, int as, u64 addr)
{
	struct pancsf_vm *vm;
	struct pancsf_vma *vma = NULL;
	struct drm_mm_node *mm_node;
	int ret;

	vm = pfdev->mmu->as.slots[as];
	if (!vm)
		return -ENOENT;

	if (!pancsf_vm_va_is_valid(vm, addr))
		return -EINVAL;

	mutex_lock(&vm->lock);
	drm_mm_for_each_node_in_range(mm_node, &vm->mm, addr >> PAGE_SHIFT,
				      (addr >> PAGE_SHIFT) + 1)
		vma = container_of(mm_node, struct pancsf_vma, vm_mm_node);

	if (!vma) {
		ret = -ENOENT;
		goto out;
	}

	if (!(vma->flags & PANCSF_VMA_MAP_ON_FAULT)) {
		dev_warn(pfdev->dev, "matching VMA is not MAP_ON_FAULT (GPU VA = %llx)",
			 vma->vm_mm_node.start << PAGE_SHIFT);
		ret = -EINVAL;
		goto out;
	}

	WARN_ON(vma->vm->as != as);

	ret = pancsf_vma_map_locked(vma, NULL);
	if (ret)
		goto out;

	dev_dbg(pfdev->dev, "mapped page fault @ AS%d %llx", as, addr);

out:
	mutex_unlock(&vm->lock);
	return ret;
}

static void pancsf_vm_release(struct kref *kref)
{
	struct pancsf_vm *vm = container_of(kref, struct pancsf_vm, refcount);
	struct pancsf_device *pfdev = vm->pfdev;

	mutex_lock(&pfdev->mmu->as.slots_lock);
	if (vm->as >= 0) {
		pm_runtime_get_noresume(pfdev->dev);
		if (pm_runtime_active(pfdev->dev))
			pancsf_vm_disable(vm);
		pm_runtime_put_autosuspend(pfdev->dev);

		pfdev->mmu->as.slots[vm->as] = NULL;
		clear_bit(vm->as, &pfdev->mmu->as.alloc_mask);
		clear_bit(vm->as, &pfdev->mmu->as.in_use_mask);
		list_del(&vm->node);
	}
	mutex_unlock(&pfdev->mmu->as.slots_lock);

	pancsf_vm_unmap_range(vm, vm->min_va, vm->max_va + 1 - vm->min_va);

	free_io_pgtable_ops(vm->pgtbl_ops);
	drm_mm_takedown(&vm->mm);
	mutex_destroy(&vm->lock);
	dma_resv_fini(&vm->resv);
	kfree(vm);
}

void pancsf_vm_put(struct pancsf_vm *vm)
{
	if (vm)
		kref_put(&vm->refcount, pancsf_vm_release);
}

struct pancsf_vm *pancsf_vm_get(struct pancsf_vm *vm)
{
	if (vm)
		kref_get(&vm->refcount);

	return vm;
}

#define PFN_4G		(SZ_4G >> PAGE_SHIFT)
#define PFN_4G_MASK	(PFN_4G - 1)
#define PFN_16M		(SZ_16M >> PAGE_SHIFT)

static void pancsf_drm_mm_color_adjust(const struct drm_mm_node *node,
				       unsigned long color,
				       u64 *start, u64 *end)
{
	if (color & MM_COLOR_FRAG_SHADER) {
		u64 next_seg;

		/* Reserve the last 16M of the 4GB block for blend shaders */
		next_seg = ALIGN(*start + 1, PFN_4G);
		if (next_seg - *start <= PFN_16M)
			*start = next_seg + 1;

		*end = min(*end, ALIGN(*start, PFN_4G) - PFN_16M);
	}
}

static u64 mair_to_memattr(u64 mair)
{
	u64 memattr = 0;
	u32 i;

	for (i = 0; i < 8; i++) {
		u8 in_attr = mair >> (8 * i), out_attr;
		u8 outer = in_attr >> 4, inner = in_attr & 0xf;

		/* For caching to be enabled, inner and outer caching policy
		 * have to be both write-back, if one of them is write-through
		 * or non-cacheable, we just choose non-cacheable. Device
		 * memory is also translated to non-cacheable.
		 */
		if (!(outer & 3) || !(outer & 4) || !(inner & 4)) {
			out_attr = AS_MEMATTR_AARCH64_INNER_OUTER_NC |
				   AS_MEMATTR_AARCH64_SH_MIDGARD_INNER |
				   AS_MEMATTR_AARCH64_INNER_ALLOC_EXPL(false, false);
		} else {
			/* Use SH_CPU_INNER mode so SH_IS, which is used when
			 * IOMMU_CACHE is set, actually maps to the standard
			 * definition of inner-shareable and not Mali's
			 * internal-shareable mode.
			 */
			out_attr = AS_MEMATTR_AARCH64_INNER_OUTER_WB |
				   AS_MEMATTR_AARCH64_SH_CPU_INNER |
				   AS_MEMATTR_AARCH64_INNER_ALLOC_EXPL(inner & 1, inner & 2);
		}

		memattr |= (u64)out_attr << (8 * i);
	}

	return memattr;
}

void pancsf_vm_unmap_mcu_pages(struct pancsf_vm *vm,
			       struct drm_mm_node *mm_node)
{
	struct io_pgtable_ops *ops = vm->pgtbl_ops;
	size_t len = mm_node->size << PAGE_SHIFT;
	u64 iova = mm_node->start << PAGE_SHIFT;
	size_t unmapped_len = 0;

	while (unmapped_len < len) {
		size_t unmapped_page, pgcount;
		size_t pgsize = get_pgsize(iova, len - unmapped_len, &pgcount);

		if (ops->iova_to_phys(ops, iova)) {
			unmapped_page = ops->unmap_pages(ops, iova, pgsize, pgcount, NULL);
			WARN_ON(unmapped_page != pgsize * pgcount);
		}
		iova += pgsize * pgcount;
		unmapped_len += pgsize * pgcount;
	}

	pancsf_vm_flush_range(vm, mm_node->start << PAGE_SHIFT, len);

	mutex_lock(&vm->lock);
	drm_mm_remove_node(mm_node);
	mutex_unlock(&vm->lock);
}

int pancsf_vm_remap_mcu_pages(struct pancsf_vm *vm,
			      struct drm_mm_node *mm_node,
			      struct sg_table *sgt,
			      int prot)
{
	if (WARN_ON(!drm_mm_node_allocated(mm_node)))
		return -EINVAL;

	pancsf_vm_map_pages(vm, mm_node->start << PAGE_SHIFT, prot, sgt, 0, -1);
	return 0;
}

int pancsf_vm_map_mcu_pages(struct pancsf_vm *vm,
			    struct drm_mm_node *mm_node,
			    struct sg_table *sgt,
			    unsigned int num_pages,
			    u64 va_start, u64 va_end,
			    int prot)
{
	int ret;

	if (WARN_ON(!vm->for_mcu))
		return -EINVAL;

	mutex_lock(&vm->lock);
	ret = drm_mm_insert_node_in_range(&vm->mm, mm_node,
					  num_pages, 0, 0,
					  va_start >> PAGE_SHIFT,
					  va_end >> PAGE_SHIFT,
					  DRM_MM_INSERT_BEST);
	mutex_unlock(&vm->lock);

	if (ret) {
		dev_err(vm->pfdev->dev, "Failed to reserve VA range %llx-%llx num_pages %d (err=%d)",
			va_start, va_end, num_pages, ret);
		return ret;
	}

	pancsf_vm_map_pages(vm, mm_node->start << PAGE_SHIFT, prot, sgt,
			    0, num_pages << PAGE_SHIFT);
	return 0;
}

struct pancsf_vm *pancsf_vm_create(struct pancsf_device *pfdev, bool for_mcu)
{
	u32 va_bits = GPU_MMU_FEATURES_VA_BITS(pfdev->gpu_info.mmu_features);
	u32 pa_bits = GPU_MMU_FEATURES_PA_BITS(pfdev->gpu_info.mmu_features);
	struct pancsf_vm *vm;

	vm = kzalloc(sizeof(*vm), GFP_KERNEL);
	if (!vm)
		return ERR_PTR(-ENOMEM);

	vm->for_mcu = for_mcu;
	vm->pfdev = pfdev;
	dma_resv_init(&vm->resv);
	mutex_init(&vm->lock);

	if (for_mcu) {
		/* CSF MCU is a cortex M7, and can only address 4G */
		vm->min_va = 0;
		vm->max_va = SZ_4G - 1;
	} else {
		vm->min_va = SZ_32M;
		vm->max_va = (1ull << va_bits) - SZ_32M - 1;
		vm->mm.color_adjust = pancsf_drm_mm_color_adjust;
	}

	drm_mm_init(&vm->mm, vm->min_va >> PAGE_SHIFT, (vm->max_va + 1) >> PAGE_SHIFT);

	INIT_LIST_HEAD(&vm->node);
	vm->as = -1;

	vm->pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= SZ_4K | SZ_2M,
		.ias		= va_bits,
		.oas		= pa_bits,
		.coherent_walk	= pfdev->coherent,
		.tlb		= &mmu_tlb_ops,
		.iommu_dev	= pfdev->dev,
	};

	vm->pgtbl_ops = alloc_io_pgtable_ops(ARM_64_LPAE_S1, &vm->pgtbl_cfg, vm);
	if (!vm->pgtbl_ops) {
		kfree(vm);
		return ERR_PTR(-EINVAL);
	}

	vm->memattr = mair_to_memattr(vm->pgtbl_cfg.arm_lpae_s1_cfg.mair);
	kref_init(&vm->refcount);

	return vm;
}

static const char *access_type_name(struct pancsf_device *pfdev,
				    u32 fault_status)
{
	switch (fault_status & AS_FAULTSTATUS_ACCESS_TYPE_MASK) {
	case AS_FAULTSTATUS_ACCESS_TYPE_ATOMIC:
		return "ATOMIC";
	case AS_FAULTSTATUS_ACCESS_TYPE_READ:
		return "READ";
	case AS_FAULTSTATUS_ACCESS_TYPE_WRITE:
		return "WRITE";
	case AS_FAULTSTATUS_ACCESS_TYPE_EX:
		return "EXECUTE";
	default:
		WARN_ON(1);
		return NULL;
	}
}

static irqreturn_t pancsf_mmu_irq_handler(int irq, void *data)
{
	struct pancsf_device *pfdev = data;

	if (!mmu_read(pfdev, MMU_INT_STAT))
		return IRQ_NONE;

	mmu_write(pfdev, MMU_INT_MASK, 0);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t pancsf_mmu_irq_handler_thread(int irq, void *data)
{
	struct pancsf_device *pfdev = data;
	u32 status = mmu_read(pfdev, MMU_INT_RAWSTAT);
	int ret;

	status = pancsf_mmu_fault_mask(pfdev, status);
	while (status) {
		u32 as = ffs(status | (status >> 16)) - 1;
		u32 mask = pancsf_mmu_as_fault_mask(pfdev, as);
		u64 addr;
		u32 fault_status;
		u32 exception_type;
		u32 access_type;
		u32 source_id;

		fault_status = mmu_read(pfdev, AS_FAULTSTATUS(as));
		addr = mmu_read(pfdev, AS_FAULTADDRESS_LO(as));
		addr |= (u64)mmu_read(pfdev, AS_FAULTADDRESS_HI(as)) << 32;

		/* decode the fault status */
		exception_type = fault_status & 0xFF;
		access_type = (fault_status >> 8) & 0x3;
		source_id = (fault_status >> 16);

		mmu_write(pfdev, MMU_INT_CLEAR, mask);

		/* Page fault only */
		ret = -1;
		mutex_lock(&pfdev->mmu->as.slots_lock);
		if ((status & mask) == BIT(as) && (exception_type & 0xF8) == 0xC0)
			ret = pancsf_mmu_map_fault_addr_locked(pfdev, as, addr);

		if (ret) {
			/* terminal fault, print info about the fault */
			dev_err(pfdev->dev,
				"Unhandled Page fault in AS%d at VA 0x%016llX\n"
				"Reason: %s\n"
				"raw fault status: 0x%X\n"
				"decoded fault status: %s\n"
				"exception type 0x%X: %s\n"
				"access type 0x%X: %s\n"
				"source id 0x%X\n",
				as, addr,
				"TODO",
				fault_status,
				(fault_status & (1 << 10) ? "DECODER FAULT" : "SLAVE FAULT"),
				exception_type, pancsf_exception_name(exception_type),
				access_type, access_type_name(pfdev, fault_status),
				source_id);

			/* Ignore MMU interrupts on this AS until it's been
			 * re-enabled.
			 */
			pfdev->mmu->as.faulty_mask |= mask;

			/* Disable the MMU to kill jobs on this AS. */
			pancsf_mmu_as_disable(pfdev, as);
		}

		mutex_unlock(&pfdev->mmu->as.slots_lock);

		status &= ~mask;

		/* If we received new MMU interrupts, process them before returning. */
		if (!status) {
			status = pancsf_mmu_fault_mask(pfdev, mmu_read(pfdev, MMU_INT_RAWSTAT));
			status &= ~pfdev->mmu->as.faulty_mask;
		}
	}

	mutex_lock(&pfdev->mmu->as.slots_lock);
	mmu_write(pfdev, MMU_INT_MASK, pancsf_mmu_fault_mask(pfdev, ~pfdev->mmu->as.faulty_mask));
	mutex_unlock(&pfdev->mmu->as.slots_lock);

	return IRQ_HANDLED;
};

int pancsf_mmu_init(struct pancsf_device *pfdev)
{
	struct pancsf_mmu *mmu;
	int ret, irq;

	mmu = kzalloc(sizeof(*mmu), GFP_KERNEL);
	if (!mmu)
		return -ENOMEM;

	INIT_LIST_HEAD(&mmu->as.lru_list);
	spin_lock_init(&mmu->as.op_lock);
	mutex_init(&mmu->as.slots_lock);

	pfdev->mmu = mmu;

	irq = platform_get_irq_byname(to_platform_device(pfdev->dev), "mmu");
	if (irq <= 0) {
		ret = -ENODEV;
		goto err_free_mmu;
	}

	mmu->irq = irq;

	mmu_write(pfdev, MMU_INT_CLEAR, pancsf_mmu_fault_mask(pfdev, ~0));
	mmu_write(pfdev, MMU_INT_MASK, pancsf_mmu_fault_mask(pfdev, ~0));
	ret = devm_request_threaded_irq(pfdev->dev, irq,
					pancsf_mmu_irq_handler,
					pancsf_mmu_irq_handler_thread,
					IRQF_SHARED, KBUILD_MODNAME "-mmu",
					pfdev);

	if (ret)
		goto err_free_mmu;

	return 0;

err_free_mmu:
	mutex_destroy(&mmu->as.slots_lock);
	kfree(mmu);
	return ret;
}

void pancsf_mmu_fini(struct pancsf_device *pfdev)
{
	mmu_write(pfdev, MMU_INT_MASK, 0);
	pfdev->mmu->as.faulty_mask = ~0;
	synchronize_irq(pfdev->mmu->irq);
	mutex_destroy(&pfdev->mmu->as.slots_lock);
	kfree(pfdev->mmu);
}
