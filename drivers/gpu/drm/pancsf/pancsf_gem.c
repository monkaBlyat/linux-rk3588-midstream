// SPDX-License-Identifier: GPL-2.0
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>

#include <drm/pancsf_drm.h>
#include "pancsf_device.h"
#include "pancsf_gem.h"
#include "pancsf_mmu.h"

/* Called DRM core on the last userspace/kernel unreference of the
 * BO.
 */
static void pancsf_gem_free_object(struct drm_gem_object *obj)
{
	struct pancsf_gem_object *bo = to_pancsf_bo(obj);

	pancsf_vm_put(bo->exclusive_vm);
	drm_gem_free_mmap_offset(&bo->base.base);
	drm_gem_shmem_free(&bo->base);
}

void pancsf_gem_unmap_and_put(struct pancsf_vm *vm, struct pancsf_gem_object *bo,
			      u64 gpu_va, void *cpu_va)
{
	if (cpu_va) {
		struct iosys_map map = IOSYS_MAP_INIT_VADDR(cpu_va);

		drm_gem_shmem_vunmap(&bo->base, &map);
	}

	WARN_ON(pancsf_vm_unmap_range(vm, gpu_va, bo->base.base.size));
	drm_gem_object_put(&bo->base.base);
}

struct pancsf_gem_object *
pancsf_gem_create_and_map(struct pancsf_device *pfdev, struct pancsf_vm *vm,
			  size_t size, u32 bo_flags, u32 vm_map_flags,
			  u64 *gpu_va, void **cpu_va)
{
	struct drm_gem_shmem_object *obj;
	struct pancsf_gem_object *bo;
	int ret;

	obj = drm_gem_shmem_create(pfdev->ddev, size);
	if (!obj)
		return ERR_PTR(-ENOMEM);

	bo = to_pancsf_bo(&obj->base);
	bo->exclusive_vm = pancsf_vm_get(vm);
	bo->base.base.resv = pancsf_vm_resv(vm);

	ret = pancsf_vm_map_bo_range(vm, bo, 0, obj->base.size, gpu_va, vm_map_flags);
	if (ret) {
		drm_gem_object_put(&obj->base);
		return ERR_PTR(ret);
	}

	if (cpu_va) {
		struct iosys_map map;
		int ret;

		ret = drm_gem_shmem_vmap(obj, &map);
		if (ret) {
			pancsf_vm_unmap_range(vm, *gpu_va, obj->base.size);
			drm_gem_object_put(&obj->base);
			return ERR_PTR(ret);
		}

		*cpu_va = map.vaddr;
	}

	return bo;
}

static int pancsf_gem_pin(struct drm_gem_object *obj)
{
	struct pancsf_gem_object *bo = to_pancsf_bo(obj);

	return drm_gem_shmem_pin(&bo->base);
}

static const struct drm_gem_object_funcs pancsf_gem_funcs = {
	.free = pancsf_gem_free_object,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = pancsf_gem_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

/**
 * pancsf_gem_create_object - Implementation of driver->gem_create_object.
 * @dev: DRM device
 * @size: Size in bytes of the memory the object will reference
 *
 * This lets the GEM helpers allocate object structs for us, and keep
 * our BO stats correct.
 */
struct drm_gem_object *pancsf_gem_create_object(struct drm_device *ddev, size_t size)
{
	struct pancsf_device *pfdev = ddev->dev_private;
	struct pancsf_gem_object *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return ERR_PTR(-ENOMEM);

	obj->base.base.funcs = &pancsf_gem_funcs;
	obj->base.map_wc = !pfdev->coherent;

	return &obj->base.base;
}

struct pancsf_gem_object *
pancsf_gem_create_with_handle(struct drm_file *file,
			      struct drm_device *ddev,
			      struct pancsf_vm *exclusive_vm,
			      size_t size,
			      u32 flags, u32 *handle)
{
	int ret;
	struct drm_gem_shmem_object *shmem;
	struct pancsf_gem_object *bo;

	shmem = drm_gem_shmem_create(ddev, size);
	if (IS_ERR(shmem))
		return ERR_CAST(shmem);

	bo = to_pancsf_bo(&shmem->base);

	if (exclusive_vm) {
		bo->exclusive_vm = pancsf_vm_get(exclusive_vm);
		bo->base.base.resv = pancsf_vm_resv(exclusive_vm);
	}

	/*
	 * Allocate an id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file, &shmem->base, handle);
	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_put(&shmem->base);
	if (ret)
		return ERR_PTR(ret);

	return bo;
}

struct drm_gem_object *
pancsf_gem_prime_import_sg_table(struct drm_device *ddev,
				 struct dma_buf_attachment *attach,
				 struct sg_table *sgt)
{
	struct drm_gem_object *obj;
	struct pancsf_gem_object *bo;

	obj = drm_gem_shmem_prime_import_sg_table(ddev, attach, sgt);
	if (IS_ERR(obj))
		return ERR_CAST(obj);

	bo = to_pancsf_bo(obj);
	return obj;
}

int pancsf_gem_prime_handle_to_fd(struct drm_device *dev,
				  struct drm_file *file_priv,
				  uint32_t handle, uint32_t flags,
				  int *prime_fd)
{
	struct drm_gem_object *obj;

	obj = drm_gem_object_lookup(file_priv, handle);
	if (!obj)
		return -ENOENT;

	/* Can't export private BOs. */
	if (to_pancsf_bo(obj)->exclusive_vm)
		return -EINVAL;

	return drm_gem_prime_handle_to_fd(dev, file_priv, handle, flags, prime_fd);
}
