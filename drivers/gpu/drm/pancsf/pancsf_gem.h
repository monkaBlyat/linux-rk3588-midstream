/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */

#ifndef __PANCSF_GEM_H__
#define __PANCSF_GEM_H__

#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_mm.h>

#include <linux/rwsem.h>

struct pancsf_vm;

struct pancsf_gem_object {
	struct drm_gem_shmem_object base;
};

static inline
struct pancsf_gem_object *to_pancsf_bo(struct drm_gem_object *obj)
{
	return container_of(to_drm_gem_shmem_obj(obj), struct pancsf_gem_object, base);
}

struct drm_gem_object *pancsf_gem_create_object(struct drm_device *ddev, size_t size);

struct drm_gem_object *
pancsf_gem_prime_import_sg_table(struct drm_device *ddev,
				 struct dma_buf_attachment *attach,
				 struct sg_table *sgt);

struct pancsf_gem_object *
pancsf_gem_create_with_handle(struct drm_file *file,
			      struct drm_device *ddev, size_t size,
			      u32 flags,
			      uint32_t *handle);

void pancsf_gem_unmap_and_put(struct pancsf_vm *vm, struct pancsf_gem_object *bo,
			      u64 gpu_va, void *cpu_va);
struct pancsf_gem_object *
pancsf_gem_create_and_map(struct pancsf_device *pfdev, struct pancsf_vm *vm,
			  size_t size, u32 bo_flags, u32 vm_map_flags,
			  u64 *gpu_va, void **cpu_va);

#endif /* __PANCSF_GEM_H__ */
