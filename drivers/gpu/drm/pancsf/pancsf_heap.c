// SPDX-License-Identifier: GPL-2.0
/* Copyright 2023 Collabora ltd. */

#include <linux/iosys-map.h>
#include <linux/rwsem.h>

#include <drm/pancsf_drm.h>

#include "pancsf_device.h"
#include "pancsf_gem.h"
#include "pancsf_heap.h"
#include "pancsf_mmu.h"

struct pancsf_heap_gpu_ctx {
	u64 first_heap_chunk;
	u32 unused1[2];
	u32 vt_started_count;
	u32 vt_completed_count;
	u32 unused2;
	u32 frag_completed_count;
};

struct pancsf_heap_chunk_header {
	u64 next;
	u32 unknown[14];
};

struct pancsf_heap_chunk {
	struct list_head node;
	struct pancsf_gem_object *bo;
	u64 gpu_va;
};

struct pancsf_heap {
	struct list_head chunks;
	u32 chunk_size;
	u32 max_chunks;
	u32 target_in_flight;
	u32 chunk_count;
};

#define MAX_HEAPS_PER_POOL    128

struct pancsf_heap_pool {
	struct pancsf_device *pfdev;
	struct pancsf_vm *vm;
	struct rw_semaphore lock;
	struct xarray xa;
	struct pancsf_gem_object *bo;
	struct pancsf_heap_gpu_ctx *gpu_contexts;
	u64 gpu_va;
};

static void pancsf_free_heap_chunk(struct pancsf_vm *vm,
				   struct pancsf_heap_chunk *chunk)
{
	if (!chunk)
		return;

	list_del(&chunk->node);
	pancsf_gem_unmap_and_put(vm, chunk->bo, chunk->gpu_va, NULL);
	kfree(chunk);
}

static int pancsf_alloc_heap_chunk(struct pancsf_device *pfdev,
				   struct pancsf_vm *vm,
				   struct pancsf_heap *heap,
				   bool initial_chunk)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(NULL);
	struct pancsf_heap_chunk *chunk;
	struct pancsf_heap_chunk_header *hdr;
	int ret;

	chunk = kmalloc(sizeof(*chunk), GFP_KERNEL);
	if (!chunk)
		return -ENOMEM;

	chunk->bo = pancsf_gem_create_and_map(pfdev, vm, heap->chunk_size, 0,
					      PANCSF_VMA_MAP_NOEXEC |
					      PANCSF_VMA_MAP_AUTO_VA,
					      &chunk->gpu_va,
					      (void **)&hdr);
	if (IS_ERR(chunk->bo)) {
		ret = PTR_ERR(chunk->bo);
		goto err_free_chunk;
	}

	memset(hdr, 0, sizeof(*hdr));

	if (initial_chunk && !list_empty(&heap->chunks)) {
		struct pancsf_heap_chunk *prev_chunk;

		prev_chunk = list_first_entry(&heap->chunks,
					      struct pancsf_heap_chunk,
					      node);

		hdr->next = (prev_chunk->gpu_va & GENMASK_ULL(63, 12)) |
			    (heap->chunk_size >> 12);
	}

	map.vaddr = hdr;
	drm_gem_shmem_vunmap(&chunk->bo->base, &map);

	if (initial_chunk)
		list_add(&chunk->node, &heap->chunks);
	else
		list_add_tail(&chunk->node, &heap->chunks);
	heap->chunk_count++;

	return 0;

err_free_chunk:
	kfree(chunk);

	return ret;
}

static void pancsf_free_heap_chunks(struct pancsf_vm *vm,
				    struct pancsf_heap *heap)
{
	struct pancsf_heap_chunk *chunk, *tmp;

	list_for_each_entry_safe(chunk, tmp, &heap->chunks, node) {
		pancsf_free_heap_chunk(vm, chunk);
	}

	heap->chunk_count = 0;
}

static int pancsf_alloc_heap_chunks(struct pancsf_device *pfdev,
				    struct pancsf_vm *vm,
				    struct pancsf_heap *heap,
				    u32 chunk_count)
{
	int ret;
	u32 i;

	for (i = 0; i < chunk_count; i++) {
		ret = pancsf_alloc_heap_chunk(pfdev,
					      vm,
					      heap, true);
		if (ret)
			return ret;
	}

	return 0;
}

static int
pancsf_heap_destroy_locked(struct pancsf_heap_pool *pool, u32 handle)
{
	struct pancsf_heap *heap = NULL;

	heap = xa_erase(&pool->xa, handle);
	if (!heap)
		return -EINVAL;

	pancsf_free_heap_chunks(pool->vm, heap);
	kfree(heap);
	return 0;
}

int pancsf_heap_destroy(struct pancsf_heap_pool *pool, u32 handle)
{
	int ret;

	down_write(&pool->lock);
	ret = pancsf_heap_destroy_locked(pool, handle);
	up_write(&pool->lock);

	return ret;
}

int pancsf_heap_create(struct pancsf_heap_pool *pool,
		       u32 initial_chunk_count,
		       u32 chunk_size,
		       u32 max_chunks,
		       u32 target_in_flight,
		       u64 *heap_ctx_gpu_va,
		       u64 *first_chunk_gpu_va)
{
	struct pancsf_heap *heap;
	struct pancsf_heap_gpu_ctx *gpu_ctx;
	struct pancsf_heap_chunk *first_chunk;
	int ret = 0;
	u32 id;

	if (initial_chunk_count == 0)
		return -EINVAL;

	if (hweight32(chunk_size) != 1 ||
	    chunk_size < SZ_256K || chunk_size > SZ_2M)
		return -EINVAL;

	heap = kzalloc(sizeof(*heap), GFP_KERNEL);
	if (!heap)
		return -ENOMEM;

	INIT_LIST_HEAD(&heap->chunks);
	heap->chunk_size = chunk_size;
	heap->max_chunks = max_chunks;
	heap->target_in_flight = target_in_flight;

	down_write(&pool->lock);
	ret = xa_alloc(&pool->xa, &id, heap, XA_LIMIT(1, MAX_HEAPS_PER_POOL), GFP_KERNEL);
	if (ret) {
		kfree(heap);
		goto out_unlock;
	}

	gpu_ctx = &pool->gpu_contexts[id];
	memset(gpu_ctx, 0, sizeof(*gpu_ctx));

	ret = pancsf_alloc_heap_chunks(pool->pfdev, pool->vm, heap,
				       initial_chunk_count);
	if (ret) {
		pancsf_heap_destroy_locked(pool, id);
		goto out_unlock;
	}

	*heap_ctx_gpu_va = pool->gpu_va + (sizeof(*pool->gpu_contexts) * id);

	first_chunk = list_first_entry(&heap->chunks,
				       struct pancsf_heap_chunk,
				       node);
	*first_chunk_gpu_va = first_chunk->gpu_va;
	ret = id;

out_unlock:
	up_write(&pool->lock);
	return ret;
}

int pancsf_heap_grow(struct pancsf_heap_pool *pool,
		     u64 heap_gpu_va,
		     u32 renderpasses_in_flight,
		     u32 pending_frag_count,
		     u64 *new_chunk_gpu_va)
{
	u64 heap_id = (heap_gpu_va - pool->gpu_va) /
		      sizeof(struct pancsf_heap_gpu_ctx);
	struct pancsf_heap_chunk *chunk;
	struct pancsf_heap *heap;
	int ret;

	down_read(&pool->lock);
	heap = xa_load(&pool->xa, heap_id);
	if (!heap) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (renderpasses_in_flight > heap->target_in_flight ||
	    (pending_frag_count > 0 && heap->chunk_count >= heap->max_chunks)) {
		ret = -EBUSY;
		goto out_unlock;
	} else if (heap->chunk_count >= heap->max_chunks) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	ret = pancsf_alloc_heap_chunk(pool->pfdev, pool->vm, heap, false);
	if (ret)
		goto out_unlock;

	chunk = list_last_entry(&heap->chunks,
				struct pancsf_heap_chunk,
				node);
	*new_chunk_gpu_va = (chunk->gpu_va & GENMASK_ULL(63, 12)) |
			    (heap->chunk_size >> 12);
	ret = 0;

out_unlock:
	up_read(&pool->lock);
	return ret;
}

void pancsf_heap_pool_destroy(struct pancsf_heap_pool *pool)
{
	struct pancsf_heap *heap;
	unsigned long i;

	if (IS_ERR_OR_NULL(pool))
		return;

	down_write(&pool->lock);
	xa_for_each(&pool->xa, i, heap)
		WARN_ON(pancsf_heap_destroy_locked(pool, i));

	if (!IS_ERR_OR_NULL(pool->bo))
		pancsf_gem_unmap_and_put(pool->vm, pool->bo, pool->gpu_va, pool->gpu_contexts);
	up_write(&pool->lock);

	pancsf_vm_put(pool->vm);
	kfree(pool);
}

struct pancsf_heap_pool *
pancsf_heap_pool_create(struct pancsf_device *pfdev, struct pancsf_vm *vm)
{
	size_t bosize = ALIGN(MAX_HEAPS_PER_POOL *
			      sizeof(struct pancsf_heap_gpu_ctx),
			      4096);
	struct pancsf_heap_pool *pool;
	int ret = 0;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	pool->pfdev = pfdev;
	pool->vm = pancsf_vm_get(vm);
	init_rwsem(&pool->lock);
	xa_init_flags(&pool->xa, XA_FLAGS_ALLOC1);

	pool->bo = pancsf_gem_create_and_map(pfdev, vm, bosize, 0,
					     PANCSF_VMA_MAP_NOEXEC |
					     PANCSF_VMA_MAP_AUTO_VA,
					     &pool->gpu_va,
					     (void *)&pool->gpu_contexts);
	if (IS_ERR(pool->bo)) {
		ret = PTR_ERR(pool->bo);
		goto err_destroy_pool;
	}

	return pool;

err_destroy_pool:
	pancsf_heap_pool_destroy(pool);
	return ERR_PTR(ret);
}
