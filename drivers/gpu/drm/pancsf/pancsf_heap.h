/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Collabora ltd. */

#ifndef __PANCSF_HEAP_H__
#define __PANCSF_HEAP_H__

#include <linux/types.h>

struct pancsf_device;
struct pancsf_heap_pool;
struct pancsf_vm;

int pancsf_heap_create(struct pancsf_heap_pool *pool,
		       u32 initial_chunk_count,
		       u32 chunk_size,
		       u32 max_chunks,
		       u32 target_in_flight,
		       u64 *heap_ctx_gpu_va,
		       u64 *first_chunk_gpu_va);
int pancsf_heap_destroy(struct pancsf_heap_pool *pool, u32 handle);
struct pancsf_heap_pool *
pancsf_heap_pool_create(struct pancsf_device *pfdev, struct pancsf_vm *vm);
void pancsf_heap_pool_destroy(struct pancsf_heap_pool *pool);
int pancsf_heap_grow(struct pancsf_heap_pool *pool,
		     u64 heap_gpu_va,
		     u32 renderpasses_in_flight,
		     u32 pending_frag_count,
		     u64 *new_chunk_gpu_va);

#endif
