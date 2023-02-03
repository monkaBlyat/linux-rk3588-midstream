/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Collabora ltd. */

#ifndef __PANCSF_SCHED_H__
#define __PANCSF_SCHED_H__

#include <drm/pancsf_drm.h>

struct dma_fence;
struct drm_file;
struct drm_gem_object;
struct pancsf_device;
struct pancsf_file;
struct pancsf_job;

struct pancsf_group_pool;

int pancsf_create_group(struct pancsf_file *pfile,
			const struct drm_pancsf_group_create *group_args,
			const struct drm_pancsf_queue_create *queue_args);
void pancsf_destroy_group(struct pancsf_file *pfile,
			  u32 group_handle);
int pancsf_group_get_state(struct pancsf_file *pfile,
			   struct drm_pancsf_group_get_state *get_state);

struct pancsf_call_info {
	u64 start;
	u32 size;
	u32 latest_flush;
};

struct pancsf_job *
pancsf_create_job(struct pancsf_file *pfile,
		  u16 group_handle, u8 stream_idx,
		  const struct pancsf_call_info *cs_call);
int pancsf_set_job_bos(struct pancsf_file *pfile,
		       struct pancsf_job *job,
		       u32 bo_count, struct drm_gem_object **bos);
void pancsf_put_job(struct pancsf_job *job);
int pancsf_push_job(struct pancsf_job *job);
int pancsf_add_job_dep(struct pancsf_job *job,
		       struct dma_fence *fence);
struct dma_fence *pancsf_get_job_done_fence(struct pancsf_job *job);

int pancsf_group_pool_create(struct pancsf_file *pfile);
void pancsf_group_pool_destroy(struct pancsf_file *pfile);

void pancsf_sched_handle_job_irqs(struct pancsf_device *pfdev, u32 status);

int pancsf_sched_init(struct pancsf_device *pfdev);
void pancsf_sched_fini(struct pancsf_device *pfdev);
int pancsf_mcu_init(struct pancsf_device *pfdev);
void pancsf_mcu_fini(struct pancsf_device *pfdev);
int pancsf_destroy_tiler_heap(struct pancsf_file *pfile,
			      u32 handle);
int pancsf_create_tiler_heap(struct pancsf_file *pfile,
			     u32 initial_chunk_count,
			     u32 chunk_size,
			     u32 max_chunks,
			     u32 target_in_flight,
			     u64 *heap_ctx_gpu_va,
			     u64 *first_chunk_gpu_va);

#ifdef CONFIG_PM
void pancsf_sched_resume(struct pancsf_device *pfdev);
int pancsf_sched_suspend(struct pancsf_device *pfdev);
#endif

void pancsf_sched_queue_reset(struct pancsf_device *pfdev);

#endif
