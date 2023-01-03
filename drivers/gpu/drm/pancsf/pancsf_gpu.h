/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2018 Marty E. Plummer <hanetzer@startmail.com> */
/* Copyright 2019 Collabora ltd. */

#ifndef __PANCSF_GPU_H__
#define __PANCSF_GPU_H__

struct pancsf_device;

int pancsf_gpu_init(struct pancsf_device *pfdev);
void pancsf_gpu_fini(struct pancsf_device *pfdev);

int pancsf_gpu_block_power_on(struct pancsf_device *pfdev,
			      const char *blk_name,
			      u32 pwron_reg, u32 pwrtrans_reg,
			      u32 rdy_reg, u64 mask, u32 timeout_us);
int pancsf_gpu_block_power_off(struct pancsf_device *pfdev,
			       const char *blk_name,
			       u32 pwroff_reg, u32 pwrtrans_reg,
			       u64 mask, u32 timeout_us);

#define pancsf_gpu_power_on(pfdev, type, mask, timeout_us) \
	pancsf_gpu_block_power_on(pfdev, #type, \
				    type ## _PWRON_LO, \
				    type ## _PWRTRANS_LO, \
				    type ## _READY_LO, \
				    mask, timeout_us)

#define pancsf_gpu_power_off(pfdev, type, mask, timeout_us) \
	pancsf_gpu_block_power_off(pfdev, #type, \
				     type ## _PWROFF_LO, \
				     type ## _PWRTRANS_LO, \
				     mask, timeout_us)

int pancsf_gpu_l2_power_on(struct pancsf_device *pfdev);
int pancsf_gpu_flush_caches(struct pancsf_device *pfdev,
			    u32 l2, u32 lsc, u32 other);
int pancsf_gpu_soft_reset(struct pancsf_device *pfdev);

#endif
