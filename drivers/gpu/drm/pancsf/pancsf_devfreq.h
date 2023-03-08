/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2019 Collabora ltd. */

#ifndef __PANCSF_DEVFREQ_H__
#define __PANCSF_DEVFREQ_H__

#include <linux/devfreq.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>

struct devfreq;
struct thermal_cooling_device;

struct pancsf_device;

struct pancsf_devfreq {
	struct devfreq *devfreq;
	struct thermal_cooling_device *cooling;
	struct devfreq_simple_ondemand_data gov_data;
	bool opp_of_table_added;

	ktime_t busy_time;
	ktime_t idle_time;
	ktime_t time_last_update;
	int busy_count;
	/*
	 * Protect busy_time, idle_time, time_last_update and busy_count
	 * because these can be updated concurrently between multiple jobs.
	 */
	spinlock_t lock;
};

int pancsf_devfreq_init(struct pancsf_device *pfdev);
void pancsf_devfreq_fini(struct pancsf_device *pfdev);

void pancsf_devfreq_resume(struct pancsf_device *pfdev);
void pancsf_devfreq_suspend(struct pancsf_device *pfdev);

void pancsf_devfreq_record_busy(struct pancsf_devfreq *devfreq);
void pancsf_devfreq_record_idle(struct pancsf_devfreq *devfreq);

#endif /* __PANCSF_DEVFREQ_H__ */
