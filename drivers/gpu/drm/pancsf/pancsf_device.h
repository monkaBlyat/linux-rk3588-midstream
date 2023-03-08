/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2018 Marty E. Plummer <hanetzer@startmail.com> */
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */

#ifndef __PANCSF_DEVICE_H__
#define __PANCSF_DEVICE_H__

#include <linux/atomic.h>
#include <linux/io-pgtable.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <drm/drm_device.h>
#include <drm/drm_mm.h>
#include <drm/gpu_scheduler.h>
#include <drm/pancsf_drm.h>

struct pancsf_csf;
struct pancsf_csf_ctx;
struct pancsf_device;
struct pancsf_fw_iface;
struct pancsf_gpu;
struct pancsf_group_pool;
struct pancsf_heap_pool;
struct pancsf_job;
struct pancsf_mmu;
struct pancsf_mcu;
struct pancsf_perfcnt;
struct pancsf_vm;
struct pancsf_vm_pool;
struct pancsf_vm_bind_queue_pool;

#define MAX_PM_DOMAINS 3

/*
 * Features that cannot be automatically detected and need matching using the
 * compatible string, typically SoC-specific.
 */
struct pancsf_compatible {
	/* Supplies count and names. */
	int num_supplies;
	const char * const *supply_names;
	/*
	 * Number of power domains required, note that values 0 and 1 are
	 * handled identically, as only values > 1 need special handling.
	 */
	int num_pm_domains;
	/* Only required if num_pm_domains > 1. */
	const char * const *pm_domain_names;

	/* Clocks count and names. */
	int num_clks;
	const char * const *clk_names;

	/* Vendor implementation quirks callback */
	void (*vendor_quirk)(struct pancsf_device *pfdev);
};

struct pancsf_device {
	struct device *dev;
	struct drm_device *ddev;
	struct platform_device *pdev;

	phys_addr_t phys_addr;
	void __iomem *iomem;
	struct clk *clock;
	struct clk *bus_clock;
	struct clk_bulk_data *platform_clocks;
	struct regulator_bulk_data *regulators;
	struct reset_control *rstc;
	/* pm_domains for devices with more than one. */
	struct device *pm_domain_devs[MAX_PM_DOMAINS];
	struct device_link *pm_domain_links[MAX_PM_DOMAINS];
	bool coherent;

	struct drm_pancsf_gpu_info gpu_info;
	struct drm_pancsf_csif_info csif_info;

	const struct pancsf_compatible *comp;

	struct pancsf_gpu *gpu;
	struct pancsf_mcu *mcu;
	struct pancsf_mmu *mmu;
	struct pancsf_fw_iface *iface;
	struct pancsf_scheduler *scheduler;
};

struct pancsf_file {
	struct pancsf_device *pfdev;
	struct pancsf_vm_pool *vms;
	struct pancsf_vm_bind_queue_pool *vm_bind_queues;

	struct mutex heaps_lock;
	struct pancsf_heap_pool *heaps;
	struct pancsf_group_pool *groups;
};

static inline struct pancsf_device *to_pancsf_device(struct drm_device *ddev)
{
	return ddev->dev_private;
}

int pancsf_device_init(struct pancsf_device *pfdev);
void pancsf_device_fini(struct pancsf_device *pfdev);

int pancsf_device_resume(struct device *dev);
int pancsf_device_suspend(struct device *dev);

enum drm_pancsf_exception_type {
	DRM_PANCSF_EXCEPTION_OK = 0x00,
	DRM_PANCSF_EXCEPTION_TERMINATED = 0x04,
	DRM_PANCSF_EXCEPTION_KABOOM = 0x05,
	DRM_PANCSF_EXCEPTION_EUREKA = 0x06,
	DRM_PANCSF_EXCEPTION_ACTIVE = 0x08,
	DRM_PANCSF_EXCEPTION_CS_RES_TERM = 0x0f,
	DRM_PANCSF_EXCEPTION_MAX_NON_FAULT = 0x3f,
	DRM_PANCSF_EXCEPTION_CS_CONFIG_FAULT = 0x40,
	DRM_PANCSF_EXCEPTION_CS_ENDPOINT_FAULT = 0x44,
	DRM_PANCSF_EXCEPTION_CS_BUS_FAULT = 0x48,
	DRM_PANCSF_EXCEPTION_CS_INSTR_INVALID = 0x49,
	DRM_PANCSF_EXCEPTION_CS_CALL_STACK_OVERFLOW = 0x4a,
	DRM_PANCSF_EXCEPTION_CS_INHERIT_FAULT = 0x4b,
	DRM_PANCSF_EXCEPTION_INSTR_INVALID_PC = 0x50,
	DRM_PANCSF_EXCEPTION_INSTR_INVALID_ENC = 0x51,
	DRM_PANCSF_EXCEPTION_INSTR_BARRIER_FAULT = 0x55,
	DRM_PANCSF_EXCEPTION_DATA_INVALID_FAULT = 0x58,
	DRM_PANCSF_EXCEPTION_TILE_RANGE_FAULT = 0x59,
	DRM_PANCSF_EXCEPTION_ADDR_RANGE_FAULT = 0x5a,
	DRM_PANCSF_EXCEPTION_IMPRECISE_FAULT = 0x5b,
	DRM_PANCSF_EXCEPTION_OOM = 0x60,
	DRM_PANCSF_EXCEPTION_CSF_FW_INTERNAL_ERROR = 0x68,
	DRM_PANCSF_EXCEPTION_CSF_RES_EVICTION_TIMEOUT = 0x69,
	DRM_PANCSF_EXCEPTION_GPU_BUS_FAULT = 0x80,
	DRM_PANCSF_EXCEPTION_GPU_SHAREABILITY_FAULT = 0x88,
	DRM_PANCSF_EXCEPTION_SYS_SHAREABILITY_FAULT = 0x89,
	DRM_PANCSF_EXCEPTION_GPU_CACHEABILITY_FAULT = 0x8a,
	DRM_PANCSF_EXCEPTION_TRANSLATION_FAULT_0 = 0xc0,
	DRM_PANCSF_EXCEPTION_TRANSLATION_FAULT_1 = 0xc1,
	DRM_PANCSF_EXCEPTION_TRANSLATION_FAULT_2 = 0xc2,
	DRM_PANCSF_EXCEPTION_TRANSLATION_FAULT_3 = 0xc3,
	DRM_PANCSF_EXCEPTION_TRANSLATION_FAULT_4 = 0xc4,
	DRM_PANCSF_EXCEPTION_PERM_FAULT_0 = 0xc8,
	DRM_PANCSF_EXCEPTION_PERM_FAULT_1 = 0xc9,
	DRM_PANCSF_EXCEPTION_PERM_FAULT_2 = 0xca,
	DRM_PANCSF_EXCEPTION_PERM_FAULT_3 = 0xcb,
	DRM_PANCSF_EXCEPTION_ACCESS_FLAG_1 = 0xd9,
	DRM_PANCSF_EXCEPTION_ACCESS_FLAG_2 = 0xda,
	DRM_PANCSF_EXCEPTION_ACCESS_FLAG_3 = 0xdb,
	DRM_PANCSF_EXCEPTION_ADDR_SIZE_FAULT_IN = 0xe0,
	DRM_PANCSF_EXCEPTION_ADDR_SIZE_FAULT_OUT0 = 0xe4,
	DRM_PANCSF_EXCEPTION_ADDR_SIZE_FAULT_OUT1 = 0xe5,
	DRM_PANCSF_EXCEPTION_ADDR_SIZE_FAULT_OUT2 = 0xe6,
	DRM_PANCSF_EXCEPTION_ADDR_SIZE_FAULT_OUT3 = 0xe7,
	DRM_PANCSF_EXCEPTION_MEM_ATTR_FAULT_0 = 0xe8,
	DRM_PANCSF_EXCEPTION_MEM_ATTR_FAULT_1 = 0xe9,
	DRM_PANCSF_EXCEPTION_MEM_ATTR_FAULT_2 = 0xea,
	DRM_PANCSF_EXCEPTION_MEM_ATTR_FAULT_3 = 0xeb,
};

static inline bool
pancsf_exception_is_fault(u32 exception_code)
{
	return exception_code > DRM_PANCSF_EXCEPTION_MAX_NON_FAULT;
}

const char *pancsf_exception_name(u32 exception_code);

#endif
