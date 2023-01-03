/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 Collabora ltd. */

#ifndef __PANCSF_MCU_H__
#define __PANCSF_MCU_H__

#include <linux/types.h>

#include "pancsf_device.h"

struct pancsf_fw_mem;

#define MAX_CSGS				31
#define MAX_CS_PER_CSG                          32

struct pancsf_ringbuf_input_iface {
	u64 insert;
	u64 extract;
} __packed;

struct pancsf_ringbuf_output_iface {
	u64 extract;
	u32 active;
} __packed;

struct pancsf_cs_control_iface {
#define CS_FEATURES_WORK_REGS(x)		(((x) & GENMASK(7, 0)) + 1)
#define CS_FEATURES_SCOREBOARDS(x)		(((x) & GENMASK(15, 8)) >> 8)
#define CS_FEATURES_COMPUTE			BIT(16)
#define CS_FEATURES_FRAGMENT			BIT(17)
#define CS_FEATURES_TILER			BIT(18)
	u32 features;
	u32 input_va;
	u32 output_va;
} __packed;

struct pancsf_cs_input_iface {
#define CS_STATE_MASK				GENMASK(2, 0)
#define CS_STATE_STOP				0
#define CS_STATE_START				1
#define CS_EXTRACT_EVENT			BIT(4)
#define CS_IDLE_SYNC_WAIT			BIT(8)
#define CS_IDLE_PROTM_PENDING			BIT(9)
#define CS_IDLE_EMPTY				BIT(10)
#define CS_IDLE_RESOURCE_REQ			BIT(11)
#define CS_TILER_OOM				BIT(26)
#define CS_PROTM_PENDING			BIT(27)
#define CS_FATAL				BIT(30)
#define CS_FAULT				BIT(31)

	u32 req;

#define CS_CONFIG_PRIORITY(x)			((x) & GENMASK(3, 0))
#define CS_CONFIG_DOORBELL(x)			(((x) << 8) & GENMASK(15, 8))
	u32 config;
	u32 reserved1;
	u32 ack_irq_mask;
	u64 ringbuf_base;
	u32 ringbuf_size;
	u32 reserved2;
	u64 heap_start;
	u64 heap_end;
	u64 ringbuf_input;
	u64 ringbuf_output;
	u32 instr_config;
	u32 instrbuf_size;
	u64 instrbuf_base;
	u64 instrbuf_offset_ptr;
} __packed;

struct pancsf_cs_output_iface {
	u32 ack;
	u32 reserved1[15];
	u64 status_cmd_ptr;

#define CS_STATUS_WAIT_SB_MASK			GENMASK(15, 0)
#define CS_STATUS_WAIT_SB_SRC_MASK		GENMASK(19, 16)
#define CS_STATUS_WAIT_SB_SRC_NONE		(0 << 16)
#define CS_STATUS_WAIT_SB_SRC_WAIT		(8 << 16)
#define CS_STATUS_WAIT_SYNC_COND_LE		(0 << 24)
#define CS_STATUS_WAIT_SYNC_COND_GT		(1 << 24)
#define CS_STATUS_WAIT_SYNC_COND_MASK		GENMASK(27, 24)
#define CS_STATUS_WAIT_PROGRESS			BIT(28)
#define CS_STATUS_WAIT_PROTM			BIT(29)
#define CS_STATUS_WAIT_SYNC_64B			BIT(30)
#define CS_STATUS_WAIT_SYNC			BIT(31)
	u32 status_wait;
	u32 status_req_resource;
	u64 status_wait_sync_ptr;
	u32 status_wait_sync_value;
	u32 status_scoreboards;

#define CS_STATUS_BLOCKED_REASON_UNBLOCKED	0
#define CS_STATUS_BLOCKED_REASON_SB_WAIT	1
#define CS_STATUS_BLOCKED_REASON_PROGRESS_WAIT	2
#define CS_STATUS_BLOCKED_REASON_SYNC_WAIT	3
#define CS_STATUS_BLOCKED_REASON_DEFERRED	5
#define CS_STATUS_BLOCKED_REASON_RES		6
#define CS_STATUS_BLOCKED_REASON_FLUSH		7
#define CS_STATUS_BLOCKED_REASON_MASK		GENMASK(3, 0)
	u32 status_blocked_reason;
	u32 status_wait_sync_value_hi;
	u32 reserved2[6];

#define CS_EXCEPTION_TYPE(x)			((x) & GENMASK(7, 0))
#define CS_EXCEPTION_DATA(x)			(((x) >> 8) & GENMASK(23, 0))
	u32 fault;
	u32 fatal;
	u64 fault_info;
	u64 fatal_info;
	u32 reserved3[10];
	u32 heap_vt_start;
	u32 heap_vt_end;
	u32 reserved4;
	u32 heap_frag_end;
	u64 heap_address;
} __packed;

struct pancsf_csg_control_iface {
	u32 features;
	u32 input_va;
	u32 output_va;
	u32 suspend_size;
	u32 protm_suspend_size;
	u32 stream_num;
	u32 stream_stride;
} __packed;

struct pancsf_csg_input_iface {
#define CSG_STATE_MASK				GENMASK(2, 0)
#define CSG_STATE_TERMINATE			0
#define CSG_STATE_START				1
#define CSG_STATE_SUSPEND			2
#define CSG_STATE_RESUME			3
#define CSG_ENDPOINT_CONFIG			BIT(4)
#define CSG_STATUS_UPDATE			BIT(5)
#define CSG_SYNC_UPDATE				BIT(28)
#define CSG_IDLE				BIT(29)
#define CSG_DOORBELL				BIT(30)
#define CSG_PROGRESS_TIMER_EVENT		BIT(31)
	u32 req;
	u32 ack_irq_mask;

	u32 doorbell_req;
	u32 irq_ack;
	u32 reserved1[4];
	u64 allow_compute;
	u64 allow_fragment;
	u32 allow_other;

#define CSG_EP_REQ_COMPUTE(x)			((x) & GENMASK(7, 0))
#define CSG_EP_REQ_FRAGMENT(x)			(((x) << 8) & GENMASK(15, 8))
#define CSG_EP_REQ_TILER(x)			(((x) << 16) & GENMASK(19, 16))
#define CSG_EP_REQ_EXCL_COMPUTE			BIT(20)
#define CSG_EP_REQ_EXCL_FRAGMENT		BIT(21)
#define CSG_EP_REQ_PRIORITY(x)			(((x) << 28) & GENMASK(31, 28))
#define CSG_EP_REQ_PRIORITY_MASK		GENMASK(31, 28)
	u32 endpoint_req;
	u32 reserved2[2];
	u64 suspend_buf;
	u64 protm_suspend_buf;
	u32 config;
	u32 iter_trace_config;
} __packed;

struct pancsf_csg_output_iface {
	u32 ack;
	u32 reserved1;
	u32 doorbell_ack;
	u32 irq_req;
	u32 status_endpoint_current;
	u32 status_endpoint_req;

#define CSG_STATUS_STATE_IS_IDLE		BIT(0)
	u32 status_state;
	u32 resource_dep;
} __packed;

struct pancsf_global_control_iface {
	u32 version;
	u32 features;
	u32 input_va;
	u32 output_va;
	u32 group_num;
	u32 group_stride;
	u32 perfcnt_size;
	u32 instr_features;
} __packed;

struct pancsf_global_input_iface {
#define GLB_HALT				BIT(0)
#define GLB_CFG_PROGRESS_TIMER			BIT(1)
#define GLB_CFG_ALLOC_EN			BIT(2)
#define GLB_CFG_POWEROFF_TIMER			BIT(3)
#define GLB_PROTM_ENTER				BIT(4)
#define GLB_PERFCNT_EN				BIT(5)
#define GLB_PERFCNT_SAMPLER			BIT(6)
#define GLB_COUNTER_EN				BIT(7)
#define GLB_PING				BIT(8)
#define GLB_FWCFG_UPDATE			BIT(9)
#define GLB_IDLE_EN				BIT(10)
#define GLB_SLEEP				BIT(12)
#define GLB_INACTIVE_COMPUTE			BIT(20)
#define GLB_INACTIVE_FRAGMENT			BIT(21)
#define GLB_INACTIVE_TILER			BIT(22)
#define GLB_PROTM_EXIT				BIT(23)
#define GLB_PERFCNT_THRESHOLD			BIT(24)
#define GLB_PERFCNT_OVERFLOW			BIT(25)
#define GLB_IDLE				BIT(26)
#define GLB_DBG_CSF				BIT(30)
#define GLB_DBG_HOST				BIT(31)
	u32 req;
	u32 ack_irq_mask;
	u32 doorbell_req;
	u32 reserved1;
	u32 progress_timer;

#define GLB_TIMER_VAL(x)			((x) & GENMASK(30, 0))
#define GLB_TIMER_SOURCE_GPU_COUNTER		BIT(31)
	u32 poweroff_timer;
	u64 core_en_mask;
	u32 reserved2;
	u32 perfcnt_as;
	u64 perfcnt_base;
	u32 perfcnt_extract;
	u32 reserved3[3];
	u32 perfcnt_config;
	u32 perfcnt_csg_select;
	u32 perfcnt_fw_enable;
	u32 perfcnt_csg_enable;
	u32 perfcnt_csf_enable;
	u32 perfcnt_shader_enable;
	u32 perfcnt_tiler_enable;
	u32 perfcnt_mmu_l2_enable;
	u32 reserved4[8];
	u32 idle_timer;
} __packed;

struct pancsf_global_output_iface {
	u32 ack;
	u32 reserved1;
	u32 doorbell_ack;
	u32 reserved2;
	u32 halt_status;
	u32 perfcnt_status;
	u32 perfcnt_insert;
} __packed;

static inline u32 pancsf_toggle_reqs(u32 cur_req_val, u32 ack_val, u32 req_mask)
{
	return ((ack_val ^ req_mask) & req_mask) | (cur_req_val & ~req_mask);
}

static inline u32 pancsf_update_reqs(u32 cur_req_val, u32 new_reqs, u32 req_mask)
{
	return (cur_req_val & ~req_mask) | (new_reqs & req_mask);
}


struct pancsf_fw_cs_iface {
	struct pancsf_cs_control_iface *control;
	struct pancsf_cs_input_iface *input;
	const struct pancsf_cs_output_iface *output;
};

struct pancsf_fw_csg_iface {
	const struct pancsf_csg_control_iface *control;
	struct pancsf_csg_input_iface *input;
	const struct pancsf_csg_output_iface *output;
	struct pancsf_fw_cs_iface streams[MAX_CS_PER_CSG];
};

struct pancsf_fw_global_iface {
	const struct pancsf_global_control_iface *control;
	struct pancsf_global_input_iface *input;
	const struct pancsf_global_output_iface *output;
};

struct pancsf_fw_iface {
	struct pancsf_fw_global_iface global;
	struct pancsf_fw_csg_iface groups[MAX_CSGS];
};

static inline struct pancsf_fw_global_iface *
pancsf_get_glb_iface(struct pancsf_device *pfdev)
{
	return &pfdev->iface->global;
}

static inline struct pancsf_fw_csg_iface *
pancsf_get_csg_iface(struct pancsf_device *pfdev, u32 csg_slot)
{
	return &pfdev->iface->groups[csg_slot];
}

static inline struct pancsf_fw_cs_iface *
pancsf_get_cs_iface(struct pancsf_device *pfdev, u32 csg_slot, u32 cs_slot)
{
	return &pfdev->iface->groups[csg_slot].streams[cs_slot];
}

void pancsf_fw_mem_vunmap(struct pancsf_fw_mem *mem);
void *pancsf_fw_mem_vmap(struct pancsf_fw_mem *mem, pgprot_t prot);
u64 pancsf_fw_mem_va(struct pancsf_fw_mem *mem);
void pancsf_fw_mem_free(struct pancsf_device *pfdev, struct pancsf_fw_mem *mem);
struct pancsf_fw_mem *pancsf_fw_alloc_queue_iface_mem(struct pancsf_device *pfdev);
struct pancsf_fw_mem *
pancsf_fw_alloc_suspend_buf_mem(struct pancsf_device *pfdev, size_t size);

void pancsf_mcu_pre_reset(struct pancsf_device *pfdev);
int pancsf_mcu_reset(struct pancsf_device *pfdev, bool full_fw_reload);

#endif
