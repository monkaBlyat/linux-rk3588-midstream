/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 */

#ifndef _ROCKCHIP_DRM_VOP2_H
#define _ROCKCHIP_DRM_VOP2_H

#include "rockchip_drm_vop.h"

#include <linux/regmap.h>
#include <drm/drm_modes.h>

#define VOP_VERSION_RK3568	VOP_VERSION(0x40, 0x15)
#define VOP_VERSION_RK3588	VOP_VERSION(0x40, 0x17)

#define ROCKCHIP_OUTPUT_DUAL_CHANNEL_LEFT_RIGHT_MODE	BIT(0)
#define ROCKCHIP_OUTPUT_DUAL_CHANNEL_ODD_EVEN_MODE	BIT(1)
#define ROCKCHIP_OUTPUT_DATA_SWAP			BIT(2)
/* MIPI DSI DataStream(cmd) mode on rk3588 */
#define ROCKCHIP_OUTPUT_MIPI_DS_MODE			BIT(3)

#define AFBDC_FMT_RGB565	0x0
#define AFBDC_FMT_U8U8U8U8	0x5
#define AFBDC_FMT_U8U8U8	0x4

#define VOP_FEATURE_OUTPUT_RGB10	BIT(0)
#define VOP_FEATURE_INTERNAL_RGB	BIT(1)
#define VOP_FEATURE_ALPHA_SCALE		BIT(2)
#define VOP_FEATURE_HDR10		BIT(3)
#define VOP_FEATURE_NEXT_HDR		BIT(4)
/* a feature to splice two windows and two vps to support resolution > 4096 */
#define VOP_FEATURE_SPLICE		BIT(5)
#define VOP_FEATURE_OVERSCAN		BIT(6)

#define VOP_FEATURE_OUTPUT_10BIT	VOP_FEATURE_OUTPUT_RGB10


#define WIN_FEATURE_HDR2SDR		BIT(0)
#define WIN_FEATURE_SDR2HDR		BIT(1)
#define WIN_FEATURE_PRE_OVERLAY		BIT(2)
#define WIN_FEATURE_AFBDC		BIT(3)
#define WIN_FEATURE_CLUSTER_MAIN	BIT(4)
#define WIN_FEATURE_CLUSTER_SUB		BIT(5)
/* Left win in splice mode */
#define WIN_FEATURE_SPLICE_LEFT		BIT(6)
/* a mirror win can only get fb address
 * from source win:
 * Cluster1---->Cluster0
 * Esmart1 ---->Esmart0
 * Smart1  ---->Smart0
 * This is a feature on rk3566
 */
#define WIN_FEATURE_MIRROR		BIT(6)
#define WIN_FEATURE_MULTI_AREA		BIT(7)


#define VOP2_SOC_VARIANT		4

#define ROCKCHIP_DSC_PPS_SIZE_BYTE	88

enum vop_win_phy_id {
	ROCKCHIP_VOP_WIN0 = 0,
	ROCKCHIP_VOP_WIN1,
	ROCKCHIP_VOP_WIN2,
	ROCKCHIP_VOP_WIN3,
	ROCKCHIP_VOP_PHY_ID_INVALID = -1,
};

enum bcsh_out_mode {
	BCSH_OUT_MODE_BLACK,
	BCSH_OUT_MODE_BLUE,
	BCSH_OUT_MODE_COLOR_BAR,
	BCSH_OUT_MODE_NORMAL_VIDEO,
};

enum cabc_stage_mode {
	LAST_FRAME_PWM_VAL	= 0x0,
	CUR_FRAME_PWM_VAL	= 0x1,
	STAGE_BY_STAGE		= 0x2
};

enum cabc_stage_up_mode {
	MUL_MODE,
	ADD_MODE,
};


/*
 *  the delay number of a window in different mode.
 */
enum win_dly_mode {
	VOP2_DLY_MODE_DEFAULT,   /**< default mode */
	VOP2_DLY_MODE_HISO_S,    /** HDR in SDR out mode, as a SDR window */
	VOP2_DLY_MODE_HIHO_H,    /** HDR in HDR out mode, as a HDR window */
	VOP2_DLY_MODE_MAX,
};

/*
 * vop2 dsc id
 */
#define ROCKCHIP_VOP2_DSC_8K	0
#define ROCKCHIP_VOP2_DSC_4K	1

/*
 * vop2 internal power domain id,
 * should be all none zero, 0 will be
 * treat as invalid;
 */
#define VOP2_PD_CLUSTER0	BIT(0)
#define VOP2_PD_CLUSTER1	BIT(1)
#define VOP2_PD_CLUSTER2	BIT(2)
#define VOP2_PD_CLUSTER3	BIT(3)
#define VOP2_PD_DSC_8K		BIT(5)
#define VOP2_PD_DSC_4K		BIT(6)
#define VOP2_PD_ESMART		BIT(7)

/*
 * vop2 submem power gate,
 * should be all none zero, 0 will be
 * treat as invalid;
 */
#define VOP2_MEM_PG_VP0		BIT(0)
#define VOP2_MEM_PG_VP1		BIT(1)
#define VOP2_MEM_PG_VP2		BIT(2)
#define VOP2_MEM_PG_VP3		BIT(3)
#define VOP2_MEM_PG_DB0		BIT(4)
#define VOP2_MEM_PG_DB1		BIT(5)
#define VOP2_MEM_PG_DB2		BIT(6)
#define VOP2_MEM_PG_WB		BIT(7)

#define DSP_BG_SWAP		0x1
#define DSP_RB_SWAP		0x2
#define DSP_RG_SWAP		0x4
#define DSP_DELTA_SWAP		0x8

enum vop_csc_mode {
	CSC_RGB,
	CSC_YUV,
};

enum vop_dsc_interface_mode {
	VOP_DSC_IF_DISABLE = 0,
	VOP_DSC_IF_HDMI = 1,
	VOP_DSC_IF_MIPI_DS_MODE = 2,
	VOP_DSC_IF_MIPI_VIDEO_MODE = 3,
};

struct vop_reg_data {
	uint32_t offset;
	uint32_t value;
};

struct vop_csc {
	struct vop_reg y2r_en;
	struct vop_reg r2r_en;
	struct vop_reg r2y_en;
	struct vop_reg csc_mode;

	uint32_t y2r_offset;
	uint32_t r2r_offset;
	uint32_t r2y_offset;
};

struct vop_rect {
	int width;
	int height;
};

enum vop2_scale_up_mode {
	VOP2_SCALE_UP_NRST_NBOR,
	VOP2_SCALE_UP_BIL,
	VOP2_SCALE_UP_BIC,
};

enum vop2_scale_down_mode {
	VOP2_SCALE_DOWN_NRST_NBOR,
	VOP2_SCALE_DOWN_BIL,
	VOP2_SCALE_DOWN_AVG,
};

struct vop2_cluster_regs {
	struct vop_reg enable;
	struct vop_reg afbc_enable;
	struct vop_reg lb_mode;

	struct vop_reg src_color_ctrl;
	struct vop_reg dst_color_ctrl;
	struct vop_reg src_alpha_ctrl;
	struct vop_reg dst_alpha_ctrl;
};

struct vop2_scl_regs {
	struct vop_reg scale_yrgb_x;
	struct vop_reg scale_yrgb_y;
	struct vop_reg scale_cbcr_x;
	struct vop_reg scale_cbcr_y;
	struct vop_reg yrgb_hor_scl_mode;
	struct vop_reg yrgb_hscl_filter_mode;
	struct vop_reg yrgb_ver_scl_mode;
	struct vop_reg yrgb_vscl_filter_mode;
	struct vop_reg cbcr_ver_scl_mode;
	struct vop_reg cbcr_hscl_filter_mode;
	struct vop_reg cbcr_hor_scl_mode;
	struct vop_reg cbcr_vscl_filter_mode;
	struct vop_reg vsd_cbcr_gt2;
	struct vop_reg vsd_cbcr_gt4;
	struct vop_reg vsd_yrgb_gt2;
	struct vop_reg vsd_yrgb_gt4;
	struct vop_reg bic_coe_sel;
};

struct vop2_win_regs {
	const struct vop2_scl_regs *scl;
	const struct vop2_cluster_regs *cluster;
	const struct vop_afbc *afbc;

	struct vop_reg gate;
	struct vop_reg enable;
	struct vop_reg format;
	struct vop_reg csc_mode;
	struct vop_reg xmirror;
	struct vop_reg ymirror;
	struct vop_reg rb_swap;
	struct vop_reg uv_swap;
	struct vop_reg act_info;
	struct vop_reg dsp_info;
	struct vop_reg dsp_st;
	struct vop_reg yrgb_mst;
	struct vop_reg uv_mst;
	struct vop_reg yrgb_vir;
	struct vop_reg uv_vir;
	struct vop_reg yuv_clip;
	struct vop_reg lb_mode;
	struct vop_reg y2r_en;
	struct vop_reg r2y_en;
	struct vop_reg channel;
	struct vop_reg dst_alpha_ctl;
	struct vop_reg src_alpha_ctl;
	struct vop_reg alpha_mode;
	struct vop_reg alpha_en;
	struct vop_reg global_alpha_val;
	struct vop_reg color_key;
	struct vop_reg color_key_en;
	struct vop_reg dither_up;
	struct vop_reg axi_id;
	struct vop_reg axi_yrgb_id;
	struct vop_reg axi_uv_id;
};

struct vop2_video_port_regs {
	struct vop_reg cfg_done;
	struct vop_reg overlay_mode;
	struct vop_reg dsp_background;
	struct vop_reg port_mux;
	struct vop_reg out_mode;
	struct vop_reg standby;
	struct vop_reg dsp_interlace;
	struct vop_reg dsp_filed_pol;
	struct vop_reg dsp_data_swap;
	struct vop_reg post_dsp_out_r2y;
	struct vop_reg pre_scan_htiming;
	struct vop_reg htotal_pw;
	struct vop_reg hact_st_end;
	struct vop_reg vtotal_pw;
	struct vop_reg vact_st_end;
	struct vop_reg vact_st_end_f1;
	struct vop_reg vs_st_end_f1;
	struct vop_reg hpost_st_end;
	struct vop_reg vpost_st_end;
	struct vop_reg vpost_st_end_f1;
	struct vop_reg post_scl_factor;
	struct vop_reg post_scl_ctrl;
	struct vop_reg dither_down_sel;
	struct vop_reg dither_down_mode;
	struct vop_reg dither_down_en;
	struct vop_reg pre_dither_down_en;
	struct vop_reg dither_up_en;
	struct vop_reg bg_dly;

	struct vop_reg core_dclk_div;
	struct vop_reg p2i_en;
	struct vop_reg dual_channel_en;
	struct vop_reg dual_channel_swap;
	struct vop_reg dsp_lut_en;

	struct vop_reg dclk_div2;
	struct vop_reg dclk_div2_phase_lock;

	struct vop_reg hdr10_en;
	struct vop_reg hdr_lut_update_en;
	struct vop_reg hdr_lut_mode;
	struct vop_reg hdr_lut_mst;
	struct vop_reg sdr2hdr_eotf_en;
	struct vop_reg sdr2hdr_r2r_en;
	struct vop_reg sdr2hdr_r2r_mode;
	struct vop_reg sdr2hdr_oetf_en;
	struct vop_reg sdr2hdr_bypass_en;
	struct vop_reg sdr2hdr_auto_gating_en;
	struct vop_reg sdr2hdr_path_en;
	struct vop_reg hdr2sdr_en;
	struct vop_reg hdr2sdr_bypass_en;
	struct vop_reg hdr2sdr_auto_gating_en;
	struct vop_reg hdr2sdr_src_min;
	struct vop_reg hdr2sdr_src_max;
	struct vop_reg hdr2sdr_normfaceetf;
	struct vop_reg hdr2sdr_dst_min;
	struct vop_reg hdr2sdr_dst_max;
	struct vop_reg hdr2sdr_normfacgamma;
	uint32_t hdr2sdr_eetf_oetf_y0_offset;
	uint32_t hdr2sdr_sat_y0_offset;
	uint32_t sdr2hdr_eotf_oetf_y0_offset;
	uint32_t sdr2hdr_oetf_dx_pow1_offset;
	uint32_t sdr2hdr_oetf_xn1_offset;
	struct vop_reg hdr_src_color_ctrl;
	struct vop_reg hdr_dst_color_ctrl;
	struct vop_reg hdr_src_alpha_ctrl;
	struct vop_reg hdr_dst_alpha_ctrl;
	struct vop_reg bg_mix_ctrl;

	/* BCSH */
	struct vop_reg bcsh_brightness;
	struct vop_reg bcsh_contrast;
	struct vop_reg bcsh_sat_con;
	struct vop_reg bcsh_sin_hue;
	struct vop_reg bcsh_cos_hue;
	struct vop_reg bcsh_r2y_csc_mode;
	struct vop_reg bcsh_r2y_en;
	struct vop_reg bcsh_y2r_csc_mode;
	struct vop_reg bcsh_y2r_en;
	struct vop_reg bcsh_out_mode;
	struct vop_reg bcsh_en;

	/* 3d lut */
	struct vop_reg cubic_lut_en;
	struct vop_reg cubic_lut_update_en;
	struct vop_reg cubic_lut_mst;

	/* cru */
	struct vop_reg dclk_core_div;
	struct vop_reg dclk_out_div;
	struct vop_reg dclk_src_sel;

	struct vop_reg splice_en;

	struct vop_reg edpi_wms_hold_en;
	struct vop_reg edpi_te_en;
	struct vop_reg edpi_wms_fs;
	struct vop_reg gamma_update_en;
	struct vop_reg lut_dma_rid;
};

struct vop2_power_domain_regs {
	struct vop_reg pd;
	struct vop_reg status;
	struct vop_reg bisr_en_status;
	struct vop_reg pmu_status;
};

struct vop2_dsc_regs {
	/* DSC SYS CTRL */
	struct vop_reg dsc_port_sel;
	struct vop_reg dsc_man_mode;
	struct vop_reg dsc_interface_mode;
	struct vop_reg dsc_pixel_num;
	struct vop_reg dsc_pxl_clk_div;
	struct vop_reg dsc_cds_clk_div;
	struct vop_reg dsc_txp_clk_div;
	struct vop_reg dsc_init_dly_mode;
	struct vop_reg dsc_scan_en;
	struct vop_reg dsc_halt_en;
	struct vop_reg rst_deassert;
	struct vop_reg dsc_flush;
	struct vop_reg dsc_cfg_done;
	struct vop_reg dsc_init_dly_num;
	struct vop_reg scan_timing_para_imd_en;
	struct vop_reg dsc_htotal_pw;
	struct vop_reg dsc_hact_st_end;
	struct vop_reg dsc_vtotal_pw;
	struct vop_reg dsc_vact_st_end;
	struct vop_reg dsc_error_status;

	/* DSC encoder */
	struct vop_reg dsc_pps0_3;
	struct vop_reg dsc_en;
	struct vop_reg dsc_rbit;
	struct vop_reg dsc_rbyt;
	struct vop_reg dsc_flal;
	struct vop_reg dsc_mer;
	struct vop_reg dsc_epb;
	struct vop_reg dsc_epl;
	struct vop_reg dsc_nslc;
	struct vop_reg dsc_sbo;
	struct vop_reg dsc_ifep;
	struct vop_reg dsc_pps_upd;
	struct vop_reg dsc_status;
	struct vop_reg dsc_ecw;
};

struct vop2_wb_regs {
	struct vop_reg enable;
	struct vop_reg format;
	struct vop_reg dither_en;
	struct vop_reg r2y_en;
	struct vop_reg yrgb_mst;
	struct vop_reg uv_mst;
	struct vop_reg vp_id;
	struct vop_reg fifo_throd;
	struct vop_reg scale_x_factor;
	struct vop_reg scale_x_en;
	struct vop_reg scale_y_en;
	struct vop_reg axi_yrgb_id;
	struct vop_reg axi_uv_id;
};

struct vop2_power_domain_data {
	uint8_t id;
	uint8_t parent_id;
	/*
	 * @module_id_mask: module id of which module this power domain is belongs to.
	 * PD_CLUSTER0,1,2,3 only belongs to CLUSTER0/1/2/3, PD_Esmart0 shared by Esmart1/2/3
	 */
	uint32_t module_id_mask;

	const struct vop2_power_domain_regs *regs;
};

/*
 * connector interface(RGB/HDMI/eDP/DP/MIPI) data
 */
struct vop2_connector_if_data {
	u32 id;
	const char *clk_src_name;
	const char *clk_parent_name;
	const char *pixclk_name;
	const char *dclk_name;
	u32 post_proc_div_shift;
	u32 if_div_shift;
	u32 if_div_yuv420_shift;
	u32 bus_div_shift;
	u32 pixel_clk_div_shift;
};

struct vop2_win_data {
	const char *name;
	uint8_t phys_id;
	uint8_t splice_win_id;
	uint8_t pd_id;
	uint8_t axi_id;
	uint8_t axi_yrgb_id;
	uint8_t axi_uv_id;

	uint32_t base;
	enum drm_plane_type type;

	uint32_t nformats;
	const uint32_t *formats;
	const uint64_t *format_modifiers;
	const unsigned int supported_rotations;

	const struct vop2_win_regs *regs;
	const struct vop2_win_regs **area;
	unsigned int area_size;

	/*
	 * vertical/horizontal scale up/down filter mode
	 */
	const u8 hsu_filter_mode;
	const u8 hsd_filter_mode;
	const u8 vsu_filter_mode;
	const u8 vsd_filter_mode;
	/**
	 * @layer_sel_id: defined by register OVERLAY_LAYER_SEL of VOP2
	 */
	int layer_sel_id;
	uint64_t feature;

	unsigned int max_upscale_factor;
	unsigned int max_downscale_factor;
	const uint8_t dly[VOP2_DLY_MODE_MAX];
};

struct dsc_error_info {
	u32 dsc_error_val;
	char dsc_error_info[50];
};

struct vop2_dsc_data {
	uint8_t id;
	uint8_t pd_id;
	uint8_t max_slice_num;
	uint8_t max_linebuf_depth;	/* used to generate the bitstream */
	uint8_t min_bits_per_pixel;	/* bit num after encoder compress */
	const char *dsc_txp_clk_src_name;
	const char *dsc_txp_clk_name;
	const char *dsc_pxl_clk_name;
	const char *dsc_cds_clk_name;
	const struct vop2_dsc_regs *regs;
};

struct vop2_wb_data {
	uint32_t nformats;
	const uint32_t *formats;
	struct vop_rect max_output;
	const struct vop2_wb_regs *regs;
	uint32_t fifo_depth;
};

struct vop2_video_port_data {
	char id;
	uint8_t splice_vp_id;
	uint16_t lut_dma_rid;
	u32 feature;
	uint64_t soc_id[VOP2_SOC_VARIANT];
	u16 gamma_lut_len;
	u16 cubic_lut_len;
	unsigned long dclk_max;
	struct vop_rect max_output;
	const u8 pre_scan_max_dly[4];
	const struct vop_intr *intr;
	const struct vop_hdr_table *hdr_table;
	const struct vop2_video_port_regs *regs;
};

struct vop2_layer_regs {
	struct vop_reg layer_sel;
};

/**
 * struct vop2_layer_data - The logic graphic layer in vop2
 *
 * The zorder:
 *   LAYERn
 *   LAYERn-1
 *     .
 *     .
 *     .
 *   LAYER5
 *   LAYER4
 *   LAYER3
 *   LAYER2
 *   LAYER1
 *   LAYER0
 *
 * Each layer can select a unused window as input than feed to
 * mixer for overlay.
 *
 * The pipeline in vop2:
 *
 * win-->layer-->mixer-->vp--->connector(RGB/LVDS/HDMI/MIPI)
 *
 */
struct vop2_layer_data {
	char id;
	const struct vop2_layer_regs *regs;
};

struct vop_grf_ctrl {
	struct vop_reg grf_dclk_inv;
	struct vop_reg grf_bt1120_clk_inv;
	struct vop_reg grf_bt656_clk_inv;
	struct vop_reg grf_edp0_en;
	struct vop_reg grf_edp1_en;
	struct vop_reg grf_hdmi0_en;
	struct vop_reg grf_hdmi1_en;
	struct vop_reg grf_hdmi0_dsc_en;
	struct vop_reg grf_hdmi1_dsc_en;
	struct vop_reg grf_hdmi0_pin_pol;
	struct vop_reg grf_hdmi1_pin_pol;
};

struct vop2_ctrl {
	struct vop_reg cfg_done_en;
	struct vop_reg wb_cfg_done;
	struct vop_reg auto_gating_en;
	struct vop_reg ovl_cfg_done_port;
	struct vop_reg ovl_port_mux_cfg_done_imd;
	struct vop_reg ovl_port_mux_cfg;
	struct vop_reg if_ctrl_cfg_done_imd;
	struct vop_reg version;
	struct vop_reg standby;
	struct vop_reg dma_stop;
	struct vop_reg lut_dma_en;
	struct vop_reg axi_outstanding_max_num;
	struct vop_reg axi_max_outstanding_en;
	struct vop_reg hdmi_dclk_out_en;
	struct vop_reg rgb_en;
	struct vop_reg hdmi0_en;
	struct vop_reg hdmi1_en;
	struct vop_reg dp0_en;
	struct vop_reg dp1_en;
	struct vop_reg edp0_en;
	struct vop_reg edp1_en;
	struct vop_reg mipi0_en;
	struct vop_reg mipi1_en;
	struct vop_reg lvds0_en;
	struct vop_reg lvds1_en;
	struct vop_reg bt656_en;
	struct vop_reg bt1120_en;
	struct vop_reg dclk_pol;
	struct vop_reg pin_pol;
	struct vop_reg rgb_dclk_pol;
	struct vop_reg rgb_pin_pol;
	struct vop_reg lvds_dclk_pol;
	struct vop_reg lvds_pin_pol;
	struct vop_reg hdmi_dclk_pol;
	struct vop_reg hdmi_pin_pol;
	struct vop_reg edp_dclk_pol;
	struct vop_reg edp_pin_pol;
	struct vop_reg mipi_dclk_pol;
	struct vop_reg mipi_pin_pol;
	struct vop_reg dp0_dclk_pol;
	struct vop_reg dp0_pin_pol;
	struct vop_reg dp1_dclk_pol;
	struct vop_reg dp1_pin_pol;

	/* This will be reference by win_phy_id */
	struct vop_reg win_vp_id[16];
	struct vop_reg win_dly[16];

	/* connector mux */
	struct vop_reg rgb_mux;
	struct vop_reg hdmi0_mux;
	struct vop_reg hdmi1_mux;
	struct vop_reg dp0_mux;
	struct vop_reg dp1_mux;
	struct vop_reg edp0_mux;
	struct vop_reg edp1_mux;
	struct vop_reg mipi0_mux;
	struct vop_reg mipi1_mux;
	struct vop_reg lvds0_mux;
	struct vop_reg lvds1_mux;

	struct vop_reg lvds_dual_en;
	struct vop_reg lvds_dual_mode;
	struct vop_reg lvds_dual_channel_swap;

	struct vop_reg dp_dual_en;
	struct vop_reg edp_dual_en;
	struct vop_reg hdmi_dual_en;
	struct vop_reg mipi_dual_en;

	struct vop_reg hdmi0_dclk_div;
	struct vop_reg hdmi0_pixclk_div;
	struct vop_reg edp0_dclk_div;
	struct vop_reg edp0_pixclk_div;

	struct vop_reg hdmi1_dclk_div;
	struct vop_reg hdmi1_pixclk_div;
	struct vop_reg edp1_dclk_div;
	struct vop_reg edp1_pixclk_div;

	struct vop_reg mipi0_pixclk_div;
	struct vop_reg mipi1_pixclk_div;
	struct vop_reg mipi0_ds_mode;
	struct vop_reg mipi1_ds_mode;

	struct vop_reg src_color_ctrl;
	struct vop_reg dst_color_ctrl;
	struct vop_reg src_alpha_ctrl;
	struct vop_reg dst_alpha_ctrl;

	struct vop_reg bt1120_yc_swap;
	struct vop_reg bt656_yc_swap;
	struct vop_reg gamma_port_sel;
	struct vop_reg pd_off_imd;

	struct vop_reg otp_en;
	struct vop_reg reg_done_frm;
	struct vop_reg cfg_done;
};

/**
 * VOP2 data structe
 *
 * @version: VOP IP version
 * @win_size: hardware win number
 */
struct vop2_data {
	uint32_t version;
	uint32_t feature;
	uint8_t nr_dscs;
	uint8_t nr_dsc_ecw;
	uint8_t nr_dsc_buffer_flow;
	uint8_t nr_vps;
	uint8_t nr_mixers;
	uint8_t nr_layers;
	uint8_t nr_axi_intr;
	uint8_t nr_gammas;
	uint8_t nr_conns;
	uint8_t nr_pds;
	uint8_t nr_mem_pgs;
	bool delayed_pd;
	const struct vop_intr *axi_intr;
	const struct vop2_ctrl *ctrl;
	const struct vop2_dsc_data *dsc;
	const struct dsc_error_info *dsc_error_ecw;
	const struct dsc_error_info *dsc_error_buffer_flow;
	const struct vop2_win_data *win;
	const struct vop2_video_port_data *vp;
	const struct vop2_connector_if_data *conn;
	const struct vop2_wb_data *wb;
	const struct vop2_layer_data *layer;
	const struct vop2_power_domain_data *pd;
	const struct vop2_power_domain_data *mem_pg;
	const struct vop_csc_table *csc_table;
	const struct vop_hdr_table *hdr_table;
	const struct vop_grf_ctrl *sys_grf;
	const struct vop_grf_ctrl *grf;
	const struct vop_grf_ctrl *vo0_grf;
	const struct vop_grf_ctrl *vo1_grf;
	struct vop_rect max_input;
	struct vop_rect max_output;

	unsigned int win_size;
};

/* interrupt define */
#define FS_NEW_INTR			BIT(4)
#define ADDR_SAME_INTR			BIT(5)
#define LINE_FLAG1_INTR			BIT(6)
#define WIN0_EMPTY_INTR			BIT(7)
#define WIN1_EMPTY_INTR			BIT(8)
#define WIN2_EMPTY_INTR			BIT(9)
#define WIN3_EMPTY_INTR			BIT(10)
#define HWC_EMPTY_INTR			BIT(11)
#define POST_BUF_EMPTY_INTR		BIT(12)
#define PWM_GEN_INTR			BIT(13)
#define DMA_FINISH_INTR			BIT(14)
#define FS_FIELD_INTR			BIT(15)
#define FE_INTR				BIT(16)
#define WB_UV_FIFO_FULL_INTR		BIT(17)
#define WB_YRGB_FIFO_FULL_INTR		BIT(18)
#define WB_COMPLETE_INTR		BIT(19)

/*
 * display output interface supported by rockchip lcdc
 */
#define ROCKCHIP_OUT_MODE_P888		0
#define ROCKCHIP_OUT_MODE_BT1120	0
#define ROCKCHIP_OUT_MODE_P666		1
#define ROCKCHIP_OUT_MODE_P565		2
#define ROCKCHIP_OUT_MODE_BT656		5
#define ROCKCHIP_OUT_MODE_S888		8
#define ROCKCHIP_OUT_MODE_S888_DUMMY	12
#define ROCKCHIP_OUT_MODE_YUV420	14
/* for use special outface */
#define ROCKCHIP_OUT_MODE_AAAA		15

enum vop_csc_format {
	CSC_BT601L,
	CSC_BT709L,
	CSC_BT601F,
	CSC_BT2020,
};

enum src_factor_mode {
	SRC_FAC_ALPHA_ZERO,
	SRC_FAC_ALPHA_ONE,
	SRC_FAC_ALPHA_DST,
	SRC_FAC_ALPHA_DST_INVERSE,
	SRC_FAC_ALPHA_SRC,
	SRC_FAC_ALPHA_SRC_GLOBAL,
};

enum dst_factor_mode {
	DST_FAC_ALPHA_ZERO,
	DST_FAC_ALPHA_ONE,
	DST_FAC_ALPHA_SRC,
	DST_FAC_ALPHA_SRC_INVERSE,
	DST_FAC_ALPHA_DST,
	DST_FAC_ALPHA_DST_GLOBAL,
};

#define RK3568_GRF_VO_CON1			0x0364
/* System registers definition */
#define RK3568_REG_CFG_DONE			0x000
#define RK3568_VOP2_WB_CFG_DONE			BIT(14)
#define RK3568_VOP2_GLB_CFG_DONE_EN		BIT(15)
#define RK3568_VERSION_INFO			0x004
#define RK3568_SYS_AUTO_GATING_CTRL		0x008
#define RK3568_SYS_AXI_LUT_CTRL			0x024
#define RK3568_DSP_IF_EN			0x028
#define RK3568_DSP_IF_CTRL			0x02c
#define RK3568_DSP_IF_POL			0x030
#define RK3568_SYS_PD_CTRL			0x034
#define RK3568_WB_CTRL				0x40
#define RK3568_WB_XSCAL_FACTOR			0x44
#define RK3568_WB_YRGB_MST			0x48
#define RK3568_WB_CBR_MST			0x4C
#define RK3568_OTP_WIN_EN			0x050
#define RK3568_LUT_PORT_SEL			0x058
#define RK3568_SYS_STATUS0			0x060
#define RK3568_SYS_STATUS1			0x64
#define RK3568_SYS_STATUS2			0x68
#define RK3568_SYS_STATUS3			0x6C
#define RK3568_VP_LINE_FLAG(vp)			(0x70 + (vp) * 0x4)
#define RK3588_VP_LINE_FLAG(vp)			RK3568_VP_LINE_FLAG(vp)
#define RK3568_SYS0_INT_EN			0x80
#define RK3568_SYS0_INT_CLR			0x84
#define RK3568_SYS0_INT_STATUS			0x88
#define RK3568_SYS1_INT_EN			0x90
#define RK3568_SYS1_INT_CLR			0x94
#define RK3568_SYS1_INT_STATUS			0x98
#define RK3568_VP_INT_EN(vp)			(0xA0 + (vp) * 0x10)
#define RK3588_VP_INT_EN(vp)			RK3568_VP_INT_EN(vp)
#define RK3568_VP_INT_CLR(vp)			(0xA4 + (vp) * 0x10)
#define RK3588_VP_INT_CLR(vp)			RK3568_VP_INT_CLR(vp)
#define RK3568_VP_INT_STATUS(vp)		(0xA8 + (vp) * 0x10)
#define RK3588_VP_INT_STATUS(vp)		RK3568_VP_INT_STATUS(vp)
#define RK3568_VP_INT_RAW_STATUS(vp)		(0xAC + (vp) * 0x10)
#define RK3588_VP_INT_RAW_STATUS(vp)		RK3568_VP_INT_RAW_STATUS(vp)

#define RK3588_DSC_8K_SYS_CTRL			0x200
#define RK3588_DSC_8K_RST			0x204
#define RK3588_DSC_8K_CFG_DONE			0x208
#define RK3588_DSC_8K_INIT_DLY			0x20C
#define RK3588_DSC_8K_HTOTAL_HS_END		0x210
#define RK3588_DSC_8K_HACT_ST_END		0x214
#define RK3588_DSC_8K_VTOTAL_VS_END		0x218
#define RK3588_DSC_8K_VACT_ST_END		0x21C
#define RK3588_DSC_8K_STATUS			0x220
#define RK3588_DSC_4K_SYS_CTRL			0x230
#define RK3588_DSC_4K_RST			0x234
#define RK3588_DSC_4K_CFG_DONE			0x238
#define RK3588_DSC_4K_INIT_DLY			0x23C
#define RK3588_DSC_4K_HTOTAL_HS_END		0x240
#define RK3588_DSC_4K_HACT_ST_END		0x244
#define RK3588_DSC_4K_VTOTAL_VS_END		0x248
#define RK3588_DSC_4K_VACT_ST_END		0x24C
#define RK3588_DSC_4K_STATUS			0x250

/* Video Port registers definition */
#define RK3568_VP0_DSP_CTRL				0xC00
#define RK3568_VP0_DUAL_CHANNEL_CTRL			0xC04
#define RK3568_VP0_COLOR_BAR_CTRL			0xC08
#define RK3568_VP0_CLK_CTRL				0xC0C
#define RK3568_VP0_3D_LUT_CTRL				0xC10
#define RK3568_VP0_3D_LUT_MST				0xC20
#define RK3568_VP0_DSP_BG				0xC2C
#define RK3568_VP0_PRE_SCAN_HTIMING			0xC30
#define RK3568_VP0_POST_DSP_HACT_INFO			0xC34
#define RK3568_VP0_POST_DSP_VACT_INFO			0xC38
#define RK3568_VP0_POST_SCL_FACTOR_YRGB			0xC3C
#define RK3568_VP0_POST_SCL_CTRL			0xC40
#define RK3568_VP0_POST_DSP_VACT_INFO_F1		0xC44
#define RK3568_VP0_DSP_HTOTAL_HS_END			0xC48
#define RK3568_VP0_DSP_HACT_ST_END			0xC4C
#define RK3568_VP0_DSP_VTOTAL_VS_END			0xC50
#define RK3568_VP0_DSP_VACT_ST_END			0xC54
#define RK3568_VP0_DSP_VS_ST_END_F1			0xC58
#define RK3568_VP0_DSP_VACT_ST_END_F1			0xC5C
#define RK3568_VP0_BCSH_CTRL				0xC60
#define RK3568_VP0_BCSH_BCS				0xC64
#define RK3568_VP0_BCSH_H				0xC68
#define RK3568_VP0_BCSH_COLOR_BAR			0xC6C

#define RK3568_VP1_DSP_CTRL				0xD00
#define RK3568_VP1_DUAL_CHANNEL_CTRL			0xD04
#define RK3568_VP1_COLOR_BAR_CTRL			0xD08
#define RK3568_VP1_CLK_CTRL				0xD0C
#define RK3588_VP1_3D_LUT_CTRL				0xD10
#define RK3588_VP1_3D_LUT_MST				0xD20
#define RK3568_VP1_DSP_BG				0xD2C
#define RK3568_VP1_PRE_SCAN_HTIMING			0xD30
#define RK3568_VP1_POST_DSP_HACT_INFO			0xD34
#define RK3568_VP1_POST_DSP_VACT_INFO			0xD38
#define RK3568_VP1_POST_SCL_FACTOR_YRGB			0xD3C
#define RK3568_VP1_POST_SCL_CTRL			0xD40
#define RK3568_VP1_DSP_HACT_INFO			0xD34
#define RK3568_VP1_DSP_VACT_INFO			0xD38
#define RK3568_VP1_POST_DSP_VACT_INFO_F1		0xD44
#define RK3568_VP1_DSP_HTOTAL_HS_END			0xD48
#define RK3568_VP1_DSP_HACT_ST_END			0xD4C
#define RK3568_VP1_DSP_VTOTAL_VS_END			0xD50
#define RK3568_VP1_DSP_VACT_ST_END			0xD54
#define RK3568_VP1_DSP_VS_ST_END_F1			0xD58
#define RK3568_VP1_DSP_VACT_ST_END_F1			0xD5C
#define RK3568_VP1_BCSH_CTRL				0xD60
#define RK3568_VP1_BCSH_BCS				0xD64
#define RK3568_VP1_BCSH_H				0xD68
#define RK3568_VP1_BCSH_COLOR_BAR			0xD6C

#define RK3568_VP2_DSP_CTRL				0xE00
#define RK3568_VP2_DUAL_CHANNEL_CTRL			0xE04
#define RK3568_VP2_COLOR_BAR_CTRL			0xE08
#define RK3568_VP2_CLK_CTRL				0xE0C
#define RK3588_VP2_3D_LUT_CTRL				0xE10
#define RK3588_VP2_3D_LUT_MST				0xE20
#define RK3568_VP2_DSP_BG				0xE2C
#define RK3568_VP2_PRE_SCAN_HTIMING			0xE30
#define RK3568_VP2_POST_DSP_HACT_INFO			0xE34
#define RK3568_VP2_POST_DSP_VACT_INFO			0xE38
#define RK3568_VP2_POST_SCL_FACTOR_YRGB			0xE3C
#define RK3568_VP2_POST_SCL_CTRL			0xE40
#define RK3568_VP2_DSP_HACT_INFO			0xE34
#define RK3568_VP2_DSP_VACT_INFO			0xE38
#define RK3568_VP2_POST_DSP_VACT_INFO_F1		0xE44
#define RK3568_VP2_DSP_HTOTAL_HS_END			0xE48
#define RK3568_VP2_DSP_HACT_ST_END			0xE4C
#define RK3568_VP2_DSP_VTOTAL_VS_END			0xE50
#define RK3568_VP2_DSP_VACT_ST_END			0xE54
#define RK3568_VP2_DSP_VS_ST_END_F1			0xE58
#define RK3568_VP2_DSP_VACT_ST_END_F1			0xE5C
#define RK3568_VP2_BCSH_CTRL				0xE60
#define RK3568_VP2_BCSH_BCS				0xE64
#define RK3568_VP2_BCSH_H				0xE68
#define RK3568_VP2_BCSH_COLOR_BAR			0xE6C

#define RK3588_VP3_DSP_CTRL				0xF00
#define RK3588_VP3_DUAL_CHANNEL_CTRL			0xF04
#define RK3588_VP3_COLOR_BAR_CTRL			0xF08
#define RK3568_VP3_CLK_CTRL				0xF0C
#define RK3588_VP3_DSP_BG				0xF2C
#define RK3588_VP3_PRE_SCAN_HTIMING			0xF30
#define RK3588_VP3_POST_DSP_HACT_INFO			0xF34
#define RK3588_VP3_POST_DSP_VACT_INFO			0xF38
#define RK3588_VP3_POST_SCL_FACTOR_YRGB			0xF3C
#define RK3588_VP3_POST_SCL_CTRL			0xF40
#define RK3588_VP3_DSP_HACT_INFO			0xF34
#define RK3588_VP3_DSP_VACT_INFO			0xF38
#define RK3588_VP3_POST_DSP_VACT_INFO_F1		0xF44
#define RK3588_VP3_DSP_HTOTAL_HS_END			0xF48
#define RK3588_VP3_DSP_HACT_ST_END			0xF4C
#define RK3588_VP3_DSP_VTOTAL_VS_END			0xF50
#define RK3588_VP3_DSP_VACT_ST_END			0xF54
#define RK3588_VP3_DSP_VS_ST_END_F1			0xF58
#define RK3588_VP3_DSP_VACT_ST_END_F1			0xF5C
#define RK3588_VP3_BCSH_CTRL				0xF60
#define RK3588_VP3_BCSH_BCS				0xF64
#define RK3588_VP3_BCSH_H				0xF68
#define RK3588_VP3_BCSH_COLOR_BAR			0xF6C

/* Overlay registers definition    */
#define RK3568_OVL_CTRL				0x600
#define RK3568_OVL_LAYER_SEL			0x604
#define RK3568_OVL_PORT_SEL			0x608
#define RK3568_CLUSTER0_MIX_SRC_COLOR_CTRL	0x610
#define RK3568_CLUSTER0_MIX_DST_COLOR_CTRL	0x614
#define RK3568_CLUSTER0_MIX_SRC_ALPHA_CTRL	0x618
#define RK3568_CLUSTER0_MIX_DST_ALPHA_CTRL	0x61C
#define RK3568_CLUSTER1_MIX_SRC_COLOR_CTRL	0x620
#define RK3568_CLUSTER1_MIX_DST_COLOR_CTRL	0x624
#define RK3568_CLUSTER1_MIX_SRC_ALPHA_CTRL	0x628
#define RK3568_CLUSTER1_MIX_DST_ALPHA_CTRL	0x62C
#define RK3588_CLUSTER2_MIX_SRC_COLOR_CTRL	0x630
#define RK3588_CLUSTER2_MIX_DST_COLOR_CTRL	0x634
#define RK3588_CLUSTER2_MIX_SRC_ALPHA_CTRL	0x638
#define RK3588_CLUSTER2_MIX_DST_ALPHA_CTRL	0x63C
#define RK3588_CLUSTER3_MIX_SRC_COLOR_CTRL	0x640
#define RK3588_CLUSTER3_MIX_DST_COLOR_CTRL	0x644
#define RK3588_CLUSTER3_MIX_SRC_ALPHA_CTRL	0x648
#define RK3588_CLUSTER3_MIX_DST_ALPHA_CTRL	0x64C
#define RK3568_MIX0_SRC_COLOR_CTRL		0x650
#define RK3568_MIX0_DST_COLOR_CTRL		0x654
#define RK3568_MIX0_SRC_ALPHA_CTRL		0x658
#define RK3568_MIX0_DST_ALPHA_CTRL		0x65C
#define RK3568_HDR0_SRC_COLOR_CTRL		0x6C0
#define RK3568_HDR0_DST_COLOR_CTRL		0x6C4
#define RK3568_HDR0_SRC_ALPHA_CTRL		0x6C8
#define RK3568_HDR0_DST_ALPHA_CTRL		0x6CC
#define RK3568_HDR1_SRC_COLOR_CTRL		0x6D0
#define RK3568_HDR1_DST_COLOR_CTRL		0x6D4
#define RK3568_HDR1_SRC_ALPHA_CTRL		0x6D8
#define RK3568_HDR1_DST_ALPHA_CTRL		0x6DC
#define RK3568_VP0_BG_MIX_CTRL			0x6E0
#define RK3568_VP1_BG_MIX_CTRL			0x6E4
#define RK3568_VP2_BG_MIX_CTRL			0x6E8
#define RK3588_VP3_BG_MIX_CTRL			0x6EC
#define RK3568_CLUSTER_DLY_NUM			0x6F0
#define RK3568_CLUSTER_DLY_NUM1			0x6F4
#define RK3568_SMART_DLY_NUM			0x6F8

/* Cluster0 register definition */
#define RK3568_CLUSTER0_WIN0_CTRL0		0x1000
#define RK3568_CLUSTER0_WIN0_CTRL1		0x1004
#define RK3568_CLUSTER0_WIN0_CTRL2		0x1008
#define RK3568_CLUSTER0_WIN0_YRGB_MST		0x1010
#define RK3568_CLUSTER0_WIN0_CBR_MST		0x1014
#define RK3568_CLUSTER0_WIN0_VIR		0x1018
#define RK3568_CLUSTER0_WIN0_ACT_INFO		0x1020
#define RK3568_CLUSTER0_WIN0_DSP_INFO		0x1024
#define RK3568_CLUSTER0_WIN0_DSP_ST		0x1028
#define RK3568_CLUSTER0_WIN0_SCL_FACTOR_YRGB	0x1030
#define RK3568_CLUSTER0_WIN0_AFBCD_TRANSFORM_OFFSET	0x103C
#define RK3568_CLUSTER0_WIN0_AFBCD_OUTPUT_CTRL	0x1050
#define RK3568_CLUSTER0_WIN0_AFBCD_ROTATE_MODE	0x1054
#define RK3568_CLUSTER0_WIN0_AFBCD_HDR_PTR	0x1058
#define RK3568_CLUSTER0_WIN0_AFBCD_VIR_WIDTH	0x105C
#define RK3568_CLUSTER0_WIN0_AFBCD_PIC_SIZE	0x1060
#define RK3568_CLUSTER0_WIN0_AFBCD_PIC_OFFSET	0x1064
#define RK3568_CLUSTER0_WIN0_AFBCD_DSP_OFFSET	0x1068
#define RK3568_CLUSTER0_WIN0_AFBCD_CTRL		0x106C

#define RK3568_CLUSTER0_WIN1_CTRL0		0x1080
#define RK3568_CLUSTER0_WIN1_CTRL1		0x1084
#define RK3568_CLUSTER0_WIN1_YRGB_MST		0x1090
#define RK3568_CLUSTER0_WIN1_CBR_MST		0x1094
#define RK3568_CLUSTER0_WIN1_VIR		0x1098
#define RK3568_CLUSTER0_WIN1_ACT_INFO		0x10A0
#define RK3568_CLUSTER0_WIN1_DSP_INFO		0x10A4
#define RK3568_CLUSTER0_WIN1_DSP_ST		0x10A8
#define RK3568_CLUSTER0_WIN1_SCL_FACTOR_YRGB	0x10B0
#define RK3568_CLUSTER0_WIN1_AFBCD_OUTPUT_CTRL	0x10D0
#define RK3568_CLUSTER0_WIN1_AFBCD_ROTATE_MODE	0x10D4
#define RK3568_CLUSTER0_WIN1_AFBCD_HDR_PTR	0x10D8
#define RK3568_CLUSTER0_WIN1_AFBCD_VIR_WIDTH	0x10DC
#define RK3568_CLUSTER0_WIN1_AFBCD_PIC_SIZE	0x10E0
#define RK3568_CLUSTER0_WIN1_AFBCD_PIC_OFFSET	0x10E4
#define RK3568_CLUSTER0_WIN1_AFBCD_DSP_OFFSET	0x10E8
#define RK3568_CLUSTER0_WIN1_AFBCD_CTRL		0x10EC

#define RK3568_CLUSTER0_CTRL			0x1100

#define RK3568_CLUSTER1_WIN0_CTRL0		0x1200
#define RK3568_CLUSTER1_WIN0_CTRL1		0x1204
#define RK3568_CLUSTER1_WIN0_CTRL2		0x1208
#define RK3568_CLUSTER1_WIN0_YRGB_MST		0x1210
#define RK3568_CLUSTER1_WIN0_CBR_MST		0x1214
#define RK3568_CLUSTER1_WIN0_VIR		0x1218
#define RK3568_CLUSTER1_WIN0_ACT_INFO		0x1220
#define RK3568_CLUSTER1_WIN0_DSP_INFO		0x1224
#define RK3568_CLUSTER1_WIN0_DSP_ST		0x1228
#define RK3568_CLUSTER1_WIN0_SCL_FACTOR_YRGB	0x1230
#define RK3568_CLUSTER1_WIN0_AFBCD_TRANSFORM_OFFSET	0x123C
#define RK3568_CLUSTER1_WIN0_AFBCD_OUTPUT_CTRL	0x1250
#define RK3568_CLUSTER1_WIN0_AFBCD_ROTATE_MODE	0x1254
#define RK3568_CLUSTER1_WIN0_AFBCD_HDR_PTR	0x1258
#define RK3568_CLUSTER1_WIN0_AFBCD_VIR_WIDTH	0x125C
#define RK3568_CLUSTER1_WIN0_AFBCD_PIC_SIZE	0x1260
#define RK3568_CLUSTER1_WIN0_AFBCD_PIC_OFFSET	0x1264
#define RK3568_CLUSTER1_WIN0_AFBCD_DSP_OFFSET	0x1268
#define RK3568_CLUSTER1_WIN0_AFBCD_CTRL		0x126C

#define RK3568_CLUSTER1_WIN1_CTRL0		0x1280
#define RK3568_CLUSTER1_WIN1_CTRL1		0x1284
#define RK3568_CLUSTER1_WIN1_YRGB_MST		0x1290
#define RK3568_CLUSTER1_WIN1_CBR_MST		0x1294
#define RK3568_CLUSTER1_WIN1_VIR		0x1298
#define RK3568_CLUSTER1_WIN1_ACT_INFO		0x12A0
#define RK3568_CLUSTER1_WIN1_DSP_INFO		0x12A4
#define RK3568_CLUSTER1_WIN1_DSP_ST		0x12A8
#define RK3568_CLUSTER1_WIN1_SCL_FACTOR_YRGB	0x12B0
#define RK3568_CLUSTER1_WIN1_AFBCD_OUTPUT_CTRL	0x12D0
#define RK3568_CLUSTER1_WIN1_AFBCD_ROTATE_MODE	0x12D4
#define RK3568_CLUSTER1_WIN1_AFBCD_HDR_PTR	0x12D8
#define RK3568_CLUSTER1_WIN1_AFBCD_VIR_WIDTH	0x12DC
#define RK3568_CLUSTER1_WIN1_AFBCD_PIC_SIZE	0x12E0
#define RK3568_CLUSTER1_WIN1_AFBCD_PIC_OFFSET	0x12E4
#define RK3568_CLUSTER1_WIN1_AFBCD_DSP_OFFSET	0x12E8
#define RK3568_CLUSTER1_WIN1_AFBCD_CTRL		0x12EC

#define RK3568_CLUSTER1_CTRL			0x1300

#define RK3588_CLUSTER2_WIN0_CTRL0		0x1400
#define RK3588_CLUSTER2_WIN0_CTRL1		0x1404
#define RK3588_CLUSTER2_WIN0_CTRL2		0x1408
#define RK3588_CLUSTER2_WIN0_YRGB_MST		0x1410
#define RK3588_CLUSTER2_WIN0_CBR_MST		0x1414
#define RK3588_CLUSTER2_WIN0_VIR		0x1418
#define RK3588_CLUSTER2_WIN0_ACT_INFO		0x1420
#define RK3588_CLUSTER2_WIN0_DSP_INFO		0x1424
#define RK3588_CLUSTER2_WIN0_DSP_ST		0x1428
#define RK3588_CLUSTER2_WIN0_SCL_FACTOR_YRGB	0x1430
#define RK3588_CLUSTER2_WIN0_AFBCD_TRANSFORM_OFFSET	0x143C
#define RK3588_CLUSTER2_WIN0_AFBCD_OUTPUT_CTRL	0x1450
#define RK3588_CLUSTER2_WIN0_AFBCD_ROTATE_MODE	0x1454
#define RK3588_CLUSTER2_WIN0_AFBCD_HDR_PTR	0x1458
#define RK3588_CLUSTER2_WIN0_AFBCD_VIR_WIDTH	0x145C
#define RK3588_CLUSTER2_WIN0_AFBCD_PIC_SIZE	0x1460
#define RK3588_CLUSTER2_WIN0_AFBCD_PIC_OFFSET	0x1464
#define RK3588_CLUSTER2_WIN0_AFBCD_DSP_OFFSET	0x1468
#define RK3588_CLUSTER2_WIN0_AFBCD_CTRL		0x146C

#define RK3588_CLUSTER2_WIN1_CTRL0		0x1480
#define RK3588_CLUSTER2_WIN1_CTRL1		0x1484
#define RK3588_CLUSTER2_WIN1_YRGB_MST		0x1490
#define RK3588_CLUSTER2_WIN1_CBR_MST		0x1494
#define RK3588_CLUSTER2_WIN1_VIR		0x1498
#define RK3588_CLUSTER2_WIN1_ACT_INFO		0x14A0
#define RK3588_CLUSTER2_WIN1_DSP_INFO		0x14A4
#define RK3588_CLUSTER2_WIN1_DSP_ST		0x14A8
#define RK3588_CLUSTER2_WIN1_SCL_FACTOR_YRGB	0x14B0
#define RK3588_CLUSTER2_WIN1_AFBCD_OUTPUT_CTRL	0x14D0
#define RK3588_CLUSTER2_WIN1_AFBCD_ROTATE_MODE	0x14D4
#define RK3588_CLUSTER2_WIN1_AFBCD_HDR_PTR	0x14D8
#define RK3588_CLUSTER2_WIN1_AFBCD_VIR_WIDTH	0x14DC
#define RK3588_CLUSTER2_WIN1_AFBCD_PIC_SIZE	0x14E0
#define RK3588_CLUSTER2_WIN1_AFBCD_PIC_OFFSET	0x14E4
#define RK3588_CLUSTER2_WIN1_AFBCD_DSP_OFFSET	0x14E8
#define RK3588_CLUSTER2_WIN1_AFBCD_CTRL		0x14EC

#define RK3588_CLUSTER2_CTRL			0x1500

#define RK3588_CLUSTER3_WIN0_CTRL0		0x1600
#define RK3588_CLUSTER3_WIN0_CTRL1		0x1604
#define RK3588_CLUSTER3_WIN0_CTRL2		0x1608
#define RK3588_CLUSTER3_WIN0_YRGB_MST		0x1610
#define RK3588_CLUSTER3_WIN0_CBR_MST		0x1614
#define RK3588_CLUSTER3_WIN0_VIR		0x1618
#define RK3588_CLUSTER3_WIN0_ACT_INFO		0x1620
#define RK3588_CLUSTER3_WIN0_DSP_INFO		0x1624
#define RK3588_CLUSTER3_WIN0_DSP_ST		0x1628
#define RK3588_CLUSTER3_WIN0_SCL_FACTOR_YRGB	0x1630
#define RK3588_CLUSTER3_WIN0_AFBCD_TRANSFORM_OFFSET	0x163C
#define RK3588_CLUSTER3_WIN0_AFBCD_OUTPUT_CTRL	0x1650
#define RK3588_CLUSTER3_WIN0_AFBCD_ROTATE_MODE	0x1654
#define RK3588_CLUSTER3_WIN0_AFBCD_HDR_PTR	0x1658
#define RK3588_CLUSTER3_WIN0_AFBCD_VIR_WIDTH	0x165C
#define RK3588_CLUSTER3_WIN0_AFBCD_PIC_SIZE	0x1660
#define RK3588_CLUSTER3_WIN0_AFBCD_PIC_OFFSET	0x1664
#define RK3588_CLUSTER3_WIN0_AFBCD_DSP_OFFSET	0x1668
#define RK3588_CLUSTER3_WIN0_AFBCD_CTRL		0x166C

#define RK3588_CLUSTER3_WIN1_CTRL0		0x1680
#define RK3588_CLUSTER3_WIN1_CTRL1		0x1684
#define RK3588_CLUSTER3_WIN1_YRGB_MST		0x1690
#define RK3588_CLUSTER3_WIN1_CBR_MST		0x1694
#define RK3588_CLUSTER3_WIN1_VIR		0x1698
#define RK3588_CLUSTER3_WIN1_ACT_INFO		0x16A0
#define RK3588_CLUSTER3_WIN1_DSP_INFO		0x16A4
#define RK3588_CLUSTER3_WIN1_DSP_ST		0x16A8
#define RK3588_CLUSTER3_WIN1_SCL_FACTOR_YRGB	0x16B0
#define RK3588_CLUSTER3_WIN1_AFBCD_OUTPUT_CTRL	0x16D0
#define RK3588_CLUSTER3_WIN1_AFBCD_ROTATE_MODE	0x16D4
#define RK3588_CLUSTER3_WIN1_AFBCD_HDR_PTR	0x16D8
#define RK3588_CLUSTER3_WIN1_AFBCD_VIR_WIDTH	0x16DC
#define RK3588_CLUSTER3_WIN1_AFBCD_PIC_SIZE	0x16E0
#define RK3588_CLUSTER3_WIN1_AFBCD_PIC_OFFSET	0x16E4
#define RK3588_CLUSTER3_WIN1_AFBCD_DSP_OFFSET	0x16E8
#define RK3588_CLUSTER3_WIN1_AFBCD_CTRL		0x16EC

#define RK3588_CLUSTER3_CTRL			0x1700

/* Esmart register definition */
#define RK3568_ESMART0_CTRL0			0x1800
#define RK3568_ESMART0_CTRL1			0x1804
#define RK3568_ESMART0_AXI_CTRL			0x1808
#define RK3568_ESMART0_REGION0_CTRL		0x1810
#define RK3568_ESMART0_REGION0_YRGB_MST		0x1814
#define RK3568_ESMART0_REGION0_CBR_MST		0x1818
#define RK3568_ESMART0_REGION0_VIR		0x181C
#define RK3568_ESMART0_REGION0_ACT_INFO		0x1820
#define RK3568_ESMART0_REGION0_DSP_INFO		0x1824
#define RK3568_ESMART0_REGION0_DSP_ST		0x1828
#define RK3568_ESMART0_REGION0_SCL_CTRL		0x1830
#define RK3568_ESMART0_REGION0_SCL_FACTOR_YRGB	0x1834
#define RK3568_ESMART0_REGION0_SCL_FACTOR_CBR	0x1838
#define RK3568_ESMART0_REGION0_SCL_OFFSET	0x183C
#define RK3568_ESMART0_REGION1_CTRL		0x1840
#define RK3568_ESMART0_REGION1_YRGB_MST		0x1844
#define RK3568_ESMART0_REGION1_CBR_MST		0x1848
#define RK3568_ESMART0_REGION1_VIR		0x184C
#define RK3568_ESMART0_REGION1_ACT_INFO		0x1850
#define RK3568_ESMART0_REGION1_DSP_INFO		0x1854
#define RK3568_ESMART0_REGION1_DSP_ST		0x1858
#define RK3568_ESMART0_REGION1_SCL_CTRL		0x1860
#define RK3568_ESMART0_REGION1_SCL_FACTOR_YRGB	0x1864
#define RK3568_ESMART0_REGION1_SCL_FACTOR_CBR	0x1868
#define RK3568_ESMART0_REGION1_SCL_OFFSET	0x186C
#define RK3568_ESMART0_REGION2_CTRL		0x1870
#define RK3568_ESMART0_REGION2_YRGB_MST		0x1874
#define RK3568_ESMART0_REGION2_CBR_MST		0x1878
#define RK3568_ESMART0_REGION2_VIR		0x187C
#define RK3568_ESMART0_REGION2_ACT_INFO		0x1880
#define RK3568_ESMART0_REGION2_DSP_INFO		0x1884
#define RK3568_ESMART0_REGION2_DSP_ST		0x1888
#define RK3568_ESMART0_REGION2_SCL_CTRL		0x1890
#define RK3568_ESMART0_REGION2_SCL_FACTOR_YRGB	0x1894
#define RK3568_ESMART0_REGION2_SCL_FACTOR_CBR	0x1898
#define RK3568_ESMART0_REGION2_SCL_OFFSET	0x189C
#define RK3568_ESMART0_REGION3_CTRL		0x18A0
#define RK3568_ESMART0_REGION3_YRGB_MST		0x18A4
#define RK3568_ESMART0_REGION3_CBR_MST		0x18A8
#define RK3568_ESMART0_REGION3_VIR		0x18AC
#define RK3568_ESMART0_REGION3_ACT_INFO		0x18B0
#define RK3568_ESMART0_REGION3_DSP_INFO		0x18B4
#define RK3568_ESMART0_REGION3_DSP_ST		0x18B8
#define RK3568_ESMART0_REGION3_SCL_CTRL		0x18C0
#define RK3568_ESMART0_REGION3_SCL_FACTOR_YRGB	0x18C4
#define RK3568_ESMART0_REGION3_SCL_FACTOR_CBR	0x18C8
#define RK3568_ESMART0_REGION3_SCL_OFFSET	0x18CC
#define RK3568_ESMART0_COLOR_KEY_CTRL		0x18D0

#define RK3568_ESMART1_CTRL0			0x1A00
#define RK3568_ESMART1_CTRL1			0x1A04
#define RK3568_ESMART1_REGION0_CTRL		0x1A10
#define RK3568_ESMART1_REGION0_YRGB_MST		0x1A14
#define RK3568_ESMART1_REGION0_CBR_MST		0x1A18
#define RK3568_ESMART1_REGION0_VIR		0x1A1C
#define RK3568_ESMART1_REGION0_ACT_INFO		0x1A20
#define RK3568_ESMART1_REGION0_DSP_INFO		0x1A24
#define RK3568_ESMART1_REGION0_DSP_ST		0x1A28
#define RK3568_ESMART1_REGION0_SCL_CTRL		0x1A30
#define RK3568_ESMART1_REGION0_SCL_FACTOR_YRGB	0x1A34
#define RK3568_ESMART1_REGION0_SCL_FACTOR_CBR	0x1A38
#define RK3568_ESMART1_REGION0_SCL_OFFSET	0x1A3C
#define RK3568_ESMART1_REGION1_CTRL		0x1A40
#define RK3568_ESMART1_REGION1_YRGB_MST		0x1A44
#define RK3568_ESMART1_REGION1_CBR_MST		0x1A48
#define RK3568_ESMART1_REGION1_VIR		0x1A4C
#define RK3568_ESMART1_REGION1_ACT_INFO		0x1A50
#define RK3568_ESMART1_REGION1_DSP_INFO		0x1A54
#define RK3568_ESMART1_REGION1_DSP_ST		0x1A58
#define RK3568_ESMART1_REGION1_SCL_CTRL		0x1A60
#define RK3568_ESMART1_REGION1_SCL_FACTOR_YRGB	0x1A64
#define RK3568_ESMART1_REGION1_SCL_FACTOR_CBR	0x1A68
#define RK3568_ESMART1_REGION1_SCL_OFFSET	0x1A6C
#define RK3568_ESMART1_REGION2_CTRL		0x1A70
#define RK3568_ESMART1_REGION2_YRGB_MST		0x1A74
#define RK3568_ESMART1_REGION2_CBR_MST		0x1A78
#define RK3568_ESMART1_REGION2_VIR		0x1A7C
#define RK3568_ESMART1_REGION2_ACT_INFO		0x1A80
#define RK3568_ESMART1_REGION2_DSP_INFO		0x1A84
#define RK3568_ESMART1_REGION2_DSP_ST		0x1A88
#define RK3568_ESMART1_REGION2_SCL_CTRL		0x1A90
#define RK3568_ESMART1_REGION2_SCL_FACTOR_YRGB	0x1A94
#define RK3568_ESMART1_REGION2_SCL_FACTOR_CBR	0x1A98
#define RK3568_ESMART1_REGION2_SCL_OFFSET	0x1A9C
#define RK3568_ESMART1_REGION3_CTRL		0x1AA0
#define RK3568_ESMART1_REGION3_YRGB_MST		0x1AA4
#define RK3568_ESMART1_REGION3_CBR_MST		0x1AA8
#define RK3568_ESMART1_REGION3_VIR		0x1AAC
#define RK3568_ESMART1_REGION3_ACT_INFO		0x1AB0
#define RK3568_ESMART1_REGION3_DSP_INFO		0x1AB4
#define RK3568_ESMART1_REGION3_DSP_ST		0x1AB8
#define RK3568_ESMART1_REGION3_SCL_CTRL		0x1AC0
#define RK3568_ESMART1_REGION3_SCL_FACTOR_YRGB	0x1AC4
#define RK3568_ESMART1_REGION3_SCL_FACTOR_CBR	0x1AC8
#define RK3568_ESMART1_REGION3_SCL_OFFSET	0x1ACC

#define RK3568_SMART0_CTRL0			0x1C00
#define RK3568_SMART0_CTRL1			0x1C04
#define RK3568_SMART0_REGION0_CTRL		0x1C10
#define RK3568_SMART0_REGION0_YRGB_MST		0x1C14
#define RK3568_SMART0_REGION0_CBR_MST		0x1C18
#define RK3568_SMART0_REGION0_VIR		0x1C1C
#define RK3568_SMART0_REGION0_ACT_INFO		0x1C20
#define RK3568_SMART0_REGION0_DSP_INFO		0x1C24
#define RK3568_SMART0_REGION0_DSP_ST		0x1C28
#define RK3568_SMART0_REGION0_SCL_CTRL		0x1C30
#define RK3568_SMART0_REGION0_SCL_FACTOR_YRGB	0x1C34
#define RK3568_SMART0_REGION0_SCL_FACTOR_CBR	0x1C38
#define RK3568_SMART0_REGION0_SCL_OFFSET	0x1C3C
#define RK3568_SMART0_REGION1_CTRL		0x1C40
#define RK3568_SMART0_REGION1_YRGB_MST		0x1C44
#define RK3568_SMART0_REGION1_CBR_MST		0x1C48
#define RK3568_SMART0_REGION1_VIR		0x1C4C
#define RK3568_SMART0_REGION1_ACT_INFO		0x1C50
#define RK3568_SMART0_REGION1_DSP_INFO		0x1C54
#define RK3568_SMART0_REGION1_DSP_ST		0x1C58
#define RK3568_SMART0_REGION1_SCL_CTRL		0x1C60
#define RK3568_SMART0_REGION1_SCL_FACTOR_YRGB	0x1C64
#define RK3568_SMART0_REGION1_SCL_FACTOR_CBR	0x1C68
#define RK3568_SMART0_REGION1_SCL_OFFSET	0x1C6C
#define RK3568_SMART0_REGION2_CTRL		0x1C70
#define RK3568_SMART0_REGION2_YRGB_MST		0x1C74
#define RK3568_SMART0_REGION2_CBR_MST		0x1C78
#define RK3568_SMART0_REGION2_VIR		0x1C7C
#define RK3568_SMART0_REGION2_ACT_INFO		0x1C80
#define RK3568_SMART0_REGION2_DSP_INFO		0x1C84
#define RK3568_SMART0_REGION2_DSP_ST		0x1C88
#define RK3568_SMART0_REGION2_SCL_CTRL		0x1C90
#define RK3568_SMART0_REGION2_SCL_FACTOR_YRGB	0x1C94
#define RK3568_SMART0_REGION2_SCL_FACTOR_CBR	0x1C98
#define RK3568_SMART0_REGION2_SCL_OFFSET	0x1C9C
#define RK3568_SMART0_REGION3_CTRL		0x1CA0
#define RK3568_SMART0_REGION3_YRGB_MST		0x1CA4
#define RK3568_SMART0_REGION3_CBR_MST		0x1CA8
#define RK3568_SMART0_REGION3_VIR		0x1CAC
#define RK3568_SMART0_REGION3_ACT_INFO		0x1CB0
#define RK3568_SMART0_REGION3_DSP_INFO		0x1CB4
#define RK3568_SMART0_REGION3_DSP_ST		0x1CB8
#define RK3568_SMART0_REGION3_SCL_CTRL		0x1CC0
#define RK3568_SMART0_REGION3_SCL_FACTOR_YRGB	0x1CC4
#define RK3568_SMART0_REGION3_SCL_FACTOR_CBR	0x1CC8
#define RK3568_SMART0_REGION3_SCL_OFFSET	0x1CCC

#define RK3568_SMART1_CTRL0			0x1E00
#define RK3568_SMART1_CTRL1			0x1E04
#define RK3568_SMART1_REGION0_CTRL		0x1E10
#define RK3568_SMART1_REGION0_YRGB_MST		0x1E14
#define RK3568_SMART1_REGION0_CBR_MST		0x1E18
#define RK3568_SMART1_REGION0_VIR		0x1E1C
#define RK3568_SMART1_REGION0_ACT_INFO		0x1E20
#define RK3568_SMART1_REGION0_DSP_INFO		0x1E24
#define RK3568_SMART1_REGION0_DSP_ST		0x1E28
#define RK3568_SMART1_REGION0_SCL_CTRL		0x1E30
#define RK3568_SMART1_REGION0_SCL_FACTOR_YRGB	0x1E34
#define RK3568_SMART1_REGION0_SCL_FACTOR_CBR	0x1E38
#define RK3568_SMART1_REGION0_SCL_OFFSET	0x1E3C
#define RK3568_SMART1_REGION1_CTRL		0x1E40
#define RK3568_SMART1_REGION1_YRGB_MST		0x1E44
#define RK3568_SMART1_REGION1_CBR_MST		0x1E48
#define RK3568_SMART1_REGION1_VIR		0x1E4C
#define RK3568_SMART1_REGION1_ACT_INFO		0x1E50
#define RK3568_SMART1_REGION1_DSP_INFO		0x1E54
#define RK3568_SMART1_REGION1_DSP_ST		0x1E58
#define RK3568_SMART1_REGION1_SCL_CTRL		0x1E60
#define RK3568_SMART1_REGION1_SCL_FACTOR_YRGB	0x1E64
#define RK3568_SMART1_REGION1_SCL_FACTOR_CBR	0x1E68
#define RK3568_SMART1_REGION1_SCL_OFFSET	0x1E6C
#define RK3568_SMART1_REGION2_CTRL		0x1E70
#define RK3568_SMART1_REGION2_YRGB_MST		0x1E74
#define RK3568_SMART1_REGION2_CBR_MST		0x1E78
#define RK3568_SMART1_REGION2_VIR		0x1E7C
#define RK3568_SMART1_REGION2_ACT_INFO		0x1E80
#define RK3568_SMART1_REGION2_DSP_INFO		0x1E84
#define RK3568_SMART1_REGION2_DSP_ST		0x1E88
#define RK3568_SMART1_REGION2_SCL_CTRL		0x1E90
#define RK3568_SMART1_REGION2_SCL_FACTOR_YRGB	0x1E94
#define RK3568_SMART1_REGION2_SCL_FACTOR_CBR	0x1E98
#define RK3568_SMART1_REGION2_SCL_OFFSET	0x1E9C
#define RK3568_SMART1_REGION3_CTRL		0x1EA0
#define RK3568_SMART1_REGION3_YRGB_MST		0x1EA4
#define RK3568_SMART1_REGION3_CBR_MST		0x1EA8
#define RK3568_SMART1_REGION3_VIR		0x1EAC
#define RK3568_SMART1_REGION3_ACT_INFO		0x1EB0
#define RK3568_SMART1_REGION3_DSP_INFO		0x1EB4
#define RK3568_SMART1_REGION3_DSP_ST		0x1EB8
#define RK3568_SMART1_REGION3_SCL_CTRL		0x1EC0
#define RK3568_SMART1_REGION3_SCL_FACTOR_YRGB	0x1EC4
#define RK3568_SMART1_REGION3_SCL_FACTOR_CBR	0x1EC8
#define RK3568_SMART1_REGION3_SCL_OFFSET	0x1ECC

/* HDR register definition */
#define RK3568_HDR_LUT_CTRL			0x2000
#define RK3568_HDR_LUT_MST			0x2004
#define RK3568_SDR2HDR_CTRL			0x2010
/* for HDR10 controller1 */
#define RK3568_SDR2HDR_CTRL1				0x2018
#define RK3568_HDR2SDR_CTRL1				0x201C
#define RK3568_HDR2SDR_CTRL			0x2020
#define RK3568_HDR2SDR_SRC_RANGE		0x2024
#define RK3568_HDR2SDR_NORMFACEETF		0x2028
#define RK3568_HDR2SDR_DST_RANGE		0x202C
#define RK3568_HDR2SDR_NORMFACCGAMMA		0x2030
#define RK3568_HDR_EETF_OETF_Y0			0x203C
#define RK3568_HDR_SAT_Y0			0x20C0
#define RK3568_HDR_EOTF_OETF_Y0			0x20F0
#define RK3568_HDR_OETF_DX_POW1			0x2200
#define RK3568_HDR_OETF_XN1			0x2300

/* DSC register definition */
#define RK3588_DSC_8K_PPS0_3				0x4000
#define RK3588_DSC_8K_CTRL0				0x40A0
#define RK3588_DSC_8K_CTRL1				0x40A4
#define RK3588_DSC_8K_STS0				0x40A8
#define RK3588_DSC_8K_ERS				0x40C4

#define RK3588_DSC_4K_PPS0_3				0x4100
#define RK3588_DSC_4K_CTRL0				0x41A0
#define RK3588_DSC_4K_CTRL1				0x41A4
#define RK3588_DSC_4K_STS0				0x41A8
#define RK3588_DSC_4K_ERS				0x41C4

#define RK3588_GRF_SOC_CON1				0x0304
#define RK3588_GRF_VOP_CON2				0x08
#define RK3588_GRF_VO1_CON0				0x00


#define RK3588_PMU_PWR_GATE_CON1			0x150
#define RK3588_PMU_SUBMEM_PWR_GATE_CON1			0x1B4
#define RK3588_PMU_SUBMEM_PWR_GATE_CON2			0x1B8
#define RK3588_PMU_SUBMEM_PWR_GATE_STATUS		0x1BC
#define RK3588_PMU_BISR_CON3				0x20C
#define RK3588_PMU_BISR_STATUS5				0x294

#define RK3568_REG_CFG_DONE__GLB_CFG_DONE_EN		BIT(15)

#define RK3568_VP_DSP_CTRL__STANDBY			BIT(31)
#define RK3568_VP_DSP_CTRL__DITHER_DOWN_MODE		BIT(20)
#define RK3568_VP_DSP_CTRL__DITHER_DOWN_SEL		GENMASK(19, 18)
#define RK3568_VP_DSP_CTRL__DITHER_DOWN_EN		BIT(17)
#define RK3568_VP_DSP_CTRL__PRE_DITHER_DOWN_EN		BIT(16)
#define RK3568_VP_DSP_CTRL__POST_DSP_OUT_R2Y		BIT(15)
#define RK3568_VP_DSP_CTRL__DSP_RB_SWAP			BIT(9)
#define RK3568_VP_DSP_CTRL__DSP_INTERLACE		BIT(7)
#define RK3568_VP_DSP_CTRL__DSP_FILED_POL		BIT(6)
#define RK3568_VP_DSP_CTRL__P2I_EN			BIT(5)
#define RK3568_VP_DSP_CTRL__CORE_DCLK_DIV		BIT(4)
#define RK3568_VP_DSP_CTRL__OUT_MODE			GENMASK(3, 0)

#define RK3568_VP_POST_SCL_CTRL__VSCALEDOWN		BIT(1)
#define RK3568_VP_POST_SCL_CTRL__HSCALEDOWN		BIT(0)

#define RK3568_SYS_DSP_INFACE_EN_LVDS1_MUX		GENMASK(26, 25)
#define RK3568_SYS_DSP_INFACE_EN_LVDS1			BIT(24)
#define RK3568_SYS_DSP_INFACE_EN_MIPI1_MUX		GENMASK(22, 21)
#define RK3568_SYS_DSP_INFACE_EN_MIPI1			BIT(20)
#define RK3568_SYS_DSP_INFACE_EN_LVDS0_MUX		GENMASK(19, 18)
#define RK3568_SYS_DSP_INFACE_EN_MIPI0_MUX		GENMASK(17, 16)
#define RK3568_SYS_DSP_INFACE_EN_EDP_MUX		GENMASK(15, 14)
#define RK3568_SYS_DSP_INFACE_EN_HDMI_MUX		GENMASK(11, 10)
#define RK3568_SYS_DSP_INFACE_EN_RGB_MUX		GENMASK(9, 8)
#define RK3568_SYS_DSP_INFACE_EN_LVDS0			BIT(5)
#define RK3568_SYS_DSP_INFACE_EN_MIPI0			BIT(4)
#define RK3568_SYS_DSP_INFACE_EN_EDP			BIT(3)
#define RK3568_SYS_DSP_INFACE_EN_HDMI			BIT(1)
#define RK3568_SYS_DSP_INFACE_EN_RGB			BIT(0)

#define RK3588_SYS_DSP_INFACE_EN_HDMI_MUX		GENMASK(17, 16)
#define RK3588_SYS_DSP_INFACE_EN_HDMI0			BIT(3)
#define RK3588_SYS_DSP_INFACE_EN_EDP0			BIT(2)

#define RK3568_DSP_IF_POL__MIPI_PIN_POL			GENMASK(19, 16)
#define RK3568_DSP_IF_POL__EDP_PIN_POL			GENMASK(15, 12)
#define RK3568_DSP_IF_POL__HDMI_PIN_POL			GENMASK(7, 4)
#define RK3568_DSP_IF_POL__RGB_LVDS_PIN_POL		GENMASK(3, 0)

#define RK3568_VP0_MIPI_CTRL__DCLK_DIV2_PHASE_LOCK	BIT(5)
#define RK3568_VP0_MIPI_CTRL__DCLK_DIV2			BIT(4)

#define RK3568_SYS_AUTO_GATING_CTRL__AUTO_GATING_EN	BIT(31)

#define RK3568_DSP_IF_POL__CFG_DONE_IMD			BIT(28)

#define VOP2_SYS_AXI_BUS_NUM				2

#define VOP2_CLUSTER_YUV444_10				0x12

#define VOP2_COLOR_KEY_MASK				BIT(31)

#define RK3568_OVL_CTRL__LAYERSEL_REGDONE_IMD		BIT(28)

#define RK3568_VP_BG_MIX_CTRL__BG_DLY			GENMASK(31, 24)

#define RK3568_OVL_PORT_SEL__SEL_PORT			GENMASK(31, 16)
#define RK3568_OVL_PORT_SEL__SMART1			GENMASK(31, 30)
#define RK3568_OVL_PORT_SEL__SMART0			GENMASK(29, 28)
#define RK3568_OVL_PORT_SEL__ESMART1			GENMASK(27, 26)
#define RK3568_OVL_PORT_SEL__ESMART0			GENMASK(25, 24)
#define RK3568_OVL_PORT_SEL__CLUSTER1			GENMASK(19, 18)
#define RK3568_OVL_PORT_SEL__CLUSTER0			GENMASK(17, 16)
#define RK3568_OVL_PORT_SET__PORT2_MUX			GENMASK(11, 8)
#define RK3568_OVL_PORT_SET__PORT1_MUX			GENMASK(7, 4)
#define RK3568_OVL_PORT_SET__PORT0_MUX			GENMASK(3, 0)
#define RK3568_OVL_LAYER_SEL__LAYER(layer, x)		((x) << ((layer) * 4))

#define RK3568_CLUSTER_DLY_NUM__CLUSTER1_1		GENMASK(31, 24)
#define RK3568_CLUSTER_DLY_NUM__CLUSTER1_0		GENMASK(23, 16)
#define RK3568_CLUSTER_DLY_NUM__CLUSTER0_1		GENMASK(15, 8)
#define RK3568_CLUSTER_DLY_NUM__CLUSTER0_0		GENMASK(7, 0)

#define RK3568_SMART_DLY_NUM__SMART1			GENMASK(31, 24)
#define RK3568_SMART_DLY_NUM__SMART0			GENMASK(23, 16)
#define RK3568_SMART_DLY_NUM__ESMART1			GENMASK(15, 8)
#define RK3568_SMART_DLY_NUM__ESMART0			GENMASK(7, 0)

#define VP_INT_DSP_HOLD_VALID	BIT(6)
#define VP_INT_FS_FIELD		BIT(5)
#define VP_INT_POST_BUF_EMPTY	BIT(4)
#define VP_INT_LINE_FLAG1	BIT(3)
#define VP_INT_LINE_FLAG0	BIT(2)
#define VOP2_INT_BUS_ERRPR	BIT(1)
#define VP_INT_FS		BIT(0)

#define POLFLAG_DCLK_INV	BIT(3)

extern const struct component_ops vop2_component_ops;

#endif /* _ROCKCHIP_DRM_VOP2_H */
