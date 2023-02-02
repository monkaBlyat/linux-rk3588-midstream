// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 *
 * based on exynos_drm_drv.c
 */

#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/component.h>
#include <linux/console.h>
#include <linux/iommu.h>
#include <linux/clk.h>

#include <drm/drm_aperture.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_displayid.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#if defined(CONFIG_ARM_DMA_USE_IOMMU)
#include <asm/dma-iommu.h>
#else
#define arm_iommu_detach_device(...)	({ })
#define arm_iommu_release_mapping(...)	({ })
#define to_dma_iommu_mapping(dev) NULL
#endif

#include "rockchip_drm_drv.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_gem.h"

#define DRIVER_NAME	"rockchip"
#define DRIVER_DESC	"RockChip Soc DRM"
#define DRIVER_DATE	"20140818"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static const struct drm_driver rockchip_drm_driver;

void drm_mode_convert_to_split_mode(struct drm_display_mode *mode)
{
	u16 hactive, hfp, hsync, hbp;

	hactive = mode->hdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	mode->clock *= 2;
	mode->hdisplay = hactive * 2;
	mode->hsync_start = mode->hdisplay + hfp * 2;
	mode->hsync_end = mode->hsync_start + hsync * 2;
	mode->htotal = mode->hsync_end + hbp * 2;
	drm_mode_set_name(mode);
}
EXPORT_SYMBOL(drm_mode_convert_to_split_mode);

void drm_mode_convert_to_origin_mode(struct drm_display_mode *mode)
{
	u16 hactive, hfp, hsync, hbp;

	hactive = mode->hdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	mode->clock /= 2;
	mode->hdisplay = hactive / 2;
	mode->hsync_start = mode->hdisplay + hfp / 2;
	mode->hsync_end = mode->hsync_start + hsync / 2;
	mode->htotal = mode->hsync_end + hbp / 2;
}
EXPORT_SYMBOL(drm_mode_convert_to_origin_mode);

uint32_t rockchip_drm_get_bpp(const struct drm_format_info *info)
{
	/* use whatever a driver has set */
	if (info->cpp[0])
		return info->cpp[0] * 8;

	switch (info->format) {
	case DRM_FORMAT_YUV420_8BIT:
		return 12;
	case DRM_FORMAT_YUV420_10BIT:
		return 15;
	case DRM_FORMAT_VUY101010:
		return 30;
	default:
		break;
	}

	/* all attempts failed */
	return 0;
}
EXPORT_SYMBOL(rockchip_drm_get_bpp);

/**
 * rockchip_drm_of_find_possible_crtcs - find the possible CRTCs for an active
 * encoder port
 * @dev: DRM device
 * @port: encoder port to scan for endpoints
 *
 * Scan all active endpoints attached to a port, locate their attached CRTCs,
 * and generate the DRM mask of CRTCs which may be attached to this
 * encoder.
 *
 * See Documentation/devicetree/bindings/graph.txt for the bindings.
 */
uint32_t rockchip_drm_of_find_possible_crtcs(struct drm_device *dev,
					     struct device_node *port)
{
	struct device_node *remote_port, *ep;
	uint32_t possible_crtcs = 0;

	for_each_endpoint_of_node(port, ep) {
		if (!of_device_is_available(ep))
			continue;

		remote_port = of_graph_get_remote_port(ep);
		if (!remote_port) {
			of_node_put(ep);
			return 0;
		}

		possible_crtcs |= drm_of_crtc_port_mask(dev, remote_port);

		of_node_put(remote_port);
	}

	return possible_crtcs;
}
EXPORT_SYMBOL(rockchip_drm_of_find_possible_crtcs);

static DEFINE_MUTEX(rockchip_drm_sub_dev_lock);
static LIST_HEAD(rockchip_drm_sub_dev_list);

void rockchip_drm_register_sub_dev(struct rockchip_drm_sub_dev *sub_dev)
{
	mutex_lock(&rockchip_drm_sub_dev_lock);
	list_add_tail(&sub_dev->list, &rockchip_drm_sub_dev_list);
	mutex_unlock(&rockchip_drm_sub_dev_lock);
}
EXPORT_SYMBOL(rockchip_drm_register_sub_dev);

void rockchip_drm_unregister_sub_dev(struct rockchip_drm_sub_dev *sub_dev)
{
	mutex_lock(&rockchip_drm_sub_dev_lock);
	list_del(&sub_dev->list);
	mutex_unlock(&rockchip_drm_sub_dev_lock);
}
EXPORT_SYMBOL(rockchip_drm_unregister_sub_dev);

struct rockchip_drm_sub_dev *rockchip_drm_get_sub_dev(struct device_node *node)
{
	struct rockchip_drm_sub_dev *sub_dev = NULL;
	bool found = false;

	mutex_lock(&rockchip_drm_sub_dev_lock);
	list_for_each_entry(sub_dev, &rockchip_drm_sub_dev_list, list) {
		if (sub_dev->of_node == node) {
			found = true;
			break;
		}
	}
	mutex_unlock(&rockchip_drm_sub_dev_lock);

	return found ? sub_dev : NULL;
}
EXPORT_SYMBOL(rockchip_drm_get_sub_dev);

int rockchip_drm_get_sub_dev_type(void)
{
	int connector_type = DRM_MODE_CONNECTOR_Unknown;
	struct rockchip_drm_sub_dev *sub_dev = NULL;

	mutex_lock(&rockchip_drm_sub_dev_lock);
	list_for_each_entry(sub_dev, &rockchip_drm_sub_dev_list, list) {
		if (sub_dev->connector->encoder) {
			connector_type = sub_dev->connector->connector_type;
			break;
		}
	}
	mutex_unlock(&rockchip_drm_sub_dev_lock);

	return connector_type;
}
EXPORT_SYMBOL(rockchip_drm_get_sub_dev_type);

static int
cea_db_tag(const u8 *db)
{
	return db[0] >> 5;
}

static int
cea_db_payload_len(const u8 *db)
{
	return db[0] & 0x1f;
}

#define for_each_cea_db(cea, i, start, end) \
	for ((i) = (start); \
	     (i) < (end) && (i) + cea_db_payload_len(&(cea)[(i)]) < (end); \
	     (i) += cea_db_payload_len(&(cea)[(i)]) + 1)

#define HDMI_NEXT_HDR_VSDB_OUI 0xd04601

static bool cea_db_is_hdmi_next_hdr_block(const u8 *db)
{
	unsigned int oui;

	if (cea_db_tag(db) != 0x07)
		return false;

	if (cea_db_payload_len(db) < 11)
		return false;

	oui = db[3] << 16 | db[2] << 8 | db[1];

	return oui == HDMI_NEXT_HDR_VSDB_OUI;
}

static bool cea_db_is_hdmi_forum_vsdb(const u8 *db)
{
	unsigned int oui;

	if (cea_db_tag(db) != 0x03)
		return false;

	if (cea_db_payload_len(db) < 7)
		return false;

	oui = db[3] << 16 | db[2] << 8 | db[1];

	return oui == HDMI_FORUM_IEEE_OUI;
}

static int
cea_db_offsets(const u8 *cea, int *start, int *end)
{
	/* DisplayID CTA extension blocks and top-level CEA EDID
	 * block header definitions differ in the following bytes:
	 *   1) Byte 2 of the header specifies length differently,
	 *   2) Byte 3 is only present in the CEA top level block.
	 *
	 * The different definitions for byte 2 follow.
	 *
	 * DisplayID CTA extension block defines byte 2 as:
	 *   Number of payload bytes
	 *
	 * CEA EDID block defines byte 2 as:
	 *   Byte number (decimal) within this block where the 18-byte
	 *   DTDs begin. If no non-DTD data is present in this extension
	 *   block, the value should be set to 04h (the byte after next).
	 *   If set to 00h, there are no DTDs present in this block and
	 *   no non-DTD data.
	 */
	if (cea[0] == 0x81) {
		/*
		 * for_each_displayid_db() has already verified
		 * that these stay within expected bounds.
		 */
		*start = 3;
		*end = *start + cea[2];
	} else if (cea[0] == 0x02) {
		/* Data block offset in CEA extension block */
		*start = 4;
		*end = cea[2];
		if (*end == 0)
			*end = 127;
		if (*end < 4 || *end > 127)
			return -ERANGE;
	} else {
		return -EOPNOTSUPP;
	}

	return 0;
}

static int validate_displayid(const u8 *displayid, int length, int idx)
{
	int i, dispid_length;
	u8 csum = 0;
	struct displayid_header *base;

	base = (struct displayid_header *)&displayid[idx];

	DRM_DEBUG_KMS("base revision 0x%x, length %d, %d %d\n",
		      base->rev, base->bytes, base->prod_id, base->ext_count);

	/* +1 for DispID checksum */
	dispid_length = sizeof(*base) + base->bytes + 1;
	if (dispid_length > length - idx)
		return -EINVAL;

	for (i = 0; i < dispid_length; i++)
		csum += displayid[idx + i];
	if (csum) {
		DRM_NOTE("DisplayID checksum invalid, remainder is %d\n", csum);
		return -EINVAL;
	}

	return 0;
}

static const u8 *find_cea_extension(const struct drm_edid *edid)
{
	const struct displayid_block *block;
	struct displayid_iter iter;
	const u8 *cea;
	int ext_index = 0;

	/* Look for a top level CEA extension block */
	/* FIXME: make callers iterate through multiple CEA ext blocks? */
	cea = drm_find_edid_extension(edid, 0x02, &ext_index);
	if (cea)
		return cea;

	/* CEA blocks can also be found embedded in a DisplayID block */
	displayid_iter_edid_begin(edid, &iter);
	displayid_iter_for_each(block, &iter) {
		if (block->tag == DATA_BLOCK_CTA) {
			cea = (const u8 *)block;
			break;
		}
	}
	displayid_iter_end(&iter);

	return cea;
}

#define EDID_CEA_YCBCR422	(1 << 4)

int rockchip_drm_get_yuv422_format(struct drm_connector *connector,
				   struct drm_edid *edid)
{
	struct drm_display_info *info;
	const u8 *edid_ext;

	if (!connector || !edid)
		return -EINVAL;

	info = &connector->display_info;

	edid_ext = find_cea_extension(edid);
	if (!edid_ext)
		return -EINVAL;

	if (edid_ext[3] & EDID_CEA_YCBCR422)
		info->color_formats |= DRM_COLOR_FORMAT_YCBCR422;

	return 0;
}
EXPORT_SYMBOL(rockchip_drm_get_yuv422_format);

static
void get_max_frl_rate(int max_frl_rate, u8 *max_lanes, u8 *max_rate_per_lane)
{
	switch (max_frl_rate) {
	case 1:
		*max_lanes = 3;
		*max_rate_per_lane = 3;
		break;
	case 2:
		*max_lanes = 3;
		*max_rate_per_lane = 6;
		break;
	case 3:
		*max_lanes = 4;
		*max_rate_per_lane = 6;
		break;
	case 4:
		*max_lanes = 4;
		*max_rate_per_lane = 8;
		break;
	case 5:
		*max_lanes = 4;
		*max_rate_per_lane = 10;
		break;
	case 6:
		*max_lanes = 4;
		*max_rate_per_lane = 12;
		break;
	case 0:
	default:
		*max_lanes = 0;
		*max_rate_per_lane = 0;
	}
}

#define EDID_DSC_10BPC			(1 << 0)
#define EDID_DSC_12BPC			(1 << 1)
#define EDID_DSC_16BPC			(1 << 2)
#define EDID_DSC_ALL_BPP		(1 << 3)
#define EDID_DSC_NATIVE_420		(1 << 6)
#define EDID_DSC_1P2			(1 << 7)
#define EDID_DSC_MAX_FRL_RATE_MASK	0xf0
#define EDID_DSC_MAX_SLICES		0xf
#define EDID_DSC_TOTAL_CHUNK_KBYTES	0x3f
#define EDID_MAX_FRL_RATE_MASK		0xf0

static
void parse_edid_forum_vsdb(struct rockchip_drm_dsc_cap *dsc_cap,
			   u8 *max_frl_rate_per_lane, u8 *max_lanes,
			   const u8 *hf_vsdb)
{
	u8 max_frl_rate;
	u8 dsc_max_frl_rate;
	u8 dsc_max_slices;

	if (!hf_vsdb[7])
		return;

	DRM_DEBUG_KMS("hdmi_21 sink detected. parsing edid\n");
	max_frl_rate = (hf_vsdb[7] & EDID_MAX_FRL_RATE_MASK) >> 4;
	get_max_frl_rate(max_frl_rate, max_lanes,
			 max_frl_rate_per_lane);

	if (cea_db_payload_len(hf_vsdb) < 13)
		return;

	dsc_cap->v_1p2 = hf_vsdb[11] & EDID_DSC_1P2;

	if (!dsc_cap->v_1p2)
		return;

	dsc_cap->native_420 = hf_vsdb[11] & EDID_DSC_NATIVE_420;
	dsc_cap->all_bpp = hf_vsdb[11] & EDID_DSC_ALL_BPP;

	if (hf_vsdb[11] & EDID_DSC_16BPC)
		dsc_cap->bpc_supported = 16;
	else if (hf_vsdb[11] & EDID_DSC_12BPC)
		dsc_cap->bpc_supported = 12;
	else if (hf_vsdb[11] & EDID_DSC_10BPC)
		dsc_cap->bpc_supported = 10;
	else
		dsc_cap->bpc_supported = 0;

	dsc_max_frl_rate = (hf_vsdb[12] & EDID_DSC_MAX_FRL_RATE_MASK) >> 4;
	get_max_frl_rate(dsc_max_frl_rate, &dsc_cap->max_lanes,
			 &dsc_cap->max_frl_rate_per_lane);
	dsc_cap->total_chunk_kbytes = hf_vsdb[13] & EDID_DSC_TOTAL_CHUNK_KBYTES;

	dsc_max_slices = hf_vsdb[12] & EDID_DSC_MAX_SLICES;
	switch (dsc_max_slices) {
	case 1:
		dsc_cap->max_slices = 1;
		dsc_cap->clk_per_slice = 340;
		break;
	case 2:
		dsc_cap->max_slices = 2;
		dsc_cap->clk_per_slice = 340;
		break;
	case 3:
		dsc_cap->max_slices = 4;
		dsc_cap->clk_per_slice = 340;
		break;
	case 4:
		dsc_cap->max_slices = 8;
		dsc_cap->clk_per_slice = 340;
		break;
	case 5:
		dsc_cap->max_slices = 8;
		dsc_cap->clk_per_slice = 400;
		break;
	case 6:
		dsc_cap->max_slices = 12;
		dsc_cap->clk_per_slice = 400;
		break;
	case 7:
		dsc_cap->max_slices = 16;
		dsc_cap->clk_per_slice = 400;
		break;
	case 0:
	default:
		dsc_cap->max_slices = 0;
		dsc_cap->clk_per_slice = 0;
	}
}

enum {
	VER_26_BYTE_V0,
	VER_15_BYTE_V1,
	VER_12_BYTE_V1,
	VER_12_BYTE_V2,
};

static int check_next_hdr_version(const u8 *next_hdr_db)
{
	u16 ver;

	ver = (next_hdr_db[5] & 0xf0) << 8 | next_hdr_db[0];

	switch (ver) {
	case 0x00f9:
		return VER_26_BYTE_V0;
	case 0x20ee:
		return VER_15_BYTE_V1;
	case 0x20eb:
		return VER_12_BYTE_V1;
	case 0x40eb:
		return VER_12_BYTE_V2;
	default:
		return -ENOENT;
	}
}

static void parse_ver_26_v0_data(struct ver_26_v0 *hdr, const u8 *data)
{
	hdr->yuv422_12bit = data[5] & BIT(0);
	hdr->support_2160p_60 = (data[5] & BIT(1)) >> 1;
	hdr->global_dimming = (data[5] & BIT(2)) >> 2;

	hdr->dm_major_ver = (data[21] & 0xf0) >> 4;
	hdr->dm_minor_ver = data[21] & 0xf;

	hdr->t_min_pq = (data[19] << 4) | ((data[18] & 0xf0) >> 4);
	hdr->t_max_pq = (data[20] << 4) | (data[18] & 0xf);

	hdr->rx = (data[7] << 4) | ((data[6] & 0xf0) >> 4);
	hdr->ry = (data[8] << 4) | (data[6] & 0xf);
	hdr->gx = (data[10] << 4) | ((data[9] & 0xf0) >> 4);
	hdr->gy = (data[11] << 4) | (data[9] & 0xf);
	hdr->bx = (data[13] << 4) | ((data[12] & 0xf0) >> 4);
	hdr->by = (data[14] << 4) | (data[12] & 0xf);
	hdr->wx = (data[16] << 4) | ((data[15] & 0xf0) >> 4);
	hdr->wy = (data[17] << 4) | (data[15] & 0xf);
}

static void parse_ver_15_v1_data(struct ver_15_v1 *hdr, const u8 *data)
{
	hdr->yuv422_12bit = data[5] & BIT(0);
	hdr->support_2160p_60 = (data[5] & BIT(1)) >> 1;
	hdr->global_dimming = data[6] & BIT(0);

	hdr->dm_version = (data[5] & 0x1c) >> 2;

	hdr->colorimetry = data[7] & BIT(0);

	hdr->t_max_lum = (data[6] & 0xfe) >> 1;
	hdr->t_min_lum = (data[7] & 0xfe) >> 1;

	hdr->rx = data[9];
	hdr->ry = data[10];
	hdr->gx = data[11];
	hdr->gy = data[12];
	hdr->bx = data[13];
	hdr->by = data[14];
}

static void parse_ver_12_v1_data(struct ver_12_v1 *hdr, const u8 *data)
{
	hdr->yuv422_12bit = data[5] & BIT(0);
	hdr->support_2160p_60 = (data[5] & BIT(1)) >> 1;
	hdr->global_dimming = data[6] & BIT(0);

	hdr->dm_version = (data[5] & 0x1c) >> 2;

	hdr->colorimetry = data[7] & BIT(0);

	hdr->t_max_lum = (data[6] & 0xfe) >> 1;
	hdr->t_min_lum = (data[7] & 0xfe) >> 1;

	hdr->low_latency = data[8] & 0x3;

	hdr->unique_rx = (data[11] & 0xf8) >> 3;
	hdr->unique_ry = (data[11] & 0x7) << 2 | (data[10] & BIT(0)) << 1 |
		(data[9] & BIT(0));
	hdr->unique_gx = (data[9] & 0xfe) >> 1;
	hdr->unique_gy = (data[10] & 0xfe) >> 1;
	hdr->unique_bx = (data[8] & 0xe0) >> 5;
	hdr->unique_by = (data[8] & 0x1c) >> 2;
}

static void parse_ver_12_v2_data(struct ver_12_v2 *hdr, const u8 *data)
{
	hdr->yuv422_12bit = data[5] & BIT(0);
	hdr->backlt_ctrl = (data[5] & BIT(1)) >> 1;
	hdr->global_dimming = (data[6] & BIT(2)) >> 2;

	hdr->dm_version = (data[5] & 0x1c) >> 2;
	hdr->backlt_min_luma = data[6] & 0x3;
	hdr->interface = data[7] & 0x3;
	hdr->yuv444_10b_12b = (data[8] & BIT(0)) << 1 | (data[9] & BIT(0));

	hdr->t_min_pq_v2 = (data[6] & 0xf8) >> 3;
	hdr->t_max_pq_v2 = (data[7] & 0xf8) >> 3;

	hdr->unique_rx = (data[10] & 0xf8) >> 3;
	hdr->unique_ry = (data[11] & 0xf8) >> 3;
	hdr->unique_gx = (data[8] & 0xfe) >> 1;
	hdr->unique_gy = (data[9] & 0xfe) >> 1;
	hdr->unique_bx = data[10] & 0x7;
	hdr->unique_by = data[11] & 0x7;
}

static
void parse_next_hdr_block(struct next_hdr_sink_data *sink_data,
			  const u8 *next_hdr_db)
{
	int version;

	version = check_next_hdr_version(next_hdr_db);
	if (version < 0)
		return;

	sink_data->version = version;

	switch (version) {
	case VER_26_BYTE_V0:
		parse_ver_26_v0_data(&sink_data->ver_26_v0, next_hdr_db);
		break;
	case VER_15_BYTE_V1:
		parse_ver_15_v1_data(&sink_data->ver_15_v1, next_hdr_db);
		break;
	case VER_12_BYTE_V1:
		parse_ver_12_v1_data(&sink_data->ver_12_v1, next_hdr_db);
		break;
	case VER_12_BYTE_V2:
		parse_ver_12_v2_data(&sink_data->ver_12_v2, next_hdr_db);
		break;
	default:
		break;
	}
}

int rockchip_drm_parse_cea_ext(struct rockchip_drm_dsc_cap *dsc_cap,
			       u8 *max_frl_rate_per_lane, u8 *max_lanes,
			       const struct drm_edid *edid)
{
	const u8 *edid_ext;
	int i, start, end;

	if (!dsc_cap || !max_frl_rate_per_lane || !max_lanes || !edid)
		return -EINVAL;

	edid_ext = find_cea_extension(edid);
	if (!edid_ext)
		return -EINVAL;

	if (cea_db_offsets(edid_ext, &start, &end))
		return -EINVAL;

	for_each_cea_db(edid_ext, i, start, end) {
		const u8 *db = &edid_ext[i];

		if (cea_db_is_hdmi_forum_vsdb(db))
			parse_edid_forum_vsdb(dsc_cap, max_frl_rate_per_lane,
					      max_lanes, db);
	}

	return 0;
}
EXPORT_SYMBOL(rockchip_drm_parse_cea_ext);

int rockchip_drm_parse_next_hdr(struct next_hdr_sink_data *sink_data,
				const struct drm_edid *edid)
{
	const u8 *edid_ext;
	int i, start, end;

	if (!sink_data || !edid)
		return -EINVAL;

	memset(sink_data, 0, sizeof(struct next_hdr_sink_data));

	edid_ext = find_cea_extension(edid);
	if (!edid_ext)
		return -EINVAL;

	if (cea_db_offsets(edid_ext, &start, &end))
		return -EINVAL;

	for_each_cea_db(edid_ext, i, start, end) {
		const u8 *db = &edid_ext[i];

		if (cea_db_is_hdmi_next_hdr_block(db))
			parse_next_hdr_block(sink_data, db);
	}

	return 0;
}
EXPORT_SYMBOL(rockchip_drm_parse_next_hdr);

/*
 * Attach a (component) device to the shared drm dma mapping from master drm
 * device.  This is used by the VOPs to map GEM buffers to a common DMA
 * mapping.
 */
int rockchip_drm_dma_attach_device(struct drm_device *drm_dev,
				   struct device *dev)
{
	struct rockchip_drm_private *private = drm_dev->dev_private;
	int ret;

	if (!private->domain)
		return 0;

	if (IS_ENABLED(CONFIG_ARM_DMA_USE_IOMMU)) {
		struct dma_iommu_mapping *mapping = to_dma_iommu_mapping(dev);

		if (mapping) {
			arm_iommu_detach_device(dev);
			arm_iommu_release_mapping(mapping);
		}
	}

	ret = iommu_attach_device(private->domain, dev);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to attach iommu device\n");
		return ret;
	}

	return 0;
}

void rockchip_drm_dma_detach_device(struct drm_device *drm_dev,
				    struct device *dev)
{
	struct rockchip_drm_private *private = drm_dev->dev_private;

	if (!private->domain)
		return;

	iommu_detach_device(private->domain, dev);
}

void rockchip_drm_dma_init_device(struct drm_device *drm_dev,
				  struct device *dev)
{
	struct rockchip_drm_private *private = drm_dev->dev_private;

	if (!device_iommu_mapped(dev))
		private->iommu_dev = ERR_PTR(-ENODEV);
	else if (!private->iommu_dev)
		private->iommu_dev = dev;
}

int rockchip_register_crtc_funcs(struct drm_crtc *crtc,
				 const struct rockchip_crtc_funcs *crtc_funcs)
{
	int pipe = drm_crtc_index(crtc);
	struct rockchip_drm_private *priv = crtc->dev->dev_private;

	if (pipe >= ROCKCHIP_MAX_CRTC)
		return -EINVAL;

	priv->crtc_funcs[pipe] = crtc_funcs;

	return 0;
}

void rockchip_unregister_crtc_funcs(struct drm_crtc *crtc)
{
	int pipe = drm_crtc_index(crtc);
	struct rockchip_drm_private *priv = crtc->dev->dev_private;

	if (pipe >= ROCKCHIP_MAX_CRTC)
		return;

	priv->crtc_funcs[pipe] = NULL;
}

static int rockchip_drm_fault_handler(struct iommu_domain *iommu,
				      struct device *dev,
				      unsigned long iova, int flags, void *arg)
{
	struct drm_device *drm_dev = arg;
	struct rockchip_drm_private *priv = drm_dev->dev_private;
	struct drm_crtc *crtc;

	DRM_ERROR("iommu fault handler flags: 0x%x\n", flags);
	drm_for_each_crtc(crtc, drm_dev) {
		int pipe = drm_crtc_index(crtc);

		if (priv->crtc_funcs[pipe] &&
		    priv->crtc_funcs[pipe]->regs_dump)
			priv->crtc_funcs[pipe]->regs_dump(crtc, NULL);

		if (priv->crtc_funcs[pipe] &&
		    priv->crtc_funcs[pipe]->debugfs_dump)
			priv->crtc_funcs[pipe]->debugfs_dump(crtc, NULL);
	}

	return 0;
}

static int rockchip_drm_init_iommu(struct drm_device *drm_dev)
{
	struct rockchip_drm_private *private = drm_dev->dev_private;
	struct iommu_domain_geometry *geometry;
	u64 start, end;

	if (IS_ERR_OR_NULL(private->iommu_dev))
		return 0;

	private->domain = iommu_domain_alloc(private->iommu_dev->bus);
	if (!private->domain)
		return -ENOMEM;

	geometry = &private->domain->geometry;
	start = geometry->aperture_start;
	end = geometry->aperture_end;

	DRM_DEBUG("IOMMU context initialized (aperture: %#llx-%#llx)\n",
		  start, end);
	drm_mm_init(&private->mm, start, end - start + 1);
	mutex_init(&private->mm_lock);

	iommu_set_fault_handler(private->domain, rockchip_drm_fault_handler,
				drm_dev);

	return 0;
}

static void rockchip_iommu_cleanup(struct drm_device *drm_dev)
{
	struct rockchip_drm_private *private = drm_dev->dev_private;

	if (!private->domain)
		return;

	drm_mm_takedown(&private->mm);
	iommu_domain_free(private->domain);
}

#ifdef CONFIG_DEBUG_FS
static int rockchip_drm_mm_dump(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct drm_minor *minor = node->minor;
	struct drm_device *drm_dev = minor->dev;
	struct rockchip_drm_private *priv = drm_dev->dev_private;
	struct drm_printer p = drm_seq_file_printer(s);

	if (!priv->domain)
		return 0;
	mutex_lock(&priv->mm_lock);
	drm_mm_print(&priv->mm, &p);
	mutex_unlock(&priv->mm_lock);

	return 0;
}

static int rockchip_drm_summary_show(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct drm_minor *minor = node->minor;
	struct drm_device *drm_dev = minor->dev;
	struct rockchip_drm_private *priv = drm_dev->dev_private;
	struct drm_crtc *crtc;

	drm_for_each_crtc(crtc, drm_dev) {
		int pipe = drm_crtc_index(crtc);

		if (priv->crtc_funcs[pipe] &&
		    priv->crtc_funcs[pipe]->debugfs_dump)
			priv->crtc_funcs[pipe]->debugfs_dump(crtc, s);
	}

	return 0;
}

static struct drm_info_list rockchip_debugfs_files[] = {
	{ "summary", rockchip_drm_summary_show, 0, NULL },
	{ "mm_dump", rockchip_drm_mm_dump, 0, NULL },
};

static void rockchip_drm_debugfs_init(struct drm_minor *minor)
{
	struct drm_device *dev = minor->dev;
	struct rockchip_drm_private *priv = dev->dev_private;
	struct drm_crtc *crtc;

	drm_debugfs_create_files(rockchip_debugfs_files,
				 ARRAY_SIZE(rockchip_debugfs_files),
				 minor->debugfs_root, minor);

	drm_for_each_crtc(crtc, dev) {
		int pipe = drm_crtc_index(crtc);

		if (priv->crtc_funcs[pipe] &&
		    priv->crtc_funcs[pipe]->debugfs_init)
			priv->crtc_funcs[pipe]->debugfs_init(minor, crtc);
	}
}
#endif

static int rockchip_drm_create_properties(struct drm_device *dev)
{
	struct drm_property *prop;
	struct rockchip_drm_private *private = dev->dev_private;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
					 "EOTF", 0, 5);
	if (!prop)
		return -ENOMEM;
	private->eotf_prop = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
					 "COLOR_SPACE", 0, 12);
	if (!prop)
		return -ENOMEM;
	private->color_space_prop = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
					 "ASYNC_COMMIT", 0, 1);
	if (!prop)
		return -ENOMEM;
	private->async_commit_prop = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC,
					 "SHARE_ID", 0, UINT_MAX);
	if (!prop)
		return -ENOMEM;
	private->share_id_prop = prop;

	prop = drm_property_create_range(dev, DRM_MODE_PROP_ATOMIC | DRM_MODE_PROP_IMMUTABLE,
					 "CONNECTOR_ID", 0, 0xf);
	if (!prop)
		return -ENOMEM;
	private->connector_id_prop = prop;

	prop = drm_property_create_object(dev,
					  DRM_MODE_PROP_ATOMIC | DRM_MODE_PROP_IMMUTABLE,
					  "SOC_ID", DRM_MODE_OBJECT_CRTC);
	private->soc_id_prop = prop;

	prop = drm_property_create_object(dev,
					  DRM_MODE_PROP_ATOMIC | DRM_MODE_PROP_IMMUTABLE,
					  "PORT_ID", DRM_MODE_OBJECT_CRTC);
	private->port_id_prop = prop;

	private->aclk_prop = drm_property_create_range(dev, 0, "ACLK", 0, UINT_MAX);
	private->bg_prop = drm_property_create_range(dev, 0, "BACKGROUND", 0, UINT_MAX);
	private->line_flag_prop = drm_property_create_range(dev, 0, "LINE_FLAG1", 0, UINT_MAX);

	return drm_mode_create_tv_properties(dev, 0, NULL);
}

static void rockchip_attach_connector_property(struct drm_device *drm)
{
	struct drm_connector *connector;
	struct drm_mode_config *conf = &drm->mode_config;
	struct drm_connector_list_iter conn_iter;

	mutex_lock(&drm->mode_config.mutex);

#define ROCKCHIP_PROP_ATTACH(prop, v) \
		drm_object_attach_property(&connector->base, prop, v)

	drm_connector_list_iter_begin(drm, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		ROCKCHIP_PROP_ATTACH(conf->tv_brightness_property, 50);
		ROCKCHIP_PROP_ATTACH(conf->tv_contrast_property, 50);
		ROCKCHIP_PROP_ATTACH(conf->tv_saturation_property, 50);
		ROCKCHIP_PROP_ATTACH(conf->tv_hue_property, 50);
	}
	drm_connector_list_iter_end(&conn_iter);
#undef ROCKCHIP_PROP_ATTACH

	mutex_unlock(&drm->mode_config.mutex);
}

static void rockchip_drm_set_property_default(struct drm_device *drm)
{
	struct drm_connector *connector;
	struct drm_mode_config *conf = &drm->mode_config;
	struct drm_atomic_state *state;
	int ret;
	struct drm_connector_list_iter conn_iter;

	drm_modeset_lock_all(drm);

	state = drm_atomic_helper_duplicate_state(drm, conf->acquire_ctx);
	if (!state) {
		DRM_ERROR("failed to alloc atomic state\n");
		goto err_unlock;
	}
	state->acquire_ctx = conf->acquire_ctx;

	drm_connector_list_iter_begin(drm, &conn_iter);
	drm_for_each_connector_iter(connector, &conn_iter) {
		struct drm_connector_state *connector_state;

		connector_state = drm_atomic_get_connector_state(state,
								 connector);
		if (IS_ERR(connector_state)) {
			DRM_ERROR("Connector[%d]: Failed to get state\n", connector->base.id);
			continue;
		}

		connector_state->tv.brightness = 50;
		connector_state->tv.contrast = 50;
		connector_state->tv.saturation = 50;
		connector_state->tv.hue = 50;
	}
	drm_connector_list_iter_end(&conn_iter);

	ret = drm_atomic_commit(state);
	WARN_ON(ret == -EDEADLK);
	if (ret)
		DRM_ERROR("Failed to update properties\n");
	drm_atomic_state_put(state);

err_unlock:
	drm_modeset_unlock_all(drm);
}

static int rockchip_drm_bind(struct device *dev)
{
	struct drm_device *drm_dev;
	struct rockchip_drm_private *private;
	int ret;

	/* Remove existing drivers that may own the framebuffer memory. */
	ret = drm_aperture_remove_framebuffers(false, &rockchip_drm_driver);
	if (ret) {
		DRM_DEV_ERROR(dev,
			      "Failed to remove existing framebuffers - %d.\n",
			      ret);
		return ret;
	}

	drm_dev = drm_dev_alloc(&rockchip_drm_driver, dev);
	if (IS_ERR(drm_dev))
		return PTR_ERR(drm_dev);

	dev_set_drvdata(dev, drm_dev);

	private = devm_kzalloc(drm_dev->dev, sizeof(*private), GFP_KERNEL);
	if (!private) {
		ret = -ENOMEM;
		goto err_free;
	}

	mutex_init(&private->ovl_lock);

	drm_dev->dev_private = private;

	INIT_LIST_HEAD(&private->psr_list);
	mutex_init(&private->psr_list_lock);
	mutex_init(&private->commit_lock);


	private->hdmi_pll.pll = devm_clk_get_optional(dev, "hdmi-tmds-pll");
	if (PTR_ERR(private->hdmi_pll.pll) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		goto err_free;
	} else if (IS_ERR(private->hdmi_pll.pll)) {
		dev_err(dev, "failed to get hdmi-tmds-pll\n");
		ret = PTR_ERR(private->hdmi_pll.pll);
		goto err_free;
	}
	private->default_pll.pll = devm_clk_get_optional(dev, "default-vop-pll");
	if (PTR_ERR(private->default_pll.pll) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		goto err_free;
	} else if (IS_ERR(private->default_pll.pll)) {
		dev_err(dev, "failed to get default vop pll\n");
		ret = PTR_ERR(private->default_pll.pll);
		goto err_free;
	}

	ret = rockchip_drm_init_iommu(drm_dev);
	if (ret)
		goto err_free;

	ret = drmm_mode_config_init(drm_dev);
	if (ret)
		goto err_free;

	rockchip_drm_mode_config_init(drm_dev);
	rockchip_drm_create_properties(drm_dev);

	/* Try to bind all sub drivers. */
	ret = component_bind_all(dev, drm_dev);
	if (ret)
		goto err_free;

	rockchip_attach_connector_property(drm_dev);

	ret = drm_vblank_init(drm_dev, drm_dev->mode_config.num_crtc);
	if (ret)
		goto err_iommu_cleanup;

	drm_mode_config_reset(drm_dev);
	rockchip_drm_set_property_default(drm_dev);

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(drm_dev);

	ret = drm_dev_register(drm_dev, 0);
	if (ret)
		goto err_kms_helper_poll_fini;

	drm_fbdev_generic_setup(drm_dev, 0);

	return 0;
err_kms_helper_poll_fini:
	drm_kms_helper_poll_fini(drm_dev);
err_iommu_cleanup:
	rockchip_iommu_cleanup(drm_dev);
	component_unbind_all(dev, drm_dev);
err_free:
	drm_dev_put(drm_dev);
	return ret;
}

static void rockchip_drm_unbind(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_dev_unregister(drm_dev);

	drm_kms_helper_poll_fini(drm_dev);

	drm_atomic_helper_shutdown(drm_dev);
	component_unbind_all(dev, drm_dev);
	rockchip_iommu_cleanup(drm_dev);

	drm_dev_put(drm_dev);
}

DEFINE_DRM_GEM_FOPS(rockchip_drm_driver_fops);

static const struct drm_driver rockchip_drm_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.dumb_create		= rockchip_gem_dumb_create,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table	= rockchip_gem_prime_import_sg_table,
	.gem_prime_mmap		= drm_gem_prime_mmap,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init		= rockchip_drm_debugfs_init,
#endif
	.fops			= &rockchip_drm_driver_fops,
	.name	= DRIVER_NAME,
	.desc	= DRIVER_DESC,
	.date	= DRIVER_DATE,
	.major	= DRIVER_MAJOR,
	.minor	= DRIVER_MINOR,
};

#ifdef CONFIG_PM_SLEEP
static int rockchip_drm_sys_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(drm);
}

static int rockchip_drm_sys_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(drm);
}
#endif

static const struct dev_pm_ops rockchip_drm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rockchip_drm_sys_suspend,
				rockchip_drm_sys_resume)
};

#define MAX_ROCKCHIP_SUB_DRIVERS 16
static struct platform_driver *rockchip_sub_drivers[MAX_ROCKCHIP_SUB_DRIVERS];
static int num_rockchip_sub_drivers;

/*
 * Get the endpoint id of the remote endpoint of the given encoder. This
 * information is used by the VOP2 driver to identify the encoder.
 *
 * @rkencoder: The encoder to get the remote endpoint id from
 * @np: The encoder device node
 * @port: The number of the port leading to the VOP2
 * @reg: The endpoint number leading to the VOP2
 */
int rockchip_drm_encoder_set_crtc_endpoint_id(struct rockchip_encoder *rkencoder,
					      struct device_node *np, int port, int reg)
{
	struct of_endpoint ep;
	struct device_node *en, *ren;
	int ret;

	en = of_graph_get_endpoint_by_regs(np, port, reg);
	if (!en)
		return -ENOENT;

	ren = of_graph_get_remote_endpoint(en);
	if (!ren)
		return -ENOENT;

	ret = of_graph_parse_endpoint(ren, &ep);
	if (ret)
		return ret;

	rkencoder->crtc_endpoint_id = ep.id;

	return 0;
}

/*
 * Check if a vop endpoint is leading to a rockchip subdriver or bridge.
 * Should be called from the component bind stage of the drivers
 * to ensure that all subdrivers are probed.
 *
 * @ep: endpoint of a rockchip vop
 *
 * returns true if subdriver, false if external bridge and -ENODEV
 * if remote port does not contain a device.
 */
int rockchip_drm_endpoint_is_subdriver(struct device_node *ep)
{
	struct device_node *node = of_graph_get_remote_port_parent(ep);
	struct platform_device *pdev;
	struct device_driver *drv;
	int i;

	if (!node)
		return -ENODEV;

	/* status disabled will prevent creation of platform-devices */
	if (!of_device_is_available(node)) {
		of_node_put(node);
		return -ENODEV;
	}

	pdev = of_find_device_by_node(node);
	of_node_put(node);

	/* enabled non-platform-devices can immediately return here */
	if (!pdev)
		return false;

	/*
	 * All rockchip subdrivers have probed at this point, so
	 * any device not having a driver now is an external bridge.
	 */
	drv = pdev->dev.driver;
	if (!drv) {
		platform_device_put(pdev);
		return false;
	}

	for (i = 0; i < num_rockchip_sub_drivers; i++) {
		if (rockchip_sub_drivers[i] == to_platform_driver(drv)) {
			platform_device_put(pdev);
			return true;
		}
	}

	platform_device_put(pdev);
	return false;
}

static void rockchip_drm_match_remove(struct device *dev)
{
	struct device_link *link;

	list_for_each_entry(link, &dev->links.consumers, s_node)
		device_link_del(link);
}

static struct component_match *rockchip_drm_match_add(struct device *dev)
{
	struct component_match *match = NULL;
	int i;

	for (i = 0; i < num_rockchip_sub_drivers; i++) {
		struct platform_driver *drv = rockchip_sub_drivers[i];
		struct device *p = NULL, *d;

		do {
			d = platform_find_device_by_driver(p, &drv->driver);
			put_device(p);
			p = d;

			if (!d)
				break;

			device_link_add(dev, d, DL_FLAG_STATELESS);
			component_match_add(dev, &match, component_compare_dev, d);
		} while (true);
	}

	if (IS_ERR(match))
		rockchip_drm_match_remove(dev);

	return match ?: ERR_PTR(-ENODEV);
}

static const struct component_master_ops rockchip_drm_ops = {
	.bind = rockchip_drm_bind,
	.unbind = rockchip_drm_unbind,
};

static int rockchip_drm_platform_of_probe(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *port;
	bool found = false;
	int i;

	if (!np)
		return -ENODEV;

	for (i = 0;; i++) {
		port = of_parse_phandle(np, "ports", i);
		if (!port)
			break;

		if (!of_device_is_available(port->parent)) {
			of_node_put(port);
			continue;
		}

		found = true;
		of_node_put(port);
	}

	if (i == 0) {
		DRM_DEV_ERROR(dev, "missing 'ports' property\n");
		return -ENODEV;
	}

	if (!found) {
		DRM_DEV_ERROR(dev,
			      "No available vop found for display-subsystem.\n");
		return -ENODEV;
	}

	return 0;
}

static int rockchip_drm_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct component_match *match = NULL;
	int ret;

	ret = rockchip_drm_platform_of_probe(dev);
	if (ret)
		return ret;

	match = rockchip_drm_match_add(dev);
	if (IS_ERR(match))
		return PTR_ERR(match);

	ret = component_master_add_with_match(dev, &rockchip_drm_ops, match);
	if (ret < 0) {
		rockchip_drm_match_remove(dev);
		return ret;
	}

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	return 0;
}

static int rockchip_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &rockchip_drm_ops);

	rockchip_drm_match_remove(&pdev->dev);

	return 0;
}

static void rockchip_drm_platform_shutdown(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	if (drm)
		drm_atomic_helper_shutdown(drm);
}

static const struct of_device_id rockchip_drm_dt_ids[] = {
	{ .compatible = "rockchip,display-subsystem", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rockchip_drm_dt_ids);

static struct platform_driver rockchip_drm_platform_driver = {
	.probe = rockchip_drm_platform_probe,
	.remove = rockchip_drm_platform_remove,
	.shutdown = rockchip_drm_platform_shutdown,
	.driver = {
		.name = "rockchip-drm",
		.of_match_table = rockchip_drm_dt_ids,
		.pm = &rockchip_drm_pm_ops,
	},
};

#define ADD_ROCKCHIP_SUB_DRIVER(drv, cond) { \
	if (IS_ENABLED(cond) && \
	    !WARN_ON(num_rockchip_sub_drivers >= MAX_ROCKCHIP_SUB_DRIVERS)) \
		rockchip_sub_drivers[num_rockchip_sub_drivers++] = &drv; \
}

static int __init rockchip_drm_init(void)
{
	int ret;

	if (drm_firmware_drivers_only())
		return -ENODEV;

	num_rockchip_sub_drivers = 0;
	ADD_ROCKCHIP_SUB_DRIVER(vop_platform_driver, CONFIG_ROCKCHIP_VOP);
	ADD_ROCKCHIP_SUB_DRIVER(vop2_platform_driver, CONFIG_ROCKCHIP_VOP2);
	ADD_ROCKCHIP_SUB_DRIVER(rockchip_lvds_driver,
				CONFIG_ROCKCHIP_LVDS);
	ADD_ROCKCHIP_SUB_DRIVER(rockchip_dp_driver,
				CONFIG_ROCKCHIP_ANALOGIX_DP);
	ADD_ROCKCHIP_SUB_DRIVER(cdn_dp_driver, CONFIG_ROCKCHIP_CDN_DP);
	ADD_ROCKCHIP_SUB_DRIVER(dw_hdmi_rockchip_pltfm_driver,
				CONFIG_ROCKCHIP_DW_HDMI);
	ADD_ROCKCHIP_SUB_DRIVER(dw_mipi_dsi_rockchip_driver,
				CONFIG_ROCKCHIP_DW_MIPI_DSI);
	ADD_ROCKCHIP_SUB_DRIVER(inno_hdmi_driver, CONFIG_ROCKCHIP_INNO_HDMI);
	ADD_ROCKCHIP_SUB_DRIVER(rk3066_hdmi_driver,
				CONFIG_ROCKCHIP_RK3066_HDMI);

	ret = platform_register_drivers(rockchip_sub_drivers,
					num_rockchip_sub_drivers);
	if (ret)
		return ret;

	ret = platform_driver_register(&rockchip_drm_platform_driver);
	if (ret)
		goto err_unreg_drivers;

	return 0;

err_unreg_drivers:
	platform_unregister_drivers(rockchip_sub_drivers,
				    num_rockchip_sub_drivers);
	return ret;
}

static void __exit rockchip_drm_fini(void)
{
	platform_driver_unregister(&rockchip_drm_platform_driver);

	platform_unregister_drivers(rockchip_sub_drivers,
				    num_rockchip_sub_drivers);
}

module_init(rockchip_drm_init);
module_exit(rockchip_drm_fini);

MODULE_AUTHOR("Mark Yao <mark.yao@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP DRM Driver");
MODULE_LICENSE("GPL v2");
