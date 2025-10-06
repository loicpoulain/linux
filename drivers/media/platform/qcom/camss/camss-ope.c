// SPDX-License-Identifier: GPL-2.0
/*
 * camss-ope.c
 *
 * Qualcomm MSM Camera Subsystem - Offline Processing Engine
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * This driver provides a minimal implementation for the Qualcomm Offline
 * Processing Engine (OPE). OPE is a memory-to-memory hardware block
 * designed for image processing on a source frame. Typically, the input
 * frame originates from the SoC CSI capture path, though not limited to.
 *
 * The hardware architecture consists of Fetch Engines and Write Engines,
 * connected through intermediate pipeline modules:
 *   [FETCH ENGINES] => [Pipeline Modules] => [WRITE ENGINES]
 *
 * Current Configuration:
 *     Fetch Engine: One fetch engine is used for Bayer frame input.
 *     Write Engines: Two display write engines for Y and UV planes output.
 *
 * Only a subset of the pipeline modules are enabled:
 *   CLC_WB: White balance for channel gain configuration
 *   CLC_DEMO: Demosaic for Bayer to RGB conversion
 *   CLC_CHROMA_ENHAN: for RGB to YUV conversion
 *   CLC_DOWNSCALE*: Downscaling for UV (YUV444 -> YUV422/YUV420) and YUV planes
 *
 * Default configuration values are based on public standards such as BT.601.
 *
 * Processing Model:
 * OPE processes frames in stripes of up to 336 pixels. Therefore, frames must
 * be split into stripes for processing. Each stripe is configured after the
 * previous one has been acquired (double buffered registers). To minimize
 * inter-stripe latency, the stripe configurations are generated ahead of time.
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/units.h>

#include <media/media-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include "camss-isp-m2m.h"
#include <uapi/linux/camss-config.h>
#include "camss-isp-params.h"
#include "camss-isp-pipeline.h"

#define OPE_ISP_NAME		"qcom-camss-ope"

/* TOP registers */
#define OPE_TOP_HW_VERSION						0x000
#define		OPE_TOP_HW_VERSION_STEP		GENMASK(15, 0)
#define		OPE_TOP_HW_VERSION_REV		GENMASK(27, 16)
#define		OPE_TOP_HW_VERSION_GEN		GENMASK(31, 28)
#define OPE_TOP_RESET_CMD						0x004
#define		OPE_TOP_RESET_CMD_HW		BIT(0)
#define		OPE_TOP_RESET_CMD_SW		BIT(1)
#define		OPE_TOP_CORE_CFG					0x010
#define OPE_TOP_IRQ_STATUS						0x014
#define OPE_TOP_IRQ_MASK						0x018
#define		OPE_TOP_IRQ_STATUS_RST_DONE	BIT(0)
#define		OPE_TOP_IRQ_STATUS_WE		BIT(1)
#define		OPE_TOP_IRQ_STATUS_FE		BIT(2)
#define		OPE_TOP_IRQ_STATUS_VIOL		BIT(3)
#define		OPE_TOP_IRQ_STATUS_IDLE		BIT(4)
#define OPE_TOP_IRQ_CLEAR						0x01c
#define OPE_TOP_IRQ_SET							0x020
#define OPE_TOP_IRQ_CMD							0x024
#define		OPE_TOP_IRQ_CMD_CLEAR		BIT(0)
#define		OPE_TOP_IRQ_CMD_SET		BIT(4)
#define OPE_TOP_VIOLATION_STATUS					0x028
#define OPE_TOP_DEBUG(i)						(0x0a0 + (i) * 4)
#define OPE_TOP_DEBUG_CFG						0x0dc

/* Fetch engines */
#define OPE_BUS_RD_INPUT_IF_IRQ_MASK					0x00c
#define OPE_BUS_RD_INPUT_IF_IRQ_CLEAR					0x010
#define OPE_BUS_RD_INPUT_IF_IRQ_CMD					0x014
#define		OPE_BUS_RD_INPUT_IF_IRQ_CMD_CLEAR	BIT(0)
#define		OPE_BUS_RD_INPUT_IF_IRQ_CMD_SET		BIT(4)
#define OPE_BUS_RD_INPUT_IF_IRQ_STATUS					0x018
#define		OPE_BUS_RD_INPUT_IF_IRQ_STATUS_RST_DONE	BIT(0)
#define		OPE_BUS_RD_INPUT_IF_IRQ_STATUS_RUP_DONE	BIT(1)
#define		OPE_BUS_RD_INPUT_IF_IRQ_STATUS_BUF_DONE	BIT(2)
#define OPE_BUS_RD_INPUT_IF_CMD						0x01c
#define		OPE_BUS_RD_INPUT_IF_CMD_GO_CMD		BIT(0)
#define OPE_BUS_RD_CLIENT_0_CORE_CFG					0x050
#define		OPE_BUS_RD_CLIENT_0_CORE_CFG_EN		BIT(0)
#define OPE_BUS_RD_CLIENT_0_CCIF_META_DATA				0x054
#define		OPE_BUS_RD_CLIENT_0_CCIF_MD_PIX_PATTERN	GENMASK(7, 2)
#define OPE_BUS_RD_CLIENT_0_ADDR_IMAGE						0x058
#define OPE_BUS_RD_CLIENT_0_RD_BUFFER_SIZE				0x05c
#define OPE_BUS_RD_CLIENT_0_RD_STRIDE					0x060
#define OPE_BUS_RD_CLIENT_0_UNPACK_CFG_0				0x064

/* Write engines */
#define OPE_BUS_WR_INPUT_IF_IRQ_MASK_0					0x018
#define OPE_BUS_WR_INPUT_IF_IRQ_MASK_1					0x01c
#define OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_0					0x020
#define OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_1					0x024
#define OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0				0x028
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE	BIT(0)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_BUF_DONE	BIT(8)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL	BIT(28)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL		BIT(30)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL	BIT(31)
#define OPE_BUS_WR_INPUT_IF_IRQ_STATUS_1				0x02c
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_1_CLIENT_DONE(c)	BIT(0 + (c))
#define OPE_BUS_WR_INPUT_IF_IRQ_CMD					0x030
#define		OPE_BUS_WR_INPUT_IF_IRQ_CMD_CLEAR		BIT(0)
#define		OPE_BUS_WR_INPUT_IF_IRQ_CMD_SET			BIT(1)
#define OPE_BUS_WR_VIOLATION_STATUS					0x064
#define OPE_BUS_WR_IMAGE_SIZE_VIOLATION_STATUS				0x070
#define OPE_BUS_WR_CLIENT_CFG(c)					(0x200 + (c) * 0x100)
#define		OPE_BUS_WR_CLIENT_CFG_EN			BIT(0)
#define		OPE_BUS_WR_CLIENT_CFG_AUTORECOVER		BIT(4)
#define OPE_BUS_WR_CLIENT_ADDR_IMAGE(c)					(0x204 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_0(c)				(0x20c + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_1(c)				(0x210 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_2(c)				(0x214 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_PACKER_CFG(c)					(0x218 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_ADDR_FRAME_HEADER(c)				(0x220 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_MAX	8

/* White Balance Gain (WB) Pipeline module */
#define OPE_PP_CLC_WB_GAIN_MODULE_CFG					(0x200 + 0x60)
#define		OPE_PP_CLC_WB_GAIN_MODULE_CFG_EN	BIT(0)
#define OPE_PP_CLC_WB_GAIN_WB_CFG(ch)					(0x200 + 0x68 + 4 * (ch))

/* Color Correction (CC) pipeline module */
#define OPE_PP_CLC_CC_BASE						0x400
#define OPE_PP_CLC_CC_MODULE_CFG					(OPE_PP_CLC_CC_BASE + 0x60)
#define		OPE_PP_CLC_CC_MODULE_CFG_EN			BIT(0)
#define OPE_PP_CLC_CC_COEFF_A_CFG_0					(OPE_PP_CLC_CC_BASE + 0x68)
#define		OPE_PP_CLC_CC_COEFF_A_CFG_0_A0			GENMASK(11, 0)
#define		OPE_PP_CLC_CC_COEFF_A_CFG_0_A1			GENMASK(27, 16)
#define OPE_PP_CLC_CC_COEFF_A_CFG_1					(OPE_PP_CLC_CC_BASE + 0x6c)
#define		OPE_PP_CLC_CC_COEFF_A_CFG_1_A2			GENMASK(11, 0)
#define OPE_PP_CLC_CC_COEFF_B_CFG_0					(OPE_PP_CLC_CC_BASE + 0x70)
#define		OPE_PP_CLC_CC_COEFF_B_CFG_0_B0			GENMASK(11, 0)
#define		OPE_PP_CLC_CC_COEFF_B_CFG_0_B1			GENMASK(27, 16)
#define OPE_PP_CLC_CC_COEFF_B_CFG_1					(OPE_PP_CLC_CC_BASE + 0x74)
#define		OPE_PP_CLC_CC_COEFF_B_CFG_1_B2			GENMASK(11, 0)
#define OPE_PP_CLC_CC_COEFF_C_CFG_0					(OPE_PP_CLC_CC_BASE + 0x78)
#define		OPE_PP_CLC_CC_COEFF_C_CFG_0_C0			GENMASK(11, 0)
#define		OPE_PP_CLC_CC_COEFF_C_CFG_0_C1			GENMASK(27, 16)
#define OPE_PP_CLC_CC_COEFF_C_CFG_1					(OPE_PP_CLC_CC_BASE + 0x7c)
#define		OPE_PP_CLC_CC_COEFF_C_CFG_1_C2			GENMASK(11, 0)
#define OPE_PP_CLC_CC_COEFF_K_CFG_0					(OPE_PP_CLC_CC_BASE + 0x80)
#define		OPE_PP_CLC_CC_COEFF_K_CFG_0_K0			GENMASK(12, 0)
#define OPE_PP_CLC_CC_COEFF_K_CFG_1					(OPE_PP_CLC_CC_BASE + 0x84)
#define		OPE_PP_CLC_CC_COEFF_K_CFG_1_K1			GENMASK(12, 0)
#define OPE_PP_CLC_CC_COEFF_K_CFG_2					(OPE_PP_CLC_CC_BASE + 0x88)
#define		OPE_PP_CLC_CC_COEFF_K_CFG_2_K2			GENMASK(12, 0)
#define OPE_PP_CLC_CC_COEFF_M_CFG					(OPE_PP_CLC_CC_BASE + 0x8c)
#define		OPE_PP_CLC_CC_COEFF_M_CFG_M			GENMASK(11, 0)

/* Demosaic (DEMO) pipeline module */
#define OPE_PP_CLC_DEMO_MODULE_CFG					(0x800 + 0x60)
#define		OPE_PP_CLC_DEMO_MODULE_CFG_EN			BIT(0)
#define		OPE_PP_CLC_DEMO_MODULE_CFG_DYN_G_CLAMP_EN	BIT(4)
#define OPE_PP_CLC_DEMO_INTERP_COEFF_CFG				(0x800 + 0x68)
#define		OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_G	GENMASK(15, 8)
#define		OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_RB	GENMASK(7, 0)
#define OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0				(0x800 + 0x6c)
#define		OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0_AK	GENMASK(15, 0)
#define OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1				(0x800 + 0x70)
#define		OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1_WK	GENMASK(15, 0)

/* Downscaler pipeline modules */
#define OPE_PP_CLC_DOWNSCALE_MN_DS_C_PRE_BASE				0x1c00
#define OPE_PP_CLC_DOWNSCALE_MN_DS_Y_DISP_BASE				0x3000
#define OPE_PP_CLC_DOWNSCALE_MN_DS_C_DISP_BASE				0x3200
#define OPE_PP_CLC_DOWNSCALE_MN_CFG(ds)					((ds) + 0x60)
#define		OPE_PP_CLC_DOWNSCALE_MN_CFG_EN			BIT(0)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_CFG(ds)				((ds) + 0x64)
#define		OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_H_SCALE_EN	BIT(9)
#define		OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_V_SCALE_EN	BIT(10)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_IMAGE_SIZE_CFG(ds)			((ds) + 0x68)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_MN_H_CFG(ds)				((ds) + 0x6c)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_MN_V_CFG(ds)				((ds) + 0x74)

/* Chroma Enhancement (RGB to YUV) pipeline module */
#define OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG				(0x1200 + 0x60)
#define		OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG_EN		BIT(0)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0				(0x1200 + 0x68)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V0		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V1		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1				(0x1200 + 0x6c)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1_K		GENMASK(31, 23)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2				(0x1200 + 0x70)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2_V2		GENMASK(11, 0)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG				(0x1200 + 0x74)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG				(0x1200 + 0x78)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG				(0x1200 + 0x7C)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG				(0x1200 + 0x80)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DP		GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DM		GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0				(0x1200 + 0x84)
#define		OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0_KCB	GENMASK(31, 21)
#define OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1				(0x1200 + 0x88)
#define		OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1_KCR	GENMASK(31, 21)

#define OPE_STRIPE_MAX_W	336
#define OPE_STRIPE_MAX_H	8192
#define OPE_STRIPE_MIN_W	16
#define OPE_STRIPE_MIN_H	OPE_STRIPE_MIN_W
#define OPE_MAX_STRIPE		16
#define OPE_ALIGN_H		1
#define OPE_ALIGN_W		1
#define OPE_MIN_W		24
#define OPE_MIN_H		16
#define OPE_MAX_W		(OPE_STRIPE_MAX_W * OPE_MAX_STRIPE)
#define OPE_MAX_H		OPE_STRIPE_MAX_H

#define OPE_RESET_TIMEOUT_MS	100

static inline unsigned int ope_timeperframe_to_fps(const struct v4l2_fract *tpf)
{
	if (!tpf->numerator || !tpf->denominator)
		return 240;

	return tpf->denominator / tpf->numerator;
}

/* Downscaler helpers */
#define Q21(v) (((uint64_t)(v)) << 21)
#define DS_Q21(n, d) ((uint32_t)(((uint64_t)(n) << 21) / (d)))
#define DS_RESOLUTION(input, output) \
	(((output) * 128 <= (input)) ? 0x0 : \
	((output) * 16 <= (input))  ? 0x1 : \
	((output) * 8  <= (input))  ? 0x2 : 0x3)
#define DS_OUTPUT_PIX(input, phase_init, phase_step) \
		((Q21(input) - (phase_init)) / (phase_step))

enum ope_downscaler {
	OPE_DS_C_PRE,
	OPE_DS_C_DISP,
	OPE_DS_Y_DISP,
	OPE_DS_MAX,
};

static const u32 ope_ds_base[OPE_DS_MAX] = { OPE_PP_CLC_DOWNSCALE_MN_DS_C_PRE_BASE,
					     OPE_PP_CLC_DOWNSCALE_MN_DS_C_DISP_BASE,
					     OPE_PP_CLC_DOWNSCALE_MN_DS_Y_DISP_BASE };

enum ope_wr_client {
	OPE_WR_CLIENT_VID_Y,
	OPE_WR_CLIENT_VID_C,
	OPE_WR_CLIENT_DISP_Y,
	OPE_WR_CLIENT_DISP_C,
	OPE_WR_CLIENT_ARGB,
	OPE_WR_CLIENT_MAX
};

enum ope_pixel_pattern {
	OPE_PIXEL_PATTERN_RGRGRG,
	OPE_PIXEL_PATTERN_GRGRGR,
	OPE_PIXEL_PATTERN_BGBGBG,
	OPE_PIXEL_PATTERN_GBGBGB,
	OPE_PIXEL_PATTERN_YCBYCR,
	OPE_PIXEL_PATTERN_YCRYCB,
	OPE_PIXEL_PATTERN_CBYCRY,
	OPE_PIXEL_PATTERN_CRYCBY
};

enum ope_stripe_location {
	OPE_STRIPE_LOCATION_FULL,
	OPE_STRIPE_LOCATION_LEFT,
	OPE_STRIPE_LOCATION_RIGHT,
	OPE_STRIPE_LOCATION_MIDDLE
};

enum ope_unpacker_format {
	OPE_UNPACKER_FMT_PLAIN_128,
	OPE_UNPACKER_FMT_PLAIN_8,
	OPE_UNPACKER_FMT_PLAIN_16_10BPP,
	OPE_UNPACKER_FMT_PLAIN_16_12BPP,
	OPE_UNPACKER_FMT_PLAIN_16_14BPP,
	OPE_UNPACKER_FMT_PLAIN_32_20BPP,
	OPE_UNPACKER_FMT_ARGB_16_10BPP,
	OPE_UNPACKER_FMT_ARGB_16_12BPP,
	OPE_UNPACKER_FMT_ARGB_16_14BPP,
	OPE_UNPACKER_FMT_PLAIN_32,
	OPE_UNPACKER_FMT_PLAIN_64,
	OPE_UNPACKER_FMT_TP_10,
	OPE_UNPACKER_FMT_MIPI_8,
	OPE_UNPACKER_FMT_MIPI_10,
	OPE_UNPACKER_FMT_MIPI_12,
	OPE_UNPACKER_FMT_MIPI_14,
	OPE_UNPACKER_FMT_PLAIN_16_16BPP,
	OPE_UNPACKER_FMT_PLAIN_128_ODD_EVEN,
	OPE_UNPACKER_FMT_PLAIN_8_ODD_EVEN
};

enum ope_packer_format {
	OPE_PACKER_FMT_PLAIN_128,
	OPE_PACKER_FMT_PLAIN_8,
	OPE_PACKER_FMT_PLAIN_8_ODD_EVEN,
	OPE_PACKER_FMT_PLAIN_8_10BPP,
	OPE_PACKER_FMT_PLAIN_8_10BPP_ODD_EVEN,
	OPE_PACKER_FMT_PLAIN_16_10BPP,
	OPE_PACKER_FMT_PLAIN_16_12BPP,
	OPE_PACKER_FMT_PLAIN_16_14BPP,
	OPE_PACKER_FMT_PLAIN_16_16BPP,
	OPE_PACKER_FMT_PLAIN_32,
	OPE_PACKER_FMT_PLAIN_64,
	OPE_PACKER_FMT_TP_10,
	OPE_PACKER_FMT_MIPI_10,
	OPE_PACKER_FMT_MIPI_12
};

/*
 * OPE HW-specific format info (pattern, packer/unpacker codes).
 * Generic info (fourcc, depth, align) lives in the camss_isp_fmt tables
 * referenced by the pipeline endpoint descriptors.
 */
struct ope_hw_fmt {
	u32 fourcc;
	enum ope_pixel_pattern pattern;
	enum ope_unpacker_format unpacker_format;
	enum ope_packer_format packer_format;
};

static const struct ope_hw_fmt ope_hw_formats[] = {
	/* Bayer MIPI 10 */
	{ V4L2_PIX_FMT_SBGGR10P, OPE_PIXEL_PATTERN_BGBGBG,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SGBRG10P, OPE_PIXEL_PATTERN_GBGBGB,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SGRBG10P, OPE_PIXEL_PATTERN_GRGRGR,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SRGGB10P, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_MIPI_10, OPE_PACKER_FMT_MIPI_10 },
	/* Bayer Plain 8 */
	{ V4L2_PIX_FMT_SRGGB8, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_SBGGR8, OPE_PIXEL_PATTERN_BGBGBG,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_SGBRG8, OPE_PIXEL_PATTERN_GBGBGB,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_SGRBG8, OPE_PIXEL_PATTERN_GRGRGR,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	/* YUV semi-planar */
	{ V4L2_PIX_FMT_NV24, OPE_PIXEL_PATTERN_YCBYCR,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV42, OPE_PIXEL_PATTERN_YCRYCB,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8_ODD_EVEN },
	{ V4L2_PIX_FMT_NV16, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV61, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8_ODD_EVEN },
	{ V4L2_PIX_FMT_NV12, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV21, OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8_ODD_EVEN },
	/* Greyscale */
	{ V4L2_PIX_FMT_GREY, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_PLAIN_8, OPE_PACKER_FMT_PLAIN_8 },
};

#define OPE_WB(n, d) (((n) << 10) / (d))

static const struct ope_hw_fmt *ope_find_hw_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ope_hw_formats); i++)
		if (ope_hw_formats[i].fourcc == fourcc)
			return &ope_hw_formats[i];
	return NULL;
}

struct ope_dev {
	struct device *dev;

	/* ISP pipeline owns v4l2_device, media_device, vnodes and subdev */
	struct camss_isp_pipeline pipeline;
	struct camss_isp_m2m_dev m2m_dev;

	void __iomem *base;
	void __iomem *base_rd;
	void __iomem *base_wr;
	void __iomem *base_pp;

	struct completion reset_complete;

	struct icc_path *icc_data;
	struct icc_path *icc_config;

	struct list_head ctx_list;
	void *context;
};

struct ope_dsc_config {
	u32 input_width;
	u32 input_height;
	u32 output_width;
	u32 output_height;
	u32 phase_step_h;
	u32 phase_step_v;
};

struct ope_stripe {
	struct {
		dma_addr_t addr;
		u32 width;
		u32 height;
		u32 stride;
		enum ope_stripe_location location;
		enum ope_pixel_pattern pattern;
		enum ope_unpacker_format format;
	} src;
	struct {
		dma_addr_t addr;
		u32 width;
		u32 height;
		u32 stride;
		u32 x_init;
		enum ope_packer_format format;
		bool enabled;
	} dst[OPE_WR_CLIENT_MAX];
	struct ope_dsc_config dsc[OPE_DS_MAX];
};

/**
 * struct ope_config - parsed ISP tuning parameters for one context
 *
 * Populated by ope_configure() from the params buffer.
 * Read by ope_prog_wb/bayer2rgb at job start.
 */
struct ope_config {
	struct camss_params_wb_gain wb_gain;
	struct camss_params_demo    demo;
	struct camss_params_chroma_enhan chroma_enhan;
	struct camss_params_color_correct color_correct;
};

struct ope_ctx {
	struct ope_dev *ope;
	struct camss_isp_m2m_ctx *mctx;	/* back-pointer to the M2M context */

	struct ope_config config;		/* current tuning parameters */

	/* Processing mode */
	int			mode;
	u8			alpha_component;
	u8			rotation;

	u8 current_stripe;
	struct ope_stripe stripe[OPE_MAX_STRIPE];

	bool started;

	struct list_head list;
};

struct ope_freq_tbl {
	unsigned int load;
	unsigned long freq;
};

static inline enum ope_stripe_location ope_stripe_location(unsigned int index,
							   unsigned int count)
{
	if (count == 1)
		return OPE_STRIPE_LOCATION_FULL;
	if (index == 0)
		return OPE_STRIPE_LOCATION_LEFT;
	if (index == (count - 1))
		return OPE_STRIPE_LOCATION_RIGHT;

	return OPE_STRIPE_LOCATION_MIDDLE;
}

static inline bool ope_stripe_is_last(struct ope_stripe *stripe)
{
	if (!stripe)
		return false;

	if (stripe->src.location == OPE_STRIPE_LOCATION_RIGHT ||
	    stripe->src.location == OPE_STRIPE_LOCATION_FULL)
		return true;

	return false;
}

static inline struct ope_stripe *ope_get_stripe(struct ope_ctx *ctx, unsigned int index)
{
	return &ctx->stripe[index];
}

static inline struct ope_stripe *ope_current_stripe(struct ope_ctx *ctx)
{
	if (!ctx)
		return NULL;

	if (ctx->current_stripe >= OPE_MAX_STRIPE)
		return NULL;

	return ope_get_stripe(ctx, ctx->current_stripe);
}

static inline unsigned int ope_stripe_index(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	return stripe - &ctx->stripe[0];
}

static inline struct ope_stripe *ope_prev_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	unsigned int index = ope_stripe_index(ctx, stripe);

	return index ? ope_get_stripe(ctx, index - 1) : NULL;
}

static inline struct ope_ctx *file2ctx(struct file *file)
{
	return camss_isp_m2m_ctx_from_file(file)->drv_priv;
}

static inline struct camss_isp_m2m_ctx *file2mctx(struct file *file)
{
	return camss_isp_m2m_ctx_from_file(file);
}

static inline u32 ope_read(struct ope_dev *ope, u32 reg)
{
	return readl(ope->base + reg);
}

static inline void ope_write(struct ope_dev *ope, u32 reg, u32 value)
{
	writel(value, ope->base + reg);
}

static inline u32 ope_read_wr(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_wr + reg);
}

static inline void ope_write_wr(struct ope_dev *ope, u32 reg, u32 value)
{
	writel_relaxed(value, ope->base_wr + reg);
}

static inline u32 ope_read_rd(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_rd + reg);
}

static inline void ope_write_rd(struct ope_dev *ope, u32 reg, u32 value)
{
	writel_relaxed(value, ope->base_rd + reg);
}

static inline u32 ope_read_pp(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_pp + reg);
}

static inline void ope_write_pp(struct ope_dev *ope, u32 reg, u32 value)
{
	writel_relaxed(value, ope->base_pp + reg);
}

static inline void ope_start(struct ope_dev *ope)
{
	wmb(); /* Ensure the next write occurs only after all prior normal memory accesses */
	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_CMD, OPE_BUS_RD_INPUT_IF_CMD_GO_CMD);
}


static inline void ope_dbg_print_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	struct ope_dev *ope = ctx->ope;
	int i;

	dev_dbg(ope->dev, "S%u/FE0: addr=%pad;W=%ub;H=%u;stride=%u;loc=%u;pattern=%u;fmt=%u\n",
		ope_stripe_index(ctx, stripe), &stripe->src.addr, stripe->src.width,
		stripe->src.height, stripe->src.stride, stripe->src.location, stripe->src.pattern,
		stripe->src.format);

	for (i = 0; i < OPE_DS_MAX; i++) {
		struct ope_dsc_config *c = &stripe->dsc[i];

		dev_dbg(ope->dev, "S%u/DSC%d: %ux%u => %ux%u\n",
			ope_stripe_index(ctx, stripe), i, c->input_width, c->input_height,
			c->output_width, c->output_height);
	}

	for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
		if (!stripe->dst[i].enabled)
			continue;

		dev_dbg(ope->dev,
			"S%u/WE%d: addr=%pad;X=%u;W=%upx;H=%u;stride=%u;fmt=%u\n",
			ope_stripe_index(ctx, stripe), i, &stripe->dst[i].addr,
			stripe->dst[i].x_init, stripe->dst[i].width, stripe->dst[i].height,
			stripe->dst[i].stride, stripe->dst[i].format);
	}
}

static void ope_gen_stripe_yuv_dst(struct ope_ctx *ctx, struct ope_stripe *stripe, dma_addr_t dst)
{
	const struct camss_isp_fmt_state *fmt_out =
		camss_isp_m2m_ctx_fmt_by_type(ctx->mctx, CAMSS_ISP_EP_FRAME_OUTPUT);
	struct ope_stripe *prev = ope_prev_stripe(ctx, stripe);
	enum ope_packer_format packer = OPE_PACKER_FMT_PLAIN_8;
	unsigned int pixelformat = fmt_out->fmt->fourcc;
	unsigned int img_width = fmt_out->width;
	unsigned int img_height = fmt_out->height;
	unsigned int width, height;
	u32 x_init = 0;

	stripe->dst[OPE_WR_CLIENT_DISP_Y].enabled = true;
	stripe->dst[OPE_WR_CLIENT_DISP_C].enabled = true;

	/* Y */
	width = stripe->dsc[OPE_DS_Y_DISP].output_width;
	height = stripe->dsc[OPE_DS_Y_DISP].output_height;

	if (prev)
		x_init = prev->dst[OPE_WR_CLIENT_DISP_Y].x_init +
			 prev->dst[OPE_WR_CLIENT_DISP_Y].width;

	stripe->dst[OPE_WR_CLIENT_DISP_Y].addr = dst;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].x_init = x_init;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].width = width;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].height = height;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].stride = img_width;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].format = packer;

	/* UV */
	width = stripe->dsc[OPE_DS_C_DISP].output_width;
	height = stripe->dsc[OPE_DS_C_DISP].output_height;

	if (prev)
		x_init = prev->dst[OPE_WR_CLIENT_DISP_C].x_init +
			 prev->dst[OPE_WR_CLIENT_DISP_C].width;

	packer = ope_find_hw_fmt(pixelformat)->packer_format;

	stripe->dst[OPE_WR_CLIENT_DISP_C].addr = dst + img_width * img_height;
	stripe->dst[OPE_WR_CLIENT_DISP_C].x_init = x_init;
	stripe->dst[OPE_WR_CLIENT_DISP_C].format = packer;
	stripe->dst[OPE_WR_CLIENT_DISP_C].width = width * 2;
	stripe->dst[OPE_WR_CLIENT_DISP_C].height = height;

	switch (pixelformat) {
	case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_NV24: /* YUV 4:4:4 */
		stripe->dst[OPE_WR_CLIENT_DISP_C].stride = img_width * 2;
		break;
	case V4L2_PIX_FMT_GREY: /* No UV */
		stripe->dst[OPE_WR_CLIENT_DISP_C].enabled = false;
		break;
	default:
		stripe->dst[OPE_WR_CLIENT_DISP_C].stride = img_width;
	}
}

static void ope_gen_stripe_dsc(struct ope_ctx *ctx, struct ope_stripe *stripe,
			       unsigned int h_scale, unsigned int v_scale)
{
	struct ope_dsc_config *dsc_c, *dsc_y;

	dsc_c = &stripe->dsc[OPE_DS_C_DISP];
	dsc_y = &stripe->dsc[OPE_DS_Y_DISP];

	dsc_c->phase_step_h = dsc_y->phase_step_h = h_scale;
	dsc_c->phase_step_v = dsc_y->phase_step_v = v_scale;

	dsc_c->input_width = stripe->dsc[OPE_DS_C_PRE].output_width;
	dsc_c->input_height = stripe->dsc[OPE_DS_C_PRE].output_height;

	dsc_y->input_width = stripe->src.width;
	dsc_y->input_height = stripe->src.height;

	dsc_c->output_width = DS_OUTPUT_PIX(dsc_c->input_width, 0, h_scale);
	dsc_c->output_height = DS_OUTPUT_PIX(dsc_c->input_height, 0, v_scale);

	dsc_y->output_width = DS_OUTPUT_PIX(dsc_y->input_width, 0, h_scale);
	dsc_y->output_height = DS_OUTPUT_PIX(dsc_y->input_height, 0, v_scale);

	/* Adjust initial phase ? */
}

static void ope_gen_stripe_chroma_dsc(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	const struct camss_isp_fmt_state *fmt_out =
		camss_isp_m2m_ctx_fmt_by_type(ctx->mctx, CAMSS_ISP_EP_FRAME_OUTPUT);
	struct ope_dsc_config *dsc;

	dsc = &stripe->dsc[OPE_DS_C_PRE];

	dsc->input_width = stripe->src.width;
	dsc->input_height = stripe->src.height;

	switch (fmt_out->fmt->fourcc) {
	case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV16:
		dsc->output_width = dsc->input_width / 2;
		dsc->output_height = dsc->input_height;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		dsc->output_width = dsc->input_width / 2;
		dsc->output_height = dsc->input_height / 2;
		break;
	default:
		dsc->output_width = dsc->input_width;
		dsc->output_height = dsc->input_height;
	}

	dsc->phase_step_h = DS_Q21(dsc->input_width, dsc->output_width);
	dsc->phase_step_v = DS_Q21(dsc->input_height, dsc->output_height);
}

static void ope_gen_stripes(struct ope_ctx *ctx, dma_addr_t src, dma_addr_t dst)
{
	const struct camss_isp_fmt_state *fmt_src_state =
		camss_isp_m2m_ctx_fmt_by_type(ctx->mctx, CAMSS_ISP_EP_FRAME_INPUT);
	const struct camss_isp_fmt_state *fmt_dst_state =
		camss_isp_m2m_ctx_fmt_by_type(ctx->mctx, CAMSS_ISP_EP_FRAME_OUTPUT);
	const struct camss_isp_fmt *src_fmt = fmt_src_state ? fmt_src_state->fmt : NULL;
	const struct ope_hw_fmt *hw_fmt = ope_find_hw_fmt(src_fmt->fourcc);
	unsigned int num_stripes, width, i;
	unsigned int h_scale, v_scale;

	width = fmt_src_state->width;
	num_stripes = DIV_ROUND_UP(fmt_src_state->width, OPE_STRIPE_MAX_W);
	h_scale = DS_Q21(fmt_src_state->width, fmt_dst_state->width);
	v_scale = DS_Q21(fmt_src_state->height, fmt_dst_state->height);

	for (i = 0; i < num_stripes; i++) {
		struct ope_stripe *stripe = &ctx->stripe[i];

		/* Clear config */
		memset(stripe, 0, sizeof(*stripe));

		/* Fetch Engine */
		stripe->src.addr = src;
		stripe->src.width = width;
		stripe->src.height = fmt_src_state->height;
		stripe->src.stride = fmt_src_state->bytesperline;
		stripe->src.location = ope_stripe_location(i, num_stripes);
		stripe->src.pattern = hw_fmt->pattern;
		stripe->src.format = hw_fmt->unpacker_format;

		/* Ensure the last stripe will be large enough */
		if (width > OPE_STRIPE_MAX_W && width < (OPE_STRIPE_MAX_W + OPE_STRIPE_MIN_W))
			stripe->src.width -= OPE_STRIPE_MIN_W * 2;

		v4l_bound_align_image(&stripe->src.width, src_fmt->align,
				      OPE_STRIPE_MAX_W, src_fmt->align,
				      &stripe->src.height, OPE_STRIPE_MIN_H, OPE_STRIPE_MAX_H,
				      OPE_ALIGN_H, 0);

		width -= stripe->src.width;
		src += stripe->src.width * src_fmt->depth / 8;

		/* YUV Chroma downscaling */
		ope_gen_stripe_chroma_dsc(ctx, stripe);

		/* YUV downscaling */
		ope_gen_stripe_dsc(ctx, stripe, h_scale, v_scale);

		/* Write Engines */
		ope_gen_stripe_yuv_dst(ctx, stripe, dst);

		/* Source width is in byte unit, not pixel */
		stripe->src.width = stripe->src.width * src_fmt->depth / 8;

		ope_dbg_print_stripe(ctx, stripe);
	}
}

/*
 * ope_module_update - check/clear DIRTY flag and program MODULE_CFG enable bit.
 *
 * Skips the dirty check on context switch (ctx != ctx->ope->context).
 * Always respects the DISABLE flag: writes 0 to MODULE_CFG and returns
 * false so the caller skips coefficient writes.
 * Returns true if the module is dirty (or forced) and enabled.
 */
static bool ope_module_update(struct ope_ctx *ctx, u32 module_cfg_reg, u32 enable_bits,
			      struct v4l2_isp_params_block_header *hdr)
{
	bool force = (ctx != ctx->ope->context);

	if (!force && !(hdr->flags & CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY))
		return false;

	hdr->flags &= ~CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;

	if (hdr->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		ope_write_pp(ctx->ope, module_cfg_reg, 0);
		return false;
	}

	ope_write_pp(ctx->ope, module_cfg_reg, enable_bits);

	return true;
}

static void ope_prog_color_correct(struct ope_ctx *ctx)
{
	const struct camss_params_color_correct *cc = &ctx->config.color_correct;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_CC_MODULE_CFG,
			       OPE_PP_CLC_CC_MODULE_CFG_EN,
			       &ctx->config.color_correct.header))
		return;

	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_A_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_A_CFG_0_A0, cc->a[0]) |
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_A_CFG_0_A1, cc->a[1]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_A_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_A_CFG_1_A2, cc->a[2]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_B_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_B_CFG_0_B0, cc->b[0]) |
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_B_CFG_0_B1, cc->b[1]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_B_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_B_CFG_1_B2, cc->b[2]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_C_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_C_CFG_0_C0, cc->c[0]) |
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_C_CFG_0_C1, cc->c[1]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_C_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_C_CFG_1_C2, cc->c[2]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_K_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_K_CFG_0_K0, cc->k[0]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_K_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_K_CFG_1_K1, cc->k[1]));
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_K_CFG_2,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_K_CFG_2_K2, cc->k[2]));
}

static void ope_prog_rgb2yuv(struct ope_ctx *ctx)
{
	const struct camss_params_chroma_enhan *cc = &ctx->config.chroma_enhan;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG,
			       OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG_EN,
			       &ctx->config.chroma_enhan.header))
		return;

	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V0, cc->luma_v0) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V1, cc->luma_v1));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2_V2, cc->luma_v2));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1_K, cc->luma_k));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AP, cc->coeff_ap) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AM, cc->coeff_am));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BP, cc->coeff_dp) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BM, cc->coeff_dm));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CP, cc->coeff_cp) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CM, cc->coeff_cm));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DP, cc->coeff_dp) |
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DM, cc->coeff_dm));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0_KCB, cc->kcb));
	ope_write_pp(ope, OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1_KCR, cc->kcr));
}

static void ope_prog_bayer2rgb(struct ope_ctx *ctx)
{
	const struct camss_params_demo *demo = &ctx->config.demo;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_DEMO_MODULE_CFG,
			       OPE_PP_CLC_DEMO_MODULE_CFG_EN |
			       OPE_PP_CLC_DEMO_MODULE_CFG_DYN_G_CLAMP_EN,
			       &ctx->config.demo.header))
		return;

	dev_dbg(ope->dev, "prog_demo: lambda_g=%u lambda_rb=%u a_k=%u w_k=%u\n",
		demo->lambda_g, demo->lambda_rb, demo->a_k, demo->w_k);

	ope_write_pp(ope, OPE_PP_CLC_DEMO_INTERP_COEFF_CFG,
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_G, demo->lambda_g) |
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_RB, demo->lambda_rb));
	ope_write_pp(ope, OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0_AK, demo->a_k));
	ope_write_pp(ope, OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1_WK, demo->w_k));
}

static const struct ope_config ope_default_config = {
	.wb_gain = {
		.header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE | CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY,
		.g_gain = OPE_WB(1, 1),
		.b_gain = OPE_WB(3, 2),
		.r_gain = OPE_WB(3, 2),
	},
	.demo = {
		.header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE | CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY,
		.lambda_rb = 0,
		.lambda_g  = 128,
		.a_k       = 128,
		.w_k       = 102,
	},
	.chroma_enhan = {
		.header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE | CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY,
		.luma_v0  = 0x04d,  /* R to Y  0.299 12sQ8 BT.601 */
		.luma_v1  = 0x096,  /* G to Y  0.587 12sQ8 BT.601 */
		.luma_v2  = 0x01d,  /* B to Y  0.114 12sQ8 BT.601 */
		.luma_k   = 0,
		.coeff_ap = 0x0e6,  /* Cb  0.886 12sQ8 BT.601 */
		.coeff_am = 0x0e6,
		.coeff_cp = 0x0b3,  /* Cr  0.701 12sQ8 BT.601 */
		.coeff_cm = 0x0b3,
		.coeff_dp = 0xfb3,  /* Cb -0.338 12sQ8 BT.601 */
		.coeff_dm = 0xfb3,
		.kcb      = 128,
		.kcr      = 128,
	},
};

static void ope_prog_wb(struct ope_ctx *ctx)
{
	const struct camss_params_wb_gain *wb = &ctx->config.wb_gain;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_WB_GAIN_MODULE_CFG,
			       OPE_PP_CLC_WB_GAIN_MODULE_CFG_EN,
			       &ctx->config.wb_gain.header))
		return;

	dev_dbg(ope->dev, "prog_wb: g=%u b=%u r=%u\n",
		wb->g_gain, wb->b_gain, wb->r_gain);

	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(0), wb->g_gain);
	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(1), wb->b_gain);
	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(2), wb->r_gain);
}

static void ope_prog_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	struct ope_dev *ope = ctx->ope;
	int i;

	dev_dbg(ope->dev, "Context %p - Programming S%u\n", ctx, ope_stripe_index(ctx, stripe));

	/* Fetch Engine */
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_UNPACK_CFG_0, stripe->src.format);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_RD_BUFFER_SIZE,
		     (stripe->src.width << 16) + stripe->src.height);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_ADDR_IMAGE, stripe->src.addr);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_RD_STRIDE, stripe->src.stride);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_CCIF_META_DATA,
		     FIELD_PREP(OPE_BUS_RD_CLIENT_0_CCIF_MD_PIX_PATTERN, stripe->src.pattern));
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_CORE_CFG, OPE_BUS_RD_CLIENT_0_CORE_CFG_EN);

	/* Write Engines */
	for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
		if (!stripe->dst[i].enabled) {
			ope_write_wr(ope, OPE_BUS_WR_CLIENT_CFG(i), 0);
			continue;
		}

		ope_write_wr(ope, OPE_BUS_WR_CLIENT_ADDR_IMAGE(i), stripe->dst[i].addr);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_0(i),
			     (stripe->dst[i].height << 16) + stripe->dst[i].width);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_1(i), stripe->dst[i].x_init);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_2(i), stripe->dst[i].stride);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_PACKER_CFG(i), stripe->dst[i].format);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_CFG(i),
			     OPE_BUS_WR_CLIENT_CFG_EN + OPE_BUS_WR_CLIENT_CFG_AUTORECOVER);
	}

	/* Downscalers */
	for (i = 0; i < OPE_DS_MAX; i++) {
		struct ope_dsc_config *dsc = &stripe->dsc[i];
		u32 base = ope_ds_base[i];
		u32 cfg = 0;

		if (dsc->input_width != dsc->output_width) {
			dsc->phase_step_h |= DS_RESOLUTION(dsc->input_width,
							   dsc->output_width) << 30;
			cfg |= OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_H_SCALE_EN;
		}

		if (dsc->input_height != dsc->output_height) {
			dsc->phase_step_v |= DS_RESOLUTION(dsc->input_height,
							   dsc->output_height) << 30;
			cfg |= OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_V_SCALE_EN;
		}

		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_CFG(base), cfg);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_IMAGE_SIZE_CFG(base),
			     ((dsc->input_width - 1) << 16) + dsc->input_height - 1);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_MN_H_CFG(base), dsc->phase_step_h);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_MN_V_CFG(base), dsc->phase_step_v);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_CFG(base),
			     cfg ? OPE_PP_CLC_DOWNSCALE_MN_CFG_EN : 0);
	}
}

/*
 * CAMSS ISP M2M callbacks
 */
static int ope_start_job(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;
	dma_addr_t src, dst;

	src = camss_isp_m2m_ctx_dma_addr(mctx, CAMSS_ISP_EP_FRAME_INPUT, 0);
	dst = camss_isp_m2m_ctx_dma_addr(mctx, CAMSS_ISP_EP_FRAME_OUTPUT, 0);

	dev_dbg(ope->dev, "start_job ctx=%p src=%pad dst=%pad\n", ctx, &src, &dst);

	/* Generate stripes from full frame */
	ope_gen_stripes(ctx, src, dst);

	if (ctx != ope->context)
		dev_dbg(ope->dev, "context switch %p -> %p\n", ope->context, ctx);

	/* IQ Modules */
	ope_prog_wb(ctx);
	ope_prog_bayer2rgb(ctx);
	ope_prog_rgb2yuv(ctx);
	ope_prog_color_correct(ctx);

	ope->context = ctx;

	/* Program the first stripe */
	ope_prog_stripe(ctx, &ctx->stripe[0]);

	/* Go! */
	ope_start(ope);

	return 0;
}

static void ope_job_done(struct ope_ctx *ctx, enum vb2_buffer_state vbstate)
{
	if (!ctx)
		return;

	camss_isp_m2m_job_done(ctx->mctx, vbstate);
}

static void ope_buf_done(struct ope_ctx *ctx)
{
	struct ope_stripe *stripe = ope_current_stripe(ctx);

	if (!ctx)
		return;

	dev_dbg(ctx->ope->dev, "Context %p Stripe %u done\n",
		ctx,  ope_stripe_index(ctx, stripe));

	if (ope_stripe_is_last(stripe)) {
		ctx->current_stripe = 0;
		ope_job_done(ctx, VB2_BUF_STATE_DONE);
	} else {
		ctx->current_stripe++;
		ope_start(ctx->ope);
	}
}

static void ope_abort_job(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;

	dev_dbg(ope->dev, "Abort context %p\n", ctx);

	ope_write(ope, OPE_TOP_RESET_CMD, OPE_TOP_RESET_CMD_SW);
}

static void ope_rup_done(struct ope_ctx *ctx)
{
	struct ope_stripe *stripe = ope_current_stripe(ctx);

	/* We can program next stripe (double buffered registers) */
	if (!ope_stripe_is_last(stripe))
		ope_prog_stripe(ctx, ++stripe);
}

/*
 * interrupt handler
 */
static void ope_fe_irq(struct ope_dev *ope)
{
	u32 status = ope_read_rd(ope, OPE_BUS_RD_INPUT_IF_IRQ_STATUS);

	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_IRQ_CLEAR, status);
	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_IRQ_CMD, OPE_BUS_RD_INPUT_IF_IRQ_CMD_CLEAR);

	/* Nothing to do */
}

static void ope_we_irq(struct ope_dev *ope, struct ope_ctx *ctx)
{
	u32 status;

	status = ope_read_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0);
	ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_0, status);
	ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_CMD,
		     OPE_BUS_WR_INPUT_IF_IRQ_CMD_CLEAR);

	if (!ctx)
		return;

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL) {
		dev_err_ratelimited(ope->dev,
			"Write Engine configuration violates constrains\n");
		ope_abort_job(ctx->mctx);
	}

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL) {
		u32 sz_viol = ope_read_wr(ope,
					OPE_BUS_WR_IMAGE_SIZE_VIOLATION_STATUS);
		int i;

		for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
			if (BIT(i) & sz_viol)
				dev_err_ratelimited(ope->dev,
					"Write Engine (WE%d) image size violation\n",
					i);
		}
		ope_abort_job(ctx->mctx);
	}

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL) {
		dev_err_ratelimited(ope->dev, "Write Engine fatal violation\n");
		ope_abort_job(ctx->mctx);
	}

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE)
		ope_rup_done(ctx);
}

static void ope_irq_init(struct ope_dev *ope)
{
	ope_write(ope, OPE_TOP_IRQ_MASK,
		  OPE_TOP_IRQ_STATUS_RST_DONE | OPE_TOP_IRQ_STATUS_WE |
		  OPE_TOP_IRQ_STATUS_VIOL | OPE_TOP_IRQ_STATUS_IDLE);

	ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_MASK_0,
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE |
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL |
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL |
		     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL);
}

static irqreturn_t ope_irq_thread(int irq, void *dev_id)
{
	struct ope_dev *ope = dev_id;
	struct ope_ctx *ctx;
	u32 status;

	guard(pm_runtime_active)(ope->dev);
	guard(mutex)(&ope->pipeline.lock);

	ctx = ope->context;

	status = ope_read(ope, OPE_TOP_IRQ_STATUS);
	ope_write(ope, OPE_TOP_IRQ_CLEAR, status);
	ope_write(ope, OPE_TOP_IRQ_CMD, OPE_TOP_IRQ_CMD_CLEAR);

	if (status & OPE_TOP_IRQ_STATUS_RST_DONE) {
		dev_dbg(ope->dev, "reset done current-ctx=%p\n", ctx);
		ope_job_done(ctx, VB2_BUF_STATE_ERROR);
		complete(&ope->reset_complete);
	}

	if (status & OPE_TOP_IRQ_STATUS_VIOL) {
		u32 violation = ope_read(ope, OPE_TOP_VIOLATION_STATUS);

		dev_warn(ope->dev, "OPE Violation: %u\n", violation);
	}

	if (status & OPE_TOP_IRQ_STATUS_FE)
		ope_fe_irq(ope);

	if (status & OPE_TOP_IRQ_STATUS_WE)
		ope_we_irq(ope, ctx);

	if (status & OPE_TOP_IRQ_STATUS_IDLE && ctx)
		ope_buf_done(ctx);

	return IRQ_HANDLED;
}

/*
 * video ioctls
 */
static void ope_adjust_power(struct ope_dev *ope)
{
	int ret;
	unsigned long pixclk = 0;
	unsigned int loadavg = 0;
	unsigned int loadpeak = 0;
	unsigned int loadconfig = 0;
	struct ope_ctx *ctx;

	list_for_each_entry(ctx, &ope->ctx_list, list) {
		if (!ctx->started)
			continue;

		const struct camss_isp_fmt_state *fmt_in =
			camss_isp_m2m_ctx_fmt_by_type(ctx->mctx, CAMSS_ISP_EP_FRAME_INPUT);
		const struct camss_isp_fmt_state *fmt_out =
			camss_isp_m2m_ctx_fmt_by_type(ctx->mctx, CAMSS_ISP_EP_FRAME_OUTPUT);

		unsigned int fps = ope_timeperframe_to_fps(&fmt_in->timeperframe);

		pixclk += (unsigned long)fmt_in->width * fmt_in->height * fps;
		loadavg += mult_frac(fmt_in->sizeimage, fps, KILO);
		loadavg += mult_frac(fmt_out->sizeimage, fps, KILO);
		loadpeak += mult_frac(fmt_in->sizeimage, fps * 2, KILO);
		loadpeak += mult_frac(fmt_out->sizeimage, fps * 2, KILO);
		loadconfig += mult_frac((fmt_in->width / OPE_STRIPE_MAX_W + 1) * 50 * 4,
					fps, KILO);
	}

	/* 30% margin for overhead */
	pixclk = mult_frac(pixclk, 13, 10);

	dev_dbg(ope->dev, "Adjusting clock:%luHz avg:%uKBps peak:%uKBps config:%uKBps\n",
		pixclk, loadavg, loadpeak, loadconfig);

	ret = dev_pm_opp_set_rate(ope->dev, pixclk);
	if (ret)
		dev_warn(ope->dev, "Failed to adjust OPP rate: %d\n", ret);

	ret = icc_set_bw(ope->icc_data, loadavg, loadpeak);
	if (ret)
		dev_warn(ope->dev, "Failed to set data path bandwidth: %d\n", ret);

	ret = icc_set_bw(ope->icc_config, loadconfig, loadconfig * 5);
	if (ret)
		dev_warn(ope->dev, "Failed to set config path bandwidth: %d\n", ret);
}

static const struct camss_isp_fmt ope_input_formats[] = {
	/* fourcc                  depth  align  planes */
	{ V4L2_PIX_FMT_SBGGR10P,  10,    2,     1 },
	{ V4L2_PIX_FMT_SGBRG10P,  10,    2,     1 },
	{ V4L2_PIX_FMT_SGRBG10P,  10,    2,     1 },
	{ V4L2_PIX_FMT_SRGGB10P,  10,    2,     1 },
	{ V4L2_PIX_FMT_SRGGB8,     8,    0,     1 },
	{ V4L2_PIX_FMT_SBGGR8,     8,    0,     1 },
	{ V4L2_PIX_FMT_SGBRG8,     8,    0,     1 },
	{ V4L2_PIX_FMT_SGRBG8,     8,    0,     1 },
};

static const struct camss_isp_fmt ope_output_formats[] = {
	/* fourcc               depth  align  planes */
	{ V4L2_PIX_FMT_NV24,   24,    0,     1 },
	{ V4L2_PIX_FMT_NV42,   24,    0,     1 },
	{ V4L2_PIX_FMT_NV16,   16,    1,     1 },
	{ V4L2_PIX_FMT_NV61,   16,    1,     1 },
	{ V4L2_PIX_FMT_NV12,   12,    1,     1 },
	{ V4L2_PIX_FMT_NV21,   12,    1,     1 },
	{ V4L2_PIX_FMT_GREY,    8,    0,     1 },
};

static const struct camss_isp_fmt ope_params_fmt = CAMSS_ISP_PARAMS_FMT_INIT;

static const struct camss_isp_video_desc ope_endpoints[] = {
	{
		.name        = "camss-isp-frame-input",
		.type = CAMSS_ISP_EP_FRAME_INPUT,
		.formats     = ope_input_formats,
		.num_formats = ARRAY_SIZE(ope_input_formats),
		.min_width   = OPE_MIN_W,
		.max_width   = OPE_MAX_W,
		.min_height  = OPE_MIN_H,
		.max_height  = OPE_MAX_H,
	},
	{
		.name        = "camss-isp-frame-output",
		.type = CAMSS_ISP_EP_FRAME_OUTPUT,
		.formats     = ope_output_formats,
		.num_formats = ARRAY_SIZE(ope_output_formats),
		.min_width   = OPE_MIN_W,
		.max_width   = OPE_MAX_W,
		.min_height  = OPE_MIN_H,
		.max_height  = OPE_MAX_H,
		.scaling     = true,
	},
	{
		.name        = "camss-isp-params",
		.type = CAMSS_ISP_EP_PARAMS,
		.formats     = &ope_params_fmt,
		.num_formats = 1,
		.buffersize  = v4l2_isp_params_buffer_size(CAMSS_PARAMS_MAX_PAYLOAD),
	},
};

static const struct camss_isp_link_desc ope_links[] = {
	{ CAMSS_ISP_LINK_EP(0), CAMSS_ISP_LINK_SD(0),
	  MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE },
	{ CAMSS_ISP_LINK_SD(1), CAMSS_ISP_LINK_EP(1),
	  MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE },
	{ CAMSS_ISP_LINK_EP(2), CAMSS_ISP_LINK_SD(2),
	  MEDIA_LNK_FL_ENABLED },
};

static const struct camss_isp_pipeline_desc ope_pipeline_desc = {
	.subdev_name      = "camss-isp-proc-engine",
	.fops             = &camss_isp_m2m_fops,
	.vb2_ops          = &camss_isp_m2m_vb2_ops,
	.num_subdev_pads  = 3,
	.endpoints        = ope_endpoints,
	.num_endpoints    = ARRAY_SIZE(ope_endpoints),
	.links            = ope_links,
	.num_links        = ARRAY_SIZE(ope_links),
};

static int ope_streaming_start(struct camss_isp_m2m_ctx *mctx,
				enum camss_isp_endpoint_type type)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;
	int ret;

	dev_dbg(ope->dev, "streaming_start ctx=%p type=%d\n", ctx, type);

	ret = pm_runtime_resume_and_get(ope->dev);
	if (ret) {
		dev_err(ope->dev, "Could not resume\n");
		return ret;
	}

	if (type == CAMSS_ISP_EP_FRAME_INPUT) {
		ctx->started = true;
		ope_adjust_power(ope);
	}

	ope_irq_init(ope);

	return 0;
}

static void ope_streaming_stop(struct camss_isp_m2m_ctx *mctx,
			       enum camss_isp_endpoint_type type)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;

	dev_dbg(ope->dev, "streaming_stop ctx=%p type=%d\n", ctx, type);

	if (type == CAMSS_ISP_EP_FRAME_INPUT) {
		ctx->started = false;
		ope_adjust_power(ope);
	}

	pm_runtime_put(ope->dev);
}

static void ope_params_apply_wb(void *priv,
				const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	dev_dbg(ctx->ope->dev, "params: wb_gain flags=0x%x\n",
		block->header.flags);

	ctx->config.wb_gain = block->wb_gain;
	ctx->config.wb_gain.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}


static void ope_params_apply_demo(void *priv,
				const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	dev_dbg(ctx->ope->dev, "params: demo flags=0x%x\n",
		block->header.flags);

	ctx->config.demo = block->demo;
	ctx->config.demo.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static void ope_params_apply_chroma_enhan(void *priv,
				const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	dev_dbg(ctx->ope->dev, "params: chroma_enhan flags=0x%x\n",
		block->header.flags);

	ctx->config.chroma_enhan = block->chroma_enhan;
	ctx->config.chroma_enhan.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static void ope_params_apply_color_correct(void *priv,
				const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	dev_dbg(ctx->ope->dev, "params: color_correct flags=0x%x\n",
		block->header.flags);

	ctx->config.color_correct = block->color_correct;
	ctx->config.color_correct.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static const struct v4l2_isp_params_block_type_info ope_params_type_info[] = {
	[CAMSS_PARAMS_WB_GAIN] = { sizeof(struct camss_params_wb_gain) },
	[CAMSS_PARAMS_DEMO]    = { sizeof(struct camss_params_demo) },
	[CAMSS_PARAMS_CHROMA_ENHAN] = { sizeof(struct camss_params_chroma_enhan) },
	[CAMSS_PARAMS_COLOR_CORRECT] = { sizeof(struct camss_params_color_correct) },
};

static const camss_isp_params_handler_fn ope_params_handlers[] = {
	[CAMSS_PARAMS_WB_GAIN]      = ope_params_apply_wb,
	[CAMSS_PARAMS_DEMO]         = ope_params_apply_demo,
	[CAMSS_PARAMS_CHROMA_ENHAN] = ope_params_apply_chroma_enhan,
	[CAMSS_PARAMS_COLOR_CORRECT] = ope_params_apply_color_correct,
};

static void ope_configure(struct camss_isp_m2m_ctx *mctx,
			  struct vb2_v4l2_buffer *params_buf)
{
	struct ope_ctx *ctx = mctx->drv_priv;

	dev_dbg(ctx->ope->dev, "configure: parsing params buffer\n");

	camss_isp_params_apply(ctx->ope->dev, &params_buf->vb2_buf,
			       ope_params_type_info, ope_params_handlers,
			       ARRAY_SIZE(ope_params_handlers), ctx);
}

static const struct camss_isp_m2m_ops ope_m2m_ops;  /* forward declaration */

/*
 * ope_hw_ready - return false if the OPE proc-engine is already busy.
 *
 * OPE has a single processing pipeline shared across all open contexts.
 * We must not dispatch a new job while one is already in flight.
 */
static bool ope_hw_ready(struct camss_isp_m2m_ctx *mctx)
{
	struct camss_isp_m2m_ctx *ctx;

	list_for_each_entry(ctx, &mctx->mdev->ctx_list, list) {
		if (ctx->running)
			return false;
	}

	return true;
}

static int ope_ctx_create(struct camss_isp_m2m_ctx *mctx)
{
	struct camss_isp_pipeline *pipe = mctx->mdev->pipe;
	struct ope_dev *ope = container_of(pipe, struct ope_dev, pipeline);
	struct ope_ctx *ctx;

	ctx = kvzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->ope    = ope;
	ctx->mctx   = mctx;
	ctx->config = ope_default_config;
	mctx->drv_priv = ctx;

	list_add(&ctx->list, &ope->ctx_list);

	dev_dbg(ope->dev, "Created ctx %p\n", ctx);

	return 0;
}

static void ope_ctx_destroy(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;

	dev_dbg(ope->dev, "Destroying ctx %p\n", ctx);

	if (ope->context == ctx)
		ope->context = NULL;

	list_del(&ctx->list);
	kvfree(ctx);
}

static const struct camss_isp_m2m_ops ope_m2m_ops = {
	.ctx_create      = ope_ctx_create,
	.ctx_destroy     = ope_ctx_destroy,
	.hw_ready        = ope_hw_ready,
	.configure       = ope_configure,
	.start_job       = ope_start_job,
	.abort_job       = ope_abort_job,
	.streaming_start = ope_streaming_start,
	.streaming_stop  = ope_streaming_stop,
};

static int ope_soft_reset(struct ope_dev *ope)
{
	u32 version;
	int ret = 0;

	guard(pm_runtime_active)(ope->dev);

	version = ope_read(ope, OPE_TOP_HW_VERSION);

	dev_dbg(ope->dev, "HW Version = %u.%u.%u\n",
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_GEN, version),
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_REV, version),
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_STEP, version));

	reinit_completion(&ope->reset_complete);

	ope_write(ope, OPE_TOP_RESET_CMD, OPE_TOP_RESET_CMD_SW);

	if (!wait_for_completion_timeout(&ope->reset_complete,
					 msecs_to_jiffies(OPE_RESET_TIMEOUT_MS))) {
		dev_err(ope->dev, "Reset timeout\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int ope_init_power(struct ope_dev *ope)
{
	struct dev_pm_domain_list *pmdomains;
	struct device *dev = ope->dev;
	int ret;

	ope->icc_data = devm_of_icc_get(dev, "data");
	if (IS_ERR(ope->icc_data))
		return dev_err_probe(dev, PTR_ERR(ope->icc_data),
				     "failed to get interconnect data path\n");

	ope->icc_config = devm_of_icc_get(dev, "config");
	if (IS_ERR(ope->icc_config))
		return dev_err_probe(dev, PTR_ERR(ope->icc_config),
				     "failed to get interconnect config path\n");

	/*  Devices with multiple PM domains must be attached separately */
	devm_pm_domain_attach_list(dev, NULL, &pmdomains);

	/* core clock is scaled as part of operating points */
	ret = devm_pm_opp_set_clkname(dev, "core");
	if (ret)
		return ret;

	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "invalid OPP table\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = devm_pm_clk_create(dev);
	if (ret)
		return ret;

	ret = of_pm_clk_add_clks(dev);
	if (ret < 0)
		return ret;

	return 0;
}

static int ope_init_mmio(struct ope_dev *ope)
{
	struct platform_device *pdev = to_platform_device(ope->dev);

	ope->base = devm_platform_ioremap_resource_byname(pdev, "top");
	if (IS_ERR(ope->base))
		return PTR_ERR(ope->base);

	ope->base_rd = devm_platform_ioremap_resource_byname(pdev, "bus_read");
	if (IS_ERR(ope->base_rd))
		return PTR_ERR(ope->base_rd);

	ope->base_wr = devm_platform_ioremap_resource_byname(pdev, "bus_write");
	if (IS_ERR(ope->base_wr))
		return PTR_ERR(ope->base_wr);

	ope->base_pp = devm_platform_ioremap_resource_byname(pdev, "pipeline");
	if (IS_ERR(ope->base_pp))
		return PTR_ERR(ope->base_pp);

	return 0;
}

static int ope_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ope_dev *ope;
	int ret, irq;

	ope = devm_kzalloc(dev, sizeof(*ope), GFP_KERNEL);
	if (!ope)
		return -ENOMEM;

	ope->dev = dev;
	init_completion(&ope->reset_complete);
	INIT_LIST_HEAD(&ope->ctx_list);

	ret = ope_init_power(ope);
	if (ret)
		return dev_err_probe(dev, ret, "Power init failed\n");

	ret = ope_init_mmio(ope);
	if (ret)
		return dev_err_probe(dev, ret, "MMIO init failed\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "Unable to get IRQ\n");

	ret = devm_request_threaded_irq(dev, irq, NULL, ope_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"ope", ope);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Requesting IRQ failed\n");

	ret = ope_soft_reset(ope);
	if (ret < 0)
		return ret;

	ret = camss_isp_m2m_dev_init(&ope->m2m_dev, &ope->pipeline, &ope_m2m_ops,
				     CAMSS_ISP_M2M_TRIGGER_ANY_OUTPUT);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init M2M device\n");

	ret = camss_isp_pipeline_build(&ope->pipeline, dev, OPE_ISP_NAME,
				       &ope_pipeline_desc);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to build ISP pipeline\n");

	platform_set_drvdata(pdev, ope);
	return 0;
}

static void ope_remove(struct platform_device *pdev)
{
	struct ope_dev *ope = platform_get_drvdata(pdev);

	camss_isp_pipeline_destroy(&ope->pipeline);
}

static const struct of_device_id ope_dt_ids[] = {
	{ .compatible = "qcom,qcm2290-camss-ope"},
	{ },
};
MODULE_DEVICE_TABLE(of, ope_dt_ids);

static const struct dev_pm_ops ope_pm_ops = {
	SET_RUNTIME_PM_OPS(pm_clk_suspend, pm_clk_resume, NULL)
};

static struct platform_driver ope_driver = {
	.probe		= ope_probe,
	.remove		= ope_remove,
	.driver		= {
		.name	= OPE_ISP_NAME,
		.of_match_table = ope_dt_ids,
		.pm = &ope_pm_ops,
	},
};

module_platform_driver(ope_driver);

MODULE_DESCRIPTION("CAMSS Offline Processing Engine");
MODULE_AUTHOR("Loic Poulain <loic.poulain@oss.qualcomm.com>");
MODULE_LICENSE("GPL");
