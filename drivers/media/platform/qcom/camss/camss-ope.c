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
#include <linux/slab.h>
#include <linux/units.h>

#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>

#include <uapi/linux/camss-config.h>

#include "camss-isp-m2m.h"
#include "camss-isp-params.h"

#define OPE_NAME	"qcom-camss-ope"

/* ---- Register map ------------------------------------------------------ */

/* TOP */
#define OPE_TOP_HW_VERSION					0x000
#define		OPE_TOP_HW_VERSION_STEP		GENMASK(15, 0)
#define		OPE_TOP_HW_VERSION_REV		GENMASK(27, 16)
#define		OPE_TOP_HW_VERSION_GEN		GENMASK(31, 28)
#define OPE_TOP_RESET_CMD					0x004
#define		OPE_TOP_RESET_CMD_HW		BIT(0)
#define		OPE_TOP_RESET_CMD_SW		BIT(1)
#define OPE_TOP_IRQ_STATUS					0x014
#define OPE_TOP_IRQ_MASK					0x018
#define		OPE_TOP_IRQ_STATUS_RST_DONE	BIT(0)
#define		OPE_TOP_IRQ_STATUS_WE		BIT(1)
#define		OPE_TOP_IRQ_STATUS_FE		BIT(2)
#define		OPE_TOP_IRQ_STATUS_VIOL		BIT(3)
#define		OPE_TOP_IRQ_STATUS_IDLE		BIT(4)
#define OPE_TOP_IRQ_CLEAR					0x01c
#define OPE_TOP_IRQ_CMD						0x024
#define		OPE_TOP_IRQ_CMD_CLEAR		BIT(0)
#define OPE_TOP_VIOLATION_STATUS				0x028

/* Fetch engine */
#define OPE_BUS_RD_INPUT_IF_IRQ_MASK				0x00c
#define OPE_BUS_RD_INPUT_IF_IRQ_CLEAR				0x010
#define OPE_BUS_RD_INPUT_IF_IRQ_CMD				0x014
#define		OPE_BUS_RD_INPUT_IF_IRQ_CMD_CLEAR	BIT(0)
#define OPE_BUS_RD_INPUT_IF_IRQ_STATUS				0x018
#define OPE_BUS_RD_INPUT_IF_CMD					0x01c
#define		OPE_BUS_RD_INPUT_IF_CMD_GO_CMD		BIT(0)
#define OPE_BUS_RD_CLIENT_0_CORE_CFG				0x050
#define		OPE_BUS_RD_CLIENT_0_CORE_CFG_EN	BIT(0)
#define OPE_BUS_RD_CLIENT_0_CCIF_META_DATA			0x054
#define		OPE_BUS_RD_CLIENT_0_CCIF_MD_PIX_PATTERN GENMASK(7, 2)
#define OPE_BUS_RD_CLIENT_0_ADDR_IMAGE				0x058
#define OPE_BUS_RD_CLIENT_0_RD_BUFFER_SIZE			0x05c
#define OPE_BUS_RD_CLIENT_0_RD_STRIDE				0x060
#define OPE_BUS_RD_CLIENT_0_UNPACK_CFG_0			0x064

/* Write engines */
#define OPE_BUS_WR_INPUT_IF_IRQ_MASK_0				0x018
#define OPE_BUS_WR_INPUT_IF_IRQ_MASK_1				0x01c
#define OPE_BUS_WR_INPUT_IF_IRQ_CLEAR_0				0x020
#define OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0			0x028
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE	BIT(0)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_BUF_DONE	BIT(8)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL	BIT(28)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL		BIT(30)
#define		OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL	BIT(31)
#define OPE_BUS_WR_INPUT_IF_IRQ_CMD				0x030
#define		OPE_BUS_WR_INPUT_IF_IRQ_CMD_CLEAR	BIT(0)
#define OPE_BUS_WR_VIOLATION_STATUS				0x064
#define OPE_BUS_WR_IMAGE_SIZE_VIOLATION_STATUS			0x070
#define OPE_BUS_WR_CLIENT_CFG(c)				(0x200 + (c) * 0x100)
#define		OPE_BUS_WR_CLIENT_CFG_EN		BIT(0)
#define		OPE_BUS_WR_CLIENT_CFG_AUTORECOVER	BIT(4)
#define OPE_BUS_WR_CLIENT_ADDR_IMAGE(c)				(0x204 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_0(c)			(0x20c + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_1(c)			(0x210 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_IMAGE_CFG_2(c)			(0x214 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_PACKER_CFG(c)				(0x218 + (c) * 0x100)
#define OPE_BUS_WR_CLIENT_MAX	8

/* Pipeline modules */
#define OPE_PP_CLC_WB_GAIN_MODULE_CFG				(0x200 + 0x60)
#define		OPE_PP_CLC_WB_GAIN_MODULE_CFG_EN	BIT(0)
#define OPE_PP_CLC_WB_GAIN_WB_CFG(ch)				(0x200 + 0x68 + 4 * (ch))

#define OPE_PP_CLC_CC_BASE					0x400
#define OPE_PP_CLC_CC_MODULE_CFG				(OPE_PP_CLC_CC_BASE + 0x60)
#define		OPE_PP_CLC_CC_MODULE_CFG_EN		BIT(0)
#define OPE_PP_CLC_CC_COEFF_A_CFG_0				(OPE_PP_CLC_CC_BASE + 0x68)
#define		OPE_PP_CLC_CC_COEFF_A_CFG_0_A0		GENMASK(11, 0)
#define		OPE_PP_CLC_CC_COEFF_A_CFG_0_A1		GENMASK(27, 16)
#define OPE_PP_CLC_CC_COEFF_A_CFG_1				(OPE_PP_CLC_CC_BASE + 0x6c)
#define		OPE_PP_CLC_CC_COEFF_A_CFG_1_A2		GENMASK(11, 0)
#define OPE_PP_CLC_CC_COEFF_B_CFG_0				(OPE_PP_CLC_CC_BASE + 0x70)
#define		OPE_PP_CLC_CC_COEFF_B_CFG_0_B0		GENMASK(11, 0)
#define		OPE_PP_CLC_CC_COEFF_B_CFG_0_B1		GENMASK(27, 16)
#define OPE_PP_CLC_CC_COEFF_B_CFG_1				(OPE_PP_CLC_CC_BASE + 0x74)
#define		OPE_PP_CLC_CC_COEFF_B_CFG_1_B2		GENMASK(11, 0)
#define OPE_PP_CLC_CC_COEFF_C_CFG_0				(OPE_PP_CLC_CC_BASE + 0x78)
#define		OPE_PP_CLC_CC_COEFF_C_CFG_0_C0		GENMASK(11, 0)
#define		OPE_PP_CLC_CC_COEFF_C_CFG_0_C1		GENMASK(27, 16)
#define OPE_PP_CLC_CC_COEFF_C_CFG_1				(OPE_PP_CLC_CC_BASE + 0x7c)
#define		OPE_PP_CLC_CC_COEFF_C_CFG_1_C2		GENMASK(11, 0)
#define OPE_PP_CLC_CC_COEFF_K_CFG_0				(OPE_PP_CLC_CC_BASE + 0x80)
#define		OPE_PP_CLC_CC_COEFF_K_CFG_0_K0		GENMASK(12, 0)
#define OPE_PP_CLC_CC_COEFF_K_CFG_1				(OPE_PP_CLC_CC_BASE + 0x84)
#define		OPE_PP_CLC_CC_COEFF_K_CFG_1_K1		GENMASK(12, 0)
#define OPE_PP_CLC_CC_COEFF_K_CFG_2				(OPE_PP_CLC_CC_BASE + 0x88)
#define		OPE_PP_CLC_CC_COEFF_K_CFG_2_K2		GENMASK(12, 0)
#define OPE_PP_CLC_CC_COEFF_M_CFG				(OPE_PP_CLC_CC_BASE + 0x8c)
#define		OPE_PP_CLC_CC_COEFF_M_CFG_M		GENMASK(11, 0)

#define OPE_PP_CLC_DEMO_MODULE_CFG				(0x800 + 0x60)
#define		OPE_PP_CLC_DEMO_MODULE_CFG_EN		BIT(0)
#define		OPE_PP_CLC_DEMO_MODULE_CFG_DYN_G_CLAMP_EN BIT(4)
#define OPE_PP_CLC_DEMO_INTERP_COEFF_CFG			(0x800 + 0x68)
#define		OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_G  GENMASK(15, 8)
#define		OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_RB GENMASK(7, 0)
#define OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0			(0x800 + 0x6c)
#define		OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0_AK GENMASK(15, 0)
#define OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1			(0x800 + 0x70)
#define		OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1_WK GENMASK(15, 0)

#define OPE_PP_CLC_DOWNSCALE_MN_DS_C_PRE_BASE			0x1c00
#define OPE_PP_CLC_DOWNSCALE_MN_DS_Y_DISP_BASE			0x3000
#define OPE_PP_CLC_DOWNSCALE_MN_DS_C_DISP_BASE			0x3200
#define OPE_PP_CLC_DOWNSCALE_MN_CFG(ds)			((ds) + 0x60)
#define		OPE_PP_CLC_DOWNSCALE_MN_CFG_EN		BIT(0)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_CFG(ds)			((ds) + 0x64)
#define		OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_H_SCALE_EN BIT(9)
#define		OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_V_SCALE_EN BIT(10)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_IMAGE_SIZE_CFG(ds)		((ds) + 0x68)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_MN_H_CFG(ds)		((ds) + 0x6c)
#define OPE_PP_CLC_DOWNSCALE_MN_DS_MN_V_CFG(ds)		((ds) + 0x74)

#define OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG			(0x1200 + 0x60)
#define		OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG_EN	BIT(0)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0			(0x1200 + 0x68)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V0	GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_0_V1	GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1			(0x1200 + 0x6c)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_1_K	GENMASK(31, 23)
#define OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2			(0x1200 + 0x70)
#define		OPE_PP_CLC_CHROMA_ENHAN_LUMA_CFG_2_V2	GENMASK(11, 0)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG			(0x1200 + 0x74)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AP	GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_A_CFG_AM	GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG			(0x1200 + 0x78)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BP	GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_B_CFG_BM	GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG			(0x1200 + 0x7C)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CP	GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_C_CFG_CM	GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG			(0x1200 + 0x80)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DP	GENMASK(11, 0)
#define		OPE_PP_CLC_CHROMA_ENHAN_COEFF_D_CFG_DM	GENMASK(27, 16)
#define OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0			(0x1200 + 0x84)
#define		OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_0_KCB GENMASK(31, 21)
#define OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1			(0x1200 + 0x88)
#define		OPE_PP_CLC_CHROMA_ENHAN_CHROMA_CFG_1_KCR GENMASK(31, 21)

/* ---- OPE-specific constants ------------------------------------------- */

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

/* Downscaler fixed-point helpers */
#define Q21(v)			(((uint64_t)(v)) << 21)
#define DS_Q21(n, d)		((uint32_t)(((uint64_t)(n) << 21) / (d)))
#define DS_RESOLUTION(in, out) \
	(((out) * 128 <= (in)) ? 0x0 : \
	 ((out) * 16  <= (in)) ? 0x1 : \
	 ((out) * 8   <= (in)) ? 0x2 : 0x3)
#define DS_OUTPUT_PIX(in, phase_init, phase_step) \
	((Q21(in) - (phase_init)) / (phase_step))

#define OPE_WB(n, d)		(((n) << 10) / (d))

/* ---- OPE-specific enums ----------------------------------------------- */

enum ope_downscaler {
	OPE_DS_C_PRE,
	OPE_DS_C_DISP,
	OPE_DS_Y_DISP,
	OPE_DS_MAX,
};

static const u32 ope_ds_base[OPE_DS_MAX] = {
	OPE_PP_CLC_DOWNSCALE_MN_DS_C_PRE_BASE,
	OPE_PP_CLC_DOWNSCALE_MN_DS_C_DISP_BASE,
	OPE_PP_CLC_DOWNSCALE_MN_DS_Y_DISP_BASE,
};

enum ope_wr_client {
	OPE_WR_CLIENT_VID_Y,
	OPE_WR_CLIENT_VID_C,
	OPE_WR_CLIENT_DISP_Y,
	OPE_WR_CLIENT_DISP_C,
	OPE_WR_CLIENT_ARGB,
	OPE_WR_CLIENT_MAX,
};

enum ope_pixel_pattern {
	OPE_PIXEL_PATTERN_RGRGRG,
	OPE_PIXEL_PATTERN_GRGRGR,
	OPE_PIXEL_PATTERN_BGBGBG,
	OPE_PIXEL_PATTERN_GBGBGB,
	OPE_PIXEL_PATTERN_YCBYCR,
	OPE_PIXEL_PATTERN_YCRYCB,
	OPE_PIXEL_PATTERN_CBYCRY,
	OPE_PIXEL_PATTERN_CRYCBY,
};

enum ope_stripe_location {
	OPE_STRIPE_LOCATION_FULL,
	OPE_STRIPE_LOCATION_LEFT,
	OPE_STRIPE_LOCATION_RIGHT,
	OPE_STRIPE_LOCATION_MIDDLE,
};

enum ope_unpacker_format {
	OPE_UNPACKER_FMT_PLAIN_8	= 1,
	OPE_UNPACKER_FMT_PLAIN_16_10BPP	= 2,
	OPE_UNPACKER_FMT_MIPI_10	= 13,
};

enum ope_packer_format {
	OPE_PACKER_FMT_PLAIN_8		= 1,
	OPE_PACKER_FMT_PLAIN_8_ODD_EVEN = 2,
	OPE_PACKER_FMT_PLAIN_64		= 10,
	OPE_PACKER_FMT_MIPI_10		= 12,
};

/* ---- OPE format table ------------------------------------------------- */

struct ope_hw_fmt {
	u32			fourcc;
	enum ope_pixel_pattern	pattern;
	enum ope_unpacker_format unpacker;
	enum ope_packer_format	packer;
};

static const struct ope_hw_fmt ope_hw_fmts[] = {
	{ V4L2_PIX_FMT_SBGGR10P, OPE_PIXEL_PATTERN_BGBGBG,
	  OPE_UNPACKER_FMT_MIPI_10,  OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SGBRG10P, OPE_PIXEL_PATTERN_GBGBGB,
	  OPE_UNPACKER_FMT_MIPI_10,  OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SGRBG10P, OPE_PIXEL_PATTERN_GRGRGR,
	  OPE_UNPACKER_FMT_MIPI_10,  OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SRGGB10P, OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_MIPI_10,  OPE_PACKER_FMT_MIPI_10 },
	{ V4L2_PIX_FMT_SRGGB8,   OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_SBGGR8,   OPE_PIXEL_PATTERN_BGBGBG,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_SGBRG8,   OPE_PIXEL_PATTERN_GBGBGB,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_SGRBG8,   OPE_PIXEL_PATTERN_GRGRGR,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV24,     OPE_PIXEL_PATTERN_YCBYCR,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV42,     OPE_PIXEL_PATTERN_YCRYCB,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8_ODD_EVEN },
	{ V4L2_PIX_FMT_NV16,     OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV61,     OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8_ODD_EVEN },
	{ V4L2_PIX_FMT_NV12,     OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
	{ V4L2_PIX_FMT_NV21,     OPE_PIXEL_PATTERN_CBYCRY,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8_ODD_EVEN },
	{ V4L2_PIX_FMT_GREY,     OPE_PIXEL_PATTERN_RGRGRG,
	  OPE_UNPACKER_FMT_PLAIN_8,  OPE_PACKER_FMT_PLAIN_8 },
};

static const struct ope_hw_fmt *ope_find_hw_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ope_hw_fmts); i++)
		if (ope_hw_fmts[i].fourcc == fourcc)
			return &ope_hw_fmts[i];
	return NULL;
}

/* ---- camss_isp_fmt tables (shared layer format descriptors) ----------- */

static const struct camss_isp_fmt ope_input_fmts[] = {
	{ V4L2_PIX_FMT_SBGGR10P, 10, 2, 1 },
	{ V4L2_PIX_FMT_SGBRG10P, 10, 2, 1 },
	{ V4L2_PIX_FMT_SGRBG10P, 10, 2, 1 },
	{ V4L2_PIX_FMT_SRGGB10P, 10, 2, 1 },
	{ V4L2_PIX_FMT_SRGGB8,    8, 0, 1 },
	{ V4L2_PIX_FMT_SBGGR8,    8, 0, 1 },
	{ V4L2_PIX_FMT_SGBRG8,    8, 0, 1 },
	{ V4L2_PIX_FMT_SGRBG8,    8, 0, 1 },
};

static const struct camss_isp_fmt ope_output_fmts[] = {
	{ V4L2_PIX_FMT_NV24,  24, 0, 1 },
	{ V4L2_PIX_FMT_NV42,  24, 0, 1 },
	{ V4L2_PIX_FMT_NV16,  16, 1, 1 },
	{ V4L2_PIX_FMT_NV61,  16, 1, 1 },
	{ V4L2_PIX_FMT_NV12,  12, 1, 1 },
	{ V4L2_PIX_FMT_NV21,  12, 1, 1 },
	{ V4L2_PIX_FMT_GREY,   8, 0, 1 },
};

/* ---- OPE stripe engine ------------------------------------------------ */

struct ope_dsc_config {
	u32 input_width, input_height;
	u32 output_width, output_height;
	u32 phase_step_h, phase_step_v;
};

struct ope_stripe {
	struct {
		dma_addr_t		addr;
		u32			width, height, stride;
		enum ope_stripe_location location;
		enum ope_pixel_pattern	pattern;
		enum ope_unpacker_format format;
	} src;
	struct {
		dma_addr_t		addr;
		u32			width, height, stride, x_init;
		enum ope_packer_format	format;
		bool			enabled;
	} dst[OPE_WR_CLIENT_MAX];
	struct ope_dsc_config dsc[OPE_DS_MAX];
};

/* ---- OPE tuning parameter state --------------------------------------- */

struct ope_config {
	struct camss_params_wb_gain	wb_gain;
	struct camss_params_demo	demo;
	struct camss_params_chroma_enhan chroma_enhan;
	struct camss_params_color_correct color_correct;
};

/* ---- OPE per-context state -------------------------------------------- */

struct ope_ctx {
	struct ope_dev		*ope;
	struct camss_isp_m2m_ctx *mctx;	/* back-pointer to shared layer ctx */

	struct ope_config	config;
	u8			current_stripe;
	struct ope_stripe	stripe[OPE_MAX_STRIPE];
};

/* ---- OPE device state ------------------------------------------------- */

struct ope_dev {
	struct device		*dev;
	struct camss_isp_m2m_dev mdev;	/* must be accessible via dev_get_drvdata */

	void __iomem		*base;
	void __iomem		*base_rd;
	void __iomem		*base_wr;
	void __iomem		*base_pp;

	struct completion	reset_complete;

	/* Currently active hardware context (set at job start) */
	struct ope_ctx		*hw_ctx;
};

/* ---- Register accessors ----------------------------------------------- */

static inline u32 ope_read(struct ope_dev *ope, u32 reg)
{
	return readl(ope->base + reg);
}

static inline void ope_write(struct ope_dev *ope, u32 reg, u32 val)
{
	writel(val, ope->base + reg);
}

static inline void ope_write_wr(struct ope_dev *ope, u32 reg, u32 val)
{
	writel_relaxed(val, ope->base_wr + reg);
}

static inline u32 ope_read_wr(struct ope_dev *ope, u32 reg)
{
	return readl_relaxed(ope->base_wr + reg);
}

static inline void ope_write_rd(struct ope_dev *ope, u32 reg, u32 val)
{
	writel_relaxed(val, ope->base_rd + reg);
}

static inline void ope_write_pp(struct ope_dev *ope, u32 reg, u32 val)
{
	writel_relaxed(val, ope->base_pp + reg);
}

static inline void ope_start(struct ope_dev *ope)
{
	wmb();
	ope_write_rd(ope, OPE_BUS_RD_INPUT_IF_CMD, OPE_BUS_RD_INPUT_IF_CMD_GO_CMD);
}

/* ---- Stripe helpers --------------------------------------------------- */

static inline enum ope_stripe_location
ope_stripe_location(unsigned int idx, unsigned int count)
{
	if (count == 1)		return OPE_STRIPE_LOCATION_FULL;
	if (idx == 0)		return OPE_STRIPE_LOCATION_LEFT;
	if (idx == count - 1)	return OPE_STRIPE_LOCATION_RIGHT;
	return OPE_STRIPE_LOCATION_MIDDLE;
}

static inline bool ope_stripe_is_last(const struct ope_stripe *s)
{
	return s && (s->src.location == OPE_STRIPE_LOCATION_RIGHT ||
		     s->src.location == OPE_STRIPE_LOCATION_FULL);
}

static inline struct ope_stripe *ope_current_stripe(struct ope_ctx *ctx)
{
	if (ctx->current_stripe >= OPE_MAX_STRIPE)
		return NULL;
	return &ctx->stripe[ctx->current_stripe];
}

static inline bool ope_pix_fmt_is_yuv(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_NV16: case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV24: case V4L2_PIX_FMT_NV61:
	case V4L2_PIX_FMT_NV21: case V4L2_PIX_FMT_NV42:
	case V4L2_PIX_FMT_GREY:
		return true;
	default:
		return false;
	}
}

static void ope_gen_stripe_chroma_dsc(struct ope_ctx *ctx,
				      struct ope_stripe *stripe)
{
	struct ope_dsc_config *dsc = &stripe->dsc[OPE_DS_C_PRE];
	u32 dst_fourcc = ctx->mctx->fmt_out.fmt->fourcc;

	dsc->input_width  = stripe->src.width;
	dsc->input_height = stripe->src.height;

	switch (dst_fourcc) {
	case V4L2_PIX_FMT_NV61: case V4L2_PIX_FMT_NV16:
		dsc->output_width  = dsc->input_width / 2;
		dsc->output_height = dsc->input_height;
		break;
	case V4L2_PIX_FMT_NV12: case V4L2_PIX_FMT_NV21:
		dsc->output_width  = dsc->input_width / 2;
		dsc->output_height = dsc->input_height / 2;
		break;
	default:
		dsc->output_width  = dsc->input_width;
		dsc->output_height = dsc->input_height;
	}

	dsc->phase_step_h = DS_Q21(dsc->input_width,  dsc->output_width);
	dsc->phase_step_v = DS_Q21(dsc->input_height, dsc->output_height);
}

static void ope_gen_stripe_dsc(struct ope_ctx *ctx, struct ope_stripe *stripe,
				u32 h_scale, u32 v_scale)
{
	struct ope_dsc_config *dsc_c = &stripe->dsc[OPE_DS_C_DISP];
	struct ope_dsc_config *dsc_y = &stripe->dsc[OPE_DS_Y_DISP];

	dsc_c->phase_step_h = dsc_y->phase_step_h = h_scale;
	dsc_c->phase_step_v = dsc_y->phase_step_v = v_scale;

	dsc_c->input_width  = stripe->dsc[OPE_DS_C_PRE].output_width;
	dsc_c->input_height = stripe->dsc[OPE_DS_C_PRE].output_height;
	dsc_y->input_width  = stripe->src.width;
	dsc_y->input_height = stripe->src.height;

	dsc_c->output_width  = DS_OUTPUT_PIX(dsc_c->input_width,  0, h_scale);
	dsc_c->output_height = DS_OUTPUT_PIX(dsc_c->input_height, 0, v_scale);
	dsc_y->output_width  = DS_OUTPUT_PIX(dsc_y->input_width,  0, h_scale);
	dsc_y->output_height = DS_OUTPUT_PIX(dsc_y->input_height, 0, v_scale);
}

static void ope_gen_stripe_yuv_dst(struct ope_ctx *ctx,
				   struct ope_stripe *stripe,
				   dma_addr_t dst)
{
	const struct camss_isp_fmt_state *fo = &ctx->mctx->fmt_out;
	unsigned int img_w = fo->width, img_h = fo->height;
	const struct ope_hw_fmt *hw = ope_find_hw_fmt(fo->fmt->fourcc);
	struct ope_stripe *prev = ctx->current_stripe ?
		&ctx->stripe[ctx->current_stripe - 1] : NULL;
	u32 x_init = 0;

	stripe->dst[OPE_WR_CLIENT_DISP_Y].enabled = true;
	stripe->dst[OPE_WR_CLIENT_DISP_C].enabled = true;

	/* Y plane */
	if (prev)
		x_init = prev->dst[OPE_WR_CLIENT_DISP_Y].x_init +
			 prev->dst[OPE_WR_CLIENT_DISP_Y].width;

	stripe->dst[OPE_WR_CLIENT_DISP_Y].addr   = dst;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].x_init = x_init;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].width  =
		stripe->dsc[OPE_DS_Y_DISP].output_width;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].height =
		stripe->dsc[OPE_DS_Y_DISP].output_height;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].stride = img_w;
	stripe->dst[OPE_WR_CLIENT_DISP_Y].format = OPE_PACKER_FMT_PLAIN_8;

	/* UV plane */
	x_init = 0;
	if (prev)
		x_init = prev->dst[OPE_WR_CLIENT_DISP_C].x_init +
			 prev->dst[OPE_WR_CLIENT_DISP_C].width;

	stripe->dst[OPE_WR_CLIENT_DISP_C].addr   = dst + img_w * img_h;
	stripe->dst[OPE_WR_CLIENT_DISP_C].x_init = x_init;
	stripe->dst[OPE_WR_CLIENT_DISP_C].format = hw ? hw->packer : OPE_PACKER_FMT_PLAIN_8;
	stripe->dst[OPE_WR_CLIENT_DISP_C].width  =
		stripe->dsc[OPE_DS_C_DISP].output_width * 2;
	stripe->dst[OPE_WR_CLIENT_DISP_C].height =
		stripe->dsc[OPE_DS_C_DISP].output_height;

	switch (fo->fmt->fourcc) {
	case V4L2_PIX_FMT_NV42: case V4L2_PIX_FMT_NV24:
		stripe->dst[OPE_WR_CLIENT_DISP_C].stride = img_w * 2;
		break;
	case V4L2_PIX_FMT_GREY:
		stripe->dst[OPE_WR_CLIENT_DISP_C].enabled = false;
		break;
	default:
		stripe->dst[OPE_WR_CLIENT_DISP_C].stride = img_w;
	}
}

static void ope_gen_stripe_argb_dst(struct ope_ctx *ctx,
				    struct ope_stripe *stripe,
				    dma_addr_t dst)
{
	unsigned int img_w = ctx->mctx->fmt_out.width;
	dma_addr_t addr;

	stripe->dst[OPE_WR_CLIENT_ARGB].enabled = true;

	if (!ctx->current_stripe) {
		addr = dst;
	} else {
		struct ope_stripe *prev = &ctx->stripe[ctx->current_stripe - 1];

		addr = prev->dst[OPE_WR_CLIENT_ARGB].addr +
		       prev->dst[OPE_WR_CLIENT_ARGB].width * 8;
	}

	stripe->dst[OPE_WR_CLIENT_ARGB].addr   = addr;
	stripe->dst[OPE_WR_CLIENT_ARGB].x_init = 0;
	stripe->dst[OPE_WR_CLIENT_ARGB].width  = stripe->src.width;
	stripe->dst[OPE_WR_CLIENT_ARGB].height = stripe->src.height;
	stripe->dst[OPE_WR_CLIENT_ARGB].stride = img_w * 8;
	stripe->dst[OPE_WR_CLIENT_ARGB].format = OPE_PACKER_FMT_PLAIN_64;
}

static void ope_gen_stripes(struct ope_ctx *ctx, dma_addr_t src, dma_addr_t dst)
{
	const struct camss_isp_fmt_state *fi = &ctx->mctx->fmt_in;
	const struct camss_isp_fmt_state *fo = &ctx->mctx->fmt_out;
	const struct ope_hw_fmt *src_hw = ope_find_hw_fmt(fi->fmt->fourcc);
	unsigned int num_stripes, width, i;
	u32 h_scale, v_scale;

	width      = fi->width;
	num_stripes = DIV_ROUND_UP(fi->width, OPE_STRIPE_MAX_W);
	h_scale    = DS_Q21(fi->width,  fo->width);
	v_scale    = DS_Q21(fi->height, fo->height);

	for (i = 0; i < num_stripes; i++) {
		struct ope_stripe *stripe = &ctx->stripe[i];

		memset(stripe, 0, sizeof(*stripe));

		stripe->src.addr     = src;
		stripe->src.width    = width;
		stripe->src.height   = fi->height;
		stripe->src.stride   = fi->bytesperline;
		stripe->src.location = ope_stripe_location(i, num_stripes);
		stripe->src.pattern  = src_hw ? src_hw->pattern : 0;
		stripe->src.format   = src_hw ? src_hw->unpacker : 0;

		/* Ensure last stripe is wide enough */
		if (width > OPE_STRIPE_MAX_W &&
		    width < OPE_STRIPE_MAX_W + OPE_STRIPE_MIN_W)
			stripe->src.width -= OPE_STRIPE_MIN_W * 2;

		v4l_bound_align_image(&stripe->src.width,
				      OPE_STRIPE_MIN_W, OPE_STRIPE_MAX_W,
				      fi->fmt->align,
				      &stripe->src.height,
				      OPE_STRIPE_MIN_H, OPE_STRIPE_MAX_H,
				      OPE_ALIGN_H, 0);

		width -= stripe->src.width;
		src   += stripe->src.width * fi->fmt->depth / 8;

		if (ope_pix_fmt_is_yuv(fo->fmt->fourcc)) {
			ope_gen_stripe_chroma_dsc(ctx, stripe);
			ope_gen_stripe_dsc(ctx, stripe, h_scale, v_scale);
			ope_gen_stripe_yuv_dst(ctx, stripe, dst);
		} else {
			ope_gen_stripe_argb_dst(ctx, stripe, dst);
		}

		/* Width in bytes for the fetch engine */
		stripe->src.width = stripe->src.width * fi->fmt->depth / 8;
	}
}

/* ---- Pipeline module programming -------------------------------------- */

/*
 * ope_module_update() - conditionally program a pipeline module
 *
 * Returns true if the module was programmed (block is enabled and dirty),
 * false if it was skipped.
 */
static bool ope_module_update(struct ope_ctx *ctx, u32 module_cfg_reg,
			      u32 enable_mask,
			      const struct v4l2_isp_params_block_header *hdr)
{
	struct ope_dev *ope = ctx->ope;
	bool enable = hdr->flags & V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	bool dirty  = hdr->flags & CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;

	ope_write_pp(ope, module_cfg_reg, enable ? enable_mask : 0);

	if (!enable || !dirty)
		return false;

	return true;
}

static void ope_prog_wb(struct ope_ctx *ctx)
{
	const struct camss_params_wb_gain *wb = &ctx->config.wb_gain;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_WB_GAIN_MODULE_CFG,
			       OPE_PP_CLC_WB_GAIN_MODULE_CFG_EN,
			       &wb->header))
		return;

	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(0), wb->g_gain);
	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(1), wb->b_gain);
	ope_write_pp(ope, OPE_PP_CLC_WB_GAIN_WB_CFG(2), wb->r_gain);
}

static void ope_prog_bayer2rgb(struct ope_ctx *ctx)
{
	const struct camss_params_demo *demo = &ctx->config.demo;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_DEMO_MODULE_CFG,
			       OPE_PP_CLC_DEMO_MODULE_CFG_EN |
			       OPE_PP_CLC_DEMO_MODULE_CFG_DYN_G_CLAMP_EN,
			       &demo->header))
		return;

	ope_write_pp(ope, OPE_PP_CLC_DEMO_INTERP_COEFF_CFG,
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_G,  demo->lambda_g) |
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_COEFF_CFG_LAMBDA_RB, demo->lambda_rb));
	ope_write_pp(ope, OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0,
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_0_AK, demo->a_k));
	ope_write_pp(ope, OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1,
		     FIELD_PREP(OPE_PP_CLC_DEMO_INTERP_CLASSIFIER_CFG_1_WK, demo->w_k));
}

static void ope_prog_rgb2yuv(struct ope_ctx *ctx)
{
	const struct camss_params_chroma_enhan *cc = &ctx->config.chroma_enhan;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG,
			       OPE_PP_CLC_CHROMA_ENHAN_MODULE_CFG_EN,
			       &cc->header))
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

static void ope_prog_color_correct(struct ope_ctx *ctx)
{
	const struct camss_params_color_correct *cc = &ctx->config.color_correct;
	struct ope_dev *ope = ctx->ope;

	if (!ope_module_update(ctx, OPE_PP_CLC_CC_MODULE_CFG,
			       OPE_PP_CLC_CC_MODULE_CFG_EN,
			       &cc->header))
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
	ope_write_pp(ope, OPE_PP_CLC_CC_COEFF_M_CFG,
		     FIELD_PREP(OPE_PP_CLC_CC_COEFF_M_CFG_M, cc->m));
}

static void ope_prog_stripe(struct ope_ctx *ctx, struct ope_stripe *stripe)
{
	struct ope_dev *ope = ctx->ope;
	int i;

	dev_dbg(ope->dev, "ctx=%p programming stripe %u\n",
		ctx, (unsigned int)(stripe - ctx->stripe));

	/* Fetch Engine */
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_UNPACK_CFG_0, stripe->src.format);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_RD_BUFFER_SIZE,
		     (stripe->src.width << 16) | stripe->src.height);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_ADDR_IMAGE, stripe->src.addr);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_RD_STRIDE, stripe->src.stride);
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_CCIF_META_DATA,
		     FIELD_PREP(OPE_BUS_RD_CLIENT_0_CCIF_MD_PIX_PATTERN,
				stripe->src.pattern));
	ope_write_rd(ope, OPE_BUS_RD_CLIENT_0_CORE_CFG,
		     OPE_BUS_RD_CLIENT_0_CORE_CFG_EN);

	/* Write Engines */
	for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
		if (!stripe->dst[i].enabled) {
			ope_write_wr(ope, OPE_BUS_WR_CLIENT_CFG(i), 0);
			continue;
		}
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_ADDR_IMAGE(i),
			     stripe->dst[i].addr);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_0(i),
			     (stripe->dst[i].height << 16) | stripe->dst[i].width);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_1(i),
			     stripe->dst[i].x_init);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_IMAGE_CFG_2(i),
			     stripe->dst[i].stride);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_PACKER_CFG(i),
			     stripe->dst[i].format);
		ope_write_wr(ope, OPE_BUS_WR_CLIENT_CFG(i),
			     OPE_BUS_WR_CLIENT_CFG_EN |
			     OPE_BUS_WR_CLIENT_CFG_AUTORECOVER);
	}

	/* Downscalers */
	for (i = 0; i < OPE_DS_MAX; i++) {
		struct ope_dsc_config *dsc = &stripe->dsc[i];
		u32 base = ope_ds_base[i];
		u32 cfg = 0;

		if (dsc->input_width != dsc->output_width) {
			dsc->phase_step_h |=
				DS_RESOLUTION(dsc->input_width,
					      dsc->output_width) << 30;
			cfg |= OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_H_SCALE_EN;
		}
		if (dsc->input_height != dsc->output_height) {
			dsc->phase_step_v |=
				DS_RESOLUTION(dsc->input_height,
					      dsc->output_height) << 30;
			cfg |= OPE_PP_CLC_DOWNSCALE_MN_DS_CFG_V_SCALE_EN;
		}

		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_CFG(base), cfg);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_IMAGE_SIZE_CFG(base),
			     ((dsc->input_width - 1) << 16) | (dsc->input_height - 1));
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_MN_H_CFG(base),
			     dsc->phase_step_h);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_DS_MN_V_CFG(base),
			     dsc->phase_step_v);
		ope_write_pp(ope, OPE_PP_CLC_DOWNSCALE_MN_CFG(base),
			     cfg ? OPE_PP_CLC_DOWNSCALE_MN_CFG_EN : 0);
	}
}

/* ---- Parameter application -------------------------------------------- */

static void ope_params_apply_wb(void *priv,
				const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	ctx->config.wb_gain = block->wb_gain;
	ctx->config.wb_gain.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static void ope_params_apply_demo(void *priv,
				  const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	ctx->config.demo = block->demo;
	ctx->config.demo.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static void ope_params_apply_chroma_enhan(void *priv,
					  const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	ctx->config.chroma_enhan = block->chroma_enhan;
	ctx->config.chroma_enhan.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static void ope_params_apply_color_correct(void *priv,
					   const union camss_isp_params_block *block)
{
	struct ope_ctx *ctx = priv;

	ctx->config.color_correct = block->color_correct;
	ctx->config.color_correct.header.flags |= CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY;
}

static const struct v4l2_isp_params_block_type_info ope_params_type_info[] = {
	[CAMSS_PARAMS_WB_GAIN]      = { sizeof(struct camss_params_wb_gain) },
	[CAMSS_PARAMS_DEMO]         = { sizeof(struct camss_params_demo) },
	[CAMSS_PARAMS_CHROMA_ENHAN] = { sizeof(struct camss_params_chroma_enhan) },
	[CAMSS_PARAMS_COLOR_CORRECT]= { sizeof(struct camss_params_color_correct) },
};

static const camss_isp_params_handler_fn ope_params_handlers[] = {
	[CAMSS_PARAMS_WB_GAIN]      = ope_params_apply_wb,
	[CAMSS_PARAMS_DEMO]         = ope_params_apply_demo,
	[CAMSS_PARAMS_CHROMA_ENHAN] = ope_params_apply_chroma_enhan,
	[CAMSS_PARAMS_COLOR_CORRECT]= ope_params_apply_color_correct,
};

static void ope_apply_params(struct ope_ctx *ctx)
{
	struct vb2_v4l2_buffer *vbuf;

	vbuf = v4l2_isp_m2m_next_buf(v4l2_isp_m2m_get_q_ctx(ctx->mctx->m2m_ctx,
							     CAMSS_ISP_M2M_QUEUE_PARAMS));
	if (!vbuf)
		return;

	camss_isp_params_apply(ctx->ope->dev, &vbuf->vb2_buf,
			       ope_params_type_info,
			       ope_params_handlers,
			       ARRAY_SIZE(ope_params_handlers),
			       ctx);
}

/* ---- Default tuning parameters ---------------------------------------- */

static const struct ope_config ope_default_config = {
	.wb_gain = {
		.header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
				CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY,
		.g_gain = OPE_WB(1, 1),
		.b_gain = OPE_WB(3, 2),
		.r_gain = OPE_WB(3, 2),
	},
	.demo = {
		.header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
				CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY,
		.lambda_rb = 0,
		.lambda_g  = 128,
		.a_k       = 128,
		.w_k       = 102,
	},
	.chroma_enhan = {
		.header.flags = V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
				CAMSS_ISP_PARAMS_FL_BLOCK_DIRTY,
		.luma_v0  = 0x04d,
		.luma_v1  = 0x096,
		.luma_v2  = 0x01d,
		.luma_k   = 0,
		.coeff_ap = 0x0e6,
		.coeff_am = 0x0e6,
		.coeff_cp = 0x0b3,
		.coeff_cm = 0x0b3,
		.coeff_dp = 0xfb3,
		.coeff_dm = 0xfb3,
		.kcb      = 128,
		.kcr      = 128,
	},
};

/* ---- camss_isp_m2m_hw_ops callbacks ----------------------------------- */

static void ope_device_run(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;
	dma_addr_t src, dst;

	src = camss_isp_m2m_buf_dma_addr(mctx, CAMSS_ISP_M2M_QUEUE_FRAME_IN,  0);
	dst = camss_isp_m2m_buf_dma_addr(mctx, CAMSS_ISP_M2M_QUEUE_FRAME_OUT, 0);

	dev_dbg(ope->dev, "device_run ctx=%p src=%pad dst=%pad\n",
		ctx, &src, &dst);

	/* Apply any pending ISP tuning parameters */
	ope_apply_params(ctx);

	/* Generate stripe decomposition */
	ope_gen_stripes(ctx, src, dst);

	/* Program IQ modules (only if context or params changed) */
	if (ctx != ope->hw_ctx) {
		ope_prog_wb(ctx);
		ope_prog_bayer2rgb(ctx);
		ope_prog_rgb2yuv(ctx);
		ope_prog_color_correct(ctx);
		ope->hw_ctx = ctx;
	}

	/* Program first stripe and kick hardware */
	ctx->current_stripe = 0;
	ope_prog_stripe(ctx, &ctx->stripe[0]);
	ope_start(ope);
}

static void ope_job_abort(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_ctx *ctx = mctx->drv_priv;

	dev_dbg(ctx->ope->dev, "job_abort ctx=%p\n", ctx);
	ope_write(ctx->ope, OPE_TOP_RESET_CMD, OPE_TOP_RESET_CMD_SW);
}

static int ope_streaming_start(struct camss_isp_m2m_ctx *mctx,
				unsigned int queue_idx)
{
	struct ope_ctx *ctx = mctx->drv_priv;
	struct ope_dev *ope = ctx->ope;

	dev_dbg(ope->dev, "streaming_start ctx=%p queue=%u\n", ctx, queue_idx);

	/* Enable interrupts on first queue to start streaming */
	if (queue_idx == CAMSS_ISP_M2M_QUEUE_FRAME_IN) {
		ope_write(ope, OPE_TOP_IRQ_MASK,
			  OPE_TOP_IRQ_STATUS_RST_DONE |
			  OPE_TOP_IRQ_STATUS_WE |
			  OPE_TOP_IRQ_STATUS_VIOL |
			  OPE_TOP_IRQ_STATUS_IDLE);
		ope_write_wr(ope, OPE_BUS_WR_INPUT_IF_IRQ_MASK_0,
			     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE |
			     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_CONS_VIOL |
			     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL |
			     OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL);
	}

	return 0;
}

static void ope_streaming_stop(struct camss_isp_m2m_ctx *mctx,
				unsigned int queue_idx)
{
	struct ope_ctx *ctx = mctx->drv_priv;

	if (queue_idx == CAMSS_ISP_M2M_QUEUE_FRAME_IN &&
	    ctx->ope->hw_ctx == ctx)
		ctx->ope->hw_ctx = NULL;
}

static int ope_ctx_init(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_dev *ope = container_of(mctx->mdev, struct ope_dev, mdev);
	struct ope_ctx *ctx;

	ctx = kvzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->ope    = ope;
	ctx->mctx   = mctx;
	ctx->config = ope_default_config;
	mctx->drv_priv = ctx;

	dev_dbg(ope->dev, "ctx_init ctx=%p\n", ctx);
	return 0;
}

static void ope_ctx_destroy(struct camss_isp_m2m_ctx *mctx)
{
	struct ope_ctx *ctx = mctx->drv_priv;

	dev_dbg(ctx->ope->dev, "ctx_destroy ctx=%p\n", ctx);

	if (ctx->ope->hw_ctx == ctx)
		ctx->ope->hw_ctx = NULL;

	kvfree(ctx);
}

static const struct camss_isp_m2m_hw_ops ope_hw_ops = {
	.device_run      = ope_device_run,
	.job_abort       = ope_job_abort,
	.streaming_start = ope_streaming_start,
	.streaming_stop  = ope_streaming_stop,
	.ctx_init        = ope_ctx_init,
	.ctx_destroy     = ope_ctx_destroy,
};

/* ---- Interrupt handler ------------------------------------------------ */

static void ope_fe_irq(struct ope_dev *ope)
{
	u32 status = readl_relaxed(ope->base_rd + OPE_BUS_RD_INPUT_IF_IRQ_STATUS);

	writel_relaxed(status, ope->base_rd + OPE_BUS_RD_INPUT_IF_IRQ_CLEAR);
	writel_relaxed(OPE_BUS_RD_INPUT_IF_IRQ_CMD_CLEAR,
		       ope->base_rd + OPE_BUS_RD_INPUT_IF_IRQ_CMD);
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
			"Write Engine: configuration constraint violation\n");
		ope_job_abort(ctx->mctx);
	}

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_IMG_SZ_VIOL) {
		u32 viol = ope_read_wr(ope, OPE_BUS_WR_IMAGE_SIZE_VIOLATION_STATUS);
		int i;

		for (i = 0; i < OPE_WR_CLIENT_MAX; i++) {
			if (BIT(i) & viol)
				dev_err_ratelimited(ope->dev,
					"Write Engine WE%d: image size violation\n", i);
		}
		ope_job_abort(ctx->mctx);
	}

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_VIOL) {
		dev_err_ratelimited(ope->dev, "Write Engine: fatal violation\n");
		ope_job_abort(ctx->mctx);
	}

	if (status & OPE_BUS_WR_INPUT_IF_IRQ_STATUS_0_RUP_DONE) {
		/* Register update done: program next stripe (double-buffered) */
		struct ope_stripe *stripe = ope_current_stripe(ctx);

		if (stripe && !ope_stripe_is_last(stripe))
			ope_prog_stripe(ctx, stripe + 1);
	}
}

static irqreturn_t ope_irq_thread(int irq, void *dev_id)
{
	struct ope_dev *ope = dev_id;
	struct ope_ctx *ctx;
	u32 status;

	guard(pm_runtime_active)(ope->dev);
	//guard(mutex)(&ope->mdev.mutex);

	ctx = ope->hw_ctx;

	status = ope_read(ope, OPE_TOP_IRQ_STATUS);
	ope_write(ope, OPE_TOP_IRQ_CLEAR, status);
	ope_write(ope, OPE_TOP_IRQ_CMD, OPE_TOP_IRQ_CMD_CLEAR);

	if (status & OPE_TOP_IRQ_STATUS_RST_DONE) {
		dev_dbg(ope->dev, "reset done ctx=%p\n", ctx);
		if (ctx)
			camss_isp_m2m_job_finish(ctx->mctx, VB2_BUF_STATE_ERROR);
		complete(&ope->reset_complete);
	}

	if (status & OPE_TOP_IRQ_STATUS_VIOL)
		dev_warn(ope->dev, "OPE violation: 0x%08x\n",
			 ope_read(ope, OPE_TOP_VIOLATION_STATUS));

	if (status & OPE_TOP_IRQ_STATUS_FE)
		ope_fe_irq(ope);

	if (status & OPE_TOP_IRQ_STATUS_WE)
		ope_we_irq(ope, ctx);

	if ((status & OPE_TOP_IRQ_STATUS_IDLE) && ctx) {
		struct ope_stripe *stripe = ope_current_stripe(ctx);

		dev_dbg(ope->dev, "stripe %u done ctx=%p\n",
			ctx->current_stripe, ctx);

		if (ope_stripe_is_last(stripe)) {
			ctx->current_stripe = 0;
			camss_isp_m2m_job_finish(ctx->mctx, VB2_BUF_STATE_DONE);
		} else {
			ctx->current_stripe++;
			ope_start(ope);
		}
	}

	return IRQ_HANDLED;
}

/* ---- Hardware initialisation ------------------------------------------ */

static int ope_soft_reset(struct ope_dev *ope)
{
	u32 version;
	int ret;

	ret = pm_runtime_resume_and_get(ope->dev);
	if (ret)
		return dev_err_probe(ope->dev, ret, "pm_runtime_resume failed\n");

	version = ope_read(ope, OPE_TOP_HW_VERSION);
	dev_dbg(ope->dev, "HW version %u.%u.%u\n",
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_GEN, version),
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_REV, version),
		(u32)FIELD_GET(OPE_TOP_HW_VERSION_STEP, version));

	reinit_completion(&ope->reset_complete);
	ope_write(ope, OPE_TOP_RESET_CMD, OPE_TOP_RESET_CMD_SW);

	if (!wait_for_completion_timeout(&ope->reset_complete,
					 msecs_to_jiffies(OPE_RESET_TIMEOUT_MS))) {
		dev_err(ope->dev, "Reset timeout\n");
		pm_runtime_put(ope->dev);
		return -ETIMEDOUT;
	}

	pm_runtime_put(ope->dev);
	return 0;
}

static int ope_init_power(struct ope_dev *ope)
{
	struct dev_pm_domain_list *pmdomains;
	struct device *dev = ope->dev;
	int ret;

	ope->mdev.icc_data = devm_of_icc_get(dev, "data");
	if (IS_ERR(ope->mdev.icc_data))
		return dev_err_probe(dev, PTR_ERR(ope->mdev.icc_data),
				     "failed to get interconnect data path\n");

	ope->mdev.icc_config = devm_of_icc_get(dev, "config");
	if (IS_ERR(ope->mdev.icc_config))
		return dev_err_probe(dev, PTR_ERR(ope->mdev.icc_config),
				     "failed to get interconnect config path\n");

	devm_pm_domain_attach_list(dev, NULL, &pmdomains);

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

/* ---- Platform driver -------------------------------------------------- */

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
	if (ret)
		return ret;

	ret = camss_isp_m2m_dev_init(&ope->mdev, dev, &ope_hw_ops,
				     ope_input_fmts,  ARRAY_SIZE(ope_input_fmts),
				     ope_output_fmts, ARRAY_SIZE(ope_output_fmts),
				     OPE_MIN_W, OPE_MAX_W,
				     OPE_MIN_H, OPE_MAX_H,
				     OPE_NAME);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to init ISP m2m device\n");

	platform_set_drvdata(pdev, ope);
	return 0;
}

static void ope_remove(struct platform_device *pdev)
{
	struct ope_dev *ope = platform_get_drvdata(pdev);

	camss_isp_m2m_dev_cleanup(&ope->mdev);
}

static const struct of_device_id ope_dt_ids[] = {
	{ .compatible = "qcom,qcm2290-camss-ope" },
	{ },
};
MODULE_DEVICE_TABLE(of, ope_dt_ids);

static const struct dev_pm_ops ope_pm_ops = {
	SET_RUNTIME_PM_OPS(pm_clk_suspend, pm_clk_resume, NULL)
};

static struct platform_driver ope_driver = {
	.probe  = ope_probe,
	.remove = ope_remove,
	.driver = {
		.name           = OPE_NAME,
		.of_match_table = ope_dt_ids,
		.pm             = &ope_pm_ops,
	},
};

module_platform_driver(ope_driver);

MODULE_DESCRIPTION("CAMSS Offline Processing Engine");
MODULE_AUTHOR("Loic Poulain <loic.poulain@oss.qualcomm.com>");
MODULE_LICENSE("GPL");
