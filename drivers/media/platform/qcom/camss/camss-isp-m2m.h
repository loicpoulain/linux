/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-isp-m2m.h
 *
 * Qualcomm CAMSS shared layer for offline ISP mem-to-mem engines.
 *
 * Sits on top of the generic v4l2-isp-m2m framework and provides the
 * CAMSS-specific plumbing that is common to all offline processing engines
 * (OPE, IPE, …): shared-context open/release, vb2 ops, format state,
 * ioctl ops, and power/bandwidth scaling helpers.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef CAMSS_ISP_M2M_H
#define CAMSS_ISP_M2M_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-isp-m2m.h>
#include <media/videobuf2-v4l2.h>

struct camss_isp_m2m_dev;
struct camss_isp_m2m_ctx;

/* ---- Queue indices ---------------------------------------------------- */

/*
 * Standard queue layout used by all CAMSS offline engines.
 * Drivers that need additional queues extend this enum.
 */
enum camss_isp_m2m_queue_idx {
	CAMSS_ISP_M2M_QUEUE_FRAME_IN  = 0,	/* Bayer / YUV input frame  */
	CAMSS_ISP_M2M_QUEUE_FRAME_OUT = 1,	/* Processed output frame   */
	CAMSS_ISP_M2M_QUEUE_PARAMS    = 2,	/* ISP tuning parameters    */
	CAMSS_ISP_M2M_QUEUE_COUNT,		/* must be last             */
};

/* ---- Format descriptor ------------------------------------------------ */

/**
 * struct camss_isp_fmt - generic ISP pixel/meta format descriptor
 *
 * @fourcc:	V4L2 pixel format identifier
 * @depth:	bits per pixel (for stride/size calculations)
 * @align:	horizontal pixel alignment expressed as log2 (0 = 1 pixel)
 * @num_planes:	number of DMA planes (0 treated as 1)
 */
struct camss_isp_fmt {
	u32		fourcc;
	unsigned int	depth;
	unsigned int	align;
	unsigned int	num_planes;
};

/* ---- Per-queue format state ------------------------------------------- */

/**
 * struct camss_isp_fmt_state - negotiated format for one queue
 *
 * Populated by the shared ioctl handlers; read by the driver's device_run.
 */
struct camss_isp_fmt_state {
	const struct camss_isp_fmt	*fmt;
	unsigned int			width;
	unsigned int			height;
	unsigned int			bytesperline;
	unsigned int			sizeimage;
	enum v4l2_colorspace		colorspace;
	enum v4l2_xfer_func		xfer_func;
	enum v4l2_ycbcr_encoding	ycbcr_enc;
	enum v4l2_quantization		quantization;
	unsigned int			sequence;
	struct v4l2_fract		timeperframe;
};

/* ---- Driver callbacks ------------------------------------------------- */

/**
 * struct camss_isp_m2m_hw_ops - hardware-specific callbacks
 *
 * @device_run:		Required. Start one job. Call camss_isp_m2m_job_finish()
 *			when done (may be from IRQ context).
 * @job_abort:		Optional. Abort the running job as fast as possible.
 * @streaming_start:	Optional. Called on the first streamon for a queue.
 *			@queue_idx is the CAMSS_ISP_M2M_QUEUE_* index.
 * @streaming_stop:	Optional. Called on streamoff.
 * @ctx_init:		Optional. Allocate driver-private per-context state.
 *			Return 0 on success; ctx->drv_priv is set before the call.
 * @ctx_destroy:	Optional. Free driver-private per-context state.
 */
struct camss_isp_m2m_hw_ops {
	void (*device_run)(struct camss_isp_m2m_ctx *ctx);
	void (*job_abort)(struct camss_isp_m2m_ctx *ctx);

	int  (*streaming_start)(struct camss_isp_m2m_ctx *ctx,
				unsigned int queue_idx);
	void (*streaming_stop)(struct camss_isp_m2m_ctx *ctx,
			       unsigned int queue_idx);

	int  (*ctx_init)(struct camss_isp_m2m_ctx *ctx);
	void (*ctx_destroy)(struct camss_isp_m2m_ctx *ctx);
};

/* ---- Per-context state ------------------------------------------------ */

/**
 * struct camss_isp_m2m_ctx - per-open-instance context
 *
 * @m2m_ctx:	underlying v4l2-isp-m2m context
 * @mdev:	back-pointer to the CAMSS ISP m2m device
 * @fmt_in:	negotiated format for the frame-input queue
 * @fmt_out:	negotiated format for the frame-output queue
 * @drv_priv:	driver-private data (set by ctx_init)
 * @list:	entry in mdev->ctx_list
 * @started:	true once the input queue has started streaming
 */
struct camss_isp_m2m_ctx {
	struct v4l2_isp_m2m_ctx	*m2m_ctx;
	struct camss_isp_m2m_dev	*mdev;

	struct camss_isp_fmt_state	fmt_in;
	struct camss_isp_fmt_state	fmt_out;

	void				*drv_priv;
	struct list_head		list;
	bool				started;
};

/* ---- Device state ----------------------------------------------------- */

/**
 * struct camss_isp_m2m_dev - CAMSS offline ISP m2m device
 *
 * @dev:	underlying Linux device
 * @m2m_dev:	v4l2-isp-m2m device handle
 * @mutex:	serialises open/release and ioctl calls
 * @hw_ops:	hardware callbacks
 * @input_fmts:	supported input pixel formats
 * @num_input_fmts: number of entries in @input_fmts
 * @output_fmts:	supported output pixel formats
 * @num_output_fmts: number of entries in @output_fmts
 * @min_width / @max_width / @min_height / @max_height: frame size limits
 * @ctx_list:	list of all open contexts
 * @shared_ctx:	single shared context (all opens share one pipeline)
 * @open_count:	number of currently open file descriptors
 * @icc_data:	interconnect path for frame data
 * @icc_config:	interconnect path for configuration
 */
struct camss_isp_m2m_dev {
	struct device			*dev;
	struct v4l2_isp_m2m_dev		*m2m_dev;

	struct mutex			mutex;
	const struct camss_isp_m2m_hw_ops *hw_ops;

	const struct camss_isp_fmt	*input_fmts;
	unsigned int			num_input_fmts;
	const struct camss_isp_fmt	*output_fmts;
	unsigned int			num_output_fmts;

	unsigned int			min_width;
	unsigned int			max_width;
	unsigned int			min_height;
	unsigned int			max_height;

	struct list_head		ctx_list;
	struct camss_isp_m2m_ctx	*shared_ctx;
	unsigned int			open_count;

	struct icc_path			*icc_data;
	struct icc_path			*icc_config;
};

/* ---- Buffer wrapper --------------------------------------------------- */

/**
 * struct camss_isp_m2m_buffer - vb2 buffer wrapper
 *
 * Must be used as the buf_struct_size in all vb2 queues managed by this layer.
 */
struct camss_isp_m2m_buffer {
	struct v4l2_isp_m2m_buffer	base;	/* must be first */
};

/**
 * struct camss_isp_m2m_queue_priv - per-queue drv_priv passed to vb2
 *
 * Holds both the context pointer and the queue index so that the shared
 * vb2 ops can identify which queue they are operating on.
 */
struct camss_isp_m2m_queue_priv {
	struct camss_isp_m2m_ctx	*ctx;
	unsigned int			queue_idx;
};

/* ---- Accessors -------------------------------------------------------- */

/**
 * camss_isp_m2m_ctx_from_file() - retrieve the shared context from a file
 */
static inline struct camss_isp_m2m_ctx *
camss_isp_m2m_ctx_from_file(struct file *file)
{
	struct v4l2_isp_m2m_ctx *m2m_ctx = file_to_v4l2_fh(file)->isp_m2m_ctx;

	return (struct camss_isp_m2m_ctx *)v4l2_isp_m2m_ctx_priv(m2m_ctx);
}

/**
 * camss_isp_m2m_buf_dma_addr() - DMA address of a buffer plane
 * @ctx:	CAMSS context
 * @queue_idx:	CAMSS_ISP_M2M_QUEUE_* index
 * @plane:	plane index (0 for single-plane formats)
 *
 * Peeks at the next ready buffer on the given queue and returns its DMA
 * address.  Returns 0 if no buffer is available.
 */
dma_addr_t camss_isp_m2m_buf_dma_addr(struct camss_isp_m2m_ctx *ctx,
				       unsigned int queue_idx,
				       unsigned int plane);

/* ---- Job completion --------------------------------------------------- */

/**
 * camss_isp_m2m_job_finish() - signal job completion to the framework
 * @ctx:   CAMSS context whose job just finished
 * @state: vb2 buffer completion state for all active buffers
 *
 * Returns all active buffers with @state and calls v4l2_isp_m2m_job_finish().
 * Safe to call from interrupt context.
 */
void camss_isp_m2m_job_finish(struct camss_isp_m2m_ctx *ctx,
			       enum vb2_buffer_state state);

/* ---- Power scaling ---------------------------------------------------- */

/**
 * camss_isp_m2m_adjust_power() - update OPP and interconnect bandwidth
 * @mdev: CAMSS ISP m2m device
 *
 * Iterates over all started contexts and aggregates pixel clock and
 * interconnect bandwidth requirements.  Must be called with mdev->mutex held.
 */
void camss_isp_m2m_adjust_power(struct camss_isp_m2m_dev *mdev);

/* ---- Device lifecycle ------------------------------------------------- */

/**
 * camss_isp_m2m_dev_init() - initialise a CAMSS ISP m2m device
 * @mdev:		device to initialise (caller-allocated, zeroed)
 * @dev:		Linux device
 * @hw_ops:		hardware callbacks
 * @input_fmts:		supported input formats
 * @num_input_fmts:	number of input formats
 * @output_fmts:	supported output formats
 * @num_output_fmts:	number of output formats
 * @min_w / @max_w / @min_h / @max_h: frame size limits
 *
 * Registers the V4L2 device, media device, and v4l2-isp-m2m device.
 * The caller must have already set up power management (OPP, ICC, PM runtime).
 *
 * Returns 0 on success or a negative error code.
 */
int camss_isp_m2m_dev_init(struct camss_isp_m2m_dev *mdev,
			    struct device *dev,
			    const struct camss_isp_m2m_hw_ops *hw_ops,
			    const struct camss_isp_fmt *input_fmts,
			    unsigned int num_input_fmts,
			    const struct camss_isp_fmt *output_fmts,
			    unsigned int num_output_fmts,
			    unsigned int min_w, unsigned int max_w,
			    unsigned int min_h, unsigned int max_h,
			    const char *name);

/**
 * camss_isp_m2m_dev_cleanup() - tear down a CAMSS ISP m2m device
 * @mdev: device to tear down
 */
void camss_isp_m2m_dev_cleanup(struct camss_isp_m2m_dev *mdev);

/* ---- File operations (to be used in video_device.fops) --------------- */


/* ---- vb2 ops (to be used in vb2_queue.ops) --------------------------- */

extern const struct vb2_ops camss_isp_m2m_vb2_ops;

/* ---- ioctl ops (to be used in video_device.ioctl_ops) ---------------- */

extern const struct v4l2_ioctl_ops camss_isp_m2m_video_ioctl_ops;
extern const struct v4l2_ioctl_ops camss_isp_m2m_meta_ioctl_ops;
extern const struct v4l2_file_operations camss_isp_m2m_fops;

/* Named convenience helpers for the standard frame queues */
static inline struct vb2_v4l2_buffer *
v4l2_isp_m2m_next_src_buf(struct v4l2_isp_m2m_ctx *ctx)
{
	return v4l2_isp_m2m_next_buf(
		v4l2_isp_m2m_get_q_ctx(ctx, CAMSS_ISP_M2M_QUEUE_FRAME_IN));
}

static inline struct vb2_v4l2_buffer *
v4l2_isp_m2m_next_dst_buf(struct v4l2_isp_m2m_ctx *ctx)
{
	return v4l2_isp_m2m_next_buf(
		v4l2_isp_m2m_get_q_ctx(ctx, CAMSS_ISP_M2M_QUEUE_FRAME_OUT));
}

static inline struct vb2_v4l2_buffer *
v4l2_isp_m2m_buf_remove_src(struct v4l2_isp_m2m_ctx *ctx)
{
	return v4l2_isp_m2m_buf_remove(
		v4l2_isp_m2m_get_q_ctx(ctx, CAMSS_ISP_M2M_QUEUE_FRAME_IN));
}

static inline struct vb2_v4l2_buffer *
v4l2_isp_m2m_buf_remove_dst(struct v4l2_isp_m2m_ctx *ctx)
{
	return v4l2_isp_m2m_buf_remove(
		v4l2_isp_m2m_get_q_ctx(ctx, CAMSS_ISP_M2M_QUEUE_FRAME_OUT));
}

#endif /* CAMSS_ISP_M2M_H */
