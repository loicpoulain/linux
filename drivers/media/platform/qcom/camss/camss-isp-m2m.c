// SPDX-License-Identifier: GPL-2.0
/*
 * camss-isp-m2m.c
 *
 * Qualcomm CAMSS shared layer for offline ISP mem-to-mem engines.
 *
 * Provides the CAMSS-specific plumbing that is common to all offline
 * processing engines (OPE, IPE, …): shared-context open/release, vb2 ops,
 * format state management, ioctl ops, and power/bandwidth scaling helpers.
 * Sits on top of the generic v4l2-isp-m2m framework.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp-m2m.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "camss-isp-m2m.h"
#include "camss-isp-params.h"

#define CAMSS_ISP_M2M_NAME	"qcom-camss-isp"

/* ---- Internal helpers ---- */
static inline struct camss_isp_m2m_ctx *ctx_from_vb2(struct vb2_queue *q)
{
	return ((struct camss_isp_m2m_queue_priv *)vb2_get_drv_priv(q))->ctx;
}

static inline unsigned int queue_idx_from_vb2(struct vb2_queue *q)
{
	return ((struct camss_isp_m2m_queue_priv *)vb2_get_drv_priv(q))->queue_idx;
}

static struct camss_isp_fmt_state *ctx_fmt_state(struct camss_isp_m2m_ctx *ctx,
						 unsigned int queue_idx)
{
	switch (queue_idx) {
	case CAMSS_ISP_M2M_QUEUE_FRAME_IN:
		return &ctx->fmt_in;
	case CAMSS_ISP_M2M_QUEUE_FRAME_OUT:
		return &ctx->fmt_out;
	default:
		return NULL;
	}
}

static const struct camss_isp_fmt *
find_fmt(const struct camss_isp_fmt *fmts, unsigned int n, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		if (fmts[i].fourcc == fourcc)
			return &fmts[i];
	return NULL;
}

/*
 * Clamp and align a pixel format.  For output (capture) queues the width and
 * height are additionally bounded by the input frame dimensions.
 */
static const struct camss_isp_fmt *
camss_isp_fmt_try(struct camss_isp_m2m_dev *mdev, bool is_output,
		  struct v4l2_pix_format_mplane *pix,
		  const struct camss_isp_fmt_state *input)
{
	const struct camss_isp_fmt *fmts = is_output ?
		mdev->output_fmts : mdev->input_fmts;
	unsigned int n = is_output ?
		mdev->num_output_fmts : mdev->num_input_fmts;
	const struct camss_isp_fmt *fmt;
	unsigned int max_w, max_h;

	fmt = find_fmt(fmts, n, pix->pixelformat);
	if (!fmt) {
		fmt = &fmts[0];
		pix->pixelformat = fmt->fourcc;
	}

	max_w = mdev->max_width;
	max_h = mdev->max_height;
	if (is_output && input && input->fmt) {
		max_w = min(max_w, input->width);
		max_h = min(max_h, input->height);
	}

	v4l_bound_align_image(&pix->width,  mdev->min_width,  max_w, fmt->align,
			      &pix->height, mdev->min_height, max_h, 0, 0);

	pix->num_planes = fmt->num_planes ? fmt->num_planes : 1;
	pix->field = V4L2_FIELD_NONE;

	if (pix->plane_fmt[0].bytesperline <
	    pix->width * fmt->depth / 8)
		pix->plane_fmt[0].bytesperline =
			pix->width * fmt->depth / 8;

	pix->plane_fmt[0].sizeimage =
		(u64)fmt->depth * pix->width * pix->height / 8;

	return fmt;
}

/* ---- Power scaling ---------------------------------------------------- */

void camss_isp_m2m_adjust_power(struct camss_isp_m2m_dev *mdev)
{
	unsigned long pixclk = 0;
	unsigned int loadavg = 0, loadpeak = 0, loadconfig = 0;
	struct camss_isp_m2m_ctx *ctx;
	int ret;

	lockdep_assert_held(&mdev->mutex);

	list_for_each_entry(ctx, &mdev->ctx_list, list) {
		const struct camss_isp_fmt_state *fi = &ctx->fmt_in;
		const struct camss_isp_fmt_state *fo = &ctx->fmt_out;
		unsigned int fps;

		if (!ctx->started || !fi->fmt)
			continue;

		fps = fi->timeperframe.denominator && fi->timeperframe.numerator ?
			fi->timeperframe.denominator / fi->timeperframe.numerator :
			30;

		pixclk += (unsigned long)fi->width * fi->height * fps;
		loadavg += mult_frac(fi->sizeimage, fps, 1000);
		loadavg += mult_frac(fo->sizeimage, fps, 1000);
		loadpeak += mult_frac(fi->sizeimage, fps * 2, 1000);
		loadpeak += mult_frac(fo->sizeimage, fps * 2, 1000);
		/* Config bandwidth: ~50 registers per stripe, 4 bytes each */
		loadconfig += mult_frac((fi->width / 336 + 1) * 50 * 4, fps, 1000);
	}

	/* 30% margin */
	pixclk = mult_frac(pixclk, 13, 10);

	dev_dbg(mdev->dev, "power: clk=%luHz avg=%uKBps peak=%uKBps cfg=%uKBps\n",
		pixclk, loadavg, loadpeak, loadconfig);

	ret = dev_pm_opp_set_rate(mdev->dev, pixclk);
	if (ret)
		dev_warn(mdev->dev, "Failed to set OPP rate: %d\n", ret);

	if (mdev->icc_data) {
		ret = icc_set_bw(mdev->icc_data, loadavg, loadpeak);
		if (ret)
			dev_warn(mdev->dev, "Failed to set data BW: %d\n", ret);
	}

	if (mdev->icc_config) {
		ret = icc_set_bw(mdev->icc_config, loadconfig, loadconfig * 5);
		if (ret)
			dev_warn(mdev->dev, "Failed to set config BW: %d\n", ret);
	}
}
EXPORT_SYMBOL_GPL(camss_isp_m2m_adjust_power);

/* ---- Buffer DMA address helper --------------------------------------- */

dma_addr_t camss_isp_m2m_buf_dma_addr(struct camss_isp_m2m_ctx *ctx, unsigned int queue_idx,
			       unsigned int plane)
{
	struct vb2_v4l2_buffer *vbuf;

	vbuf = v4l2_isp_m2m_next_buf(v4l2_isp_m2m_get_q_ctx(ctx->m2m_ctx, queue_idx));
	if (!vbuf)
		return 0;

	return vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, plane);
}
EXPORT_SYMBOL_GPL(camss_isp_m2m_buf_dma_addr);

/* ---- Job completion --------------------------------------------------- */

void camss_isp_m2m_job_finish(struct camss_isp_m2m_ctx *ctx, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *src, *dst;
	unsigned int i;

	/* Remove and complete all active buffers */
	src = v4l2_isp_m2m_buf_remove_src(ctx->m2m_ctx);
	dst = v4l2_isp_m2m_buf_remove_dst(ctx->m2m_ctx);

	/* Propagate timestamp from input to output */
	if (src && dst)
		dst->vb2_buf.timestamp = src->vb2_buf.timestamp;

	if (dst)
		v4l2_isp_m2m_buf_done(dst, state);
	if (src)
		v4l2_isp_m2m_buf_done(src, state);

	/* Return any params buffer too (optional queue) */
	{
		struct vb2_v4l2_buffer *params =
			v4l2_isp_m2m_buf_remove(v4l2_isp_m2m_get_q_ctx(ctx->m2m_ctx,
								       CAMSS_ISP_M2M_QUEUE_PARAMS));
		if (params)
			v4l2_isp_m2m_buf_done(params, state);
	}

	/* Return any extra driver-defined queues */
	for (i = CAMSS_ISP_M2M_QUEUE_COUNT;
	     i < v4l2_isp_m2m_num_queues(ctx->m2m_ctx); i++) {
		struct vb2_v4l2_buffer *extra =
			v4l2_isp_m2m_buf_remove(v4l2_isp_m2m_get_q_ctx(ctx->m2m_ctx, i));
		if (extra)
			v4l2_isp_m2m_buf_done(extra, state);
	}

	v4l2_isp_m2m_job_finish(ctx->mdev->m2m_dev, ctx->m2m_ctx);
}
EXPORT_SYMBOL_GPL(camss_isp_m2m_job_finish);

/* ---- v4l2-isp-m2m callbacks ------------------------------------------ */

static void camss_isp_m2m_device_run(void *priv)
{
	struct camss_isp_m2m_ctx *ctx = priv;

	ctx->mdev->hw_ops->device_run(ctx);
}

static void camss_isp_m2m_job_abort_cb(void *priv)
{
	struct camss_isp_m2m_ctx *ctx = priv;

	if (ctx->mdev->hw_ops->job_abort)
		ctx->mdev->hw_ops->job_abort(ctx);
}

/* ---- vb2 ops ---------------------------------------------------------- */

static int camss_isp_m2m_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
				     unsigned int *nplanes,
				     unsigned int sizes[],
				     struct device *alloc_devs[])
{
	struct camss_isp_m2m_ctx *ctx = ctx_from_vb2(q);
	unsigned int idx = queue_idx_from_vb2(q);
	struct camss_isp_fmt_state *fs = ctx_fmt_state(ctx, idx);
	unsigned int size;

	if (fs && fs->sizeimage)
		size = fs->sizeimage;
	else if (idx == CAMSS_ISP_M2M_QUEUE_PARAMS)
		size = v4l2_isp_params_buffer_size(CAMSS_PARAMS_MAX_PAYLOAD);
	else
		size = PAGE_SIZE;

	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < size)
			return -EINVAL;
	} else {
		*nplanes = 1;
		sizes[0] = size;
	}

	dev_dbg(ctx->mdev->dev,
		"queue_setup queue=%u nbuffers=%u size=%u\n",
		idx, *nbuffers, sizes[0]);

	return 0;
}

static int camss_isp_m2m_buf_prepare(struct vb2_buffer *vb)
{
	struct camss_isp_m2m_ctx *ctx = ctx_from_vb2(vb->vb2_queue);
	unsigned int idx = queue_idx_from_vb2(vb->vb2_queue);
	struct camss_isp_fmt_state *fs = ctx_fmt_state(ctx, idx);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	unsigned int sizeimage = fs ? fs->sizeimage : PAGE_SIZE;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE) {
			dev_err(ctx->mdev->dev, "unsupported field type\n");
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < sizeimage) {
		dev_err(ctx->mdev->dev, "plane size %lu < required %u\n", vb2_plane_size(vb, 0), sizeimage);
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, sizeimage);

	if (fs)
		vbuf->sequence = fs->sequence++;

	dev_dbg(ctx->mdev->dev,
		"buf_prepare queue=%u index=%u dma=%pad mmap_offset=0x%08llx size=%zu\n",
		idx, vb->index,
		&(dma_addr_t){ vb2_dma_contig_plane_dma_addr(vb, 0) },
		(u64)vb->planes[0].m.offset,
		vb2_plane_size(vb, 0));

	return 0;
}

static void camss_isp_m2m_buf_queue(struct vb2_buffer *vb)
{
	struct camss_isp_m2m_ctx *ctx = ctx_from_vb2(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	v4l2_isp_m2m_buf_queue(ctx->m2m_ctx, vbuf);
}

static int camss_isp_m2m_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camss_isp_m2m_ctx *ctx = ctx_from_vb2(q);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;
	unsigned int idx = queue_idx_from_vb2(q);
	struct camss_isp_fmt_state *fs = ctx_fmt_state(ctx, idx);
	int ret;

	dev_dbg(mdev->dev, "start_streaming ctx=%p queue=%u\n", ctx, idx);

	if (fs)
		fs->sequence = 0;

	if (mdev->hw_ops->streaming_start) {
		ret = mdev->hw_ops->streaming_start(ctx, idx);
		if (ret)
			return ret;
	}

	ret = pm_runtime_resume_and_get(mdev->dev);
	if (ret) {
		dev_err(mdev->dev, "pm_runtime_resume failed: %d\n", ret);
		if (mdev->hw_ops->streaming_stop)
			mdev->hw_ops->streaming_stop(ctx, idx);
		return ret;
	}

	if (idx == CAMSS_ISP_M2M_QUEUE_FRAME_IN) {
		ctx->started = true;
		camss_isp_m2m_adjust_power(mdev);
	}

	return 0;
}

static void camss_isp_m2m_stop_streaming(struct vb2_queue *q)
{
	struct camss_isp_m2m_ctx *ctx = ctx_from_vb2(q);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;
	unsigned int idx = queue_idx_from_vb2(q);

	dev_dbg(mdev->dev, "stop_streaming ctx=%p queue=%u\n", ctx, idx);

	if (idx == CAMSS_ISP_M2M_QUEUE_FRAME_IN) {
		ctx->started = false;
		camss_isp_m2m_adjust_power(mdev);
	}

	if (mdev->hw_ops->streaming_stop)
		mdev->hw_ops->streaming_stop(ctx, idx);

	/*
	 * Drain any buffers still sitting in the ready queue for this queue.
	 * These were queued by userspace but never picked up by device_run.
	 * Return them with ERROR so userspace can clean up properly.
	 */
	{
		struct vb2_v4l2_buffer *vbuf;

		while ((vbuf = v4l2_isp_m2m_buf_remove(v4l2_isp_m2m_get_q_ctx(ctx->m2m_ctx, idx))))
			v4l2_isp_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}

	pm_runtime_put(mdev->dev);
}

const struct vb2_ops camss_isp_m2m_vb2_ops = {
	.queue_setup	 = camss_isp_m2m_queue_setup,
	.buf_prepare	 = camss_isp_m2m_buf_prepare,
	.buf_queue	 = camss_isp_m2m_buf_queue,
	.start_streaming = camss_isp_m2m_start_streaming,
	.stop_streaming  = camss_isp_m2m_stop_streaming,
};
EXPORT_SYMBOL_GPL(camss_isp_m2m_vb2_ops);

/* ---- v4l2-isp-m2m queue_init callback -------------------------------- */

static int camss_isp_m2m_queue_init(void *priv, unsigned int index, struct vb2_queue *q)
{
	struct camss_isp_m2m_ctx *ctx = priv;
	struct camss_isp_m2m_dev *mdev = ctx->mdev;
	bool is_params = (index == CAMSS_ISP_M2M_QUEUE_PARAMS);
	bool is_output = (index == CAMSS_ISP_M2M_QUEUE_FRAME_OUT);

	struct camss_isp_m2m_queue_priv *qpriv;

	qpriv = devm_kzalloc(mdev->dev, sizeof(*qpriv), GFP_KERNEL);
	if (!qpriv)
		return -ENOMEM;

	qpriv->ctx       = ctx;
	qpriv->queue_idx = index;
	q->drv_priv      = qpriv;
	q->ops       = &camss_isp_m2m_vb2_ops;
	q->lock      = &mdev->mutex;
	q->dev       = mdev->dev;
	q->buf_struct_size = sizeof(struct camss_isp_m2m_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->io_modes  = VB2_MMAP | VB2_DMABUF;

	if (is_params) {
		q->type     = V4L2_BUF_TYPE_META_OUTPUT;
		q->mem_ops  = &vb2_vmalloc_memops;
	} else if (is_output) {
		q->type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		q->mem_ops  = &vb2_dma_contig_memops;
	} else {
		q->type     = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q->mem_ops  = &vb2_dma_contig_memops;
	}

	return vb2_queue_init(q);
}

/* ---- ioctl ops -------------------------------------------------------- */

static int camss_isp_m2m_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct camss_isp_m2m_dev *mdev =
		video_drvdata(file) ?
		((struct camss_isp_m2m_ctx *)
		 camss_isp_m2m_ctx_from_file(file))->mdev : NULL;

	strscpy(cap->driver, CAMSS_ISP_M2M_NAME, sizeof(cap->driver));
	if (mdev)
		strscpy(cap->card, dev_name(mdev->dev), sizeof(cap->card));
	return 0;
}

/* --- video output (frame input) format handlers --- */

static int camss_isp_m2m_enum_fmt_vid_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;

	if (f->index >= mdev->num_input_fmts)
		return -EINVAL;

	f->pixelformat = mdev->input_fmts[f->index].fourcc;
	return 0;
}

static int camss_isp_m2m_g_fmt_vid_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_fmt_state *fs = &ctx->fmt_in;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	if (!fs->fmt)
		return -EINVAL;

	pix->pixelformat		= fs->fmt->fourcc;
	pix->width			= fs->width;
	pix->height			= fs->height;
	pix->num_planes			= 1;
	pix->field			= V4L2_FIELD_NONE;
	pix->colorspace			= fs->colorspace;
	pix->xfer_func			= fs->xfer_func;
	pix->ycbcr_enc			= fs->ycbcr_enc;
	pix->quantization		= fs->quantization;
	pix->plane_fmt[0].bytesperline	= fs->bytesperline;
	pix->plane_fmt[0].sizeimage	= fs->sizeimage;
	return 0;
}

static int camss_isp_m2m_try_fmt_vid_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);

	camss_isp_fmt_try(ctx->mdev, false, &f->fmt.pix_mp, NULL);
	return 0;
}

static int camss_isp_m2m_s_fmt_vid_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;
	struct vb2_queue *vq = v4l2_isp_m2m_get_vq(ctx->m2m_ctx, CAMSS_ISP_M2M_QUEUE_FRAME_IN);
	const struct camss_isp_fmt *fmt;

	if (vb2_is_busy(vq))
		return -EBUSY;

	fmt = camss_isp_fmt_try(mdev, false, &f->fmt.pix_mp, NULL);

	ctx->fmt_in.fmt		    = fmt;
	ctx->fmt_in.width	    = f->fmt.pix_mp.width;
	ctx->fmt_in.height	    = f->fmt.pix_mp.height;
	ctx->fmt_in.bytesperline    = f->fmt.pix_mp.plane_fmt[0].bytesperline;
	ctx->fmt_in.sizeimage	    = f->fmt.pix_mp.plane_fmt[0].sizeimage;
	ctx->fmt_in.colorspace	    = f->fmt.pix_mp.colorspace;
	ctx->fmt_in.xfer_func	    = f->fmt.pix_mp.xfer_func;
	ctx->fmt_in.ycbcr_enc	    = f->fmt.pix_mp.ycbcr_enc;
	ctx->fmt_in.quantization    = f->fmt.pix_mp.quantization;
	return 0;
}

/* --- video capture (frame output) format handlers --- */

static int camss_isp_m2m_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;

	if (f->index >= mdev->num_output_fmts)
		return -EINVAL;

	f->pixelformat = mdev->output_fmts[f->index].fourcc;
	return 0;
}

static int camss_isp_m2m_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_fmt_state *fs = &ctx->fmt_out;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	if (!fs->fmt)
		return -EINVAL;

	pix->pixelformat		= fs->fmt->fourcc;
	pix->width			= fs->width;
	pix->height			= fs->height;
	pix->num_planes			= 1;
	pix->field			= V4L2_FIELD_NONE;
	pix->colorspace			= fs->colorspace;
	pix->xfer_func			= fs->xfer_func;
	pix->ycbcr_enc			= fs->ycbcr_enc;
	pix->quantization		= fs->quantization;
	pix->plane_fmt[0].bytesperline	= fs->bytesperline;
	pix->plane_fmt[0].sizeimage	= fs->sizeimage;
	return 0;
}

static int camss_isp_m2m_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);

	camss_isp_fmt_try(ctx->mdev, true, &f->fmt.pix_mp, &ctx->fmt_in);
	return 0;
}

static int camss_isp_m2m_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;
	struct vb2_queue *vq = v4l2_isp_m2m_get_vq(ctx->m2m_ctx, CAMSS_ISP_M2M_QUEUE_FRAME_OUT);
	const struct camss_isp_fmt *fmt;

	if (vb2_is_busy(vq))
		return -EBUSY;

	fmt = camss_isp_fmt_try(mdev, true, &f->fmt.pix_mp, &ctx->fmt_in);

	ctx->fmt_out.fmt	    = fmt;
	ctx->fmt_out.width	    = f->fmt.pix_mp.width;
	ctx->fmt_out.height	    = f->fmt.pix_mp.height;
	ctx->fmt_out.bytesperline   = f->fmt.pix_mp.plane_fmt[0].bytesperline;
	ctx->fmt_out.sizeimage	    = f->fmt.pix_mp.plane_fmt[0].sizeimage;
	ctx->fmt_out.colorspace	    = f->fmt.pix_mp.colorspace;
	ctx->fmt_out.xfer_func	    = f->fmt.pix_mp.xfer_func;
	ctx->fmt_out.ycbcr_enc	    = f->fmt.pix_mp.ycbcr_enc;
	ctx->fmt_out.quantization   = f->fmt.pix_mp.quantization;
	return 0;
}

/* --- frame size / interval --- */

static int camss_isp_m2m_enum_framesizes(struct file *file, void *priv,
					  struct v4l2_frmsizeenum *fsize)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;

	if (fsize->index > 0)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width  = mdev->min_width;
	fsize->stepwise.max_width  = mdev->max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = mdev->min_height;
	fsize->stepwise.max_height = mdev->max_height;
	fsize->stepwise.step_height = 1;
	return 0;
}

static int camss_isp_m2m_g_parm(struct file *file, void *priv, struct v4l2_streamparm *sp)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);

	if (!V4L2_TYPE_IS_OUTPUT(sp->type))
		return -EINVAL;

	sp->parm.output.capability   = V4L2_CAP_TIMEPERFRAME;
	sp->parm.output.timeperframe = ctx->fmt_in.timeperframe;
	return 0;
}

static int camss_isp_m2m_s_parm(struct file *file, void *priv, struct v4l2_streamparm *sp)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct vb2_queue *vq = v4l2_isp_m2m_get_vq(ctx->m2m_ctx, CAMSS_ISP_M2M_QUEUE_FRAME_IN);
	struct v4l2_fract *tpf = &sp->parm.output.timeperframe;

	if (!V4L2_TYPE_IS_OUTPUT(sp->type))
		return -EINVAL;

	if (vb2_is_busy(vq))
		return -EBUSY;

	if (!tpf->denominator)
		tpf->denominator = 1;
	if (!tpf->numerator)
		tpf->numerator = 1;

	ctx->fmt_in.timeperframe    = *tpf;
	sp->parm.output.capability  = V4L2_CAP_TIMEPERFRAME;
	return 0;
}

/* --- buffer management (routed through v4l2-isp-m2m) --- */

/* --- meta format handlers (params queue) --- */

static int camss_isp_m2m_g_fmt_meta(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.meta.dataformat  = V4L2_META_FMT_CAMSS_PARAMS;
	f->fmt.meta.buffersize  =
		v4l2_isp_params_buffer_size(CAMSS_PARAMS_MAX_PAYLOAD);
	return 0;
}

static int camss_isp_m2m_enum_fmt_meta_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_META_FMT_CAMSS_PARAMS;
	return 0;
}

const struct v4l2_ioctl_ops camss_isp_m2m_video_ioctl_ops = {
	.vidioc_querycap		= camss_isp_m2m_querycap,

	.vidioc_enum_fmt_vid_out	= camss_isp_m2m_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out_mplane	= camss_isp_m2m_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out_mplane	= camss_isp_m2m_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out_mplane	= camss_isp_m2m_s_fmt_vid_out,

	.vidioc_enum_fmt_vid_cap	= camss_isp_m2m_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap_mplane	= camss_isp_m2m_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap_mplane	= camss_isp_m2m_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap_mplane	= camss_isp_m2m_s_fmt_vid_cap,

	.vidioc_enum_framesizes		= camss_isp_m2m_enum_framesizes,
	.vidioc_g_parm			= camss_isp_m2m_g_parm,
	.vidioc_s_parm			= camss_isp_m2m_s_parm,

	.vidioc_reqbufs			= v4l2_isp_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_isp_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_isp_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_isp_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_isp_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_isp_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_isp_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_isp_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_isp_m2m_ioctl_streamoff,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};
EXPORT_SYMBOL_GPL(camss_isp_m2m_video_ioctl_ops);

const struct v4l2_ioctl_ops camss_isp_m2m_meta_ioctl_ops = {
	.vidioc_querycap		= camss_isp_m2m_querycap,

	.vidioc_enum_fmt_meta_out	= camss_isp_m2m_enum_fmt_meta_out,
	.vidioc_g_fmt_meta_out		= camss_isp_m2m_g_fmt_meta,
	.vidioc_s_fmt_meta_out		= camss_isp_m2m_g_fmt_meta,
	.vidioc_try_fmt_meta_out	= camss_isp_m2m_g_fmt_meta,

	.vidioc_reqbufs			= v4l2_isp_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_isp_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_isp_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_isp_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_isp_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_isp_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_isp_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_isp_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_isp_m2m_ioctl_streamoff,
};
EXPORT_SYMBOL_GPL(camss_isp_m2m_meta_ioctl_ops);


/* ---- File operations -------------------------------------------------- */

/*
 * Shared-context model: all file descriptors opened against any of the
 * pipeline's video devices share a single v4l2_isp_m2m_ctx and therefore a
 * single set of vb2 queues.  This reflects the hardware reality that OPE (and
 * similar blocks) have a single processing pipeline with no per-client
 * isolation.  The model is intended to be replaced by VIDIOC_BIND_CONTEXT
 * once that interface lands upstream.
 */
/*
 * Shared-context model: all file descriptors opened against any of the
 * pipeline's video devices share a single v4l2_isp_m2m_ctx and therefore a
 * single set of vb2 queues.
 */
static int camss_isp_m2m_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct camss_isp_m2m_dev *mdev = v4l2_isp_m2m_priv(vdev);
	struct camss_isp_m2m_ctx *ctx;
	struct v4l2_fh *fh;
	int ret = 0;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh)
		return -ENOMEM;


	if (mutex_lock_interruptible(&mdev->mutex)) {
		kfree(fh);
		return -ERESTARTSYS;
	}


	dev_dbg(mdev->dev, "open: %s\n", vdev->name);

	if (!mdev->shared_ctx) {
		ctx = kvzalloc(sizeof(*ctx), GFP_KERNEL);
		if (!ctx) {
			ret = -ENOMEM;
			goto unlock;
		}

		ctx->mdev = mdev;

		ctx->fmt_in.fmt  = &mdev->input_fmts[0];
		ctx->fmt_in.width  = mdev->min_width;
		ctx->fmt_in.height = mdev->min_height;
		ctx->fmt_in.bytesperline =
			ctx->fmt_in.width * ctx->fmt_in.fmt->depth / 8;
		ctx->fmt_in.sizeimage =
			ctx->fmt_in.bytesperline * ctx->fmt_in.height;
		ctx->fmt_in.colorspace = V4L2_COLORSPACE_RAW;

		ctx->fmt_out.fmt  = &mdev->output_fmts[0];
		ctx->fmt_out.width  = mdev->min_width;
		ctx->fmt_out.height = mdev->min_height;
		ctx->fmt_out.bytesperline =
			ctx->fmt_out.width * ctx->fmt_out.fmt->depth / 8;
		ctx->fmt_out.sizeimage =
			(u64)ctx->fmt_out.fmt->depth *
			ctx->fmt_out.width * ctx->fmt_out.height / 8;

		ctx->m2m_ctx = v4l2_isp_m2m_ctx_init(mdev->m2m_dev, ctx);
		if (IS_ERR(ctx->m2m_ctx)) {
			ret = PTR_ERR(ctx->m2m_ctx);
			kvfree(ctx);
			goto unlock;
		}

		if (mdev->hw_ops->ctx_init) {
			ret = mdev->hw_ops->ctx_init(ctx);
			if (ret) {
				v4l2_isp_m2m_ctx_release(ctx->m2m_ctx);
				kvfree(ctx);
				goto unlock;
			}
		}

		INIT_LIST_HEAD(&ctx->list);
		list_add(&ctx->list, &mdev->ctx_list);
		mdev->shared_ctx = ctx;
	} else {
		ctx = mdev->shared_ctx;
	}

	v4l2_fh_init(fh, vdev);
	fh->isp_m2m_ctx = ctx->m2m_ctx;
	v4l2_fh_add(fh, file);
	mdev->open_count++;

	dev_dbg(mdev->dev, "open: ctx=%p open_count=%u\n", ctx, mdev->open_count);

unlock:
	if (ret)
		kfree(fh);
	mutex_unlock(&mdev->mutex);
	return ret;
}

static int camss_isp_m2m_release(struct file *file)
{
	struct v4l2_fh *fh = file_to_v4l2_fh(file);
	struct camss_isp_m2m_ctx *ctx =
		(struct camss_isp_m2m_ctx *)(struct camss_isp_m2m_ctx *)v4l2_isp_m2m_ctx_priv(fh->isp_m2m_ctx);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;

	v4l2_fh_del(fh, file);
	v4l2_fh_exit(fh);
	kfree(fh);

	guard(mutex)(&mdev->mutex);

	dev_dbg(mdev->dev, "release: ctx=%p open_count=%u\n", ctx, mdev->open_count);

	if (--mdev->open_count == 0) {
		list_del(&ctx->list);
		v4l2_isp_m2m_ctx_release(ctx->m2m_ctx);
		if (mdev->hw_ops->ctx_destroy)
			mdev->hw_ops->ctx_destroy(ctx);
		kvfree(ctx);
		mdev->shared_ctx = NULL;
	}

	return 0;
}

static __poll_t camss_isp_m2m_poll(struct file *file, poll_table *wait)
{
	return v4l2_isp_m2m_fop_poll(file, wait);
}

static int camss_isp_m2m_mmap(struct file *file, struct vm_area_struct *vma)
{
	return v4l2_isp_m2m_fop_mmap(file, vma);
}

const struct v4l2_file_operations camss_isp_m2m_fops = {
	.owner		= THIS_MODULE,
	.open		= camss_isp_m2m_open,
	.release	= camss_isp_m2m_release,
	.poll		= camss_isp_m2m_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= camss_isp_m2m_mmap,
};
EXPORT_SYMBOL_GPL(camss_isp_m2m_fops);

/* ---- Queue descriptors for the standard 3-queue layout --------------- */

static const struct v4l2_isp_m2m_queue_desc camss_isp_m2m_queue_descs[] = {
	[CAMSS_ISP_M2M_QUEUE_FRAME_IN] = {
		.type      = V4L2_ISP_M2M_QUEUE_SINK_FRAME,
		.buf_type  = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.name      = "frame-input",
		.caps      = V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING,
		.fops      = &camss_isp_m2m_fops,
		.ioctl_ops = &camss_isp_m2m_video_ioctl_ops,
	},
	[CAMSS_ISP_M2M_QUEUE_FRAME_OUT] = {
		.type      = V4L2_ISP_M2M_QUEUE_SOURCE_FRAME,
		.buf_type  = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.name      = "frame-output",
		.caps      = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
		.fops      = &camss_isp_m2m_fops,
		.ioctl_ops = &camss_isp_m2m_video_ioctl_ops,
	},
	[CAMSS_ISP_M2M_QUEUE_PARAMS] = {
		.type      = V4L2_ISP_M2M_QUEUE_SINK_PARAMS,
		.buf_type  = V4L2_BUF_TYPE_META_OUTPUT,
		.name      = "params",
		.caps      = V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING,
		.optional  = true,
		.fops      = &camss_isp_m2m_fops,
		.ioctl_ops = &camss_isp_m2m_meta_ioctl_ops,
	},
};

static const struct v4l2_isp_m2m_ops camss_isp_m2m_ops = {
	.device_run = camss_isp_m2m_device_run,
	.job_abort  = camss_isp_m2m_job_abort_cb,
	.queue_init = camss_isp_m2m_queue_init,
};

/* ---- Device lifecycle ------------------------------------------------- */

int camss_isp_m2m_dev_init(struct camss_isp_m2m_dev *mdev, struct device *dev,
			    const struct camss_isp_m2m_hw_ops *hw_ops,
			    const struct camss_isp_fmt *input_fmts,
			    unsigned int num_input_fmts,
			    const struct camss_isp_fmt *output_fmts,
			    unsigned int num_output_fmts,
			    unsigned int min_w, unsigned int max_w,
			    unsigned int min_h, unsigned int max_h,
			    const char *name)
{
	int ret;

	mdev->dev		= dev;
	mdev->hw_ops		= hw_ops;
	mdev->input_fmts	= input_fmts;
	mdev->num_input_fmts	= num_input_fmts;
	mdev->output_fmts	= output_fmts;
	mdev->num_output_fmts	= num_output_fmts;
	mdev->min_width		= min_w;
	mdev->max_width		= max_w;
	mdev->min_height	= min_h;
	mdev->max_height	= max_h;

	mutex_init(&mdev->mutex);
	INIT_LIST_HEAD(&mdev->ctx_list);

	/* v4l2-isp-m2m device (owns v4l2_dev, mdev, video nodes) */
	mdev->m2m_dev = v4l2_isp_m2m_init(dev, name, &mdev->mutex,
					   mdev,
					   &camss_isp_m2m_ops,
					   camss_isp_m2m_queue_descs,
					   ARRAY_SIZE(camss_isp_m2m_queue_descs));
	if (IS_ERR(mdev->m2m_dev))
		return PTR_ERR(mdev->m2m_dev);

	ret = v4l2_isp_m2m_register(mdev->m2m_dev);
	if (ret)
		goto err_isp;

	return 0;

err_isp:
	v4l2_isp_m2m_release(mdev->m2m_dev);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_isp_m2m_dev_init);

void camss_isp_m2m_dev_cleanup(struct camss_isp_m2m_dev *mdev)
{
	v4l2_isp_m2m_unregister(mdev->m2m_dev);
	v4l2_isp_m2m_release(mdev->m2m_dev);
}
EXPORT_SYMBOL_GPL(camss_isp_m2m_dev_cleanup);

MODULE_DESCRIPTION("Qualcomm CAMSS offline ISP mem-to-mem shared layer");
MODULE_AUTHOR("Loic Poulain <loic.poulain@oss.qualcomm.com>");
MODULE_LICENSE("GPL");
