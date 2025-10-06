// SPDX-License-Identifier: GPL-2.0
/*
 * camss-isp-pipeline.c
 *
 * Qualcomm CAMSS ISP pipeline framework.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "camss-isp-pipeline.h"

#define pipe_dev(pipe) ((pipe)->v4l2_dev.dev)

static enum v4l2_buf_type eptype_to_buf_type(enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		return V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	case CAMSS_ISP_EP_PARAMS:
		return V4L2_BUF_TYPE_META_OUTPUT;
	case CAMSS_ISP_EP_STATS:
		return V4L2_BUF_TYPE_META_CAPTURE;
	default:
		return V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	}
}

static enum vfl_devnode_direction eptype_to_vfl_dir(enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
	case CAMSS_ISP_EP_PARAMS:
		return VFL_DIR_TX;
	default:
		return VFL_DIR_RX;
	}
}

static u32 eptype_to_pad_flags(enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
	case CAMSS_ISP_EP_PARAMS:
		return MEDIA_PAD_FL_SOURCE;
	default:
		return MEDIA_PAD_FL_SINK;
	}
}

static u32 eptype_to_device_caps(enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		return V4L2_CAP_VIDEO_OUTPUT_MPLANE | V4L2_CAP_STREAMING;
	case CAMSS_ISP_EP_PARAMS:
		return V4L2_CAP_META_OUTPUT | V4L2_CAP_STREAMING;
	case CAMSS_ISP_EP_STATS:
		return V4L2_CAP_META_CAPTURE | V4L2_CAP_STREAMING;
	default:
		return V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;
	}
}

static const struct vb2_mem_ops *eptype_to_mem_ops(enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_PARAMS:
		return &vb2_vmalloc_memops;
	default:
		return &vb2_dma_contig_memops;
	}
}

/*
 * Format handlers
 */
static const struct camss_isp_fmt *camss_isp_fmt_try(const struct camss_isp_vnode *vnode,
						     struct v4l2_pix_format_mplane *pix,
						     const struct camss_isp_fmt_state *input)
{
	const struct camss_isp_video_desc *desc = vnode->desc;
	const struct camss_isp_fmt *fmt = NULL;
	unsigned int max_w, max_h;
	unsigned int i;

	for (i = 0; i < desc->num_formats; i++) {
		if (desc->formats[i].fourcc == pix->pixelformat) {
			fmt = &desc->formats[i];
			break;
		}
	}

	if (!fmt) {
		fmt = &desc->formats[0];
		pix->pixelformat = fmt->fourcc;
	}

	max_w = desc->max_width;
	max_h = desc->max_height;
	if (!desc->scaling && input && input->fmt) {
		max_w = min(max_w, input->width);
		max_h = min(max_h, input->height);
	}

	v4l_bound_align_image(&pix->width,  desc->min_width, max_w, fmt->align,
			      &pix->height, desc->min_height, max_h, 0, 0);

	pix->num_planes = fmt->num_planes ? fmt->num_planes : 1;
	pix->field = V4L2_FIELD_NONE;

	if (pix->plane_fmt[0].bytesperline < pix->width * fmt->depth / 8)
		pix->plane_fmt[0].bytesperline = pix->width * fmt->depth / 8;

	pix->plane_fmt[0].sizeimage =
		(u64)fmt->depth * pix->width * pix->height / 8;

	dev_dbg(vnode->vfd.v4l2_dev->dev,
		"%s: fmt_try %ux%u %p4cc bpl=%u size=%u%s\n",
		vnode->desc->name, pix->width, pix->height, &pix->pixelformat,
		pix->plane_fmt[0].bytesperline, pix->plane_fmt[0].sizeimage,
		(!desc->scaling && input && input->fmt) ? " (input-bounded)" : "");

	return fmt;
}

static struct camss_isp_vnode_ctx *isp_vc(struct file *file)
{
	if (!file->private_data)
		return NULL;

	return camss_isp_vnode_ctx_from_file(file);
}

static const struct camss_isp_fmt_state *isp_input_fmt(struct file *file)
{
	struct camss_isp_pipeline_ctx *pctx;
	unsigned int i;

	if (!file->private_data)
		return NULL;

	pctx = camss_isp_pipeline_ctx_from_file(file);
	if (!pctx)
		return NULL;

	for (i = 0; i < pctx->num_vnodes; i++) {
		if (pctx->vnode_ctxs[i].vnode &&
		    pctx->vnode_ctxs[i].vnode->desc->type ==
		    CAMSS_ISP_EP_FRAME_INPUT)
			return &pctx->vnode_ctxs[i].fmt_state;
	}

	return NULL;
}

static int isp_vnode_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strscpy(cap->driver, "qcom-camss-isp", sizeof(cap->driver));
	strscpy(cap->card, video_devdata(file)->name, sizeof(cap->card));

	return 0;
}

static int isp_vnode_enum_fmt_vid(struct file *file, void *priv,
				  struct v4l2_fmtdesc *f)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));

	if (f->index >= vnode->desc->num_formats)
		return -EINVAL;

	f->pixelformat = vnode->desc->formats[f->index].fourcc;

	return 0;
}

static int isp_vnode_enum_fmt_meta(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));

	if (f->index >= vnode->desc->num_formats)
		return -EINVAL;

	f->pixelformat = vnode->desc->formats[f->index].fourcc;

	return 0;
}

static int isp_vnode_g_fmt_vid(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));
	struct camss_isp_vnode_ctx *vc = isp_vc(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	if (!vc)
		return -EINVAL;

	pix->pixelformat		= vc->fmt_state.fmt->fourcc;
	pix->width			= vc->fmt_state.width;
	pix->height			= vc->fmt_state.height;
	pix->num_planes			= 1;
	pix->field			= V4L2_FIELD_NONE;
	pix->colorspace			= vc->fmt_state.colorspace;
	pix->xfer_func			= vc->fmt_state.xfer_func;
	pix->ycbcr_enc			= vc->fmt_state.ycbcr_enc;
	pix->quantization		= vc->fmt_state.quantization;
	pix->plane_fmt[0].bytesperline	= vc->fmt_state.bytesperline;
	pix->plane_fmt[0].sizeimage	= vc->fmt_state.sizeimage;

	dev_dbg(vnode->vfd.v4l2_dev->dev,
		"%s: g_fmt %ux%u %p4cc bpl=%u size=%u\n",
		vnode->desc->name, pix->width, pix->height, &pix->pixelformat,
		pix->plane_fmt[0].bytesperline, pix->plane_fmt[0].sizeimage);
	return 0;
}

static int isp_vnode_try_fmt_vid(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));
	const struct camss_isp_fmt_state *input_fmt = NULL;

	/* Pass input constraints for capture (output) endpoints */
	if (vnode->vfd.vfl_dir == VFL_DIR_RX)
		input_fmt = isp_input_fmt(file);

	camss_isp_fmt_try(vnode, &f->fmt.pix_mp, input_fmt);

	return 0;
}

static int isp_vnode_s_fmt_vid(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct camss_isp_vnode_ctx *vc = isp_vc(file);
	const struct camss_isp_fmt_state *input_fmt = NULL;
	const struct camss_isp_fmt *fmt;

	if (!vc)
		return -EINVAL;

	if (vb2_is_busy(&vc->vb2_q))
		return -EBUSY;

	if (vc->vnode->vfd.vfl_dir == VFL_DIR_RX)
		input_fmt = isp_input_fmt(file);

	fmt = camss_isp_fmt_try(vc->vnode, &f->fmt.pix_mp, input_fmt);

	vc->fmt_state.fmt	    = fmt;
	vc->fmt_state.width	    = f->fmt.pix_mp.width;
	vc->fmt_state.height	    = f->fmt.pix_mp.height;
	vc->fmt_state.bytesperline  = f->fmt.pix_mp.plane_fmt[0].bytesperline;
	vc->fmt_state.sizeimage	    = f->fmt.pix_mp.plane_fmt[0].sizeimage;
	vc->fmt_state.colorspace    = f->fmt.pix_mp.colorspace;
	vc->fmt_state.xfer_func     = f->fmt.pix_mp.xfer_func;
	vc->fmt_state.ycbcr_enc	    = f->fmt.pix_mp.ycbcr_enc;
	vc->fmt_state.quantization  = f->fmt.pix_mp.quantization;

	dev_dbg(vc->vnode->vfd.v4l2_dev->dev, "%s: s_fmt %ux%u %p4cc\n",
		vc->vnode->desc->name, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		&f->fmt.pix_mp.pixelformat);

	return 0;
}

static int isp_vnode_g_fmt_meta(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));
	struct v4l2_meta_format *meta = &f->fmt.meta;

	if (vnode->desc->num_formats)
		meta->dataformat = vnode->desc->formats[0].fourcc;

	meta->buffersize = vnode->desc->buffersize ? vnode->desc->buffersize : PAGE_SIZE;

	return 0;
}

static int isp_vnode_g_parm(struct file *file, void *priv,
			    struct v4l2_streamparm *sp)
{
	struct camss_isp_vnode_ctx *vc = isp_vc(file);

	if (!vc || !V4L2_TYPE_IS_OUTPUT(sp->type))
		return -EINVAL;

	sp->parm.output.capability   = V4L2_CAP_TIMEPERFRAME;
	sp->parm.output.timeperframe = vc->fmt_state.timeperframe;

	return 0;
}

static int isp_vnode_s_parm(struct file *file, void *priv,
			    struct v4l2_streamparm *sp)
{
	struct camss_isp_vnode_ctx *vc = isp_vc(file);
	struct v4l2_fract *tpf;

	if (!vc || !V4L2_TYPE_IS_OUTPUT(sp->type))
		return -EINVAL;

	if (vb2_is_busy(&vc->vb2_q))
		return -EBUSY;

	tpf = &sp->parm.output.timeperframe;

	/* Sanitise: reject zero denominator, clamp numerator to non-zero */
	if (tpf->denominator == 0)
		tpf->denominator = 1;
	if (tpf->numerator == 0)
		tpf->numerator = 1;

	vc->fmt_state.timeperframe  = *tpf;
	sp->parm.output.capability  = V4L2_CAP_TIMEPERFRAME;

	dev_dbg(vc->vnode->vfd.v4l2_dev->dev, "%s: s_parm %u/%u fps\n",
		vc->vnode->desc->name, tpf->denominator, tpf->numerator);

	return 0;
}

/*
 * All endpoint types share the same vb2-backed buffer management.
 * Only the format-negotiation ioctls differ per type.
 */
#define ISP_VNODE_IOCTL_OPS_VB2 \
	.vidioc_reqbufs		= vb2_ioctl_reqbufs,	\
	.vidioc_querybuf	= vb2_ioctl_querybuf,	\
	.vidioc_create_bufs	= vb2_ioctl_create_bufs,\
	.vidioc_prepare_buf	= vb2_ioctl_prepare_buf,\
	.vidioc_qbuf		= vb2_ioctl_qbuf,	\
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,	\
	.vidioc_expbuf		= vb2_ioctl_expbuf,	\
	.vidioc_streamon	= vb2_ioctl_streamon,	\
	.vidioc_streamoff	= vb2_ioctl_streamoff

static const struct v4l2_ioctl_ops isp_vid_cap_ioctl_ops = {
	.vidioc_querycap		= isp_vnode_querycap,
	.vidioc_enum_fmt_vid_cap	= isp_vnode_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap_mplane	= isp_vnode_g_fmt_vid,
	.vidioc_s_fmt_vid_cap_mplane	= isp_vnode_s_fmt_vid,
	.vidioc_try_fmt_vid_cap_mplane	= isp_vnode_try_fmt_vid,
	ISP_VNODE_IOCTL_OPS_VB2,
};

static const struct v4l2_ioctl_ops isp_vid_out_ioctl_ops = {
	.vidioc_querycap		= isp_vnode_querycap,
	.vidioc_enum_fmt_vid_out	= isp_vnode_enum_fmt_vid,
	.vidioc_g_fmt_vid_out_mplane	= isp_vnode_g_fmt_vid,
	.vidioc_s_fmt_vid_out_mplane	= isp_vnode_s_fmt_vid,
	.vidioc_try_fmt_vid_out_mplane	= isp_vnode_try_fmt_vid,
	.vidioc_g_parm			= isp_vnode_g_parm,
	.vidioc_s_parm			= isp_vnode_s_parm,
	ISP_VNODE_IOCTL_OPS_VB2,
};

static const struct v4l2_ioctl_ops isp_meta_cap_ioctl_ops = {
	.vidioc_querycap		= isp_vnode_querycap,
	.vidioc_enum_fmt_meta_cap	= isp_vnode_enum_fmt_meta,
	.vidioc_g_fmt_meta_cap		= isp_vnode_g_fmt_meta,
	.vidioc_s_fmt_meta_cap		= isp_vnode_g_fmt_meta,
	.vidioc_try_fmt_meta_cap	= isp_vnode_g_fmt_meta,
	ISP_VNODE_IOCTL_OPS_VB2,
};

static const struct v4l2_ioctl_ops isp_meta_out_ioctl_ops = {
	.vidioc_querycap		= isp_vnode_querycap,
	.vidioc_enum_fmt_meta_out	= isp_vnode_enum_fmt_meta,
	.vidioc_g_fmt_meta_out		= isp_vnode_g_fmt_meta,
	.vidioc_s_fmt_meta_out		= isp_vnode_g_fmt_meta,
	.vidioc_try_fmt_meta_out	= isp_vnode_g_fmt_meta,
	ISP_VNODE_IOCTL_OPS_VB2,
};

static const struct v4l2_ioctl_ops *eptype_to_ioctl_ops(enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		return &isp_vid_out_ioctl_ops;
	case CAMSS_ISP_EP_PARAMS:
		return &isp_meta_out_ioctl_ops;
	case CAMSS_ISP_EP_STATS:
		return &isp_meta_cap_ioctl_ops;
	default:
		return &isp_vid_cap_ioctl_ops;
	}
}

/* Default file operations */
static const struct v4l2_file_operations isp_vnode_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

/* Pipeline context lifecycle */
static void isp_vnode_ctx_init_fmt(struct camss_isp_vnode_ctx *vc)
{
	const struct camss_isp_vnode *vnode = vc->vnode;
	struct v4l2_pix_format_mplane pix;
	const struct camss_isp_fmt *fmt;

	if (!vnode->desc->num_formats)
		return;

	pix = (struct v4l2_pix_format_mplane){
		.pixelformat = vnode->desc->formats[0].fourcc,
		.width       = vnode->desc->min_width,
		.height      = vnode->desc->min_height,
	};

	fmt = camss_isp_fmt_try(vnode, &pix, NULL);
	vc->fmt_state.fmt	   = fmt;
	vc->fmt_state.width	   = pix.width;
	vc->fmt_state.height	   = pix.height;
	vc->fmt_state.bytesperline = pix.plane_fmt[0].bytesperline;
	vc->fmt_state.sizeimage    = pix.plane_fmt[0].sizeimage;
	vc->fmt_state.colorspace   = V4L2_COLORSPACE_RAW;
}

struct camss_isp_pipeline_ctx *camss_isp_pipeline_ctx_create(struct camss_isp_pipeline *pipe,
						     void *priv)
{
	struct camss_isp_pipeline_ctx *pctx;
	unsigned int i;
	int ret;

	pctx = kvzalloc(sizeof(*pctx), GFP_KERNEL);
	if (!pctx)
		return ERR_PTR(-ENOMEM);

	pctx->vnode_ctxs = kvcalloc(pipe->num_vnodes,
				    sizeof(*pctx->vnode_ctxs), GFP_KERNEL);
	if (!pctx->vnode_ctxs) {
		kfree(pctx);
		return ERR_PTR(-ENOMEM);
	}

	pctx->num_vnodes = pipe->num_vnodes;
	pctx->priv = priv;

	for (i = 0; i < pipe->num_vnodes; i++) {
		struct camss_isp_vnode_ctx *vc = &pctx->vnode_ctxs[i];
		struct vb2_queue *q = &vc->vb2_q;

		vc->vnode        = &pipe->vnodes[i];
		vc->pipeline_ctx = pctx;

		q->type		    = eptype_to_buf_type(pipe->vnodes[i].desc->type);
		q->io_modes	    = VB2_MMAP | VB2_DMABUF;
		q->drv_priv	    = vc;
		q->buf_struct_size  = sizeof(struct camss_isp_buffer);
		q->ops		    = pipe->desc->vb2_ops;
		q->mem_ops	    = eptype_to_mem_ops(pipe->vnodes[i].desc->type);
		q->timestamp_flags  = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		q->lock		    = &pipe->lock;
		q->dev		    = pipe->v4l2_dev.dev;

		ret = vb2_queue_init(q);
		if (ret)
			goto err_queues;

		isp_vnode_ctx_init_fmt(vc);

		dev_dbg(pipe_dev(pipe), "%s: vnode_ctx init (type=%u)\n",
			vc->vnode->desc->name, q->type);
	}

	dev_dbg(pipe_dev(pipe), "pipeline_ctx %p created\n", pctx);

	return pctx;

err_queues:
	while (i--)
		vb2_queue_release(&pctx->vnode_ctxs[i].vb2_q);
	kvfree(pctx->vnode_ctxs);
	kvfree(pctx);
	return ERR_PTR(ret);
}

void camss_isp_pipeline_ctx_destroy(struct camss_isp_pipeline_ctx *pctx)
{
	unsigned int i;

	if (!pctx)
		return;

	for (i = 0; i < pctx->num_vnodes; i++) {
		struct camss_isp_vnode_ctx *vc = &pctx->vnode_ctxs[i];

		vb2_queue_release(&vc->vb2_q);
		if (vc->vnode)
			vc->vnode->vfd.queue = NULL;
	}

	kvfree(pctx->vnode_ctxs);
	kvfree(pctx);
}

int camss_isp_pipeline_ctx_bind(struct camss_isp_pipeline_ctx *pctx,
				struct file *file)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));
	struct camss_isp_pipeline *pipe = container_of(vnode->vfd.v4l2_dev,
						       struct camss_isp_pipeline, v4l2_dev);
	struct camss_isp_vnode_ctx *vc;

	lockdep_assert_held(&pipe->lock);

	vc = camss_isp_pipeline_ctx_get_vnode_ctx(pctx, vnode);
	if (!vc)
		return -ENODEV;

	/* Reject a second open on the same vnode within the same context.
	 * Each vnode_ctx holds a single v4l2_fh and can only be bound to
	 * one file at a time. revisit when v4l2 multi-context is supported.
	 */
	if (vc->fh.vdev) {
		dev_dbg(vnode->vfd.v4l2_dev->dev, "ctx %p: %s already bound, rejecting\n",
			pctx, vnode->desc->name);
		return -EBUSY;
	}

	v4l2_fh_init(&vc->fh, video_devdata(file));
	v4l2_fh_add(&vc->fh, file);
	file->private_data = &vc->fh;

	/* Point the video device at this context's queue */
	vnode->vfd.queue = &vc->vb2_q;

	dev_dbg(vnode->vfd.v4l2_dev->dev, "ctx %p: bound to %s\n", pctx, vnode->desc->name);

	return 0;
}

void camss_isp_pipeline_ctx_unbind(struct file *file)
{
	struct camss_isp_vnode_ctx *vc = camss_isp_vnode_ctx_from_file(file);

	v4l2_fh_del(&vc->fh, file);
	v4l2_fh_exit(&vc->fh);

	if (vc->vnode)
		vc->vnode->vfd.queue = NULL;
}

/*
 * vnode init/teardown
 */
static int isp_vnode_init(struct camss_isp_vnode *vnode,
			  const struct camss_isp_video_desc *desc,
			  const struct camss_isp_pipeline_desc *pipe_desc,
			  struct camss_isp_pipeline *pipe)
{
	struct video_device *vfd = &vnode->vfd;
	int ret;

	vnode->desc = desc;
	strscpy(vfd->name, desc->name, sizeof(vfd->name));
	vfd->entity.name = vfd->name;
	vnode->pad.flags = eptype_to_pad_flags(desc->type);

	ret = media_entity_pads_init(&vfd->entity, 1, &vnode->pad);
	if (ret)
		return ret;

	vfd->fops	  = pipe_desc->fops ? pipe_desc->fops : &isp_vnode_fops;
	vfd->ioctl_ops	  = eptype_to_ioctl_ops(desc->type);
	vfd->device_caps  = eptype_to_device_caps(desc->type);
	vfd->vfl_dir	  = eptype_to_vfl_dir(desc->type);
	vfd->v4l2_dev	  = &pipe->v4l2_dev;
	vfd->queue	  = NULL;
	vfd->lock	  = &pipe->lock;
	vfd->release	  = video_device_release_empty;
	video_set_drvdata(vfd, vnode);

	return 0;
}

static int isp_vnode_register(struct camss_isp_vnode *vnode,
			      struct camss_isp_pipeline *pipe)
{
	int ret;

	ret = video_register_device(&vnode->vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		media_entity_cleanup(&vnode->vfd.entity);
		return ret;
	}

	dev_dbg(pipe_dev(pipe), "registered vnode %s as /dev/video%d\n",
		vnode->desc->name, vnode->vfd.num);

	return 0;
}

static void isp_vnode_cleanup(struct camss_isp_vnode *vnode)
{
	if (video_is_registered(&vnode->vfd))
		video_unregister_device(&vnode->vfd);
	else
		media_entity_cleanup(&vnode->vfd.entity);
}

/*
 * subdevice init/teardown
 */
static const struct v4l2_subdev_ops isp_sd_default_ops = { };

static int isp_subdev_init(struct camss_isp_pipeline *pipe,
			   struct v4l2_device *v4l2_dev)
{
	const struct camss_isp_pipeline_desc *desc = pipe->desc;
	unsigned int i;
	int ret;

	pipe->sd_pads = kvcalloc(desc->num_subdev_pads,
				sizeof(*pipe->sd_pads), GFP_KERNEL);
	if (!pipe->sd_pads)
		return -ENOMEM;

	for (i = 0; i < desc->num_subdev_pads; i++)
		pipe->sd_pads[i].flags = MEDIA_PAD_FL_SINK;

	for (i = 0; i < desc->num_links; i++) {
		const struct camss_isp_link_desc *ld = &desc->links[i];

		if (ld->src >= 0 && (int)ld->src < (int)desc->num_subdev_pads)
			pipe->sd_pads[ld->src].flags = MEDIA_PAD_FL_SOURCE;
		if (ld->sink >= 0 && (int)ld->sink < (int)desc->num_subdev_pads)
			pipe->sd_pads[ld->sink].flags = MEDIA_PAD_FL_SINK;
	}

	v4l2_subdev_init(&pipe->subdev, &isp_sd_default_ops);
	pipe->subdev.owner = THIS_MODULE;
	strscpy(pipe->subdev.name, desc->subdev_name, sizeof(pipe->subdev.name));
	pipe->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	ret = media_entity_pads_init(&pipe->subdev.entity,
				     desc->num_subdev_pads, pipe->sd_pads);
	if (ret)
		goto err_free;

	ret = v4l2_device_register_subdev(v4l2_dev, &pipe->subdev);
	if (ret)
		goto err_entity;

	return 0;

err_entity:
	media_entity_cleanup(&pipe->subdev.entity);
err_free:
	kvfree(pipe->sd_pads);
	pipe->sd_pads = NULL;
	return ret;
}

static void isp_subdev_cleanup(struct camss_isp_pipeline *pipe)
{
	v4l2_device_unregister_subdev(&pipe->subdev);
	media_entity_cleanup(&pipe->subdev.entity);
	kvfree(pipe->sd_pads);
	pipe->sd_pads = NULL;
}

/*
 * Media links
 */
static struct media_entity *isp_link_entity(struct camss_isp_pipeline *pipe, int idx)
{
	unsigned int ep_idx;

	if (idx >= 0)
		return &pipe->subdev.entity;

	ep_idx = (unsigned int)(-(idx + 1));

	if (ep_idx >= pipe->num_vnodes)
		return NULL;

	return &pipe->vnodes[ep_idx].vfd.entity;
}

static u16 isp_link_pad(struct camss_isp_pipeline *pipe, int idx)
{
	if (idx >= 0)
		return (u16)idx;

	return 0;
}

static int isp_create_links(struct camss_isp_pipeline *pipe)
{
	const struct camss_isp_pipeline_desc *desc = pipe->desc;
	unsigned int i;

	for (i = 0; i < desc->num_links; i++) {
		const struct camss_isp_link_desc *ld = &desc->links[i];
		struct media_entity *src = isp_link_entity(pipe, ld->src);
		struct media_entity *sink = isp_link_entity(pipe, ld->sink);
		u16 sp = isp_link_pad(pipe, ld->src);
		u16 dp = isp_link_pad(pipe, ld->sink);
		int ret;

		if (!src || !sink)
			return -EINVAL;

		dev_dbg(pipe_dev(pipe), "link[%u]: %s:%u -> %s:%u flags=0x%x\n",
			i, src->name, sp, sink->name, dp, ld->flags);

		ret = media_create_pad_link(src, sp, sink, dp, ld->flags);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * pipeline build/teardown
 */
static int camss_isp_pipeline_register(struct camss_isp_pipeline *pipe)
{
	unsigned int i;
	int ret;

	for (i = 0; i < pipe->num_vnodes; i++) {
		ret = isp_vnode_register(&pipe->vnodes[i], pipe);
		if (ret) {
			while (i--)
				video_unregister_device(&pipe->vnodes[i].vfd);
			return ret;
		}
	}

	ret = isp_create_links(pipe);
	if (ret) {
		for (i = 0; i < pipe->num_vnodes; i++)
			video_unregister_device(&pipe->vnodes[i].vfd);
	}

	return ret;
}

int camss_isp_pipeline_build(struct camss_isp_pipeline *pipe,
			     struct device *dev,
			     const char *model,
			     const struct camss_isp_pipeline_desc *desc)
{
	unsigned int i;
	int ret;

	pipe->desc = desc;
	mutex_init(&pipe->lock);

	pipe->mdev.dev = dev;
	strscpy(pipe->mdev.model, model, sizeof(pipe->mdev.model));
	media_device_init(&pipe->mdev);

	ret = v4l2_device_register(dev, &pipe->v4l2_dev);
	if (ret)
		goto err_mdev;

	pipe->v4l2_dev.mdev = &pipe->mdev;

	ret = media_device_register(&pipe->mdev);
	if (ret)
		goto err_v4l2;

	pipe->vnodes = kvcalloc(desc->num_endpoints,
			       sizeof(*pipe->vnodes), GFP_KERNEL);
	if (!pipe->vnodes) {
		ret = -ENOMEM;
		goto err_mdev_unreg;
	}
	pipe->num_vnodes = desc->num_endpoints;

	ret = isp_subdev_init(pipe, &pipe->v4l2_dev);
	if (ret)
		goto err_free;

	for (i = 0; i < desc->num_endpoints; i++) {
		ret = isp_vnode_init(&pipe->vnodes[i], &desc->endpoints[i],
				     desc, pipe);
		if (ret)
			goto err_vnodes;
	}

	/* Register video devices and create links now that all entities exist */
	ret = camss_isp_pipeline_register(pipe);
	if (ret)
		goto err_vnodes;

	dev_dbg(pipe_dev(pipe), "ISP pipeline built: %u endpoints, %u links\n",
		desc->num_endpoints, desc->num_links);

	return 0;

err_vnodes:
	while (i--)
		isp_vnode_cleanup(&pipe->vnodes[i]);
	isp_subdev_cleanup(pipe);
err_free:
	kvfree(pipe->vnodes);
	pipe->vnodes = NULL;
	pipe->num_vnodes = 0;
err_mdev_unreg:
	media_device_unregister(&pipe->mdev);
err_v4l2:
	v4l2_device_unregister(&pipe->v4l2_dev);
err_mdev:
	media_device_cleanup(&pipe->mdev);
	return ret;
}

void camss_isp_pipeline_destroy(struct camss_isp_pipeline *pipe)
{
	unsigned int i;

	if (!pipe)
		return;

	dev_dbg(pipe_dev(pipe), "ISP pipeline teardown\n");
	media_device_unregister(&pipe->mdev);

	for (i = 0; i < pipe->num_vnodes; i++)
		isp_vnode_cleanup(&pipe->vnodes[i]);

	isp_subdev_cleanup(pipe);
	kvfree(pipe->vnodes);
	pipe->vnodes = NULL;
	pipe->num_vnodes = 0;

	v4l2_device_unregister(&pipe->v4l2_dev);
	media_device_cleanup(&pipe->mdev);
}
