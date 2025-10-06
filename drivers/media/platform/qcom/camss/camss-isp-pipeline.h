/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-isp-pipeline.h
 *
 * Qualcomm CAMSS ISP pipeline framework.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef CAMSS_ISP_PIPELINE_H
#define CAMSS_ISP_PIPELINE_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <media/media-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>


/*
 * Type/ID of a video-device endpoint
 */
enum camss_isp_endpoint_type {
	CAMSS_ISP_EP_FRAME_INPUT,
	CAMSS_ISP_EP_PARAMS,
	CAMSS_ISP_EP_FRAME_OUTPUT,
	CAMSS_ISP_EP_FRAME_OUTPUT2,
	CAMSS_ISP_EP_STATS,
	CAMSS_ISP_EP_MAX,
};

/*
 * Generic ISP pixel format descriptor
 */
struct camss_isp_fmt {
	u32          fourcc;
	unsigned int depth;
	unsigned int align;
	unsigned int num_planes;
};

/*
 * Negotiated format for one video-node context
 */
struct camss_isp_fmt_state {
	const struct camss_isp_fmt *fmt;
	unsigned int width;
	unsigned int height;
	unsigned int bytesperline;
	unsigned int sizeimage;
	enum v4l2_colorspace     colorspace;
	enum v4l2_xfer_func      xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization   quantization;
	unsigned int sequence;
	struct v4l2_fract timeperframe;
};

/*
 * Static description for one video-device endpoint
 */
struct camss_isp_video_desc {
	const char *name;
	enum camss_isp_endpoint_type type;
	const struct camss_isp_fmt *formats;
	unsigned int num_formats;
	unsigned int min_width;
	unsigned int max_width;
	unsigned int min_height;
	unsigned int max_height;
	bool scaling;
	unsigned int buffersize;
};

/*
 * Static description of one media-controller link
 */
#define CAMSS_ISP_LINK_EP(n)  (-(int)(n) - 1)
#define CAMSS_ISP_LINK_SD(p)  ((int)(p))

struct camss_isp_link_desc {
	int src;
	int sink;
	u32 flags;
};

/*
 * Full static description of an ISP pipeline
 */
struct camss_isp_pipeline_desc {
	const char *subdev_name;
	const struct v4l2_file_operations *fops;
	const struct vb2_ops *vb2_ops;
	unsigned int num_subdev_pads;

	const struct camss_isp_video_desc *endpoints;
	unsigned int num_endpoints;

	const struct camss_isp_link_desc *links;
	unsigned int num_links;
};

/*
 * Video node endpoint
 */
struct camss_isp_vnode {
	struct video_device vfd;
	struct media_pad pad;
	const struct camss_isp_video_desc *desc;
};

struct camss_isp_pipeline_ctx;

/*
 * Video node context
 */
struct camss_isp_vnode_ctx {
	struct v4l2_fh fh; /* must be first */
	struct camss_isp_vnode *vnode;
	struct camss_isp_pipeline_ctx *pipeline_ctx;
	struct vb2_queue vb2_q;
	bool streaming;
	struct camss_isp_fmt_state fmt_state;
};

/*
 * Runtime pipeline instance
 */
struct camss_isp_pipeline {
	struct mutex lock;
	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct v4l2_subdev subdev;
	struct media_pad *sd_pads;
	struct camss_isp_vnode *vnodes;
	unsigned int num_vnodes;
	const struct camss_isp_pipeline_desc *desc;
	void *owner;
};

/*
 * Runtime pipeline context
 */
struct camss_isp_pipeline_ctx {
	struct camss_isp_vnode_ctx *vnode_ctxs;
	unsigned int num_vnodes;
	void *priv;
};

/*
 * Pipeline build/teardown API
 */
int camss_isp_pipeline_build(struct camss_isp_pipeline *pipe,
			     struct device *dev,
			     const char *model,
			     const struct camss_isp_pipeline_desc *desc);

void camss_isp_pipeline_destroy(struct camss_isp_pipeline *pipe);

/*
 * Pipeline context lifecycle API
 */
struct camss_isp_pipeline_ctx *camss_isp_pipeline_ctx_create(struct camss_isp_pipeline *pipe,
							     void *priv);

void camss_isp_pipeline_ctx_destroy(struct camss_isp_pipeline_ctx *pctx);

int camss_isp_pipeline_ctx_bind(struct camss_isp_pipeline_ctx *pctx, struct file *file);

void camss_isp_pipeline_ctx_unbind(struct file *file);

/* Accessors */

/**
 * camss_isp_pipeline_ctx_from_file - get the pipeline context from a file
 *
 * Maps to the future media_device_context_from_file().
 */
static inline struct camss_isp_pipeline_ctx *camss_isp_pipeline_ctx_from_file(struct file *file)
{
	struct v4l2_fh *fh = file->private_data;
	struct camss_isp_vnode_ctx *vc = container_of(fh, struct camss_isp_vnode_ctx, fh);

	return vc->pipeline_ctx;
}

/**
 * camss_isp_vnode_ctx_from_file - get the vnode context for the accessed vnode
 *
 * Maps to the future video_device_context_from_file().
 */
static inline struct camss_isp_vnode_ctx *camss_isp_vnode_ctx_from_file(struct file *file)
{
	return container_of((struct v4l2_fh *)file->private_data, struct camss_isp_vnode_ctx, fh);
}

/**
 * camss_isp_pipeline_ctx_get_vnode_ctx - find the vnode_ctx for a given vnode
 */
static inline struct camss_isp_vnode_ctx *
camss_isp_pipeline_ctx_get_vnode_ctx(struct camss_isp_pipeline_ctx *pctx,
				     const struct camss_isp_vnode *vnode)
{
	unsigned int i;

	for (i = 0; i < pctx->num_vnodes; i++) {
		if (pctx->vnode_ctxs[i].vnode == vnode)
			return &pctx->vnode_ctxs[i];
	}

	return NULL;
}

struct camss_isp_buffer {
	struct vb2_v4l2_buffer vb;  /* must be first */
	struct list_head list;
};

static inline struct camss_isp_buffer *to_camss_isp_buffer(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct camss_isp_buffer, vb);
}

#endif /* CAMSS_ISP_PIPELINE_H */
