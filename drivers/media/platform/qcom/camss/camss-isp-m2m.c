// SPDX-License-Identifier: GPL-2.0
/*
 * camss-isp-m2m.c
 *
 * Qualcomm CAMSS ISP memory-to-memory scheduler.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>

#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>

#include "camss-isp-m2m.h"

#define mdev_dev(mdev) ((mdev)->pipe->v4l2_dev.dev)
#define ctx_dev(ctx)   mdev_dev((ctx)->mdev)
#define mdev_lock(mdev) (&(mdev)->pipe->lock)
#define ctx_lock(ctx)   mdev_lock((ctx)->mdev)

static unsigned int __output_slot(enum camss_isp_endpoint_type type)
{
	BUG_ON(type < CAMSS_ISP_EP_FRAME_OUTPUT);
	return type - CAMSS_ISP_EP_FRAME_OUTPUT;
}

static bool __inputs_ready(struct camss_isp_m2m_ctx *ctx)
{
	return ctx->input_streaming && !list_empty(&ctx->input_q);
}

static bool __outputs_ready(struct camss_isp_m2m_ctx *ctx)
{
	unsigned int i;
	bool any = false;

	for (i = 0; i < CAMSS_ISP_M2M_MAX_OUTPUTS; i++) {
		if (!ctx->output_streaming[i])
			continue;
		if (list_empty(&ctx->output_q[i])) {
			if (ctx->trigger == CAMSS_ISP_M2M_TRIGGER_ALL_OUTPUTS)
				return false;
		} else {
			any = true;
		}
	}

	return (ctx->trigger == CAMSS_ISP_M2M_TRIGGER_ANY_OUTPUT) ? any : true;
}

static void __dequeue_buffers(struct camss_isp_m2m_ctx *ctx)
{
	unsigned int i;

	ctx->cur_input = &list_first_entry(&ctx->input_q,
					  struct camss_isp_buffer, list)->vb;
	list_del_init(&to_camss_isp_buffer(ctx->cur_input)->list);

	for (i = 0; i < CAMSS_ISP_M2M_MAX_OUTPUTS; i++) {
		ctx->cur_outputs[i] = NULL;
		if (!ctx->output_streaming[i] || list_empty(&ctx->output_q[i]))
			continue;
		ctx->cur_outputs[i] = &list_first_entry(&ctx->output_q[i],
							struct camss_isp_buffer, list)->vb;
		list_del_init(&to_camss_isp_buffer(ctx->cur_outputs[i])->list);
	}

	ctx->cur_params = NULL;
	if (ctx->params_streaming && !list_empty(&ctx->params_q)) {
		ctx->cur_params = &list_first_entry(&ctx->params_q,
						    struct camss_isp_buffer, list)->vb;
		list_del_init(&to_camss_isp_buffer(ctx->cur_params)->list);
	}
}

static void __return_buffers(struct camss_isp_m2m_ctx *ctx,
			     enum vb2_buffer_state state)
{
	unsigned int i;

	/* Input Frame */
	if (ctx->cur_input) {
		ctx->cur_input->vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&ctx->cur_input->vb2_buf, state);
		ctx->cur_input = NULL;
	}

	/* Outputs */
	for (i = 0; i < CAMSS_ISP_M2M_MAX_OUTPUTS; i++) {
		if (!ctx->cur_outputs[i])
			continue;
		ctx->cur_outputs[i]->vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&ctx->cur_outputs[i]->vb2_buf, state);
		ctx->cur_outputs[i] = NULL;
	}

	/* Params */
	if (ctx->cur_params) {
		ctx->cur_params->vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&ctx->cur_params->vb2_buf, state);
		ctx->cur_params = NULL;
	}
}

static struct vb2_v4l2_buffer *camss_isp_m2m_ctx_buf_by_type(struct camss_isp_m2m_ctx *ctx,
							     enum camss_isp_endpoint_type type)
{
	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		return ctx->cur_input;
	case CAMSS_ISP_EP_PARAMS:
		return ctx->cur_params;
	default:
		return ctx->cur_outputs[__output_slot(type)];
	}
}

dma_addr_t camss_isp_m2m_ctx_dma_addr(struct camss_isp_m2m_ctx *ctx,
				      enum camss_isp_endpoint_type type,
				      unsigned int plane)
{
	struct vb2_v4l2_buffer *vbuf = camss_isp_m2m_ctx_buf_by_type(ctx, type);

	if (!vbuf)
		return 0;

	return vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, plane);
}

/*
 * Scheduler internals
 */
static bool isp_m2m_ctx_try_run(struct camss_isp_m2m_ctx *ctx)
{
	const struct camss_isp_m2m_ops *hwops = ctx->mdev->ops;
	bool inputs_ok, outputs_ok;
	int ret;

	lockdep_assert_held(ctx_lock(ctx));

	if (ctx->running)
		return false;

	inputs_ok  = __inputs_ready(ctx);
	outputs_ok = __outputs_ready(ctx);

	if (!inputs_ok || !outputs_ok) {
		dev_dbg(ctx_dev(ctx), "ctx %p: not ready (input=%d outputs=%d)\n",
			ctx, inputs_ok, outputs_ok);
		return false;
	}

	if (hwops->hw_ready && !hwops->hw_ready(ctx))
		return false;

	__dequeue_buffers(ctx);

	if (hwops->configure && ctx->cur_params)
		hwops->configure(ctx, ctx->cur_params);

	ctx->running = true;

	dev_dbg(ctx_dev(ctx), "ctx %p: dispatching job\n", ctx);

	ret = hwops->start_job(ctx);
	if (ret) {
		ctx->running = false;
		__return_buffers(ctx, VB2_BUF_STATE_ERROR);
		return false;
	}

	return true;
}

static void camss_isp_m2m_schedule(struct camss_isp_m2m_dev *mdev,
				   struct camss_isp_m2m_ctx *current_ctx)
{
	struct camss_isp_m2m_ctx *ctx;

	lockdep_assert_held(mdev_lock(mdev));

	if (list_empty(&mdev->ctx_list))
		return;

	/*
	 * Walk the list starting after current_ctx (round-robin fairness).
	 * If current_ctx is NULL, start from the list head.
	 */
	if (current_ctx) {
		ctx = list_next_entry(current_ctx, list);
		list_for_each_entry_from(ctx, &mdev->ctx_list, list) {
			if (isp_m2m_ctx_try_run(ctx))
				return;
		}
	}

	/* Wrap around: try from head up to (and including) current_ctx */
	list_for_each_entry(ctx, &mdev->ctx_list, list) {
		if (isp_m2m_ctx_try_run(ctx))
			return;
		if (ctx == current_ctx)
			break;
	}
}

void camss_isp_m2m_job_done(struct camss_isp_m2m_ctx *ctx,
			    enum vb2_buffer_state state)
{
	struct camss_isp_m2m_dev *mdev = ctx->mdev;

	lockdep_assert_held(mdev_lock(mdev));

	if (!ctx->running)
		return;

	dev_dbg(ctx_dev(ctx), "ctx %p: job done (%s)\n",
		ctx, state == VB2_BUF_STATE_DONE ? "ok" : "error");

	ctx->running = false;
	__return_buffers(ctx, state);
	/* TODO: we can return params earlier */

	/* Round-robin: start after the context that just finished */
	camss_isp_m2m_schedule(mdev, ctx);
}

static void camss_isp_m2m_queue_input(struct camss_isp_m2m_ctx *ctx,
			       struct vb2_v4l2_buffer *vbuf)
{
	lockdep_assert_held(ctx_lock(ctx));

	list_add_tail(&to_camss_isp_buffer(vbuf)->list, &ctx->input_q);
	camss_isp_m2m_schedule(ctx->mdev, ctx);
}

static void camss_isp_m2m_queue_output(struct camss_isp_m2m_ctx *ctx,
				unsigned int output_idx,
				struct vb2_v4l2_buffer *vbuf)
{
	lockdep_assert_held(ctx_lock(ctx));

	if (output_idx >= CAMSS_ISP_M2M_MAX_OUTPUTS)
		return;

	list_add_tail(&to_camss_isp_buffer(vbuf)->list, &ctx->output_q[output_idx]);
	camss_isp_m2m_schedule(ctx->mdev, ctx);
}

static void camss_isp_m2m_queue_params(struct camss_isp_m2m_ctx *ctx,
				struct vb2_v4l2_buffer *vbuf)
{
	lockdep_assert_held(ctx_lock(ctx));

	list_add_tail(&to_camss_isp_buffer(vbuf)->list, &ctx->params_q);
	camss_isp_m2m_schedule(ctx->mdev, ctx);
}

static void camss_isp_m2m_set_streaming(struct camss_isp_m2m_ctx *ctx,
				 enum camss_isp_endpoint_type type,
				 unsigned int output_idx,
				 bool streaming)
{
	lockdep_assert_held(ctx_lock(ctx));

	dev_dbg(ctx_dev(ctx), "ctx %p: set_streaming type=%d idx=%u %s\n",
		ctx, type, output_idx, streaming ? "on" : "off");

	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		ctx->input_streaming = streaming;
		break;
	case CAMSS_ISP_EP_PARAMS:
		ctx->params_streaming = streaming;
		break;
	default:
		ctx->output_streaming[__output_slot(type)] = streaming;
		break;
	}

	if (streaming)
		camss_isp_m2m_schedule(ctx->mdev, ctx);
}

static void camss_isp_m2m_stop_input(struct camss_isp_m2m_ctx *ctx, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vbuf;

	lockdep_assert_held(ctx_lock(ctx));

	if (ctx->running) {
		ctx->running = false;
		__return_buffers(ctx, state);
	}

	while (!list_empty(&ctx->input_q)) {
		vbuf = &list_first_entry(&ctx->input_q, struct camss_isp_buffer, list)->vb;
		list_del_init(&to_camss_isp_buffer(vbuf)->list);
		vb2_buffer_done(&vbuf->vb2_buf, state);
	}
}

static void camss_isp_m2m_stop_output(struct camss_isp_m2m_ctx *ctx,
				unsigned int output_idx,
				enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vbuf;

	lockdep_assert_held(ctx_lock(ctx));

	if (output_idx >= CAMSS_ISP_M2M_MAX_OUTPUTS)
		return;

	if (ctx->cur_outputs[output_idx]) {
		vb2_buffer_done(&ctx->cur_outputs[output_idx]->vb2_buf, state);
		ctx->cur_outputs[output_idx] = NULL;
	}

	while (!list_empty(&ctx->output_q[output_idx])) {
		vbuf = &list_first_entry(&ctx->output_q[output_idx],
					struct camss_isp_buffer, list)->vb;
		list_del_init(&to_camss_isp_buffer(vbuf)->list);
		vb2_buffer_done(&vbuf->vb2_buf, state);
	}
}

static void camss_isp_m2m_stop_params(struct camss_isp_m2m_ctx *ctx,
				enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vbuf;

	lockdep_assert_held(ctx_lock(ctx));

	if (ctx->cur_params) {
		vb2_buffer_done(&ctx->cur_params->vb2_buf, state);
		ctx->cur_params = NULL;
	}

	while (!list_empty(&ctx->params_q)) {
		vbuf = &list_first_entry(&ctx->params_q, struct camss_isp_buffer, list)->vb;
		list_del_init(&to_camss_isp_buffer(vbuf)->list);
		vb2_buffer_done(&vbuf->vb2_buf, state);
	}
}

/* The vb2_queue.drv_priv points to the camss_isp_vnode_ctx.
 * We reach the m2m_ctx via vnode_ctx->pipeline_ctx->priv.
 */
static struct camss_isp_m2m_ctx *ctx_from_vc(struct camss_isp_vnode_ctx *vc)
{
	return vc->pipeline_ctx ? vc->pipeline_ctx->priv : NULL;
}

static int isp_m2m_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
			       unsigned int *num_planes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct camss_isp_vnode_ctx *vc = vb2_get_drv_priv(q);
	unsigned int size = vc->fmt_state.sizeimage;

	if (!size)
		size = PAGE_SIZE;

	if (*num_planes) {
		if (*num_planes != 1 || sizes[0] < size)
			return -EINVAL;
	} else {
		*num_planes = 1;
		sizes[0] = size;
	}

	return 0;
}

static int isp_m2m_buf_prepare(struct vb2_buffer *vb)
{
	struct camss_isp_vnode_ctx *vc = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_isp_m2m_ctx *ctx = ctx_from_vc(vc);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	unsigned int sizeimage = vc->fmt_state.sizeimage;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE) {
			dev_err(ctx_dev(ctx), "%s: unsupported field type\n",
				vc->vnode->desc->name);
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < sizeimage) {
		dev_err(ctx_dev(ctx), "%s: plane size %lu < required %u\n",
			vc->vnode->desc->name, vb2_plane_size(vb, 0), sizeimage);
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, sizeimage);

	vbuf->sequence = vc->fmt_state.sequence++;

	return 0;
}

static void isp_m2m_buf_queue(struct vb2_buffer *vb)
{
	struct camss_isp_vnode_ctx *vc = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_isp_m2m_ctx *ctx = ctx_from_vc(vc);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	enum camss_isp_endpoint_type type = vc->vnode->desc->type;

	dev_dbg(ctx_dev(ctx), "ctx %p: buf queued on %s (idx %u)\n",
		ctx, vc->vnode->desc->name, vb->index);

	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		camss_isp_m2m_queue_input(ctx, vbuf);
		break;
	case CAMSS_ISP_EP_PARAMS:
		camss_isp_m2m_queue_params(ctx, vbuf);
		break;
	default:
		camss_isp_m2m_queue_output(ctx, __output_slot(type), vbuf);
		break;
	}
}

static int isp_m2m_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camss_isp_vnode_ctx *vc = vb2_get_drv_priv(q);
	struct camss_isp_m2m_ctx *ctx = ctx_from_vc(vc);
	const struct camss_isp_m2m_ops *hwops = ctx->mdev->ops;
	enum camss_isp_endpoint_type type = vc->vnode->desc->type;
	int ret = 0;

	dev_dbg(ctx_dev(ctx), "ctx %p: start streaming on %s\n",
		ctx, vc->vnode->desc->name);

	vc->fmt_state.sequence = 0;

	if (hwops->streaming_start) {
		ret = hwops->streaming_start(ctx, type);
		if (ret)
			return ret;
	}

	vc->streaming = true;
	camss_isp_m2m_set_streaming(ctx, type, 0, true);

	return 0;
}

static void isp_m2m_stop_streaming(struct vb2_queue *q)
{
	struct camss_isp_vnode_ctx *vc = vb2_get_drv_priv(q);
	struct camss_isp_m2m_ctx *ctx = ctx_from_vc(vc);
	enum camss_isp_endpoint_type type = vc->vnode->desc->type;
	const struct camss_isp_m2m_ops *hwops = ctx->mdev->ops;

	dev_dbg(ctx_dev(ctx), "ctx %p: stop streaming on %s\n",
		ctx, vc->vnode->desc->name);

	vc->streaming = false;
	camss_isp_m2m_set_streaming(ctx, type, 0, false);

	/*
	 * For the frame-input endpoint, abort any in-flight job first so
	 * the HW is stopped before streaming_stop().
	 */
	if (type == CAMSS_ISP_EP_FRAME_INPUT && ctx->running) {
		if (hwops->abort_job)
			hwops->abort_job(ctx);
		ctx->running = false;
		__return_buffers(ctx, VB2_BUF_STATE_ERROR);
	}

	if (hwops->streaming_stop)
		hwops->streaming_stop(ctx, type);

	switch (type) {
	case CAMSS_ISP_EP_FRAME_INPUT:
		camss_isp_m2m_stop_input(ctx, VB2_BUF_STATE_ERROR);
		break;
	case CAMSS_ISP_EP_PARAMS:
		camss_isp_m2m_stop_params(ctx, VB2_BUF_STATE_ERROR);
		break;
	default:
		camss_isp_m2m_stop_output(ctx, __output_slot(type), VB2_BUF_STATE_ERROR);
		break;
	}
}

const struct vb2_ops camss_isp_m2m_vb2_ops = {
	.queue_setup	 = isp_m2m_queue_setup,
	.buf_prepare	 = isp_m2m_buf_prepare,
	.buf_queue	 = isp_m2m_buf_queue,
	.start_streaming = isp_m2m_start_streaming,
	.stop_streaming  = isp_m2m_stop_streaming,
};

/*
 * Context handling
 */
static struct camss_isp_m2m_ctx *isp_m2m_ctx_alloc(struct camss_isp_m2m_dev *mdev)
{
	const struct camss_isp_m2m_ops *hwops = mdev->ops;
	struct camss_isp_m2m_ctx *ctx;
	struct camss_isp_pipeline_ctx *pctx;
	unsigned int i;
	int ret;

	ctx = kvzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->mdev	 = mdev;
	ctx->trigger	 = mdev->trigger;
	INIT_LIST_HEAD(&ctx->input_q);
	INIT_LIST_HEAD(&ctx->params_q);
	for (i = 0; i < CAMSS_ISP_M2M_MAX_OUTPUTS; i++)
		INIT_LIST_HEAD(&ctx->output_q[i]);

	/* Create the pipeline context — owns all vnode_ctxs and queues */
	pctx = camss_isp_pipeline_ctx_create(mdev->pipe, ctx);
	if (IS_ERR(pctx)) {
		kvfree(ctx);
		return ERR_CAST(pctx);
	}
	ctx->pipeline_ctx = pctx;

	dev_dbg(mdev_dev(mdev), "allocating context\n");

	ret = hwops->ctx_create(ctx);
	if (ret) {
		camss_isp_pipeline_ctx_destroy(pctx);
		kvfree(ctx);
		return ERR_PTR(ret);
	}

	INIT_LIST_HEAD(&ctx->list);
	list_add_tail(&ctx->list, &mdev->ctx_list);

	return ctx;
}

static void isp_m2m_ctx_free(struct camss_isp_m2m_ctx *ctx)
{
	struct camss_isp_m2m_dev *mdev = ctx->mdev;
	const struct camss_isp_m2m_ops *hwops = mdev->ops;

	dev_dbg(mdev_dev(mdev), "freeing context %p\n", ctx);

	list_del(&ctx->list);
	camss_isp_pipeline_ctx_destroy(ctx->pipeline_ctx);
	hwops->ctx_destroy(ctx);
	kvfree(ctx);
}

/*
 * File operations
 */
static int isp_m2m_open(struct file *file)
{
	struct camss_isp_vnode *vnode = video_get_drvdata(video_devdata(file));
	struct camss_isp_pipeline *pipe =
		container_of(vnode->vfd.v4l2_dev,
			     struct camss_isp_pipeline, v4l2_dev);
	struct camss_isp_m2m_dev *mdev = pipe->owner;
	struct camss_isp_m2m_ctx *ctx;
	int ret = 0;

	if (mutex_lock_interruptible(mdev_lock(mdev)))
		return -ERESTARTSYS;

	dev_dbg(mdev_dev(mdev), "open: %s\n", vnode->desc->name);

	/*
	 * Shared context: one pipeline_ctx for all opens.
	 * TODO: replace with VIDIOC_BIND_CONTEXT when upstream support lands.
	 */
	if (!mdev->shared_ctx) {
		ctx = isp_m2m_ctx_alloc(mdev);
		if (IS_ERR(ctx)) {
			ret = PTR_ERR(ctx);
			goto unlock;
		}
		mdev->shared_ctx = ctx;
	} else {
		ctx = mdev->shared_ctx;
	}
	mdev->open_count++;

	/* Bind this file to the vnode_ctx for the vnode being opened */
	ret = camss_isp_pipeline_ctx_bind(ctx->pipeline_ctx, file);
	if (ret) {
		dev_dbg(mdev_dev(mdev), "open: bind failed (%d) for %s\n",
			ret, vnode->desc->name);
		if (--mdev->open_count == 0) {
			isp_m2m_ctx_free(mdev->shared_ctx);
			mdev->shared_ctx = NULL;
		}
	}

unlock:
	mutex_unlock(mdev_lock(mdev));
	return ret;
}

static int isp_m2m_release(struct file *file)
{
	struct camss_isp_m2m_ctx *ctx = camss_isp_m2m_ctx_from_file(file);
	struct camss_isp_m2m_dev *mdev = ctx->mdev;

	guard(mutex)(mdev_lock(mdev));

	dev_dbg(mdev_dev(mdev), "release: ctx=%p\n", ctx);

	camss_isp_pipeline_ctx_unbind(file);

	if (--mdev->open_count == 0) {
		isp_m2m_ctx_free(mdev->shared_ctx);
		mdev->shared_ctx = NULL;
	}

	return 0;
}

const struct v4l2_file_operations camss_isp_m2m_fops = {
	.owner		= THIS_MODULE,
	.open		= isp_m2m_open,
	.release	= isp_m2m_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

/*
 * Device init / cleanup
 */
int camss_isp_m2m_dev_init(struct camss_isp_m2m_dev *mdev,
			   struct camss_isp_pipeline *pipe,
			   const struct camss_isp_m2m_ops *ops,
			   enum camss_isp_m2m_trigger trigger)
{
	if (!mdev || !pipe || !ops ||
	    !ops->ctx_create || !ops->ctx_destroy)
		return -EINVAL;

	mdev->pipe	  = pipe;
	pipe->owner	  = mdev;
	mdev->shared_ctx  = NULL;
	mdev->open_count  = 0;
	INIT_LIST_HEAD(&mdev->ctx_list);
	mdev->ops	  = ops;
	mdev->trigger	  = trigger;

	return 0;
}

/*
 * Context helpers
 */
struct camss_isp_m2m_ctx *camss_isp_m2m_ctx_from_file(struct file *file)
{
	struct camss_isp_pipeline_ctx *pctx = camss_isp_pipeline_ctx_from_file(file);

	return pctx ? pctx->priv : NULL;
}

struct camss_isp_fmt_state *camss_isp_m2m_ctx_fmt_by_type(struct camss_isp_m2m_ctx *ctx,
							  enum camss_isp_endpoint_type type)
{
	struct camss_isp_pipeline_ctx *pctx = ctx->pipeline_ctx;
	unsigned int i;

	for (i = 0; i < pctx->num_vnodes; i++) {
		if (pctx->vnode_ctxs[i].vnode &&
		    pctx->vnode_ctxs[i].vnode->desc->type == type)
			return &pctx->vnode_ctxs[i].fmt_state;
	}

	return NULL;
}

struct camss_isp_vnode_ctx *camss_isp_m2m_ctx_vnode_ctx_by_type(struct camss_isp_m2m_ctx *ctx,
								enum camss_isp_endpoint_type type)
{
	struct camss_isp_pipeline_ctx *pctx = ctx->pipeline_ctx;
	unsigned int i;

	for (i = 0; i < pctx->num_vnodes; i++) {
		if (pctx->vnode_ctxs[i].vnode &&
		    pctx->vnode_ctxs[i].vnode->desc->type == type)
			return &pctx->vnode_ctxs[i];
	}

	return NULL;
}
