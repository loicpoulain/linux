// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 ISP mem-to-mem framework
 *
 * Framework for ISP memory-to-memory pipeline:
 * multiple input queues (frame data + metadata/parameters) and multiple
 * output queues (processed frames + statistics...).
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-isp-m2m.h>
#include <media/videobuf2-v4l2.h>

#include <trace/events/v4l2_isp_m2m.h>


/* Job state flags (stored in ctx->job_flags) */
#define ISP_TRANS_QUEUED	BIT(0)
#define ISP_TRANS_RUNNING	BIT(1)
#define ISP_TRANS_ABORT		BIT(2)

/* Device-level queue flags */
#define ISP_QUEUE_PAUSED	BIT(0)

/**
 * struct v4l2_isp_m2m_dev - ISP Memory to memory device
 *
 * @dev:		Underlying device.
 * @v4l2_dev:		V4L2 device, owns the video nodes.
 * @mdev:		Media device, owns the media graph.
 * @lock:		Mutex serialising all queue ioctls;
 * @drv_priv:		Driver private data passed to v4l2_isp_m2m_init().
 * @curr_ctx:		Context currently running on the hardware.
 * @job_queue:		List of contexts waiting to run.
 * @job_spinlock:	Protects job queue and context.
 * @job_work:		Worker to run queued jobs.
 * @job_queue_flags:	flags of the queue status, %ISP_QUEUE_PAUSED.
 * @ops:		Driver callbacks.
 * @queue_descs:	Queue descriptor array.
 * @num_queues:		Number of entries in @queue_descs.
 * @vdevs:		Video devices, one per queue; flexible array sized to @num_queues.
 * @vdev_pads:		Media pads for each video device.
 * @proc_entity:	Media entity representing the ISP processing block.
 * @proc_pads:		Pads for @proc_entity (sinks first, then sources).
 * @num_sink_pads:	Number of sink pads on @proc_entity.
 * @num_source_pads:	Number of source pads on @proc_entity.
 */
struct v4l2_isp_m2m_dev {
	struct device			*dev;

	struct v4l2_device		v4l2_dev;
	struct media_device		mdev;
	struct mutex			*lock;
	void				*drv_priv;

	struct v4l2_isp_m2m_ctx		*curr_ctx;

	struct list_head		job_queue;
	spinlock_t			job_spinlock;
	struct work_struct		job_work;
	unsigned long			job_queue_flags;

	const struct v4l2_isp_m2m_ops		*ops;
	const struct v4l2_isp_m2m_queue_desc	*queue_descs;
	unsigned int				num_queues;

	struct media_pad		*vdev_pads;
	struct media_entity		proc_entity;
	struct media_pad		*proc_pads;
	unsigned int			num_sink_pads;
	unsigned int			num_source_pads;

	struct video_device		vdevs[] __counted_by(num_queues);
};

/**
 * struct v4l2_isp_m2m_queue_ctx - Per-queue context
 *
 * @q:		Pointer to struct &vb2_queue
 * @rdy_queue:	List of buffers ready to be processed.
 * @rdy_spinlock: Protects @rdy_queue and @num_rdy.
 * @num_rdy:	Number of buffers in @rdy_queue.
 * @optional:	Is the queue required to schedule processing?
 */
struct v4l2_isp_m2m_queue_ctx {
	struct vb2_queue	q;

	struct list_head	rdy_queue;
	spinlock_t		rdy_spinlock;
	u32			num_rdy;
	bool			optional;
};

/**
 * struct v4l2_isp_m2m_ctx - ISP Memory to memory context structure
 *
 * @q_lock:	Mutext shared by all queues in this context.
 * @m2m_dev:	Back-pointer to the ISP m2m device.
 * @queue:	Entry in the device job queue.
 * @job_flags:	Internal job state flags:
 *		%ISP_TRANS_QUEUED, %ISP_TRANS_RUNNING, %ISP_TRANS_ABORT
 * @finished:	Wait queue used to signalize when a job queue finished.
 * @priv:	Instance private data.
 * @num_queues:	Number of entries in @q_ctx.
 * @q_ctx:	Per-queue contexts, flexible array sized to @num_queues.
 */
struct v4l2_isp_m2m_ctx {
	struct mutex			*q_lock;

	struct v4l2_isp_m2m_dev		*m2m_dev;

	/* Job queue linkage */
	struct list_head		queue;
	unsigned long			job_flags;
	wait_queue_head_t		finished;

	void				*priv;

	unsigned int			num_queues;
	struct v4l2_isp_m2m_queue_ctx	q_ctx[] __counted_by(num_queues);
};

/* -------- Internal helpers -------- */

static bool isp_queue_is_sink(enum v4l2_isp_m2m_queue_type type)
{
	return type == V4L2_ISP_M2M_QUEUE_SINK_FRAME ||
	       type == V4L2_ISP_M2M_QUEUE_SINK_PARAMS;
}

static struct v4l2_isp_m2m_queue_ctx *get_queue_ctx(struct v4l2_isp_m2m_ctx *ctx,
						 struct vb2_queue *vq)
{
	unsigned int i;

	for (i = 0; i < ctx->num_queues; i++) {
		if (&ctx->q_ctx[i].q == vq)
			return &ctx->q_ctx[i];
	}

	return NULL;
}

/* -------- Queue accessors -------- */

struct vb2_queue *v4l2_isp_m2m_get_vq(struct v4l2_isp_m2m_ctx *ctx, unsigned int index)
{
	if (WARN_ON(index >= ctx->num_queues))
		return NULL;
	return &ctx->q_ctx[index].q;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_get_vq);

struct v4l2_isp_m2m_queue_ctx *v4l2_isp_m2m_get_q_ctx(struct v4l2_isp_m2m_ctx *ctx,
						      unsigned int index)
{
	if (WARN_ON(index >= ctx->num_queues))
		return NULL;
	return &ctx->q_ctx[index];
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_get_q_ctx);

/* -------- Buffer helpers -------- */

struct vb2_v4l2_buffer *v4l2_isp_m2m_next_buf(struct v4l2_isp_m2m_queue_ctx *q_ctx)
{
	struct vb2_v4l2_buffer *vb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&q_ctx->rdy_spinlock, flags);
	if (!list_empty(&q_ctx->rdy_queue)) {
		struct v4l2_isp_m2m_buffer *buf = list_first_entry(&q_ctx->rdy_queue,
								   struct v4l2_isp_m2m_buffer,
								   list);
		vb = &buf->vb;
	}
	spin_unlock_irqrestore(&q_ctx->rdy_spinlock, flags);

	return vb;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_next_buf);

struct vb2_v4l2_buffer *v4l2_isp_m2m_buf_remove(struct v4l2_isp_m2m_queue_ctx *q_ctx)
{
	struct vb2_v4l2_buffer *vb = NULL;
	unsigned long flags;

	spin_lock_irqsave(&q_ctx->rdy_spinlock, flags);

	if (!list_empty(&q_ctx->rdy_queue)) {
		struct v4l2_isp_m2m_buffer *buf = list_first_entry(&q_ctx->rdy_queue,
								   struct v4l2_isp_m2m_buffer,
								   list);
		list_del(&buf->list);
		q_ctx->num_rdy--;
		vb = &buf->vb;

		trace_v4l2_isp_m2m_buf_remove(q_ctx, 0,
					      vb->vb2_buf.index,
					      vb->vb2_buf.vb2_queue->type,
					      q_ctx->num_rdy);
	}

	spin_unlock_irqrestore(&q_ctx->rdy_spinlock, flags);

	return vb;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_buf_remove);

void v4l2_isp_m2m_buf_queue(struct v4l2_isp_m2m_ctx *ctx, struct vb2_v4l2_buffer *vbuf)
{
	struct v4l2_isp_m2m_buffer *buf = container_of(vbuf, struct v4l2_isp_m2m_buffer, vb);
	struct v4l2_isp_m2m_queue_ctx *q_ctx;
	unsigned long flags;

	q_ctx = get_queue_ctx(ctx, vbuf->vb2_buf.vb2_queue);
	if (WARN_ON(!q_ctx))
		return;

	spin_lock_irqsave(&q_ctx->rdy_spinlock, flags);

	list_add_tail(&buf->list, &q_ctx->rdy_queue);
	q_ctx->num_rdy++;

	trace_v4l2_isp_m2m_buf_queue(ctx, q_ctx - ctx->q_ctx,
				     vbuf->vb2_buf.index,
				     vbuf->vb2_buf.vb2_queue->type,
				     q_ctx->num_rdy);

	spin_unlock_irqrestore(&q_ctx->rdy_spinlock, flags);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_buf_queue);

/* -------- Job scheduling -------- */

static void isp_m2m_try_run(struct v4l2_isp_m2m_dev *m2m_dev)
{
	struct v4l2_isp_m2m_ctx *ctx;
	unsigned long flags;

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);

	if (m2m_dev->curr_ctx) {
		dev_dbg(m2m_dev->dev, "Another job is running\n");
		goto exit_unlock;
	}

	if (list_empty(&m2m_dev->job_queue)) {
		dev_dbg(m2m_dev->dev, "No pending jobs\n");
		goto exit_unlock;
	}

	if (m2m_dev->job_queue_flags & ISP_QUEUE_PAUSED) {
		dev_dbg(m2m_dev->dev, "Job queue is paused\n");
		goto exit_unlock;
	}

	ctx = list_first_entry(&m2m_dev->job_queue, struct v4l2_isp_m2m_ctx, queue);
	ctx->job_flags |= ISP_TRANS_RUNNING;
	m2m_dev->curr_ctx = ctx;

	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	dev_dbg(m2m_dev->dev, "Running job for ctx %p\n", ctx);
	trace_v4l2_isp_m2m_job_run(ctx);
	m2m_dev->ops->device_run(ctx->priv);

	return;
exit_unlock:
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
	return;
}

static bool isp_m2m_ctx_is_ready(struct v4l2_isp_m2m_dev *m2m_dev, struct v4l2_isp_m2m_ctx *ctx)
{
	unsigned int i;
	bool has_sink = false, has_source = false;

	for (i = 0; i < ctx->num_queues; i++) {
		struct v4l2_isp_m2m_queue_ctx *q_ctx = &ctx->q_ctx[i];
		enum v4l2_isp_m2m_queue_type type = m2m_dev->queue_descs[i].type;

		bool ready = vb2_is_streaming(&q_ctx->q) && (q_ctx->num_rdy > 0);

		/* Non-optional queues must be streaming with a buffer ready */
		if (!q_ctx->optional && !ready)
			return false;

		/* Count ready frame queues (optional or not) */
		if (ready && type == V4L2_ISP_M2M_QUEUE_SINK_FRAME)
			has_sink = true;
		else if (ready && type == V4L2_ISP_M2M_QUEUE_SOURCE_FRAME)
			has_source = true;
	}

	if (!has_sink || !has_source)
		return false;

	if (m2m_dev->ops->job_ready)
		return m2m_dev->ops->job_ready(ctx->priv);

	return true;
}

static void isp_m2m_try_queue(struct v4l2_isp_m2m_dev *m2m_dev, struct v4l2_isp_m2m_ctx *ctx)
{
	unsigned long flags;

	dev_dbg(m2m_dev->dev, "Trying to schedule ctx %p\n", ctx);

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);

	if (ctx->job_flags & ISP_TRANS_ABORT) {
		dev_dbg(m2m_dev->dev, "Context is being aborted\n");
		goto unlock;
	}

	if (ctx->job_flags & ISP_TRANS_QUEUED) {
		dev_dbg(m2m_dev->dev, "Context already queued\n");
		goto unlock;
	}

	if (!isp_m2m_ctx_is_ready(m2m_dev, ctx)) {
		dev_dbg(m2m_dev->dev, "Context not ready\n");
		goto unlock;
	}

	list_add_tail(&ctx->queue, &m2m_dev->job_queue);
	ctx->job_flags |= ISP_TRANS_QUEUED;
	trace_v4l2_isp_m2m_job_queued(ctx, list_count_nodes(&m2m_dev->job_queue));

unlock:
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
}

static void isp_m2m_job_work(struct work_struct *work)
{
	struct v4l2_isp_m2m_dev *m2m_dev =
		container_of(work, struct v4l2_isp_m2m_dev, job_work);

	isp_m2m_try_run(m2m_dev);
}

static void isp_m2m_try_schedule(struct v4l2_isp_m2m_ctx *ctx)
{
	isp_m2m_try_queue(ctx->m2m_dev, ctx);
	isp_m2m_try_run(ctx->m2m_dev);
}

static void isp_m2m_cancel_job(struct v4l2_isp_m2m_ctx *ctx)
{
	struct v4l2_isp_m2m_dev *m2m_dev = ctx->m2m_dev;
	unsigned long flags;

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	ctx->job_flags |= ISP_TRANS_ABORT;

	if (ctx->job_flags & ISP_TRANS_RUNNING) {
		trace_v4l2_isp_m2m_job_cancel(ctx, true, false);
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		if (m2m_dev->ops->job_abort)
			m2m_dev->ops->job_abort(ctx->priv);
		dev_dbg(m2m_dev->dev, "Waiting for running job on ctx %p\n", ctx);
		wait_event(ctx->finished, !(ctx->job_flags & ISP_TRANS_RUNNING));
	} else if (ctx->job_flags & ISP_TRANS_QUEUED) {
		trace_v4l2_isp_m2m_job_cancel(ctx, false, true);
		list_del(&ctx->queue);
		ctx->job_flags &= ~(ISP_TRANS_QUEUED | ISP_TRANS_RUNNING);
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
		dev_dbg(m2m_dev->dev, "Removed queued ctx %p\n", ctx);
	} else {
		trace_v4l2_isp_m2m_job_cancel(ctx, false, false);
		spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);
	}
}

static bool isp_m2m_job_finish_locked(struct v4l2_isp_m2m_dev *m2m_dev,
				      struct v4l2_isp_m2m_ctx *ctx)
{
	if (!m2m_dev->curr_ctx || m2m_dev->curr_ctx != ctx) {
		dev_dbg(m2m_dev->dev, "job_finish called by non-running context\n");
		/*
		 * streamoff may have already cleared curr_ctx while job_abort
		 * was in flight.  If this context is still marked RUNNING,
		 * clear the flag and wake any cancel_job waiter so it can
		 * proceed — the buffer has already been returned by the caller
		 * (camss_isp_m2m_job_finish) before reaching here.
		 */
		if (ctx->job_flags & ISP_TRANS_RUNNING) {
			ctx->job_flags &= ~(ISP_TRANS_QUEUED | ISP_TRANS_RUNNING);
			wake_up(&ctx->finished);
		}
		return false;
	}

	list_del(&m2m_dev->curr_ctx->queue);
	m2m_dev->curr_ctx->job_flags &= ~(ISP_TRANS_QUEUED | ISP_TRANS_RUNNING);
	wake_up(&m2m_dev->curr_ctx->finished);
	m2m_dev->curr_ctx = NULL;
	return true;
}

void v4l2_isp_m2m_job_finish(struct v4l2_isp_m2m_dev *m2m_dev, struct v4l2_isp_m2m_ctx *ctx)
{
	unsigned long flags;
	bool schedule_next;

	dev_dbg(m2m_dev->dev, "Job finished of ctx %p\n", ctx);

	trace_v4l2_isp_m2m_job_finish(ctx);

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	schedule_next = isp_m2m_job_finish_locked(m2m_dev, ctx);
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	if (schedule_next) {
		/*
		 * Re-queue this context in case it has more buffers ready,
		 * then schedule the next job via a work item so we can be
		 * called from interrupt context.
		 */
		isp_m2m_try_queue(m2m_dev, ctx);
		schedule_work(&m2m_dev->job_work);
	}
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_job_finish);

void v4l2_isp_m2m_suspend(struct v4l2_isp_m2m_dev *m2m_dev)
{
	struct v4l2_isp_m2m_ctx *curr;
	unsigned long flags;

	trace_v4l2_isp_m2m_suspend(m2m_dev);

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	m2m_dev->job_queue_flags |= ISP_QUEUE_PAUSED;
	curr = m2m_dev->curr_ctx;
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	if (curr)
		wait_event(curr->finished, !(curr->job_flags & ISP_TRANS_RUNNING));
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_suspend);

void v4l2_isp_m2m_resume(struct v4l2_isp_m2m_dev *m2m_dev)
{
	unsigned long flags;

	trace_v4l2_isp_m2m_resume(m2m_dev);

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags);
	m2m_dev->job_queue_flags &= ~ISP_QUEUE_PAUSED;
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags);

	isp_m2m_try_run(m2m_dev);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_resume);

/* -------- ioctl helpers -------- */

static struct vb2_queue *isp_m2m_vq_from_file(struct file *file, struct v4l2_isp_m2m_ctx *ctx)
{
	unsigned int index =
		(unsigned int)(uintptr_t)video_get_drvdata(video_devdata(file));

	if (index >= ctx->num_queues)
		return NULL;
	return &ctx->q_ctx[index].q;
}

static int isp_m2m_reqbufs(struct file *file, struct v4l2_isp_m2m_ctx *ctx,
			   struct v4l2_requestbuffers *rb)
{
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);
	int ret;

	if (!vq)
		return -EINVAL;

	ret = vb2_reqbufs(vq, rb);
	if (ret == 0)
		vq->owner = rb->count ? file->private_data : NULL;

	return ret;
}

static int isp_m2m_querybuf(struct file *file, struct v4l2_isp_m2m_ctx *ctx, struct v4l2_buffer *buf)
{
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);

	if (!vq)
		return -EINVAL;

	return vb2_querybuf(vq, buf);
}

static int isp_m2m_qbuf(struct file *file, struct v4l2_isp_m2m_ctx *ctx, struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);
	int ret;

	if (!vq)
		return -EINVAL;

	ret = vb2_qbuf(vq, vdev->v4l2_dev->mdev, buf);
	if (ret)
		return ret;

	if (!(buf->flags & V4L2_BUF_FLAG_IN_REQUEST))
		isp_m2m_try_schedule(ctx);

	return 0;
}

static int isp_m2m_dqbuf(struct file *file, struct v4l2_isp_m2m_ctx *ctx, struct v4l2_buffer *buf)
{
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);

	if (!vq)
		return -EINVAL;

	return vb2_dqbuf(vq, buf, file->f_flags & O_NONBLOCK);
}

static int isp_m2m_prepare_buf(struct file *file, struct v4l2_isp_m2m_ctx *ctx,
			      struct v4l2_buffer *buf)
{
	struct video_device *vdev = video_devdata(file);
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);

	if (!vq)
		return -EINVAL;

	return vb2_prepare_buf(vq, vdev->v4l2_dev->mdev, buf);
}

static int isp_m2m_create_bufs(struct file *file, struct v4l2_isp_m2m_ctx *ctx,
			      struct v4l2_create_buffers *create)
{
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);

	if (!vq)
		return -EINVAL;

	return vb2_create_bufs(vq, create);
}

static int isp_m2m_expbuf(struct file *file, struct v4l2_isp_m2m_ctx *ctx,
			 struct v4l2_exportbuffer *eb)
{
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);

	if (!vq)
		return -EINVAL;

	return vb2_expbuf(vq, eb);
}

static int isp_m2m_streamon(struct file *file, struct v4l2_isp_m2m_ctx *ctx, enum v4l2_buf_type type)
{
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);
	struct v4l2_isp_m2m_queue_ctx *q_ctx;
	int ret;

	if (!vq)
		return -EINVAL;

	ret = vb2_streamon(vq, type);
	if (!ret) {
		q_ctx = get_queue_ctx(ctx, vq);
		if (q_ctx)
			trace_v4l2_isp_m2m_streamon(ctx, q_ctx - ctx->q_ctx, type);
		isp_m2m_try_schedule(ctx);
	}

	return ret;
}

static int isp_m2m_streamoff(struct file *file, struct v4l2_isp_m2m_ctx *ctx, enum v4l2_buf_type type)
{
	struct v4l2_isp_m2m_dev *m2m_dev = ctx->m2m_dev;
	struct v4l2_isp_m2m_queue_ctx *q_ctx;
	struct vb2_queue *vq;
	unsigned long flags_job, flags;
	int ret;

	vq = isp_m2m_vq_from_file(file, ctx);
	if (!vq)
		return -EINVAL;

	isp_m2m_cancel_job(ctx);

	ret = vb2_streamoff(vq, type);
	if (ret)
		return ret;

	q_ctx = get_queue_ctx(ctx, vq);
	if (WARN_ON(!q_ctx))
		return 0;

	trace_v4l2_isp_m2m_streamoff(ctx, q_ctx - ctx->q_ctx, type);

	spin_lock_irqsave(&m2m_dev->job_spinlock, flags_job);
	if (ctx->job_flags & ISP_TRANS_QUEUED)
		list_del(&ctx->queue);
	ctx->job_flags &= ~(ISP_TRANS_QUEUED | ISP_TRANS_ABORT);

	spin_lock_irqsave(&q_ctx->rdy_spinlock, flags);
	INIT_LIST_HEAD(&q_ctx->rdy_queue);
	q_ctx->num_rdy = 0;
	spin_unlock_irqrestore(&q_ctx->rdy_spinlock, flags);

	if (m2m_dev->curr_ctx == ctx) {
		m2m_dev->curr_ctx = NULL;
		/*
		 * Only wake waiters if the job has already completed.  If
		 * RUNNING is still set, job_abort is in flight and the IRQ
		 * will call job_finish_locked which wakes ctx->finished after
		 * returning the buffers to vb2.  Waking here prematurely would
		 * let cancel_job unblock before vb2_buffer_done is called,
		 * leaving buffers in ACTIVE state for __vb2_queue_cancel.
		 */
		if (!(ctx->job_flags & ISP_TRANS_RUNNING))
			wake_up(&ctx->finished);
	}
	/*
	 * If the context is still marked RUNNING here it means job_abort was
	 * called but the completion IRQ has not fired yet.  Do not wake
	 * ctx->finished — cancel_job is already waiting for the IRQ to do so
	 * via job_finish_locked.  Clearing curr_ctx above is enough to let
	 * the IRQ path detect the stale finish and handle it.
	 */
	spin_unlock_irqrestore(&m2m_dev->job_spinlock, flags_job);

	return 0;
}

static struct v4l2_isp_m2m_ctx *isp_m2m_ctx_from_fh(struct file *file)
{
	return file_to_v4l2_fh(file)->isp_m2m_ctx;
}

int v4l2_isp_m2m_ioctl_reqbufs(struct file *file, void *priv,
				struct v4l2_requestbuffers *rb)
{
	return isp_m2m_reqbufs(file, isp_m2m_ctx_from_fh(file), rb);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_reqbufs);

int v4l2_isp_m2m_ioctl_querybuf(struct file *file, void *priv,
				 struct v4l2_buffer *buf)
{
	return isp_m2m_querybuf(file, isp_m2m_ctx_from_fh(file), buf);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_querybuf);

int v4l2_isp_m2m_ioctl_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	return isp_m2m_qbuf(file, isp_m2m_ctx_from_fh(file), buf);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_qbuf);

int v4l2_isp_m2m_ioctl_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	return isp_m2m_dqbuf(file, isp_m2m_ctx_from_fh(file), buf);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_dqbuf);

int v4l2_isp_m2m_ioctl_prepare_buf(struct file *file, void *priv,
				    struct v4l2_buffer *buf)
{
	return isp_m2m_prepare_buf(file, isp_m2m_ctx_from_fh(file), buf);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_prepare_buf);

int v4l2_isp_m2m_ioctl_create_bufs(struct file *file, void *priv,
				    struct v4l2_create_buffers *create)
{
	return isp_m2m_create_bufs(file, isp_m2m_ctx_from_fh(file), create);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_create_bufs);

int v4l2_isp_m2m_ioctl_expbuf(struct file *file, void *priv,
			       struct v4l2_exportbuffer *eb)
{
	return isp_m2m_expbuf(file, isp_m2m_ctx_from_fh(file), eb);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_expbuf);

int v4l2_isp_m2m_ioctl_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	return isp_m2m_streamon(file, isp_m2m_ctx_from_fh(file), type);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_streamon);

int v4l2_isp_m2m_ioctl_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	return isp_m2m_streamoff(file, isp_m2m_ctx_from_fh(file), type);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ioctl_streamoff);

/* -------- File operation helpers -------- */

__poll_t v4l2_isp_m2m_fop_poll(struct file *file, poll_table *wait)
{
	struct v4l2_isp_m2m_ctx *ctx = file_to_v4l2_fh(file)->isp_m2m_ctx;
	struct v4l2_fh *fh = file_to_v4l2_fh(file);
	__poll_t rc = 0;
	unsigned int i;
	unsigned long flags;

	if (ctx->q_lock)
		mutex_lock(ctx->q_lock);

	for (i = 0; i < ctx->num_queues; i++)
		poll_wait(file, &ctx->q_ctx[i].q.done_wq, wait);

	poll_wait(file, &fh->wait, wait);

	for (i = 0; i < ctx->num_queues; i++) {
		struct vb2_queue *vq = &ctx->q_ctx[i].q;

		spin_lock_irqsave(&vq->done_lock, flags);
		if (!list_empty(&vq->done_list)) {
			if (V4L2_TYPE_IS_OUTPUT(vq->type))
				rc |= EPOLLOUT | EPOLLWRNORM;
			else
				rc |= EPOLLIN | EPOLLRDNORM;
		}
		spin_unlock_irqrestore(&vq->done_lock, flags);
	}

	if (v4l2_event_pending(fh))
		rc |= EPOLLPRI;

	if (ctx->q_lock)
		mutex_unlock(ctx->q_lock);

	return rc;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_fop_poll);

int v4l2_isp_m2m_fop_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct v4l2_isp_m2m_ctx *ctx = file_to_v4l2_fh(file)->isp_m2m_ctx;
	struct v4l2_isp_m2m_dev *m2m_dev = ctx->m2m_dev;
	struct vb2_queue *vq = isp_m2m_vq_from_file(file, ctx);
	int ret;

	if (!vq)
		return -EINVAL;

	ret = vb2_mmap(vq, vma);
	if (!ret)
		dev_dbg(m2m_dev->dev,
			"mmap queue=%u pgoff=0x%08lx va=[0x%08lx-0x%08lx] size=%lu\n",
			(unsigned int)(uintptr_t)video_get_drvdata(video_devdata(file)),
			vma->vm_pgoff,
			vma->vm_start, vma->vm_end,
			vma->vm_end - vma->vm_start);
	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_fop_mmap);

/* -------- Device / context lifecycle -------- */

struct v4l2_isp_m2m_dev *v4l2_isp_m2m_init(struct device *dev, const char *name,
					   struct mutex *lock, void *drv_priv,
					   const struct v4l2_isp_m2m_ops *ops,
					   const struct v4l2_isp_m2m_queue_desc *queue_descs,
					   unsigned int num_queues)
{
	struct v4l2_isp_m2m_dev *m2m_dev;

	if (!dev || !name || !lock || !drv_priv || !ops || WARN_ON(!ops->device_run) || WARN_ON(!ops->queue_init))
		return ERR_PTR(-EINVAL);

	if (!queue_descs || !num_queues)
		return ERR_PTR(-EINVAL);

	m2m_dev = kzalloc(struct_size(m2m_dev, vdevs, num_queues), GFP_KERNEL);
	if (!m2m_dev)
		return ERR_PTR(-ENOMEM);

	m2m_dev->dev = dev;
	m2m_dev->lock = lock;
	m2m_dev->drv_priv = drv_priv;
	m2m_dev->ops = ops;
	m2m_dev->queue_descs = queue_descs;
	m2m_dev->num_queues = num_queues;

	INIT_LIST_HEAD(&m2m_dev->job_queue);
	spin_lock_init(&m2m_dev->job_spinlock);
	INIT_WORK(&m2m_dev->job_work, isp_m2m_job_work);

	m2m_dev->mdev.dev = dev;
	strscpy(m2m_dev->mdev.model, name, sizeof(m2m_dev->mdev.model));
	media_device_init(&m2m_dev->mdev);
	m2m_dev->v4l2_dev.mdev = &m2m_dev->mdev;

	return m2m_dev;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_init);

void v4l2_isp_m2m_release(struct v4l2_isp_m2m_dev *m2m_dev)
{
	media_device_cleanup(&m2m_dev->mdev);
	kfree(m2m_dev);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_release);

struct v4l2_isp_m2m_ctx *v4l2_isp_m2m_ctx_init(struct v4l2_isp_m2m_dev *m2m_dev, void *drv_priv)
{
	struct v4l2_isp_m2m_ctx *ctx;
	unsigned int i;
	int ret;

	ctx = kzalloc(struct_size(ctx, q_ctx, m2m_dev->num_queues), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->m2m_dev = m2m_dev;
	ctx->priv = drv_priv;
	ctx->num_queues = m2m_dev->num_queues;
	init_waitqueue_head(&ctx->finished);
	INIT_LIST_HEAD(&ctx->queue);

	/* init context for each queue */
	for (i = 0; i < m2m_dev->num_queues; i++) {
		struct v4l2_isp_m2m_queue_ctx *q_ctx = &ctx->q_ctx[i];

		INIT_LIST_HEAD(&q_ctx->rdy_queue);
		spin_lock_init(&q_ctx->rdy_spinlock);
		q_ctx->optional = m2m_dev->queue_descs[i].optional;

		ret = m2m_dev->ops->queue_init(drv_priv, i, &q_ctx->q);
		if (ret)
			goto err;

		/* All queues must share the same lock */
		if (i == 0) {
			ctx->q_lock = q_ctx->q.lock;
		} else if (ctx->q_ctx[i].q.lock != q_ctx->q.lock) {
			ret = -EINVAL;
			goto err;
		}
	}

	return ctx;
err:
	while (i--)
		vb2_queue_release(&ctx->q_ctx[i].q);
	kfree(ctx);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ctx_init);

void v4l2_isp_m2m_ctx_release(struct v4l2_isp_m2m_ctx *ctx)
{
	unsigned int i;

	isp_m2m_cancel_job(ctx);

	for (i = 0; i < ctx->num_queues; i++)
		vb2_queue_release(&ctx->q_ctx[i].q);

	kfree(ctx);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ctx_release);

void *v4l2_isp_m2m_priv(struct video_device *vdev)
{
	struct v4l2_isp_m2m_dev *m2m_dev =
			container_of(vdev->v4l2_dev, struct v4l2_isp_m2m_dev, v4l2_dev);
	return m2m_dev->drv_priv;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_priv);

void v4l2_isp_m2m_unregister(struct v4l2_isp_m2m_dev *m2m_dev)
{
	unsigned int i;

	for (i = 0; i < m2m_dev->num_queues; i++) {
		if (!m2m_dev->vdevs[i].name[0])
			continue;
		video_unregister_device(&m2m_dev->vdevs[i]);
	}

	media_entity_remove_links(&m2m_dev->proc_entity);
	media_device_unregister_entity(&m2m_dev->proc_entity);
	kfree(m2m_dev->proc_pads);

	m2m_dev->proc_pads = NULL;
	kfree(m2m_dev->vdev_pads);

	media_device_unregister(&m2m_dev->mdev);
	v4l2_device_unregister(&m2m_dev->v4l2_dev);
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_unregister);

static void isp_m2m_count_pads(struct v4l2_isp_m2m_dev *m2m_dev, unsigned int *num_sink,
				unsigned int *num_source)
{
	unsigned int i;

	*num_sink = 0;
	*num_source = 0;

	for (i = 0; i < m2m_dev->num_queues; i++) {
		if (isp_queue_is_sink(m2m_dev->queue_descs[i].type))
			(*num_sink)++;
		else
			(*num_source)++;
	}
}

static int isp_m2m_register_proc_entity(struct v4l2_isp_m2m_dev *m2m_dev)
{
	unsigned int num_sink = m2m_dev->num_sink_pads;
	unsigned int num_source = m2m_dev->num_source_pads;
	unsigned int i;
	int ret;

	m2m_dev->vdev_pads = kcalloc(m2m_dev->num_queues, sizeof(*m2m_dev->vdev_pads), GFP_KERNEL);
	if (!m2m_dev->vdev_pads) {
		ret = -ENOMEM;
		return ret;
	}

	/*
	 * Pads on the processing entity:
	 *   [0 .. num_sink-1]                  -> MEDIA_PAD_FL_SINK
	 *   [num_sink .. num_sink+num_source-1] -> MEDIA_PAD_FL_SOURCE
	 */
	m2m_dev->proc_pads = kcalloc(num_sink + num_source,
				     sizeof(*m2m_dev->proc_pads), GFP_KERNEL);
	if (!m2m_dev->proc_pads) {
		ret = -ENOMEM;
		goto err_free_vpads;
	}

	for (i = 0; i < num_sink; i++)
		m2m_dev->proc_pads[i].flags = MEDIA_PAD_FL_SINK;
	for (i = 0; i < num_source; i++)
		m2m_dev->proc_pads[num_sink + i].flags = MEDIA_PAD_FL_SOURCE;

	m2m_dev->proc_entity.obj_type = MEDIA_ENTITY_TYPE_BASE;
	m2m_dev->proc_entity.name = "proc";
	m2m_dev->proc_entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;

	ret = media_entity_pads_init(&m2m_dev->proc_entity,
				     num_sink + num_source, m2m_dev->proc_pads);
	if (ret)
		goto err_free_ppads;

	ret = media_device_register_entity(&m2m_dev->mdev, &m2m_dev->proc_entity);
	if (ret)
		goto err_free_ppads;

	return 0;

err_free_ppads:
	kfree(m2m_dev->proc_pads);
err_free_vpads:
	kfree(m2m_dev->vdev_pads);
	return ret;
}

static int isp_m2m_register_vdev(struct v4l2_isp_m2m_dev *m2m_dev, unsigned int index,
				 unsigned int *sink_idx, unsigned int *source_idx)
{
	const struct v4l2_isp_m2m_queue_desc *desc = &m2m_dev->queue_descs[index];
	struct v4l2_device *v4l2_dev = &m2m_dev->v4l2_dev;
	struct media_pad *vdev_pad = &m2m_dev->vdev_pads[index];
	struct video_device *vdev = &m2m_dev->vdevs[index];
	bool is_sink = isp_queue_is_sink(desc->type);
	unsigned int proc_pad_idx;
	int ret;

	strscpy(vdev->name, desc->name, sizeof(vdev->name));
	vdev->vfl_dir       = is_sink ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->v4l2_dev      = v4l2_dev;
	vdev->device_caps   = desc->caps;
	vdev->release       = video_device_release_empty;
	vdev->lock          = m2m_dev->lock;
	if (desc->fops)
		vdev->fops = desc->fops;
	if (desc->ioctl_ops)
		vdev->ioctl_ops = desc->ioctl_ops;

	vdev->entity.obj_type = MEDIA_ENTITY_TYPE_VIDEO_DEVICE;
	vdev_pad->flags = is_sink ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, vdev_pad);
	if (ret)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	video_set_drvdata(vdev, (void *)(uintptr_t)index);

	if (is_sink) {
		proc_pad_idx = (*sink_idx)++;
		ret = media_create_pad_link(&vdev->entity, 0,
					    &m2m_dev->proc_entity, proc_pad_idx,
					    MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	} else {
		proc_pad_idx = m2m_dev->num_sink_pads + (*source_idx)++;
		ret = media_create_pad_link(&m2m_dev->proc_entity, proc_pad_idx,
					    &vdev->entity, 0,
					    MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	}
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	video_unregister_device(vdev);
	return ret;
}

int v4l2_isp_m2m_register(struct v4l2_isp_m2m_dev *m2m_dev)
{
	unsigned int sink_idx = 0, source_idx = 0;
	unsigned int i;
	int ret;

	isp_m2m_count_pads(m2m_dev, &m2m_dev->num_sink_pads,
			   &m2m_dev->num_source_pads);

	ret = v4l2_device_register(m2m_dev->dev, &m2m_dev->v4l2_dev);
	if (ret)
		return ret;

	ret = media_device_register(&m2m_dev->mdev);
	if (ret) {
		v4l2_device_unregister(&m2m_dev->v4l2_dev);
		return ret;
	}

	ret = isp_m2m_register_proc_entity(m2m_dev);
	if (ret)
		goto err_unregister;

	for (i = 0; i < m2m_dev->num_queues; i++) {
		ret = isp_m2m_register_vdev(m2m_dev, i, &sink_idx, &source_idx);
		if (ret)
			goto err_unregister;
	}

	return 0;

err_unregister:
	v4l2_isp_m2m_unregister(m2m_dev);
	return ret;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_register);

/* -------- Internal helpers -------- */

void *v4l2_isp_m2m_ctx_priv(struct v4l2_isp_m2m_ctx *ctx)
{
	return ctx->priv;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_ctx_priv);

unsigned int v4l2_isp_m2m_num_queues(struct v4l2_isp_m2m_ctx *ctx)
{
	return ctx->num_queues;
}
EXPORT_SYMBOL_GPL(v4l2_isp_m2m_num_queues);
