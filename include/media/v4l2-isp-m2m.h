/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * V4L2 ISP mem-to-mem framework
 *
 * Framework for ISP-like mem-to-mem devices with complex pipelines:
 * multiple input queues (frame data + metadata/parameters) and multiple
 * output queues (processed frames + statistics).
 *
 * Copyright (C) 2025 Qualcomm Technologies, Inc.
 */

#ifndef _MEDIA_V4L2_ISP_MEM2MEM_H
#define _MEDIA_V4L2_ISP_MEM2MEM_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <media/media-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>

struct v4l2_isp_m2m_dev;
struct v4l2_isp_m2m_ctx;
struct v4l2_isp_m2m_queue_ctx;

/**
 * enum v4l2_isp_m2m_queue_type - ISP queue role classification
 *
 * @V4L2_ISP_M2M_QUEUE_SINK_FRAME:	Input video frame (OUTPUT direction)
 * @V4L2_ISP_M2M_QUEUE_SINK_PARAMS:	Input ISP parameters/metadata (OUTPUT direction)
 * @V4L2_ISP_M2M_QUEUE_SOURCE_FRAME:	Output processed video frame (CAPTURE direction)
 * @V4L2_ISP_M2M_QUEUE_SOURCE_STATS:	Output statistics (CAPTURE direction)
 */
enum v4l2_isp_m2m_queue_type {
	V4L2_ISP_M2M_QUEUE_SINK_FRAME,
	V4L2_ISP_M2M_QUEUE_SINK_PARAMS,
	V4L2_ISP_M2M_QUEUE_SOURCE_FRAME,
	V4L2_ISP_M2M_QUEUE_SOURCE_STATS,
};

/**
 * struct v4l2_isp_m2m_queue_desc - descriptor for one ISP queue/video device
 *
 * @type:	Role of this queue in the ISP pipeline.
 * @buf_type:	vb2 buffer type for this queue.
 * @name:	Human-readable name suffix for the video device.
 * @caps:	V4L2 device capabilities (V4L2_CAP_*).
 * @optional:	If true, a job can be scheduled even without a buffer on this
 *		queue (e.g. a statistics output that may not always be consumed).
 * @fops:	File operations for the video device representing this queue.
 * @ioctl_ops:	ioctl operations for the video device representing this queue.
 */
struct v4l2_isp_m2m_queue_desc {
	enum v4l2_isp_m2m_queue_type		type;
	enum v4l2_buf_type			buf_type;
	const char				*name;
	u32					caps;
	bool					optional;
	const struct v4l2_file_operations	*fops;
	const struct v4l2_ioctl_ops		*ioctl_ops;
};

/**
 * struct v4l2_isp_m2m_ops - ISP memory to memory driver callbacks
 *
 * @device_run:	Required. Start processing one job. The job does not have to
 *		complete before this returns. Call v4l2_isp_m2m_job_finish()
 *		when done.
 * @job_ready:	Optional. Return true if the driver/hardware can run a job.
 * @job_abort:	Optional. Abort the currently running job as soon as possible.
 *		The driver must still call v4l2_isp_m2m_job_finish() afterwards.
 * @queue_init:	Required. Initialize the vb2_queue for the given queue index.
 *		@index corresponds to the position in the queue descriptor array.
 */
struct v4l2_isp_m2m_ops {
	void (*device_run)(void *priv);
	bool (*job_ready)(void *priv);
	void (*job_abort)(void *priv);
	int  (*queue_init)(void *priv, unsigned int index, struct vb2_queue *vq);
};

/**
 * struct v4l2_isp_m2m_buffer - ISP memory to memory buffer wrapper
 *
 * @vb:		The vb2 V4L2 buffer.
 * @list:	List of isp_m2m buffers.
 */
struct v4l2_isp_m2m_buffer {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
};

/* -------- Buffer helpers -------- */

/**
 * v4l2_isp_m2m_next_buf() - return next buffer from the list of ready buffers
 *
 * @q_ctx: pointer to queue context, struct @v4l2_isp_m2m_queue_ctx
 */
struct vb2_v4l2_buffer *v4l2_isp_m2m_next_buf(struct v4l2_isp_m2m_queue_ctx *q_ctx);

/**
 * v4l2_isp_m2m_buf_remove() - take off and return a buffer from the list of ready buffers
 *
 * @q_ctx: pointer to queue context, struct @v4l2_isp_m2m_queue_ctx
 */
struct vb2_v4l2_buffer *v4l2_isp_m2m_buf_remove(struct v4l2_isp_m2m_queue_ctx *q_ctx);

/**
 * v4l2_isp_m2m_buf_done() - Return a buffers with state
 *
 * @buf:   buffer to complete
 * @state: completion state
 */
static inline void v4l2_isp_m2m_buf_done(struct vb2_v4l2_buffer *buf, enum vb2_buffer_state state)
{
	vb2_buffer_done(&buf->vb2_buf, state);
}

/**
 * v4l2_isp_m2m_buf_queue() - add a buffer to the proper ready buffers list.
 *
 * Called from the driver's vb2 buf_queue callback.
 *
 * @ctx:  pointer to ISP m2m context, struct @v4l2_isp_m2m_ctx
 * @vbuf: buffer to enqueue
 */
void v4l2_isp_m2m_buf_queue(struct v4l2_isp_m2m_ctx *ctx, struct vb2_v4l2_buffer *vbuf);

/* -------- Queue accessors -------- */

/**
 * v4l2_isp_m2m_get_vq() - return the vb2_queue for a given queue index
 *
 * @ctx:   pointer to ISP m2m context, struct @v4l2_isp_m2m_ctx
 * @index: queue index (0 .. num_queues-1)
 */
struct vb2_queue *v4l2_isp_m2m_get_vq(struct v4l2_isp_m2m_ctx *ctx, unsigned int index);

/**
 * v4l2_isp_m2m_get_q_ctx() - return the queue context for a given index
 *
 * @ctx:   pointer to ISP m2m context, struct @v4l2_isp_m2m_ctx
 * @index: queue index (0 .. num_queues-1)
 */
struct v4l2_isp_m2m_queue_ctx *v4l2_isp_m2m_get_q_ctx(struct v4l2_isp_m2m_ctx *ctx,
						      unsigned int index);

/* -------- Scheduling -------- */

/**
 * v4l2_isp_m2m_job_finish() - inform the framework that a job has been finished
 * @m2m_dev: pointer to ISP m2m device, struct @v4l2_isp_m2m_dev
 * @ctx:     pointer to ISP m2m context, struct @v4l2_isp_m2m_ctx
 *
 * Must be called after device_run() completes.
 */
void v4l2_isp_m2m_job_finish(struct v4l2_isp_m2m_dev *m2m_dev, struct v4l2_isp_m2m_ctx *ctx);

/**
 * v4l2_isp_m2m_suspend() - pause job scheduling and wait for current job
 * @m2m_dev: pointer to ISP m2m device, struct @v4l2_isp_m2m_dev
 */
void v4l2_isp_m2m_suspend(struct v4l2_isp_m2m_dev *m2m_dev);

/**
 * v4l2_isp_m2m_resume() - resume job scheduling
 * @m2m_dev: pointer to ISP m2m device, struct @v4l2_isp_m2m_dev
 */
void v4l2_isp_m2m_resume(struct v4l2_isp_m2m_dev *m2m_dev);

/* -------- ioctl/fop helpers -------- */

int v4l2_isp_m2m_ioctl_reqbufs(struct file *file, void *priv,
				struct v4l2_requestbuffers *rb);
int v4l2_isp_m2m_ioctl_querybuf(struct file *file, void *priv,
				 struct v4l2_buffer *buf);
int v4l2_isp_m2m_ioctl_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf);
int v4l2_isp_m2m_ioctl_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf);
int v4l2_isp_m2m_ioctl_prepare_buf(struct file *file, void *priv,
				    struct v4l2_buffer *buf);
int v4l2_isp_m2m_ioctl_create_bufs(struct file *file, void *priv,
				    struct v4l2_create_buffers *create);
int v4l2_isp_m2m_ioctl_expbuf(struct file *file, void *priv,
			       struct v4l2_exportbuffer *eb);
int v4l2_isp_m2m_ioctl_streamon(struct file *file, void *priv, enum v4l2_buf_type type);
int v4l2_isp_m2m_ioctl_streamoff(struct file *file, void *priv, enum v4l2_buf_type type);
__poll_t v4l2_isp_m2m_fop_poll(struct file *file, poll_table *wait);
int v4l2_isp_m2m_fop_mmap(struct file *file, struct vm_area_struct *vma);

/* -------- Device / Context lifecycle -------- */

/**
 * v4l2_isp_m2m_init() - Initialize per-device ISP m2m data
 *
 * @dev:         underlying Linux device
 * @name:        model name for the media device and processing entity
 * @lock:        mutex used to serialize ioctls across all video devices
 * @drv_priv:    driver-private pointer, retrievable via v4l2_isp_m2m_priv()
 * @ops:         ISP m2m driver callbacks
 * @queue_descs: array of queue descriptors describing the pipeline
 * @num_queues:  number of entries in @queue_descs
 *
 * Returns a pointer to the new device or an ERR_PTR on failure.
 */
struct v4l2_isp_m2m_dev *v4l2_isp_m2m_init(struct device *dev, const char *name,
					   struct mutex *lock, void *drv_priv,
					   const struct v4l2_isp_m2m_ops *ops,
					   const struct v4l2_isp_m2m_queue_desc *queue_descs,
					   unsigned int num_queues);

/**
 * v4l2_isp_m2m_release() - release an ISP m2m device
 *
 * @m2m_dev: pointer to ISP m2m device to release, struct @v4l2_isp_m2m_dev
 */
void v4l2_isp_m2m_release(struct v4l2_isp_m2m_dev *m2m_dev);

/**
 * v4l2_isp_m2m_ctx_init() - allocate and initialise a processing context
 *
 * @m2m_dev:  ISP m2m device
 * @drv_priv: driver private data stored in ctx->priv
 *
 * Returns a pointer to the new context or an ERR_PTR on failure.
 */
struct v4l2_isp_m2m_ctx *v4l2_isp_m2m_ctx_init(struct v4l2_isp_m2m_dev *m2m_dev, void *drv_priv);

/**
 * v4l2_isp_m2m_ctx_release() - release ISP m2m context
 *
 * @ctx: context to release
 */
void v4l2_isp_m2m_ctx_release(struct v4l2_isp_m2m_ctx *ctx);


/**
 * v4l2_isp_m2m_register() - register the ISP M2M device and build pipeline
 *
 * Creates one video device per queue descriptor, one ISP processing entity,
 * and the pad links between them.
 *
 * @m2m_dev: ISP m2m device
 *
 * Returns 0 on success or a negative error code.
 */
int v4l2_isp_m2m_register(struct v4l2_isp_m2m_dev *m2m_dev);

/**
 * v4l2_isp_m2m_unregister() - tear down the media graph
 *
 * @m2m_dev: ISP m2m device
 */
void v4l2_isp_m2m_unregister(struct v4l2_isp_m2m_dev *m2m_dev);

/**
 * v4l2_isp_m2m_priv() - retrieve driver-private data from a vdev
 *
 * @vdev: video device (one of the pipeline nodes)
 */
void *v4l2_isp_m2m_priv(struct video_device *vdev);

/**
 * v4l2_isp_m2m_ctx_priv() - return the driver private data for a context
 *
 * @ctx: ISP m2m context
 */
void *v4l2_isp_m2m_ctx_priv(struct v4l2_isp_m2m_ctx *ctx);

/**
 * v4l2_isp_m2m_num_queues() - return the number of queues in a context
 * @ctx: ISP m2m context
 */
unsigned int v4l2_isp_m2m_num_queues(struct v4l2_isp_m2m_ctx *ctx);

#endif /* _MEDIA_V4L2_ISP_MEM2MEM_H */
