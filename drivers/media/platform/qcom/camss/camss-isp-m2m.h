/* SPDX-License-Identifier: GPL-2.0 */
/*
 * camss-isp-m2m.h
 *
 * Qualcomm CAMSS ISP memory-to-memory scheduler.
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef CAMSS_ISP_M2M_H
#define CAMSS_ISP_M2M_H

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include <media/videobuf2-v4l2.h>

#include "camss-isp-pipeline.h"


enum camss_isp_m2m_trigger {
	CAMSS_ISP_M2M_TRIGGER_ANY_OUTPUT,
	CAMSS_ISP_M2M_TRIGGER_ALL_OUTPUTS,
};

/* Forward declarations */
struct camss_isp_m2m_dev;
struct camss_isp_m2m_ctx;

/**
 * struct camss_isp_m2m_ops - all driver callbacks
 *
 * @ctx_create:      allocate driver-private context
 * @ctx_destroy:     free driver-private context
 * @hw_ready:        check that hw is ready to process a job (optional) 
 * @configure:       apply params buffer to HW (optional)
 * @start_job:       kick off one job
 * @abort_job:       abort in-flight job
 * @streaming_start: enable streaming for specified endpoint (optional)
 * @streaming_stop:  disable streaming for specified endpoint (optional)
 */
struct camss_isp_m2m_ops {
	int  (*ctx_create)(struct camss_isp_m2m_ctx *ctx);
	void (*ctx_destroy)(struct camss_isp_m2m_ctx *ctx);

	bool (*hw_ready)(struct camss_isp_m2m_ctx *ctx);

	void (*configure)(struct camss_isp_m2m_ctx *ctx,
			  struct vb2_v4l2_buffer *params);

	int  (*start_job)(struct camss_isp_m2m_ctx *ctx);

	void (*abort_job)(struct camss_isp_m2m_ctx *ctx);

	int  (*streaming_start)(struct camss_isp_m2m_ctx *ctx,
				enum camss_isp_endpoint_type type);
	void (*streaming_stop)(struct camss_isp_m2m_ctx *ctx,
			       enum camss_isp_endpoint_type type);
};

#define CAMSS_ISP_M2M_MAX_OUTPUTS  4

/* M2M pipeline context */
struct camss_isp_m2m_ctx {
	struct camss_isp_pipeline_ctx *pipeline_ctx;
	struct camss_isp_m2m_dev *mdev;
	void *drv_priv;
	struct list_head list;

	struct list_head input_q;
	struct list_head output_q[CAMSS_ISP_M2M_MAX_OUTPUTS];
	struct list_head params_q;

	enum camss_isp_m2m_trigger trigger;

	bool input_streaming;
	bool output_streaming[CAMSS_ISP_M2M_MAX_OUTPUTS];
	bool params_streaming;

	bool running;

	struct vb2_v4l2_buffer *cur_input;
	struct vb2_v4l2_buffer *cur_outputs[CAMSS_ISP_M2M_MAX_OUTPUTS];
	struct vb2_v4l2_buffer *cur_params;
};

/* M2M pipeline */
struct camss_isp_m2m_dev {
	struct camss_isp_pipeline *pipe;
	const struct camss_isp_m2m_ops *ops;
	enum camss_isp_m2m_trigger trigger;
	struct list_head ctx_list;
	struct camss_isp_m2m_ctx *shared_ctx;
	unsigned int open_count;
};

/* Device-level API */
int camss_isp_m2m_dev_init(struct camss_isp_m2m_dev *mdev,
			   struct camss_isp_pipeline *pipe,
			   const struct camss_isp_m2m_ops *ops,
			   enum camss_isp_m2m_trigger trigger);


/* Scheduler API */
void camss_isp_m2m_job_done(struct camss_isp_m2m_ctx *ctx,
			    enum vb2_buffer_state state);


/* Accessors */
struct camss_isp_m2m_ctx *camss_isp_m2m_ctx_from_file(struct file *file);
struct camss_isp_fmt_state *camss_isp_m2m_ctx_fmt_by_type(struct camss_isp_m2m_ctx *ctx,
							  enum camss_isp_endpoint_type type);
struct camss_isp_vnode_ctx *camss_isp_m2m_ctx_vnode_ctx_by_type(struct camss_isp_m2m_ctx *ctx,
								enum camss_isp_endpoint_type type);

dma_addr_t camss_isp_m2m_ctx_dma_addr(struct camss_isp_m2m_ctx *ctx,
				   enum camss_isp_endpoint_type type,
				   unsigned int plane);

extern const struct v4l2_file_operations camss_isp_m2m_fops;
extern const struct vb2_ops camss_isp_m2m_vb2_ops;

#endif /* CAMSS_ISP_M2M_H */
