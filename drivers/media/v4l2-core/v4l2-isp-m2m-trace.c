// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * V4L2 ISP mem-to-mem framework - trace point instantiation
 *
 * Copyright (C) 2025 Qualcomm Technologies, Inc.
 */

#define CREATE_TRACE_POINTS
#include <trace/events/v4l2_isp_m2m.h>

EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_job_queued);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_job_run);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_job_finish);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_job_cancel);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_buf_queue);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_buf_remove);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_streamon);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_streamoff);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_suspend);
EXPORT_TRACEPOINT_SYMBOL_GPL(v4l2_isp_m2m_resume);
