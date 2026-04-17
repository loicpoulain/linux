// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CAMSS ISP scheduler helper — single-device job scheduling
 *
 * Copyright (C) 2025 Qualcomm Technologies, Inc.
 */

#include <linux/slab.h>

#include "camss-isp-sched.h"

/* Job state flags */
#define ISP_JOB_QUEUED		BIT(0)
#define ISP_JOB_RUNNING		BIT(1)
#define ISP_JOB_ABORT		BIT(2)

/* Scheduler flags */
#define ISP_SCHED_PAUSED	BIT(0)

/* -------- Internal helpers -------- */

static void isp_sched_try_run(struct camss_isp_sched *sched)
{
	struct camss_isp_job *job;
	void (*run_fn)(void *priv);
	void *priv;
	unsigned long flags;

	spin_lock_irqsave(&sched->lock, flags);

	if (sched->curr_job || list_empty(&sched->pending) ||
	    (sched->flags & ISP_SCHED_PAUSED)) {
		spin_unlock_irqrestore(&sched->lock, flags);
		return;
	}

	job = list_first_entry(&sched->pending, struct camss_isp_job, queue);
	job->flags |= ISP_JOB_RUNNING;
	sched->curr_job = job;
	run_fn = job->ops ? job->ops->run : NULL;
	priv   = job->priv;

	spin_unlock_irqrestore(&sched->lock, flags);

	run_fn(priv);
}

static void isp_sched_work(struct work_struct *work)
{
	struct camss_isp_sched *sched =
		container_of(work, struct camss_isp_sched, work);

	isp_sched_try_run(sched);
}

/* -------- Public API -------- */

void camss_isp_sched_init(struct camss_isp_sched *sched)
{
	sched->curr_job = NULL;
	INIT_LIST_HEAD(&sched->pending);
	spin_lock_init(&sched->lock);
	INIT_WORK(&sched->work, isp_sched_work);
	sched->flags = 0;
}

void camss_isp_sched_destroy(struct camss_isp_sched *sched)
{
	cancel_work_sync(&sched->work);
}

void camss_isp_job_init(struct camss_isp_job *job,
		       const struct camss_isp_job_ops *ops,
		       void *priv)
{
	INIT_LIST_HEAD(&job->queue);
	job->flags = 0;
	job->ops   = ops;
	job->priv  = priv;
	init_waitqueue_head(&job->finished);
}

void camss_isp_sched_try_run(struct camss_isp_sched *sched,
			     struct camss_isp_job *job)
{
	unsigned long flags;

	if (job->ops && job->ops->ready && !job->ops->ready(job->priv))
		return;

	spin_lock_irqsave(&sched->lock, flags);

	if (job->flags & (ISP_JOB_ABORT | ISP_JOB_QUEUED | ISP_JOB_RUNNING)) {
		spin_unlock_irqrestore(&sched->lock, flags);
		return;
	}

	list_add_tail(&job->queue, &sched->pending);
	job->flags |= ISP_JOB_QUEUED;

	spin_unlock_irqrestore(&sched->lock, flags);

	isp_sched_try_run(sched);
}

void camss_isp_sched_job_finish(struct camss_isp_sched *sched,
				struct camss_isp_job *job,
				bool requeue)
{
	unsigned long flags;

	spin_lock_irqsave(&sched->lock, flags);

	if (sched->curr_job != job) {
		/*
		 * curr_job may have been cleared by a racing cancel/streamoff.
		 * If this job is still marked RUNNING, clear it and wake any
		 * waiter in camss_isp_sched_cancel() so it can unblock.
		 */
		if (job->flags & ISP_JOB_RUNNING) {
			job->flags &= ~(ISP_JOB_QUEUED | ISP_JOB_RUNNING);
			wake_up(&job->finished);
		}
		spin_unlock_irqrestore(&sched->lock, flags);
		return;
	}

	list_del(&job->queue);
	job->flags &= ~(ISP_JOB_QUEUED | ISP_JOB_RUNNING);
	wake_up(&job->finished);
	sched->curr_job = NULL;

	if (requeue && !(job->flags & ISP_JOB_ABORT)) {
		job->flags |= ISP_JOB_QUEUED;
		list_add(&job->queue, &sched->pending);
	}

	spin_unlock_irqrestore(&sched->lock, flags);

	schedule_work(&sched->work);
}

void camss_isp_sched_cancel(struct camss_isp_sched *sched,
			    struct camss_isp_job *job)
{
	unsigned long flags;

	spin_lock_irqsave(&sched->lock, flags);
	job->flags |= ISP_JOB_ABORT;

	if (job->flags & ISP_JOB_RUNNING) {
		spin_unlock_irqrestore(&sched->lock, flags);
		if (job->ops && job->ops->abort)
			job->ops->abort(job->priv);
		wait_event(job->finished, !(job->flags & ISP_JOB_RUNNING));
	} else if (job->flags & ISP_JOB_QUEUED) {
		list_del(&job->queue);
		job->flags &= ~(ISP_JOB_QUEUED | ISP_JOB_RUNNING);
		spin_unlock_irqrestore(&sched->lock, flags);
	} else {
		spin_unlock_irqrestore(&sched->lock, flags);
	}

	/* Clear abort flag so the job can be reused after cancel */
	spin_lock_irqsave(&sched->lock, flags);
	job->flags &= ~ISP_JOB_ABORT;
	spin_unlock_irqrestore(&sched->lock, flags);
}

void camss_isp_sched_suspend(struct camss_isp_sched *sched)
{
	struct camss_isp_job *curr;
	unsigned long flags;

	spin_lock_irqsave(&sched->lock, flags);
	sched->flags |= ISP_SCHED_PAUSED;
	curr = sched->curr_job;
	spin_unlock_irqrestore(&sched->lock, flags);

	if (curr)
		wait_event(curr->finished, !(curr->flags & ISP_JOB_RUNNING));
}

void camss_isp_sched_resume(struct camss_isp_sched *sched)
{
	unsigned long flags;

	spin_lock_irqsave(&sched->lock, flags);
	sched->flags &= ~ISP_SCHED_PAUSED;
	spin_unlock_irqrestore(&sched->lock, flags);

	isp_sched_try_run(sched);
}

bool camss_isp_sched_is_running(struct camss_isp_sched *sched,
				struct camss_isp_job *job)
{
	unsigned long flags;
	bool running;

	spin_lock_irqsave(&sched->lock, flags);
	running = (sched->curr_job == job);
	spin_unlock_irqrestore(&sched->lock, flags);

	return running;
}

