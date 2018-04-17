/*
 * Copyright(c) 2016, 2017 Intel Corporation.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/random.h>
#include "hfi.h"
#include "qp.h"
#include "rc.h"
#include "verbs.h"
#include "tid_rdma.h"
#include "user_exp_rcv.h"
#include "trace.h"
#include <rdma/ib_umem.h>
#include "qp.h"

u32 tid_rdma_flow_wt;

void hfi1_add_tid_reap_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *qpriv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	if (!(qpriv->s_flags & HFI1_R_TID_RSC_TIMER)) {
		qpriv->s_flags |= HFI1_R_TID_RSC_TIMER;
		qpriv->s_tid_timer.expires = jiffies +
			qpriv->tid_timer_timeout_jiffies;
		add_timer(&qpriv->s_tid_timer);
	}
}

void hfi1_mod_tid_reap_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *qpriv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	qpriv->s_flags |= HFI1_R_TID_RSC_TIMER;
	mod_timer(&qpriv->s_tid_timer, jiffies +
		  qpriv->tid_timer_timeout_jiffies);
}

int hfi1_stop_tid_reap_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *qpriv = qp->priv;
	int rval = 0;

	lockdep_assert_held(&qp->s_lock);
	if (qpriv->s_flags & HFI1_R_TID_RSC_TIMER) {
		rval = del_timer(&qpriv->s_tid_timer);
		qpriv->s_flags &= ~HFI1_R_TID_RSC_TIMER;
	}
	return rval;
}

void hfi1_del_tid_reap_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *qpriv = qp->priv;

	del_timer_sync(&qpriv->s_tid_timer);
	qpriv->s_flags &= ~HFI1_R_TID_RSC_TIMER;
}

void hfi1_add_tid_retry_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct ib_qp *ibqp = &qp->ibqp;
	struct rvt_dev_info *rdi = ib_to_rvt(ibqp->device);

	lockdep_assert_held(&qp->s_lock);
	if (!(priv->s_flags & HFI1_S_TID_RETRY_TIMER)) {
		priv->s_flags |= HFI1_S_TID_RETRY_TIMER;
		priv->s_tid_retry_timer.expires = jiffies +
			priv->tid_retry_timeout_jiffies + rdi->busy_jiffies;
		add_timer(&priv->s_tid_retry_timer);
	}
}

void hfi1_mod_tid_retry_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct ib_qp *ibqp = &qp->ibqp;
	struct rvt_dev_info *rdi = ib_to_rvt(ibqp->device);

	lockdep_assert_held(&qp->s_lock);
	priv->s_flags |= HFI1_S_TID_RETRY_TIMER;
	mod_timer(&priv->s_tid_retry_timer, jiffies +
		  priv->tid_retry_timeout_jiffies + rdi->busy_jiffies);
}

int hfi1_stop_tid_retry_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	int rval = 0;

	lockdep_assert_held(&qp->s_lock);
	if (priv->s_flags & HFI1_S_TID_RETRY_TIMER) {
		rval = del_timer(&priv->s_tid_retry_timer);
		priv->s_flags &= ~HFI1_S_TID_RETRY_TIMER;
	}
	return rval;
}

void hfi1_del_tid_retry_timer(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;

	del_timer_sync(&priv->s_tid_retry_timer);
	priv->s_flags &= ~HFI1_S_TID_RETRY_TIMER;
}

static void tid_rdma_print_param(struct tid_rdma_params *local,
				 struct tid_rdma_params *remote)
{
	if (!local || !remote)
		return;

	hfi1_cdbg(OPFN, "TID RDMA State: %sABLED",
		  (remote ? "EN" : "DIS"));
	hfi1_cdbg(OPFN, "TID RDMA Local Params:");
	hfi1_cdbg(OPFN, "\tQP Num: 0x%x", local->qp);
	hfi1_cdbg(OPFN, "\tMax Read: %u", local->max_read);
	hfi1_cdbg(OPFN, "\tMax Write: %u", local->max_write);
	hfi1_cdbg(OPFN, "\tMax Length: %u", local->max_len);
	hfi1_cdbg(OPFN, "\tJ_KEY: 0x%x", local->jkey);
	hfi1_cdbg(OPFN, "\tTimeout: %u", local->timeout);
	hfi1_cdbg(OPFN, "TID RDMA Remote Params:");
	hfi1_cdbg(OPFN, "\tQP Num: 0x%x", remote->qp);
	hfi1_cdbg(OPFN, "\tMax Read: %u", remote->max_read);
	hfi1_cdbg(OPFN, "\tMax Write: %u", remote->max_write);
	hfi1_cdbg(OPFN, "\tMax Length: %u", remote->max_len);
	hfi1_cdbg(OPFN, "\tJ_KEY: 0x%x", remote->jkey);
	hfi1_cdbg(OPFN, "\tTimeout: %u", remote->timeout);
}

bool tid_rdma_conn_req(struct rvt_qp *qp, u64 *data)
{
	struct hfi1_qp_priv *priv = qp->priv;

	*data = (((u64)priv->tid_rdma.local.qp & 0xff) << 56) |
		((((u64)priv->tid_rdma.local.qp >> 16) & 0xff) << 48) |
		(((u64)((priv->tid_rdma.local.max_len >> PAGE_SHIFT) - 1) &
		  0xfff) << 37) |
		(((u64)priv->tid_rdma.local.timeout & 0x1f) << 32) |
		(((u64)priv->tid_rdma.local.jkey & 0xffff) << 16) |
		(((u64)priv->tid_rdma.local.max_read & 0x3f) << 10) |
		(((u64)priv->tid_rdma.local.max_write & 0x3f) << 4);
	return true;
}

bool tid_rdma_conn_reply(struct rvt_qp *qp, u64 data)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct tid_rdma_params *remote, *old;
	bool ret = true;

	old = rcu_dereference_protected(priv->tid_rdma.remote,
					lockdep_is_held(&priv->opfn.lock));
	data &= ~0xf;
	/*
	 * If data passed in is zero, return true so as not to continue the
	 * negotiation process
	 */
	if (!data || !HFI1_CAP_IS_KSET(TID_RDMA))
		goto null;
	/*
	 * If kzalloc fails, return false. This will result in:
	 * * at the requester a new OPFN request being generated to retry
	 *   the negotiation
	 * * at the responder, 0 being returned to the requester so as to
	 *   disable TID RDMA at both the requester and the responder
	 */
	remote = kzalloc(sizeof(*remote), GFP_ATOMIC);
	if (!remote) {
		ret = false;
		goto null;
	}

	remote->max_len = (((data >> 37) & 0xffff) + 1) << PAGE_SHIFT;
	remote->jkey = (data >> 16) & 0xffff;
	remote->max_write = (data >> 4) & 0x3f;
	remote->max_read = (data >> 10) & 0x3f;
	remote->qp = ((((data >> 48) & 0xff) << 16) |
			       ((data >> 56) & 0xff));
	remote->timeout = (data >> 32) & 0x1f;
	priv->tid_timer_timeout_jiffies =
		usecs_to_jiffies((((4096UL * (1UL << remote->timeout)) /
				   1000UL) << 3) * 7);
	tid_rdma_print_param(&priv->tid_rdma.local, remote);
	rcu_assign_pointer(priv->tid_rdma.remote, remote);
	/*
	 * A TID RDMA READ request's segment size is not equal to
	 * remote->max_len only when the request's data length is smaller
	 * than remote->max_len. In that case, there will be only one segment.
	 * Therefore, when priv->pkts_ps is used to calculate req->cur_seg
	 * during retry, it will lead to req->cur_seg = 0, which is exactly
	 * what is expected.
	 */
	priv->pkts_ps = (u16) rvt_div_mtu(qp, remote->max_len);
	priv->timeout_shift = ilog2(priv->pkts_ps - 1) + 1;
	goto free;
null:
	RCU_INIT_POINTER(priv->tid_rdma.remote, NULL);
	priv->timeout_shift = 0;
free:
	if (old)
		kfree_rcu(old, rcu_head);
	return ret;
}

bool tid_rdma_conn_resp(struct rvt_qp *qp, u64 *data)
{
	bool ret;

	ret = tid_rdma_conn_reply(qp, *data);
	*data = 0;
	/*
	 * If tid_rdma_conn_reply() returns error, set *data as 0 to indicate
	 * TID RDMA could not be enabled. This will result in TID RDMA being
	 * disabled at the requester too.
	 */
	if (ret)
		(void)tid_rdma_conn_req(qp, data);
	return ret;
}

void tid_rdma_conn_error(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct tid_rdma_params *old;

	old = rcu_dereference_protected(priv->tid_rdma.remote,
					lockdep_is_held(&priv->opfn.lock));
	RCU_INIT_POINTER(priv->tid_rdma.remote, NULL);
	if (old)
		kfree_rcu(old, rcu_head);
}

/**
 * kern_tid_node - used for managing TID's in TID groups
 *
 * @grp: TID group referred to by this TID node
 * @map: grp->map captured prior to programming this TID group in HW
 * @cnt: Only @cnt of available group entries are actually programmed
 */
struct kern_tid_node {
	struct tid_group *grp;
	u8 map;
	u8 cnt;
};

/**
 * DOC: lock ordering
 *
 * There are two locks involved with the queuing
 * routines: the qp s_lock and the exp_lock.
 *
 * Since the tid space allocation is called from
 * the send engine, the qp s_lock is already held.
 *
 * The allocation routines will get the exp_lock.
 *
 * The first_qp() call is provided to allow the head of
 * the rcd wait queue to be fetched under the exp_lock and
 * followed by a drop of the exp_lock.
 *
 * Any qp in the wait list will have the qp reference count held
 * to hold the qp in memory.
 */

/*
 * return head of rcd wait list
 *
 * Must hold the exp_lock.
 *
 * Get a reference to the QP to hold the QP in memory.
 *
 * The caller must release the reference when the local
 * is no longer being used.
 */
static struct rvt_qp *first_qp(struct hfi1_ctxtdata *rcd,
			       struct tid_queue *queue)
	__must_hold(&rcd->exp_lock)
{
	struct hfi1_qp_priv *priv;

	lockdep_assert_held(&rcd->exp_lock);
	priv = list_first_entry_or_null(
			&queue->queue_head,
			struct hfi1_qp_priv,
			tid_wait);
	if (!priv)
		return NULL;
	rvt_get_qp(priv->owner);
	return priv->owner;
}

/**
 * kernel_tid_waiters - determine rcd wait
 * @rcd: the receive context
 * @qp: the head of the qp being processed
 *
 * This routine will return false IFF
 * the list is NULL or the head of the
 * list is the indicated qp.
 *
 * Must hold the qp s_lock and the exp_lock.
 *
 * Return:
 * false if all the conditions below are statisfied:
 * 1. The list is empty or
 * 2. The indicated qp is at the head of the list and the
 *    RVT_S_WAIT_TID_SPACE is not set.
 * true is returned otherwise.
 */
static bool kernel_tid_waiters(struct hfi1_ctxtdata *rcd,
			       struct tid_queue *queue, struct rvt_qp *qp)
	__must_hold(&rcd->exp_lock) __must_hold(&qp->s_lock)
{
	struct rvt_qp *fqp;
	bool ret = true;

	lockdep_assert_held(&qp->s_lock);
	lockdep_assert_held(&rcd->exp_lock);
	fqp = first_qp(rcd, queue);
	if (!fqp || (fqp == qp && (qp->s_flags & RVT_S_WAIT_TID_SPACE)))
		ret = false;
	rvt_put_qp(fqp);
	return ret;
}

/**
 * dequeue_tid_waiter - dequeue the qp from the list
 * @qp - the qp to remove the wait list
 *
 * This routine removes the indicated qp from the
 * wait list if it is there.
 *
 * This should be done after the hardware flow and
 * tid array resources have been allocated.
 *
 * Must hold the qp s_lock and the rcd exp_lock.
 *
 * It assumes the s_lock to protect the s_flags
 * field and to reliably test the RVT_S_WAIT_TID_SPACE flag.
 */
static void dequeue_tid_waiter(struct hfi1_ctxtdata *rcd,
			       struct tid_queue *queue, struct rvt_qp *qp)
	__must_hold(&rcd->exp_lock) __must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *priv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	lockdep_assert_held(&rcd->exp_lock);
	if (list_empty(&priv->tid_wait))
		return;
	list_del_init(&priv->tid_wait);
	qp->s_flags &= ~RVT_S_WAIT_TID_SPACE;
	queue->dequeue++;
	rvt_put_qp(qp);
}

/**
 * queue_qp_for_tid_wait - suspend QP on tid space
 * @rcd: the receive context
 * @qp: the qp
 *
 * The qp is inserted at the tail of the rcd
 * wait queue and the RVT_S_WAIT_TID_SPACE s_flag is set.
 *
 * Must hold the qp s_lock and the exp_lock.
 */
static void queue_qp_for_tid_wait(struct hfi1_ctxtdata *rcd,
				  struct tid_queue *queue, struct rvt_qp *qp)
	__must_hold(&rcd->exp_lock) __must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *priv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	lockdep_assert_held(&rcd->exp_lock);
	if (list_empty(&priv->tid_wait)) {
		qp->s_flags |= RVT_S_WAIT_TID_SPACE;
		list_add_tail(&priv->tid_wait, &queue->queue_head);
		priv->tid_enqueue = ++queue->enqueue;
		rcd->dd->verbs_dev.n_tidwait++;
		trace_hfi1_qpsleep(qp, RVT_S_WAIT_TID_SPACE);
		rvt_get_qp(qp);
	}
}

/**
 * __trigger_tid_waiter - trigger tid waiter
 * @qp: the qp
 *
 * This is a private entrance to schedule the qp
 * assuming the caller is holding the qp->s_lock.
 */
static void __trigger_tid_waiter(struct rvt_qp *qp)
	__must_hold(&qp->s_lock)
{
	lockdep_assert_held(&qp->s_lock);
	if (!(qp->s_flags & RVT_S_WAIT_TID_SPACE))
		return;
	trace_hfi1_qpwakeup(qp, RVT_S_WAIT_TID_SPACE);
	hfi1_schedule_send(qp);
}

/**
 * tid_rdma_schedule_tid_wakeup - schedule wakeup for a qp
 * @qp - the qp
 *
 * trigger a schedule or a waiting qp in a deadlock
 * safe manner.  The qp reference is held prior
 * to this call via first_qp().
 *
 * If the qp trigger was already scheduled (!rval)
 * the the reference is dropped, otherwise the resume
 * or the destroy cancel will dispatch the reference.
 */
static void tid_rdma_schedule_tid_wakeup(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv;
	struct hfi1_ibport *ibp;
	struct hfi1_pportdata *ppd;
	struct hfi1_devdata *dd;
	bool rval;

	if (!qp)
		return;

	priv = qp->priv;
	ibp = to_iport(qp->ibqp.device, qp->port_num);
	ppd = ppd_from_ibp(ibp);
	dd = dd_from_ibdev(qp->ibqp.device);

	rval = queue_work_on(
		priv->s_sde ?
			priv->s_sde->cpu :
			cpumask_first(cpumask_of_node(dd->node)),
		ppd->hfi1_wq,
		&priv->tid_rdma.trigger_work);
	if (!rval)
		rvt_put_qp(qp);
}

/**
 * tid_rdma_trigger_resume - field a trigger work request
 * @work - the work item
 *
 * Complete the off qp trigger processing by directly
 * calling the progress routine.
 */
void tid_rdma_trigger_resume(struct work_struct *work)
{
	struct tid_rdma_qp_params *tr;
	struct hfi1_qp_priv *priv;
	struct rvt_qp *qp;

	tr = container_of(work, struct tid_rdma_qp_params, trigger_work);
	priv = container_of(tr, struct hfi1_qp_priv, tid_rdma);
	qp = priv->owner;
	spin_lock_irq(&qp->s_lock);
	if (qp->s_flags & RVT_S_WAIT_TID_SPACE) {
		spin_unlock_irq(&qp->s_lock);
		hfi1_do_send(priv->owner, true);
	} else {
		spin_unlock_irq(&qp->s_lock);
	}
	rvt_put_qp(qp);
}

/**
 * tid_rdma_flush_wait - unwind any tid space wait
 *
 * This is called when resetting a qp to
 * allow a destroy or reset to get rid
 * of any tid space linkage and reference counts.
 */
static void _tid_rdma_flush_wait(struct rvt_qp *qp, struct tid_queue *queue)
	__must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *priv;

	if (!qp)
		return;
	lockdep_assert_held(&qp->s_lock);
	priv = qp->priv;
	qp->s_flags &= ~RVT_S_WAIT_TID_SPACE;
	spin_lock(&priv->rcd->exp_lock);
	if (!list_empty(&priv->tid_wait)) {
		list_del_init(&priv->tid_wait);
		qp->s_flags &= ~RVT_S_WAIT_TID_SPACE;
		queue->dequeue++;
		rvt_put_qp(qp);
	}
	spin_unlock(&priv->rcd->exp_lock);
}

void tid_rdma_flush_wait(struct rvt_qp *qp)
	__must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *priv = qp->priv;

	_tid_rdma_flush_wait(qp, &priv->rcd->flow_queue);
	_tid_rdma_flush_wait(qp, &priv->rcd->rarr_queue);
}

static inline u32 qpn(struct tid_rdma_flow *flow)
{
	return flow->req->qp->ibqp.qp_num;
}

void hfi1_compute_tid_rdma_flow_wt(void)
{
	/*
	 * Heuristic for computing the RNR timeout when waiting on the flow
	 * queue. Rather than a computationaly expensive exact estimate of when
	 * a flow will be available, we assume that if a QP is at position N in
	 * the flow queue it has to wait approximately (N + 1) * (number of
	 * segments between two sync points), assuming PMTU of 4K. The rationale
	 * for this is that flows are released and recycled at each sync point.
	 */
	tid_rdma_flow_wt = MAX_TID_FLOW_PSN * enum_to_mtu(OPA_MTU_4096) /
		hfi1_tid_rdma_seg_max_size;
}

/**
 * kern_reserve_flow - allocate a hardware flow
 * @rcd - the context to use for allocation
 * @last - the index of the preferred flow. Use RXE_NUM_TID_FLOWS to
 *         signify "don't care".
 *
 * Use a bit mask based allocation to reserve a hardware
 * flow for use in receiving KDETH data packets. If a preferred flow is
 * specified the function will attempt to reserve that flow again, if
 * available.
 *
 * The exp_lock must be held.
 *
 * Return:
 * On success: a value postive value between 0 and RXE_NUM_TID_FLOWS - 1
 * On failure: -EAGAIN
 */
static int kern_reserve_flow(struct hfi1_ctxtdata *rcd, int last)
	__must_hold(&rcd->exp_lock)
{
	int nr;

	/* Attempt to reserve the preferred flow index */
	if (last >= 0 && last < RXE_NUM_TID_FLOWS &&
	    !test_and_set_bit(last, &rcd->flow_mask))
		return last;

	nr = ffz(rcd->flow_mask);
	BUILD_BUG_ON(
		RXE_NUM_TID_FLOWS >= (sizeof(rcd->flow_mask) * BITS_PER_BYTE));
	if (nr > (RXE_NUM_TID_FLOWS - 1))
		return -EAGAIN;
	set_bit(nr, &rcd->flow_mask);
	return nr;
}

static void kern_set_hw_flow(struct hfi1_ctxtdata *rcd, u32 generation,
			     u32 flow_idx)
{
	u64 reg;

	reg = ((u64)generation << HFI1_KDETH_BTH_SEQ_SHIFT) |
		RCV_TID_FLOW_TABLE_CTRL_FLOW_VALID_SMASK |
		RCV_TID_FLOW_TABLE_CTRL_KEEP_AFTER_SEQ_ERR_SMASK |
		RCV_TID_FLOW_TABLE_CTRL_KEEP_ON_GEN_ERR_SMASK |
		RCV_TID_FLOW_TABLE_STATUS_SEQ_MISMATCH_SMASK |
		RCV_TID_FLOW_TABLE_STATUS_GEN_MISMATCH_SMASK;

	if (generation != KERN_GENERATION_RESERVED)
		reg |= RCV_TID_FLOW_TABLE_CTRL_HDR_SUPP_EN_SMASK;

	write_uctxt_csr(rcd->dd, rcd->ctxt,
			RCV_TID_FLOW_TABLE + 8 * flow_idx, reg);
}

static u32 kern_setup_hw_flow(struct hfi1_ctxtdata *rcd, u32 flow_idx)
	__must_hold(&rcd->exp_lock)
{
	u32 generation = rcd->flows[flow_idx].generation;

	kern_set_hw_flow(rcd, generation, flow_idx);
	return generation;
}

static void kern_clear_hw_flow(struct hfi1_ctxtdata *rcd, u32 flow_idx)
	__must_hold(&rcd->exp_lock)
{
	u32 generation;

	generation = mask_generation(rcd->flows[flow_idx].generation + 1);
	if (generation == KERN_GENERATION_RESERVED)
		generation = mask_generation(generation + 1);
	rcd->flows[flow_idx].generation = generation;
	kern_set_hw_flow(rcd, KERN_GENERATION_RESERVED, flow_idx);
}

int hfi1_kern_setup_hw_flow(struct hfi1_ctxtdata *rcd, struct rvt_qp *qp)
{
	struct hfi1_qp_priv *qpriv = (struct hfi1_qp_priv *)qp->priv;
	struct tid_flow_state *fs = &qpriv->flow_state;
	struct rvt_qp *fqp;
	unsigned long flags;
	int ret = 0;

	/* The QP already has an allocated flow */
	if (fs->index != RXE_NUM_TID_FLOWS)
		return ret;

	spin_lock_irqsave(&rcd->exp_lock, flags);
	if (kernel_tid_waiters(rcd, &rcd->flow_queue, qp))
		goto queue;

	ret = kern_reserve_flow(rcd, fs->last_index);
	if (ret < 0)
		goto queue;
	fs->index = ret;
	fs->last_index = fs->index;
	fs->generation = kern_setup_hw_flow(rcd, fs->index);
	fs->psn = 0;
	fs->flags = 0;
	dequeue_tid_waiter(rcd, &rcd->flow_queue, qp);
	/* get head before dropping lock */
	fqp = first_qp(rcd, &rcd->flow_queue);
	spin_unlock_irqrestore(&rcd->exp_lock, flags);

	tid_rdma_schedule_tid_wakeup(fqp);
	return 0;
queue:
	queue_qp_for_tid_wait(rcd, &rcd->flow_queue, qp);
	spin_unlock_irqrestore(&rcd->exp_lock, flags);
	return -EAGAIN;
}

void hfi1_kern_clear_hw_flow(struct hfi1_ctxtdata *rcd, struct rvt_qp *qp)
{
	struct hfi1_qp_priv *qpriv = (struct hfi1_qp_priv *)qp->priv;
	struct tid_flow_state *fs = &qpriv->flow_state;
	struct rvt_qp *fqp;
	unsigned long flags;

	if (fs->index >= RXE_NUM_TID_FLOWS)
		return;
	spin_lock_irqsave(&rcd->exp_lock, flags);
	kern_clear_hw_flow(rcd, fs->index);
	clear_bit(fs->index, &rcd->flow_mask);
	fs->index = RXE_NUM_TID_FLOWS;
	fs->psn = 0;
	/* get head before dropping lock */
	fqp = first_qp(rcd, &rcd->flow_queue);
	spin_unlock_irqrestore(&rcd->exp_lock, flags);

	if (fqp == qp) {
		__trigger_tid_waiter(fqp);
		rvt_put_qp(fqp);
	} else {
		tid_rdma_schedule_tid_wakeup(fqp);
	}
}

void hfi1_kern_init_ctxt_generations(struct hfi1_ctxtdata *rcd)
{
	int i;

	for (i = 0; i < RXE_NUM_TID_FLOWS; i++) {
		rcd->flows[i].generation = mask_generation(prandom_u32());
		kern_set_hw_flow(rcd, KERN_GENERATION_RESERVED, i);
	}
}

/**
 * tid_rdma_find_phys_blocks_4k - get groups base on mr info
 * @npages - number of pages
 * @list - page set array to return
 *
 * This routine returns the number of groups associated with
 * the current sge information.  This implementation is based
 * on the expected receive find_phys_blocks() adjusted to
 * use the MR information vs. the pfn.
 *
 * Return:
 * the number of RcvArray entries
 */
static u32 tid_rdma_find_phys_blocks_4k(struct tid_rdma_flow *flow,
					u32 npages,
					struct tid_rdma_pageset *list)
{
	u32 pagecount, pageidx, setcount = 0, i;
	void *vaddr, *this_vaddr;

	if (!npages)
		return 0;

	/*
	 * Look for sets of physically contiguous pages in the user buffer.
	 * This will allow us to optimize Expected RcvArray entry usage by
	 * using the bigger supported sizes.
	 */
	vaddr = page_address(flow->pages[0]);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TID, "QP%u page[%u] page %p vaddr %p",
		  qpn(flow), 0, flow->pages[0], vaddr);
#endif
	for (pageidx = 0, pagecount = 1, i = 1; i <= npages; i++) {
		this_vaddr = i < npages ? page_address(flow->pages[i]) : NULL;
#ifdef TIDRDMA_DEBUG
		if (this_vaddr)
			hfi1_cdbg(TID, "QP%u page[%u] page %p vaddr %p",
				  qpn(flow), i, flow->pages[i], this_vaddr);
#endif
		/*
		 * If the vaddr's are not sequential, pages are not physically
		 * contiguous.
		 */
		if (this_vaddr != (vaddr + PAGE_SIZE)) {
			/*
			 * At this point we have to loop over the set of
			 * physically contiguous pages and break them down it
			 * sizes supported by the HW.
			 * There are two main constraints:
			 *     1. The max buffer size is MAX_EXPECTED_BUFFER.
			 *        If the total set size is bigger than that
			 *        program only a MAX_EXPECTED_BUFFER chunk.
			 *     2. The buffer size has to be a power of two. If
			 *        it is not, round down to the closes power of
			 *        2 and program that size.
			 */
			while (pagecount) {
				int maxpages = pagecount;
				u32 bufsize = pagecount * PAGE_SIZE;

				if (bufsize > MAX_EXPECTED_BUFFER)
					maxpages =
						MAX_EXPECTED_BUFFER >>
						PAGE_SHIFT;
				else if (!is_power_of_2(bufsize))
					maxpages =
						rounddown_pow_of_two(bufsize) >>
						PAGE_SHIFT;

				list[setcount].idx = pageidx;
				list[setcount].count = maxpages;
#ifdef TIDRDMA_DEBUG
				hfi1_cdbg(TID, "QP%u list[%u] idx %u count %u",
					  qpn(flow), setcount,
					  list[setcount].idx,
					  list[setcount].count);
#endif
				pagecount -= maxpages;
				pageidx += maxpages;
				setcount++;
			}
			pageidx = i;
			pagecount = 1;
			vaddr = this_vaddr;
		} else {
			vaddr += PAGE_SIZE;
			pagecount++;
		}
	}
	/* insure we always return an even number of sets */
	if (setcount & 1)
		list[setcount++].count = 0;
	return setcount;
}

/**
 * tid_flush_pages - dump out pages into pagesets
 * @list - list of pagesets
 * @idx - pointer to current page index
 * @pages - number of pages to dump
 * @sets - current number of pagesset
 *
 * This routine flushes out accumuated pages.
 *
 * To insure an even number of sets the
 * code may add a filler.
 *
 * This can happen with when pages is not
 * a power of 2 or pages is a power of 2
 * less than the maximum pages.
 *
 * Return:
 * The new number of sets
 */

static u32 tid_flush_pages(struct tid_rdma_pageset *list,
			   u32 *idx, u32 pages, u32 sets)
{
	while (pages) {
		u32 maxpages = pages;

		if (maxpages > MAX_EXPECTED_PAGES)
			maxpages = MAX_EXPECTED_PAGES;
		else if (!is_power_of_2(maxpages))
			maxpages = rounddown_pow_of_two(maxpages);
		list[sets].idx = *idx;
		list[sets++].count = maxpages;
		*idx += maxpages;
		pages -= maxpages;
	}
	/* might need a filler */
	if (sets & 1)
		list[sets++].count = 0;
	return sets;
}

/**
 * tid_rdma_find_phys_blocks_8k - get groups base on mr info
 * @npages - number of pages
 * @list - page set array to return
 *
 * This routine parses an array of pages to compute pagesets
 * in an 8k compatible way.
 *
 * pages are tested two at a time, i, i + 1 for contiguous
 * pages and i - 1 and i contiguous pages.
 *
 * If any condition is false, any accumlated pages are flushed and
 * v0,v1 are emitted as separate PAGE_SIZE pagesets
 *
 * Otherwise, the current 8k is totaled for a future flush.
 *
 * Return:
 * The number of pagesets
 * list set with the returned number of pagesets
 *
 */
static u32 tid_rdma_find_phys_blocks_8k(struct tid_rdma_flow *flow,
					u32 npages,
					struct tid_rdma_pageset *list)
{
	u32 idx, sets = 0, i;
	u32 pages = 0;
	void *v0, *v1, *vm1;

	if (!npages)
		return 0;
	for (idx = 0, i = 0, vm1 = NULL; i < npages; i += 2) {
		/* get a new v0 */
		v0 = page_address(flow->pages[i]);
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TID, "QP%u page[%u] page %p v0 %p",
			  qpn(flow), i, flow->pages[i], v0);
#endif
		v1 = i + 1 < npages ?
				page_address(flow->pages[i + 1]) : NULL;
#ifdef TIDRDMA_DEBUG
		if (v1)
			hfi1_cdbg(TID, "QP%u page[%u] page %p v1 %p",
				  qpn(flow), i + 1, flow->pages[i + 1],
				  v1);
#endif
		/* compare i, i + 1 vaddr */
		if (v1 != (v0 + PAGE_SIZE)) {
			/* flush out pages */
			sets = tid_flush_pages(list, &idx, pages, sets);
			/* output v0,v1 as two pagesets */
			list[sets].idx = idx++;
			list[sets++].count = 1;
			if (v1) {
				list[sets].count = 1;
				list[sets++].idx = idx++;
			} else {
				list[sets++].count = 0;
			}
			vm1 = NULL;
			pages = 0;
			continue;
		}
		/* i,i+1 consecutive, look at i-1,i */
		if (vm1 && v0 != (vm1 + PAGE_SIZE)) {
			/* flush out pages */
			sets = tid_flush_pages(list, &idx, pages, sets);
			pages = 0;
		}
		/* pages will always be a multiple of 8k */
		pages += 2;
		/* save i-1 */
		vm1 = v1;
		/* move to next pair */
	}
	/* dump residual pages at end */
	sets = tid_flush_pages(list, &idx, npages - idx, sets);
	/* by design cannot be odd sets */
	WARN_ON(sets & 1);
	return sets;
}

/**
 * Find pages for one segment of a sge array represented by @ss. The function
 * does not check the sge, the sge must have been checked for alignment with a
 * prior call to hfi1_kern_trdma_ok. Other sge checking is done as part of
 * rvt_lkey_ok and rvt_rkey_ok. Also, the function only modifies the local sge
 * copy maintained in @ss->sge, the original sge is not modified.
 *
 * Unlike IB RDMA WRITE, we can't decrement ss->num_sge here because we are not
 * releasing the MR reference count at the same time. Otherwise, we'll "leak"
 * references to the MR. This difference requires that we keep track of progress
 * into the sg_list. This is done by the cur_seg cursor in the tid_rdma_request
 * structure.
 */
static u32 kern_find_pages(struct tid_rdma_flow *flow,
			   struct rvt_sge_state *ss, bool *last)
{
	struct tid_rdma_request *req = flow->req;
	struct rvt_sge *sge = &ss->sge;
	u32 length = flow->req->seg_len;
	u32 len = PAGE_SIZE;
	u32 i = 0;

	while (length && req->isge < ss->num_sge) {
		flow->pages[i++] = virt_to_page(sge->vaddr);

		sge->vaddr += len;
		sge->length -= len;
		sge->sge_length -= len;
		if (!sge->sge_length) {
			if (++req->isge < ss->num_sge)
				*sge = ss->sg_list[req->isge - 1];
		} else if (sge->length == 0 && sge->mr->lkey) {
			if (++sge->n >= RVT_SEGSZ) {
				++sge->m;
				sge->n = 0;
			}
			sge->vaddr = sge->mr->map[sge->m]->segs[sge->n].vaddr;
			sge->length = sge->mr->map[sge->m]->segs[sge->n].length;
		}
		length -= len;
	}

	flow->length = flow->req->seg_len - length;
	*last = req->isge == ss->num_sge ? false : true;
	return i;
}

static void dma_unmap_flow(struct tid_rdma_flow *flow)
{
	struct hfi1_devdata *dd;
	int i;

	/*
	 * The flow's back-pointer to the request is only set when
	 * the flow is initialized. This does not happen for non-TID
	 * RDMA requests.
	 * So, if the request pointer is NULL, there is nothing to
	 * do.
	 */
	if (!flow->req)
		return;
	dd = flow->req->rcd->dd;
	for (i = 0; i < flow->npagesets; i++) {
		struct tid_rdma_pageset *pset = &flow->pagesets[i];

		if (pset->count && pset->addr){
			dma_unmap_page(&dd->pcidev->dev,
				       pset->addr,
				       PAGE_SIZE * pset->count,
				       DMA_FROM_DEVICE);
			pset->addr = 0;
		}
	}
}

static int dma_map_flow(struct tid_rdma_flow *flow)
{
	int i;
	struct hfi1_devdata *dd = flow->req->rcd->dd;

	for (i = 0; i < flow->npagesets; i++) {
		struct tid_rdma_pageset *pset = &flow->pagesets[i];

		if (pset->count) {
			pset->addr = dma_map_page(&dd->pcidev->dev,
						  flow->pages[pset->idx],
						  0,
						  PAGE_SIZE * pset->count,
						  DMA_FROM_DEVICE);

			if (dma_mapping_error(&dd->pcidev->dev, pset->addr)) {
				dma_unmap_flow(flow);
				return -ENOMEM;
			}
		}
	}
	return 0;
}

static inline bool dma_mapped(struct tid_rdma_flow *flow)
{
	return !!flow->pagesets[0].addr;
}

/*
 * Get pages pointers and identify contiguous physical memory chunks for a
 * segment. All segments are of length flow->req->seg_len.
 */
static int kern_get_phys_blocks(struct tid_rdma_flow *flow,
				 struct rvt_sge_state *ss, bool *last)
{

	/* Reuse previously computed pagesets, if any */
	if (flow->npagesets) {
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TID, "QP%u Reusing prev pagesets: pages %u psets %u",
			  qpn(flow), flow->npages, flow->npagesets);
#endif
		if (!dma_mapped(flow))
			return dma_map_flow(flow);
		return 0;
	}

	flow->npages = kern_find_pages(flow, ss, last);

	if (flow->req->qp->pmtu == enum_to_mtu(OPA_MTU_4096))
		flow->npagesets = tid_rdma_find_phys_blocks_4k(
					flow,
					flow->npages,
					flow->pagesets);
	else
		flow->npagesets = tid_rdma_find_phys_blocks_8k(
					flow,
					flow->npages,
					flow->pagesets);

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TID, "QP%u segment: pages %u psets %u",
		  qpn(flow), flow->npages, flow->npagesets);
#endif

	return dma_map_flow(flow);
}

static inline void kern_add_tid_node(struct tid_rdma_flow *flow, char *s,
				     struct tid_group *grp, u8 cnt)
{
	struct kern_tid_node *node = &flow->tnode[flow->tnode_cnt++];

	WARN_ON_ONCE(flow->tnode_cnt >=
		     (hfi1_tid_rdma_seg_max_size >> PAGE_SHIFT));
	if (WARN_ON_ONCE(cnt & 1)) {
		struct hfi1_ctxtdata *rcd = flow->req->rcd;
		struct hfi1_devdata *dd = rcd->dd;

		dd_dev_err(dd, "unexpected odd allocation cnt %u map 0x%x used %u",
			   cnt, grp->map, grp->used);
	}
	node->grp = grp;
	node->map = grp->map;
	node->cnt = cnt;
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TID, "QP%u %s idx %d grp base 0x%x map 0x%x used %u cnt %u",
		  qpn(flow), s, flow->tnode_cnt - 1, grp->base, grp->map,
		  grp->used, cnt);
#endif
}

/*
 * Try to allocate pageset_count TID's from TID groups for a context
 *
 * This function allocates TID's without moving groups between lists or
 * modifying grp->map. This is done as follows, being cogizant of the lists
 * between which the TID groups will move:
 * 1. First allocate complete groups of 8 TID's since this is more efficient,
 *    these groups will move from group->full without affecting used
 * 2. If more TID's are needed allocate from used (will move from used->full or
 *    stay in used)
 * 3. If we still don't have the required number of TID's go back and look again
 *    at a complete group (will move from group->used)
 */
static int kern_alloc_tids(struct tid_rdma_flow *flow)
{
	struct hfi1_ctxtdata *rcd = flow->req->rcd;
	struct hfi1_devdata *dd = rcd->dd;
	u32 ngroups, pageidx = 0;
	struct tid_group *group = NULL, *used;
	u8 use;

	flow->tnode_cnt = 0;
	ngroups = flow->npagesets / dd->rcv_entries.group_size;
	if (!ngroups)
		goto used_list;

	/* First look at complete groups */
	list_for_each_entry(group,  &rcd->tid_group_list.list, list) {
		kern_add_tid_node(flow, "complete groups", group, group->size);

		pageidx += group->size;
		if (!--ngroups)
			break;
	}

	if (pageidx >= flow->npagesets)
		goto ok;

used_list:
	/* Now look at partially used groups */
	list_for_each_entry(used, &rcd->tid_used_list.list, list) {
		use = min_t(u32, flow->npagesets - pageidx,
			    used->size - used->used);
		kern_add_tid_node(flow, "used groups", used, use);

		pageidx += use;
		if (pageidx >= flow->npagesets)
			goto ok;
	}

	/*
	 * Look again at a complete group, continuing from where we left.
	 * However, if we are at the head, we have reached the end of the
	 * complete groups list from the first loop above
	 */
	if (group && &group->list == &rcd->tid_group_list.list)
		goto bail_eagain;
	group = list_prepare_entry(group, &rcd->tid_group_list.list,
				   list);
	if (list_is_last(&group->list, &rcd->tid_group_list.list))
		goto bail_eagain;
	group = list_next_entry(group, list);
	use = min_t(u32, flow->npagesets - pageidx, group->size);
	kern_add_tid_node(flow, "complete continue", group, use);
	pageidx += use;
	if (pageidx >= flow->npagesets)
		goto ok;
bail_eagain:
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TID, "QP%u insufficient tids: needed %u have %u",
		  qpn(flow), flow->npagesets, pageidx);
#endif
	return -EAGAIN;
ok:
	return 0;
}

static void kern_program_rcv_group(struct tid_rdma_flow *flow, int grp_num,
				   u32 *pset_idx)
{
	struct hfi1_ctxtdata *rcd = flow->req->rcd;
	struct hfi1_devdata *dd = rcd->dd;
	struct kern_tid_node *node = &flow->tnode[grp_num];
	struct tid_group *grp = node->grp;
	struct tid_rdma_pageset *pset;
	u32 pmtu_pg = flow->req->qp->pmtu >> PAGE_SHIFT;
	u32 rcventry, npages = 0, pair = 0, tidctrl;
	u8 i, cnt = 0;

	for (i = 0; i < grp->size; i++) {
		rcventry = grp->base + i;

		if (node->map & BIT(i) || cnt >= node->cnt) {
			rcv_array_wc_fill(dd, rcventry);
			continue;
		}
		pset = &flow->pagesets[(*pset_idx)++];
		if (pset->count) {
			hfi1_put_tid(dd, rcventry, PT_EXPECTED,
				     pset->addr,
				     ilog2(pset->count) + 1);
		} else {
			hfi1_put_tid(dd, rcventry, PT_INVALID, 0, 0);
		}
		npages += pset->count;

		rcventry -= rcd->expected_base;
		tidctrl = pair ? 0x3 : rcventry & 0x1 ? 0x2 : 0x1;
		/*
		 * A single TID entry will be used to use a rcvarr pair (with
		 * tidctrl 0x3), if ALL these are true (a) the bit pos is even
		 * (b) the group map shows current and the next bits as free
		 * indicating two consecutive rcvarry entries are available (c)
		 * we actually need 2 more entries
		 */
		pair = !(i & 0x1) && !((node->map >> i) & 0x3) &&
			node->cnt >= cnt + 2;
		if (!pair) {
			if (!pset->count)
				tidctrl = 0x1;
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TID, "QP%u: tid idx %u ctrl 0x%x len %u",
				  qpn(flow), flow->tidcnt, tidctrl, npages);
#endif
			flow->tid_entry[flow->tidcnt++] =
				EXP_TID_SET(IDX, rcventry >> 1) |
				EXP_TID_SET(CTRL, tidctrl) |
				EXP_TID_SET(LEN, npages);
			/* Efficient DIV_ROUND_UP(npages, pmtu_pg) */
			flow->npkts += (npages + pmtu_pg - 1) >> ilog2(pmtu_pg);
			npages = 0;
		}

		if (grp->used == grp->size - 1)
			tid_group_move(grp, &rcd->tid_used_list,
				       &rcd->tid_full_list);
		else if (!grp->used)
			tid_group_move(grp, &rcd->tid_group_list,
				       &rcd->tid_used_list);

		grp->used++;
		grp->map |= BIT(i);
		cnt++;
	}
}

static void kern_unprogram_rcv_group(struct tid_rdma_flow *flow, int grp_num)
{
	struct hfi1_ctxtdata *rcd = flow->req->rcd;
	struct hfi1_devdata *dd = rcd->dd;
	struct kern_tid_node *node = &flow->tnode[grp_num];
	struct tid_group *grp = node->grp;
	u32 rcventry;
	u8 i, cnt = 0;

	for (i = 0; i < grp->size; i++) {
		rcventry = grp->base + i;

		if (node->map & BIT(i) || cnt >= node->cnt) {
			rcv_array_wc_fill(dd, rcventry);
			continue;
		}

		hfi1_put_tid(dd, rcventry, PT_INVALID, 0, 0);

		grp->used--;
		grp->map &= ~BIT(i);
		cnt++;

		if (grp->used == grp->size - 1)
			tid_group_move(grp, &rcd->tid_full_list,
				       &rcd->tid_used_list);
		else if (!grp->used)
			tid_group_move(grp, &rcd->tid_used_list,
				       &rcd->tid_group_list);
	}
	if (WARN_ON_ONCE(cnt & 1)) {
		struct hfi1_ctxtdata *rcd = flow->req->rcd;
		struct hfi1_devdata *dd = rcd->dd;

		dd_dev_err(dd, "unexpected odd free cnt %u map 0x%x used %u",
			   cnt, grp->map, grp->used);
	}
}

static void kern_program_rcvarray(struct tid_rdma_flow *flow)
{
	u32 pset_idx = 0;
	int i;

	flow->npkts = 0;
	flow->tidcnt = 0;
	for (i = 0; i < flow->tnode_cnt; i++) {
		kern_program_rcv_group(flow, i, &pset_idx);
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TID, "QP%u pset_idx %u", qpn(flow), pset_idx);
#endif
	}
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TID, "QP%u flow %u grps %u tidcnt %u pages %u npkts %u",
		  qpn(flow), flow->idx, flow->tnode_cnt, flow->tidcnt,
		  flow->npages, flow->npkts);
#endif
}

/**
 * hfi1_kern_exp_rcv_setup() - setup TID's and flow for one segment of a
 * TID RDMA request
 *
 * @req: TID RDMA request for which the segment/flow is being set up
 * @ss: sge state, maintains state across successive segments of a sge
 * @last: set to true after the last sge segment has been processed
 *
 * This function
 * (1) finds a free flow entry in the flow circular buffer
 * (2) finds pages and continuous physical chunks constituing one segment
 *     of an sge
 * (3) allocates TID group entries for those chunks
 * (4) programs rcvarray entries in the hardware corresponding to those
 *     TID's
 * (5) computes a tidarray with formatted TID entries which can be sent
 *     to the sender
 * (6) Reserves and programs HW flows.
 * (7) It also manages queing the QP when TID/flow resources are not
 *     available.
 *
 * @req points to struct tid_rdma_request of which the segments are a part. The
 * function uses qp, rcd and seg_len members of @req. In the absence of errors,
 * req->flow_idx is the index of the flow which has been prepared in this
 * invocation of function call. With flow = &req->flows[req->flow_idx],
 * flow->tid_entry contains the TID array which the sender can use for TID RDMA
 * sends and flow->npkts contains number of packets required to send the
 * segment.
 *
 * hfi1_check_sge_align should be called prior to calling this function and if
 * it signals error TID RDMA cannot be used for this sge and this function
 * should not be called.
 *
 * For the queuing, caller must hold the flow->req->qp s_lock from the send
 * engine and the function will procure the exp_lock.
 *
 * Return:
 * The function returns -EAGAIN if sufficient number of TID/flow resources to
 * map the segment could not be allocated. In this case the function should be
 * called again with previous arguments to retry the TID allocation. There are
 * no other error returns. The function returns 0 on success.
 */
int hfi1_kern_exp_rcv_setup(struct tid_rdma_request *req,
			    struct rvt_sge_state *ss, bool *last)
	__must_hold(&req->qp->s_lock)
{
	struct tid_rdma_flow *flow = &req->flows[req->setup_head];
	struct hfi1_ctxtdata *rcd = req->rcd;
	struct hfi1_qp_priv *qpriv = req->qp->priv;
	unsigned long flags;
	struct rvt_qp *fqp;
	u16 clear_tail = READ_ONCE(req->clear_tail);

	lockdep_assert_held(&req->qp->s_lock);
	/*
	 * We return error if either (a) we don't have space in the flow
	 * circular buffer, or (b) we already have max entries in the buffer.
	 * Max entries depend on the type of request we are processing and the
	 * negotiated TID RDMA parameters.
	 */
	if (!CIRC_SPACE(req->setup_head, clear_tail, req->n_max_flows) ||
	    CIRC_CNT(req->setup_head, clear_tail, req->n_max_flows) >=
	    req->n_flows)
		return -EINVAL;

	flow->req = req;
	/*
	 * Get pages, identify contiguous physical memory chunks for the segment
	 * If we can not determine a DMA address mapping we will treat it just
	 * like if we ran out of space above.
	 */
	if (kern_get_phys_blocks(flow, ss, last)) {
		hfi1_wait_kmem(flow->req->qp);
		return -ENOMEM;
	}

	spin_lock_irqsave(&rcd->exp_lock, flags);
	if (kernel_tid_waiters(rcd, &rcd->rarr_queue, flow->req->qp))
		goto queue;

	/*
	 * At this point we know the number of pagesets and hence the number of
	 * TID's to map the segment. Allocate the TID's from the TID groups. If
	 * we cannot allocate the required number we exit and try again later
	 */
	if (kern_alloc_tids(flow))
		goto queue;
	/*
	 * Finally program the TID entries with the pagesets, compute the
	 * tidarray and enable the HW flow
	 */
	kern_program_rcvarray(flow);

	/*
	 * Setup the flow state with relevant information.
	 * This information is used for tracking the sequence of data packets
	 * for the segment.
	 * The flow is setup here as this is the most accurate time and place
	 * to do so. Doing at a later time runs the risk of the flow data in
	 * qpriv getting out of sync.
	 */
	memset(&flow->flow_state, 0x0, sizeof(flow->flow_state));
	flow->idx = qpriv->flow_state.index;
	flow->flow_state.generation = qpriv->flow_state.generation;
	flow->flow_state.spsn = qpriv->flow_state.psn;
	flow->flow_state.lpsn = flow->flow_state.spsn + flow->npkts - 1;
	flow->flow_state.r_next_psn =
		full_flow_psn(flow, flow->flow_state.spsn);
	qpriv->flow_state.psn += flow->npkts;

	dequeue_tid_waiter(rcd, &rcd->rarr_queue, flow->req->qp);
	/* get head before dropping lock */
	fqp = first_qp(rcd, &rcd->rarr_queue);
	spin_unlock_irqrestore(&rcd->exp_lock, flags);
	tid_rdma_schedule_tid_wakeup(fqp);

	/* Ensure ordering for circular buffer head stores */
	smp_store_release(&req->setup_head,
			  (req->setup_head + 1) & (req->n_max_flows - 1));
	return 0;
queue:
	queue_qp_for_tid_wait(rcd, &rcd->rarr_queue, flow->req->qp);
	spin_unlock_irqrestore(&rcd->exp_lock, flags);
	return -EAGAIN;
}

/*
 * This function is called after one segment has been successfully sent to
 * release the flow and TID HW/SW resources for that segment. The segments for a
 * TID RDMA request are setup and cleared in FIFO order which is managed using a
 * circular buffer.
 */
int hfi1_kern_exp_rcv_clear(struct tid_rdma_request *req)
	__must_hold(&req->qp->s_lock)
{
	struct tid_rdma_flow *flow = &req->flows[req->clear_tail];
	struct hfi1_ctxtdata *rcd = req->rcd;
	unsigned long flags;
	int i;
	struct rvt_qp *fqp;

	lockdep_assert_held(&req->qp->s_lock);
	/* Exit if we have nothing in the flow circular buffer */
	if (!CIRC_CNT(smp_load_acquire(&req->setup_head), req->clear_tail,
		      req->n_max_flows))
		return -EINVAL;

	spin_lock_irqsave(&rcd->exp_lock, flags);

	for (i = 0; i < flow->tnode_cnt; i++)
		kern_unprogram_rcv_group(flow, i);
	/* get head before dropping lock */
	fqp = first_qp(rcd, &rcd->rarr_queue);
	spin_unlock_irqrestore(&rcd->exp_lock, flags);

	dma_unmap_flow(flow);

	hfi1_tid_rdma_reset_flow(flow);
	/* Ensure ordering for circular buffer tail stores */
	smp_store_release(&req->clear_tail,
			  (req->clear_tail + 1) & (req->n_max_flows - 1));

	if (fqp == req->qp) {
		__trigger_tid_waiter(fqp);
		rvt_put_qp(fqp);
	} else {
		tid_rdma_schedule_tid_wakeup(fqp);
	}

	return 0;
}

/*
 * This function is called to release all the tid entries for
 * a request.
 */
void hfi1_kern_exp_rcv_clear_all(struct tid_rdma_request *req)
	__must_hold(&req->qp->s_lock)
{
	while (CIRC_CNT(smp_load_acquire(&req->setup_head), req->clear_tail,
			req->n_max_flows)) {
		if (hfi1_kern_exp_rcv_clear(req))
			break;
	}
}

static void hfi1_kern_exp_rcv_dealloc(struct tid_rdma_flow *flow)
{

	kfree(flow->pages);
	kfree(flow->pagesets);
	kfree(flow->tid_entry);
	kfree(flow->tnode);
	flow->pages = NULL;
	flow->pagesets = NULL;
	flow->tid_entry = NULL;
	flow->tnode = NULL;
}

static int hfi1_kern_exp_rcv_alloc(struct tid_rdma_flow *flow, gfp_t gfp)
{
	u32 npages;

	npages = hfi1_tid_rdma_seg_max_size >> PAGE_SHIFT;
	if (npages & 1)
		npages++;
	/*
	 * Worst case allocations: in the worst case there are no contigous
	 * physical chunks so there are N (= npages) pagesets and TID entries,
	 * also in the worst case each TID comes from a separate TID group so
	 * there are N TID groups and therefore N TID nodes
	 */
	flow->pages = kcalloc(npages, sizeof(*flow->pages), gfp);
	flow->pagesets = kcalloc(npages, sizeof(*flow->pagesets), gfp);
	flow->tnode = kcalloc(npages, sizeof(*flow->tnode), gfp);
	flow->tid_entry = kcalloc(npages, sizeof(*flow->tid_entry), gfp);
	if (!flow->pages || !flow->pagesets || !flow->tid_entry ||!flow->tnode)
		goto nomem;

	return 0;
nomem:
	hfi1_kern_exp_rcv_dealloc(flow);
	return -ENOMEM;
}

/*
 * This is called at context initialization time to initialize the TID
 * groups and data structures
 */
int hfi1_kern_exp_rcv_init(struct hfi1_ctxtdata *rcd, int reinit)
{
	if (reinit)
		return 0;

	rcd->jkey = TID_RDMA_JKEY;
	hfi1_set_ctxt_jkey(rcd->dd, rcd->ctxt, rcd->jkey);
	return hfi1_alloc_ctxt_rcv_groups(rcd);
}

/* Called at context teardown to free TID group resources */
void hfi1_kern_exp_rcv_free(struct hfi1_ctxtdata *rcd)
{
	hfi1_free_ctxt_rcv_groups(rcd);
}

/* Called at QP destroy time to free TID RDMA resources */
void hfi1_kern_exp_rcv_free_flows(struct tid_rdma_request *req)
{
	int i;

	for (i = 0; req->flows && i < req->n_max_flows; i++)
		hfi1_kern_exp_rcv_dealloc(&req->flows[i]);

	kfree(req->flows);
	req->flows = NULL;
	req->n_max_flows = 0;
	req->n_flows = 0;
}

/*
 * This is called at QP create time to allocate resources for TID RDMA
 * segments/flows. This is done to keep all required memory pre-allocated and
 * avoid memory allocation in the data path.
 */
int hfi1_kern_exp_rcv_alloc_flows(struct tid_rdma_request *req, gfp_t gfp)
{
	struct tid_rdma_flow *flows;
	int i, ret;
	u16 nflows;

	/* Size of the flow circular buffer is the next higher power of 2 */
	nflows = max_t(u16, TID_RDMA_MAX_READ_SEGS_PER_REQ,
		       TID_RDMA_MAX_WRITE_SEGS_PER_REQ);
	req->n_max_flows = roundup_pow_of_two(nflows + 1);
	flows = kcalloc(req->n_max_flows, sizeof(*flows), gfp);
	if (!flows) {
		ret = -ENOMEM;
		goto err;
	}
	req->flows = flows;

	for (i = 0; i < req->n_max_flows; i++) {
		ret = hfi1_kern_exp_rcv_alloc(&req->flows[i], gfp);
		if (ret)
			goto err;
	}
	return 0;
err:
	hfi1_kern_exp_rcv_free_flows(req);
	return ret;
}

/*
 * Validate and accept the TID RDMA READ request parameters.
 * Return 0 if the request is accepted successfully;
 * Return 1 otherwise.
 */
static int tid_rdma_rcv_read_request(struct rvt_qp *qp,
				     struct rvt_ack_entry *e,
				     struct hfi1_packet *packet,
				     struct ib_other_headers *ohdr,
				     u32 bth0, u32 psn, u64 vaddr, u32 len)
{
	int ret = 0;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	u32 flow_psn, i, tidlen = 0, pktlen, tlen;

	req = ack_to_tid_req(e);

	/* Validate the payload first */
	flow = &req->flows[req->setup_head];

	/* payload length = packet length - (header length + ICRC length) */
	pktlen = packet->tlen - (packet->hlen + 4);
	memcpy(flow->tid_entry, packet->ebuf, pktlen);
	flow->tidcnt = pktlen / sizeof(*flow->tid_entry);

	/*
	 * Walk the TID_ENTRY list to make sure we have enough space for a
	 * complete segment. Also calculate the number of required packets.
	 */
	flow->npkts = rvt_div_round_up_mtu(qp, len);
	for (i = 0; i < flow->tidcnt; i++) {
		tlen = EXP_TID_GET(flow->tid_entry[i], LEN);
		if (!tlen) {
			ret = 1;
			goto done;
		}
		/*
		 * For tid pair (tidctr == 3), the buffer size of the pair
		 * should be the sum of the buffer size described by each
		 * tid entry. However, only the first entry needs to be
		 * specified in the request (see WFR HAS Section 8.5.7.1).
		 */
		tidlen += tlen;
	}
	if (tidlen * PAGE_SIZE < len) {
		ret = 1;
		goto done;
	}

	/* Empty the flow array */
	req->clear_tail = req->setup_head;
	flow->req = req;
	flow->pkt = 0;
	flow->tid_idx = 0;
	flow->tid_offset = 0;
	flow->sent = 0;
	flow->tid_qpn = be32_to_cpu(ohdr->u.tid_rdma.r_req.tid_flow_qp);
	flow->idx = (flow->tid_qpn >> TID_RDMA_DESTQP_FLOW_SHIFT) &
		TID_RDMA_DESTQP_FLOW_MASK;
	flow_psn = mask_psn(be32_to_cpu(ohdr->u.tid_rdma.r_req.tid_flow_psn));
	flow->flow_state.generation = flow_psn >> HFI1_KDETH_BTH_SEQ_SHIFT;
	flow->flow_state.spsn = flow_psn & HFI1_KDETH_BTH_SEQ_MASK;
	flow->length = len;

	flow->flow_state.lpsn = flow->flow_state.spsn +
		flow->npkts - 1;
	flow->flow_state.ib_spsn = psn;
	flow->flow_state.ib_lpsn = flow->flow_state.ib_spsn + flow->npkts - 1;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] READ Req: flow_idx: %u, idx: %u, gen: %u, flow (s/l): 0x%x/0x%x, ib (s/l): 0x%x/0x%x, len: %u, flow len: %u",
		  qp->ibqp.qp_num, req->setup_head, flow->idx,
		  flow->flow_state.generation,
		  full_flow_psn(flow, flow->flow_state.spsn),
		  full_flow_psn(flow, flow->flow_state.lpsn),
		  flow->flow_state.ib_spsn, flow->flow_state.ib_lpsn,
		  tidlen * PAGE_SIZE, flow->length);
#endif

	 /* Set the initial flow index to the current flow. */
	smp_store_release(&req->flow_idx, req->setup_head);

	/* advance circular buffer head */
	smp_store_release(&req->setup_head,
			  (req->setup_head + 1) & (req->n_max_flows - 1));
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] flow_idx: %u, setup_head: %u, clear_tail: %u, psn: 0x%x, s_cur: %u",
		  qp->ibqp.qp_num, req->flow_idx, req->setup_head,
		  req->clear_tail, qp->s_cur);
#endif

	/*
	 * Compute last PSN for request.
	 */
	e->opcode = (bth0 >> 24) & 0xff;
	e->psn = psn;
	e->lpsn = psn + flow->npkts - 1;
	e->sent = 0;

	req->n_flows = qpriv->tid_rdma.local.max_read;
	req->state = TID_REQUEST_ACTIVE;
	req->cur_seg = 0;
	req->comp_seg = 0;
	req->ack_seg = 0;
	req->isge = 0;
	req->vaddr = vaddr;
	req->seg_len = qpriv->tid_rdma.local.max_len;
	req->total_len = len;
	req->total_segs = 1;
	req->r_flow_psn = e->psn;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] Accepting TID RDMA READ request PSN 0x%x, LPSN 0x%x, r_psn 0x%x, r_flow_psn 0x%x, s_tail_ack %u, r_head_ack %u",
		  qp->ibqp.qp_num, e->psn, e->lpsn, qp->r_psn, req->r_flow_psn,
		  qp->s_tail_ack_queue, qp->r_head_ack_queue);
#endif
done:
	return ret;
}

static int tid_rdma_rcv_error(struct hfi1_packet *packet,
			      struct ib_other_headers *ohdr,
			      struct rvt_qp *qp, u32 psn, int diff)
{
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_ctxtdata *rcd = ((struct hfi1_qp_priv *)qp->priv)->rcd;
	struct hfi1_ibdev *dev = to_idev(qp->ibqp.device);
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct rvt_ack_entry *e;
	struct tid_rdma_request *req;
	unsigned long flags;
	u8 prev;
	bool old_req;

	trace_hfi1_rcv_error(qp, psn);
	if (diff > 0) {
		/* sequence error */
		if (!qp->r_nak_state) {
			ibp->rvp.n_rc_seqnak++;
			qp->r_nak_state = IB_NAK_PSN_ERROR;
			qp->r_ack_psn = qp->r_psn;
			rc_defered_ack(rcd, qp);
		}
		goto done;
	}

	ibp->rvp.n_rc_dupreq++;

	spin_lock_irqsave(&qp->s_lock, flags);
	e = find_prev_entry(qp, psn, &prev, NULL, &old_req);
	if (!e || (e->opcode != TID_OP(WRITE_REQ) &&
		   e->opcode != TID_OP(READ_REQ)))
		goto unlock;

	req = ack_to_tid_req(e);
	req->r_flow_psn = psn;
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] e->psn 0x%x psn 0x%x prev %u s_tail_ack_queue %u s_flags 0x%x io flags 0x%x req->state 0x%x old_req %d",
		  qp->ibqp.qp_num, e->psn, psn, prev, qp->s_tail_ack_queue,
		  qp->s_flags, qpriv->s_iowait.flags, req->state, old_req);
#endif
	if (e->opcode == TID_OP(WRITE_REQ)) {
		struct flow_state *fstate;
		u8 i;

		if (req->state == TID_REQUEST_RESEND)
			req->state = TID_REQUEST_RESEND_ACTIVE;

		/*
		 * True if the request is already scheduled (between
		 * qp->s_tail_ack_queue and qp->r_head_ack_queue).
		 * Also, don't change requests, which are at the SYNC
		 * point and haven't generated any responses yet.
		 * There is nothing to retransmit for them yet.
		 */
		if (old_req || req->state == TID_REQUEST_INIT ||
		    (req->state == TID_REQUEST_SYNC && !req->cur_seg))
			goto unlock;

		fstate = &req->flows[req->clear_tail].flow_state;
		req->flow_idx =
			CIRC_ADD(req->clear_tail,
				 delta_psn(psn, fstate->resp_ib_psn),
				 req->n_max_flows);
		/*
		 * When flow_idx == setup_head, we've gotten a duplicate request
		 * for a segment, which has not been allocated yet. In that
		 * case, we just schedule the QP without changing any state (the
		 * request or or the s_ack_state).
		 */
		if (req->flow_idx == req->setup_head)
			goto schedule;
		req->cur_seg = delta_psn(psn, e->psn);
		qpriv->pending_tid_w_segs -=
			CIRC_CNT(req->setup_head,
				 req->clear_tail,
				 req->n_max_flows);
		req->state = TID_REQUEST_RESEND_ACTIVE;

		for (i = prev + 1; ; i++) {
			/*
			 * Look at everything up to and including
			 * s_tail_ack_queue
			 */
			if (i == qp->s_tail_ack_queue + 1)
				break;
			if (i > rvt_size_atomic(&dev->rdi))
				i = 0;
			e = &qp->s_ack_queue[i];
			req = ack_to_tid_req(e);
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA,
				  "[QP%u] entry %u e->psn 0x%x cur_seg %u comp_seg %u",
				  qp->ibqp.qp_num, i, e->psn, req->cur_seg,
				  req->comp_seg);
#endif
			if (e->opcode != TID_OP(WRITE_REQ) ||
			    req->cur_seg == req->comp_seg)
				continue;
			if (req->state == TID_REQUEST_INIT)
				break;
			qpriv->pending_tid_w_segs -=
				CIRC_CNT(req->setup_head,
					 req->clear_tail,
					 req->n_max_flows);
			req->flow_idx = req->clear_tail;
			req->state = TID_REQUEST_RESEND;
			req->cur_seg = req->comp_seg;
		}
		qpriv->s_flags &= ~HFI1_R_TID_WAIT_INTERLCK;
	} else {
		struct ib_reth *reth;
		u32 offset;
		u32 len;
		u32 rkey;
		u64 vaddr;
		int ok;
		u32 bth0;


		reth = &ohdr->u.tid_rdma.r_req.reth;
		/*
		 * The requester always restarts from the start of the original
		 * request.
		 */
		offset = delta_psn(psn, e->psn) * qp->pmtu;
		len = be32_to_cpu(reth->length);
		if (psn != e->psn || len != req->total_len)
			goto unlock;

		if (e->rdma_sge.mr) {
			rvt_put_mr(e->rdma_sge.mr);
			e->rdma_sge.mr = NULL;
		}

		rkey = be32_to_cpu(reth->rkey);
		vaddr = get_ib_reth_vaddr(reth);

		qp->r_len = len;
		ok = rvt_rkey_ok(qp, &e->rdma_sge, len, vaddr, rkey,
				IB_ACCESS_REMOTE_READ);
		if (unlikely(!ok))
			goto unlock;

		/*
		 * If all the response packets for the current request have
		 * been sent out and this request is complete (old_request
		 * == false) and the TID flow may be unusable (the
		 * req->clear_tail is advanced). However, when an earlier
		 * request is received, this request will not be complete any
		 * more (qp->s_tail_ack_queue is moved back, see below).
		 * Consequently, we need to update the TID flow info everytime
		 * a duplicate request is received.
		 */
		bth0 = be32_to_cpu(ohdr->bth[0]);
		if (tid_rdma_rcv_read_request(qp, e, packet, ohdr, bth0, psn,
					      vaddr, len))
			goto unlock;

		/*
		 * True if the request is already scheduled (between
		 * qp->s_tail_ack_queue and qp->r_head_ack_queue);
		 */
		if (old_req)
			goto unlock;

	}
	/* Re-process old requests.*/
	if (qp->s_acked_ack_queue == qp->s_tail_ack_queue)
		qp->s_acked_ack_queue = prev;
	qp->s_tail_ack_queue = prev;
	/*
	 * Since the qp->s_tail_ack_queue is modified, the
	 * qp->s_ack_state must be changed to re-initialize
	 * qp->s_ack_rdma_sge; Otherwise, we will end up in
	 * wrong memory region.
	 */
	qp->s_ack_state = OP(ACKNOWLEDGE);
schedule:
	qp->r_state = e->opcode;
	qp->r_nak_state = 0;
	qp->s_flags |= RVT_S_RESP_PENDING;
	hfi1_schedule_send(qp);
unlock:
	spin_unlock_irqrestore(&qp->s_lock, flags);
done:
	return 1;
}

static u32 position_in_queue(struct hfi1_qp_priv *qpriv,
			     struct tid_queue *queue)
{
	return qpriv->tid_enqueue - READ_ONCE(queue->dequeue);
}

/**
 * Central place for resource allocation at TID write responder, is called from
 * write_req and write_data interrupt handlers as well as the send thread when a
 * queued QP is scheduled for resource allocation.
 *
 * Iterates over (a) segments of a request and then (b) queued requests
 * themselves to allocate resources for upto local->max_write segments across
 * multiple requests. Stop allocating when we hit a sync point, resume
 * allocating after data packets at sync point have been received.
 *
 * Resource allocation and sending of responses is decoupled. The
 * request/segment which are being allocated and sent are as follows.
 * Resources are allocated for:
 *     [request: qpriv->r_tid_alloc, segment: req->alloc_seg]
 * The send thread sends:
 *     [request: qp->s_tail_ack_queue, segment:req->cur_seg]
 */
void hfi1_tid_write_alloc_resources(struct rvt_qp *qp, bool intr_ctx)
{
	struct tid_rdma_request *req;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct hfi1_ctxtdata *rcd = qpriv->rcd;
	struct tid_rdma_params *local = &qpriv->tid_rdma.local;
	struct rvt_ack_entry *e;
	u32 npkts, to_seg;
	bool last;
	int ret = 0;

	lockdep_assert_held(&qp->s_lock);

	while (1) {
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] r_head_ack %u s_tail_ack %u s_head %u s_tail %u r_head %u r_tail %u r_ack %u r_alloc %u",
			  qp->ibqp.qp_num,
			  qp->r_head_ack_queue, qp->s_tail_ack_queue,
			  qpriv->s_tid_head, qpriv->s_tid_tail,
			  qpriv->r_tid_head, qpriv->r_tid_tail,
			  qpriv->r_tid_ack, qpriv->r_tid_alloc);
#endif

		/*
		 * Don't allocate more segments if a RNR NAK has already been
		 * scheduled to avoid messing up qp->r_psn: the RNR NAK will
		 * be sent only when all allocated segments have been sent.
		 * However, if more segments are allocated before that, TID RDMA
		 * WRITE RESP packets will be sent out for these new segments
		 * before the RNR NAK packet. When the requester receives the
		 * RNR NAK packet, it will restart with qp->s_last_psn + 1,
		 * which does not match qp->r_psn and will be dropped.
		 * Consequently, the requester will exhaust its retries and
		 * put the qp into error state.
		 */
		if (qpriv->rnr_nak_state == TID_RNR_NAK_SEND)
			break;

		/* No requests left to process */
		if (qpriv->r_tid_alloc == qpriv->r_tid_head) {
			/* If all data has been received, clear the flow */
			if (qpriv->flow_state.index < RXE_NUM_TID_FLOWS &&
			    !qpriv->alloc_w_segs)
				hfi1_kern_clear_hw_flow(rcd, qp);
			break;
		}

		e = &qp->s_ack_queue[qpriv->r_tid_alloc];
		if (e->opcode != TID_OP(WRITE_REQ))
			goto next_req;
		req = ack_to_tid_req(e);

#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] tail %u head %u num %u idx %u cur %u ack %u comp %u alloc %u total %u alloc_w %u sync_pt %u",
			  qp->ibqp.qp_num, req->clear_tail, req->setup_head,
			  req->n_flows, req->flow_idx, req->cur_seg,
			  req->ack_seg, req->comp_seg, req->alloc_seg,
			  req->total_segs, qpriv->alloc_w_segs, qpriv->sync_pt);
#endif
		/* Finished allocating for all segments of this request */
		if (req->alloc_seg >= req->total_segs)
			goto next_req;

		/* Can allocate only a maximum of local->max_write for a QP */
		if (qpriv->alloc_w_segs >= local->max_write)
			break;

		/* Don't allocate at a sync point with data packets pending */
		if (qpriv->sync_pt && qpriv->alloc_w_segs)
			break;

		/* All data received at the sync point, continue */
		if (qpriv->sync_pt && !qpriv->alloc_w_segs) {
			hfi1_kern_clear_hw_flow(rcd, qp);
			qpriv->sync_pt = false;
			if (qpriv->s_flags & HFI1_R_TID_SW_PSN)
				qpriv->s_flags &= ~HFI1_R_TID_SW_PSN;
		}

		/* Allocate flow if we don't have one */
		if (qpriv->flow_state.index >= RXE_NUM_TID_FLOWS) {
			ret = hfi1_kern_setup_hw_flow(qpriv->rcd, qp);
			if (ret) {
				to_seg = tid_rdma_flow_wt *
					position_in_queue(qpriv,
							  &rcd->flow_queue);
				break;
			}
		}

		npkts = rvt_div_round_up_mtu(qp, req->seg_len);

		/* We are at a sync point if we run out of KDETH PSN space */
		if (qpriv->flow_state.psn + npkts > MAX_TID_FLOW_PSN) {
			qpriv->sync_pt = true;
			break;
		}

		/*
		 * If overtaking req->acked_tail, send an RNR NAK. Because the
		 * QP is not queued in this case, and the issue can only be
		 * caused due a delay in scheduling the second leg which we
		 * cannot estimate, we use a rather arbitrary RNR timeout of
		 * (req->n_max_flows / 2) segments
		 */
		if (!CIRC_SPACE(req->setup_head, req->acked_tail,
				req->n_max_flows)) {
			ret = -EAGAIN;
			to_seg = req->n_max_flows >> 1;
			/*
			 * It has been seen once that under this condition,
			 * the second leg failed to run, causing the qp to
			 * hang. Schedule the second leg as a safeguard.
			 */
			qpriv->s_flags |= RVT_S_RESP_PENDING;
			hfi1_schedule_tid_send(qp);
			break;
		}

		/* Try to allocate rcv array / TID entries */
		ret = hfi1_kern_exp_rcv_setup(req, &req->ss, &last);
		if (ret == -EAGAIN)
			to_seg = position_in_queue(qpriv, &rcd->rarr_queue);
		if (ret)
			break;

		qpriv->alloc_w_segs++;
		req->alloc_seg++;
		continue;
next_req:
		/* Begin processing the next request */
		if (++qpriv->r_tid_alloc >
		    rvt_size_atomic(ib_to_rvt(qp->ibqp.device)))
			qpriv->r_tid_alloc = 0;
	}

	/*
	 * Schedule an RNR NAK to be sent if (a) flow or rcv array allocation
	 * has failed (b) we are called from the rcv handler interrupt context
	 * (c) an RNR NAK has not already been scheduled
	 */
	if (ret == -EAGAIN && intr_ctx && !qp->r_nak_state)
		goto send_rnr_nak;

	return;

send_rnr_nak:
	/*
	 * NO_RNR_NAK turns off sending RNR NAK's. RNR NAK's will not be sent
	 * but everything else will work as expected. The remote QP might time
	 * out and resend the request if it doesn't receive a reponse. The
	 * resent request will be discarded as duplicate. This works as long as
	 * the remote QP does not run out of retries
	 */
#ifdef NO_RNR_NAK
	return;
#endif
	lockdep_assert_held(&qp->r_lock);

	/* Set r_nak_state to prevent unrelated events from generating NAK's */
	qp->r_nak_state = hfi1_compute_tid_rnr_timeout(qp, to_seg) | IB_RNR_NAK;

	/* Pull back r_psn to the segment being RNR NAK'd */
	qp->r_psn = e->psn + req->alloc_seg;
	qp->r_ack_psn = qp->r_psn;
	/*
	 * Pull back r_head_ack_queue to the ack entry following the request
	 * being RNR NAK'd. This allows resources to be allocated to the request
	 * if the queued QP is scheduled.
	 */
	qp->r_head_ack_queue = qpriv->r_tid_alloc + 1;
	if (qp->r_head_ack_queue > rvt_size_atomic(ib_to_rvt(qp->ibqp.device)))
		qp->r_head_ack_queue = 0;
	qpriv->r_tid_head = qp->r_head_ack_queue;
	/*
	 * These send side fields are used in make_rc_ack(). They are set in
	 * hfi1_send_rc_ack() but must be set here before dropping qp->s_lock
	 * for consistency
	 */
	qp->s_nak_state = qp->r_nak_state;
	qp->s_ack_psn = qp->r_ack_psn;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] Send RNR NAK r_psn 0x%x r_head_ack_queue %u",
		  qp->ibqp.qp_num, qp->r_psn, qp->r_head_ack_queue);
#endif
	/*
	 * qpriv->rnr_nak_state is used to determine when the scheduled RNR NAK
	 * has actually been sent. qp->s_flags RVT_S_ACK_PENDING bit cannot be
	 * used for this because qp->s_lock is dropped before calling
	 * hfi1_send_rc_ack() leading to inconsistency between the receive
	 * interrupt handlers and the send thread in make_rc_ack()
	 */
	qpriv->rnr_nak_state = TID_RNR_NAK_SEND;

	/*
	 * Schedule RNR NAK to be sent. RNR NAK's are scheduled from the receive
	 * interrupt handlers but will be sent from the send engine behind any
	 * previous responses that may have been scheduled
	 */
	rc_defered_ack(rcd, qp);
}

void hfi1_rc_rcv_tid_rdma_write_req(struct hfi1_packet *packet)
{
	/* HANDLER FOR TID RDMA WRITE REQUEST packet (Responder side)*/

	/*
	 * 1. Verify TID RDMA WRITE REQ as per IB_OPCODE_RC_RDMA_WRITE_FIRST
	 *    (see hfi1_rc_rcv())
	 *     - Don't allow 0-length requests.
	 * 2. Put TID RDMA WRITE REQ into the response queueu (s_ack_queue)
	 *     - Setup struct tid_rdma_req with request info
	 *     - Prepare struct tid_rdma_flow array?
	 * 3. Set the qp->s_ack_state as state diagram in design doc.
	 * 4. Set RVT_S_RESP_PENDING in s_flags.
	 * 5. Kick the send engine (hfi1_schedule_send())
	 */
	struct hfi1_ctxtdata *rcd = packet->rcd;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	struct hfi1_ibport_priv *ibp_priv = ibp->rvp.priv;
#endif
	struct ib_other_headers *ohdr = packet->ohdr;
	struct rvt_ack_entry *e;
	unsigned long flags;
	struct ib_reth *reth;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_request *req;
	u32 bth0, psn, len, rkey, num_segs;
	bool is_fecn;
	u8 next;
	u64 vaddr;
	int diff;

	bth0 = be32_to_cpu(ohdr->bth[0]);
	if (hfi1_ruc_check_hdr(ibp, packet))
		return;

#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	this_cpu_inc(*ibp_priv->trdma_cnts.rx.w_req);
#endif
	is_fecn = process_ecn(qp, packet, false);
	psn = mask_psn(be32_to_cpu(ohdr->bth[2]));

	if (qp->state == IB_QPS_RTR && !(qp->r_flags & RVT_R_COMM_EST))
		rvt_comm_est(qp);

	if (unlikely(!(qp->qp_access_flags & IB_ACCESS_REMOTE_WRITE)))
		goto nack_inv;

	reth = &ohdr->u.tid_rdma.w_req.reth;
	vaddr = be64_to_cpu(reth->vaddr);
	len = be32_to_cpu(reth->length);

	num_segs = DIV_ROUND_UP(len, qpriv->tid_rdma.local.max_len);
	diff = delta_psn(psn, qp->r_psn);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] psn 0x%x, qp->r_psn 0x%x, diff %d",
		  qp->ibqp.qp_num, psn, qp->r_psn, diff);
#endif
	if (unlikely(diff)) {
		if (tid_rdma_rcv_error(packet, ohdr, qp, psn, diff))
			return;
		goto send_ack;
	}

	/*
	 * The resent request which was previously RNR NAK'd is inserted at the
	 * location of the original request, which is one entry behind
	 * r_head_ack_queue
	 */
	if (qpriv->rnr_nak_state)
		qp->r_head_ack_queue = qp->r_head_ack_queue ?
			qp->r_head_ack_queue - 1 :
			rvt_size_atomic(ib_to_rvt(qp->ibqp.device));

	/* We've verified the request, insert it into the ack queue. */
	next = qp->r_head_ack_queue + 1;
	if (next > rvt_size_atomic(ib_to_rvt(qp->ibqp.device)))
		next = 0;
	spin_lock_irqsave(&qp->s_lock, flags);
	if (unlikely(next == qp->s_acked_ack_queue)) {
		if (!qp->s_ack_queue[next].sent)
			goto nack_inv_unlock;
		update_ack_queue(qp, next);
	}
	e = &qp->s_ack_queue[qp->r_head_ack_queue];
	req = ack_to_tid_req(e);

	/* Bring previously RNR NAK'd request back to life */
	if (qpriv->rnr_nak_state) {
		qp->r_nak_state = 0;
		qp->s_nak_state = 0;
		qpriv->rnr_nak_state = TID_RNR_NAK_INIT;
		qp->r_psn = e->lpsn + 1;
		req->state = TID_REQUEST_INIT;
		goto update_head;
	}

	if (e->rdma_sge.mr) {
		rvt_put_mr(e->rdma_sge.mr);
		e->rdma_sge.mr = NULL;
	}

	/* The length needs to be in multiples of PAGE_SIZE */
	if (!len || len & ~PAGE_MASK)
		goto nack_inv_unlock;

	rkey = be32_to_cpu(reth->rkey);
	qp->r_len = len;

	if (e->opcode == TID_OP(WRITE_REQ) &&
	    (READ_ONCE(req->setup_head) != READ_ONCE(req->clear_tail) ||
	     req->clear_tail != READ_ONCE(req->acked_tail)))
		goto nack_inv_unlock;

	if (unlikely(!rvt_rkey_ok(qp, &e->rdma_sge, qp->r_len, vaddr,
				  rkey, IB_ACCESS_REMOTE_WRITE)))
		goto nack_acc;

	qp->r_psn += num_segs - 1;

	e->opcode = (bth0 >> 24) & 0xff;
	e->psn = psn;
	e->lpsn = qp->r_psn;
	e->sent = 0;

	req->n_flows = min_t(u16, num_segs, qpriv->tid_rdma.local.max_write);
	req->state = TID_REQUEST_INIT;
	req->cur_seg = 0;
	req->comp_seg = 0;
	req->ack_seg = 0;
	req->alloc_seg = 0;
	req->isge = 0;
	req->vaddr = vaddr;
	req->seg_len = qpriv->tid_rdma.local.max_len;
	req->total_len = len;
	req->total_segs = num_segs;
	req->r_flow_psn = e->psn;
	req->ss.sge = e->rdma_sge;
	req->ss.num_sge = 1;

	req->flow_idx = req->setup_head;
	req->clear_tail = req->setup_head;
	smp_store_release(&req->acked_tail, req->setup_head);

	qp->r_state = e->opcode;
	qp->r_nak_state = 0;
	/*
	 * We need to increment the MSN here instead of when we
	 * finish sending the result since a duplicate request would
	 * increment it more than once.
	 */
	qp->r_msn++;
	qp->r_psn++;
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] Accepting TID RDMA WRITE request PSN 0x%x, LPSN 0x%x, r_psn 0x%x, r_flow_psn 0x%x, s_tail_ack %u, r_head_ack %u, tid_tail %u, tid_head %u",
		  qp->ibqp.qp_num, e->psn, e->lpsn, qp->r_psn, req->r_flow_psn,
		  qp->s_tail_ack_queue, qp->r_head_ack_queue, qpriv->r_tid_tail,
		  qpriv->r_tid_head);
#endif
	if (qpriv->r_tid_tail == HFI1_QP_WQE_INVALID) {
		qpriv->r_tid_tail = qp->r_head_ack_queue;
	} else if (qpriv->r_tid_tail == qpriv->r_tid_head) {
		struct tid_rdma_request *ptr;

		e = &qp->s_ack_queue[qpriv->r_tid_tail];
		ptr = ack_to_tid_req(e);

		if (e->opcode != TID_OP(WRITE_REQ) ||
		    ptr->comp_seg == ptr->total_segs) {
			if (qpriv->r_tid_tail == qpriv->r_tid_ack)
				qpriv->r_tid_ack = qp->r_head_ack_queue;
			qpriv->r_tid_tail = qp->r_head_ack_queue;
		}
	}
update_head:
	qp->r_head_ack_queue = next;
	qpriv->r_tid_head = qp->r_head_ack_queue;

	hfi1_tid_write_alloc_resources(qp, true);

	/* Schedule the send tasklet. */
	qp->s_flags |= RVT_S_RESP_PENDING;
	hfi1_schedule_send(qp);

	spin_unlock_irqrestore(&qp->s_lock, flags);
	if (is_fecn)
		goto send_ack;
	return;

nack_inv_unlock:
	spin_unlock_irqrestore(&qp->s_lock, flags);
nack_inv:
	rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
	qp->r_nak_state = IB_NAK_INVALID_REQUEST;
	qp->r_ack_psn = qp->r_psn;
	/* Queue NAK for later */
	rc_defered_ack(rcd, qp);
	return;
nack_acc:
	spin_unlock_irqrestore(&qp->s_lock, flags);
	rvt_rc_error(qp, IB_WC_LOC_PROT_ERR);
	qp->r_nak_state = IB_NAK_REMOTE_ACCESS_ERROR;
	qp->r_ack_psn = qp->r_psn;
send_ack:
	hfi1_send_rc_ack(rcd, qp, is_fecn);
}

void hfi1_rc_rcv_tid_rdma_write_data(struct hfi1_packet *packet)
{
	struct rvt_qp *qp = packet->qp;
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_ctxtdata *rcd = priv->rcd;
	struct ib_other_headers *ohdr = packet->ohdr;
	struct rvt_ack_entry *e;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	struct hfi1_ibdev *dev = to_idev(qp->ibqp.device);
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_ibport_priv *ibp_priv = ibp->rvp.priv;
#endif
	unsigned long flags;
	u32 psn, next;
	u8 opcode;

	psn = mask_psn(be32_to_cpu(ohdr->bth[2]));
	opcode = (be32_to_cpu(ohdr->bth[0]) >> 24) & 0xff;
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	if (opcode == TID_OP(WRITE_DATA))
		this_cpu_inc(*ibp_priv->trdma_cnts.rx.w_data);
	else
		this_cpu_inc(*ibp_priv->trdma_cnts.rx.w_datalast);
#endif

	/*
	 * All error handling should be done by now. If we are here, the packet
	 * is either good or been accepted by the error handler.
	 */
	spin_lock_irqsave(&qp->s_lock, flags);
	e = &qp->s_ack_queue[priv->r_tid_tail];
	req = ack_to_tid_req(e);
	flow = &req->flows[req->clear_tail];
	if (cmp_psn(psn, full_flow_psn(flow, flow->flow_state.lpsn))) {
		if (cmp_psn(psn, flow->flow_state.r_next_psn))
			goto send_nak;
		flow->flow_state.r_next_psn++;
		goto exit;
	}
	hfi1_kern_exp_rcv_clear(req);
	priv->alloc_w_segs--;
	rcd->flows[flow->idx].psn = psn & HFI1_KDETH_BTH_SEQ_MASK;
	req->comp_seg++;

	/*
	 * Release the flow if one of the following conditions has been met:
	 *  - The request has reached a sync point AND all outstanding
	 *    segments have been completed, or
	 *  - The entire request is complete and there are no more requests
	 *    (of any kind) in the queue.
	 */
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] r_tid_tail %u state 0x%x comp_seg %u cur_seg %u total_seg %u s_tail_ack_queue %u r_head_ack_queue %u",
		  qp->ibqp.qp_num, priv->r_tid_tail, req->state, req->comp_seg,
		  req->cur_seg, req->total_segs, qp->s_tail_ack_queue,
		  qp->r_head_ack_queue);
#endif

	if (priv->r_tid_ack == HFI1_QP_WQE_INVALID)
		priv->r_tid_ack = priv->r_tid_tail;

	if (opcode == TID_OP(WRITE_DATA_LAST)) {
		for (next = priv->r_tid_tail + 1; ; next++) {
			if (next > rvt_size_atomic(&dev->rdi))
				next = 0;
			if (next == priv->r_tid_head)
				break;
			e = &qp->s_ack_queue[next];
			if (e->opcode == TID_OP(WRITE_REQ))
				break;
		}
		priv->r_tid_tail = next;
		if (++qp->s_acked_ack_queue > rvt_size_atomic(&dev->rdi))
			qp->s_acked_ack_queue = 0;
	}

	hfi1_tid_write_alloc_resources(qp, true);

	/*
	 * If we need to generate more responses, schedule the
	 * send engine.
	 */
	if ((req->cur_seg < req->total_segs) ||
	    (qp->s_tail_ack_queue != qp->r_head_ack_queue)) {
		qp->s_flags |= RVT_S_RESP_PENDING;
		hfi1_schedule_send(qp);
	}

	if (priv->s_flags & HFI1_R_TID_RSC_TIMER) {
		if (--priv->pending_tid_w_segs)
			hfi1_mod_tid_reap_timer(req->qp);
		else
			hfi1_stop_tid_reap_timer(req->qp);
	}

done:
	priv->s_flags |= RVT_S_RESP_PENDING;
	hfi1_schedule_tid_send(qp);
exit:
	spin_unlock_irqrestore(&qp->s_lock, flags);
	return;

send_nak:
	if (!priv->s_nak_state) {
		priv->s_nak_state = IB_NAK_PSN_ERROR;
		priv->s_nak_psn = flow->flow_state.r_next_psn;
		priv->s_flags |= RVT_S_ACK_PENDING | RVT_S_RESP_PENDING;
		if (priv->r_tid_ack == HFI1_QP_WQE_INVALID)
			priv->r_tid_ack = priv->r_tid_tail;
		hfi1_schedule_tid_send(qp);
	}
	goto done;
}

void hfi1_rc_rcv_tid_rdma_write_resp(struct hfi1_packet *packet)
{
	/* HANDLER FOR TID RDMA WRITE RESPONSE packet (Requestor side */

	/*
	 * 1. Find matching SWQE
	 * 2. Check that TIDENTRY array has enough space for a complete
	 *    segment. If not, put QP in error state.
	 * 3. Save response data in struct tid_rdma_req and struct tid_rdma_flow
	 * 4. Remove RVT_S_WAIT_TID_RESP from s_flags.
	 * 5. Set qp->s_state as per state diagram in design doc
	 * 6. Kick the send engine (hfi1_schedule_send())
	 */
	struct ib_other_headers *ohdr = packet->ohdr;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct hfi1_ctxtdata *rcd = packet->rcd;
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_ibport_priv *ibp_priv = ibp->rvp.priv;
#endif
	struct rvt_swqe *wqe;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	enum ib_wc_status status;
	u32 opcode, aeth, psn, flow_psn, i, tidlen = 0, pktlen;
	bool is_fecn;
	unsigned long flags;

#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	this_cpu_inc(*ibp_priv->trdma_cnts.rx.w_resp);
#endif
	is_fecn = process_ecn(qp, packet, false);
	psn = mask_psn(be32_to_cpu(ohdr->bth[2]));
	aeth = be32_to_cpu(ohdr->u.tid_rdma.w_rsp.aeth);
	opcode = (be32_to_cpu(ohdr->bth[0]) >> 24) & 0xff;

	spin_lock_irqsave(&qp->s_lock, flags);

	/* Ignore invalid responses */
	smp_read_barrier_depends(); /* see post_one_send */
	if (cmp_psn(psn, READ_ONCE(qp->s_next_psn)) >= 0)
		goto ack_done;

	/* Ignore duplicate responses. */
	/* XXX Check with TID RDMA spec if this is possible */
	if (unlikely(cmp_psn(psn, qp->s_last_psn) <= 0))
		goto ack_done;

	if (unlikely(qp->s_acked == qp->s_tail))
		goto ack_done;

	/*
	 * If we are waiting for a particular packet sequence number
	 * due to a request being resent, check for it. Otherwise,
	 * ensure that we haven't missed anything.
	 */
	if (qp->r_flags & RVT_R_RDMAR_SEQ) {
		if (cmp_psn(psn, qp->s_last_psn + 1) != 0)
			goto ack_done;
		qp->r_flags &= ~RVT_R_RDMAR_SEQ;
	}

	wqe = rvt_get_swqe_ptr(qp, qpriv->s_tid_cur);
	if (unlikely(wqe->wr.opcode != IB_WR_TID_RDMA_WRITE))
		goto ack_op_err;

	req = wqe_to_tid_req(wqe);
	if (!CIRC_SPACE(req->setup_head, READ_ONCE(req->clear_tail),
			req->n_max_flows))
		goto ack_op_err;

	/*
	 * The call to do_rc_ack() should be last in the chain of
	 * packet checks because it will end up updating the QP state.
	 * Therefore, anything that would prevent the packet from
	 * being accepted as a successful response should be prior
	 * to it.
	 */
	if (!do_rc_ack(qp, aeth, psn, opcode, 0, rcd))
		goto ack_done;

	trace_hfi1_ack(qp, psn);

	flow = &req->flows[req->setup_head];
	flow->req = req;
	flow->pkt = 0;
	flow->tid_idx = 0;
	flow->sent = 0;
	flow->tid_qpn = be32_to_cpu(ohdr->u.tid_rdma.w_rsp.tid_flow_qp);
	flow->idx = (flow->tid_qpn >> TID_RDMA_DESTQP_FLOW_SHIFT) &
		TID_RDMA_DESTQP_FLOW_MASK;
	flow_psn = mask_psn(be32_to_cpu(ohdr->u.tid_rdma.w_rsp.tid_flow_psn));
	flow->flow_state.generation = flow_psn >> HFI1_KDETH_BTH_SEQ_SHIFT;
	flow->flow_state.spsn = flow_psn & HFI1_KDETH_BTH_SEQ_MASK;

	flow->length = min_t(u32, req->seg_len,
			     (wqe->length - (req->comp_seg * req->seg_len)));

	flow->npkts = rvt_div_round_up_mtu(qp, flow->length);
	flow->flow_state.lpsn = flow->flow_state.spsn +
		flow->npkts - 1;
	/* payload length = packet length - (header length + ICRC length) */
	pktlen = packet->tlen - (packet->hlen + 4);
	memcpy(flow->tid_entry, packet->ebuf, pktlen);
	flow->tidcnt = pktlen / sizeof(*flow->tid_entry);

#if defined(TIDRDMA_DEBUG) && defined(TIDRDMA_EXTRA_DEBUG)
	dump_tid_array(flow->tid_entry, flow->tidcnt);
#endif

	req->comp_seg++;
	if (unlikely(qpriv->s_tid_tail == HFI1_QP_WQE_INVALID))
		qpriv->s_tid_tail = qpriv->s_tid_cur;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] s_cur %u s_tid_head %u s_tid_cur %u s_tid_tail %u",
		  qp->ibqp.qp_num, qp->s_cur, qpriv->s_tid_head,
		  qpriv->s_tid_cur, qpriv->s_tid_tail);
#endif
	/*
	 * Walk the TID_ENTRY list to make sure we have enough space for a
	 * complete segment.
	 */
	for (i = 0; i < flow->tidcnt; i++) {
		if (!EXP_TID_GET(flow->tid_entry[i], LEN)) {
			status = IB_WC_LOC_LEN_ERR;
			goto ack_err;
		}
		tidlen += EXP_TID_GET(flow->tid_entry[i], LEN);
	}
	if (tidlen * PAGE_SIZE < flow->length) {
		status = IB_WC_LOC_LEN_ERR;
		goto ack_err;
	}

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] WRITE Resp: psn 0x%x, wqe->psn 0x%x wqe->lpsn 0x%x comp_seg %u total_segs %u / 0x%x",
		  qp->ibqp.qp_num, psn, wqe->psn, wqe->lpsn, req->comp_seg,
		  req->total_segs, mask_psn(wqe->psn - 1));
#endif
	/*
	 * If this is the first response for this request, set the initial
	 * flow index to the current flow.
	 */
	if (!cmp_psn(psn, wqe->psn)) {
		req->r_last_acked = mask_psn(wqe->psn - 1);
		/* Set acked flow index to head index */
		smp_store_release(&req->acked_tail, req->setup_head);
	}

	/* advance circular buffer head */
	smp_store_release(&req->setup_head,
			  CIRC_NEXT(req->setup_head, req->n_max_flows));
	req->state = TID_REQUEST_ACTIVE;

	/*
	 * If all responses for this TID RDMA WRITE request have been received
	 * advance the pointer to the next one.
	 * Since TID RDMA requests could be mixed in with regular IB requests,
	 * they might not appear sequentially in the queue. Therefore, the
	 * next request needs to be "found".
	 */
	if (qpriv->s_tid_cur != qpriv->s_tid_head &&
	    req->comp_seg == req->total_segs) {
		for (i = qpriv->s_tid_cur + 1; ; i++) {
			if (i == qp->s_size)
				i = 0;
			wqe = rvt_get_swqe_ptr(qp, i);
			if (i == qpriv->s_tid_head)
				break;
			if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
				break;
		}
		qpriv->s_tid_cur = i;
	}
	qp->s_flags &= ~RVT_S_WAIT_TID_RESP;

	hfi1_schedule_tid_send(qp);
	goto ack_done;

ack_op_err:
	status = IB_WC_LOC_QP_OP_ERR;
ack_err:
	/*
	 * XXX This is used when the response does not have enough room for
	 * XXX a complete segment. Is the 'if' check below still correct?
	 */
	if (qp->s_last == qp->s_acked) {
		hfi1_send_complete(qp, wqe, status);
		rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);
	}
ack_done:
	spin_unlock_irqrestore(&qp->s_lock, flags);
	if (is_fecn)
		hfi1_send_rc_ack(rcd, qp, is_fecn);
	return;
}

void hfi1_rc_rcv_tid_rdma_read_req(struct hfi1_packet *packet)
{
	/* HANDLER FOR TID RDMA READ REQUEST packet (Responder side)*/

	/*
	 * 1. Verify TID RDMA READ REQ as per IB_OPCODE_RC_RDMA_READ
	 *    (see hfi1_rc_rcv())
	 * 2. Put TID RDMA READ REQ into the response queueu (s_ack_queue)
	 *     - Setup struct tid_rdma_req with request info
	 *     - Initialize struct tid_rdma_flow info;
	 *     - Copy TID entries;
	 * 3. Set the qp->s_ack_state as state diagram in design doc.
	 * 4. Set RVT_S_RESP_PENDING in s_flags.
	 * 5. Kick the send engine (hfi1_schedule_send())
	 */
	struct hfi1_ctxtdata *rcd = packet->rcd;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	struct hfi1_ibport_priv *ibp_priv = ibp->rvp.priv;
#endif
	struct ib_other_headers *ohdr = packet->ohdr;
	struct rvt_ack_entry *e;
	unsigned long flags;
	struct ib_reth *reth;
	struct hfi1_qp_priv *qpriv = qp->priv;
	u32 bth0, psn, len, rkey;
	bool is_fecn;
	u8 next;
	u64 vaddr;
	int diff;
	u8 nack_state = IB_NAK_INVALID_REQUEST;

#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	this_cpu_inc(*ibp_priv->trdma_cnts.rx.r_req);
#endif
	bth0 = be32_to_cpu(ohdr->bth[0]);
	if (hfi1_ruc_check_hdr(ibp, packet))
		return;

	is_fecn = process_ecn(qp, packet, false);
	psn = mask_psn(be32_to_cpu(ohdr->bth[2]));

	if (qp->state == IB_QPS_RTR && !(qp->r_flags & RVT_R_COMM_EST))
		rvt_comm_est(qp);

	if (unlikely(!(qp->qp_access_flags & IB_ACCESS_REMOTE_READ)))
		goto nack_inv;

	reth = &ohdr->u.tid_rdma.r_req.reth;
	vaddr = be64_to_cpu(reth->vaddr);
	len = be32_to_cpu(reth->length);
	/* The length needs to be in multiples of PAGE_SIZE */
	if (!len || len & ~PAGE_MASK || len > qpriv->tid_rdma.local.max_len)
		goto nack_inv;

	diff = delta_psn(psn, qp->r_psn);
	if (unlikely(diff)) {
		if (tid_rdma_rcv_error(packet, ohdr, qp, psn, diff))
			return;
		goto send_ack;
	}

	/* We've verified the request, insert it into the ack queue. */
	next = qp->r_head_ack_queue + 1;
	if (next > rvt_size_atomic(ib_to_rvt(qp->ibqp.device)))
		next = 0;
	spin_lock_irqsave(&qp->s_lock, flags);
	if (unlikely(next == qp->s_tail_ack_queue)) {
		if (!qp->s_ack_queue[next].sent) {
			nack_state = IB_NAK_REMOTE_OPERATIONAL_ERROR;
			goto nack_inv_unlock;
		}
		update_ack_queue(qp, next);
	}
	e = &qp->s_ack_queue[qp->r_head_ack_queue];
	if (e->rdma_sge.mr) {
		rvt_put_mr(e->rdma_sge.mr);
		e->rdma_sge.mr = NULL;
	}

	rkey = be32_to_cpu(reth->rkey);
	qp->r_len = len;

	if (unlikely(!rvt_rkey_ok(qp, &e->rdma_sge, qp->r_len, vaddr,
				  rkey, IB_ACCESS_REMOTE_READ)))
		goto nack_acc;

	/* Accept the request parameters */
	if (tid_rdma_rcv_read_request(qp, e, packet, ohdr, bth0, psn, vaddr,
				      len))
		goto nack_inv_unlock;

	qp->r_state = e->opcode;
	qp->r_nak_state = 0;
	/*
	 * We need to increment the MSN here instead of when we
	 * finish sending the result since a duplicate request would
	 * increment it more than once.
	 */
	qp->r_msn++;
	qp->r_psn += e->lpsn - e->psn + 1;

	qp->r_head_ack_queue = next;

	/*
	 * For all requests other than TID WRITE which are added to the ack
	 * queue, qpriv->r_tid_alloc follows qp->r_head_ack_queue. It is ok to
	 * do this because of interlocks between these and TID WRITE
	 * requests. The same change has also been made in hfi1_rc_rcv().
	 */
	qpriv->r_tid_alloc = qp->r_head_ack_queue;

	/* Schedule the send tasklet. */
	qp->s_flags |= RVT_S_RESP_PENDING;
	hfi1_schedule_send(qp);

	spin_unlock_irqrestore(&qp->s_lock, flags);
	if (is_fecn)
		goto send_ack;
	return;

nack_inv_unlock:
	spin_unlock_irqrestore(&qp->s_lock, flags);
nack_inv:
	rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
	qp->r_nak_state = nack_state;
	qp->r_ack_psn = qp->r_psn;
	/* Queue NAK for later */
	rc_defered_ack(rcd, qp);
	return;
nack_acc:
	spin_unlock_irqrestore(&qp->s_lock, flags);
	rvt_rc_error(qp, IB_WC_LOC_PROT_ERR);
	qp->r_nak_state = IB_NAK_REMOTE_ACCESS_ERROR;
	qp->r_ack_psn = qp->r_psn;
send_ack:
	hfi1_send_rc_ack(rcd, qp, is_fecn);
}

static inline struct tid_rdma_request *
find_tid_request(struct rvt_qp *qp, u32 psn, enum ib_wr_opcode opcode)
	__must_hold(&qp->s_lock)
{
	struct rvt_swqe *wqe;
	struct tid_rdma_request *req = NULL;
	u32 i, end;

	end = qp->s_cur + 1;
	if (end == qp->s_size)
		end = 0;
	for (i = qp->s_acked; i != end;) {
		wqe = rvt_get_swqe_ptr(qp, i);
		if (cmp_psn(psn, wqe->psn) >= 0 &&
		    cmp_psn(psn, wqe->lpsn) <= 0) {
			if (wqe->wr.opcode == opcode)
				req = wqe_to_tid_req(wqe);
			break;
		}
		if (++i == qp->s_size)
			i = 0;
	}

	return req;
}

void hfi1_rc_rcv_tid_rdma_read_resp(struct hfi1_packet *packet)
{
	/* HANDLER FOR TID RDMA READ RESPONSE packet (Requestor side */

	/*
	 * 1. Find matching SWQE
	 * 2. Check that the entire segment has been read.
	 * 3. Remove RVT_S_WAIT_TID_RESP from s_flags.
	 * 4. Free the TID flow resources.
	 * 5. Kick the send engine (hfi1_schedule_send())
	 */
	struct ib_other_headers *ohdr = packet->ohdr;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_ctxtdata *rcd = packet->rcd;
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_ibport_priv *ibp_priv = ibp->rvp.priv;
#endif
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	u32 opcode, aeth;
	bool is_fecn;
	unsigned long flags;
	u32 kpsn, ipsn;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] Received TID RDMA READ resp packet",
		  qp->ibqp.qp_num);
#endif

#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	this_cpu_inc(*ibp_priv->trdma_cnts.rx.r_resp);
#endif
	is_fecn = process_ecn(qp, packet, false);

	kpsn = mask_psn(be32_to_cpu(ohdr->bth[2]));
	aeth = be32_to_cpu(ohdr->u.tid_rdma.r_rsp.aeth);
	opcode = (be32_to_cpu(ohdr->bth[0]) >> 24) & 0xff;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] s_last_psn 0x%x, s_acked %u s_tail %u",
		  qp->ibqp.qp_num, qp->s_last_psn, qp->s_acked, qp->s_tail);
#endif

	spin_lock_irqsave(&qp->s_lock, flags);
	ipsn = mask_psn(be32_to_cpu(ohdr->u.tid_rdma.r_rsp.verbs_psn));
	req = find_tid_request(qp, ipsn, IB_WR_TID_RDMA_READ);
	if (unlikely(!req))
		goto ack_op_err;

	flow = &req->flows[req->clear_tail];
	req->ack_pending--;
	priv->pending_tid_r_segs--;
	qp->s_num_rd_atomic--;
	if ((qp->s_flags & RVT_S_WAIT_FENCE) &&
	    !qp->s_num_rd_atomic) {
		qp->s_flags &= ~(RVT_S_WAIT_FENCE |
				 RVT_S_WAIT_ACK);
		hfi1_schedule_send(qp);
	}
	if (qp->s_flags & RVT_S_WAIT_RDMAR) {
		qp->s_flags &= ~(RVT_S_WAIT_RDMAR | RVT_S_WAIT_ACK);
		hfi1_schedule_send(qp);
	}

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] kpsn 0x%x ipsn 0x%x flow %p, fidx %u, comp_seg %u, n %u",
		  qp->ibqp.qp_num, kpsn, ipsn, flow, (flow ? flow->idx : -1),
		  req->comp_seg, req->total_segs);
#endif

	trace_hfi1_ack(qp, ipsn);

	if (!do_rc_ack(qp, aeth, ipsn, opcode, 0, rcd))
		goto ack_done;

	/* Release the tid resources */
	hfi1_kern_exp_rcv_clear(req);

	/* If not done yet, build next read request */
	if (++req->comp_seg >= req->total_segs) {
		priv->tid_r_comp++;
		req->state = TID_REQUEST_COMPLETE;
	}

	/*
	 * Clear the hw flow under two conditions:
	 * 1. This request is a sync point and it is complete;
	 * 2. Current request is completed and there are no more requests.
	 */
	if ((req->state == TID_REQUEST_SYNC && req->comp_seg == req->cur_seg)
	    || priv->tid_r_comp == READ_ONCE(priv->tid_r_reqs)) {
		hfi1_kern_clear_hw_flow(priv->rcd, qp);
		if (req->state == TID_REQUEST_SYNC)
			req->state = TID_REQUEST_ACTIVE;
	}

	hfi1_schedule_send(qp);
	goto ack_done;

ack_op_err:
	/*
	 * The test indicates that the send engine has finished its cleanup
	 * after sending the request and it's now safe to put the QP into error
	 * state. However, if the wqe queue is empty (qp->s_acked == qp->s_tail
	 * == qp->s_head), it would be unsafe to complete the wqe pointed by
	 * qp->s_acked here. Putting the qp into error state will safely flush
	 * all remaining requests.
	 */
	if (qp->s_last == qp->s_acked)
		rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);

ack_done:
	spin_unlock_irqrestore(&qp->s_lock, flags);
	if (is_fecn)
		hfi1_send_rc_ack(rcd, qp, is_fecn);
	return;
}

void hfi1_rc_rcv_tid_rdma_ack(struct hfi1_packet *packet)
{
	struct ib_other_headers *ohdr = packet->ohdr;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_qp_priv *qpriv = qp->priv;
#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_ibport_priv *ibp_priv = ibp->rvp.priv;
#endif
	struct rvt_swqe *wqe;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	u32 aeth, psn, req_psn, ack_psn;
	bool is_fecn;
	unsigned long flags;
	int nsegs;
	u32 last_acked;
#ifdef TIDRDMA_DEBUG
	u16 fidx;
#endif

#ifdef CONFIG_HFI1_TID_RDMA_COUNTERS
	this_cpu_inc(*ibp_priv->trdma_cnts.rx.ack);
#endif
	is_fecn = process_ecn(qp, packet, false);
	psn = mask_psn(be32_to_cpu(ohdr->bth[2]));
	aeth = be32_to_cpu(ohdr->u.tid_rdma.ack.aeth);
	req_psn = mask_psn(be32_to_cpu(ohdr->u.tid_rdma.ack.verbs_psn));

	spin_lock_irqsave(&qp->s_lock, flags);
	trace_hfi1_ack(qp, psn);

	ack_psn = req_psn;
	if (aeth >> 29) {
		ack_psn--;
		psn--;
	}

	wqe = rvt_get_swqe_ptr(qp, qp->s_acked);

	if (wqe->wr.opcode != IB_WR_TID_RDMA_WRITE)
		goto ack_op_err;

	req = wqe_to_tid_req(wqe);
	/* Deal with coalesced ACKs */
	nsegs = delta_psn(ack_psn, req->r_last_acked);

	/* Drop stale ACK/NAK */
	if (nsegs < 0)
		goto ack_op_err;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] received TID RDMA ACK psn 0x%x s_acked %u opcode 0x%x r_last_acked 0x%x wpsn 0x%x-%x req_psn 0x%x nsegs %u s_tid_tail %u",
		  qp->ibqp.qp_num, psn, qp->s_acked,
		  wqe->wr.opcode, req->r_last_acked, wqe->psn,
		  wqe->lpsn, ack_psn, nsegs, qpriv->s_tid_tail);
#endif

	last_acked = qp->s_acked;
	flow = &req->flows[req->acked_tail];
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] psn 0x%x flow lpsn 0x%x cmp %d",
		  qp->ibqp.qp_num, psn, full_flow_psn(flow,
						      flow->flow_state.lpsn),
		  cmp_psn(psn,
			  full_flow_psn(flow, flow->flow_state.lpsn)));
#endif
	while (cmp_psn(psn,
		       full_flow_psn(flow, flow->flow_state.lpsn)) >= 0) {
		req->ack_seg++;
		/* advance acked segment pointer */
		smp_store_release(&req->acked_tail,
				  CIRC_NEXT(req->acked_tail, req->n_max_flows));
		req->r_last_acked = flow->flow_state.resp_ib_psn;
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] r_last_acked 0x%x ack_seg %u total_segs %u",
			  qp->ibqp.qp_num, req->r_last_acked, req->ack_seg,
			  req->total_segs);
#endif
		if (req->ack_seg == req->total_segs) {
			req->state = TID_REQUEST_COMPLETE;
			last_acked = qp->s_acked;
			wqe = do_rc_completion(qp, wqe,
					       to_iport(qp->ibqp.device,
							qp->port_num));
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA,
				  "[QP%u] last_acked %u s_acked %u s_tail %u",
				  qp->ibqp.qp_num, last_acked, qp->s_acked,
				  qp->s_tail);
#endif
			atomic_dec(&qpriv->n_tid_requests);
			if (qp->s_acked == qp->s_tail)
				break;
			if (wqe->wr.opcode != IB_WR_TID_RDMA_WRITE)
				break;
			req = wqe_to_tid_req(wqe);
		}
		if (req->ack_seg == req->comp_seg)
			break;
		flow = &req->flows[req->acked_tail];
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA, "[QP%u] psn 0x%x flow lpsn 0x%x cmp %d",
			  qp->ibqp.qp_num, psn,
			  full_flow_psn(flow, flow->flow_state.lpsn),
			  cmp_psn(psn,
				  full_flow_psn(flow, flow->flow_state.lpsn)));
#endif
	}

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] AETH: 0x%x, %u, PSN 0x%x, acked_tail %u clear_tail %u setup_head %u, s_acked %u, last_acked %u, s_tid_tail %u, s_tid_cur %u",
		  qp->ibqp.qp_num, aeth, (aeth >> 29), psn, req->acked_tail,
		  req->clear_tail, req->setup_head, qp->s_acked, last_acked,
		  qpriv->s_tid_tail, qpriv->s_tid_cur);
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] wpsn 0x%x-%x ack_seg %u cur_seg %u s_flags 0x%x",
		  qp->ibqp.qp_num, wqe->psn, wqe->lpsn, req->ack_seg,
		  req->cur_seg, qpriv->s_flags);
#endif
	switch (aeth >> 29) {
	case 0:
		/* Check if there is any pending TID ACK */
		if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE &&
		    req->ack_seg < req->cur_seg)
			hfi1_mod_tid_retry_timer(qp);
		else
			hfi1_stop_tid_retry_timer(qp);

		qpriv->s_retry = qp->s_retry_cnt;
		hfi1_schedule_send(qp);
		break;

	case 3:
		hfi1_stop_tid_retry_timer(qp);
		switch ((aeth >> IB_AETH_CREDIT_SHIFT) &
			IB_AETH_CREDIT_MASK) {
		case 0: /* PSN sequence error */
			if (!qpriv->s_retry) {
				hfi1_send_complete(qp, wqe,
						   IB_WC_RETRY_EXC_ERR);
				rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);
			} else {
				u32 fspsn;

				qpriv->s_retry--;
				flow = &req->flows[req->acked_tail];
				fspsn = full_flow_psn(flow,
						      flow->flow_state.spsn);

#ifdef TIDRDMA_DEBUG
				hfi1_cdbg(TIDRDMA,
					  "[QP%u] PSN 0x%x, delta %d, fidx %u, spn 0x%x lspn 0x%x clear_tail %u setup_head %u",
					  qp->ibqp.qp_num, psn,
					  delta_psn(psn, fspsn), fidx, fspsn,
					  full_flow_psn(flow,
							flow->flow_state.lpsn),
					  req->clear_tail, req->setup_head);
#endif
				req->r_ack_psn =
					mask_psn(be32_to_cpu(ohdr->bth[2]));
				req->cur_seg = req->ack_seg;
				qpriv->s_tid_tail = qp->s_acked;
				qpriv->s_state = TID_OP(WRITE_REQ);
				hfi1_schedule_tid_send(qp);
			}
			break;

		default:
			break;
		}
		break;

	default:
		break;
	}

ack_op_err:
	spin_unlock_irqrestore(&qp->s_lock, flags);
	return;
}

void hfi1_tid_timeout(unsigned long arg)
{
	struct rvt_qp *qp = (struct rvt_qp *)arg;
	struct rvt_dev_info *rdi = ib_to_rvt(qp->ibqp.device);
	struct hfi1_qp_priv *qpriv = qp->priv;
	unsigned long flags;
	u32 i;

	spin_lock_irqsave(&qp->r_lock, flags);
	spin_lock(&qp->s_lock);
	if (qpriv->s_flags & HFI1_R_TID_RSC_TIMER) {
#ifdef TIDRDMA_DEBUG
		dd_dev_warn(dd_from_ibdev(qp->ibqp.device), "[QP%u] %s %d\n",
			    qp->ibqp.qp_num, __func__, __LINE__);
		hfi1_cdbg(TIDRDMA, "[QP%u] %d", qp->ibqp.qp_num, __LINE__);
#endif
		hfi1_stop_tid_reap_timer(qp);
		/*
		 * Go though the entire ack queue and clear any outstanding
		 * HW flow and RcvArray resources.
		 */
		hfi1_kern_clear_hw_flow(qpriv->rcd, qp);
		for (i = 0; i < rvt_max_atomic(rdi); i++) {
			struct tid_rdma_request *req =
				ack_to_tid_req(&qp->s_ack_queue[i]);

			hfi1_kern_exp_rcv_clear_all(req);
		}
		spin_unlock(&qp->s_lock);
		if (qp->ibqp.event_handler) {
			struct ib_event ev;

			ev.device = qp->ibqp.device;
			ev.element.qp = &qp->ibqp;
			ev.event = IB_EVENT_QP_FATAL;
			qp->ibqp.event_handler(&ev, qp->ibqp.qp_context);
		}
		rvt_rc_error(qp, IB_WC_RESP_TIMEOUT_ERR);
		goto unlock_r_lock;
	}
	spin_unlock(&qp->s_lock);
unlock_r_lock:
	spin_unlock_irqrestore(&qp->r_lock, flags);
}

static inline struct rvt_ack_entry *
find_prev_entry_kdeth(struct rvt_qp *qp, u32 psn, u8 *prev, u8 *prev_ack,
		      bool *scheduled, u16 *fidx)
	__must_hold(&qp->s_lock)
{
	struct rvt_ack_entry *e = NULL;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	u32 ibpsn = KDETH_INVALID_PSN;
	u16 idx = 0;
	u8 i, p;
	bool s = true;

	for (i = qp->r_head_ack_queue; ; i = p) {
		if (i == qp->s_tail_ack_queue)
			s = false;
		if (i)
			p = i - 1;
		else
			p = rvt_size_atomic(ib_to_rvt(qp->ibqp.device));
		if (p == qp->r_head_ack_queue) {
			e = NULL;
			break;
		}
		e = &qp->s_ack_queue[p];
		if (!e->opcode) {
			e = NULL;
			break;
		}
		req = ack_to_tid_req(e);
		flow = __find_flow_ranged(req, req->n_max_flows - 1, 0,
					  psn, &idx);
		if (flow) {
			ibpsn = flow->flow_state.resp_ib_psn;
			if (cmp_psn(ibpsn, e->psn) >= 0) {
				if (p == qp->s_tail_ack_queue &&
				    cmp_psn(ibpsn, e->lpsn) <= 0)
					s = false;
				break;
			}
		}
	}
	if (prev)
		*prev = p;
	if (prev_ack)
		*prev_ack = i;
	if (scheduled)
		*scheduled = s;
	if (fidx)
		*fidx = idx;
	return e;
}

/* Free all read-associated tid resources from the given qp */
void hfi1_kern_read_tid_flow_free(struct rvt_qp *qp)
	__must_hold(&qp->s_lock)
{
	u32 n = qp->s_acked;
	struct rvt_swqe *wqe;
	struct tid_rdma_request *req;
	struct hfi1_qp_priv *priv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	/* Free any TID entries */
	while (n != qp->s_tail) {
		wqe = rvt_get_swqe_ptr(qp, n);
		if (wqe->wr.opcode == IB_WR_TID_RDMA_READ) {
			req = wqe_to_tid_req(wqe);
			hfi1_kern_exp_rcv_clear_all(req);
		}

		if (++n == qp->s_size)
			n = 0;
	}
	/* Free flow */
	hfi1_kern_clear_hw_flow(priv->rcd, qp);
}

static bool tid_rdma_tid_err(struct hfi1_ctxtdata *rcd,
			     struct hfi1_packet *packet, u8 rcv_type,
			     u8 opcode, u32 psn)
{
	struct rvt_ack_entry *e;
	//struct tid_rdma_request *req;
	//struct tid_rdma_flow *flow;
	struct rvt_qp *qp = packet->qp;
	u32 ibpsn;
	bool ret = true;
	//u8 prev;
	struct ib_other_headers *ohdr = packet->ohdr;

	if (rcv_type >= RHF_RCV_TYPE_IB)
		goto done;

	spin_lock(&qp->s_lock);

	/*
	 * We've ran out of space in the eager buffer.
	 * Eagerly received KDETH packets which require space in the
	 * Eager buffer (packet that have payload) are TID RDMA WRITE
	 * response packets. In this case, we have to re-transmit the
	 * TID RDMA WRITE request.
	 */
	if (rcv_type == RHF_RCV_TYPE_EAGER) {
		hfi1_restart_rc(qp, qp->s_last_psn + 1, 1);
		hfi1_schedule_send(qp);
		goto done_unlock;
	}

	/*
	 * For TID READ response, error out QP after freeing the tid
	 * resources.
	 */
	if (opcode == TID_OP(READ_RESP)) {
		ibpsn = ohdr->u.tid_rdma.r_rsp.verbs_psn;
		if (cmp_psn(psn, qp->s_last_psn) > 0 &&
		    cmp_psn(psn, qp->s_psn) < 0) {

			hfi1_kern_read_tid_flow_free(qp);
			spin_unlock(&qp->s_lock);
			rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
		}
		goto done;
	}

	e = &qp->s_ack_queue[qp->s_tail_ack_queue];

	/*
	 * There are two ways in which we can end up here:
	 *   1. The buffer programmed into the RcvArray for this
	 *      expected transfer do not have enough room for
	 *      the data. This can only occur due to a software
	 *      error, so error out the QP.
	 *   2. We are getting re-trasmissions of TID RDMA DATA
	 *      packets for previous segments or requests.
	 */
#if 0
	ibpsn = kdeth_to_ib_psn_ack(e, psn);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] ibpsn 0x%x", qp->ibqp.qp_num, ibpsn);
#endif
	if (ibpsn != KDETH_INVALID_PSN) {
		spin_unlock(&qp->s_lock);
		qp->r_nak_state = IB_NAK_REMOTE_ACCESS_ERROR;
		qp->r_ack_psn = ibpsn;
		rvt_rc_error(qp, IB_WC_LOC_LEN_ERR);
		rc_defered_ack(rcd, qp);
		goto done;
	} else {
		e = find_prev_entry_kdeth(qp, psn, &prev, NULL,
					  NULL, &fidx);
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA, "[QP%u] e %p, prev %u, fidx %u",
			  qp->ibqp.qp_num, e, prev, fidx);
#endif
		if (!e) {
			/* XXX Something went horribly wrong!! */
			spin_unlock(&qp->s_lock);
			/* We can't NAK since we don't know the IB
			 * PSN. */
			rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
			goto done;
		}
		req = ack_to_tid_req(e);
		flow = &req->flows[fidx];
		flow->flow_state.flags |= TID_FLOW_SW_PSN;
		flow->flow_state.r_next_psn = psn;
		qp->s_tail_ack_queue = prev;
		ret = false;
	}
#endif /* if 0 */
done_unlock:
	spin_unlock(&qp->s_lock);
done:
	return ret;
}

static void restart_tid_rdma_read_req(struct hfi1_ctxtdata *rcd,
				      struct rvt_qp *qp, struct rvt_swqe *wqe)
{
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;

	/* Start from the right segment */
	qp->r_flags |= RVT_R_RDMAR_SEQ;
	req = wqe_to_tid_req(wqe);
	flow = &req->flows[req->clear_tail];
	hfi1_restart_rc(qp, flow->flow_state.ib_spsn, 0);
	if (list_empty(&qp->rspwait)) {
		qp->r_flags |= RVT_R_RSP_SEND;
		rvt_get_qp(qp);
		list_add_tail(&qp->rspwait, &rcd->qp_wait_list);
	}
}

/*
 * Handle the KDETH eflags for TID RDMA READ response. 
 *  
 * Return true if the last packet for a segment has been received and it is 
 * time to process the response normally; otherwise, return true. 
 */
static bool handle_read_kdeth_eflags(struct hfi1_ctxtdata *rcd,
				    struct hfi1_packet *packet, u8 rcv_type,
				    u8 rte, u32 psn, u32 ibpsn)
{
	struct hfi1_pportdata *ppd = rcd->ppd;
	struct hfi1_devdata *dd = ppd->dd;
	struct hfi1_ibport *ibp;
	struct rvt_swqe *wqe;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	u32 ack_psn;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_qp_priv *priv = qp->priv;
	bool ret = true;
	int diff = 0;
	u32 fpsn;

	/* If the psn is out of valid range, drop the packet */
	if (cmp_psn(ibpsn, qp->s_last_psn) < 0 ||
	    cmp_psn(ibpsn, qp->s_psn) > 0)
		return ret;

	spin_lock(&qp->s_lock);
	/*
	 * Note that NAKs implicitly ACK outstanding SEND and RDMA write
	 * requests and implicitly NAK RDMA read and atomic requests issued
	 * before the NAK'ed request.
	 */
	ack_psn = ibpsn - 1;
	wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
	ibp = to_iport(qp->ibqp.device, qp->port_num);

	/* Complete WQEs that the PSN finishes. */
	while ((int)delta_psn(ack_psn, wqe->lpsn) >= 0) {
		/*
		 * If this request is a RDMA read or atomic, and the NACK is
		 * for a later operation, this NACK NAKs the RDMA read or
		 * atomic.
		 */
		if (wqe->wr.opcode == IB_WR_RDMA_READ ||
		    wqe->wr.opcode == IB_WR_TID_RDMA_READ ||
		    wqe->wr.opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
		    wqe->wr.opcode == IB_WR_ATOMIC_FETCH_AND_ADD) {
			/* Retry this request. */
			if (!(qp->r_flags & RVT_R_RDMAR_SEQ)) {
				qp->r_flags |= RVT_R_RDMAR_SEQ;
				if (wqe->wr.opcode == IB_WR_TID_RDMA_READ) {
					restart_tid_rdma_read_req(rcd, qp,
								  wqe);
				} else {
					hfi1_restart_rc(qp, qp->s_last_psn + 1, 0);
					if (list_empty(&qp->rspwait)) {
						qp->r_flags |= RVT_R_RSP_SEND;
						rvt_get_qp(qp);
						list_add_tail(
						   &qp->rspwait,
						   &rcd->qp_wait_list);
					}
				}
			}
			/*
			 * No need to process the NAK since we are
			 * restarting an earlier request.
			 */
			break;
		}

		/*
		 * TID RDMA WRITE requests will be completed by the TID RDMA
		 * ACK packet handler (see tid_rdma.c).
		 */
		/* It might not be possible due to interlock */
		/* if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
			break; */

		wqe = do_rc_completion(qp, wqe, ibp);
		if (qp->s_acked == qp->s_tail)
			break;
	}

	/* Handle the eflags for the request */
	if (wqe->wr.opcode != IB_WR_TID_RDMA_READ)
		goto s_unlock;

	req = wqe_to_tid_req(wqe);
	switch (rcv_type) {
	case RHF_RCV_TYPE_EXPECTED:
		switch (rte) {
		case RHF_RTE_EXPECTED_FLOW_SEQ_ERR:
			/*
			 * On the first occurrence of a Flow Sequence error,
			 * the flag TID_FLOW_SW_PSN is set.
			 *
			 * After that, the flow is *not* reprogrammed and the
			 * protocol falls back to SW PSN checking. This is done
			 * to prevent continuous Flow Sequence errors for any
			 * packets that could be still in the fabric.
			 */
			flow = find_flow(req, psn, NULL);
			if (!flow) {
				/*
				 * We can't find the IB PSN matching the
				 * received KDETH PSN. The only thing we can
				 * do at this point is report the error to
				 * the QP.
				 */
				hfi1_kern_read_tid_flow_free(qp);
				spin_unlock(&qp->s_lock);
				rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
				return ret;
			}
			if (priv->flow_state.flags & TID_FLOW_SW_PSN) {
				diff = cmp_psn(psn,
					       priv->flow_state.r_next_psn);
				if (diff > 0) {
					if (!(qp->r_flags & RVT_R_RDMAR_SEQ))
						restart_tid_rdma_read_req(rcd,
									  qp,
									  wqe);

					/* Drop the packet.*/
					goto s_unlock;
				} else if (diff < 0) {
					/*
					 * If a response packet for a restarted
					 * request has come back, reset the
					 * restart flag.
					 */
					if (qp->r_flags & RVT_R_RDMAR_SEQ)
						qp->r_flags &=
							~RVT_R_RDMAR_SEQ;

					/* Drop the packet.*/
					goto s_unlock;
				}

				/*
				 * If SW PSN verification is successful and
				 * this is the last packet in the segment, tell
				 * the caller to process it as a normal packet.
				 */
				fpsn = full_flow_psn(flow,
						     flow->flow_state.lpsn);
				if (cmp_psn(fpsn, psn) == 0) {
					ret = false;
					if (qp->r_flags & RVT_R_RDMAR_SEQ)
						qp->r_flags &=
							~RVT_R_RDMAR_SEQ;
				}
				priv->flow_state.r_next_psn++;
			} else {
				u64 reg;
				u32 last_psn;

				/*
				 * The only sane way to get the amount of
				 * progress is to read the HW flow state.
				 */
				reg = read_uctxt_csr(dd, rcd->ctxt,
						     RCV_TID_FLOW_TABLE +
						     (8 * flow->idx));
				last_psn = mask_psn(reg);

				priv->flow_state.r_next_psn = last_psn;
				priv->flow_state.flags |= TID_FLOW_SW_PSN;
				/*
				 * If no request has been restarted yet,
				 * restart the current one.
				 */
				if (!(qp->r_flags & RVT_R_RDMAR_SEQ))
					restart_tid_rdma_read_req(rcd, qp,
								  wqe);
			}

			break;

		case RHF_RTE_EXPECTED_FLOW_GEN_ERR:
			/*
			 * Since the TID flow is able to ride through 
			 * generation mismatch, drop this stale
			 * packet.
			 */
			break;

		default:
			break;
		}
		break;

	case RHF_RCV_TYPE_ERROR:
		switch (rte) {
		case RHF_RTE_ERROR_OP_CODE_ERR:
		case RHF_RTE_ERROR_KHDR_MIN_LEN_ERR:
		case RHF_RTE_ERROR_KHDR_HCRC_ERR:
		case RHF_RTE_ERROR_KHDR_KVER_ERR:
		case RHF_RTE_ERROR_CONTEXT_ERR:
		case RHF_RTE_ERROR_KHDR_TID_ERR:
		default:
			break;
		}
	default:
		break;
	}
s_unlock:
	spin_unlock(&qp->s_lock);
	return ret;
}

bool hfi1_handle_kdeth_eflags(struct hfi1_ctxtdata *rcd,
			      struct hfi1_pportdata *ppd,
			      struct hfi1_packet *packet)
{
	struct hfi1_ibport *ibp = &ppd->ibport_data;
	struct hfi1_devdata *dd = ppd->dd;
	struct rvt_dev_info *rdi = &dd->verbs_dev.rdi;
	u8 rcv_type = rhf_rcv_type(packet->rhf);
	u8 rte = rhf_rcv_type_err(packet->rhf);
	struct ib_header *hdr = packet->hdr;
	struct ib_other_headers *ohdr = NULL;
	struct rvt_ack_entry *e;
	struct tid_rdma_request *req;
	struct tid_rdma_flow *flow;
	int lnh = be16_to_cpu(hdr->lrh[0]) & 3;
	u16 lid  = be16_to_cpu(hdr->lrh[1]);
	u8 opcode;
	u32 qp_num, psn, ibpsn;
	struct rvt_qp *qp;
	struct hfi1_qp_priv *qpriv;
	unsigned long flags;
	bool ret = true;

	hfi1_cdbg(TIDRDMA, "receive context %u: rhf 0x%016llx", rcd->ctxt,
		  packet->rhf);

	if (packet->rhf & (RHF_VCRC_ERR | RHF_ICRC_ERR))
		return ret;

	packet->ohdr = &hdr->u.oth;
	ohdr = packet->ohdr;
	trace_input_ibhdr(rcd->dd, packet,
			  !!(packet->rhf & RHF_DC_INFO_SMASK));

	/* Get the destination QP number. */
	qp_num = be32_to_cpu(ohdr->u.tid_rdma.w_data.verbs_qp) &
		RVT_QPN_MASK;
	if (lid >= be16_to_cpu(IB_MULTICAST_LID_BASE))
		goto drop;

	psn = mask_psn(be32_to_cpu(ohdr->bth[2]));
	opcode = (be32_to_cpu(ohdr->bth[0]) >> 24) & 0xff;

	rcu_read_lock();
	qp = rvt_lookup_qpn(rdi, &ibp->rvp, qp_num);
	if (!qp)
		goto rcu_unlock;

	packet->qp = qp;

	/* Check for valid receive state. */
	spin_lock_irqsave(&qp->r_lock, flags);
	if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK)) {
		ibp->rvp.n_pkt_drops++;
		goto r_unlock;
	}

	if (packet->rhf & RHF_TID_ERR) {
		/* For TIDERR and RC QPs preemptively schedule a NAK */
		u32 tlen = rhf_pkt_len(packet->rhf); /* in bytes */

		/* Sanity check packet */
		if (tlen < 24)
			goto unlock;

		/*
		 * Check for GRH. We should never get packets with GRH in this
		 * path.
		 */
		if (lnh == HFI1_LRH_GRH)
			goto unlock;

		if (tid_rdma_tid_err(rcd, packet, rcv_type, opcode, psn))
			goto unlock;
	}

		/* handle TID RDMA READ */
	if (opcode == TID_OP(READ_RESP)) {
		ibpsn = be32_to_cpu(ohdr->u.tid_rdma.r_rsp.verbs_psn);
		ibpsn = mask_psn(ibpsn);
		ret = handle_read_kdeth_eflags(rcd, packet, rcv_type, rte, psn,
					       ibpsn);
		goto r_unlock;
	}

	/*
	 * qp->s_tail_ack_queue points to the rvt_ack_entry currently being
	 * processed. These a completed sequentially so we can be sure that
	 * the pointer will not change until the entire request has completed.
	 */
	spin_lock(&qp->s_lock);
	qpriv = qp->priv;
	e = &qp->s_ack_queue[qpriv->r_tid_tail];
	req = ack_to_tid_req(e);
	flow = &req->flows[req->clear_tail];

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] TID ERR: RcvType: 0x%x, RcvTypeErr: 0x%x, PSN: 0x%x",
		  qp->ibqp.qp_num, rcv_type, rte, psn);
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] s_tail_ack_queue %u r_tid_tail %u e->psn 0x%x, e->lpsn 0x%x, cur_se %u, comp_seg %u, total_segs %u, setup_head %u clear_tail %u flow_idx %u",
		  qp->ibqp.qp_num, qp->s_tail_ack_queue, qpriv->r_tid_tail,
		  e->psn, e->lpsn, req->cur_seg, req->comp_seg, req->total_segs,
		  req->setup_head, req->clear_tail, req->flow_idx);
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] resp_ib_psn 0x%x, spsn 0x%x, lpsn 0x%x, r_next_psn 0x%x",
		  qp->ibqp.qp_num, flow->flow_state.resp_ib_psn,
		  flow->flow_state.spsn, flow->flow_state.lpsn,
		  flow->flow_state.r_next_psn);
#endif

	switch (rcv_type) {
	case RHF_RCV_TYPE_EXPECTED:
		switch (rte) {
		case RHF_RTE_EXPECTED_FLOW_SEQ_ERR:
			if (!(qpriv->s_flags & HFI1_R_TID_SW_PSN)) {
				u64 reg;

				qpriv->s_flags |= HFI1_R_TID_SW_PSN;
				/*
				 * The only sane way to get the amount of
				 * progress is to read the HW flow state.
				 */
				reg = read_uctxt_csr(dd, rcd->ctxt,
						     RCV_TID_FLOW_TABLE +
						     (8 * flow->idx));
				flow->flow_state.r_next_psn = mask_psn(reg);
				goto nak_psn;
			} else {
				/*
				 * If the received PSN does not match the next
				 * expected PSN, NAK the packet.
				 * However, only do that if we know that the a
				 * NAK has already been sent. Otherwise, this
				 * mismatch could be due to packets that were
				 * already in flight.
				 */
				if (psn != flow->flow_state.r_next_psn) {
					psn = flow->flow_state.r_next_psn;
					goto nak_psn;
				}

				qpriv->s_nak_state = 0;
				/*
				 * If SW PSN verification is successful and this
				 * is the last packet in the segment, tell the
				 * caller to process it as a normal packet.
				 */
				if (psn == full_flow_psn(flow,
							 flow->flow_state.lpsn))
					ret = false;
				flow->flow_state.r_next_psn++;
			}
			break;

		case RHF_RTE_EXPECTED_FLOW_GEN_ERR:
#if 0
			opcode = (be32_to_cpu(ohdr->bth[0]) >> 24) & 0xff;
			flow = __find_flow_ranged(req, req->n_max_flows - 1, 0,
						  psn, NULL);
			flpsn = full_flow_psn(flow,
					      flow->flow_state.lpsn);
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA,
				  "[QP%u] flags 0x%x psn 0x%x r_next_psn 0x%x lpsn 0x%x",
				  qp->ibqp.qp_num, flow->flow_state.flags, psn,
				  flow->flow_state.r_next_psn, flpsn);
#endif
			if ((flow->flow_state.flags & TID_FLOW_SW_PSN) &&
			    (psn == flow->flow_state.r_next_psn)) {
				if (psn == flpsn) {
					qp->s_ack_state = opcode;
					qp->s_flags |= RVT_S_ACK_PENDING |
						RVT_S_RESP_PENDING;
					hfi1_schedule_send(qp);
					goto unlock;
				}
				flow->flow_state.r_next_psn = psn + 1;
			} else {
				goto nak_psn;
			}
#endif
			goto nak_psn;
			break;

		default:
			break;
		}
		break;

	case RHF_RCV_TYPE_ERROR:
		switch (rte) {
		case RHF_RTE_ERROR_OP_CODE_ERR:
		case RHF_RTE_ERROR_KHDR_MIN_LEN_ERR:
		case RHF_RTE_ERROR_KHDR_HCRC_ERR:
		case RHF_RTE_ERROR_KHDR_KVER_ERR:
		case RHF_RTE_ERROR_CONTEXT_ERR:
		case RHF_RTE_ERROR_KHDR_TID_ERR:
		default:
			break;
		}
	default:
		break;
	}

unlock:
	spin_unlock(&qp->s_lock);
r_unlock:
	spin_unlock_irqrestore(&qp->r_lock, flags);
rcu_unlock:
	rcu_read_unlock();
drop:
	return ret;
nak_psn:
	ibp->rvp.n_rc_seqnak++;
	if (!qpriv->s_nak_state) {
		qpriv->s_nak_state = IB_NAK_PSN_ERROR;
		qpriv->s_nak_psn = flow->flow_state.r_next_psn;
		qpriv->s_flags |= RVT_S_ACK_PENDING | RVT_S_RESP_PENDING;
		if (qpriv->r_tid_ack == HFI1_QP_WQE_INVALID)
			qpriv->r_tid_ack = qpriv->r_tid_tail;
		hfi1_schedule_tid_send(qp);
	}
	spin_unlock(&qp->s_lock);
	goto r_unlock;
}

/*
 * @qp: points to rvt_qp context.
 * @to_seg: desired RNR timeout in segments.
 * Return: index of the next highest timeout in the ib_hfi1_rnr_table[]
 */
u32 hfi1_compute_tid_rnr_timeout(struct rvt_qp *qp, u32 to_seg)
{
	struct hfi1_qp_priv *qpriv = qp->priv;
	u64 timeout;
	u32 bytes_per_us;
	u8 i;

	bytes_per_us = active_egress_rate(qpriv->rcd->ppd) / 8;
	timeout = (to_seg * hfi1_tid_rdma_seg_max_size) / bytes_per_us;
	/*
	 * Find the next highest value in the RNR table to the required
	 * timeout. This gives the responder some padding.
	 */
	for (i = 1; i <= IB_AETH_CREDIT_MASK; i++)
		if (rvt_rnr_tbl_to_usec(i) >= timeout)
			return i;
	return 0;
}

bool hfi1_tid_rdma_wqe_interlock(struct rvt_qp *qp, struct rvt_swqe *wqe)
{
	struct rvt_swqe *prev;
	struct tid_rdma_request *req;
	struct hfi1_qp_priv *priv = qp->priv;
	u32 s_prev;

	s_prev = (qp->s_cur == 0 ? qp->s_size : qp->s_cur) - 1;
	prev = rvt_get_swqe_ptr(qp, s_prev);

	switch (wqe->wr.opcode) {
	case IB_WR_SEND:
	case IB_WR_SEND_WITH_IMM:
	case IB_WR_SEND_WITH_INV:
	case IB_WR_ATOMIC_CMP_AND_SWP:
	case IB_WR_ATOMIC_FETCH_AND_ADD:
	case IB_WR_RDMA_WRITE:
		switch (prev->wr.opcode) {
		case IB_WR_TID_RDMA_WRITE:
			req = wqe_to_tid_req(prev);
			if (req->ack_seg != req->total_segs)
				goto interlock;
		default:
			break;
		}
	case IB_WR_RDMA_READ:
		if (prev->wr.opcode != IB_WR_TID_RDMA_WRITE)
			break;
		/* fall through */
	case IB_WR_TID_RDMA_READ:
		switch (prev->wr.opcode) {
		case IB_WR_RDMA_READ:
			if (qp->s_acked != qp->s_cur)
				goto interlock;
			break;
		case IB_WR_TID_RDMA_WRITE:
			req = wqe_to_tid_req(prev);
			if (req->ack_seg != req->total_segs)
				goto interlock;
		default:
			break;
		}
	default:
		break;
	}
	return false;

interlock:
	priv->s_flags |= HFI1_S_TID_WAIT_INTERLCK;
	return true;
}

bool hfi1_tid_rdma_ack_interlock(struct rvt_qp *qp, struct rvt_ack_entry *e)
{
	struct rvt_ack_entry *prev;
	struct tid_rdma_request *req;
	struct hfi1_ibdev *dev = to_idev(qp->ibqp.device);
	struct hfi1_qp_priv *priv = qp->priv;
	u32 s_prev;

	s_prev = qp->s_tail_ack_queue == 0 ? rvt_size_atomic(&dev->rdi) :
		(qp->s_tail_ack_queue - 1);
	prev = &qp->s_ack_queue[s_prev];

	if ((e->opcode == TID_OP(READ_REQ) ||
	     e->opcode == OP(RDMA_READ_REQUEST)) &&
	    prev->opcode == TID_OP(WRITE_REQ)) {
		req = ack_to_tid_req(prev);
		if (req->ack_seg != req->total_segs) {
			priv->s_flags |= HFI1_R_TID_WAIT_INTERLCK;
			return true;
		}
	}
	return false;
}

void hfi1_tid_retry_timeout(unsigned long arg)
{
	struct rvt_qp *qp = (struct rvt_qp *)arg;
	struct hfi1_qp_priv *priv = qp->priv;
	unsigned long flags;
	struct hfi1_devdata *dd = dd_from_ibdev(qp->ibqp.device);

	spin_lock_irqsave(&qp->r_lock, flags);
	spin_lock(&qp->s_lock);
	if (priv->s_flags & HFI1_S_TID_RETRY_TIMER) {
		hfi1_stop_tid_retry_timer(qp);
		spin_unlock(&qp->s_lock);
		/* For the time being, put the qp into error state */
		dd_dev_warn(dd, "Missing TID ACK: put qp into error state.\n");
		if (qp->ibqp.event_handler) {
			struct ib_event ev;

			ev.device = qp->ibqp.device;
			ev.element.qp = &qp->ibqp;
			ev.event = IB_EVENT_QP_FATAL;
			qp->ibqp.event_handler(&ev, qp->ibqp.qp_context);
		}
		rvt_rc_error(qp, IB_WC_RESP_TIMEOUT_ERR);
		goto unlock_r_lock;

		/* restart_tid_write_data(qp); */
		/* hfi1_schedule_tid_send(qp); */
	}
	spin_unlock(&qp->s_lock);
unlock_r_lock:
	spin_unlock_irqrestore(&qp->r_lock, flags);
}
