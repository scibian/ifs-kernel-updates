/*
 * Copyright(c) 2015 - 2017 Intel Corporation.
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

#include <linux/err.h>
#include <linux/vmalloc.h>
#include <linux/hash.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <rdma/rdma_vt.h>
#include <rdma/rdmavt_qp.h>
#include <rdma/ib_verbs.h>

#include "hfi.h"
#include "qp.h"
#include "trace.h"
#include "verbs_txreq.h"
#include "user_exp_rcv.h"

unsigned int hfi1_qp_table_size = 256;
module_param_named(qp_table_size, hfi1_qp_table_size, uint, S_IRUGO);
MODULE_PARM_DESC(qp_table_size, "QP table size");

unsigned int hfi1_tid_rdma_seg_min_size = TID_RDMA_MIN_SEGMENT_SIZE;
module_param_named(tid_rdma_seg_min_size, hfi1_tid_rdma_seg_min_size,
		   uint, S_IRUGO);
MODULE_PARM_DESC(tid_rdma_seg_min_size,
		 "Min TID RDMA segment size, default: 64KiB");

unsigned int hfi1_tid_rdma_seg_max_size = TID_RDMA_MAX_SEGMENT_SIZE;
module_param_named(tid_rdma_seg_max_size, hfi1_tid_rdma_seg_max_size,
		   uint, S_IRUGO);
MODULE_PARM_DESC(tid_rdma_seg_max_size,
		 "Max TID RDMA segment size, default: 256KiB");

unsigned int hfi1_tid_rdma_max_read_segs = TID_RDMA_MAX_READ_SEGS_PER_REQ;
module_param_named(tid_rdma_max_read_segs, hfi1_tid_rdma_max_read_segs,
		   uint, S_IRUGO);
MODULE_PARM_DESC(tid_rdma_max_read_segs,
		 "Max TID RDMA READ segments per QP request, default: 6");

unsigned int hfi1_tid_rdma_max_write_segs = TID_RDMA_MAX_WRITE_SEGS_PER_REQ;
module_param_named(tid_rdma_max_write_segs, hfi1_tid_rdma_max_write_segs,
		   uint, S_IRUGO);
MODULE_PARM_DESC(tid_rdma_max_write_segs,
		 "Max TID RDMA WRITE segments per QP request, default: 2");

#define HFI1_TID_RDMA_PROTO_EN_WRITE BIT(0)
#define HFI1_TID_RDMA_PROTO_EN_READ  BIT(1)

unsigned int hfi1_tid_rdma_proto_enable =
	(HFI1_TID_RDMA_PROTO_EN_WRITE | HFI1_TID_RDMA_PROTO_EN_READ);
module_param_named(tid_rdma_proto_enable, hfi1_tid_rdma_proto_enable,
		   uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tid_rdma_proto_enable,
		 "[En|Dis]able TID RDMA READ and/or WRITE protocols.");

static struct hfi1_ctxtdata *qp_to_rcd(
	struct rvt_dev_info *rdi,
	struct rvt_qp *qp);
static void flush_tx_list(struct rvt_qp *qp);
static int iowait_sleep(
	struct sdma_engine *sde,
	struct iowait_work *wait,
	struct sdma_txreq *stx,
	unsigned int seq,
	bool pkts_sent);
static void iowait_wakeup(struct iowait *wait, int reason);
static void iowait_sdma_drained(struct iowait *wait);
static void qp_pio_drain(struct rvt_qp *qp);
static void setup_tid_rdma_wqe(struct rvt_qp *qp, struct rvt_swqe *wqe);

const struct rvt_operation_params hfi1_post_parms[RVT_OPERATION_MAX] = {
[IB_WR_RDMA_WRITE] = {
	.length = sizeof(struct ib_rdma_wr),
	.qpt_support = BIT(IB_QPT_UC) | BIT(IB_QPT_RC),
},

[IB_WR_RDMA_READ] = {
	.length = sizeof(struct ib_rdma_wr),
	.qpt_support = BIT(IB_QPT_RC),
	.flags = RVT_OPERATION_ATOMIC,
},

[IB_WR_ATOMIC_CMP_AND_SWP] = {
	.length = sizeof(struct ib_atomic_wr),
	.qpt_support = BIT(IB_QPT_RC),
	.flags = RVT_OPERATION_ATOMIC | RVT_OPERATION_ATOMIC_SGE,
},

[IB_WR_ATOMIC_FETCH_AND_ADD] = {
	.length = sizeof(struct ib_atomic_wr),
	.qpt_support = BIT(IB_QPT_RC),
	.flags = RVT_OPERATION_ATOMIC | RVT_OPERATION_ATOMIC_SGE,
},

[IB_WR_RDMA_WRITE_WITH_IMM] = {
	.length = sizeof(struct ib_rdma_wr),
	.qpt_support = BIT(IB_QPT_UC) | BIT(IB_QPT_RC),
},

[IB_WR_SEND] = {
	.length = sizeof(struct ib_send_wr),
	.qpt_support = BIT(IB_QPT_UD) | BIT(IB_QPT_SMI) | BIT(IB_QPT_GSI) |
		       BIT(IB_QPT_UC) | BIT(IB_QPT_RC),
},

[IB_WR_SEND_WITH_IMM] = {
	.length = sizeof(struct ib_send_wr),
	.qpt_support = BIT(IB_QPT_UD) | BIT(IB_QPT_SMI) | BIT(IB_QPT_GSI) |
		       BIT(IB_QPT_UC) | BIT(IB_QPT_RC),
},

/* [IB_WR_REG_MR] = {
	.length = sizeof(struct ib_reg_wr),
	.qpt_support = BIT(IB_QPT_UC) | BIT(IB_QPT_RC),
	.flags = RVT_OPERATION_LOCAL,
}, */

[IB_WR_LOCAL_INV] = {
	.length = sizeof(struct ib_send_wr),
	.qpt_support = BIT(IB_QPT_UC) | BIT(IB_QPT_RC),
	.flags = RVT_OPERATION_LOCAL,
},

[IB_WR_SEND_WITH_INV] = {
	.length = sizeof(struct ib_send_wr),
	.qpt_support = BIT(IB_QPT_RC),
},

[IB_WR_OPFN] = {
	.length = sizeof(struct ib_atomic_wr),
	.qpt_support = BIT(IB_QPT_RC),
	.flags = RVT_OPERATION_USE_RESERVE,
},

};

static void flush_list_head(struct list_head *l)
{
	while (!list_empty(l)) {
		struct sdma_txreq *tx;

		tx = list_first_entry(
			l,
			struct sdma_txreq,
			list);
		list_del_init(&tx->list);
		hfi1_put_txreq(
			container_of(tx, struct verbs_txreq, txreq));
	}
}

static void flush_tx_list(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;

	flush_list_head(&iowait_get_ib_work(&priv->s_iowait)->tx_head);
	flush_list_head(&iowait_get_tid_work(&priv->s_iowait)->tx_head);
}

static void flush_iowait(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	unsigned long flags;
	seqlock_t *lock = priv->s_iowait.lock;

	if (!lock)
		return;
	write_seqlock_irqsave(lock, flags);
	if (!list_empty(&priv->s_iowait.list)) {
		list_del_init(&priv->s_iowait.list);
		priv->s_iowait.lock = NULL;
		rvt_put_qp(qp);
	}
	write_sequnlock_irqrestore(lock, flags);
}

static inline int opa_mtu_enum_to_int(int mtu)
{
	switch (mtu) {
	case OPA_MTU_8192:  return 8192;
	case OPA_MTU_10240: return 10240;
	default:            return -1;
	}
}

/**
 * This function is what we would push to the core layer if we wanted to be a
 * "first class citizen".  Instead we hide this here and rely on Verbs ULPs
 * to blindly pass the MTU enum value from the PathRecord to us.
 */
static inline int verbs_mtu_enum_to_int(struct ib_device *dev, enum ib_mtu mtu)
{
	int val;

	/* Constraining 10KB packets to 8KB packets */
	if (mtu == (enum ib_mtu)OPA_MTU_10240)
		mtu = OPA_MTU_8192;
	val = opa_mtu_enum_to_int((int)mtu);
	if (val > 0)
		return val;
	return ib_mtu_enum_to_int(mtu);
}

int hfi1_check_modify_qp(struct rvt_qp *qp, struct ib_qp_attr *attr,
			 int attr_mask, struct ib_udata *udata)
{
	struct ib_qp *ibqp = &qp->ibqp;
	struct hfi1_ibdev *dev = to_idev(ibqp->device);
	struct hfi1_devdata *dd = dd_from_dev(dev);
	u8 sc;

	if (attr_mask & IB_QP_AV) {
		sc = ah_to_sc(ibqp->device, &attr->ah_attr);
		if (sc == 0xf)
			return -EINVAL;

		if (!qp_to_sdma_engine(qp, sc) &&
		    dd->flags & HFI1_HAS_SEND_DMA)
			return -EINVAL;

		if (!qp_to_send_context(qp, sc))
			return -EINVAL;
	}

	if (attr_mask & IB_QP_ALT_PATH) {
		sc = ah_to_sc(ibqp->device, &attr->alt_ah_attr);
		if (sc == 0xf)
			return -EINVAL;

		if (!qp_to_sdma_engine(qp, sc) &&
		    dd->flags & HFI1_HAS_SEND_DMA)
			return -EINVAL;

		if (!qp_to_send_context(qp, sc))
			return -EINVAL;
	}

	return 0;
}

void hfi1_modify_qp(struct rvt_qp *qp, struct ib_qp_attr *attr,
		    int attr_mask, struct ib_udata *udata)
{
	struct ib_qp *ibqp = &qp->ibqp;
	struct hfi1_qp_priv *priv = qp->priv;
	unsigned long flags;

	if (attr_mask & IB_QP_AV) {
		priv->s_sc = ah_to_sc(ibqp->device, &qp->remote_ah_attr);
		priv->s_sde = qp_to_sdma_engine(qp, priv->s_sc);
		priv->s_sendcontext = qp_to_send_context(qp, priv->s_sc);
	}

	if (attr_mask & IB_QP_PATH_MIG_STATE &&
	    attr->path_mig_state == IB_MIG_MIGRATED &&
	    qp->s_mig_state == IB_MIG_ARMED) {
		qp->s_flags |= HFI1_S_AHG_CLEAR;
		priv->s_sc = ah_to_sc(ibqp->device, &qp->remote_ah_attr);
		priv->s_sde = qp_to_sdma_engine(qp, priv->s_sc);
		priv->s_sendcontext = qp_to_send_context(qp, priv->s_sc);
	}

	if (attr_mask & IB_QP_RETRY_CNT)
		priv->s_retry = attr->retry_cnt;

	spin_lock_irqsave(&priv->opfn.lock, flags);
	if (ibqp->qp_type == IB_QPT_RC && HFI1_CAP_IS_KSET(TID_RDMA)) {
		struct tid_rdma_params *local = &priv->tid_rdma.local;

		if (attr_mask & IB_QP_TIMEOUT)
			priv->tid_retry_timeout_jiffies =
				HFI1_TID_RETRY_TO_FACTOR * qp->timeout_jiffies;
		if (qp->pmtu == enum_to_mtu(OPA_MTU_4096) ||
		    qp->pmtu == enum_to_mtu(OPA_MTU_8192)) {
			local->qp =
				TID_RDMA_LOCAL_KDETH_QP_BASE | priv->rcd->ctxt;
			local->max_len = hfi1_tid_rdma_seg_max_size;
			local->jkey = priv->rcd->jkey;
			local->max_read = hfi1_tid_rdma_max_read_segs;
			local->max_write = hfi1_tid_rdma_max_write_segs;
			local->timeout = qp->timeout;

			/*
			 * We only want to set the OPFN requested bit when the
			 * QP transitions to RTS.
			 */
			if (attr_mask & IB_QP_STATE &&
			    attr->qp_state == IB_QPS_RTS) {
				priv->opfn.requested |= OPFN_MASK(TID_RDMA);
				/*
				 * If the QP is transitioning to RTS and the
				 * opfn.completed for TID RDMA has already been
				 * set, the QP is being moved *back* into RTS.
				 * We can now renegotiate the TID RDMA
				 * parameters.
				 */
				if (priv->opfn.completed &
				    OPFN_MASK(TID_RDMA)) {
					priv->opfn.completed &=
						~OPFN_MASK(TID_RDMA);
					/*
					 * Since the opfn.completed bit was
					 * already set, it is safe to assume
					 * that the opfn.extended is also set.
					 */
					opfn_schedule_conn_request(qp);
				}
			}
		} else {
			memset(local, 0, sizeof(*local));
		}
	}
	spin_unlock_irqrestore(&priv->opfn.lock, flags);
}

/**
 * hfi1_setup_wqe - setup the wqe
 * @qp - The qp
 * @wqe - The built wqe
 *
 * Perform setup of the wqe.  This is called
 * prior to inserting the wqe into the ring but after
 * the wqe has been setup by RDMAVT. This function
 * allow the driver the opportunity to perform
 * validation and additional setup of the wqe.
 *
 * Returns 0 on success, -EINVAL on failure
 *
 */
int hfi1_setup_wqe(struct rvt_qp *qp,
		   struct rvt_swqe *wqe)
{
	struct hfi1_ibport *ibp = to_iport(qp->ibqp.device, qp->port_num);
	struct rvt_ah *ah;

	switch (qp->ibqp.qp_type) {
	case IB_QPT_RC:
		setup_tid_rdma_wqe(qp, wqe);
	case IB_QPT_UC:
		if (wqe->length > 0x80000000U)
			return -EINVAL;
		break;
	case IB_QPT_SMI:
		ah = ibah_to_rvtah(wqe->ud_wr.ah);
		if (wqe->length > (1 << ah->log_pmtu))
			return -EINVAL;
		break;
	case IB_QPT_GSI:
	case IB_QPT_UD:
		ah = ibah_to_rvtah(wqe->ud_wr.ah);
		if (wqe->length > (1 << ah->log_pmtu))
			return -EINVAL;
		if (ibp->sl_to_sc[ah->attr.sl] == 0xf)
			return -EINVAL;
	default:
		break;
	}
	return wqe->length <= piothreshold;
}

/**
 * _hfi1_schedule_send - schedule progress
 * @qp: the QP
 *
 * This schedules qp progress w/o regard to the s_flags.
 *
 * It is only used in the post send, which doesn't hold
 * the s_lock.
 */
bool _hfi1_schedule_send(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_ibport *ibp =
		to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_pportdata *ppd = ppd_from_ibp(ibp);
	struct hfi1_devdata *dd = dd_from_ibdev(qp->ibqp.device);

	return iowait_schedule(&priv->s_iowait, ppd->hfi1_wq,
			       priv->s_sde ?
			       priv->s_sde->cpu :
			       cpumask_first(cpumask_of_node(dd->node)));
}

bool _hfi1_schedule_tid_send(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_ibport *ibp =
		to_iport(qp->ibqp.device, qp->port_num);
	struct hfi1_pportdata *ppd = ppd_from_ibp(ibp);
	struct hfi1_devdata *dd = dd_from_ibdev(qp->ibqp.device);

	return iowait_tid_schedule(&priv->s_iowait, ppd->hfi1_wq,
				   priv->s_sde ?
				   priv->s_sde->cpu :
				   cpumask_first(cpumask_of_node(dd->node)));
}

static void qp_pio_drain(struct rvt_qp *qp)
{
	struct hfi1_ibdev *dev;
	struct hfi1_qp_priv *priv = qp->priv;

	if (!priv->s_sendcontext)
		return;
	dev = to_idev(qp->ibqp.device);
	while (iowait_pio_pending(&priv->s_iowait)) {
		write_seqlock_irq(&dev->iowait_lock);
		hfi1_sc_wantpiobuf_intr(priv->s_sendcontext, 1);
		write_sequnlock_irq(&dev->iowait_lock);
		iowait_pio_drain(&priv->s_iowait);
		write_seqlock_irq(&dev->iowait_lock);
		hfi1_sc_wantpiobuf_intr(priv->s_sendcontext, 0);
		write_sequnlock_irq(&dev->iowait_lock);
	}
}

/**
 * hfi1_schedule_send - schedule progress
 * @qp: the QP
 *
 * This schedules qp progress and caller should hold
 * the s_lock.
 * @return true if the second leg is scheduled;
 * false if the second leg is not scheduled.
 */
bool hfi1_schedule_send(struct rvt_qp *qp)
{
	lockdep_assert_held(&qp->s_lock);
	if (hfi1_send_ok(qp)) {
		_hfi1_schedule_send(qp);
		return true;
	}
	if (qp->s_flags & HFI1_S_ANY_WAIT_IO)
		iowait_set_flag(
			&((struct hfi1_qp_priv *)qp->priv)->s_iowait,
			IOWAIT_PENDING_IB);
	return false;
}

/**
 * hfi1_schedule_tid_send - schedule progress on TID RDMA state machine
 * @qp: the QP
 *
 * This schedules qp progress on the TID RDMA state machine. Caller
 * should hold the s_lock.
 * Unlike hfi1_schedule_send(), this cannot use hfi1_send_ok() because
 * the two state machines can step on each other with respect to the
 * RVT_S_BUSY flag.
 * Therefore, a modified test is used.
 * @return true if the second leg is scheduled;
 *  false if the second leg is not scheduled.
 */
bool hfi1_schedule_tid_send(struct rvt_qp *qp)
{
	lockdep_assert_held(&qp->s_lock);
	if (hfi1_send_tid_ok(qp)) {
		/*
		 * The following call returns true if the qp is not on the
		 * queue and false if the qp is already on the queue before
		 * this call. Either way, the qp will be on the queue when the
		 * call returns.
		 */
		_hfi1_schedule_tid_send(qp);
		return true;
	}
	if (qp->s_flags & HFI1_S_ANY_WAIT_IO)
		iowait_set_flag(
			&((struct hfi1_qp_priv *)qp->priv)->s_iowait,
			IOWAIT_PENDING_TID);
	return false;
}

void hfi1_qp_schedule(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	bool ret;

	if (iowait_flag_set(&priv->s_iowait, IOWAIT_PENDING_IB)) {
		ret = hfi1_schedule_send(qp);
		if (ret)
			iowait_clear_flag(&priv->s_iowait, IOWAIT_PENDING_IB);
	}
	if (iowait_flag_set(&priv->s_iowait, IOWAIT_PENDING_TID)) {
		ret = hfi1_schedule_tid_send(qp);
		if (ret)
			iowait_clear_flag(&priv->s_iowait, IOWAIT_PENDING_TID);
	}
}

void hfi1_qp_wakeup(struct rvt_qp *qp, u32 flag)
{
	unsigned long flags;

	spin_lock_irqsave(&qp->s_lock, flags);
	if (qp->s_flags & flag) {
		qp->s_flags &= ~flag;
		trace_hfi1_qpwakeup(qp, flag);
		hfi1_qp_schedule(qp);
	}
	spin_unlock_irqrestore(&qp->s_lock, flags);
	/* Notify hfi1_destroy_qp() if it is waiting. */
	rvt_put_qp(qp);
}

void hfi1_qp_unbusy(struct rvt_qp *qp, struct iowait_work *wait)
{
	struct hfi1_qp_priv *priv = qp->priv;

	if (iowait_set_work_flag(wait) == IOWAIT_IB_SE) {
		qp->s_flags &= ~RVT_S_BUSY;
		/*
		 * If we are sending a first-leg packet from the second leg,
		 * we need to clear the busy flag from priv->s_flags to
		 * avoid a race condition when the qp wakes up before
		 * the call to hfi1_verbs_send() returns to the second
		 * leg. In that case, the second leg will terminate without
		 * being re-scheduled, resulting in failure to send TID RDMA
		 * WRITE DATA and TID RDMA ACK packets.
		 */
		if (priv->s_flags & HFI1_S_TID_BUSY_SET) {
			priv->s_flags &= ~(HFI1_S_TID_BUSY_SET |
					   RVT_S_BUSY);
			iowait_set_flag(&priv->s_iowait, IOWAIT_PENDING_TID);
		}
	} else {
		priv->s_flags &= ~RVT_S_BUSY;
	}
}

static int iowait_sleep(
	struct sdma_engine *sde,
	struct iowait_work *wait,
	struct sdma_txreq *stx,
	uint seq,
	bool pkts_sent)
{
	struct verbs_txreq *tx = container_of(stx, struct verbs_txreq, txreq);
	struct rvt_qp *qp;
	struct hfi1_qp_priv *priv;
	unsigned long flags;
	int ret = 0;
	struct hfi1_ibdev *dev;

	qp = tx->qp;
	priv = qp->priv;

	spin_lock_irqsave(&qp->s_lock, flags);
	if (ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK) {
		/*
		 * If we couldn't queue the DMA request, save the info
		 * and try again later rather than destroying the
		 * buffer and undoing the side effects of the copy.
		 */
		/* Make a common routine? */
		dev = &sde->dd->verbs_dev;
		list_add_tail(&stx->list, &wait->tx_head);
		write_seqlock(&dev->iowait_lock);
		if (sdma_progress(sde, seq, stx))
			goto eagain;
		if (list_empty(&priv->s_iowait.list)) {
			struct hfi1_ibport *ibp =
				to_iport(qp->ibqp.device, qp->port_num);

			ibp->rvp.n_dmawait++;
			qp->s_flags |= RVT_S_WAIT_DMA_DESC;
			iowait_queue(pkts_sent, &priv->s_iowait,
				     &sde->dmawait);
			priv->s_iowait.lock = &dev->iowait_lock;
			trace_hfi1_qpsleep(qp, RVT_S_WAIT_DMA_DESC);
			rvt_get_qp(qp);
		}
		write_sequnlock(&dev->iowait_lock);
		hfi1_qp_unbusy(qp, wait);
		spin_unlock_irqrestore(&qp->s_lock, flags);
		ret = -EBUSY;
	} else {
		spin_unlock_irqrestore(&qp->s_lock, flags);
		hfi1_put_txreq(tx);
	}
	return ret;
eagain:
	write_sequnlock(&dev->iowait_lock);
	spin_unlock_irqrestore(&qp->s_lock, flags);
	list_del_init(&stx->list);
	return -EAGAIN;
}

static void iowait_wakeup(struct iowait *wait, int reason)
{
	struct rvt_qp *qp = iowait_to_qp(wait);

	WARN_ON(reason != SDMA_AVAIL_REASON);
	hfi1_qp_wakeup(qp, RVT_S_WAIT_DMA_DESC);
}

static void iowait_sdma_drained(struct iowait *wait)
{
	struct rvt_qp *qp = iowait_to_qp(wait);
	unsigned long flags;

	/*
	 * This happens when the send engine notes
	 * a QP in the error state and cannot
	 * do the flush work until that QP's
	 * sdma work has finished.
	 */
	spin_lock_irqsave(&qp->s_lock, flags);
	if (qp->s_flags & RVT_S_WAIT_DMA) {
		qp->s_flags &= ~RVT_S_WAIT_DMA;
		hfi1_schedule_send(qp);
	}
	spin_unlock_irqrestore(&qp->s_lock, flags);
}

/**
 *
 * qp_to_sdma_engine - map a qp to a send engine
 * @qp: the QP
 * @sc5: the 5 bit sc
 *
 * Return:
 * A send engine for the qp or NULL for SMI type qp.
 */
struct sdma_engine *qp_to_sdma_engine(struct rvt_qp *qp, u8 sc5)
{
	struct hfi1_devdata *dd = dd_from_ibdev(qp->ibqp.device);
	struct sdma_engine *sde;

	if (!(dd->flags & HFI1_HAS_SEND_DMA))
		return NULL;
	switch (qp->ibqp.qp_type) {
	case IB_QPT_SMI:
		return NULL;
	default:
		break;
	}
	sde = sdma_select_engine_sc(dd, qp->ibqp.qp_num >> dd->qos_shift, sc5);
	return sde;
}

/*
 * qp_to_send_context - map a qp to a send context
 * @qp: the QP
 * @sc5: the 5 bit sc
 *
 * Return:
 * A send context for the qp
 */
struct send_context *qp_to_send_context(struct rvt_qp *qp, u8 sc5)
{
	struct hfi1_devdata *dd = dd_from_ibdev(qp->ibqp.device);

	switch (qp->ibqp.qp_type) {
	case IB_QPT_SMI:
		/* SMA packets to VL15 */
		return dd->vld[15].sc;
	default:
		break;
	}

	return pio_select_send_context_sc(dd, qp->ibqp.qp_num >> dd->qos_shift,
					  sc5);
}

struct qp_iter {
	struct hfi1_ibdev *dev;
	struct rvt_qp *qp;
	int specials;
	int n;
};

struct qp_iter *qp_iter_init(struct hfi1_ibdev *dev)
{
	struct qp_iter *iter;

	iter = kzalloc(sizeof(*iter), GFP_KERNEL);
	if (!iter)
		return NULL;

	iter->dev = dev;
	iter->specials = dev->rdi.ibdev.phys_port_cnt * 2;

	return iter;
}

int qp_iter_next(struct qp_iter *iter)
{
	struct hfi1_ibdev *dev = iter->dev;
	int n = iter->n;
	int ret = 1;
	struct rvt_qp *pqp = iter->qp;
	struct rvt_qp *qp;

	/*
	 * The approach is to consider the special qps
	 * as an additional table entries before the
	 * real hash table.  Since the qp code sets
	 * the qp->next hash link to NULL, this works just fine.
	 *
	 * iter->specials is 2 * # ports
	 *
	 * n = 0..iter->specials is the special qp indices
	 *
	 * n = iter->specials..dev->rdi.qp_dev->qp_table_size+iter->specials are
	 * the potential hash bucket entries
	 *
	 */
	for (; n <  dev->rdi.qp_dev->qp_table_size + iter->specials; n++) {
		if (pqp) {
			qp = rcu_dereference(pqp->next);
		} else {
			if (n < iter->specials) {
				struct hfi1_pportdata *ppd;
				struct hfi1_ibport *ibp;
				int pidx;

				pidx = n % dev->rdi.ibdev.phys_port_cnt;
				ppd = &dd_from_dev(dev)->pport[pidx];
				ibp = &ppd->ibport_data;

				if (!(n & 1))
					qp = rcu_dereference(ibp->rvp.qp[0]);
				else
					qp = rcu_dereference(ibp->rvp.qp[1]);
			} else {
				qp = rcu_dereference(
					dev->rdi.qp_dev->qp_table[
						(n - iter->specials)]);
			}
		}
		pqp = qp;
		if (qp) {
			iter->qp = qp;
			iter->n = n;
			return 0;
		}
	}
	return ret;
}

static const char * const qp_type_str[] = {
	"SMI", "GSI", "RC", "UC", "UD",
};

static int qp_idle(struct rvt_qp *qp)
{
	return
		qp->s_last == qp->s_acked &&
		qp->s_acked == qp->s_cur &&
		qp->s_cur == qp->s_tail &&
		qp->s_tail == qp->s_head;
}

static void qp_tid_wqe_print(struct seq_file *s, const char *header,
		      struct rvt_qp *qp, u32 i)
{
	struct rvt_swqe *wqe;
	struct tid_rdma_request *req;

	if (i != HFI1_QP_WQE_INVALID) {
		wqe = rvt_get_swqe_ptr(qp, i);
		req = wqe_to_tid_req(wqe);
		seq_printf(s,
			   "\t\t%s: op 0x%x psn 0x%x lpsn 0x%x s 0x%x %u %u %u %u / %u %u %u\n",
			   header, wqe->wr.opcode, wqe->psn, wqe->lpsn,
			   req->state, req->total_segs, req->cur_seg,
			   req->comp_seg, req->ack_seg, req->setup_head,
			   req->clear_tail, req->acked_tail);
	}
}

static void qp_tid_ack_print(struct seq_file *s, const char *header,
		      struct rvt_qp *qp, u32 i)
{
	struct rvt_ack_entry *e;
	struct tid_rdma_request *req;

	if (i != HFI1_QP_WQE_INVALID) {
		e = &qp->s_ack_queue[i];
		req = ack_to_tid_req(e);
		seq_printf(s,
			   "\t\t%s: op 0x%x psn 0x%x lpsn 0x%x s 0x%x %u %u %u %u / %u %u %u\n",
			   header, e->opcode, e->psn, e->lpsn, req->state,
			   req->total_segs, req->cur_seg, req->comp_seg,
			   req->ack_seg, req->setup_head, req->clear_tail,
			   req->acked_tail);
	}
}

void qp_iter_print(struct seq_file *s, struct qp_iter *iter)
{
	struct rvt_swqe *wqe;
	struct rvt_qp *qp = iter->qp;
	struct hfi1_qp_priv *priv = qp->priv;
	struct sdma_engine *sde;
	struct send_context *send_context;

	sde = qp_to_sdma_engine(qp, priv->s_sc);
	wqe = rvt_get_swqe_ptr(qp, qp->s_last);
	send_context = qp_to_send_context(qp, priv->s_sc);
	seq_printf(s,
		   "N %d %s QP %x R %u %s %u %u W %x %u %x %x f=%x pf=%lx %u %u %u %u %u %u %u %u %u SPSN %x %x %x %x %x RPSN %x S(%u %u %u %u %u %u %u) R(%u %u %u %u) RQP %x LID %x SL %u MTU %u %u %u %u %u SDE %p,%u SC %p,%u SCQ %u %u PID %d RCD %u %x %x\n",
		   iter->n,
		   qp_idle(qp) ? "I" : "B",
		   qp->ibqp.qp_num,
		   atomic_read(&qp->refcount),
		   qp_type_str[qp->ibqp.qp_type],
		   qp->state,
		   qp->s_hdrwords,
		   /* current wqe fields */
		   wqe ? wqe->wr.opcode : 0,
		   wqe ? wqe->ssn : 0,
		   wqe ? wqe->psn : 0,
		   wqe ? wqe->lpsn : 0,
		   /* send flags */
		   qp->s_flags,
		   priv->s_iowait.flags,
		   iowait_sdma_pending(&priv->s_iowait),
		   iowait_pio_pending(&priv->s_iowait),
		   !list_empty(&priv->s_iowait.list),
		   !list_empty(&iowait_get_ib_work(&priv->s_iowait)->tx_head),
		   !list_empty(&iowait_get_tid_work(&priv->s_iowait)->tx_head),
		   qp->timeout,
		   /* acks pending egress */
		   qp->s_rdma_ack_cnt,
		   /* request sequence numbers */
		   qp->s_lsn,
		   qp->s_ssn,
		   /* send packet sequence numbers */
		   qp->s_last_psn,
		   qp->s_psn, qp->s_next_psn,
		   qp->s_sending_psn, qp->s_sending_hpsn,
		   /* receive packet sequence numbers */
		   qp->r_psn,
		   /* swqe ring pointers, sizes */
		   qp->s_last, qp->s_acked, qp->s_cur,
		   qp->s_tail, qp->s_head, qp->s_size,
		   qp->s_avail,
		   /* ack_queue ring pointers, size */
		   qp->s_acked_ack_queue, qp->s_tail_ack_queue,
		   qp->r_head_ack_queue,
		   rvt_size_atomic(ib_to_rvt(qp->ibqp.device)),
		   /* remote QP info  */
		   qp->remote_qpn,
		   qp->remote_ah_attr.dlid,
		   qp->remote_ah_attr.sl,
		   qp->pmtu,
		   qp->s_retry,
		   qp->s_retry_cnt,
		   qp->s_rnr_retry_cnt,
		   qp->s_rnr_retry,
		   sde,
		   sde ? sde->this_idx : 0,
		   send_context,
		   send_context ? send_context->sw_index : 0,
		   ibcq_to_rvtcq(qp->ibqp.send_cq)->queue->head,
		   ibcq_to_rvtcq(qp->ibqp.send_cq)->queue->tail,
		   qp->pid,
		   priv->rcd ? priv->rcd->ctxt : 0,
		   qp->s_state,
		   qp->s_ack_state);

	if (qp->ibqp.qp_type == IB_QPT_RC && HFI1_CAP_IS_KSET(TID_RDMA)) {
		seq_printf(s,
			   "\tTID RDMA f 0x%x 0x%x h %u c %u t %u h %u t %u a %u\n",
			   priv->s_flags,
			   priv->s_state,
			   priv->s_tid_head,
			   priv->s_tid_cur,
			   priv->s_tid_tail,
			   priv->r_tid_head,
			   priv->r_tid_tail,
			   priv->r_tid_ack);
		qp_tid_wqe_print(s, "s_tid_head", qp, priv->s_tid_head);
		qp_tid_wqe_print(s, "s_tid_cur", qp, priv->s_tid_cur);
		qp_tid_wqe_print(s, "s_tid_tail", qp, priv->s_tid_tail);
		qp_tid_ack_print(s, "r_tid_head", qp, priv->r_tid_head);
		qp_tid_ack_print(s, "r_tid_tail", qp, priv->r_tid_tail);
		qp_tid_ack_print(s, "r_tid_ack", qp, priv->r_tid_ack);
	}

}

void *qp_priv_alloc(struct rvt_dev_info *rdi, struct rvt_qp *qp,
		    gfp_t gfp)
{
	struct hfi1_qp_priv *priv;

	priv = kzalloc_node(sizeof(*priv), gfp, rdi->dparms.node);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->owner = qp;

	priv->s_ahg = kzalloc_node(sizeof(*priv->s_ahg), gfp,
				   rdi->dparms.node);
	if (!priv->s_ahg) {
		kfree(priv);
		return ERR_PTR(-ENOMEM);
	}
	iowait_init(
		&priv->s_iowait,
		1,
		_hfi1_do_send,
		_hfi1_do_tid_send,
		iowait_sleep,
		iowait_wakeup,
		iowait_sdma_drained);
	setup_timer(&priv->s_tid_timer, hfi1_tid_timeout, (unsigned long)qp);
	setup_timer(&priv->s_tid_retry_timer, hfi1_tid_retry_timeout,
		    (unsigned long)qp);
	INIT_LIST_HEAD(&priv->tid_wait);
	return priv;
}

int qp_priv_init(struct rvt_dev_info *rdi, struct rvt_qp *qp,
		 struct ib_qp_init_attr *init_attr, gfp_t gfp)
{
	struct hfi1_qp_priv *qpriv = qp->priv;
	int i, ret;

	qpriv->rcd = qp_to_rcd(rdi, qp);
	spin_lock_init(&qpriv->opfn.lock);
	INIT_WORK(&qpriv->opfn.opfn_work, opfn_send_conn_request);
	INIT_WORK(&qpriv->tid_rdma.trigger_work, tid_rdma_trigger_resume);
	qpriv->r_tid_tail = qp->s_tail_ack_queue;
	qpriv->flow_state.psn = 0;
	qpriv->flow_state.index = RXE_NUM_TID_FLOWS;
	qpriv->flow_state.last_index = RXE_NUM_TID_FLOWS;
	qpriv->flow_state.generation = 0;
	qpriv->s_state = TID_OP(WRITE_RESP);
	qpriv->s_tid_cur = HFI1_QP_WQE_INVALID;
	qpriv->s_tid_tail = HFI1_QP_WQE_INVALID;
	qpriv->r_tid_tail = HFI1_QP_WQE_INVALID;
	qpriv->r_tid_ack = HFI1_QP_WQE_INVALID;
	atomic_set(&qpriv->n_requests, 0);
	atomic_set(&qpriv->n_tid_requests, 0);

	if (init_attr->qp_type == IB_QPT_RC && HFI1_CAP_IS_KSET(TID_RDMA)) {
		for (i = 0; i < qp->s_size; i++) {
			struct hfi1_swqe_priv *priv;
			struct rvt_swqe *wqe = rvt_get_swqe_ptr(qp, i);

			priv = kzalloc(sizeof(*priv), gfp);
			if (!priv)
				return -ENOMEM;

			ret = hfi1_kern_exp_rcv_alloc_flows(&priv->tid_req,
							    gfp);
			if (ret)
				return ret;

			/*
			 * Initialize various TID RDMA request variables.
			 * These variables are "static", which is why they
			 * can be pre-initialized here before the WRs has
			 * even been submitted.
			 * However, non-NULL values for these variables do not
			 * imply that this WQE has been enabled for TID RDMA.
			 * Drivers should check the WQE's opcode to determine
			 * if a request is a TID RDMA one or not.
			 */
			priv->tid_req.qp = qp;
			priv->tid_req.rcd = qpriv->rcd;
			priv->tid_req.e.swqe = wqe;
			wqe->priv = priv;
		}
		for (i = 0; i < rvt_max_atomic(rdi); i++) {
			struct hfi1_ack_priv *priv;

			priv = kzalloc(sizeof(*priv), gfp);
			if (!priv)
				return -ENOMEM;

			ret = hfi1_kern_exp_rcv_alloc_flows(&priv->tid_req,
							    gfp);
			if (ret)
				return ret;

			priv->tid_req.qp = qp;
			priv->tid_req.rcd = qpriv->rcd;
			priv->tid_req.e.ack = &qp->s_ack_queue[i];
			qp->s_ack_queue[i].priv = priv;
		}
	}
	return 0;
}

void qp_priv_free(struct rvt_dev_info *rdi, struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct rvt_swqe *wqe;
	u32 i;

	lockdep_assert_held(&qp->s_lock);
	if (qp->ibqp.qp_type == IB_QPT_RC && HFI1_CAP_IS_KSET(TID_RDMA)) {
		for (i = 0; i < qp->s_size; i++) {
			struct hfi1_swqe_priv *priv;

			wqe = rvt_get_swqe_ptr(qp, i);
			priv = wqe->priv;
			if (priv)
				hfi1_kern_exp_rcv_free_flows(&priv->tid_req);
			kfree(priv);
			wqe->priv = NULL;
		}
		for (i = 0; i < rvt_max_atomic(rdi); i++) {
			struct hfi1_ack_priv *priv = qp->s_ack_queue[i].priv;

			if (priv)
				hfi1_kern_exp_rcv_free_flows(&priv->tid_req);
			kfree(priv);
			qp->s_ack_queue[i].priv = NULL;
		}
	}
	cancel_work_sync(&priv->opfn.opfn_work);
	kfree(priv->s_ahg);
	kfree(priv);
}

unsigned free_all_qps(struct rvt_dev_info *rdi)
{
	struct hfi1_ibdev *verbs_dev = container_of(rdi,
						    struct hfi1_ibdev,
						    rdi);
	struct hfi1_devdata *dd = container_of(verbs_dev,
					       struct hfi1_devdata,
					       verbs_dev);
	int n;
	unsigned qp_inuse = 0;

	for (n = 0; n < dd->num_pports; n++) {
		struct hfi1_ibport *ibp = &dd->pport[n].ibport_data;

		rcu_read_lock();
		if (rcu_dereference(ibp->rvp.qp[0]))
			qp_inuse++;
		if (rcu_dereference(ibp->rvp.qp[1]))
			qp_inuse++;
		rcu_read_unlock();
	}

	return qp_inuse;
}

void flush_qp_waiters(struct rvt_qp *qp)
{
	lockdep_assert_held(&qp->s_lock);
	flush_iowait(qp);
	tid_rdma_flush_wait(qp);
}

void stop_send_queue(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;

	iowait_cancel_work(&priv->s_iowait);
	if (cancel_work_sync(&priv->tid_rdma.trigger_work))
		rvt_put_qp(qp);
}

void quiesce_qp(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;

	hfi1_del_tid_reap_timer(qp);
	hfi1_del_tid_retry_timer(qp);
	iowait_sdma_drain(&priv->s_iowait);
	qp_pio_drain(qp);
	flush_tx_list(qp);
}

static void kern_exp_rcv_clear_all(struct rvt_qp *qp)
{
	int i, ret;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_flow_state *fs;

	if (qp->ibqp.qp_type != IB_QPT_RC || !HFI1_CAP_IS_KSET(TID_RDMA))
		return;

	/*
	 * First, clear the flow to help prevent any delayed packets from
	 * being delivered.
	 */
	if (qpriv) {
		fs = &qpriv->flow_state;
		if (fs->index != RXE_NUM_TID_FLOWS)
			hfi1_kern_clear_hw_flow(qpriv->rcd, qp);
	}
	for (i = 0; i < qp->s_size; i++) {
		struct rvt_swqe *wqe = rvt_get_swqe_ptr(qp, i);
		struct hfi1_swqe_priv *priv = wqe->priv;

		if (!priv)
			continue;
		do {
			ret = hfi1_kern_exp_rcv_clear(&priv->tid_req);
		} while (!ret);
	}
	for (i = 0; i < rvt_max_atomic(ib_to_rvt(qp->ibqp.device)); i++) {
		struct hfi1_ack_priv *priv = qp->s_ack_queue[i].priv;

		if (!priv)
			continue;
		do {
			ret = hfi1_kern_exp_rcv_clear(&priv->tid_req);
		} while (!ret);
	}
}

void notify_qp_reset(struct rvt_qp *qp)
{
	kern_exp_rcv_clear_all(qp);
	qp->r_adefered = 0;
	clear_ahg(qp);

	/* Clear any OPFN state */
	if (qp->ibqp.qp_type == IB_QPT_RC)
		opfn_conn_error(qp);
}

/*
 * Switch to alternate path.
 * The QP s_lock should be held and interrupts disabled.
 */
void hfi1_migrate_qp(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct ib_event ev;

	qp->s_mig_state = IB_MIG_MIGRATED;
	qp->remote_ah_attr = qp->alt_ah_attr;
	qp->port_num = qp->alt_ah_attr.port_num;
	qp->s_pkey_index = qp->s_alt_pkey_index;
	qp->s_flags |= HFI1_S_AHG_CLEAR;
	priv->s_sc = ah_to_sc(qp->ibqp.device, &qp->remote_ah_attr);
	priv->s_sde = qp_to_sdma_engine(qp, priv->s_sc);

	ev.device = qp->ibqp.device;
	ev.element.qp = &qp->ibqp;
	ev.event = IB_EVENT_PATH_MIG;
	qp->ibqp.event_handler(&ev, qp->ibqp.qp_context);
}

int mtu_to_path_mtu(u32 mtu)
{
	return mtu_to_enum(mtu, OPA_MTU_8192);
}

u32 mtu_from_qp(struct rvt_dev_info *rdi, struct rvt_qp *qp, u32 pmtu)
{
	u32 mtu;
	struct hfi1_ibdev *verbs_dev = container_of(rdi,
						    struct hfi1_ibdev,
						    rdi);
	struct hfi1_devdata *dd = container_of(verbs_dev,
					       struct hfi1_devdata,
					       verbs_dev);
	struct hfi1_ibport *ibp;
	u8 sc, vl;

	ibp = &dd->pport[qp->port_num - 1].ibport_data;
	sc = ibp->sl_to_sc[qp->remote_ah_attr.sl];
	vl = sc_to_vlt(dd, sc);

	mtu = verbs_mtu_enum_to_int(qp->ibqp.device, pmtu);
	if (vl < PER_VL_SEND_CONTEXTS)
		mtu = min_t(u32, mtu, dd->vld[vl].mtu);
	return mtu;
}

int get_pmtu_from_attr(struct rvt_dev_info *rdi, struct rvt_qp *qp,
		       struct ib_qp_attr *attr)
{
	int mtu, pidx = qp->port_num - 1;
	struct hfi1_ibdev *verbs_dev = container_of(rdi,
						    struct hfi1_ibdev,
						    rdi);
	struct hfi1_devdata *dd = container_of(verbs_dev,
					       struct hfi1_devdata,
					       verbs_dev);
	mtu = verbs_mtu_enum_to_int(qp->ibqp.device, attr->path_mtu);
	if (mtu == -1)
		return -1; /* values less than 0 are error */

	if (mtu > dd->pport[pidx].ibmtu)
		return mtu_to_enum(dd->pport[pidx].ibmtu, IB_MTU_2048);
	else
		return attr->path_mtu;
}

void notify_error_qp(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	seqlock_t *lock = priv->s_iowait.lock;

	if (lock) {
		write_seqlock(lock);
		if (!list_empty(&priv->s_iowait.list) &&
		    !(qp->s_flags & RVT_S_BUSY) &&
		    !(priv->s_flags & RVT_S_BUSY)) {
			qp->s_flags &= ~RVT_S_ANY_WAIT_IO;
			list_del_init(&priv->s_iowait.list);
			priv->s_iowait.lock = NULL;
			rvt_put_qp(qp);
		}
		write_sequnlock(lock);
	}

	if (!(qp->s_flags & RVT_S_BUSY) && !(priv->s_flags & RVT_S_BUSY)) {
		if (qp->s_rdma_mr) {
			rvt_put_mr(qp->s_rdma_mr);
			qp->s_rdma_mr = NULL;
		}
		flush_tx_list(qp);
	}
}

/**
 * hfi1_error_port_qps - put a port's RC/UC qps into error state
 * @ibp: the ibport.
 * @sl: the service level.
 *
 * This function places all RC/UC qps with a given service level into error
 * state. It is generally called to force upper lay apps to abandon stale qps
 * after an sl->sc mapping change.
 */
void hfi1_error_port_qps(struct hfi1_ibport *ibp, u8 sl)
{
	struct rvt_qp *qp = NULL;
	struct hfi1_pportdata *ppd = ppd_from_ibp(ibp);
	struct hfi1_ibdev *dev = &ppd->dd->verbs_dev;
	int n;
	int lastwqe;
	struct ib_event ev;

	rcu_read_lock();

	/* Deal only with RC/UC qps that use the given SL. */
	for (n = 0; n < dev->rdi.qp_dev->qp_table_size; n++) {
		for (qp = rcu_dereference(dev->rdi.qp_dev->qp_table[n]); qp;
			qp = rcu_dereference(qp->next)) {
			if (qp->port_num == ppd->port &&
			    (qp->ibqp.qp_type == IB_QPT_UC ||
			     qp->ibqp.qp_type == IB_QPT_RC) &&
			    qp->remote_ah_attr.sl == sl &&
			    (ib_rvt_state_ops[qp->state] &
			     RVT_POST_SEND_OK)) {
				spin_lock_irq(&qp->r_lock);
				spin_lock(&qp->s_hlock);
				spin_lock(&qp->s_lock);
				lastwqe = rvt_error_qp(qp,
						       IB_WC_WR_FLUSH_ERR);
				spin_unlock(&qp->s_lock);
				spin_unlock(&qp->s_hlock);
				spin_unlock_irq(&qp->r_lock);
				if (lastwqe) {
					ev.device = qp->ibqp.device;
					ev.element.qp = &qp->ibqp;
					ev.event =
						IB_EVENT_QP_LAST_WQE_REACHED;
					qp->ibqp.event_handler(&ev,
						qp->ibqp.qp_context);
				}
			}
		}
	}

	rcu_read_unlock();
}

/**
 * qp_to_rcd - determine the receive context used by a qp
 * @qp - the qp
 *
 * This routine returns the receive context associated
 * with a a qp's qpn.
 *
 * Returns the context on success, NULL otherwise.
 */
static struct hfi1_ctxtdata *qp_to_rcd(
	struct rvt_dev_info *rdi,
	struct rvt_qp *qp)
{
	struct hfi1_ibdev *verbs_dev = container_of(rdi,
						    struct hfi1_ibdev,
						    rdi);
	struct hfi1_devdata *dd = container_of(verbs_dev,
					       struct hfi1_devdata,
					       verbs_dev);
	unsigned ctxt;

	if (qp->ibqp.qp_num == 0) {
		ctxt = 0;
	} else {
		ctxt = ((qp->ibqp.qp_num >> dd->qos_shift) %
			(dd->n_krcv_queues - 1)) + 1;

		if (ctxt > dd->num_rcv_contexts)
			return NULL;
	}

	return dd->rcd[ctxt];
}

/* Does @sge meet the alignment requirements for tid rdma? */
static inline bool hfi1_check_sge_align(struct rvt_sge *sge, int num_sge)
{
	int i;

	for (i = 0; i < num_sge; i++, sge++)
		if ((u64)sge->vaddr & ~PAGE_MASK ||
		    sge->sge_length & ~PAGE_MASK)
			return false;
	return true;
}

static void setup_tid_rdma_wqe(struct rvt_qp *qp, struct rvt_swqe *wqe)
{
	if (wqe->wr.opcode == IB_WR_RDMA_READ ||
	    wqe->wr.opcode == IB_WR_RDMA_WRITE) {
		struct hfi1_qp_priv *qpriv =
			(struct hfi1_qp_priv *)qp->priv;
		struct hfi1_swqe_priv *priv = wqe->priv;
		struct tid_rdma_params *remote;
		bool do_tid_rdma = false;

		rcu_read_lock();
		remote = rcu_dereference(qpriv->tid_rdma.remote);
		/*
		 * If TID RDMA is disabled by the negotiation, don't
		 * use it.
		 */
		if (!remote)
			goto exit;

		if (wqe->wr.opcode == IB_WR_RDMA_READ &&
		    (hfi1_tid_rdma_proto_enable &
		     HFI1_TID_RDMA_PROTO_EN_READ)) {
			if (wqe->length >= hfi1_tid_rdma_seg_min_size &&
			    hfi1_check_sge_align(&wqe->sg_list[0],
						 wqe->wr.num_sge) &&
			    (qpriv->tid_rdma.n_read < remote->max_read)) {
				wqe->wr.opcode = IB_WR_TID_RDMA_READ;
				do_tid_rdma = true;
				priv->tid_req.n_flows = remote->max_read;
				qpriv->tid_r_reqs++;
			}
		} else if (wqe->wr.opcode == IB_WR_RDMA_WRITE &&
			   (hfi1_tid_rdma_proto_enable &
			    HFI1_TID_RDMA_PROTO_EN_WRITE)) {
			/*
			 * TID RDMA is enabled for this RDMA WRITE
			 * request iff:
			 *   1. The remote address is page-aligned,
			 *   2. The length is larger than the minimum
			 *      segment size,
			 *   3. The length is page-multiple, and
			 *   4. There are available TID RDMA WRITEs
			 *      on the remote end.
			 */
			if (!(wqe->rdma_wr.remote_addr & ~PAGE_MASK) &&
			    (wqe->length >= hfi1_tid_rdma_seg_min_size) &&
			    !(wqe->length & ~PAGE_MASK) &&
			    (qpriv->tid_rdma.n_write < remote->max_write)) {
				wqe->wr.opcode = IB_WR_TID_RDMA_WRITE;
				do_tid_rdma = true;
			}
		}

		if (do_tid_rdma) {
			priv->tid_req.seg_len =
				min_t(u32, remote->max_len, wqe->length);
			priv->tid_req.total_segs =
				DIV_ROUND_UP(wqe->length,
					     priv->tid_req.seg_len);
			/* Compute the last PSN of the request */
			wqe->lpsn = wqe->psn;
			if (wqe->wr.opcode == IB_WR_TID_RDMA_READ) {
				wqe->lpsn +=
					rvt_div_round_up_mtu(qp, wqe->length)
					- 1;
			} else {
				wqe->lpsn += priv->tid_req.total_segs - 1;
				atomic_inc(&qpriv->n_requests);
			}
#if 0
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA, "Setting up TID RDMA Request: psn 0x%x lpsn 0x%x seg_len %u total_segs %u",
				  wqe->psn, wqe->lpsn, priv->tid_req.seg_len,
				  priv->tid_req.total_segs);
#endif
#endif

			priv->tid_req.cur_seg = 0;
			priv->tid_req.comp_seg = 0;
			priv->tid_req.ack_seg = 0;
			priv->tid_req.state = TID_REQUEST_INACTIVE;
		}
exit:
		rcu_read_unlock();
	}
}
