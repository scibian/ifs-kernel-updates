/*
 * Copyright(c) 2015, 2016, 2017 Intel Corporation.
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

#include <linux/io.h>
#include <rdma/rdma_vt.h>
#include <rdma/rdmavt_qp.h>
#include <linux/circ_buf.h>

#include "hfi.h"
#include "qp.h"
#include "rc.h"
#include "verbs_txreq.h"
#include "trace.h"
#include "user_sdma.h"  /* for KDETH_* defines and macros */

static u32 build_tid_rdma_write_req(struct rvt_qp *, struct rvt_swqe *,
				    struct ib_other_headers *, u32 *, u32 *,
				    u32 *);
static u32 build_tid_rdma_write_resp(struct rvt_qp *, struct rvt_ack_entry *,
				     struct ib_other_headers *, u32 *,
				     u32, u32 *, struct rvt_sge_state **);
static u32 build_tid_rdma_write_ack(struct rvt_qp *qp, struct rvt_ack_entry *e,
				    struct ib_other_headers *ohdr, u16 iflow,
				    u32 *bth1, u32 *bth2);
static void tid_rdma_restart_req(struct rvt_qp *qp, struct rvt_swqe *wqe,
				 u32 *bth2);
static bool build_tid_rdma_packet(struct rvt_swqe *wqe,
				  struct ib_other_headers *ohdr,
				  u32 *bth1, u32 *bth2, u32 *len);
static u32 build_tid_rdma_read_packet(struct rvt_swqe *wqe,
				      struct ib_other_headers *ohdr,
				      u32 *bth1, u32 *bth2, u32 *len);
static u32 build_tid_rdma_read_req(struct rvt_qp *qp, struct rvt_swqe *wqe,
				   struct ib_other_headers *ohdr,
				   u32 *bth1, u32 *bth2, u32 *len);
static u32 build_tid_rdma_read_resp(struct rvt_qp *qp, struct rvt_ack_entry *e,
				    struct ib_other_headers *ohdr, u32 *bth0,
				    u32 *bth1, u32 *bth2, u32 *len, bool *last
				    );

static u32 restart_sge(struct rvt_sge_state *ss, struct rvt_swqe *wqe,
		       u32 psn, u32 pmtu)
{
	u32 len;

	len = delta_psn(psn, wqe->psn) * pmtu;
	ss->sge = wqe->sg_list[0];
	ss->sg_list = wqe->sg_list + 1;
	ss->num_sge = wqe->wr.num_sge;
	ss->total_len = wqe->length;
	rvt_skip_sge(ss, len, false);
	return wqe->length - len;
}

static int make_tid_rdma_ack(struct rvt_qp *qp,
			     struct ib_other_headers *ohdr,
			     struct hfi1_pkt_state *ps)
{
	struct rvt_ack_entry *e;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct hfi1_ibdev *dev = to_idev(qp->ibqp.device);
	u32 hwords, next;
	u32 len = 0;
	u32 bth1 = 0, bth2 = 0;
	int middle = 0;
	u16 flow;
	struct tid_rdma_request *req, *nreq;

	/* Don't send an ACK if we aren't supposed to. */
	if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK))
		goto bail;

	/* header size in 32-bit words LRH+BTH = (8+12)/4. */
	hwords = 5;

	e = &qp->s_ack_queue[qpriv->r_tid_ack];
	req = ack_to_tid_req(e);

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] r_tid_tail %u, r_tid_head %u, r_tid_ack %u, s_tail_ack_queue %u, r_head_ack_queue %u, e->psn 0x%x ack_seg %u comp_seg %u total_seg %u",
		  qp->ibqp.qp_num, qpriv->r_tid_tail, qpriv->r_tid_head,
		  qpriv->r_tid_ack, qp->s_tail_ack_queue, qp->r_head_ack_queue,
		  e->psn, req->ack_seg, req->comp_seg, req->total_segs);
#endif
	/*
	 * If we've sent all the ACKs that we can, we are done
	 * until we get more segments...
	 */
	if (!qpriv->s_nak_state && req->ack_seg == req->comp_seg)
		goto bail;

	do {
		/*
		 * To deal with coalesced ACKs, the acked_tail pointer
		 * into the flow array is used. The distance between it
		 * and the clear_tail is the number of flows that are
		 * being ACK'ed.
		 */
		req->ack_seg +=
			/* Get up-to-date value */
			CIRC_CNT(smp_load_acquire(&req->clear_tail),
				 req->acked_tail, req->n_max_flows);
			/* Advance acked index */
		smp_store_release(&req->acked_tail, req->clear_tail);

		/*
		 * req->clear_tail points to the segment currently being
		 * received. So, when sending an ACK, the previous
		 * segment is being ACK'ed.
		 */
		flow = CIRC_PREV(req->acked_tail, req->n_max_flows);
		if (req->ack_seg != req->total_segs)
			break;
		req->state = TID_REQUEST_COMPLETE;

		next = qpriv->r_tid_ack + 1;
		if (next > rvt_size_atomic(&dev->rdi))
			next = 0;
		qpriv->r_tid_ack = next;
		if (qp->s_ack_queue[next].opcode != TID_OP(WRITE_REQ))
			break;
		nreq = ack_to_tid_req(&qp->s_ack_queue[next]);
		if (!nreq->comp_seg || nreq->ack_seg == nreq->comp_seg)
			break;

		/* Move to the next ack entry now */
		e = &qp->s_ack_queue[qpriv->r_tid_ack];
		req = ack_to_tid_req(e);
	} while (1);

	/*
	 * At this point qpriv->r_tid_ack == qpriv->r_tid_tail but e and
	 * req could be pointing at the previous ack queue entry
	 */
	if (qpriv->s_nak_state) {
		/*
		 * A NAK will implicitly acknowledge all previous TID RDMA
		 * requests. Therefore, we NAK with the req->acked_tail
		 * segment for the request at qpriv->r_tid_ack (same at
		 * this point as the req->clear_tail segment for the
		 * qpriv->r_tid_tail request)
		 */
		e = &qp->s_ack_queue[qpriv->r_tid_ack];
		req = ack_to_tid_req(e);
		flow = req->acked_tail;
	} else if (req->ack_seg == req->total_segs &&
		   qpriv->s_flags & HFI1_R_TID_WAIT_INTERLCK)
		qpriv->s_flags &= ~HFI1_R_TID_WAIT_INTERLCK;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] r_tid_ack %u cur_seg %u, ack_seg %u total %u, acked_tail %u clear_tail %u setup_head %u",
		  qp->ibqp.qp_num, qpriv->r_tid_ack, req->cur_seg,
		  req->ack_seg, req->total_segs, req->acked_tail,
		  req->clear_tail, req->setup_head);
#endif

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] s_nak_state 0x%x acked_tail %u clear_tail %u setup_head %u flow %u",
		  qp->ibqp.qp_num, qpriv->s_nak_state, req->acked_tail,
		  req->clear_tail, req->setup_head, flow);
#endif
	hwords += build_tid_rdma_write_ack(qp, e, ohdr, flow, &bth1,
					   &bth2);
	len = 0;
	qpriv->s_flags &= ~(RVT_S_RESP_PENDING | RVT_S_ACK_PENDING);
	ps->s_txreq->hdr_dwords = hwords;
	ps->s_txreq->sde = qpriv->s_sde;
	ps->s_txreq->s_cur_size = len;
	ps->s_txreq->ss = NULL;
	hfi1_make_ruc_header(qp, ohdr, (TID_OP(ACK) << 24), bth1, bth2, middle,
			     ps);
	return 1;
bail:
	/*
	 * Ensure s_rdma_ack_cnt changes are committed prior to resetting
	 * RVT_S_RESP_PENDING
	 */
	smp_wmb();
	qpriv->s_flags &= ~(RVT_S_RESP_PENDING | RVT_S_ACK_PENDING);
	return 0;
}

/**
 * make_rc_ack - construct a response packet (ACK, NAK, or RDMA read)
 * @dev: the device for this QP
 * @qp: a pointer to the QP
 * @ohdr: a pointer to the IB header being constructed
 * @ps: the xmit packet state
 *
 * Return 1 if constructed; otherwise, return 0.
 * Note that we are in the responder's side of the QP context.
 * Note the QP s_lock must be held.
 */
static int make_rc_ack(struct hfi1_ibdev *dev, struct rvt_qp *qp,
		       struct ib_other_headers *ohdr,
		       struct hfi1_pkt_state *ps)
{
	struct rvt_ack_entry *e;
	u32 hwords, hdrlen;
	u32 len = 0;
	u32 bth0 = 0, bth2 = 0;
	u32 bth1 = qp->remote_qpn | (HFI1_CAP_IS_KSET(OPFN) << IB_BTHE_E_SHIFT);
	int middle = 0;
	u32 pmtu = qp->pmtu;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_request *req;
	u8 next = qp->s_tail_ack_queue;
	bool last_pkt;
	u32 delta;

	lockdep_assert_held(&qp->s_lock);
	/* Don't send an ACK if we aren't supposed to. */
	if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK))
		goto bail;

	/* header size in 32-bit words LRH+BTH = (8+12)/4. */
	hwords = 5;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] s_ack_state 0x%x, s_tail_ack_queue %u, r_head_ack_queue %u",
		  qp->ibqp.qp_num, qp->s_ack_state, qp->s_tail_ack_queue,
		  qp->r_head_ack_queue);
#endif
	switch (qp->s_ack_state) {
	case OP(RDMA_READ_RESPONSE_LAST):
	case OP(RDMA_READ_RESPONSE_ONLY):
		e = &qp->s_ack_queue[qp->s_tail_ack_queue];
		if (e->rdma_sge.mr) {
			rvt_put_mr(e->rdma_sge.mr);
			e->rdma_sge.mr = NULL;
		}
		/* FALLTHROUGH */
	case OP(ATOMIC_ACKNOWLEDGE):
		/*
		 * We can increment the tail pointer now that the last
		 * response has been sent instead of only being
		 * constructed.
		 */
		if (++next > rvt_size_atomic(&dev->rdi))
			next = 0;
		/*
		 * Only advance the s_acked_ack_queue pointer if there
		 * have been no TID RDMA requests.
		 */
		e = &qp->s_ack_queue[qp->s_tail_ack_queue];
		if (e->opcode != TID_OP(WRITE_REQ) &&
		    qp->s_acked_ack_queue == qp->s_tail_ack_queue)
			qp->s_acked_ack_queue = next;
		qp->s_tail_ack_queue = next;
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] s_ack_state 0x%x, s_tail_ack_queue %u, r_head_ack_queue %u",
			  qp->ibqp.qp_num, qp->s_ack_state, qp->s_tail_ack_queue,
			  qp->r_head_ack_queue);
#endif
		/* FALLTHROUGH */
	case OP(SEND_ONLY):
	case OP(ACKNOWLEDGE):
		/* Check for no next entry in the queue. */
		if (qp->r_head_ack_queue == qp->s_tail_ack_queue) {
			if (qp->s_flags & RVT_S_ACK_PENDING)
				goto normal;
			goto bail;
		}

		e = &qp->s_ack_queue[qp->s_tail_ack_queue];
		/* Check for tid write fence */
		if ((qpriv->s_flags & HFI1_R_TID_WAIT_INTERLCK) ||
		    hfi1_tid_rdma_ack_interlock(qp, e)) {
			iowait_set_flag(&qpriv->s_iowait, IOWAIT_PENDING_IB);
			goto bail;
		}
		if (e->opcode == OP(RDMA_READ_REQUEST)) {
			/*
			 * If a RDMA read response is being resent and
			 * we haven't seen the duplicate request yet,
			 * then stop sending the remaining responses the
			 * responder has seen until the requester re-sends it.
			 */
			len = e->rdma_sge.sge_length;
			if (len && !e->rdma_sge.mr) {
				qp->s_acked_ack_queue = qp->r_head_ack_queue;
				goto bail;
			}
			/* Copy SGE state in case we need to resend */
			ps->s_txreq->mr = e->rdma_sge.mr;
			if (ps->s_txreq->mr)
				rvt_get_mr(ps->s_txreq->mr);
			qp->s_ack_rdma_sge.sge = e->rdma_sge;
			qp->s_ack_rdma_sge.num_sge = 1;
			ps->s_txreq->ss = &qp->s_ack_rdma_sge;
			if (len > pmtu) {
				len = pmtu;
				qp->s_ack_state = OP(RDMA_READ_RESPONSE_FIRST);
			} else {
				qp->s_ack_state = OP(RDMA_READ_RESPONSE_ONLY);
				e->sent = 1;
			}
			ohdr->u.aeth = rvt_compute_aeth(qp);
			hwords++;
			qp->s_ack_rdma_psn = e->psn;
			bth2 = mask_psn(qp->s_ack_rdma_psn++);
		} else if (e->opcode == TID_OP(WRITE_REQ)) {
			/*
			 * If a TID RDMA WRITE RESP is being resent, we have to
			 * wait for the actual request. All requests that are to
			 * be resent will have their state set to
			 * TID_REQUEST_RESEND. When the new request arrives, the
			 * state will be changed to TID_REQUEST_RESEND_ACTIVE.
			 */
			req = ack_to_tid_req(e);
			if (req->state == TID_REQUEST_RESEND)
				goto bail;
			qp->s_ack_state = TID_OP(WRITE_RESP);
			qp->s_ack_rdma_psn = mask_psn(e->psn + req->cur_seg);
			goto write_resp;
		} else if (e->opcode == TID_OP(READ_REQ)) {
			/*
			 * If a TID RDMA read response is being resent and
			 * we haven't seen the duplicate request yet,
			 * then stop sending the remaining responses the
			 * responder has seen until the requester re-sends it.
			 */
			len = e->rdma_sge.sge_length;
			if (len && !e->rdma_sge.mr) {
				qp->s_tail_ack_queue = qp->r_head_ack_queue;
				goto bail;
			}
			/* Copy SGE state in case we need to resend */
			ps->s_txreq->mr = e->rdma_sge.mr;
			if (ps->s_txreq->mr)
				rvt_get_mr(ps->s_txreq->mr);
			qp->s_ack_rdma_sge.sge = e->rdma_sge;
			qp->s_ack_rdma_sge.num_sge = 1;
			qp->s_ack_state = TID_OP(READ_RESP);
			goto read_resp;
		} else {
			/* COMPARE_SWAP or FETCH_ADD */
			ps->s_txreq->ss = NULL;
			len = 0;
			qp->s_ack_state = OP(ATOMIC_ACKNOWLEDGE);
			ohdr->u.at.aeth = rvt_compute_aeth(qp);
			ib_u64_put(e->atomic_data, &ohdr->u.at.atomic_ack_eth);
			hwords += sizeof(ohdr->u.at) / sizeof(u32);
			bth2 = mask_psn(e->psn);
			e->sent = 1;
		}
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] e->opcode = 0x%x, r_tid_tail %u, r_tid_head %u",
			  qp->ibqp.qp_num, e->opcode, qpriv->r_tid_tail,
			  qpriv->r_tid_head);
#endif
		if (!bth0)
			bth0 = qp->s_ack_state << 24;
		break;

	case OP(RDMA_READ_RESPONSE_FIRST):
		qp->s_ack_state = OP(RDMA_READ_RESPONSE_MIDDLE);
		/* FALLTHROUGH */
	case OP(RDMA_READ_RESPONSE_MIDDLE):
		ps->s_txreq->ss = &qp->s_ack_rdma_sge;
		ps->s_txreq->mr = qp->s_ack_rdma_sge.sge.mr;
		if (ps->s_txreq->mr)
			rvt_get_mr(ps->s_txreq->mr);
		len = qp->s_ack_rdma_sge.sge.sge_length;
		if (len > pmtu) {
			len = pmtu;
			middle = HFI1_CAP_IS_KSET(SDMA_AHG);
		} else {
			ohdr->u.aeth = rvt_compute_aeth(qp);
			hwords++;
			qp->s_ack_state = OP(RDMA_READ_RESPONSE_LAST);
			e = &qp->s_ack_queue[qp->s_tail_ack_queue];
			e->sent = 1;
		}
		bth0 = qp->s_ack_state << 24;
		bth2 = mask_psn(qp->s_ack_rdma_psn++);
		break;

	case TID_OP(WRITE_RESP):
write_resp:
		/*
		 * 1. Check if RVT_S_ACK_PENDING is set. If yes,
		 *    goto normal.
		 * 2. Attempt to allocate TID resources.
		 * 3. Remove RVT_S_RESP_PENDING flags from s_flags
		 * 4. If resources not available:
		 *    4.1 Set RVT_S_WAIT_TID_SPACE
		 *    4.2 Queue QP on RCD TID queue
		 *    4.3 Put QP on iowait list.
		 *    4.4 Build IB RNR NAK with appropriate timeout value
		 *    4.5 Return indication progress made.
		 * 5. If resources are available:
		 *    5.1 Program HW flow CSRs
		 *    5.2 Build TID RDMA WRITE RESP packet
		 *    5.3 If more resources needed, do 2.1 - 2.3.
		 *    5.4 Wake up next QP on RCD TID queue.
		 *    5.5 Return indication progress made.
		 */

		e = &qp->s_ack_queue[qp->s_tail_ack_queue];
		req = ack_to_tid_req(e);

		/*
		 * Send scheduled RNR NAK's. RNR NAK's need to be sent at
		 * segment boundaries, not at request boundaries. Don't change
		 * s_ack_state because we are still in the middle of a request
		 */
		if (qpriv->rnr_nak_state == TID_RNR_NAK_SEND &&
		    qp->s_tail_ack_queue == qpriv->r_tid_alloc &&
		    req->cur_seg == req->alloc_seg) {
			qpriv->rnr_nak_state = TID_RNR_NAK_SENT;
			goto normal_no_state;
		}

		bth2 = mask_psn(qp->s_ack_rdma_psn);
		hdrlen = build_tid_rdma_write_resp(qp, e, ohdr, &bth1,
						   bth2, &len,
						   &ps->s_txreq->ss);
		if (!hdrlen)
			return 0;

		hwords += hdrlen;
		bth0 = qp->s_ack_state << 24;
		qp->s_ack_rdma_psn++;
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA, "[QP%u] cur_seg %u comp_seg %u ack_seg %u total_segs %u",
			  qp->ibqp.qp_num, req->cur_seg, req->comp_seg, req->ack_seg,
			  req->total_segs);
#endif
		if (req->cur_seg != req->total_segs)
			break;

		e->sent = 1;
		qp->s_ack_state = OP(RDMA_READ_RESPONSE_LAST);
		break;

	case TID_OP(READ_RESP):
read_resp:
		e = &qp->s_ack_queue[qp->s_tail_ack_queue];
		ps->s_txreq->ss = &qp->s_ack_rdma_sge;
		delta = build_tid_rdma_read_resp(
					qp, e, ohdr, &bth0, &bth1, &bth2,
					&len, &last_pkt);
		if (delta == 0)
			goto error_qp;
		hwords += delta;
		if (last_pkt) {
			e->sent = 1;
			/*
			 * Increment qp->s_tail_ack_queue through s_ack_state
			 * transition.
			 */
			qp->s_ack_state = OP(RDMA_READ_RESPONSE_LAST);
		}
		break;
	case TID_OP(READ_REQ):
		goto bail;

	default:
normal:
		e = &qp->s_ack_queue[qp->s_tail_ack_queue];
		/*
		 * Send a regular ACK.
		 * Set the s_ack_state so we wait until after sending
		 * the ACK before setting s_ack_state to ACKNOWLEDGE
		 * (see above).
		 */
		qp->s_ack_state = OP(SEND_ONLY);
normal_no_state:
		if (qp->s_nak_state)
			ohdr->u.aeth =
				cpu_to_be32((qp->r_msn &
					     IB_MSN_MASK) |
					    (qp->s_nak_state <<
					     IB_AETH_CREDIT_SHIFT));
		else
			ohdr->u.aeth = rvt_compute_aeth(qp);
		hwords++;
		bth0 = OP(ACKNOWLEDGE) << 24;
		bth2 = mask_psn(qp->s_ack_psn);
		len = 0;
		qp->s_flags &= ~RVT_S_ACK_PENDING;
		ps->s_txreq->ss = NULL;
	}
	qp->s_rdma_ack_cnt++;
	ps->s_txreq->hdr_dwords = hwords;
	ps->s_txreq->sde = qpriv->s_sde;
	ps->s_txreq->s_cur_size = len;
	hfi1_make_ruc_header(qp, ohdr, bth0, bth1, bth2, middle, ps);
	return 1;
error_qp:
	spin_unlock_irqrestore(&qp->s_lock, ps->flags);
	spin_lock_irqsave(&qp->r_lock, ps->flags);
	spin_lock(&qp->s_lock);
	rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);
	spin_unlock(&qp->s_lock);
	spin_unlock_irqrestore(&qp->r_lock, ps->flags);
	spin_lock_irqsave(&qp->s_lock, ps->flags);
bail:
	qp->s_ack_state = OP(ACKNOWLEDGE);
	/*
	 * Ensure s_rdma_ack_cnt changes are committed prior to resetting
	 * RVT_S_RESP_PENDING
	 */
	smp_wmb();
	qp->s_flags &= ~(RVT_S_RESP_PENDING
				| RVT_S_ACK_PENDING
				| HFI1_S_AHG_VALID);
	return 0;
}

/*
 * Call this function when the last TID RDMA WRITE DATA packet for a request
 * is built.
 */
static void update_tid_tail(struct rvt_qp *qp)
	__must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *priv = qp->priv;
	u32 i;
	struct rvt_swqe *wqe;

	lockdep_assert_held(&qp->s_lock);
	/* Can't move beyond s_tid_cur */
	if (priv->s_tid_tail == priv->s_tid_cur)
		return;
	for (i = priv->s_tid_tail + 1; ; i++) {
		if (i == qp->s_size)
			i = 0;

		if (i == priv->s_tid_cur)
			break;
		wqe = rvt_get_swqe_ptr(qp, i);
		if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
			break;
	}
	priv->s_tid_tail = i;
	priv->s_state = TID_OP(WRITE_RESP);
}

int hfi1_make_tid_rdma_pkt(struct rvt_qp *qp, struct hfi1_pkt_state *ps)
	__must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct rvt_swqe *wqe;
	u32 bth1 = 0, bth2 = 0, hwords = 5, len, middle = 0;
	struct ib_other_headers *ohdr;
	struct rvt_sge_state *ss = &qp->s_sge;
	struct rvt_ack_entry *e = &qp->s_ack_queue[qp->s_tail_ack_queue];
	struct tid_rdma_request *req = ack_to_tid_req(e);
	bool last = false;
	u8 opcode = TID_OP(WRITE_DATA);

	lockdep_assert_held(&qp->s_lock);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "priv->n_tid_requests %u priv->n_requests %u priv->s_flags 0x%x",
		  atomic_read(&priv->n_tid_requests),
		  atomic_read(&priv->n_requests),
		  priv->s_flags);
#endif
	/*
	 * Prioritize the sending of the requests and responses over the
	 * sending of the TID RDMA data packets.
	 */
	if (((atomic_read(&priv->n_tid_requests) < HFI1_TID_RDMA_WRITE_CNT) &&
	    atomic_read(&priv->n_requests) &&
	    !(qp->s_flags & (RVT_S_BUSY | RVT_S_WAIT_ACK |
			     HFI1_S_ANY_WAIT_IO))) ||
	    (e->opcode == TID_OP(WRITE_REQ) && req->cur_seg < req->alloc_seg &&
	     !(qp->s_flags & (RVT_S_BUSY | HFI1_S_ANY_WAIT_IO)))) {
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] scheduling reqs: qp->s_flags 0x%x qpriv->s_flags 0x%x priv->n_tid_requests %u priv->n_requests %u l1queued %u",
			  qp->ibqp.qp_num, qp->s_flags, priv->s_flags,
			  atomic_read(&priv->n_tid_requests),
			  atomic_read(&priv->n_requests),
			  verbs_txreq_queued(
				iowait_get_ib_work(&priv->s_iowait)));
#endif
		ps->s_txreq = get_waiting_verbs_txreq(
				iowait_get_ib_work(&priv->s_iowait));
		if (ps->s_txreq || hfi1_make_rc_req(qp, ps)) {
			priv->s_flags |= HFI1_S_TID_BUSY_SET;
			return 1;
		}
	}

	ps->s_txreq = get_txreq(ps->dev, qp);
	if (IS_ERR(ps->s_txreq))
		goto bail_no_tx;

	ohdr = &ps->s_txreq->phdr.hdr.u.oth;

	if ((priv->s_flags & RVT_S_RESP_PENDING) &&
	    make_tid_rdma_ack(qp, ohdr, ps))
		return 1;

	if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_SEND_OK)) {
		if (!(ib_rvt_state_ops[qp->state] & RVT_FLUSH_SEND))
			goto bail;
		/* We are in the error state, flush the work request. */
		smp_read_barrier_depends(); /* see post_one_send() */
		if (qp->s_last == READ_ONCE(qp->s_head))
			goto bail;
		/* If DMAs are in progress, we can't flush immediately. */
		if (iowait_sdma_pending(&priv->s_iowait)) {
			qp->s_flags |= RVT_S_WAIT_DMA;
			goto bail;
		}
		clear_ahg(qp);
		wqe = rvt_get_swqe_ptr(qp, qp->s_last);
		hfi1_send_complete(qp, wqe, qp->s_last != qp->s_acked ?
				   IB_WC_SUCCESS : IB_WC_WR_FLUSH_ERR);
		/* will get called again */
		goto done_free_tx;
	}

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] s_state 0x%x s_tid_tail %u s_tid_cur %u",
		  qp->ibqp.qp_num, priv->s_state,
		  priv->s_tid_tail, priv->s_tid_cur);
#endif
	/* Check whether there is anything to do. */
	if (priv->s_tid_tail == HFI1_QP_WQE_INVALID)
		goto bail;
	wqe = rvt_get_swqe_ptr(qp, priv->s_tid_tail);
	req = wqe_to_tid_req(wqe);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] cur_seg %u comp_seg %u", qp->ibqp.qp_num,
		  req->cur_seg, req->comp_seg);
#endif
	switch (priv->s_state) {
	case TID_OP(WRITE_REQ):
	case TID_OP(WRITE_RESP):
		priv->tid_ss.sge = wqe->sg_list[0];
		priv->tid_ss.sg_list = wqe->sg_list + 1;
		priv->tid_ss.num_sge = wqe->wr.num_sge;
		priv->tid_ss.total_len = wqe->length;

		if (priv->s_state == TID_OP(WRITE_REQ))
			tid_rdma_restart_req(qp, wqe, &bth2);
		priv->s_state = TID_OP(WRITE_DATA);
		/* fall through */

	case TID_OP(WRITE_DATA):
		/*
		 * 1. Check whether TID RDMA WRITE RESP available.
		 * 2. If no:
		 *    2.1 If have more segments and no TID RDMA WRITE RESP,
		 *        set RVT_S_WAIT_TID_RESP
		 *    2.2 Return indicating no progress made.
		 * 3. If yes:
		 *    3.1 Build TID RDMA WRITE DATA packet.
		 *    3.2 If last packet in segment:
		 *        3.2.1 Change KDETH header bits
		 *        3.2.2 Advance RESP pointers.
		 *    3.3 Return indicating progress made.
		 */
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] qp->s_cur %u qp->s_tail %u qp->s_acked %u qp->s_head %u priv->s_tid_cur %u priv->s_tid_tail %u priv->s_tid_head %u",
		  qp->ibqp.qp_num, qp->s_cur, qp->s_tail, qp->s_acked, qp->s_head,
		  priv->s_tid_cur, priv->s_tid_tail, priv->s_tid_head);
#endif
		wqe = rvt_get_swqe_ptr(qp, priv->s_tid_tail);
		req = wqe_to_tid_req(wqe);
		len = wqe->length;

		if (!req->comp_seg || req->cur_seg == req->comp_seg)
			goto bail;
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] state 0x%x cur_seg %u comp_seg %u total_segs %u ack_seg %u",
			  qp->ibqp.qp_num, req->state, req->cur_seg, req->comp_seg,
			  req->total_segs, req->ack_seg);

		if (req->state == TID_REQUEST_INACTIVE ||
		    req->state == TID_REQUEST_COMPLETE) {
			hfi1_cdbg(TIDRDMA, "[QP%u] !!!!!!!!!!!!! Request inactive",
				  qp->ibqp.qp_num);
			goto bail;
		}
#endif
		last = build_tid_rdma_packet(wqe, ohdr, &bth1, &bth2, &len);

		if (last) {
			/* move pointer to next flow */
			smp_store_release(&req->clear_tail,
					  CIRC_NEXT(req->clear_tail,
						    req->n_max_flows));
			if (++req->cur_seg < req->total_segs) {
				if (!CIRC_CNT(req->setup_head, req->clear_tail,
					      req->n_max_flows))
					qp->s_flags |= RVT_S_WAIT_TID_RESP;
			} else {
				priv->s_state = TID_OP(WRITE_DATA_LAST);
				opcode = TID_OP(WRITE_DATA_LAST);

				/* Advance the s_tid_tail now */
				update_tid_tail(qp);
			}
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA,
				  "[QP%u] tid timeout_jiffies %lu  qp timeout_jiffies %lu s_flags 0x%x",
				  qp->ibqp.qp_num,
				  priv->tid_retry_timeout_jiffies,
				  qp->timeout_jiffies,
				  priv->s_flags);
#endif
		}
		hwords += sizeof(ohdr->u.tid_rdma.w_data) / sizeof(u32);
		ss = &priv->tid_ss;
		break;

	default:
		goto bail;
	}
	qp->s_len -= len;
	ps->s_txreq->hdr_dwords = hwords;
	ps->s_txreq->sde = priv->s_sde;
	ps->s_txreq->ss = ss;
	ps->s_txreq->s_cur_size = len;
	hfi1_make_ruc_header(qp, ohdr, (opcode << 24), bth1, bth2,
			     middle, ps);
	return 1;
done_free_tx:
	hfi1_put_txreq(ps->s_txreq);
	ps->s_txreq = NULL;
	return 1;

bail:
	hfi1_put_txreq(ps->s_txreq);
bail_no_tx:
	ps->s_txreq = NULL;
	priv->s_flags &= ~RVT_S_BUSY;
	/*
	 * If we didn't get a txreq, the QP will be woken up later to try
	 * again, set the flags to the the wake up which work item to wake
	 * up.
	 * (A better algorithm should be found to do this and generalize the
	 * sleep/wakeup flags.)
	 */
	iowait_set_flag(&priv->s_iowait, IOWAIT_PENDING_TID);
	return 0;
}

/**
 * hfi1_make_rc_req - construct a request packet (SEND, RDMA r/w, ATOMIC)
 * @qp: a pointer to the QP
 *
 * Assumes s_lock is held.
 *
 * Return 1 if constructed; otherwise, return 0.
 */
int hfi1_make_rc_req(struct rvt_qp *qp, struct hfi1_pkt_state *ps)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_ibdev *dev = to_idev(qp->ibqp.device);
	struct ib_other_headers *ohdr;
	struct rvt_sge_state *ss = NULL;
	struct rvt_swqe *wqe;
	struct hfi1_swqe_priv *wpriv;
	struct tid_rdma_request *req = NULL;
	struct tid_rdma_params *remote;
	/* header size in 32-bit words LRH+BTH = (8+12)/4. */
	u32 hwords = 5;
	u32 len = 0;
	u32 bth0 = 0, bth2 = 0;
	u32 bth1 = qp->remote_qpn | (HFI1_CAP_IS_KSET(OPFN) << IB_BTHE_E_SHIFT);
	u32 pmtu = qp->pmtu;
	char newreq;
	int middle = 0;
	int delta;
	struct tid_rdma_flow *flow = NULL;

	lockdep_assert_held(&qp->s_lock);
	ps->s_txreq = get_txreq(ps->dev, qp);
	if (IS_ERR(ps->s_txreq))
		goto bail_no_tx;

	ohdr = &ps->s_txreq->phdr.hdr.u.oth;
	if (qp->remote_ah_attr.ah_flags & IB_AH_GRH)
		ohdr = &ps->s_txreq->phdr.hdr.u.l.oth;

	/* Sending responses has higher priority over sending requests. */
	if ((qp->s_flags & RVT_S_RESP_PENDING) &&
	    make_rc_ack(dev, qp, ohdr, ps))
		return 1;

	if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_SEND_OK)) {
		if (!(ib_rvt_state_ops[qp->state] & RVT_FLUSH_SEND))
			goto bail;
		/* We are in the error state, flush the work request. */
		smp_read_barrier_depends(); /* see post_one_send() */
		if (qp->s_last == READ_ONCE(qp->s_head))
			goto bail;
		/* If DMAs are in progress, we can't flush immediately. */
		if (iowait_sdma_pending(&priv->s_iowait)) {
			qp->s_flags |= RVT_S_WAIT_DMA;
			goto bail;
		}
		clear_ahg(qp);
		wqe = rvt_get_swqe_ptr(qp, qp->s_last);
		hfi1_send_complete(qp, wqe, qp->s_last != qp->s_acked ?
			IB_WC_SUCCESS : IB_WC_WR_FLUSH_ERR);
		/* will get called again */
		goto done_free_tx;
	}

	if (qp->s_flags & (RVT_S_WAIT_RNR | RVT_S_WAIT_ACK))
		goto bail;

	if (cmp_psn(qp->s_psn, qp->s_sending_hpsn) <= 0) {
		if (cmp_psn(qp->s_sending_psn, qp->s_sending_hpsn) <= 0) {
			qp->s_flags |= RVT_S_WAIT_PSN;
			goto bail;
		}
		qp->s_sending_psn = qp->s_psn;
		qp->s_sending_hpsn = qp->s_psn - 1;
	}

#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] s_cur %u, s_tail %u s_head %u flags 0x%x s_state 0x%x",
			  qp->ibqp.qp_num, qp->s_cur, qp->s_tail, qp->s_head,
			  qp->s_flags, qp->s_state);
#endif
	/* Send a request. */
	wqe = rvt_get_swqe_ptr(qp, qp->s_cur);
check_s_state:
	switch (qp->s_state) {
	default:
		if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_NEXT_SEND_OK))
			goto bail;
		/*
		 * Resend an old request or start a new one.
		 *
		 * We keep track of the current SWQE so that
		 * we don't reset the "furthest progress" state
		 * if we need to back up.
		 */
		newreq = 0;
		if (qp->s_cur == qp->s_tail) {
			/* Check if send work queue is empty. */
			smp_read_barrier_depends(); /* see post_one_send() */
			if (qp->s_tail == READ_ONCE(qp->s_head)) {
				clear_ahg(qp);
				goto bail;
			}
			/*
			 * If a fence is requested, wait for previous
			 * RDMA read and atomic operations to finish.
			 * However, this is no need to guard against
			 * TID RDMA READ after TID RDMA READ.
			 */
			if ((wqe->wr.send_flags & IB_SEND_FENCE) &&
			    qp->s_num_rd_atomic &&
			    (wqe->wr.opcode != IB_WR_TID_RDMA_READ ||
			     priv->pending_tid_r_segs < qp->s_num_rd_atomic)) {
				qp->s_flags |= RVT_S_WAIT_FENCE;
				goto bail;
			}
			/*
			 * Local operations are processed immediately
			 * after all prior requests have completed
			 */
			if (wqe->wr.opcode == IB_WR_REG_MR ||
			    wqe->wr.opcode == IB_WR_LOCAL_INV) {
				int local_ops = 0;
				int err = 0;

				if (qp->s_last != qp->s_cur)
					goto bail;
				if (++qp->s_cur == qp->s_size)
					qp->s_cur = 0;
				if (++qp->s_tail == qp->s_size)
					qp->s_tail = 0;
				if (!(wqe->wr.send_flags &
				      RVT_SEND_COMPLETION_ONLY)) {
					err = rvt_invalidate_rkey(
						qp,
						wqe->wr.ex.invalidate_rkey);
					local_ops = 1;
				}
				hfi1_send_complete(qp, wqe,
						   err ? IB_WC_LOC_PROT_ERR
						       : IB_WC_SUCCESS);
				if (local_ops)
					atomic_dec(&qp->local_ops_pending);
				goto done_free_tx;
			}

			newreq = 1;
			qp->s_psn = wqe->psn;
		}
		/*
		 * Note that we have to be careful not to modify the
		 * original work request since we may need to resend
		 * it.
		 */
		len = wqe->length;
		ss = &qp->s_sge;
		bth2 = mask_psn(qp->s_psn);

		/*
		 * Interlock between various IB requests and TID RDMA
		 * if necessary.
		 */
		if ((priv->s_flags & HFI1_S_TID_WAIT_INTERLCK) ||
		    hfi1_tid_rdma_wqe_interlock(qp, wqe))
			goto bail;

		switch (wqe->wr.opcode) {
		case IB_WR_SEND:
		case IB_WR_SEND_WITH_IMM:
		case IB_WR_SEND_WITH_INV:
			/* If no credit, return. */
			if (!(qp->s_flags & RVT_S_UNLIMITED_CREDIT) &&
			    rvt_cmp_msn(wqe->ssn, qp->s_lsn + 1) > 0) {
				qp->s_flags |= RVT_S_WAIT_SSN_CREDIT;
				goto bail;
			}
			if (len > pmtu) {
				qp->s_state = OP(SEND_FIRST);
				len = pmtu;
				break;
			}
			if (wqe->wr.opcode == IB_WR_SEND) {
				qp->s_state = OP(SEND_ONLY);
			} else if (wqe->wr.opcode == IB_WR_SEND_WITH_IMM) {
				qp->s_state = OP(SEND_ONLY_WITH_IMMEDIATE);
				/* Immediate data comes after the BTH */
				ohdr->u.imm_data = wqe->wr.ex.imm_data;
				hwords += 1;
			} else {
				qp->s_state = OP(SEND_ONLY_WITH_INVALIDATE);
				/* Invalidate rkey comes after the BTH */
				ohdr->u.ieth = cpu_to_be32(
						wqe->wr.ex.invalidate_rkey);
				hwords += 1;
			}
			if (wqe->wr.send_flags & IB_SEND_SOLICITED)
				bth0 |= IB_BTH_SOLICITED;
			bth2 |= IB_BTH_REQ_ACK;
			if (++qp->s_cur == qp->s_size)
				qp->s_cur = 0;
			break;

		case IB_WR_RDMA_WRITE:
			if (newreq && !(qp->s_flags & RVT_S_UNLIMITED_CREDIT))
				qp->s_lsn++;
			goto no_flow_control;
		case IB_WR_RDMA_WRITE_WITH_IMM:
			/* If no credit, return. */
			if (!(qp->s_flags & RVT_S_UNLIMITED_CREDIT) &&
			    rvt_cmp_msn(wqe->ssn, qp->s_lsn + 1) > 0) {
				qp->s_flags |= RVT_S_WAIT_SSN_CREDIT;
				goto bail;
			}
no_flow_control:
			put_ib_reth_vaddr(
				wqe->rdma_wr.remote_addr,
				&ohdr->u.rc.reth);
			ohdr->u.rc.reth.rkey =
				cpu_to_be32(wqe->rdma_wr.rkey);
			ohdr->u.rc.reth.length = cpu_to_be32(len);
			hwords += sizeof(struct ib_reth) / sizeof(u32);
			if (len > pmtu) {
				qp->s_state = OP(RDMA_WRITE_FIRST);
				len = pmtu;
				break;
			}
			if (wqe->wr.opcode == IB_WR_RDMA_WRITE) {
				qp->s_state = OP(RDMA_WRITE_ONLY);
			} else {
				qp->s_state =
					OP(RDMA_WRITE_ONLY_WITH_IMMEDIATE);
				/* Immediate data comes after RETH */
				ohdr->u.rc.imm_data = wqe->wr.ex.imm_data;
				hwords += 1;
				if (wqe->wr.send_flags & IB_SEND_SOLICITED)
					bth0 |= IB_BTH_SOLICITED;
			}
			bth2 |= IB_BTH_REQ_ACK;
			if (++qp->s_cur == qp->s_size)
				qp->s_cur = 0;
			break;

		case IB_WR_TID_RDMA_WRITE:
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA,
				  "[QP%u] spsn 0x%x, newreq %u, n_tid_requests %u",
				  qp->ibqp.qp_num, wqe->psn, newreq,
				  atomic_read(&priv->n_tid_requests));
#endif
			if (newreq) {
				/*
				 * Limit the number of TID RDMA WRITE requests.
				 */
				if (atomic_read(&priv->n_tid_requests) >=
				    HFI1_TID_RDMA_WRITE_CNT)
					goto bail;

				if (!(qp->s_flags & RVT_S_UNLIMITED_CREDIT))
					qp->s_lsn++;

			}

			hwords += build_tid_rdma_write_req(qp, wqe, ohdr, &bth1,
							   &bth2, &len);
			ss = NULL;
			if (priv->s_tid_cur == HFI1_QP_WQE_INVALID) {
				priv->s_tid_cur = qp->s_cur;
				if (priv->s_tid_tail == HFI1_QP_WQE_INVALID) {
					priv->s_tid_tail = qp->s_cur;
					priv->s_state = TID_OP(WRITE_RESP);
				}
			} else if (priv->s_tid_cur == priv->s_tid_head) {
				struct rvt_swqe *__w;
				struct tid_rdma_request *__r;

				__w = rvt_get_swqe_ptr(qp, priv->s_tid_cur);
				__r = wqe_to_tid_req(__w);

				/*
				 * The s_tid_cur pointer is advanced to s_cur if
				 * any of the following conditions about the WQE
				 * to which s_ti_cur currently points to are
				 * satisfied:
				 *   1. The request is not a TID RDMA WRITE
				 *      request,
				 *   2. The request is in the INACTIVE or
				 *      COMPLETE states (TID RDMA READ requests
				 *      stay at INACTIVE and TID RDMA WRITE
				 *      transition to COMPLETE when done),
				 *   3. The request is in the ACTIVE or SYNC
				 *      state and the number of completed
				 *      segments is equal to the total segment
				 *      count.
				 *      (If ACTIVE, the request is waiting for
				 *       ACKs. If SYNC, the request has not
				 *       received any responses because it's
				 *       waiting on a sync point.)
				 */
				if (__w->wr.opcode != IB_WR_TID_RDMA_WRITE ||
				    __r->state == TID_REQUEST_INACTIVE ||
				    __r->state == TID_REQUEST_COMPLETE ||
				    ((__r->state == TID_REQUEST_ACTIVE ||
				      __r->state == TID_REQUEST_SYNC) &&
				     __r->comp_seg == __r->total_segs)) {
					if (priv->s_tid_tail ==
					    priv->s_tid_cur &&
					    priv->s_state ==
					    TID_OP(WRITE_DATA_LAST)) {
						priv->s_tid_tail = qp->s_cur;
						priv->s_state =
							TID_OP(WRITE_RESP);
					}
					priv->s_tid_cur = qp->s_cur;
				}
				/*
				 * A corner case: when the last TID RDMA WRITE
				 * request was completed, s_tid_head,
				 * s_tid_cur, and s_tid_tail all point to the
				 * same location. Other requests are posted and
				 * s_cur wraps around to the same location,
				 * where a new TID RDMA WRITE is posted. In
				 * this case, none of the indices need to be
				 * updated. However, the priv->s_state should.
				 */
				if (priv->s_tid_tail == qp->s_cur &&
				    priv->s_state == TID_OP(WRITE_DATA_LAST))
					priv->s_state = TID_OP(WRITE_RESP);
			}
			req = wqe_to_tid_req(wqe);
			if (newreq) {
				priv->s_tid_head = qp->s_cur;
				priv->pending_tid_w_resp += req->total_segs;
				atomic_inc(&priv->n_tid_requests);
				atomic_dec(&priv->n_requests);
			} else {
				req->state = TID_REQUEST_RESEND;
				req->comp_seg = 0;
				priv->pending_tid_w_resp +=
					delta_psn(wqe->lpsn, bth2) + 1;
			}

#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA,
				  "[QP%u] s_cur %u s_tid_head %u s_tid_cur %u s_tid_tail %u",
				  qp->ibqp.qp_num, qp->s_cur, priv->s_tid_head,
				  priv->s_tid_cur, priv->s_tid_tail);
#endif
			if (++qp->s_cur == qp->s_size)
				qp->s_cur = 0;
			break;

		case IB_WR_RDMA_READ:
			/*
			 * Don't allow more operations to be started
			 * than the QP limits allow.
			 */
			if (qp->s_num_rd_atomic >=
			    qp->s_max_rd_atomic) {
				qp->s_flags |= RVT_S_WAIT_RDMAR;
				goto bail;
			}
			qp->s_num_rd_atomic++;
			if (newreq && !(qp->s_flags & RVT_S_UNLIMITED_CREDIT))
				qp->s_lsn++;
			put_ib_reth_vaddr(
				wqe->rdma_wr.remote_addr,
				&ohdr->u.rc.reth);
			ohdr->u.rc.reth.rkey =
				cpu_to_be32(wqe->rdma_wr.rkey);
			ohdr->u.rc.reth.length = cpu_to_be32(len);
			qp->s_state = OP(RDMA_READ_REQUEST);
			hwords += sizeof(ohdr->u.rc.reth) / sizeof(u32);
			ss = NULL;
			len = 0;
			bth2 |= IB_BTH_REQ_ACK;
			if (++qp->s_cur == qp->s_size)
				qp->s_cur = 0;
			break;

		case IB_WR_TID_RDMA_READ:
#ifdef TIDRDMA_DEBUG
			hfi1_cdbg(TIDRDMA, "op 0x%x QP%u spsn 0x%x, newreq %u",
				  wqe->wr.opcode, qp->ibqp.qp_num, wqe->psn,
				  newreq);
#endif
			wpriv = wqe->priv;
			req = wqe_to_tid_req(wqe);
			delta = cmp_psn(qp->s_psn, wqe->psn);

			/*
			 * Don't allow more operations to be started
			 * than the QP limits allow. We could get here under
			 * three conditions; (1) It's a new request; (2) We are
			 * sending the second or later segment of a request,
			 * but the qp->s_state is set to OP(RDMA_READ_REQUEST)
			 * when the last segment of a previous request is
			 * received just before this; (3) We are re-sending a
			 * request.
			 */
			if (qp->s_num_rd_atomic >= qp->s_max_rd_atomic) {
				qp->s_flags |= RVT_S_WAIT_RDMAR;
				goto bail;
			}
			if (newreq) {
				/*
				 * Set up s_sge as it is needed for TID
				 * allocation.
				 */
				qp->s_sge.sge = wqe->sg_list[0];
				qp->s_sge.sg_list = wqe->sg_list + 1;
				qp->s_sge.num_sge = wqe->wr.num_sge;
				qp->s_sge.total_len = wqe->length;
				qp->s_len = wqe->length;
				req->isge = 0;
				req->clear_tail = req->setup_head;
				req->flow_idx = req->setup_head;
				req->state = TID_REQUEST_ACTIVE;
			} else if (delta == 0){
				/* Re-send a request */
				req->cur_seg = 0;
				req->comp_seg = 0;
				req->ack_pending = 0;
				req->flow_idx = req->clear_tail;
				req->state = TID_REQUEST_RESEND;
			}
			req->s_next_psn = qp->s_psn;
			/* Read one segment at a time */
			len = min_t(u32, req->seg_len,
				    wqe->length - req->seg_len * req->cur_seg);
			delta = build_tid_rdma_read_req(qp, wqe, ohdr, &bth1,
							&bth2, &len);
			if (delta <= 0) {
				/* Wait for TID space */
				goto bail;
			}
			if (newreq && !(qp->s_flags & RVT_S_UNLIMITED_CREDIT))
				qp->s_lsn++;
			hwords += delta;
			ss = &wpriv->ss;
			/* Check if this is the last segment */
			if (req->cur_seg >= req->total_segs &&
			    ++qp->s_cur == qp->s_size)
				qp->s_cur = 0;
			break;

		case IB_WR_ATOMIC_CMP_AND_SWP:
		case IB_WR_ATOMIC_FETCH_AND_ADD:
			/*
			 * Don't allow more operations to be started
			 * than the QP limits allow.
			 */
			if (qp->s_num_rd_atomic >=
			    qp->s_max_rd_atomic) {
				qp->s_flags |= RVT_S_WAIT_RDMAR;
				goto bail;
			}
			qp->s_num_rd_atomic++;

			/* FALLTHROUGH */
		case IB_WR_OPFN:
			if (newreq && !(qp->s_flags & RVT_S_UNLIMITED_CREDIT))
				qp->s_lsn++;
			if (wqe->wr.opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
			    wqe->wr.opcode == IB_WR_OPFN) {
				qp->s_state = OP(COMPARE_SWAP);
				put_ib_ateth_swap(wqe->atomic_wr.swap,
						  &ohdr->u.atomic_eth);
				put_ib_ateth_compare(wqe->atomic_wr.compare_add,
						     &ohdr->u.atomic_eth);
			} else {
				qp->s_state = OP(FETCH_ADD);
				put_ib_ateth_swap(wqe->atomic_wr.compare_add,
						  &ohdr->u.atomic_eth);
				put_ib_ateth_compare(0, &ohdr->u.atomic_eth);
			}
			put_ib_ateth_vaddr(wqe->atomic_wr.remote_addr,
					   &ohdr->u.atomic_eth);
			ohdr->u.atomic_eth.rkey = cpu_to_be32(
				wqe->atomic_wr.rkey);
			hwords += sizeof(struct ib_atomic_eth) / sizeof(u32);
			ss = NULL;
			len = 0;
			bth2 |= IB_BTH_REQ_ACK;
			if (++qp->s_cur == qp->s_size)
				qp->s_cur = 0;
			break;

		default:
			goto bail;
		}
		if (wqe->wr.opcode != IB_WR_TID_RDMA_READ) {
			qp->s_sge.sge = wqe->sg_list[0];
			qp->s_sge.sg_list = wqe->sg_list + 1;
			qp->s_sge.num_sge = wqe->wr.num_sge;
			qp->s_sge.total_len = wqe->length;
			qp->s_len = wqe->length;
		}
		if (newreq) {
			qp->s_tail++;
			if (qp->s_tail >= qp->s_size)
				qp->s_tail = 0;
		}
		if (wqe->wr.opcode == IB_WR_RDMA_READ ||
		    wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
			qp->s_psn = wqe->lpsn + 1;
		else if ( wqe->wr.opcode == IB_WR_TID_RDMA_READ)
			qp->s_psn = req->s_next_psn;
		else
			qp->s_psn++;
		break;

	case OP(RDMA_READ_RESPONSE_FIRST):
		/*
		 * qp->s_state is normally set to the opcode of the
		 * last packet constructed for new requests and therefore
		 * is never set to RDMA read response.
		 * RDMA_READ_RESPONSE_FIRST is used by the ACK processing
		 * thread to indicate a SEND needs to be restarted from an
		 * earlier PSN without interfering with the sending thread.
		 * See restart_rc().
		 */
		qp->s_len = restart_sge(&qp->s_sge, wqe, qp->s_psn, pmtu);
		/* FALLTHROUGH */
	case OP(SEND_FIRST):
		qp->s_state = OP(SEND_MIDDLE);
		/* FALLTHROUGH */
	case OP(SEND_MIDDLE):
		bth2 = mask_psn(qp->s_psn++);
		ss = &qp->s_sge;
		len = qp->s_len;
		if (len > pmtu) {
			len = pmtu;
			middle = HFI1_CAP_IS_KSET(SDMA_AHG);
			break;
		}
		if (wqe->wr.opcode == IB_WR_SEND) {
			qp->s_state = OP(SEND_LAST);
		} else if (wqe->wr.opcode == IB_WR_SEND_WITH_IMM) {
			qp->s_state = OP(SEND_LAST_WITH_IMMEDIATE);
			/* Immediate data comes after the BTH */
			ohdr->u.imm_data = wqe->wr.ex.imm_data;
			hwords += 1;
		} else {
			qp->s_state = OP(SEND_LAST_WITH_INVALIDATE);
			/* invalidate data comes after the BTH */
			ohdr->u.ieth = cpu_to_be32(wqe->wr.ex.invalidate_rkey);
			hwords += 1;
		}
		if (wqe->wr.send_flags & IB_SEND_SOLICITED)
			bth0 |= IB_BTH_SOLICITED;
		bth2 |= IB_BTH_REQ_ACK;
		qp->s_cur++;
		if (qp->s_cur >= qp->s_size)
			qp->s_cur = 0;
		break;

	case OP(RDMA_READ_RESPONSE_LAST):
		/*
		 * qp->s_state is normally set to the opcode of the
		 * last packet constructed for new requests and therefore
		 * is never set to RDMA read response.
		 * RDMA_READ_RESPONSE_LAST is used by the ACK processing
		 * thread to indicate a RDMA write needs to be restarted from
		 * an earlier PSN without interfering with the sending thread.
		 * See restart_rc().
		 */
		qp->s_len = restart_sge(&qp->s_sge, wqe, qp->s_psn, pmtu);
		/* FALLTHROUGH */
	case OP(RDMA_WRITE_FIRST):
		qp->s_state = OP(RDMA_WRITE_MIDDLE);
		/* FALLTHROUGH */
	case OP(RDMA_WRITE_MIDDLE):
		bth2 = mask_psn(qp->s_psn++);
		ss = &qp->s_sge;
		len = qp->s_len;
		if (len > pmtu) {
			len = pmtu;
			middle = HFI1_CAP_IS_KSET(SDMA_AHG);
			break;
		}
		if (wqe->wr.opcode == IB_WR_RDMA_WRITE) {
			qp->s_state = OP(RDMA_WRITE_LAST);
		} else {
			qp->s_state = OP(RDMA_WRITE_LAST_WITH_IMMEDIATE);
			/* Immediate data comes after the BTH */
			ohdr->u.imm_data = wqe->wr.ex.imm_data;
			hwords += 1;
			if (wqe->wr.send_flags & IB_SEND_SOLICITED)
				bth0 |= IB_BTH_SOLICITED;
		}
		bth2 |= IB_BTH_REQ_ACK;
		qp->s_cur++;
		if (qp->s_cur >= qp->s_size)
			qp->s_cur = 0;
		break;

	case OP(RDMA_READ_RESPONSE_MIDDLE):
		/*
		 * qp->s_state is normally set to the opcode of the
		 * last packet constructed for new requests and therefore
		 * is never set to RDMA read response.
		 * RDMA_READ_RESPONSE_MIDDLE is used by the ACK processing
		 * thread to indicate a RDMA read needs to be restarted from
		 * an earlier PSN without interfering with the sending thread.
		 * See restart_rc().
		 */
		len = (delta_psn(qp->s_psn, wqe->psn)) * pmtu;
		put_ib_reth_vaddr(
			wqe->rdma_wr.remote_addr + len,
			&ohdr->u.rc.reth);
		ohdr->u.rc.reth.rkey =
			cpu_to_be32(wqe->rdma_wr.rkey);
		ohdr->u.rc.reth.length = cpu_to_be32(wqe->length - len);
		qp->s_state = OP(RDMA_READ_REQUEST);
		hwords += sizeof(ohdr->u.rc.reth) / sizeof(u32);
		bth2 = mask_psn(qp->s_psn) | IB_BTH_REQ_ACK;
		qp->s_psn = wqe->lpsn + 1;
		ss = NULL;
		len = 0;
		qp->s_cur++;
		if (qp->s_cur == qp->s_size)
			qp->s_cur = 0;
		break;

	case TID_OP(WRITE_RESP):
		/*
		 * This value for s_state is used for restarting a TID RDMA
		 * WRITE request. See comment in OP(RDMA_READ_RESPONSE_MIDDLE
		 * for more).
		 */
		req = wqe_to_tid_req(wqe);
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] Resending TID RDMA WRITE request s_psn 0x%x wqe->psn 0x%x",
			  qp->ibqp.qp_num, qp->s_psn, wqe->psn);
#endif
		req->state = TID_REQUEST_RESEND;
		rcu_read_lock();
		remote = rcu_dereference(priv->tid_rdma.remote);
		req->comp_seg = delta_psn(qp->s_psn, wqe->psn);
		len = wqe->length - (req->comp_seg * remote->max_len);
		rcu_read_unlock();

		bth2 = mask_psn(qp->s_psn);
		hwords += build_tid_rdma_write_req(qp, wqe, ohdr, &bth1,
						   &bth2, &len);
		qp->s_psn = wqe->lpsn + 1;
		ss = NULL;
		qp->s_state = TID_OP(WRITE_REQ);
		priv->pending_tid_w_resp += delta_psn(wqe->lpsn, bth2) + 1;
		priv->s_tid_cur = qp->s_cur;
		if (++qp->s_cur == qp->s_size)
			qp->s_cur = 0;
		break;

	case TID_OP(READ_RESP):
		if (wqe->wr.opcode != IB_WR_TID_RDMA_READ)
			goto bail;
		/* This is used to restart a TID read request */
		req = wqe_to_tid_req(wqe);
		wpriv = wqe->priv;
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] Resending request qp->s_psn 0x%x, wqe->psn 0x%x",
			  qp->ibqp.qp_num, qp->s_psn, wqe->psn);
#endif
		/*
		 * Back down. The field qp->s_psn has been set to the psn with
		 * which the request should be restart. It's OK to use division
		 * as this is on the retry path.
		 */
		req->cur_seg = delta_psn(qp->s_psn, wqe->psn) / priv->pkts_ps;

		/*
		 * The following function need to be redefined to return the
		 * status to make sure that we find the flow. At the same
		 * time, we can use the req->state change to check if the
		 * call succeeds or not.
		 */
		req->state = TID_REQUEST_RESEND;
		tid_rdma_restart_req(qp, wqe, &bth2);
		if (req->state != TID_REQUEST_ACTIVE) {
			/*
			 * Failed to find the flow. Release all allocated tid
			 * resources.
			 */
			hfi1_kern_exp_rcv_clear_all(req);
			hfi1_kern_clear_hw_flow(priv->rcd, qp);

			hfi1_send_complete(qp, wqe, IB_WC_LOC_QP_OP_ERR);
			goto bail;
		}
		req->state = TID_REQUEST_RESEND;
		len = min_t(u32, req->seg_len,
			    wqe->length - req->seg_len * req->cur_seg);
		flow = &req->flows[READ_ONCE(req->flow_idx)];
		len -= flow->sent;
		req->s_next_psn = flow->flow_state.ib_lpsn + 1;
		delta = build_tid_rdma_read_packet(wqe, ohdr, &bth1,
						   &bth2, &len);
		if (delta <= 0) {
			/* Wait for TID space */
			goto bail;
		}
		hwords += delta;
		ss = &wpriv->ss;
		/* Check if this is the last segment */
		if (req->cur_seg >= req->total_segs &&
		    ++qp->s_cur == qp->s_size)
			qp->s_cur = 0;
		qp->s_psn = req->s_next_psn;
		break;
	case TID_OP(READ_REQ):
		req = wqe_to_tid_req(wqe);
		delta = cmp_psn(qp->s_psn, wqe->psn);
		/*
		 * If the current WR is not TID RDMA READ, or this is the start
		 * of a new request, we need to change the qp->s_state so that
		 * the request can be set up properly.
		 */
		if (wqe->wr.opcode != IB_WR_TID_RDMA_READ || delta == 0 ||
		    qp->s_cur == qp->s_tail) {
			qp->s_state = OP(RDMA_READ_REQUEST);
			if (delta == 0 || qp->s_cur == qp->s_tail)
				goto check_s_state;
			else
				goto bail;
		}

		/* Rate limiting */
		if (qp->s_num_rd_atomic >= qp->s_max_rd_atomic) {
			qp->s_flags |= RVT_S_WAIT_RDMAR;
			goto bail;
		}

		wpriv = wqe->priv;
		/* Read one segment at a time */
		len = min_t(u32, req->seg_len,
			    wqe->length - req->seg_len * req->cur_seg);
		delta = build_tid_rdma_read_req(qp, wqe, ohdr, &bth1,
						&bth2, &len);
		if (delta <= 0) {
			/* Wait for TID space */
			goto bail;
		}
		hwords += delta;
		ss = &wpriv->ss;
		/* Check if this is the last segment */
		if (req->cur_seg >= req->total_segs &&
		    ++qp->s_cur == qp->s_size)
			qp->s_cur = 0;
		qp->s_psn = req->s_next_psn;
		break;
	}
	qp->s_sending_hpsn = bth2;
	delta = delta_psn(bth2, wqe->psn);
	if (delta && delta % HFI1_PSN_CREDIT == 0 &&
	    wqe->wr.opcode != IB_WR_TID_RDMA_WRITE)
		bth2 |= IB_BTH_REQ_ACK;
	if (qp->s_flags & RVT_S_SEND_ONE) {
		qp->s_flags &= ~RVT_S_SEND_ONE;
		qp->s_flags |= RVT_S_WAIT_ACK;
		bth2 |= IB_BTH_REQ_ACK;
	}
	qp->s_len -= len;
	ps->s_txreq->hdr_dwords = hwords;
	ps->s_txreq->sde = priv->s_sde;
	ps->s_txreq->ss = ss;
	ps->s_txreq->s_cur_size = len;
	hfi1_make_ruc_header(
		qp,
		ohdr,
		bth0 | (qp->s_state << 24),
		bth1,
		bth2,
		middle,
		ps);
	return 1;

done_free_tx:
	hfi1_put_txreq(ps->s_txreq);
	ps->s_txreq = NULL;
	return 1;

bail:
	hfi1_put_txreq(ps->s_txreq);

bail_no_tx:
	ps->s_txreq = NULL;
	qp->s_flags &= ~RVT_S_BUSY;
	/*
	 * If we didn't get a txreq, the QP will be woken up later to try
	 * again, set the flags to the the wake up which work item to wake
	 * up.
	 * (A better algorithm should be found to do this and generalize the
	 * sleep/wakeup flags.)
	 */
	iowait_set_flag(&priv->s_iowait, IOWAIT_PENDING_IB);
	return 0;
}

/**
 * hfi1_send_rc_ack - Construct an ACK packet and send it
 * @qp: a pointer to the QP
 *
 * This is called from hfi1_rc_rcv() and handle_receive_interrupt().
 * Note that RDMA reads and atomics are handled in the
 * send side QP state and send engine.
 */
void hfi1_send_rc_ack(struct hfi1_ctxtdata *rcd, struct rvt_qp *qp,
		      int is_fecn)
{
	struct hfi1_ibport *ibp = rcd_to_iport(rcd);
	struct hfi1_pportdata *ppd = ppd_from_ibp(ibp);
	u64 pbc, pbc_flags = 0;
	u16 lrh0;
	u16 sc5;
	u32 bth0;
	u32 hwords;
	u32 vl, plen;
	struct send_context *sc;
	struct pio_buf *pbuf;
	struct ib_header hdr;
	struct ib_other_headers *ohdr;
	unsigned long flags;

	/* clear the defer count */
	qp->r_adefered = 0;

	/* Don't send ACK or NAK if a RDMA read or atomic is pending. */
	if (qp->s_flags & RVT_S_RESP_PENDING)
		goto queue_ack;

	/* Ensure s_rdma_ack_cnt changes are committed */
	smp_read_barrier_depends();
	if (qp->s_rdma_ack_cnt)
		goto queue_ack;

	/* Construct the header */
	/* header size in 32-bit words LRH+BTH+AETH = (8+12+4)/4 */
	hwords = 6;
	if (unlikely(qp->remote_ah_attr.ah_flags & IB_AH_GRH)) {
		hwords += hfi1_make_grh(ibp, &hdr.u.l.grh,
				       &qp->remote_ah_attr.grh, hwords, 0);
		ohdr = &hdr.u.l.oth;
		lrh0 = HFI1_LRH_GRH;
	} else {
		ohdr = &hdr.u.oth;
		lrh0 = HFI1_LRH_BTH;
	}
	/* read pkey_index w/o lock (its atomic) */
	bth0 = hfi1_get_pkey(ibp, qp->s_pkey_index) | (OP(ACKNOWLEDGE) << 24);
	if (qp->s_mig_state == IB_MIG_MIGRATED)
		bth0 |= IB_BTH_MIG_REQ;
	if (qp->r_nak_state)
		ohdr->u.aeth = cpu_to_be32((qp->r_msn & IB_MSN_MASK) |
					    (qp->r_nak_state <<
					     IB_AETH_CREDIT_SHIFT));
	else
		ohdr->u.aeth = rvt_compute_aeth(qp);
	sc5 = ibp->sl_to_sc[qp->remote_ah_attr.sl];
	/* set PBC_DC_INFO bit (aka SC[4]) in pbc_flags */
	pbc_flags |= (ib_is_sc5(sc5) << PBC_DC_INFO_SHIFT);
	lrh0 |= (sc5 & 0xf) << 12 | (qp->remote_ah_attr.sl & 0xf) << 4;
	hdr.lrh[0] = cpu_to_be16(lrh0);
	hdr.lrh[1] = cpu_to_be16(qp->remote_ah_attr.dlid);
	hdr.lrh[2] = cpu_to_be16(hwords + SIZE_OF_CRC);
	hdr.lrh[3] = cpu_to_be16(ppd->lid | qp->remote_ah_attr.src_path_bits);
	ohdr->bth[0] = cpu_to_be32(bth0);
	ohdr->bth[1] = cpu_to_be32(qp->remote_qpn);
	ohdr->bth[1] |= cpu_to_be32((!!is_fecn) << IB_BECN_SHIFT);
	/*
	 * Inline ACKs go out without the use of the Verbs send engine, so
	 * we need to set the STL Verbs Extended bit here
	 */
	ohdr->bth[1] |= cpu_to_be32(HFI1_CAP_IS_KSET(OPFN) << IB_BTHE_E_SHIFT);
	ohdr->bth[2] = cpu_to_be32(mask_psn(qp->r_ack_psn));

	/* Don't try to send ACKs if the link isn't ACTIVE */
	if (driver_lstate(ppd) != IB_PORT_ACTIVE)
		return;

	sc = rcd->sc;
	plen = 2 /* PBC */ + hwords;
	vl = sc_to_vlt(ppd->dd, sc5);
	pbc = create_pbc(ppd, pbc_flags, qp->srate_mbps, vl, plen);

	pbuf = sc_buffer_alloc(sc, plen, NULL, NULL);
	if (!pbuf) {
		/*
		 * We have no room to send at the moment.  Pass
		 * responsibility for sending the ACK to the send engine
		 * so that when enough buffer space becomes available,
		 * the ACK is sent ahead of other outgoing packets.
		 */
		goto queue_ack;
	}

	trace_ack_output_ibhdr(dd_from_ibdev(qp->ibqp.device),
			       &hdr, ib_is_sc5(sc5));

	/* write the pbc and data */
	ppd->dd->pio_inline_send(ppd->dd, pbuf, pbc, &hdr, hwords);

	return;

queue_ack:
	spin_lock_irqsave(&qp->s_lock, flags);
	if (!(ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK))
		goto unlock;
	this_cpu_inc(*ibp->rvp.rc_qacks);
	qp->s_flags |= RVT_S_ACK_PENDING | RVT_S_RESP_PENDING;
	qp->s_nak_state = qp->r_nak_state;
	qp->s_ack_psn = qp->r_ack_psn;
	if (is_fecn)
		qp->s_flags |= RVT_S_ECN;

	/* Schedule the send engine. */
	hfi1_schedule_send(qp);
unlock:
	spin_unlock_irqrestore(&qp->s_lock, flags);
}

/**
 * reset_psn - reset the QP state to send starting from PSN
 * @qp: the QP
 * @psn: the packet sequence number to restart at
 *
 * This is called from hfi1_rc_rcv() to process an incoming RC ACK
 * for the given QP.
 * Called at interrupt level with the QP s_lock held.
 */
static void reset_psn(struct rvt_qp *qp, u32 psn)
{
	u32 n = qp->s_acked;
	struct rvt_swqe *wqe = rvt_get_swqe_ptr(qp, n);
	struct tid_rdma_request *req;
	u32 opcode;
	struct hfi1_qp_priv *priv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	qp->s_cur = n;
	qp->s_timeout_shift = 0;
	priv->pending_tid_r_segs = 0;
	priv->pending_tid_w_resp = 0;
	qp->s_num_rd_atomic = 0;

	/*
	 * If we are starting the request from the beginning,
	 * let the normal send code handle initialization.
	 */
	if (cmp_psn(psn, wqe->psn) <= 0) {
		qp->s_state = OP(SEND_LAST);
		goto done;
	}

	/* Find the work request opcode corresponding to the given PSN. */
	opcode = wqe->wr.opcode;
	for (;;) {
		int diff;

		if (++n == qp->s_size)
			n = 0;
		if (n == qp->s_tail)
			break;
		wqe = rvt_get_swqe_ptr(qp, n);
		diff = cmp_psn(psn, wqe->psn);
		if (diff < 0)
			break;
		qp->s_cur = n;
		/*
		 * If we are starting the request from the beginning,
		 * let the normal send code handle initialization.
		 */
		if (diff == 0) {
			qp->s_state = OP(SEND_LAST);
			goto done;
		}
		opcode = wqe->wr.opcode;
		if (opcode == IB_WR_RDMA_READ ||
		    opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
		    opcode == IB_WR_ATOMIC_FETCH_AND_ADD) {
			qp->s_num_rd_atomic++;
		} else if (opcode == IB_WR_TID_RDMA_READ) {
			req = wqe_to_tid_req(wqe);
			if (cmp_psn(psn, wqe->lpsn) <= 0) {
				u32 cur_seg;

				cur_seg = (psn - wqe->psn) / priv->pkts_ps;
				req->ack_pending = cur_seg - req->comp_seg;
				priv->pending_tid_r_segs += req->ack_pending;
				qp->s_num_rd_atomic += req->ack_pending;
			} else {
				priv->pending_tid_r_segs += req->total_segs;
				qp->s_num_rd_atomic += req->total_segs;
			}
		}
	}

	/*
	 * Set the state to restart in the middle of a request.
	 * Don't change the s_sge, s_cur_sge, or s_cur_size.
	 * See hfi1_make_rc_req().
	 */
	switch (opcode) {
	case IB_WR_SEND:
	case IB_WR_SEND_WITH_IMM:
		qp->s_state = OP(RDMA_READ_RESPONSE_FIRST);
		break;

	case IB_WR_RDMA_WRITE:
	case IB_WR_RDMA_WRITE_WITH_IMM:
		qp->s_state = OP(RDMA_READ_RESPONSE_LAST);
		break;

	case IB_WR_TID_RDMA_WRITE:
		qp->s_state = TID_OP(WRITE_RESP);
		break;

	case IB_WR_RDMA_READ:
		qp->s_state = OP(RDMA_READ_RESPONSE_MIDDLE);
		break;

	case IB_WR_TID_RDMA_READ:
		qp->s_state = TID_OP(READ_RESP);
		break;

	default:
		/*
		 * This case shouldn't happen since its only
		 * one PSN per req.
		 */
		qp->s_state = OP(SEND_LAST);
	}
done:
	priv->s_flags &= ~HFI1_S_TID_WAIT_INTERLCK;
	qp->s_psn = psn;
	/*
	 * Set RVT_S_WAIT_PSN as rc_complete() may start the timer
	 * asynchronously before the send engine can get scheduled.
	 * Doing it in hfi1_make_rc_req() is too late.
	 */
	if ((cmp_psn(qp->s_psn, qp->s_sending_hpsn) <= 0) &&
	    (cmp_psn(qp->s_sending_psn, qp->s_sending_hpsn) <= 0))
		qp->s_flags |= RVT_S_WAIT_PSN;
	qp->s_flags &= ~HFI1_S_AHG_VALID;
}

/*
 * Back up requester to resend the last un-ACKed request.
 * The QP r_lock and s_lock should be held and interrupts disabled.
 */
void hfi1_restart_rc(struct rvt_qp *qp, u32 psn, int wait)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct rvt_swqe *wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
	struct hfi1_ibport *ibp;

	lockdep_assert_held(&qp->r_lock);
	lockdep_assert_held(&qp->s_lock);
	if (qp->s_retry == 0) {
		if (qp->s_mig_state == IB_MIG_ARMED) {
			hfi1_migrate_qp(qp);
			qp->s_retry = qp->s_retry_cnt;
		} else if (qp->s_last == qp->s_acked) {
			/*
			 * We need special handling for the OPFN request WQEs as
			 * they are not allowed to generate real user errors
			 */
			if (wqe->wr.opcode == IB_WR_OPFN) {
				struct hfi1_ibport *ibp =
					to_iport(qp->ibqp.device, qp->port_num);
				/*
				 * Call opfn_conn_reply() with capcode and
				 * remaining data as 0 to close out the
				 * current request
				 */
				opfn_conn_reply(qp, priv->opfn.curr);
				wqe = do_rc_completion(qp, wqe, ibp);
				qp->s_flags &= ~RVT_S_WAIT_ACK;
			} else {
				if (wqe->wr.opcode == IB_WR_TID_RDMA_READ) {
					struct tid_rdma_request *req;

					req = wqe_to_tid_req(wqe);
					hfi1_kern_exp_rcv_clear_all(req);
					hfi1_kern_clear_hw_flow(priv->rcd, qp);
				}

				hfi1_send_complete(qp, wqe,
						   IB_WC_RETRY_EXC_ERR);
				rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);
			}
			return;
		} else { /* need to handle delayed completion */
			return;
		}
	} else {
		qp->s_retry--;
	}

	ibp = to_iport(qp->ibqp.device, qp->port_num);
	if (wqe->wr.opcode == IB_WR_RDMA_READ ||
	    wqe->wr.opcode == IB_WR_TID_RDMA_READ)
		ibp->rvp.n_rc_resends++;
	else
		ibp->rvp.n_rc_resends += delta_psn(qp->s_psn, psn);

	qp->s_flags &= ~(RVT_S_WAIT_FENCE | RVT_S_WAIT_RDMAR |
			 RVT_S_WAIT_SSN_CREDIT | RVT_S_WAIT_PSN |
			 RVT_S_WAIT_ACK | RVT_S_WAIT_TID_RESP);
	if (wait)
		qp->s_flags |= RVT_S_SEND_ONE;
	reset_psn(qp, psn);
}

#if defined(TIDRDMA_DEBUG) && defined(TIDRDMA_EXTRA_DEBUG)
static void dump_flows(struct tid_rdma_request *req)
{
	u16 idx;
	struct flow_state *state;

	hfi1_cdbg(TIDRDMA, "Unacked flows:");
	for (idx = req->clear_tail;
	     CIRC_CNT(req->setup_head, idx, req->n_max_flows);
	     idx = CIRC_NEXT(idx, req->n_max_flows)) {
		state = &req->flows[idx].flow_state;
		hfi1_cdbg(TIDRDMA, "%u spsn 0x%x lpsn 0x%x",
			  idx, __full_flow_psn(state, state->spsn),
			  __full_flow_psn(state, state->lpsn));
	}
}
#endif

/*
 * Set qp->s_sending_psn to the next PSN after the given one.
 * This would be psn+1 except when RDMA reads or TID RDMA ops
 * are present.
 */
static void reset_sending_psn(struct rvt_qp *qp, u32 psn, u32 opcode)
{
	struct rvt_swqe *wqe;
	u32 n = qp->s_last;

	lockdep_assert_held(&qp->s_lock);
	/* Find the work request corresponding to the given PSN. */
	for (;;) {
		wqe = rvt_get_swqe_ptr(qp, n);
		if (cmp_psn(psn, wqe->lpsn) <= 0) {
			if (wqe->wr.opcode == IB_WR_RDMA_READ ||
			    wqe->wr.opcode == IB_WR_TID_RDMA_READ ||
			    wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
				qp->s_sending_psn = wqe->lpsn + 1;
			else
				qp->s_sending_psn = psn + 1;
			break;
		}
		if (++n == qp->s_size)
			n = 0;
		if (n == qp->s_tail)
			break;
	}
}

/*
 * This should be called with the QP s_lock held and interrupts disabled.
 */
void hfi1_rc_send_complete(struct rvt_qp *qp, struct ib_header *hdr)
{
	struct ib_other_headers *ohdr;
	struct rvt_swqe *wqe;
	u32 opcode, head, tail;
	u32 psn;
	struct hfi1_qp_priv *priv = qp->priv;
	struct tid_rdma_request *req;

	lockdep_assert_held(&qp->s_lock);
	if (!(ib_rvt_state_ops[qp->state] & RVT_SEND_OR_FLUSH_OR_RECV_OK))
		return;

	/* Find out where the BTH is */
	if (ib_get_lnh(hdr) == HFI1_LRH_BTH)
		ohdr = &hdr->u.oth;
	else
		ohdr = &hdr->u.l.oth;

	opcode = ib_bth_get_opcode(ohdr);
	if ((opcode >= OP(RDMA_READ_RESPONSE_FIRST) &&
	     opcode <= OP(ATOMIC_ACKNOWLEDGE)) ||
	    opcode == TID_OP(WRITE_RESP) ||
	    opcode == TID_OP(READ_RESP)) {
		WARN_ON(!qp->s_rdma_ack_cnt);
		qp->s_rdma_ack_cnt--;
		return;
	}

	psn = ib_bth_get_psn(ohdr);
	/*
	 * Don't attempt to reset the sending PSN for packets in the
	 * KDETH PSN space since the PSN does not match anything.
	 */
	if (opcode != TID_OP(WRITE_DATA) &&
	    opcode != TID_OP(WRITE_DATA_LAST) &&
	    opcode != TID_OP(ACK))
		reset_sending_psn(qp, psn, opcode);

	/* Handle TID RDMA WRITE packets differently */
	if (opcode >= TID_OP(WRITE_REQ) &&
	    opcode <= TID_OP(WRITE_DATA_LAST)) {
		head = priv->s_tid_head;
		tail = priv->s_tid_cur;
		/*
		 * s_tid_cur is set to s_tid_head in the case, where
		 * a new TID RDMA request is being started and all
		 * previous ones have been completed.
		 * Therefore, we need to do a secondary check in order
		 * to properly determine whether we should start the
		 * RC timer.
		 */
		wqe = rvt_get_swqe_ptr(qp, tail);
		req = wqe_to_tid_req(wqe);
		if (head == tail && req->comp_seg < req->total_segs)
			tail -= 1;
	} else {
		head = qp->s_tail;
		tail = qp->s_acked;
	}

	/*
	 * Start timer after a packet requesting an ACK has been sent and
	 * there are still requests that haven't been acked.
	 */
	if ((psn & IB_BTH_REQ_ACK) && tail != head &&
	    !(qp->s_flags &
	      (RVT_S_TIMER | RVT_S_WAIT_RNR | RVT_S_WAIT_PSN)) &&
	    (ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK)) {
		if (opcode == TID_OP(READ_REQ))
			qp->s_timeout_shift = priv->timeout_shift;
		else
			qp->s_timeout_shift = 0;
		rvt_add_retry_timer(qp);
	}

	/* Start TID RDMA ACK timer */
	if ((opcode == TID_OP(WRITE_DATA) ||
	     opcode == TID_OP(WRITE_DATA_LAST)) &&
	    (psn & IB_BTH_REQ_ACK) &&
	    !(priv->s_flags & HFI1_S_TID_RETRY_TIMER) &&
	    (ib_rvt_state_ops[qp->state] & RVT_PROCESS_RECV_OK)) {
		/*
		 * The TID RDMA ACK packet could be received before this
		 * function is called. Therefore, add the timer only if TID
		 * RDMA ACK packets are actually pending.
		 */
		wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
		req = wqe_to_tid_req(wqe);
		if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE &&
		    req->ack_seg < req->cur_seg)
			hfi1_add_tid_retry_timer(qp);
	}

	while (qp->s_last != qp->s_acked) {
		u32 s_last;

		wqe = rvt_get_swqe_ptr(qp, qp->s_last);
		if (cmp_psn(wqe->lpsn, qp->s_sending_psn) >= 0 &&
		    cmp_psn(qp->s_sending_psn, qp->s_sending_hpsn) <= 0)
			break;
		rvt_qp_wqe_unreserve(qp, wqe);
		s_last = qp->s_last;
		trace_hfi1_qp_send_completion(qp, wqe, s_last);
		if (++s_last >= qp->s_size)
			s_last = 0;
		qp->s_last = s_last;
		/* see post_send() */
		barrier();
		rvt_put_swqe(wqe);
		rvt_qp_swqe_complete(qp,
				     wqe,
				     ib_hfi1_wc_opcode[wqe->wr.opcode],
				     IB_WC_SUCCESS);
	}
	/*
	 * If we were waiting for sends to complete before re-sending,
	 * and they are now complete, restart sending.
	 */
	trace_hfi1_sendcomplete(qp, psn);
	if (qp->s_flags & RVT_S_WAIT_PSN &&
	    cmp_psn(qp->s_sending_psn, qp->s_sending_hpsn) > 0) {
		qp->s_flags &= ~RVT_S_WAIT_PSN;
		qp->s_sending_psn = qp->s_psn;
		qp->s_sending_hpsn = qp->s_psn - 1;
		hfi1_schedule_send(qp);
	}
}

static inline void update_last_psn(struct rvt_qp *qp, u32 psn)
{
	qp->s_last_psn = psn;
}

/*
 * Generate a SWQE completion.
 * This is similar to hfi1_send_complete but has to check to be sure
 * that the SGEs are not being referenced if the SWQE is being resent.
 */
struct rvt_swqe *do_rc_completion(struct rvt_qp *qp, struct rvt_swqe *wqe,
				  struct hfi1_ibport *ibp)
{
	struct hfi1_qp_priv *priv = qp->priv;

	lockdep_assert_held(&qp->s_lock);
	/*
	 * Don't decrement refcount and don't generate a
	 * completion if the SWQE is being resent until the send
	 * is finished.
	 */
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] lpsn 0x%x, s_sending_psn 0x%x, s_sending_hpsn 0x%x %d %d",
		  qp->ibqp.qp_num, wqe->lpsn, qp->s_sending_psn,
		  qp->s_sending_hpsn, cmp_psn(wqe->lpsn, qp->s_sending_psn),
		  cmp_psn(qp->s_sending_psn, qp->s_sending_hpsn));
#endif
	if (cmp_psn(wqe->lpsn, qp->s_sending_psn) < 0 ||
	    cmp_psn(qp->s_sending_psn, qp->s_sending_hpsn) > 0) {
		u32 s_last;

		rvt_put_swqe(wqe);
		rvt_qp_wqe_unreserve(qp, wqe);
		s_last = qp->s_last;
		trace_hfi1_qp_send_completion(qp, wqe, s_last);
		if (++s_last >= qp->s_size)
			s_last = 0;
		qp->s_last = s_last;
		/* see post_send() */
		barrier();
		rvt_qp_swqe_complete(qp,
				     wqe,
				     ib_hfi1_wc_opcode[wqe->wr.opcode],
				     IB_WC_SUCCESS);
	} else {
		struct hfi1_pportdata *ppd = ppd_from_ibp(ibp);

		this_cpu_inc(*ibp->rvp.rc_delayed_comp);
		/*
		 * If send progress not running attempt to progress
		 * SDMA queue.
		 */
		if (ppd->dd->flags & HFI1_HAS_SEND_DMA) {
			struct sdma_engine *engine;
			u8 sc5;

			/* For now use sc to find engine */
			sc5 = ibp->sl_to_sc[qp->remote_ah_attr.sl];
			engine = qp_to_sdma_engine(qp, sc5);
			sdma_engine_progress_schedule(engine);
		}
	}

	qp->s_retry = qp->s_retry_cnt;
	/*
	 * Don't update the last PSN if the request being completed is
	 * a TID RDMA WRITE request.
	 * Completion of the TID RDMA WRITE requests are done by the
	 * TID RDMA ACKs and as such could be for a request that has
	 * already been ACKed as far as the IB state machine is
	 * concerned.
	 */
	if (wqe->wr.opcode != IB_WR_TID_RDMA_WRITE)
		update_last_psn(qp, wqe->lpsn);

	/*
	 * If we are completing a request which is in the process of
	 * being resent, we can stop re-sending it since we know the
	 * responder has already seen it.
	 */
	if (qp->s_acked == qp->s_cur) {
		if (++qp->s_cur >= qp->s_size)
			qp->s_cur = 0;
		qp->s_acked = qp->s_cur;
		wqe = rvt_get_swqe_ptr(qp, qp->s_cur);
		if (qp->s_acked != qp->s_tail) {
			qp->s_state = OP(SEND_LAST);
			qp->s_psn = wqe->psn;
		}
	} else {
		if (++qp->s_acked >= qp->s_size)
			qp->s_acked = 0;
		if (qp->state == IB_QPS_SQD && qp->s_acked == qp->s_cur)
			qp->s_draining = 0;
		wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
	}
	if (priv->s_flags & HFI1_S_TID_WAIT_INTERLCK) {
		priv->s_flags &= ~HFI1_S_TID_WAIT_INTERLCK;
		hfi1_schedule_send(qp);
	}
	return wqe;
}

static void set_restart_qp(struct rvt_qp *qp, struct hfi1_ctxtdata *rcd)
{
	/* Retry this request. */
	if (!(qp->r_flags & RVT_R_RDMAR_SEQ)) {
		qp->r_flags |= RVT_R_RDMAR_SEQ;
		hfi1_restart_rc(qp, qp->s_last_psn + 1, 0);
		if (list_empty(&qp->rspwait)) {
			qp->r_flags |= RVT_R_RSP_SEND;
			rvt_get_qp(qp);
			list_add_tail(&qp->rspwait, &rcd->qp_wait_list);
		}
	}
}

/**
 * do_rc_ack - process an incoming RC ACK
 * @qp: the QP the ACK came in on
 * @psn: the packet sequence number of the ACK
 * @opcode: the opcode of the request that resulted in the ACK
 *
 * This is called from rc_rcv_resp() to process an incoming RC ACK
 * for the given QP.
 * May be called at interrupt level, with the QP s_lock held.
 * Returns 1 if OK, 0 if current operation should be aborted (NAK).
 */
int do_rc_ack(struct rvt_qp *qp, u32 aeth, u32 psn, int opcode, u64 val,
	      struct hfi1_ctxtdata *rcd)
{
	struct hfi1_ibport *ibp;
	enum ib_wc_status status;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct rvt_swqe *wqe;
	int ret = 0;
	u32 ack_psn;
	int diff;

	lockdep_assert_held(&qp->s_lock);
	/*
	 * Note that NAKs implicitly ACK outstanding SEND and RDMA write
	 * requests and implicitly NAK RDMA read and atomic requests issued
	 * before the NAK'ed request.  The MSN won't include the NAK'ed
	 * request but will include an ACK'ed request(s).
	 */
	ack_psn = psn;
	if (aeth >> IB_AETH_NAK_SHIFT)
		ack_psn--;
	wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
	ibp = rcd_to_iport(rcd);

	/*
	 * The MSN might be for a later WQE than the PSN indicates so
	 * only complete WQEs that the PSN finishes.
	 */
	while ((diff = delta_psn(ack_psn, wqe->lpsn)) >= 0) {
		/*
		 * RDMA_READ_RESPONSE_ONLY is a special case since
		 * we want to generate completion events for everything
		 * before the RDMA read, copy the data, then generate
		 * the completion for the read.
		 */
		if (wqe->wr.opcode == IB_WR_RDMA_READ &&
		    opcode == OP(RDMA_READ_RESPONSE_ONLY) &&
		    diff == 0) {
			ret = 1;
			goto bail_stop;
		}
		/*
		 * If this request is a RDMA read or atomic, and the ACK is
		 * for a later operation, this ACK NAKs the RDMA read or
		 * atomic.  In other words, only a RDMA_READ_LAST or ONLY
		 * can ACK a RDMA read and likewise for atomic ops.  Note
		 * that the NAK case can only happen if relaxed ordering is
		 * used and requests are sent after an RDMA read or atomic
		 * is sent but before the response is received.
		 */
		if ((wqe->wr.opcode == IB_WR_RDMA_READ &&
		     (opcode != OP(RDMA_READ_RESPONSE_LAST) || diff != 0)) ||
		    (wqe->wr.opcode == IB_WR_TID_RDMA_READ &&
		     (opcode != TID_OP(READ_RESP) || diff != 0)) ||
		    ((wqe->wr.opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
		      wqe->wr.opcode == IB_WR_ATOMIC_FETCH_AND_ADD) &&
		     (opcode != OP(ATOMIC_ACKNOWLEDGE) || diff != 0)) ||
		    (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE &&
		     (delta_psn(psn, qp->s_last_psn) != 1))) {
			set_restart_qp(qp, rcd);
			/*
			 * No need to process the ACK/NAK since we are
			 * restarting an earlier request.
			 */
			goto bail_stop;
		}
		if (wqe->wr.opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
		    wqe->wr.opcode == IB_WR_ATOMIC_FETCH_AND_ADD) {
			u64 *vaddr = wqe->sg_list[0].vaddr;
			*vaddr = val;
		}
		if (wqe->wr.opcode == IB_WR_OPFN) {
			hfi1_cdbg(OPFN,
				  "Completion Opcode 0x%x, PSN 0x%x, LPSN 0x%x",
				  wqe->wr.opcode, wqe->psn, wqe->lpsn);
			opfn_conn_reply(qp, val);
		}
		if (qp->s_num_rd_atomic &&
		    (wqe->wr.opcode == IB_WR_RDMA_READ ||
		     wqe->wr.opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
		     wqe->wr.opcode == IB_WR_ATOMIC_FETCH_AND_ADD)) {
			qp->s_num_rd_atomic--;
			/* Restart sending task if fence is complete */
			if ((qp->s_flags & RVT_S_WAIT_FENCE) &&
			    !qp->s_num_rd_atomic) {
				qp->s_flags &= ~(RVT_S_WAIT_FENCE |
						 RVT_S_WAIT_ACK);
				hfi1_schedule_send(qp);
			} else if (qp->s_flags & RVT_S_WAIT_RDMAR) {
				qp->s_flags &= ~(RVT_S_WAIT_RDMAR |
						 RVT_S_WAIT_ACK);
				hfi1_schedule_send(qp);
			}
		}

		/*
		 * TID RDMA WRITE requests will be completed by the TID RDMA
		 * ACK packet handler (see tid_rdma.c).
		 */
		if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
			break;

		wqe = do_rc_completion(qp, wqe, ibp);
		if (qp->s_acked == qp->s_tail)
			break;
	}

	switch (aeth >> IB_AETH_NAK_SHIFT) {
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] AETH: 0x%x, %u, PSN 0x%x, s_acked %u, s_tail %u, opcode 0x%x, wqe->psn 0x%x wqe->lpsn 0x%x qp->s_psn",
		  qp->ibqp.qp_num, aeth, (aeth >> 29), psn, qp->s_acked,
		  qp->s_tail, wqe->wr.opcode, wqe->psn, wqe->lpsn, qp->s_psn);
#endif
	case 0:         /* ACK */
		this_cpu_inc(*ibp->rvp.rc_acks);
		qp->s_timeout_shift = 0;
		if (wqe->wr.opcode == IB_WR_TID_RDMA_READ) {
			if (wqe_to_tid_req(wqe)->ack_pending) {
				qp->s_timeout_shift = qpriv->timeout_shift;
				rvt_mod_retry_timer(qp);
			} else {
				rvt_stop_rc_timers(qp);
			}
		} else if (qp->s_acked != qp->s_tail) {
			struct rvt_swqe *__w = NULL;

			if (qpriv->s_tid_cur != HFI1_QP_WQE_INVALID)
				__w = rvt_get_swqe_ptr(qp, qpriv->s_tid_cur);

			/*
			 * Stop timers if we've received all of the TID RDMA WRITE
			 * responses.
			 */
			if (__w && __w->wr.opcode == IB_WR_TID_RDMA_WRITE &&
			    opcode == TID_OP(WRITE_RESP)) {
				/*
				 * Normally, the loop above would correctly
				 * process all WQEs from s_acked onward and
				 * either complete them or check for correct PSN
				 * sequencing.
				 * However, for TID RDMA, due to pipelining, the
				 * response may not be for the request at
				 * s_acked so the above look would just be
				 * skipped. This does not allow for checking the
				 * PSN sequencing. It has to be done separately.
				 */
				if (cmp_psn(psn, qp->s_last_psn + 1)) {
					set_restart_qp(qp, rcd);
					goto bail_stop;
				}
				if (--qpriv->pending_tid_w_resp)
					rvt_mod_retry_timer(qp);
				else
					rvt_stop_rc_timers(qp);
			} else {
				/*
				 * We are expecting more ACKs so
				 * mod the retry timer.
				 */
				rvt_mod_retry_timer(qp);
				/*
				 * We can stop re-sending the earlier packets and
				 * continue with the next packet the receiver wants.
				 */
				if (cmp_psn(qp->s_psn, psn) <= 0)
					reset_psn(qp, psn + 1);
			}
                } else {
                        /* No more acks - kill all timers */
                        rvt_stop_rc_timers(qp);
                        if (cmp_psn(qp->s_psn, psn) <= 0) {
                                qp->s_state = OP(SEND_LAST);
                                qp->s_psn = psn + 1;
                        }
                }
                if (qp->s_flags & RVT_S_WAIT_ACK) {
                        qp->s_flags &= ~RVT_S_WAIT_ACK;
                        hfi1_schedule_send(qp);
                }

		rvt_get_credit(qp, aeth);
                qp->s_rnr_retry = qp->s_rnr_retry_cnt;
                qp->s_retry = qp->s_retry_cnt;
                update_last_psn(qp, psn);
                return 1;

	case 1:         /* RNR NAK */
		ibp->rvp.n_rnr_naks++;
		if (qp->s_acked == qp->s_tail)
			goto bail_stop;
		if (qp->s_flags & RVT_S_WAIT_RNR)
			goto bail_stop;
		if (qp->s_rnr_retry == 0) {
			status = IB_WC_RNR_RETRY_EXC_ERR;
			goto class_b;
		}
		if (qp->s_rnr_retry_cnt < 7)
			qp->s_rnr_retry--;

		/* The last valid PSN is the previous PSN. */
		update_last_psn(qp, psn - 1);

		ibp->rvp.n_rc_resends += delta_psn(qp->s_psn, psn);

		reset_psn(qp, psn);

		qp->s_flags &= ~(RVT_S_WAIT_SSN_CREDIT | RVT_S_WAIT_ACK);
		qp->s_timeout_shift = 0;
		rvt_stop_rc_timers(qp);
		rvt_add_rnr_timer(qp, aeth);
		return 0;

	case 3:         /* NAK */
		if (qp->s_acked == qp->s_tail)
			goto bail_stop;
		/* The last valid PSN is the previous PSN. */
		update_last_psn(qp, psn - 1);
		switch ((aeth >> IB_AETH_CREDIT_SHIFT) &
			IB_AETH_CREDIT_MASK) {
		case 0: /* PSN sequence error */
			ibp->rvp.n_seq_naks++;
			/*
			 * Back up to the responder's expected PSN.
			 * Note that we might get a NAK in the middle of an
			 * RDMA READ response which terminates the RDMA
			 * READ.
			 */
			hfi1_restart_rc(qp, psn, 0);
			hfi1_schedule_send(qp);
			break;

		case 1: /* Invalid Request */
			status = IB_WC_REM_INV_REQ_ERR;
			ibp->rvp.n_other_naks++;
			goto class_b;

		case 2: /* Remote Access Error */
			status = IB_WC_REM_ACCESS_ERR;
			ibp->rvp.n_other_naks++;
			goto class_b;

		case 3: /* Remote Operation Error */
			status = IB_WC_REM_OP_ERR;
			ibp->rvp.n_other_naks++;
class_b:
			if (qp->s_last == qp->s_acked) {
				if (wqe->wr.opcode == IB_WR_TID_RDMA_READ)
					hfi1_kern_read_tid_flow_free(qp);

				hfi1_send_complete(qp, wqe, status);
				rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);
			}
			break;

		default:
			/* Ignore other reserved NAK error codes */
			goto reserved;
		}
		qp->s_retry = qp->s_retry_cnt;
		qp->s_rnr_retry = qp->s_rnr_retry_cnt;
		goto bail_stop;

	default:                /* 2: reserved */
reserved:
		/* Ignore reserved NAK codes. */
		goto bail_stop;
	}
	/* cannot be reached  */
bail_stop:
	qp->s_timeout_shift = 0;
	rvt_stop_rc_timers(qp);
	return ret;
}

/*
 * We have seen an out of sequence RDMA read middle or last packet.
 * This ACKs SENDs and RDMA writes up to the first RDMA read or atomic SWQE.
 */
void rdma_seq_err(struct rvt_qp *qp, struct hfi1_ibport *ibp, u32 psn,
			 struct hfi1_ctxtdata *rcd)
{
	struct rvt_swqe *wqe;

	lockdep_assert_held(&qp->s_lock);
	/* Remove QP from retry timer */
	rvt_stop_rc_timers(qp);

	wqe = rvt_get_swqe_ptr(qp, qp->s_acked);

	while (cmp_psn(psn, wqe->lpsn) > 0) {
		if (wqe->wr.opcode == IB_WR_RDMA_READ ||
		    wqe->wr.opcode == IB_WR_TID_RDMA_READ ||
		    wqe->wr.opcode == IB_WR_TID_RDMA_WRITE ||
		    wqe->wr.opcode == IB_WR_ATOMIC_CMP_AND_SWP ||
		    wqe->wr.opcode == IB_WR_ATOMIC_FETCH_AND_ADD)
			break;
		wqe = do_rc_completion(qp, wqe, ibp);
	}

	ibp->rvp.n_rdma_seq++;
	qp->r_flags |= RVT_R_RDMAR_SEQ;
	hfi1_restart_rc(qp, qp->s_last_psn + 1, 0);
	if (list_empty(&qp->rspwait)) {
		qp->r_flags |= RVT_R_RSP_SEND;
		rvt_get_qp(qp);
		list_add_tail(&qp->rspwait, &rcd->qp_wait_list);
	}
}

/**
 * rc_rcv_resp - process an incoming RC response packet
 * @ibp: the port this packet came in on
 * @ohdr: the other headers for this packet
 * @data: the packet data
 * @tlen: the packet length
 * @qp: the QP for this packet
 * @opcode: the opcode for this packet
 * @psn: the packet sequence number for this packet
 * @hdrsize: the header length
 * @pmtu: the path MTU
 *
 * This is called from hfi1_rc_rcv() to process an incoming RC response
 * packet for the given QP.
 * Called at interrupt level.
 */
static void rc_rcv_resp(struct hfi1_ibport *ibp,
			struct ib_other_headers *ohdr,
			void *data, u32 tlen, struct rvt_qp *qp,
			u32 opcode, u32 psn, u32 hdrsize, u32 pmtu,
			struct hfi1_ctxtdata *rcd)
{
	struct rvt_swqe *wqe;
	enum ib_wc_status status;
	unsigned long flags;
	int diff;
	u32 pad;
	u32 aeth;
	u64 val;

	spin_lock_irqsave(&qp->s_lock, flags);

	trace_hfi1_ack(qp, psn);

	/* Ignore invalid responses. */
	smp_read_barrier_depends(); /* see post_one_send */
	if (cmp_psn(psn, READ_ONCE(qp->s_next_psn)) >= 0)
		goto ack_done;

	/* Ignore duplicate responses. */
	diff = cmp_psn(psn, qp->s_last_psn);
	if (unlikely(diff <= 0)) {
		/* Update credits for "ghost" ACKs */
		if (diff == 0 && opcode == OP(ACKNOWLEDGE)) {
			aeth = be32_to_cpu(ohdr->u.aeth);
			if ((aeth >> IB_AETH_NAK_SHIFT) == 0)
				rvt_get_credit(qp, aeth);
		}
		goto ack_done;
	}

	/*
	 * Skip everything other than the PSN we expect, if we are waiting
	 * for a reply to a restarted RDMA read or atomic op.
	 */
	if (qp->r_flags & RVT_R_RDMAR_SEQ) {
		if (cmp_psn(psn, qp->s_last_psn + 1) != 0)
			goto ack_done;
		qp->r_flags &= ~RVT_R_RDMAR_SEQ;
	}

	if (unlikely(qp->s_acked == qp->s_tail))
		goto ack_done;
	wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
	status = IB_WC_SUCCESS;

	switch (opcode) {
	case OP(ACKNOWLEDGE):
	case OP(ATOMIC_ACKNOWLEDGE):
	case OP(RDMA_READ_RESPONSE_FIRST):
#if 0
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA, "[QP%u] Receive ACK packet",
			  qp->ibqp.qp_num);
#endif
#endif
		aeth = be32_to_cpu(ohdr->u.aeth);
		if (opcode == OP(ATOMIC_ACKNOWLEDGE))
			val = ib_u64_get(&ohdr->u.at.atomic_ack_eth);
		else
			val = 0;
		if (!do_rc_ack(qp, aeth, psn, opcode, val, rcd) ||
		    opcode != OP(RDMA_READ_RESPONSE_FIRST))
			goto ack_done;
		wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
		if (unlikely(wqe->wr.opcode != IB_WR_RDMA_READ))
			goto ack_op_err;
		/*
		 * If this is a response to a resent RDMA read, we
		 * have to be careful to copy the data to the right
		 * location.
		 */
		qp->s_rdma_read_len = restart_sge(&qp->s_rdma_read_sge,
						  wqe, psn, pmtu);
		goto read_middle;

	case OP(RDMA_READ_RESPONSE_MIDDLE):
		/* no AETH, no ACK */
		if (unlikely(cmp_psn(psn, qp->s_last_psn + 1)))
			goto ack_seq_err;
		if (unlikely(wqe->wr.opcode != IB_WR_RDMA_READ))
			goto ack_op_err;
read_middle:
		if (unlikely(tlen != (hdrsize + pmtu + 4)))
			goto ack_len_err;
		if (unlikely(pmtu >= qp->s_rdma_read_len))
			goto ack_len_err;

		/*
		 * We got a response so update the timeout.
		 * 4.096 usec. * (1 << qp->timeout)
		 */
		rvt_mod_retry_timer(qp);
		if (qp->s_flags & RVT_S_WAIT_ACK) {
			qp->s_flags &= ~RVT_S_WAIT_ACK;
			hfi1_schedule_send(qp);
		}

		if (opcode == OP(RDMA_READ_RESPONSE_MIDDLE))
			qp->s_retry = qp->s_retry_cnt;

		/*
		 * Update the RDMA receive state but do the copy w/o
		 * holding the locks and blocking interrupts.
		 */
		qp->s_rdma_read_len -= pmtu;
		update_last_psn(qp, psn);
		spin_unlock_irqrestore(&qp->s_lock, flags);
		hfi1_copy_sge(&qp->s_rdma_read_sge, data, pmtu, false, false);
		goto bail;

	case OP(RDMA_READ_RESPONSE_ONLY):
		aeth = be32_to_cpu(ohdr->u.aeth);
		if (!do_rc_ack(qp, aeth, psn, opcode, 0, rcd))
			goto ack_done;
		/* Get the number of bytes the message was padded by. */
		pad = ib_bth_get_pad(ohdr);
		/*
		 * Check that the data size is >= 0 && <= pmtu.
		 * Remember to account for ICRC (4).
		 */
		if (unlikely(tlen < (hdrsize + pad + 4)))
			goto ack_len_err;
		/*
		 * If this is a response to a resent RDMA read, we
		 * have to be careful to copy the data to the right
		 * location.
		 */
		wqe = rvt_get_swqe_ptr(qp, qp->s_acked);
		qp->s_rdma_read_len = restart_sge(&qp->s_rdma_read_sge,
						  wqe, psn, pmtu);
		goto read_last;

	case OP(RDMA_READ_RESPONSE_LAST):
		/* ACKs READ req. */
		if (unlikely(cmp_psn(psn, qp->s_last_psn + 1)))
			goto ack_seq_err;
		if (unlikely(wqe->wr.opcode != IB_WR_RDMA_READ))
			goto ack_op_err;
		/* Get the number of bytes the message was padded by. */
		pad = ib_bth_get_pad(ohdr);
		/*
		 * Check that the data size is >= 1 && <= pmtu.
		 * Remember to account for ICRC (4).
		 */
		if (unlikely(tlen <= (hdrsize + pad + 4)))
			goto ack_len_err;
read_last:
		tlen -= hdrsize + pad + 4;
		if (unlikely(tlen != qp->s_rdma_read_len))
			goto ack_len_err;
		aeth = be32_to_cpu(ohdr->u.aeth);
		hfi1_copy_sge(&qp->s_rdma_read_sge, data, tlen, false, false);
		WARN_ON(qp->s_rdma_read_sge.num_sge);
		(void)do_rc_ack(qp, aeth, psn,
				 OP(RDMA_READ_RESPONSE_LAST), 0, rcd);
		goto ack_done;

	/* XXX (Mitko): I think that some of these opcodes should not be here.
	 * XXX (Mitko): For example, I don't think that we would ever get here
	 * XXX (Mitko): by receiving a TID RDMA Write Request. */
	case TID_OP(WRITE_REQ):
	case TID_OP(WRITE_RESP):
	case TID_OP(WRITE_DATA):
		break;
	}

ack_op_err:
	status = IB_WC_LOC_QP_OP_ERR;
	goto ack_err;

ack_seq_err:
	rdma_seq_err(qp, ibp, psn, rcd);
	goto ack_done;

ack_len_err:
	status = IB_WC_LOC_LEN_ERR;
ack_err:
	if (qp->s_last == qp->s_acked) {
		hfi1_send_complete(qp, wqe, status);
		rvt_error_qp(qp, IB_WC_WR_FLUSH_ERR);
	}
ack_done:
	spin_unlock_irqrestore(&qp->s_lock, flags);
bail:
	return;
}

static inline void rc_cancel_ack(struct rvt_qp *qp)
{
	qp->r_adefered = 0;
	if (list_empty(&qp->rspwait))
		return;
	list_del_init(&qp->rspwait);
	qp->r_flags &= ~RVT_R_RSP_NAK;
	rvt_put_qp(qp);
}

/**
 * rc_rcv_error - process an incoming duplicate or error RC packet
 * @ohdr: the other headers for this packet
 * @data: the packet data
 * @qp: the QP for this packet
 * @opcode: the opcode for this packet
 * @psn: the packet sequence number for this packet
 * @diff: the difference between the PSN and the expected PSN
 *
 * This is called from hfi1_rc_rcv() to process an unexpected
 * incoming RC packet for the given QP.
 * Called at interrupt level.
 * Return 1 if no more processing is needed; otherwise return 0 to
 * schedule a response to be sent.
 */
static noinline int rc_rcv_error(struct ib_other_headers *ohdr, void *data,
				 struct rvt_qp *qp, u32 opcode, u32 psn,
				 int diff, struct hfi1_ctxtdata *rcd)
{
	struct hfi1_ibport *ibp = rcd_to_iport(rcd);
	struct rvt_ack_entry *e;
	unsigned long flags;
	u8 prev;
	u8 mra; /* most recent ACK */
	bool old_req;

	trace_hfi1_rcv_error(qp, psn);
	if (diff > 0) {
		/*
		 * Packet sequence error.
		 * A NAK will ACK earlier sends and RDMA writes.
		 * Don't queue the NAK if we already sent one.
		 */
		if (!qp->r_nak_state) {
			ibp->rvp.n_rc_seqnak++;
			qp->r_nak_state = IB_NAK_PSN_ERROR;
			/* Use the expected PSN. */
			qp->r_ack_psn = qp->r_psn;
			/*
			 * Wait to send the sequence NAK until all packets
			 * in the receive queue have been processed.
			 * Otherwise, we end up propagating congestion.
			 */
			rc_defered_ack(rcd, qp);
		}
		goto done;
	}

	/*
	 * Handle a duplicate request.  Don't re-execute SEND, RDMA
	 * write or atomic op.  Don't NAK errors, just silently drop
	 * the duplicate request.  Note that r_sge, r_len, and
	 * r_rcv_len may be in use so don't modify them.
	 *
	 * We are supposed to ACK the earliest duplicate PSN but we
	 * can coalesce an outstanding duplicate ACK.  We have to
	 * send the earliest so that RDMA reads can be restarted at
	 * the requester's expected PSN.
	 *
	 * First, find where this duplicate PSN falls within the
	 * ACKs previously sent.
	 * old_req is true if there is an older response that is scheduled
	 * to be sent before sending this one.
	 */
	e = NULL;
	old_req = 1;
	ibp->rvp.n_rc_dupreq++;

	spin_lock_irqsave(&qp->s_lock, flags);

	e = find_prev_entry(qp, psn, &prev, &mra, &old_req);

	switch (opcode) {
	case OP(RDMA_READ_REQUEST): {
		struct ib_reth *reth;
		u32 offset;
		u32 len;

		/*
		 * If we didn't find the RDMA read request in the ack queue,
		 * we can ignore this request.
		 */
		if (!e || e->opcode != OP(RDMA_READ_REQUEST))
			goto unlock_done;
		/* RETH comes after BTH */
		reth = &ohdr->u.rc.reth;
		/*
		 * Address range must be a subset of the original
		 * request and start on pmtu boundaries.
		 * We reuse the old ack_queue slot since the requester
		 * should not back up and request an earlier PSN for the
		 * same request.
		 */
		offset = delta_psn(psn, e->psn) * qp->pmtu;
		len = be32_to_cpu(reth->length);
		if (unlikely(offset + len != e->rdma_sge.sge_length))
			goto unlock_done;
		if (e->rdma_sge.mr) {
			rvt_put_mr(e->rdma_sge.mr);
			e->rdma_sge.mr = NULL;
		}
		if (len != 0) {
			u32 rkey = be32_to_cpu(reth->rkey);
			u64 vaddr = get_ib_reth_vaddr(reth);
			int ok;

			ok = rvt_rkey_ok(qp, &e->rdma_sge, len, vaddr, rkey,
					 IB_ACCESS_REMOTE_READ);
			if (unlikely(!ok))
				goto unlock_done;
		} else {
			e->rdma_sge.vaddr = NULL;
			e->rdma_sge.length = 0;
			e->rdma_sge.sge_length = 0;
		}
		e->psn = psn;
		if (old_req)
			goto unlock_done;
		if (qp->s_acked_ack_queue == qp->s_tail_ack_queue)
			qp->s_acked_ack_queue = prev;
		qp->s_tail_ack_queue = prev;
		break;
	}

	case OP(COMPARE_SWAP):
	case OP(FETCH_ADD): {
		/*
		 * If we didn't find the atomic request in the ack queue
		 * or the send engine is already backed up to send an
		 * earlier entry, we can ignore this request.
		 */
		if (!e || e->opcode != (u8)opcode || old_req)
			goto unlock_done;
		if (qp->s_tail_ack_queue == qp->s_acked_ack_queue)
			qp->s_acked_ack_queue = prev;
		qp->s_tail_ack_queue = prev;
		break;
	}

	default:
		/*
		 * Ignore this operation if it doesn't request an ACK
		 * or an earlier RDMA read or atomic is going to be resent.
		 */
		if (!(psn & IB_BTH_REQ_ACK) || old_req)
			goto unlock_done;
		/*
		 * Resend the most recent ACK if this request is
		 * after all the previous RDMA reads and atomics.
		 */
		if (mra == qp->r_head_ack_queue) {
			spin_unlock_irqrestore(&qp->s_lock, flags);
			qp->r_nak_state = 0;
			qp->r_ack_psn = qp->r_psn - 1;
			goto send_ack;
		}

		/*
		 * Resend the RDMA read or atomic op which
		 * ACKs this duplicate request.
		 */
		if (qp->s_tail_ack_queue == qp->s_acked_ack_queue)
			qp->s_acked_ack_queue = mra;
		qp->s_tail_ack_queue = mra;
		break;
	}
	qp->s_ack_state = OP(ACKNOWLEDGE);
	qp->s_flags |= RVT_S_RESP_PENDING;
	qp->r_nak_state = 0;
	hfi1_schedule_send(qp);

unlock_done:
	spin_unlock_irqrestore(&qp->s_lock, flags);
done:
	return 1;

send_ack:
	return 0;
}

static void log_cca_event(struct hfi1_pportdata *ppd, u8 sl, u32 rlid,
			  u32 lqpn, u32 rqpn, u8 svc_type)
{
	struct opa_hfi1_cong_log_event_internal *cc_event;
	unsigned long flags;

	if (sl >= OPA_MAX_SLS)
		return;

	spin_lock_irqsave(&ppd->cc_log_lock, flags);

	ppd->threshold_cong_event_map[sl / 8] |= 1 << (sl % 8);
	ppd->threshold_event_counter++;

	cc_event = &ppd->cc_events[ppd->cc_log_idx++];
	if (ppd->cc_log_idx == OPA_CONG_LOG_ELEMS)
		ppd->cc_log_idx = 0;
	cc_event->lqpn = lqpn & RVT_QPN_MASK;
	cc_event->rqpn = rqpn & RVT_QPN_MASK;
	cc_event->sl = sl;
	cc_event->svc_type = svc_type;
	cc_event->rlid = rlid;
	/* keep timestamp in units of 1.024 usec */
	cc_event->timestamp = ktime_to_ns(ktime_get()) / 1024;

	spin_unlock_irqrestore(&ppd->cc_log_lock, flags);
}

void process_becn(struct hfi1_pportdata *ppd, u8 sl, u16 rlid, u32 lqpn,
		  u32 rqpn, u8 svc_type)
{
	struct cca_timer *cca_timer;
	u16 ccti, ccti_incr, ccti_timer, ccti_limit;
	u8 trigger_threshold;
	struct cc_state *cc_state;
	unsigned long flags;

	if (sl >= OPA_MAX_SLS)
		return;

	cc_state = get_cc_state(ppd);

	if (!cc_state)
		return;

	/*
	 * 1) increase CCTI (for this SL)
	 * 2) select IPG (i.e., call set_link_ipg())
	 * 3) start timer
	 */
	ccti_limit = cc_state->cct.ccti_limit;
	ccti_incr = cc_state->cong_setting.entries[sl].ccti_increase;
	ccti_timer = cc_state->cong_setting.entries[sl].ccti_timer;
	trigger_threshold =
		cc_state->cong_setting.entries[sl].trigger_threshold;

	spin_lock_irqsave(&ppd->cca_timer_lock, flags);

	cca_timer = &ppd->cca_timer[sl];
	if (cca_timer->ccti < ccti_limit) {
		if (cca_timer->ccti + ccti_incr <= ccti_limit)
			cca_timer->ccti += ccti_incr;
		else
			cca_timer->ccti = ccti_limit;
		set_link_ipg(ppd);
	}

	ccti = cca_timer->ccti;

	if (!hrtimer_active(&cca_timer->hrtimer)) {
		/* ccti_timer is in units of 1.024 usec */
		unsigned long nsec = 1024 * ccti_timer;

		hrtimer_start(&cca_timer->hrtimer, ns_to_ktime(nsec),
			      HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&ppd->cca_timer_lock, flags);

	if ((trigger_threshold != 0) && (ccti >= trigger_threshold))
		log_cca_event(ppd, sl, rlid, lqpn, rqpn, svc_type);
}

static void opfn_trigger_conn_request(struct rvt_qp *qp, u32 bth1)
{
	struct hfi1_qp_priv *priv = qp->priv;

	if (!priv->opfn.extended && hfi1_opfn_extended(bth1) &&
	    HFI1_CAP_IS_KSET(OPFN)) {
		priv->opfn.extended = true;
		if (qp->state == IB_QPS_RTS)
			opfn_conn_request(qp);
	}
}

/**
 * hfi1_rc_rcv - process an incoming RC packet
 * @rcd: the context pointer
 * @hdr: the header of this packet
 * @rcv_flags: flags relevant to rcv processing
 * @data: the packet data
 * @tlen: the packet length
 * @qp: the QP for this packet
 *
 * This is called from qp_rcv() to process an incoming RC packet
 * for the given QP.
 * May be called at interrupt level.
 */
void hfi1_rc_rcv(struct hfi1_packet *packet)
{
	struct hfi1_ctxtdata *rcd = packet->rcd;
	void *data = packet->ebuf;
	u32 tlen = packet->tlen;
	struct rvt_qp *qp = packet->qp;
	struct hfi1_ibport *ibp = rcd_to_iport(rcd);
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct ib_other_headers *ohdr = packet->ohdr;
	u32 bth0, bth1 = be32_to_cpu(ohdr->bth[1]);
	u32 opcode = packet->opcode;
	u32 hdrsize = packet->hlen;
	u32 psn;
	u32 pad = packet->pad;
	struct ib_wc wc;
	u32 pmtu = qp->pmtu;
	int diff;
	struct ib_reth *reth;
	unsigned long flags;
	int ret;
	bool is_fecn = false;
	bool copy_last = false;
	u32 rkey;

	lockdep_assert_held(&qp->r_lock);

	bth0 = be32_to_cpu(ohdr->bth[0]);
	if (hfi1_ruc_check_hdr(ibp, packet))
		return;

	is_fecn = process_ecn(qp, packet, false);
	opfn_trigger_conn_request(qp, bth1);

	psn = ib_bth_get_psn(ohdr);
	opcode = (bth0 >> 24) & 0xff;

	/*
	 * Process responses (ACKs) before anything else.  Note that the
	 * packet sequence number will be for something in the send work
	 * queue rather than the expected receive packet sequence number.
	 * In other words, this QP is the requester.
	 */
	if (opcode >= OP(RDMA_READ_RESPONSE_FIRST) &&
	    opcode <= OP(ATOMIC_ACKNOWLEDGE)) {
		rc_rcv_resp(ibp, ohdr, data, tlen, qp, opcode, psn,
			    hdrsize, pmtu, rcd);
		if (is_fecn)
			goto send_ack;
		return;
	}

	/* Compute 24 bits worth of difference. */
	diff = delta_psn(psn, qp->r_psn);
	if (unlikely(diff)) {
		if (rc_rcv_error(ohdr, data, qp, opcode, psn, diff, rcd))
			return;
		goto send_ack;
	}

	/* Check for opcode sequence errors. */
	switch (qp->r_state) {
	case OP(SEND_FIRST):
	case OP(SEND_MIDDLE):
		if (opcode == OP(SEND_MIDDLE) ||
		    opcode == OP(SEND_LAST) ||
		    opcode == OP(SEND_LAST_WITH_IMMEDIATE) ||
		    opcode == OP(SEND_LAST_WITH_INVALIDATE))
			break;
		goto nack_inv;

	case OP(RDMA_WRITE_FIRST):
	case OP(RDMA_WRITE_MIDDLE):
		if (opcode == OP(RDMA_WRITE_MIDDLE) ||
		    opcode == OP(RDMA_WRITE_LAST) ||
		    opcode == OP(RDMA_WRITE_LAST_WITH_IMMEDIATE))
			break;
		goto nack_inv;

	default:
		if (opcode == OP(SEND_MIDDLE) ||
		    opcode == OP(SEND_LAST) ||
		    opcode == OP(SEND_LAST_WITH_IMMEDIATE) ||
		    opcode == OP(SEND_LAST_WITH_INVALIDATE) ||
		    opcode == OP(RDMA_WRITE_MIDDLE) ||
		    opcode == OP(RDMA_WRITE_LAST) ||
		    opcode == OP(RDMA_WRITE_LAST_WITH_IMMEDIATE))
			goto nack_inv;
		/*
		 * Note that it is up to the requester to not send a new
		 * RDMA read or atomic operation before receiving an ACK
		 * for the previous operation.
		 */
		break;
	}

	if (qp->state == IB_QPS_RTR && !(qp->r_flags & RVT_R_COMM_EST))
		rvt_comm_est(qp);

	/* OK, process the packet. */
	switch (opcode) {
	case OP(SEND_FIRST):
		ret = hfi1_rvt_get_rwqe(qp, 0);
		if (ret < 0)
			goto nack_op_err;
		if (!ret)
			goto rnr_nak;
		qp->r_rcv_len = 0;
		/* FALLTHROUGH */
	case OP(SEND_MIDDLE):
	case OP(RDMA_WRITE_MIDDLE):
send_middle:
		/* Check for invalid length PMTU or posted rwqe len. */
		if (unlikely(tlen != (hdrsize + pmtu + 4)))
			goto nack_inv;
		qp->r_rcv_len += pmtu;
		if (unlikely(qp->r_rcv_len > qp->r_len))
			goto nack_inv;
		hfi1_copy_sge(&qp->r_sge, data, pmtu, true, false);
		break;

	case OP(RDMA_WRITE_LAST_WITH_IMMEDIATE):
		/* consume RWQE */
		ret = hfi1_rvt_get_rwqe(qp, 1);
		if (ret < 0)
			goto nack_op_err;
		if (!ret)
			goto rnr_nak;
		goto send_last_imm;

	case OP(SEND_ONLY):
	case OP(SEND_ONLY_WITH_IMMEDIATE):
	case OP(SEND_ONLY_WITH_INVALIDATE):
		ret = hfi1_rvt_get_rwqe(qp, 0);
		if (ret < 0)
			goto nack_op_err;
		if (!ret)
			goto rnr_nak;
		qp->r_rcv_len = 0;
		if (opcode == OP(SEND_ONLY))
			goto no_immediate_data;
		if (opcode == OP(SEND_ONLY_WITH_INVALIDATE))
			goto send_last_inv;
		/* FALLTHROUGH for SEND_ONLY_WITH_IMMEDIATE */
	case OP(SEND_LAST_WITH_IMMEDIATE):
send_last_imm:
		wc.ex.imm_data = ohdr->u.imm_data;
		wc.wc_flags = IB_WC_WITH_IMM;
		goto send_last;
	case OP(SEND_LAST_WITH_INVALIDATE):
send_last_inv:
		rkey = be32_to_cpu(ohdr->u.ieth);
		if (rvt_invalidate_rkey(qp, rkey))
			goto no_immediate_data;
		wc.ex.invalidate_rkey = rkey;
		wc.wc_flags = IB_WC_WITH_INVALIDATE;
		goto send_last;
	case OP(RDMA_WRITE_LAST):
		copy_last = rvt_is_user_qp(qp);
		/* fall through */
	case OP(SEND_LAST):
no_immediate_data:
		wc.wc_flags = 0;
		wc.ex.imm_data = 0;
send_last:
		/* Check for invalid length. */
		/* LAST len should be >= 1 */
		if (unlikely(tlen < (hdrsize + pad + 4)))
			goto nack_inv;
		/* Don't count the CRC. */
		tlen -= (hdrsize + pad + 4);
		wc.byte_len = tlen + qp->r_rcv_len;
		if (unlikely(wc.byte_len > qp->r_len))
			goto nack_inv;
		hfi1_copy_sge(&qp->r_sge, data, tlen, true, copy_last);
		rvt_put_ss(&qp->r_sge);
		qp->r_msn++;
		if (!__test_and_clear_bit(RVT_R_WRID_VALID, &qp->r_aflags))
			break;
		wc.wr_id = qp->r_wr_id;
		wc.status = IB_WC_SUCCESS;
		if (opcode == OP(RDMA_WRITE_LAST_WITH_IMMEDIATE) ||
		    opcode == OP(RDMA_WRITE_ONLY_WITH_IMMEDIATE))
			wc.opcode = IB_WC_RECV_RDMA_WITH_IMM;
		else
			wc.opcode = IB_WC_RECV;
		wc.qp = &qp->ibqp;
		wc.src_qp = qp->remote_qpn;
		wc.slid = qp->remote_ah_attr.dlid;
		/*
		 * It seems that IB mandates the presence of an SL in a
		 * work completion only for the UD transport (see section
		 * 11.4.2 of IBTA Vol. 1).
		 *
		 * However, the way the SL is chosen below is consistent
		 * with the way that IB/qib works and is trying avoid
		 * introducing incompatibilities.
		 *
		 * See also OPA Vol. 1, section 9.7.6, and table 9-17.
		 */
		wc.sl = qp->remote_ah_attr.sl;
		/* zero fields that are N/A */
		wc.vendor_err = 0;
		wc.pkey_index = 0;
		wc.dlid_path_bits = 0;
		wc.port_num = 0;
		/* Signal completion event if the solicited bit is set. */
		rvt_cq_enter(ibcq_to_rvtcq(qp->ibqp.recv_cq), &wc,
			     (bth0 & IB_BTH_SOLICITED) != 0);
		break;

	case OP(RDMA_WRITE_ONLY):
		copy_last = rvt_is_user_qp(qp);
		/* fall through */
	case OP(RDMA_WRITE_FIRST):
	case OP(RDMA_WRITE_ONLY_WITH_IMMEDIATE):
		if (unlikely(!(qp->qp_access_flags & IB_ACCESS_REMOTE_WRITE)))
			goto nack_inv;
		/* consume RWQE */
		reth = &ohdr->u.rc.reth;
		qp->r_len = be32_to_cpu(reth->length);
		qp->r_rcv_len = 0;
		qp->r_sge.sg_list = NULL;
		if (qp->r_len != 0) {
			u32 rkey = be32_to_cpu(reth->rkey);
			u64 vaddr = get_ib_reth_vaddr(reth);
			int ok;

			/* Check rkey & NAK */
			ok = rvt_rkey_ok(qp, &qp->r_sge.sge, qp->r_len, vaddr,
					 rkey, IB_ACCESS_REMOTE_WRITE);
			if (unlikely(!ok))
				goto nack_acc;
			qp->r_sge.num_sge = 1;
		} else {
			qp->r_sge.num_sge = 0;
			qp->r_sge.sge.mr = NULL;
			qp->r_sge.sge.vaddr = NULL;
			qp->r_sge.sge.length = 0;
			qp->r_sge.sge.sge_length = 0;
		}
		if (opcode == OP(RDMA_WRITE_FIRST))
			goto send_middle;
		else if (opcode == OP(RDMA_WRITE_ONLY))
			goto no_immediate_data;
		ret = hfi1_rvt_get_rwqe(qp, 1);
		if (ret < 0)
			goto nack_op_err;
		if (!ret) {
			/* peer will send again */
			rvt_put_ss(&qp->r_sge);
			goto rnr_nak;
		}
		wc.ex.imm_data = ohdr->u.rc.imm_data;
		wc.wc_flags = IB_WC_WITH_IMM;
		goto send_last;

	case OP(RDMA_READ_REQUEST): {
		struct rvt_ack_entry *e;
		u32 len;
		u8 next;

		if (unlikely(!(qp->qp_access_flags & IB_ACCESS_REMOTE_READ)))
			goto nack_inv;
		next = qp->r_head_ack_queue + 1;
		/* s_ack_queue is size HFI1_MAX_RDMA_ATOMIC+1 so use > not >= */
		if (next > rvt_size_atomic(ib_to_rvt(qp->ibqp.device)))
			next = 0;
		spin_lock_irqsave(&qp->s_lock, flags);
		if (unlikely(next == qp->s_acked_ack_queue)) {
			if (!qp->s_ack_queue[next].sent)
				goto nack_inv_unlck;
			update_ack_queue(qp, next);
		}
		e = &qp->s_ack_queue[qp->r_head_ack_queue];
		if (e->rdma_sge.mr) {
			rvt_put_mr(e->rdma_sge.mr);
			e->rdma_sge.mr = NULL;
		}
		reth = &ohdr->u.rc.reth;
		len = be32_to_cpu(reth->length);
		if (len) {
			u32 rkey = be32_to_cpu(reth->rkey);
			u64 vaddr = get_ib_reth_vaddr(reth);
			int ok;

			/* Check rkey & NAK */
			ok = rvt_rkey_ok(qp, &e->rdma_sge, len, vaddr,
					 rkey, IB_ACCESS_REMOTE_READ);
			if (unlikely(!ok))
				goto nack_acc_unlck;
			/*
			 * Update the next expected PSN.  We add 1 later
			 * below, so only add the remainder here.
			 */
			qp->r_psn += rvt_div_mtu(qp, len - 1);
		} else {
			e->rdma_sge.mr = NULL;
			e->rdma_sge.vaddr = NULL;
			e->rdma_sge.length = 0;
			e->rdma_sge.sge_length = 0;
		}
		e->opcode = opcode;
		e->sent = 0;
		e->psn = psn;
		e->lpsn = qp->r_psn;
		/*
		 * We need to increment the MSN here instead of when we
		 * finish sending the result since a duplicate request would
		 * increment it more than once.
		 */
		qp->r_msn++;
		qp->r_psn++;
		qp->r_state = opcode;
		qp->r_nak_state = 0;
		qp->r_head_ack_queue = next;
		qpriv->r_tid_alloc = qp->r_head_ack_queue;

		/* Schedule the send engine. */
		qp->s_flags |= RVT_S_RESP_PENDING;
		hfi1_schedule_send(qp);

		spin_unlock_irqrestore(&qp->s_lock, flags);
		if (is_fecn)
			goto send_ack;
		return;
	}

	case OP(COMPARE_SWAP):
	case OP(FETCH_ADD): {
		struct ib_atomic_eth *ateth = &ohdr->u.atomic_eth;
		u64 vaddr = get_ib_ateth_vaddr(ateth);
		bool opfn = opcode == OP(COMPARE_SWAP) &&
			vaddr == HFI1_VERBS_E_ATOMIC_VADDR;
		struct rvt_ack_entry *e;
		atomic64_t *maddr;
		u64 sdata;
		u32 rkey;
		u8 next;

		if (unlikely(!(qp->qp_access_flags & IB_ACCESS_REMOTE_ATOMIC) &&
			     !opfn))
			goto nack_inv;
		next = qp->r_head_ack_queue + 1;
		if (next > rvt_size_atomic((ib_to_rvt(qp->ibqp.device))))
			next = 0;
		spin_lock_irqsave(&qp->s_lock, flags);
		if (unlikely(next == qp->s_acked_ack_queue)) {
			if (!qp->s_ack_queue[next].sent)
				goto nack_inv_unlck;
			update_ack_queue(qp, next);
		}
		e = &qp->s_ack_queue[qp->r_head_ack_queue];
		if (e->rdma_sge.mr) {
			rvt_put_mr(e->rdma_sge.mr);
			e->rdma_sge.mr = NULL;
		}
		/* Process OPFN special virtual address */
		if (opfn) {
			opfn_conn_response(qp, e, ateth);
			goto ack;
		}
		if (unlikely(vaddr & (sizeof(u64) - 1)))
			goto nack_inv_unlck;
		rkey = be32_to_cpu(ateth->rkey);
		/* Check rkey & NAK */
		if (unlikely(!rvt_rkey_ok(qp, &qp->r_sge.sge, sizeof(u64),
					  vaddr, rkey,
					  IB_ACCESS_REMOTE_ATOMIC)))
			goto nack_acc_unlck;
		/* Perform atomic OP and save result. */
		maddr = (atomic64_t *)qp->r_sge.sge.vaddr;
		sdata = get_ib_ateth_swap(ateth);
		e->atomic_data = (opcode == OP(FETCH_ADD)) ?
			(u64)atomic64_add_return(sdata, maddr) - sdata :
			(u64)cmpxchg((u64 *)qp->r_sge.sge.vaddr,
				      get_ib_ateth_compare(ateth),
				      sdata);
		rvt_put_mr(qp->r_sge.sge.mr);
		qp->r_sge.num_sge = 0;
ack:
		e->opcode = opcode;
		e->sent = 0;
		e->psn = psn;
		e->lpsn = psn;
		qp->r_msn++;
		qp->r_psn++;
		qp->r_state = opcode;
		qp->r_nak_state = 0;
		qp->r_head_ack_queue = next;
		qpriv->r_tid_alloc = qp->r_head_ack_queue;

		/* Schedule the send engine. */
		qp->s_flags |= RVT_S_RESP_PENDING;
		hfi1_schedule_send(qp);

		spin_unlock_irqrestore(&qp->s_lock, flags);
		if (is_fecn)
			goto send_ack;
		return;
	}

	default:
		/* NAK unknown opcodes. */
		goto nack_inv;
	}
	qp->r_psn++;
	qp->r_state = opcode;
	qp->r_ack_psn = psn;
	qp->r_nak_state = 0;
	/* Send an ACK if requested or required. */
	if (psn & IB_BTH_REQ_ACK) {
		if (packet->numpkt == 0) {
			rc_cancel_ack(qp);
			goto send_ack;
		}
		if (qp->r_adefered >= HFI1_PSN_CREDIT) {
			rc_cancel_ack(qp);
			goto send_ack;
		}
		if (unlikely(is_fecn)) {
			rc_cancel_ack(qp);
			goto send_ack;
		}
		qp->r_adefered++;
		rc_defered_ack(rcd, qp);
	}
	return;

rnr_nak:
	qp->r_nak_state = qp->r_min_rnr_timer | IB_RNR_NAK;
	qp->r_ack_psn = qp->r_psn;
	/* Queue RNR NAK for later */
	rc_defered_ack(rcd, qp);
	return;

nack_op_err:
	rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
	qp->r_nak_state = IB_NAK_REMOTE_OPERATIONAL_ERROR;
	qp->r_ack_psn = qp->r_psn;
	/* Queue NAK for later */
	rc_defered_ack(rcd, qp);
	return;

nack_inv_unlck:
	spin_unlock_irqrestore(&qp->s_lock, flags);
nack_inv:
	rvt_rc_error(qp, IB_WC_LOC_QP_OP_ERR);
	qp->r_nak_state = IB_NAK_INVALID_REQUEST;
	qp->r_ack_psn = qp->r_psn;
	/* Queue NAK for later */
	rc_defered_ack(rcd, qp);
	return;

nack_acc_unlck:
	spin_unlock_irqrestore(&qp->s_lock, flags);
nack_acc:
	rvt_rc_error(qp, IB_WC_LOC_PROT_ERR);
	qp->r_nak_state = IB_NAK_REMOTE_ACCESS_ERROR;
	qp->r_ack_psn = qp->r_psn;
send_ack:
	hfi1_send_rc_ack(rcd, qp, is_fecn);
}

void hfi1_rc_hdrerr(
	struct hfi1_ctxtdata *rcd,
	struct hfi1_packet *packet,
	struct rvt_qp *qp)
{
	struct hfi1_ibport *ibp = rcd_to_iport(rcd);
	int diff;
	u32 opcode;
	u32 psn;

	if (hfi1_ruc_check_hdr(ibp, packet))
		return;

	psn = ib_bth_get_psn(packet->ohdr);
	opcode = ib_bth_get_opcode(packet->ohdr);

	/* Only deal with RDMA Writes for now */
	if (opcode < IB_OPCODE_RC_RDMA_READ_RESPONSE_FIRST) {
		diff = delta_psn(psn, qp->r_psn);
		if (!qp->r_nak_state && diff >= 0) {
			ibp->rvp.n_rc_seqnak++;
			qp->r_nak_state = IB_NAK_PSN_ERROR;
			/* Use the expected PSN. */
			qp->r_ack_psn = qp->r_psn;
			/*
			 * Wait to send the sequence
			 * NAK until all packets
			 * in the receive queue have
			 * been processed.
			 * Otherwise, we end up
			 * propagating congestion.
			 */
			rc_defered_ack(rcd, qp);
		} /* Out of sequence NAK */
	} /* QP Request NAKs */
}

static u32 build_tid_rdma_write_req(struct rvt_qp *qp, struct rvt_swqe *wqe,
				    struct ib_other_headers *ohdr,
				    u32 *bth1, u32 *bth2, u32 *len)
{
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_request *req = wqe_to_tid_req(wqe);
	struct tid_rdma_params *remote;

	rcu_read_lock();
	remote = rcu_dereference(qpriv->tid_rdma.remote);
	/*
	 * Set the number of flow to be used based on negotiated
	 * parameters.
	 */
	req->n_flows = remote->max_write;
	req->state = TID_REQUEST_ACTIVE;

	KDETH_RESET(ohdr->u.tid_rdma.w_req.kdeth0, KVER, 0x1);
	KDETH_RESET(ohdr->u.tid_rdma.w_req.kdeth1, JKEY, remote->jkey);
	ohdr->u.tid_rdma.w_req.reth.vaddr =
		cpu_to_be64(wqe->rdma_wr.remote_addr + (wqe->length - *len));
	ohdr->u.tid_rdma.w_req.reth.rkey =
		cpu_to_be32(wqe->rdma_wr.rkey);
	ohdr->u.tid_rdma.w_req.reth.length = cpu_to_be32(*len);
	ohdr->u.tid_rdma.w_req.verbs_qp = cpu_to_be32(qp->remote_qpn);
	*bth1 &= ~RVT_QPN_MASK;
	*bth1 |= remote->qp;
	qp->s_state = TID_OP(WRITE_REQ);
	qp->s_flags |= RVT_S_WAIT_TID_RESP;
	*bth2 |= IB_BTH_REQ_ACK;
	*len = 0;

	rcu_read_unlock();
	return sizeof(ohdr->u.tid_rdma.w_req) / sizeof(u32);
}

static u32 build_tid_rdma_write_resp(struct rvt_qp *qp, struct rvt_ack_entry *e,
				     struct ib_other_headers *ohdr, u32 *bth1,
				     u32 bth2, u32 *len,
				     struct rvt_sge_state **ss)
{
	struct hfi1_ack_priv *epriv = e->priv;
	struct tid_rdma_request *req = &epriv->tid_req;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_flow *flow = NULL;
	u32 resp_len = 0, hdwords = 0;
	void *resp_addr = NULL;
	struct tid_rdma_params *remote;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] Processing TID_RDMA_WRITE_REQ: vaddr 0x%llx, len %u, PSN: 0x%x, 0x%x",
		  qp->ibqp.qp_num, req->vaddr, req->total_len, e->psn,
		  req->state);
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] Flows: tail %u, head %u, num %u, idx %u, cur %u, alloc %u, ack %u, comp %u, total %u",
		  qp->ibqp.qp_num, req->clear_tail, req->setup_head,
		  req->n_flows, req->flow_idx, req->cur_seg, req->alloc_seg,
		  req->ack_seg, req->comp_seg, req->total_segs);
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] r_ack_head %u s_ack_tail %u s_head %u s_tail %u r_head %u r_tail %u r_ack %u r_alloc %u",
		  qp->ibqp.qp_num,
		  qp->r_head_ack_queue, qp->s_tail_ack_queue,
		  qpriv->s_tid_head, qpriv->s_tid_tail,
		  qpriv->r_tid_head, qpriv->r_tid_tail,
		  qpriv->r_tid_ack, qpriv->r_tid_alloc);
#endif

	flow = &req->flows[req->flow_idx];
	switch (req->state) {
	default:
		/*
		 * Try to allocate resources here in case QP was queued and was
		 * later scheduled when resources became available
		 */
		hfi1_tid_write_alloc_resources(qp, false);

		/* We've already sent everything which is ready */
		if (req->cur_seg >= req->alloc_seg)
			goto done;

		/*
		 * Resources can be assigned but responses cannot be sent in
		 * rnr_nak state, till the resent request is received
		 */
		if (qpriv->rnr_nak_state == TID_RNR_NAK_SENT)
			goto done;

		req->state = TID_REQUEST_ACTIVE;
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] Segment PSNs: spsn: 0x%x, lpsn: 0x%x",
			  qp->ibqp.qp_num,
			  full_flow_psn(flow, flow->flow_state.spsn),
			  full_flow_psn(flow, flow->flow_state.lpsn));
#endif
		req->flow_idx = CIRC_NEXT(req->flow_idx, req->n_max_flows);
		hfi1_add_tid_reap_timer(qp);
		break;

	case TID_REQUEST_RESEND_ACTIVE:
	case TID_REQUEST_RESEND:
#ifdef TIDRDMA_DEBUG
		hfi1_cdbg(TIDRDMA,
			  "[QP%u] Resending RESP: spsn: 0x%x, lpsn: 0x%x",
			  qp->ibqp.qp_num,
			  full_flow_psn(flow, flow->flow_state.spsn),
			  full_flow_psn(flow, flow->flow_state.lpsn));
#endif
		req->flow_idx = CIRC_NEXT(req->flow_idx, req->n_max_flows);
		if (!CIRC_CNT(req->setup_head, req->flow_idx,
			      req->n_max_flows))
			req->state = TID_REQUEST_ACTIVE;

		hfi1_mod_tid_reap_timer(qp);
		break;
	}
	flow->flow_state.resp_ib_psn = bth2;
	resp_addr = (void *)flow->tid_entry;
	resp_len = sizeof(*flow->tid_entry) * flow->tidcnt;
	req->cur_seg++;

#if defined(TIDRDMA_DEBUG) && defined(TIDRDMA_EXTRA_DEBUG)
	dump_tid_array(resp_addr, resp_len / sizeof(u32));
#endif

	memset(&ohdr->u.tid_rdma.w_rsp, 0, sizeof(ohdr->u.tid_rdma.w_rsp));
	epriv->ss.sge.vaddr = resp_addr;
	epriv->ss.sge.sge_length = resp_len;
	epriv->ss.sge.length = epriv->ss.sge.sge_length;
	/*
	 * We can safely zero these out. Since the first SGE covers the
	 * entire packet, nothing else should even look at the MR.
	 */
	epriv->ss.sge.mr = NULL;
	epriv->ss.sge.m = 0;
	epriv->ss.sge.n = 0;

	epriv->ss.sg_list = NULL;
	epriv->ss.total_len = epriv->ss.sge.sge_length;
	epriv->ss.num_sge = 1;

	*ss = &epriv->ss;
	*len = epriv->ss.total_len;

	/* Construct the TID RDMA WRITE RESP packet header */
	rcu_read_lock();
	remote = rcu_dereference(qpriv->tid_rdma.remote);

	KDETH_RESET(ohdr->u.tid_rdma.w_rsp.kdeth0, KVER, 0x1);
	KDETH_RESET(ohdr->u.tid_rdma.w_rsp.kdeth1, JKEY, remote->jkey);
	ohdr->u.tid_rdma.w_rsp.aeth = rvt_compute_aeth(qp);
	ohdr->u.tid_rdma.w_rsp.tid_flow_psn =
		cpu_to_be32((flow->flow_state.generation <<
			     HFI1_KDETH_BTH_SEQ_SHIFT) |
			    (flow->flow_state.spsn &
			     HFI1_KDETH_BTH_SEQ_MASK));
	ohdr->u.tid_rdma.w_rsp.tid_flow_qp =
		cpu_to_be32(qpriv->tid_rdma.local.qp |
			    ((flow->idx & TID_RDMA_DESTQP_FLOW_MASK) <<
			     TID_RDMA_DESTQP_FLOW_SHIFT) |
			    qpriv->rcd->ctxt);
	ohdr->u.tid_rdma.w_rsp.verbs_qp = cpu_to_be32(qp->remote_qpn);
	*bth1 = remote->qp;
	rcu_read_unlock();
	hdwords = sizeof(ohdr->u.tid_rdma.w_rsp) / sizeof(u32);
	qpriv->pending_tid_w_segs++;
done:
	return hdwords;
}

static bool build_tid_rdma_packet(struct rvt_swqe *wqe,
				  struct ib_other_headers *ohdr,
				  u32 *bth1, u32 *bth2, u32 *len)
{
	struct tid_rdma_request *req = wqe_to_tid_req(wqe);
	struct tid_rdma_flow *flow = &req->flows[READ_ONCE(req->clear_tail)];
	struct tid_rdma_params *remote;
	struct rvt_qp *qp = req->qp;
	struct hfi1_qp_priv *qpriv = qp->priv;
	u32 tidentry = flow->tid_entry[flow->tid_idx];
	u32 tidlen = EXP_TID_GET(tidentry, LEN) << PAGE_SHIFT;
	struct tid_rdma_write_data *wd = &ohdr->u.tid_rdma.w_data;
	u32 next_offset, om = KDETH_OM_LARGE;
	bool last_pkt;

	if (!tidlen) {
		hfi1_send_complete(qp, wqe, IB_WC_REM_INV_RD_REQ_ERR);
		rvt_error_qp(qp, IB_WC_REM_INV_RD_REQ_ERR);
	}

	*len = min_t(u32, qp->pmtu, tidlen - flow->tid_offset);
	flow->sent += *len;
	next_offset = flow->tid_offset + *len;
	last_pkt = (flow->tid_idx == (flow->tidcnt - 1) &&
		    next_offset >= tidlen) || (flow->sent >= flow->length);
#if defined(TIDRDMA_DEBUG) && defined(TIDRDMA_EXTRA_DEBUG)
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] TID: idx: %u, ctrl: 0x%x, tidlen: %u, offset: %u, next: %u, sent: %u, len: %u, tidx: %u, cnt: %u, len: %u, last: %u",
		  qp->ibqp.qp_num, EXP_TID_GET(tidentry, IDX),
		  EXP_TID_GET(tidentry, CTRL), tidlen, flow->tid_offset,
		  next_offset, flow->sent, flow->length, flow->tid_idx,
		  flow->tidcnt, *len, last_pkt);
#endif

	rcu_read_lock();
	remote = rcu_dereference(qpriv->tid_rdma.remote);
	KDETH_RESET(wd->kdeth0, KVER, 0x1);
	KDETH_SET(wd->kdeth0, SH, !last_pkt);
	KDETH_SET(wd->kdeth0, TIDCTRL, EXP_TID_GET(tidentry, CTRL));
	KDETH_SET(wd->kdeth0, TID, EXP_TID_GET(tidentry, IDX));
	KDETH_SET(wd->kdeth0, OM, om == KDETH_OM_LARGE);
	KDETH_SET(wd->kdeth0, OFFSET, flow->tid_offset / om);
	KDETH_RESET(wd->kdeth1, JKEY, remote->jkey);
	wd->verbs_qp = cpu_to_be32(qp->remote_qpn);
	rcu_read_unlock();

	*bth1 = flow->tid_qpn;
	*bth2 = mask_psn(((flow->flow_state.spsn + flow->pkt++) &
			 HFI1_KDETH_BTH_SEQ_MASK) |
			 (flow->flow_state.generation <<
			  HFI1_KDETH_BTH_SEQ_SHIFT));
	if (last_pkt) {
		/* PSNs are zero-based, so +1 to count number of packets */
		if (flow->flow_state.lpsn + 1 +
		    rvt_div_round_up_mtu(qp, req->seg_len) >
		    MAX_TID_FLOW_PSN)
			req->state = TID_REQUEST_SYNC;
		*bth2 |= IB_BTH_REQ_ACK;
	}

	if (next_offset >= tidlen) {
		flow->tid_offset = 0;
		flow->tid_idx++;
	} else {
		flow->tid_offset = next_offset;
	}
	return last_pkt;
}

static u32 build_tid_rdma_write_ack(struct rvt_qp *qp, struct rvt_ack_entry *e,
				    struct ib_other_headers *ohdr, u16 iflow,
				    u32 *bth1, u32 *bth2)
{
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_request *req = ack_to_tid_req(e);
	struct tid_rdma_flow *flow = &req->flows[iflow];
	struct tid_rdma_params *remote;

	rcu_read_lock();
	remote = rcu_dereference(qpriv->tid_rdma.remote);
	KDETH_RESET(ohdr->u.tid_rdma.ack.kdeth1, JKEY, remote->jkey);
	ohdr->u.tid_rdma.ack.verbs_qp = cpu_to_be32(qp->remote_qpn);
	*bth1 = remote->qp;
	rcu_read_unlock();

	if (qpriv->s_nak_state) {
		*bth2 = mask_psn(qpriv->s_nak_psn);
		ohdr->u.tid_rdma.ack.aeth =
			cpu_to_be32((qp->r_msn & IB_MSN_MASK) |
				    (qpriv->s_nak_state <<
				     IB_AETH_CREDIT_SHIFT));
	} else {
		*bth2 = mask_psn((flow->flow_state.generation <<
				  HFI1_KDETH_BTH_SEQ_SHIFT) |
				 (flow->flow_state.lpsn &
				  HFI1_KDETH_BTH_SEQ_MASK));
		ohdr->u.tid_rdma.ack.aeth = rvt_compute_aeth(qp);
	}
	KDETH_RESET(ohdr->u.tid_rdma.ack.kdeth0, KVER, 0x1);
	ohdr->u.tid_rdma.ack.tid_flow_qp =
		cpu_to_be32(qpriv->tid_rdma.local.qp |
			    ((flow->idx & TID_RDMA_DESTQP_FLOW_MASK) <<
			     TID_RDMA_DESTQP_FLOW_SHIFT) |
			    qpriv->rcd->ctxt);
	ohdr->u.tid_rdma.ack.verbs_psn =
		cpu_to_be32(flow->flow_state.resp_ib_psn);

	return sizeof(ohdr->u.tid_rdma.ack) / sizeof(u32);
}

/*
 * "Rewind" the TID request information.
 * This means that we reset the state back to ACTIVE,
 * find the proper flow, set the flow index to that flow,
 * and reset the flow information.
 */
static void tid_rdma_restart_req(struct rvt_qp *qp, struct rvt_swqe *wqe,
				 u32 *bth2)
{
	struct tid_rdma_request *req = wqe_to_tid_req(wqe);
	struct tid_rdma_flow *flow;
	struct hfi1_qp_priv *qpriv = qp->priv;
	int diff;
	u32 tididx = 0, i;
	u16 fidx;

	if (wqe->wr.opcode == IB_WR_TID_RDMA_READ) {
		*bth2 = mask_psn(qp->s_psn);
		flow = find_flow_ib(req, *bth2, &fidx);
		if (!flow) {
#if defined(TIDRDMA_DEBUG) && defined(TIDRDMA_EXTRA_DEBUG)
			hfi1_cdbg(TIDRDMA,
				  "!!!!!!!!!! Could not find flow to restart");
			dump_flows(req);
#endif
			return;
		}
	} else {
		fidx = req->acked_tail;
		flow = &req->flows[fidx];
		*bth2 = req->r_ack_psn;
	}

	if (wqe->wr.opcode == IB_WR_TID_RDMA_READ)
		diff = delta_psn(*bth2, flow->flow_state.ib_spsn);
	else
		diff = delta_psn(*bth2, full_flow_psn(flow,
						      flow->flow_state.spsn));

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "[QP%u] PSN 0x%x, fidx %u 0x%x-0x%x, diff %u",
		  qp->ibqp.qp_num, *bth2, fidx,
		  full_flow_psn(flow, flow->flow_state.spsn),
		  full_flow_psn(flow, flow->flow_state.lpsn), diff);
#endif
	flow->sent = 0;
	flow->pkt = 0;
	flow->tid_idx = 0;
	flow->tid_offset = 0;
	if (diff) {
		for (tididx = 0; tididx < flow->tidcnt; tididx++) {
			u32 tidentry = flow->tid_entry[tididx], tidlen,
				tidnpkts, npkts;

			flow->tid_offset = 0;
			tidlen = EXP_TID_GET(tidentry, LEN) * PAGE_SIZE;
			tidnpkts = rvt_div_round_up_mtu(qp, tidlen);
			npkts = min_t(u32, diff, tidnpkts);
			flow->pkt += npkts;
			flow->sent += (npkts == tidnpkts ? tidlen :
				       npkts * qp->pmtu);
			flow->tid_offset += npkts * qp->pmtu;
			diff -= npkts;
			if (!diff)
				break;
		}
	}
	if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE)
		rvt_skip_sge(&qpriv->tid_ss, (req->cur_seg * req->seg_len) +
			     flow->sent, 0);

	if (flow->tid_offset == EXP_TID_GET(flow->tid_entry[tididx], LEN) *
	    PAGE_SIZE) {
		tididx++;
		flow->tid_offset = 0;
	}
	flow->tid_idx = tididx;
	if (wqe->wr.opcode == IB_WR_TID_RDMA_READ)
		/* Move flow_idx to correct index */
		smp_store_release(&req->flow_idx, fidx);
	else
		/* Move clear_tail to correct index */
		smp_store_release(&req->clear_tail, fidx);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] flow: sent %u pkt %u tid_idx %u tid_offset %u, acked_tail %u, clear_tail %u setup_head %u",
		  qp->ibqp.qp_num, flow->sent, flow->pkt, flow->tid_idx,
		  flow->tid_offset, req->acked_tail, req->clear_tail,
		  req->setup_head);
#endif
	req->state = TID_REQUEST_ACTIVE;
	if (wqe->wr.opcode == IB_WR_TID_RDMA_WRITE) {
		/* Reset all the flows that we are going to resend */
		fidx = CIRC_NEXT(fidx, req->n_max_flows);
		i = qpriv->s_tid_tail;
		do {
			for (;
			      CIRC_CNT(req->setup_head, fidx, req->n_max_flows);
			      fidx = CIRC_NEXT(fidx, req->n_max_flows)) {
				req->flows[fidx].sent = 0;
				req->flows[fidx].pkt = 0;
				req->flows[fidx].tid_idx = 0;
				req->flows[fidx].tid_offset = 0;
			}
			if (i == qpriv->s_tid_cur)
				break;
			do {
				i = (++i == qp->s_size ? 0 : i);
				wqe = rvt_get_swqe_ptr(qp, i);
			} while (wqe->wr.opcode != IB_WR_TID_RDMA_WRITE);
			req = wqe_to_tid_req(wqe);
			req->cur_seg = req->ack_seg;
			fidx = req->acked_tail;
			/* Pull req->clear_tail back */
			smp_store_release(&req->clear_tail, fidx);
		} while (1);
	}

}

static u32 build_tid_rdma_read_packet(struct rvt_swqe *wqe,
				      struct ib_other_headers *ohdr,
				      u32 *bth1, u32 *bth2, u32 *len)
{
	struct tid_rdma_request *req = wqe_to_tid_req(wqe);
	struct tid_rdma_flow *flow = &req->flows[READ_ONCE(req->flow_idx)];
	struct rvt_qp *qp = req->qp;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct hfi1_swqe_priv *wpriv = wqe->priv;
	struct tid_rdma_read_req *rreq = &ohdr->u.tid_rdma.r_req;
	struct tid_rdma_params *remote;
	u32 req_len = 0;
	void *req_addr = NULL;

	/* This is the IB psn used to send the request */
	*bth2 = mask_psn(flow->flow_state.ib_spsn + flow->pkt);

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] TID: sent: %u, flen: %u, tidx: %u, cnt: %u, len: %u",
		  qp->ibqp.qp_num, flow->sent, flow->length, flow->tid_idx,
		  flow->tidcnt, *len);
#endif

	/* TID Entries for TID RDMA READ payload */
	req_addr = &flow->tid_entry[flow->tid_idx];
	req_len = sizeof(*flow->tid_entry) * (flow->tidcnt - flow->tid_idx);

	memset(&ohdr->u.tid_rdma.r_req, 0, sizeof(ohdr->u.tid_rdma.r_req));
	wpriv->ss.sge.vaddr = req_addr;
	wpriv->ss.sge.sge_length = req_len;
	wpriv->ss.sge.length = wpriv->ss.sge.sge_length;
	/*
	 * We can safely zero these out. Since the first SGE covers the
	 * entire packet, nothing else should even look at the MR.
	 */
	wpriv->ss.sge.mr = NULL;
	wpriv->ss.sge.m = 0;
	wpriv->ss.sge.n = 0;

	wpriv->ss.sg_list = NULL;
	wpriv->ss.total_len = wpriv->ss.sge.sge_length;
	wpriv->ss.num_sge = 1;

	/* Construct the TID RDMA READ REQ packet header */
	rcu_read_lock();
	remote = rcu_dereference(qpriv->tid_rdma.remote);

	KDETH_RESET(rreq->kdeth0, KVER, 0x1);
	KDETH_RESET(rreq->kdeth1, JKEY, remote->jkey);
	rreq->reth.vaddr = cpu_to_be64(wqe->rdma_wr.remote_addr +
			    req->cur_seg * req->seg_len + flow->sent);
	rreq->reth.rkey = cpu_to_be32(wqe->rdma_wr.rkey);
	rreq->reth.length = cpu_to_be32(*len);
	rreq->tid_flow_psn =
		cpu_to_be32((flow->flow_state.generation <<
			     HFI1_KDETH_BTH_SEQ_SHIFT) |
			    ((flow->flow_state.spsn + flow->pkt)&
			     HFI1_KDETH_BTH_SEQ_MASK));
	rreq->tid_flow_qp =
		cpu_to_be32(qpriv->tid_rdma.local.qp |
			    ((flow->idx & TID_RDMA_DESTQP_FLOW_MASK) <<
			     TID_RDMA_DESTQP_FLOW_SHIFT) |
			    qpriv->rcd->ctxt);
	rreq->verbs_qp = cpu_to_be32(qp->remote_qpn);
	*bth1 &= ~RVT_QPN_MASK;
	*bth1 |= remote->qp;
	*bth2 |= IB_BTH_REQ_ACK;
	rcu_read_unlock();

	/* We are done with this segment */
	flow->sent += *len;
	req->cur_seg++;
	qp->s_state = TID_OP(READ_REQ);
	req->ack_pending++;
	req->flow_idx = (req->flow_idx + 1) & (req->n_max_flows - 1);
	qpriv->pending_tid_r_segs++;
	qp->s_num_rd_atomic++;

	/* Set the TID RDMA READ request payload size */
	*len = req_len;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA, "BTH1 0x%x BTH2 0x%x KDETH0 0x%x KDETH1 0x%x",
		  *bth1, *bth2, ohdr->u.tid_rdma.r_req.kdeth0,
		  ohdr->u.tid_rdma.r_req.kdeth1);
#endif
	return sizeof(ohdr->u.tid_rdma.r_req) / sizeof(u32);
}

/*
 * @len: contains the data length to read upon entry and the read request
 *       payload length upon exit.
 */ 
static u32 build_tid_rdma_read_req(struct rvt_qp *qp, struct rvt_swqe *wqe,
				   struct ib_other_headers *ohdr,
				   u32 *bth1, u32 *bth2, u32 *len)
	__must_hold(&qp->s_lock)
{
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_request *req = wqe_to_tid_req(wqe);
	struct tid_rdma_flow *flow = NULL;
	u32 hdwords = 0;
	bool last;
	bool retry = true;
	u32 npkts = rvt_div_round_up_mtu(qp, *len);

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "Tot_seg 0x%u, cur_seg %u, comp_seg %u PSN: 0x%x, state 0x%x",
		  req->total_segs, req->cur_seg, req->comp_seg, wqe->psn,
		  req->state);
#endif

	/* Attempt to allocate resources */
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "Flows: tail: %u, head: %u, num: %u, idx: %u",
		  req->clear_tail, req->setup_head, req->n_flows,
		  req->flow_idx);
#endif

	/*
	* Check sync conditions. Make sure that there are no pending
	* segments before freeing the flow.
	*/
sync_check:
	if (req->state == TID_REQUEST_SYNC) {
		if (qpriv->pending_tid_r_segs)
			goto done;

		hfi1_kern_clear_hw_flow(req->rcd, qp);
		req->state = TID_REQUEST_ACTIVE;
	}

	/*
	 * If the request for this segment is resent, the tid resources should
	 * have been allocated before. In this case, req->flow_idx should
	 * fall behind req->setup_head.
	 */
	if (READ_ONCE(req->flow_idx) == req->setup_head) {
		retry = false;
		if (req->state == TID_REQUEST_RESEND) {
			/*
			 * This is the first new segment for a request whose
			 * earlier segments have been re-sent. We need to
			 * set up the sge pointer correctly.
			 */
			restart_sge(&qp->s_sge, wqe, req->s_next_psn,
				    qp->pmtu);
			req->isge = 0;
			req->state = TID_REQUEST_ACTIVE;
		}

		/* Check sync */
		if ((qpriv->flow_state.psn + npkts) > MAX_TID_FLOW_PSN) {
			req->state = TID_REQUEST_SYNC;
			goto sync_check;
		}

		/* Allocate the flow if not yet */
		if (hfi1_kern_setup_hw_flow(qpriv->rcd, qp))
			goto done;

		/*
		 * The following call will advance req->setup_head after
		 * allocating the tid entries.
		 */
		if (hfi1_kern_exp_rcv_setup(req, &qp->s_sge, &last)) {
			req->state = TID_REQUEST_QUEUED;

			/*
			 * We don't have resources for this segment. The QP has
			 * already been queued.
			 */
			goto done;
		}
	}

	/* req->flow_idx should only be one slot behind req->setup_head */
	flow = &req->flows[req->flow_idx];
	flow->pkt = 0;
	flow->tid_idx = 0;
	flow->sent = 0;
	if (!retry) {
		/* Set the first and last IB PSN for the flow in use.*/
		flow->flow_state.ib_spsn = req->s_next_psn;
		flow->flow_state.ib_lpsn =
			flow->flow_state.ib_spsn + flow->npkts - 1;
	}

	/* Calculate the next segment start psn.*/
	req->s_next_psn += flow->npkts;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "Segment PSNs: sib: 0x%x, lib: 0x%x, sflow: 0x%x, lflow: 0x%x",
		  flow->flow_state.ib_spsn, flow->flow_state.ib_lpsn,
		  full_flow_psn(flow, flow->flow_state.spsn),
		  full_flow_psn(flow, flow->flow_state.lpsn));
#endif

	/* Build the packet header */
	hdwords = build_tid_rdma_read_packet(wqe, ohdr, bth1, bth2, len);
done:
	return hdwords;
}

static u32 build_tid_rdma_read_resp(struct rvt_qp *qp, struct rvt_ack_entry *e,
				    struct ib_other_headers *ohdr, u32 *bth0,
				    u32 *bth1, u32 *bth2, u32 *len, bool *last)
{
	struct hfi1_ack_priv *epriv = e->priv;
	struct tid_rdma_request *req = &epriv->tid_req;
	struct hfi1_qp_priv *qpriv = qp->priv;
	struct tid_rdma_flow *flow = &req->flows[READ_ONCE(req->clear_tail)];
	u32 tidentry = flow->tid_entry[flow->tid_idx];
	u32 tidlen = EXP_TID_GET(tidentry, LEN) << PAGE_SHIFT;
	struct tid_rdma_read_resp *resp = &ohdr->u.tid_rdma.r_rsp;
	u32 next_offset, om = KDETH_OM_LARGE;
	bool last_pkt;
	u32 hdwords = 0;
	struct tid_rdma_params *remote;

#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] READ_RESP: vaddr 0x%llx, len %u, PSN: 0x%x, st 0x%x",
		  qp->ibqp.qp_num, req->vaddr, req->total_len, e->psn,
		  req->state);
#endif

	*len = min_t(u32, qp->pmtu, tidlen - flow->tid_offset);
	flow->sent += *len;
	next_offset = flow->tid_offset + *len;
	last_pkt = (flow->sent >= flow->length);
#ifdef TIDRDMA_DEBUG
	hfi1_cdbg(TIDRDMA,
		  "[QP%u] TID: idx: %u, ctrl: 0x%x, tidlen: %u, offset: %u, next: %u, sent: %u, flen: %u, tidx: %u, cnt: %u, len: %u, last: %u",
		  qp->ibqp.qp_num, EXP_TID_GET(tidentry, IDX),
		  EXP_TID_GET(tidentry, CTRL), tidlen, flow->tid_offset,
		  next_offset, flow->sent, flow->length, flow->tid_idx,
		  flow->tidcnt, *len, last_pkt);
#endif

	rcu_read_lock();
	remote = rcu_dereference(qpriv->tid_rdma.remote);
	if (!remote) {
		rcu_read_unlock();
		goto done;
	}
	KDETH_RESET(resp->kdeth0, KVER, 0x1);
	KDETH_SET(resp->kdeth0, SH, !last_pkt);
	KDETH_SET(resp->kdeth0, TIDCTRL, EXP_TID_GET(tidentry, CTRL));
	KDETH_SET(resp->kdeth0, TID, EXP_TID_GET(tidentry, IDX));
	KDETH_SET(resp->kdeth0, OM, om == KDETH_OM_LARGE);
	KDETH_SET(resp->kdeth0, OFFSET, flow->tid_offset / om);
	KDETH_RESET(resp->kdeth1, JKEY, remote->jkey);
	resp->verbs_qp = cpu_to_be32(qp->remote_qpn);
	rcu_read_unlock();

	resp->aeth = rvt_compute_aeth(qp);
	resp->verbs_psn = cpu_to_be32(mask_psn(flow->flow_state.ib_spsn +
					       flow->pkt));

	*bth0 = TID_OP(READ_RESP) << 24;
	*bth1 = flow->tid_qpn;
	*bth2 = mask_psn(((flow->flow_state.spsn + flow->pkt++) &
			 HFI1_KDETH_BTH_SEQ_MASK) |
			 (flow->flow_state.generation <<
			  HFI1_KDETH_BTH_SEQ_SHIFT));
	*last = last_pkt;
	if (last_pkt)
		smp_store_release(&req->clear_tail,
			  (req->clear_tail + 1) & (req->n_max_flows - 1));

	if (next_offset >= tidlen) {
		flow->tid_offset = 0;
		flow->tid_idx++;
	} else {
		flow->tid_offset = next_offset;
	}

	hdwords = sizeof(ohdr->u.tid_rdma.r_rsp) / sizeof(u32);

done:
	return hdwords;
}
