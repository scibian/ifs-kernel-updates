/*
 * Copyright(c) 2016 Intel Corporation.
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
#include "hfi.h"
#include "trace.h"
#include "qp.h"

struct hfi1_opfn_type {
	bool (*request)(struct rvt_qp *, u64 *);
	bool (*response)(struct rvt_qp *, u64 *);
	bool (*reply)(struct rvt_qp *, u64);
	void (*error)(struct rvt_qp *);
};

struct hfi1_opfn_type hfi1_opfn_handlers[STL_VERBS_EXTD_MAX] = {
	{ 0 },
	{ tid_rdma_conn_req, tid_rdma_conn_resp, tid_rdma_conn_reply,
	  tid_rdma_conn_error },
};

void opfn_conn_request(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct ib_atomic_wr wr;
	struct ib_send_wr *bad_send_wr;
	u16 mask, capcode;
	struct hfi1_opfn_type *extd;
	u64 data;
	unsigned long flags;
	int ret = 0;

	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);

	spin_lock_irqsave(&priv->opfn.lock, flags);
	/*
	 * Exit if the extended bit is not set, or if nothing is requested, or
	 * if we have completed all requests, or if a previous request is in
	 * progress
	 */
	if (!priv->opfn.extended || !priv->opfn.requested ||
	    priv->opfn.requested == priv->opfn.completed || priv->opfn.curr)
		goto done;

	mask = priv->opfn.requested & ~priv->opfn.completed;
	capcode = ilog2(mask & ~(mask - 1)) + 1;
	if (capcode >= STL_VERBS_EXTD_MAX) {
		priv->opfn.completed |= OPFN_CODE(capcode);
		goto done;
	}

	extd = &hfi1_opfn_handlers[capcode];
	if (!extd || !extd->request || !extd->request(qp, &data)) {
		/*
		 * Either there is no handler for this capability or the request
		 * packet could not be generated. Either way, mark it as done so
		 * we don't keep attempting to complete it.
		 */
		priv->opfn.completed |= OPFN_CODE(capcode);
		goto done;
	}

	hfi1_cdbg(OPFN, "QP%u(0x%x) send ID%u data 0x%llx",
		  qp->ibqp.qp_num, qp->state, capcode, data);
	data = (data & ~0xf) | capcode;

	memset(&wr, 0, sizeof(wr));
	wr.wr.opcode = IB_WR_OPFN;
	wr.remote_addr = HFI1_VERBS_E_ATOMIC_VADDR;
	wr.compare_add = data;

	priv->opfn.curr = capcode;	/* A new request is now in progress */
	/* Drop opfn.lock before calling ib_post_send() */
	spin_unlock_irqrestore(&priv->opfn.lock, flags);

	ret = ib_post_send(&qp->ibqp, &wr.wr, &bad_send_wr);
	if (ret)
		goto err;
	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);
	return;
err:
	hfi1_cdbg(OPFN, "ib_post_send failed ret %d", ret);
	spin_lock_irqsave(&priv->opfn.lock, flags);
	/*
	 * In case of an unexpected error return from ib_post_send
	 * clear opfn.curr and reschedule to try again
	 */
	priv->opfn.curr = STL_VERBS_EXTD_NONE;
	opfn_schedule_conn_request(qp);
done:
	spin_unlock_irqrestore(&priv->opfn.lock, flags);
}

void opfn_send_conn_request(struct work_struct *work)
{
	struct hfi1_opfn_data *od;
	struct hfi1_qp_priv *qpriv;

	od = container_of(work, struct hfi1_opfn_data, opfn_work);
	qpriv = container_of(od, struct hfi1_qp_priv, opfn);

	opfn_conn_request(qpriv->owner);
}

/*
 * When QP s_lock is held in the caller, the OPFN request must be scheduled
 * to a different workqueue to avoid double locking QP s_lock in call to
 * ib_post_send in opfn_conn_request
 */
void opfn_schedule_conn_request(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;

	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);

	/* XXX: should we be scheduling to a different workqueue? */
	schedule_work(&priv->opfn.opfn_work);
}

void opfn_conn_response(struct rvt_qp *qp, struct rvt_ack_entry *e,
			struct ib_atomic_eth *ateth)
{
	struct hfi1_qp_priv *priv = qp->priv;
	u64 data = be64_to_cpu(ateth->compare_data);
	struct hfi1_opfn_type *extd;
	u8 capcode;
	unsigned long flags;

	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);
	capcode = data & 0xf;
	hfi1_cdbg(OPFN, "QP%u(0x%x) resp ID%u data 0x%llx",
		  qp->ibqp.qp_num, qp->state, capcode, data);
	if (!capcode || capcode >= STL_VERBS_EXTD_MAX)
		return;

	extd = &hfi1_opfn_handlers[capcode];

	if (!extd || !extd->response) {
		e->atomic_data = capcode;
		return;
	}

	spin_lock_irqsave(&priv->opfn.lock, flags);
	if (priv->opfn.completed & OPFN_CODE(capcode)) {
		/*
		 * We are receiving a request for a feature that has already
		 * been negotiated. This may mean that the other side has reset
		 */
		priv->opfn.completed &= ~OPFN_CODE(capcode);
		if (extd->error)
			extd->error(qp);
	}

	if (extd->response(qp, &data))
		priv->opfn.completed |= OPFN_CODE(capcode);
	e->atomic_data = (data & ~0xf) | capcode;
	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);
	spin_unlock_irqrestore(&priv->opfn.lock, flags);
}

void opfn_conn_reply(struct rvt_qp *qp, u64 data)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_opfn_type *extd;
	u8 capcode;
	unsigned long flags;

	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);
	capcode = data & 0xf;
	hfi1_cdbg(OPFN, "QP%u(0x%x) rcv'ed ID%u data 0x%llx",
		  qp->ibqp.qp_num, qp->state, capcode, data);
	if (!capcode || capcode >= STL_VERBS_EXTD_MAX)
		return;

	spin_lock_irqsave(&priv->opfn.lock, flags);
	/*
	 * Either there is no previous request or the reply is not for the
	 * current request
	 */
	if (!priv->opfn.curr || capcode != priv->opfn.curr)
		goto done;

	extd = &hfi1_opfn_handlers[capcode];

	if (!extd || !extd->reply)
		goto clear;

	if (extd->reply(qp, data))
		priv->opfn.completed |= OPFN_CODE(capcode);
clear:
	/*
	 * Clear opfn.curr to indicate that the previous request is no longer in
	 * progress
	 */
	priv->opfn.curr = STL_VERBS_EXTD_NONE;
	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);
done:
	spin_unlock_irqrestore(&priv->opfn.lock, flags);
}

void opfn_conn_error(struct rvt_qp *qp)
{
	struct hfi1_qp_priv *priv = qp->priv;
	struct hfi1_opfn_type *extd = NULL;
	unsigned long flags;
	u16 capcode;

	hfi1_cdbg(OPFN, "requested=0x%x, completed=0x%x, current=0x%x",
		  priv->opfn.requested, priv->opfn.completed, priv->opfn.curr);
	/*
	 * The QP has gone into the Error state. We have to invalidate all
	 * negotiated feature, including the one in progress (if any). The RC
	 * QP handling will clean the WQE for the connection request.
	 */
	hfi1_cdbg(OPFN, "QP%u(0x%x) error", qp->ibqp.qp_num, qp->state);
	spin_lock_irqsave(&priv->opfn.lock, flags);
	while (priv->opfn.completed) {
		capcode = priv->opfn.completed & ~(priv->opfn.completed - 1);
		extd = &hfi1_opfn_handlers[ilog2(capcode) + 1];
		if (extd->error)
			extd->error(qp);
		priv->opfn.completed &= ~OPFN_CODE(capcode);
	}
	priv->opfn.extended = false;
	priv->opfn.requested = 0;
	priv->opfn.curr = STL_VERBS_EXTD_NONE;
	spin_unlock_irqrestore(&priv->opfn.lock, flags);
}
