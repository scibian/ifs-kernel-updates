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

#ifndef HFI1_TID_RDMA_H
#define HFI1_TID_RDMA_H

#define TID_RDMA_DEFAULT_CTXT		0x0	  /* FIXME: Needs krcvqs == 1 */

#include <linux/circ_buf.h>
/* Add a convenience helper */
#define CIRC_ADD(val, add, size) (((val) + (add)) & ((size) - 1))
#define CIRC_NEXT(val, size) CIRC_ADD(val, 1, size)
#define CIRC_PREV(val, size) CIRC_ADD(val, -1, size)

/*
 * J_KEY for kernel contexts when TID RDMA is used.
 * See generate_jkey() in hfi.h for more information.
 */
#define TID_RDMA_JKEY                   32
#define TID_RDMA_LOCAL_KDETH_QP_BASE	0x800000
#define TID_RDMA_MIN_SEGMENT_SIZE	BIT(18)   /* 256 KiB (for now) */
#define TID_RDMA_MAX_SEGMENT_SIZE	BIT(18)   /* 256 KiB (for now) */
/*
 * Maximum number of receive context flows to be used for TID RDMA
 * READ and WRITE requests from any QP.
 */
#define TID_RDMA_MAX_READ_FLOWS		(RXE_NUM_TID_FLOWS / 2)
#define TID_RDMA_MAX_WRITE_FLOWS			\
	(RXE_NUM_TID_FLOWS - TID_RDMA_MAX_READ_FLOWS)
/* Maximum number of segments in flight per QP request. */
#define TID_RDMA_MAX_READ_SEGS_PER_REQ  6
#define TID_RDMA_MAX_WRITE_SEGS_PER_REQ 4
#define TID_RDMA_MAX_READ_SEGS          6

/*
 * Bit definitions for priv->s_flags.
 * These bit flags overload the bit flags defined for the QP's s_flags.
 * Due to the fact that these bit fields are used only for the QP priv
 * s_flags, there are no collisions.
 *
 * HFI1_S_TID_WAIT_INTERLCK - QP is waiting for requester interlock
 * HFI1_R_TID_WAIT_INTERLCK - QP is waiting for responder interlock
 */
#define HFI1_S_TID_BUSY_SET       BIT(0)
/* BIT(1) reserved for RVT_S_BUSY. */
#define HFI1_R_TID_RSC_TIMER      BIT(2)
/* BIT(3) reserved for RVT_S_RESP_PENDING. */
/* BIT(4) reserved for RVT_S_ACK_PENDING. */
#define HFI1_S_TID_WAIT_INTERLCK  BIT(5)
#define HFI1_R_TID_WAIT_INTERLCK  BIT(6)
/* BIT(7) - BIT(15) reserved for RVT_S_WAIT_*. */
#define HFI1_S_TID_RETRY_TIMER    BIT(16)
#define HFI1_R_TID_SW_PSN         BIT(17)
/* BIT(18) reserved for RVT_S_ECN. */
/* BIT(19) reserved for RVT_S_WAIT_TID_RESP */
/* BIT(20) reserved for RVT_S_WAIT_TID_SPACE */

/*
 * Timeout factor for TID RDMA ACK retry timer.
 * It should be set to 1 in the future.
 */
#define HFI1_TID_RETRY_TO_FACTOR       256

/*
 * Unlike regular IB RDMA VERBS, which do not require an entry
 * in the s_ack_queue, TID RDMA WRITE requests do because they
 * generate responses.
 * Therefore, the s_ack_queue needs to be extended by a certain
 * amount. The key point is that the queue needs to be extended
 * without letting the "user" know so they user doesn't end up
 * using these extra entries.
 */
#define HFI1_TID_RDMA_WRITE_CNT 8

#define TID_RDMA_DESTQP_FLOW_SHIFT      11
#define TID_RDMA_DESTQP_FLOW_MASK       0x1f

struct tid_rdma_params {
	struct rcu_head rcu_head;
	u32 qp;
	u32 max_len;
	u16 jkey;
	u8 max_read;
	u8 max_write;
	u8 timeout;
};

struct tid_rdma_qp_params {
	u8 n_read;
	u8 n_write;
	struct work_struct trigger_work;
	struct tid_rdma_params local;
	struct tid_rdma_params __rcu *remote;
};

struct tid_flow_state {
	u32 index;
	u32 last_index;
	u32 generation;
	u32 psn;
	u32 flags;
	u32 r_next_psn;      /* next PSN to be received (in TID space) */
};

/*
 * TID_RDMA_BLOCK_SIZE is the amount of data that a single TID entry
 * can accommodate
 */
#define TID_RDMA_BLOCK_SIZE	PAGE_SIZE
#define TID_RDMA_BLOCKS_PER_SEGMENT \
	(TID_RDMA_SEGMENT_SIZE / TID_RDMA_BLOCK_SIZE)

#define TID_FLOW_SW_PSN BIT(0)
#define TID_FLOW_SEQ_NAK_SENT BIT(1)

#ifdef CONFIG_HFI1_VERBS_31BIT_PSN
#define GENERATION_MASK 0xFFFFF
#else
#define GENERATION_MASK 0x1FFF
#endif

static inline u32 mask_generation(u32 a)
{
	return a & GENERATION_MASK;
}

/* Maximum number of packets withing a flow generation. */
#define MAX_TID_FLOW_PSN BIT(HFI1_KDETH_BTH_SEQ_SHIFT)

/* Reserved generation value to set to unused flows for kernel contexts */
#define KERN_GENERATION_RESERVED mask_generation(U32_MAX)

enum tid_rdma_req_state {
	TID_REQUEST_INACTIVE = 0,
	TID_REQUEST_INIT,
	TID_REQUEST_ACTIVE,
	TID_REQUEST_RESEND,
	TID_REQUEST_RESEND_ACTIVE,
	TID_REQUEST_QUEUED,
	TID_REQUEST_SYNC,
	TID_REQUEST_RNR_NAK,
	TID_REQUEST_COMPLETE,
};

struct tid_rdma_request {
	struct rvt_qp *qp;
	struct hfi1_ctxtdata *rcd;
	union {
		struct rvt_swqe *swqe;
		struct rvt_ack_entry *ack;
	} e;

	struct tid_rdma_flow *flows;	/* array of tid flows */
	struct rvt_sge_state ss; /* SGE state for TID RDMA requests */
	u16 n_max_flows;        /* size of the flow circular buffer */
	u16 n_flows;		/* size of the flow buffer window */
	u16 setup_head;		/* flow index we are setting up */
	u16 clear_tail;		/* flow index we are clearing */
	u16 flow_idx;		/* flow index most recently set up */
	u16 kdeth_seq;          /* the sequence (10bits) of the KDETH PSN */
	u16 acked_tail;

	u64 vaddr;
	u32 lkey;
	u32 rkey;
	u32 seg_len;
	u32 total_len;
	u32 r_ack_psn;          /* next expected ack PSN */
	u32 r_flow_psn;         /* IB PSN of next segment start */
	u32 r_last_acked;       /* IB PSN of last ACK'ed packet */
	u32 s_next_psn;         /* IB PSN of next segment start for read */

	u32 total_sge;		/* # sges in swqe->sg_list[] */
	u32 total_segs;		/* segments required to complete a request */
	u32 cur_seg;		/* index of current segment */
	u32 comp_seg;           /* index of last completed segment */
	u32 ack_seg;            /* index of last ack'ed segment */
	u32 alloc_seg;          /* index of next segment to be allocated */
	u32 isge;		/* index of "current" sge */
	u32 ack_pending;        /* num acks pending for this request */

	enum tid_rdma_req_state state;
};

/*
 * When header suppression is used, PSNs associated with a "flow" are
 * relevant (and not the PSNs maintained by verbs). Track per-flow
 * PSNs here.
 *
 */
struct flow_state {
	u32 flags;
	u32 resp_ib_psn;     /* The IB PSN of the response for this flow */
	u32 generation;      /* generation of flow */
	u32 spsn;            /* starting PSN in TID space */
	u32 lpsn;            /* last PSN in TID space */
	u32 r_next_psn;      /* next PSN to be received (in TID space) */

	/* For tid rdma read */
	u32 ib_spsn;         /* starting PSN in Verbs space */
	u32 ib_lpsn;         /* last PSn in Verbs space */
};

struct tid_rdma_pageset {
	dma_addr_t addr; /* Only needed for the first page */
	u16 idx;
	u16 count;
};

struct tid_rdma_flow {
	int idx;
	/*
	 * While a TID RDMA segment is being transferred, it uses a QP number
	 * from the "KDETH section of QP numbers" (which is different from the
	 * QP number that originated the request). Bits 11-15 of these QP
	 * numbers identify the "TID flow" for the segment.
	 */
	u32 tid_qpn;
	struct flow_state flow_state;
	struct tid_rdma_request *req;

	struct page **pages;
	u32 npages;
	u32 npagesets;
	struct tid_rdma_pageset *pagesets;
	struct kern_tid_node *tnode;
	u32 tnode_cnt;
	u32 tidcnt;
	u32 *tid_entry;
	u32 npkts;
	u32 pkt;
	/* send side fields */
	u32 tid_idx;
	u32 tid_offset;
	u32 length;
	u32 sent;
};

enum tid_rnr_nak_state {
	TID_RNR_NAK_INIT = 0,
	TID_RNR_NAK_SEND,
	TID_RNR_NAK_SENT,
};

bool tid_rdma_conn_req(struct rvt_qp *qp, u64 *data);
bool tid_rdma_conn_reply(struct rvt_qp *qp, u64 data);
bool tid_rdma_conn_resp(struct rvt_qp *qp, u64 *data);
void tid_rdma_conn_error(struct rvt_qp *qp);
void tid_rdma_flush_wait(struct rvt_qp *qp);

void hfi1_kern_init_ctxt_generations(struct hfi1_ctxtdata *rcd);
void tid_rdma_flush_wait(struct rvt_qp *qp);

void hfi1_compute_tid_rdma_flow_wt(void);
int hfi1_kern_setup_hw_flow(struct hfi1_ctxtdata *rcd, struct rvt_qp *qp);
void hfi1_kern_clear_hw_flow(struct hfi1_ctxtdata *rcd, struct rvt_qp *qp);

int hfi1_kern_exp_rcv_init(struct hfi1_ctxtdata *rcd, int reinit);
void hfi1_kern_exp_rcv_free(struct hfi1_ctxtdata *rcd);
int hfi1_kern_exp_rcv_setup(struct tid_rdma_request *req,
			    struct rvt_sge_state *ss, bool *last);
int hfi1_kern_exp_rcv_clear(struct tid_rdma_request *req);
void hfi1_kern_exp_rcv_clear_all(struct tid_rdma_request *req);
int hfi1_kern_exp_rcv_alloc_flows(struct tid_rdma_request *req, gfp_t gfp);
void hfi1_kern_exp_rcv_free_flows(struct tid_rdma_request *req);
void hfi1_kern_read_tid_flow_free(struct rvt_qp *qp);
void hfi1_tid_write_alloc_resources(struct rvt_qp *qp, bool intr_ctx);

void hfi1_rc_rcv_tid_rdma_write_req(struct hfi1_packet *packet);

void hfi1_rc_rcv_tid_rdma_write_data(struct hfi1_packet *packet);

void hfi1_rc_rcv_tid_rdma_write_resp(struct hfi1_packet *packet);

void hfi1_rc_rcv_tid_rdma_read_req(struct hfi1_packet *packet);

void hfi1_rc_rcv_tid_rdma_read_resp(struct hfi1_packet *packet);

void hfi1_rc_rcv_tid_rdma_ack(struct hfi1_packet *packet);

void hfi1_tid_timeout(unsigned long arg);

void hfi1_tid_retry_timeout(unsigned long arg);

u32 hfi1_compute_tid_rnr_timeout(struct rvt_qp *qp, u32 to_seg);

bool hfi1_handle_kdeth_eflags(struct hfi1_ctxtdata *rcd,
			      struct hfi1_pportdata *ppd,
			      struct hfi1_packet *packet);

void tid_rdma_trigger_resume(struct work_struct *work);

bool hfi1_tid_rdma_wqe_interlock(struct rvt_qp *qp, struct rvt_swqe *wqe);
bool hfi1_tid_rdma_ack_interlock(struct rvt_qp *qp, struct rvt_ack_entry *e);

void hfi1_add_tid_reap_timer(struct rvt_qp *qp);
void hfi1_mod_tid_reap_timer(struct rvt_qp *qp);
int hfi1_stop_tid_reap_timer(struct rvt_qp *qp);
void hfi1_del_tid_reap_timer(struct rvt_qp *qp);

void hfi1_add_tid_retry_timer(struct rvt_qp *qp);
void hfi1_mod_tid_retry_timer(struct rvt_qp *qp);
int hfi1_stop_tid_retry_timer(struct rvt_qp *qp);
void hfi1_del_tid_retry_timer(struct rvt_qp *qp);

static inline void hfi1_tid_rdma_reset_flow(struct tid_rdma_flow *flow)
{
	flow->npagesets = 0;
}

extern unsigned int hfi1_tid_rdma_seg_max_size;
extern u32 tid_rdma_flow_wt;
#if defined(TIDRDMA_DEBUG) && defined(TIDRDMA_EXTRA_DEBUG)
#define EXP_TID_TIDLEN_MASK   0x7FFULL
#define EXP_TID_TIDLEN_SHIFT  0
#define EXP_TID_TIDCTRL_MASK  0x3ULL
#define EXP_TID_TIDCTRL_SHIFT 20
#define EXP_TID_TIDIDX_MASK   0x3FFULL
#define EXP_TID_TIDIDX_SHIFT  22
#define EXP_TID_GET(tid, field)	\
	(((tid) >> EXP_TID_TID##field##_SHIFT) & EXP_TID_TID##field##_MASK)
#define hfi1_cdbg(which, fmt, ...) \
	__hfi1_trace_##which(__func__, fmt, ##__VA_ARGS__)
extern void __hfi1_trace_TIDRDMA(const char *func, char *fmt, ...);
static inline void dump_tid_array(u32 *tidentry, u32 tidcnt)
{
	u32 i;

	for (i = 0; i < tidcnt; i++)
		hfi1_cdbg(TIDRDMA, "tidentry[%u]=(idx:%u, ctrl:0x%x, len:%u)",
			  i, EXP_TID_GET(tidentry[i], IDX),
			  EXP_TID_GET(tidentry[i], CTRL),
			  EXP_TID_GET(tidentry[i], LEN) << PAGE_SHIFT);
}
#endif
#endif /* HFI1_TID_RDMA_H */
