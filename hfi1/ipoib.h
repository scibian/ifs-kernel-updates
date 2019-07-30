/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Copyright(c) 2018 Intel Corporation.
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

/*
 * This file contains HFI1 support for IPOIB functionality
 */

#ifndef HFI1_IPOIB_H
#define HFI1_IPOIB_H

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/list.h>
#include <linux/if_infiniband.h>

#include "hfi.h"
#include "iowait.h"
#include "netdev.h"

#include <rdma/ib_verbs.h>

#define HFI1_IPOIB_TXREQ_NAME_LEN   32

#define HFI1_IPOIB_PSEUDO_LEN 20
#define HFI1_IPOIB_ENCAP_LEN 4

struct hfi1_ipoib_dev_priv;

union hfi1_ipoib_flow {
	u16 as_int;
	struct {
		u8 tx_queue;
		u8 sc5;
	} __attribute__((__packed__));
};

/**
 * struct hfi1_ipoib_txq - IPOIB per Tx queue information
 * priv - private pointer
 * @sde - sdma engine
 * @wait - iowait structure
 * @stx - sdma tx request
 * @tx_list - tx request list
 * @psn - packet sequence number
 * @q_idx - ipoib Tx queue index
 * @sc5 - service channel
 * @pkts_sent - indicator packets have beeen sent from this queue
 */
struct hfi1_ipoib_txq {
	struct hfi1_ipoib_dev_priv *priv;
	struct sdma_engine *sde;
	struct iowait wait;
	struct list_head tx_list;
	u32 psn;
	union hfi1_ipoib_flow flow;
	u8 q_idx;
	bool pkts_sent;
};

struct hfi1_ipoib_dev_priv {
	struct hfi1_devdata *dd;
	struct net_device   *netdev;
	struct ib_device    *device;
	u8                   port_num;

	struct hfi1_ipoib_txq *txqs;
	struct kmem_cache *txreq_cache;

	const struct net_device_ops *netdev_ops;

	u16 pkey;
	u16 pkey_index;
	u32 qkey;

	struct rvt_qp *qp;

	struct pcpu_sw_netstats *netstats;

	unsigned long flags;
};

/* hfi1 ipoib rdma netdev's private data structure */
struct hfi1_ipoib_rdma_netdev {
	struct rdma_netdev rn;  /* keep this first */
	/* followed by device private data */
	struct hfi1_ipoib_dev_priv dev_priv;
};

static inline struct hfi1_ipoib_dev_priv *
hfi1_ipoib_priv(const struct net_device *dev)
{
	return &((struct hfi1_ipoib_rdma_netdev *)netdev_priv(dev))->dev_priv;
}

static inline void
hfi1_ipoib_update_rx_netstats(struct hfi1_ipoib_dev_priv *priv,
			      u64 packets,
			      u64 bytes)
{
	struct pcpu_sw_netstats *netstats = this_cpu_ptr(priv->netstats);

	u64_stats_update_begin(&netstats->syncp);
	netstats->rx_packets += packets;
	netstats->rx_bytes += bytes;
	u64_stats_update_end(&netstats->syncp);
}

static inline void
hfi1_ipoib_update_tx_netstats(struct hfi1_ipoib_dev_priv *priv,
			      u64 packets,
			      u64 bytes)
{
	struct pcpu_sw_netstats *netstats = this_cpu_ptr(priv->netstats);

	u64_stats_update_begin(&netstats->syncp);
	netstats->tx_packets += packets;
	netstats->tx_bytes += bytes;
	u64_stats_update_end(&netstats->syncp);
}

int hfi1_ipoib_send_dma(struct net_device *dev,
			struct sk_buff *skb,
			struct ib_ah *address,
			u32 dqpn);

int hfi1_ipoib_txreq_init(struct hfi1_ipoib_dev_priv *priv);
void hfi1_ipoib_txreq_deinit(struct hfi1_ipoib_dev_priv *priv);

int hfi1_ipoib_rxq_init(struct net_device *dev);
void hfi1_ipoib_rxq_deinit(struct net_device *dev);

struct net_device *hfi1_ipoib_alloc_rn(struct ib_device *device,
				       u8 port_num,
				       enum rdma_netdev_t type,
				       const char *name,
				       unsigned char name_assign_type,
				       void (*setup)(struct net_device *));

struct sk_buff *hfi1_ipoib_prepare_skb(struct hfi1_netdev_rxq *rxq,
				       int size, void *data);

#endif /* _IPOIB_H */
