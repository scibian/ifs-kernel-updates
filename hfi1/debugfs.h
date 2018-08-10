#ifndef _HFI1_DEBUGFS_H
#define _HFI1_DEBUGFS_H
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

struct hfi1_ibdev;
#ifdef CONFIG_DEBUG_FS
void hfi1_dbg_ibdev_init(struct hfi1_ibdev *ibd);
void hfi1_dbg_ibdev_exit(struct hfi1_ibdev *ibd);
void hfi1_dbg_init(void);
void hfi1_dbg_exit(void);

#ifdef CONFIG_FAULT_INJECTION
#include <linux/fault-inject.h>
struct fault_opcode {
	struct fault_attr attr;
	struct dentry *dir;
	bool fault_by_opcode;
	u64 n_rxfaults[256];
	u64 n_txfaults[256];
	u64 fault_skip;
	u64 skip;
	u8 opcode;
	u8 mask;
	u8 direction;
};

struct fault_packet {
	struct fault_attr attr;
	struct dentry *dir;
	bool fault_by_packet;
	u64 n_faults;
};

bool hfi1_dbg_fault_opcode(struct rvt_qp *qp, u32 opcode, bool rx);
bool hfi1_dbg_fault_packet(struct hfi1_packet *packet);
bool hfi1_dbg_fault_suppress_err(struct hfi1_ibdev *ibd);
#else
static inline bool hfi1_dbg_fault_packet(struct hfi1_packet *packet)
{
	return false;
}

static inline bool hfi1_dbg_fault_opcode(struct rvt_qp *qp,
					 u32 opcode, bool rx)
{
	return false;
}

static inline bool hfi1_dbg_fault_suppress_err(struct hfi1_ibdev *ibd)
{
	return false;
}
#endif

#else
static inline void hfi1_dbg_ibdev_init(struct hfi1_ibdev *ibd)
{
}

static inline void hfi1_dbg_ibdev_exit(struct hfi1_ibdev *ibd)
{
}

static inline void hfi1_dbg_init(void)
{
}

static inline void hfi1_dbg_exit(void)
{
}

static inline bool hfi1_dbg_fault_packet(struct hfi1_packet *packet)
{
	return false;
}

static inline bool hfi1_dbg_fault_opcode(struct rvt_qp *qp,
					 u32 opcode, bool rx)
{
	return false;
}

static inline bool hfi1_dbg_fault_suppress_err(struct hfi1_ibdev *ibd)
{
	return false;
}
#endif

#endif                          /* _HFI1_DEBUGFS_H */
