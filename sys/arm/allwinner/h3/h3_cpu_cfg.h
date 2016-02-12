/*-
 * Copyright (c) 2013 Ganbold Tsagaankhuu <ganbold@freebsd.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef _H3_CPU_CFG_H_
#define _H3_CPU_CFG_H_

#define CPU_CFG_BASE		0xe1f01c00

#define CPU0_RST_CTRL		0x0040
#define CPU0_CTRL_REG		0x0044
#define CPU0_STATUS_REG 	0x0048

#define CPU1_RST_CTRL		0x0080
#define CPU1_CTRL_REG		0x0084
#define CPU1_STATUS_REG 	0x0088

#define CPU2_RST_CTRL		0x00c0
#define CPU2_CTRL_REG		0x00c4
#define CPU2_STATUS_REG 	0x00c8

#define CPU3_RST_CTRL		0x0100
#define CPU3_CTRL_REG		0x0104
#define CPU3_STATUS_REG 	0x0108

#define GENER_CTRL_REG		0x0184

#define OSC24M_CNT64_CTRL_REG	0x0280
#define OSC24M_CNT64_LOW_REG	0x0284
#define OSC24M_CNT64_HIGH_REG	0x0288

#define CNT64_RL_EN		0x02 /* read latch enable */

uint64_t h3_read_counter64(void);

#endif /* _H3_CPU_CFG_H_ */
