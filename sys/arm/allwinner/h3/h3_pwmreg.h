/*-
 * Copyright (c) 2016 Teodora Kireva <teodora_kireva@smartcom.bg> 
 * Copyright (c) 2016 Tsvetko Tsvetkov <tsvetko_tsvetkov@smartcom.bg>
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
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _H3_PWM_H_
#define _H3_PWM_H_

#define PWM_CH_CTRL		0x00
#define PWM_RDY			(1 << 28)

#define PWM_BYPASS		(1 << 9)
#define PWM_CH_PUL_START	(1 << 8)
#define PWM_CH_MODE_OPM		(1 << 7)
#define SCLK_CH_GATING		(1 << 6)
#define PWM_CH_ACT_STA		(1 << 5)
#define PWM_CH_EN		(1 << 4)
#define PWM_CH0_CTRL_SHIFT_N	0
#define PWM_CH1_CTRL_SHIFT_N	15


#define PWM_CH0_PERIOD			0x04
#define PWM_CH0_ENTIRE_CYS_SHIFT_N	16
#define PWM_CH0_ENTIRE_ACT_CYS_SHIFT_N	0

#define PWM_CH1_PERIOD			0x08
#define PWM_CH1_ENTIRE_CYS_SHIFT_N	16
#define PWM_CH1_ENTIRE_ACT_CYS_SHIFT_N	0

#endif /* _H3_PWM_H_ */
