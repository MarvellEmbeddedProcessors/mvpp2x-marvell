/*
 * Driver for Marvell PPv2x family network controllers.
 *
 * Copyright (C) 2015 Marvell
 *
 *.
 *.This program is free software; you can redistribute it and/or modify it
 *.under the terms and conditions of the GNU General Public License,
 *.version 2, as published by the Free Software Foundation.
 *
 *.This program is distributed in the hope it will be useful, but WITHOUT
 *.ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *.FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *.more details.
 *
 *.You should have received a copy of the GNU General Public License along with
 *.this program; if not, write to the Free Software Foundation, Inc.,
 *.51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *.The full GNU General Public License is included in this distribution in
 *.the file called "COPYING".
 *
 *
 * Contact Information:
 * Linux Network Driver <XXX.YYYY@marvell.com>
 */

#ifndef _MVPP2_DEBUG_H_
#define _MVPP2_DEBUG_H_

#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>

#define MV_AMPLIFY_FACTOR_MTU				(3)
#define MV_BIT_NUM_OF_BYTE				(8)
#define MV_WRR_WEIGHT_UNIT				(256)

/* Macro for alignment up. For example, MV_ALIGN_UP(0x0330, 0x20) = 0x0340   */
#define MV_ALIGN_UP(number, align)                                          \
(((number) & ((align) - 1)) ? (((number) + (align)) & ~((align)-1)) : (number))

/* Macro for alignment down. For example, MV_ALIGN_UP(0x0330, 0x20) = 0x0320 */
#define MV_ALIGN_DOWN(number, align) ((number) & ~((align)-1))
/* CPU architecture dependent 32, 16, 8 bit read/write IO addresses */
#define MV_MEMIO32_WRITE(addr, data)    \
	((*((volatile unsigned int *)(addr))) = ((unsigned int)(data)))

#define MV_MEMIO32_READ(addr)           \
	((*((volatile unsigned int *)(addr))))

#define MV_MEMIO16_WRITE(addr, data)    \
	((*((volatile unsigned short *)(addr))) = ((unsigned short)(data)))

#define MV_MEMIO16_READ(addr)           \
	((*((volatile unsigned short *)(addr))))

#define MV_MEMIO8_WRITE(addr, data)     \
	((*((volatile unsigned char *)(addr))) = ((unsigned char)(data)))

#define MV_MEMIO8_READ(addr)            \
	((*((volatile unsigned char *)(addr))))

/* This macro returns absolute value                                        */
#define MV_ABS(number)  (((int)(number) < 0) ? -(int)(number) : (int)(number))


void mvpp2_print_reg(struct mvpp2_hw *hw, unsigned int reg_addr, char *reg_name);
void mvpp2_print_reg2(struct mvpp2_hw *hw, unsigned int reg_addr, char *reg_name, unsigned int index);


void mvpp2_bm_pool_regs(struct mvpp2_hw *hw, int pool);
void mvpp2_bm_pool_drop_count(struct mvpp2_hw *hw, int pool);
void mvpp2_pool_status(struct mvpp2 *priv, int log_pool_num);
void mv_pp2_pool_stats_print(struct mvpp2 *priv, int log_pool_num);


void mvPp2RxDmaRegsPrint(struct mvpp2 *priv, bool print_all, int start, int stop);
void mvPp2RxqShow(struct mvpp2 *priv, int port, int rxq, int mode);
void mvPp2PhysRxqRegs(struct mvpp2 *pp2, int rxq);
void mvPp2PortRxqRegs(struct mvpp2 *pp2, int port, int rxq);
void mvpp22_isr_rx_group_regs(struct mvpp2 *priv, int port, bool print_all);

void mvPp2V1RxqDbgCntrs(struct mvpp2 *priv, int port, int rxq);
void mvPp2RxFifoRegs(struct mvpp2_hw *hw, int port);

void mvpp2_rx_desc_print(struct mvpp2 *priv, struct mvpp2_rx_desc *desc);

void mvpp2_skb_dump(struct sk_buff *skb, int size, int access);
void mvPp2TxqShow(struct mvpp2 *priv, int port, int txq, int mode);
void mvPp2AggrTxqShow(struct mvpp2 *priv, int cpu, int mode);
void mvPp2PhysTxqRegs(struct mvpp2 *priv, int txq);
void mvPp2PortTxqRegs(struct mvpp2 *priv, int port, int txq);
void mvPp2AggrTxqRegs(struct mvpp2 *priv, int cpu);
void mvPp2V1TxqDbgCntrs(struct mvpp2 *priv, int port, int txq);
void mvPp2V1DropCntrs(struct mvpp2 *priv, int port);
void mvPp2TxRegs(struct mvpp2 *priv);
void mvPp2TxSchedRegs(struct mvpp2 *priv, int port);
int mvPp2TxpRateSet(struct mvpp2 *priv, int port, int rate);
int mvPp2TxpBurstSet(struct mvpp2 *priv, int port, int burst);
int mvPp2TxqRateSet(struct mvpp2 *priv, int port, int txq, int rate);
int mvPp2TxqBurstSet(struct mvpp2 *priv, int port, int txq, int burst);
int mvPp2TxqFixPrioSet(struct mvpp2 *priv, int port, int txq);
int mvPp2TxqWrrPrioSet(struct mvpp2 *priv, int port, int txq, int weight);

int mvpp2_wrap_cos_mode_set(struct mvpp2_port *port, enum mvpp2_cos_classifier cos_mode);
int mvpp2_wrap_cos_mode_get(struct mvpp2_port *port);
int mvpp2_wrap_cos_pri_map_set(struct mvpp2_port *port, int cos_pri_map);
int mvpp2_wrap_cos_pri_map_get(struct mvpp2_port *port);
int mvpp2_wrap_cos_dflt_value_set(struct mvpp2_port *port, int cos_value);
int mvpp2_wrap_cos_dflt_value_get(struct mvpp2_port *port);
int mvpp22_wrap_rss_mode_set(struct mvpp2_port *port, int rss_mode);
int mvpp22_wrap_rss_dflt_cpu_set(struct mvpp2_port *port, int default_cpu);
int mvpp2_port_bind_cpu_set(struct mvpp2_port *port, u8 bind_cpu);
int mvpp2_debug_param_set(u32 param);

void mvpp2_bm_queue_map_dump_all(struct mvpp2_hw *hw);

int mvpp2_cls_c2_qos_prio_set(struct mvpp2_cls_c2_qos_entry *qos, u8 pri);
int mvpp2_cls_c2_qos_dscp_set(struct mvpp2_cls_c2_qos_entry *qos, u8 dscp);
int mvpp2_cls_c2_qos_color_set(struct mvpp2_cls_c2_qos_entry *qos, u8 color);
int mvpp2_cls_c2_queue_set(struct mvpp2_cls_c2_entry *c2, int cmd, int queue, int from);
int mvpp2_cls_c2_mtu_set(struct mvpp2_cls_c2_entry *c2, int mtu_inx);

#endif /* _MVPP2_DEBUG_H_ */
