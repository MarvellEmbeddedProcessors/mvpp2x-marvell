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

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/inetdevice.h>
#include <linux/mbus.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <uapi/linux/ppp_defs.h>
#include <net/ip.h>
#include <net/ipv6.h>


#include "mvpp2.h"
#include "mvpp2_hw.h"



void mvpp2_print_reg(struct mvpp2_hw *hw, unsigned int reg_addr,
	char *reg_name)
{
	printk("  %-32s: 0x%x = 0x%08x\n", reg_name, reg_addr,
		mvpp2_read(hw, reg_addr));
}

void mvpp2_print_reg2(struct mvpp2_hw *hw, unsigned int reg_addr,
	char *reg_name, unsigned int index)
{
	char buf[64];

	sprintf(buf, "%s[%d]", reg_name, index);
	printk("  %-32s: 0x%x = 0x%08x\n", reg_name, reg_addr,
		mvpp2_read(hw, reg_addr));
}



void mvpp2_bm_pool_regs(struct mvpp2_hw *hw, int pool)
{
	if (mvpp2_max_check(pool, MVPP2_BM_POOLS_NUM, "bm_pool"))
		return;

	printk("\n[BM pool registers: pool=%d]\n", pool);
	mvpp2_print_reg(hw, MVPP2_BM_POOL_BASE_ADDR_REG(pool), "MV_BM_POOL_BASE_REG");
	mvpp2_print_reg(hw, MVPP2_BM_POOL_SIZE_REG(pool), "MVPP2_BM_POOL_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_BM_POOL_READ_PTR_REG(pool), "MVPP2_BM_POOL_READ_PTR_REG");
	mvpp2_print_reg(hw, MVPP2_BM_POOL_PTRS_NUM_REG(pool), "MVPP2_BM_POOL_PTRS_NUM_REG");
	mvpp2_print_reg(hw, MVPP2_BM_BPPI_READ_PTR_REG(pool), "MVPP2_BM_BPPI_READ_PTR_REG");
	mvpp2_print_reg(hw, MVPP2_BM_BPPI_PTRS_NUM_REG(pool), "MVPP2_BM_BPPI_PTRS_NUM_REG");
	mvpp2_print_reg(hw, MVPP2_BM_POOL_CTRL_REG(pool), "MVPP2_BM_POOL_CTRL_REG");
	mvpp2_print_reg(hw, MVPP2_BM_INTR_CAUSE_REG(pool), "MVPP2_BM_INTR_CAUSE_REG");
	mvpp2_print_reg(hw, MVPP2_BM_INTR_MASK_REG(pool), "MVPP2_BM_INTR_MASK_REG");
}
EXPORT_SYMBOL(mvpp2_bm_pool_regs);


void mvpp2_bm_pool_drop_count(struct mvpp2_hw *hw, int pool)
{
	if (mvpp2_max_check(pool, MVPP2_BM_POOLS_NUM, "bm_pool"))
		return;

	mvpp2_print_reg2(hw, MVPP2_BM_DROP_CNTR_REG(pool), "MVPP2_BM_DROP_CNTR_REG", pool);
	mvpp2_print_reg2(hw, MVPP2_BM_MC_DROP_CNTR_REG(pool), "MVPP2_BM_MC_DROP_CNTR_REG", pool);
}
EXPORT_SYMBOL(mvpp2_bm_pool_drop_count);


void mvpp2_pool_status(struct mvpp2 *priv, int log_pool_num)
{
	struct mvpp2_bm_pool *bm_pool = NULL;
	int /*buf_size,*/ total_size, i, pool;

	if (mvpp2_max_check(log_pool_num, MVPP2_BM_SWF_POOL_OUT_OF_RANGE, "log_pool"))
		return;

	for (i=0;i<priv->num_pools;i++) {
		if (priv->bm_pools[i].log_id == log_pool_num) {
			bm_pool = &priv->bm_pools[i];
			pool = bm_pool->id;
		}
	}
	if (bm_pool == NULL) {
		pr_err("%s: Logical BM pool %d is not initialized\n", __func__, log_pool_num);
		return;
	}

	total_size = RX_TOTAL_SIZE(bm_pool->buf_size);


	printk("\n%12s log_pool=%d, phy_pool=%d: pkt_size=%d, buf_size=%d total_size=%d\n",
		mvpp2_pools[log_pool_num].description, log_pool_num, pool,
		bm_pool->pkt_size, bm_pool->buf_size, total_size);
	printk("\tcapacity=%d, buf_num=%d, in_use=%u, in_use_thresh=%u\n",
		bm_pool->size, bm_pool->buf_num, atomic_read(&bm_pool->in_use),
		bm_pool->in_use_thresh);
}
EXPORT_SYMBOL(mvpp2_pool_status);





void mvPp2RxDmaRegsPrint(struct mvpp2 *priv, bool print_all, int start, int stop)
{
	int i, num_rx_queues, result;
	bool enabled;

	struct mvpp2_hw *hw = &priv->hw;

	num_rx_queues = (MVPP2_MAX_PORTS*priv->pp2xdata->pp2x_max_port_rxqs);
	if(stop >= num_rx_queues || start > stop || start < 0) {
		printk("\nERROR: wrong inputs\n");
		return;
	}

	printk("\n[RX DMA regs]\n");
	printk("\nRXQs [0..%d], registers\n", num_rx_queues-1);

	for (i = start; i <= stop; i++) {
		if (!print_all) {
			result = mvpp2_read(hw, MVPP2_RXQ_CONFIG_REG(i));
			enabled = !(result & MVPP2_RXQ_DISABLE_MASK);
		}
		if (print_all || enabled) {
			printk("RXQ[%d]:\n", i);
			mvpp2_print_reg(hw, MVPP2_RXQ_STATUS_REG(i), "MVPP2_RX_STATUS");
			mvpp2_print_reg2(hw, MVPP2_RXQ_CONFIG_REG(i), "MVPP2_RXQ_CONFIG_REG", i);
		}
	}
	printk("\nBM pools [0..%d] registers\n", MVPP2_BM_POOLS_NUM-1);
	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		if (!print_all) {
			enabled = mvpp2_read(hw, MVPP2_BM_POOL_CTRL_REG(i)) & MVPP2_BM_STATE_MASK;
		}
		if (print_all || enabled) {
			printk("POOL[%d]:\n", i);
			mvpp2_print_reg2(hw, MVPP2_POOL_BUF_SIZE_REG(i), "MVPP2_POOL_BUF_SIZE_REG", i);
		}
	}
	printk("\nIngress ports [0..%d] registers\n", MVPP2_MAX_PORTS-1);
	for (i = 0; i < MVPP2_MAX_PORTS; i++) {
		mvpp2_print_reg2(hw, MVPP2_RX_CTRL_REG(i), "MVPP2_RX_CTRL_REG", i);
	}
	printk("\n");
}
EXPORT_SYMBOL(mvPp2RxDmaRegsPrint);

static void mvPp2QueueShow(struct mvpp2 *priv, struct mvpp2_rx_queue *pp_rxq)
{
	int i;
	struct mvpp2_rx_desc *rx_desc = pp_rxq->descs;

	for (i = 0; i < pp_rxq->size; i++) {
		printk("%3d. desc=%p, status=%08x, data_size=%4d",
			   i, rx_desc+i, rx_desc[i].status, rx_desc[i].data_size);
		if (priv->pp2_version == PPV21) {
			printk("buf_addr=%08x, buf_cookie=%08x",
				   mvpp21_rxdesc_phys_addr_get(rx_desc),
				   (u32)mvpp21_rxdesc_cookie_get(rx_desc));
		}
		else {
			printk("buf_addr=%08x, buf_cookie=%08x",
				   mvpp22_rxdesc_phys_addr_get(rx_desc),
				   (u32)mvpp22_rxdesc_cookie_get(rx_desc));
		}

		printk("parser_info=%03x\n", rx_desc->rsrvd_parser);
	}
}

/* Show Port/Rxq descriptors ring */
void mvPp2RxqShow(struct mvpp2 *priv, int port, int rxq, int mode)
{
	int pRxq;
	struct mvpp2_port *pp_port;
	struct mvpp2_rx_queue *pp_rxq;

	pp_port = mvpp2_port_struct_get(priv, port);

	if (pp_port == NULL) {
		printk("port #%d is not initialized\n", port);
		return;
	}

	if (mvpp2_max_check(rxq, pp_port->num_rx_queues, "logical rxq"))
		return;

	pp_rxq = pp_port->rxqs[rxq];

	if (pp_rxq->descs == NULL) {
		printk("rxq #%d of port #%d is not initialized\n", rxq, port);
		return;
	}

	printk("\n[PPv2 RxQ show: port=%d, logical rxq=%d -> physical rxq=%d]\n",
			port, pp_rxq->log_id, pp_rxq->id);

	printk("first_virt_addr=%p, first_dma_addr=%x, next_rx_dec=%d, rxq_cccupied=%d, rxq_nonoccupied=%d\n",
		   pp_rxq->descs, pp_rxq->descs_phys, pp_rxq->next_desc_to_proc,
		   mvpp2_rxq_received(pp_port, pp_rxq->id), mvpp2_rxq_free(pp_port, pp_rxq->id));

	if (mode)
		mvPp2QueueShow(priv, pp_rxq);
}
EXPORT_SYMBOL(mvPp2RxqShow);

void mvPp2PhysRxqRegs(struct mvpp2 *pp2, int rxq)
{
	struct mvpp2_hw *hw  = &pp2->hw;

	printk("\n[PPv2 RxQ registers: global rxq=%d]\n", rxq);

	mvpp2_write(hw, MVPP2_RXQ_NUM_REG, rxq);
	mvpp2_print_reg(hw, MVPP2_RXQ_NUM_REG, "MVPP2_RXQ_NUM_REG");
	mvpp2_print_reg(hw, MVPP21_RXQ_DESC_ADDR_REG, "MVPP2_RXQ_DESC_ADDR_REG");
	mvpp2_print_reg(hw, MVPP2_RXQ_DESC_SIZE_REG, "MVPP2_RXQ_DESC_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_RXQ_STATUS_REG(rxq), "MVPP2_RXQ_STATUS_REG");
	mvpp2_print_reg(hw, MVPP2_RXQ_THRESH_REG, "MVPP2_RXQ_THRESH_REG");
	mvpp2_print_reg(hw, MVPP2_RXQ_INDEX_REG, "MVPP2_RXQ_INDEX_REG");
	mvpp2_print_reg(hw, MVPP2_RXQ_CONFIG_REG(rxq), "MVPP2_RXQ_CONFIG_REG");
}
EXPORT_SYMBOL(mvPp2PhysRxqRegs);

void mvPp2PortRxqRegs(struct mvpp2 *pp2, int port, int rxq)
{
	int phy_rxq;
	struct mvpp2_port * pp2_port = mvpp2_port_struct_get(pp2, port);

	printk("\n[PPv2 RxQ registers: port=%d, local rxq=%d]\n", port, rxq);

	if (rxq >= MVPP2_MAX_RXQ)
		return;

	if (!pp2_port)
		return;

	phy_rxq = pp2_port->first_rxq + rxq;
	mvPp2PhysRxqRegs(pp2, phy_rxq);
}
EXPORT_SYMBOL(mvPp2PortRxqRegs);


void mvpp22_isr_rx_group_regs(struct mvpp2 *priv, int port, bool print_all)
{
	int val, i, num_threads, cpu_offset, cpu;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp_port;


	pp_port = mvpp2_port_struct_get(priv, port);
	if (!pp_port) {
		pr_crit("Input Error\n %s",__func__);
		return;
	}

	if(print_all)
		num_threads = MVPP2_MAX_SW_THREADS;
	else
		num_threads = pp_port->num_qvector;

	for (i=0;i<num_threads;i++) {
		printk("\n[PPv2 RxQ GroupConfig registers: port=%d cpu=%d]", port, i);

		val = (port << MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET) | i;
		mvpp2_write(hw, MVPP22_ISR_RXQ_GROUP_INDEX_REG, val);

		mvpp2_print_reg(hw, MVPP22_ISR_RXQ_GROUP_INDEX_REG, "MVPP22_ISR_RXQ_GROUP_INDEX_REG");
		mvpp2_print_reg(hw, MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG, "MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG");
//		reg_val = mvpp2_read(hw, MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG);
//		start_queue  = reg_val & MVPP22_ISR_RXQ_SUB_GROUP_STARTQ_MASK;
//		sub_group_size = reg_val & MVPP22_ISR_RXQ_SUB_GROUP_SIZE_MASK;
	}
	printk("\n[PPv2 Port Interrupt Enable register : port=%d]\n", port);
	mvpp2_print_reg(hw, MVPP2_ISR_ENABLE_REG(port), "MVPP2_ISR_ENABLE_REG");


	printk("\n[PPv2 Eth Occupied Interrupt registers: port=%d]\n", port);
	for (i=0;i<num_threads;i++) {
		if (print_all)
			cpu = i;
		else
			cpu = pp_port->q_vector[i].sw_thread_id;
		cpu_offset = cpu*MVPP2_ADDR_SPACE_SIZE;
		printk("cpu=%d]\n", cpu);
		mvpp2_print_reg(hw, cpu_offset + MVPP2_ISR_RX_TX_CAUSE_REG(port), "MVPP2_ISR_RX_TX_CAUSE_REG");
		mvpp2_print_reg(hw, cpu_offset + MVPP2_ISR_RX_TX_MASK_REG(port), "MVPP2_ISR_RX_TX_MASK_REG");
	}

}
EXPORT_SYMBOL(mvpp22_isr_rx_group_regs);




#if 0
/* Show Port/TXQ descriptors ring */
void mvPp2TxqShow(struct mvpp2_hw *hw, int port, int txp, int txq, int mode)
{
	int pTxq;
	MVPP2_PHYS_TXQ_CTRL *pTxqCtrl;
	MVPP2_QUEUE_CTRL *pQueueCtrl;

	printk("\n[PPv2 TxQ show: port=%d, txp=%d, txq=%d]\n", port, txp, txq);

	if (mvPp2TxpCheck(port, txp))
		return;

	if (mvPp2MaxCheck(txq, MVPP2_MAX_TXQ, "logical txq"))
		return;

	pTxq = MV_PPV2_TXQ_PHYS(port, txp, txq);

	pTxqCtrl = &mvPp2PhysTxqs[pTxq];
	pQueueCtrl = &pTxqCtrl->queueCtrl;
	if (!pQueueCtrl->pFirst) {
		printk("txq %d wasn't created\n", txq);
		return;
	}

	printk("nextToProc=%d (%p), txqPending=%d\n",
		   pQueueCtrl->nextToProc,
		   MVPP2_QUEUE_DESC_PTR(pQueueCtrl, pQueueCtrl->nextToProc),
		   mvPp2TxqPendDescNumGet(port, txp, txq));

	mvPp2QueueShow(pQueueCtrl, mode, 1);
}

/* Show CPU aggregation TXQ descriptors ring */
void mvPp2AggrTxqShow(struct mvpp2_hw *hw, int cpu, int mode)
{
	MVPP2_AGGR_TXQ_CTRL *pTxqCtrl;
	MVPP2_QUEUE_CTRL *pQueueCtrl;

	printk("\n[PPv2 AggrTxQ: cpu=%d]\n", cpu);

	if (mvPp2CpuCheck(cpu))
		return;

	pTxqCtrl = &mvPp2AggrTxqs[cpu];
	pQueueCtrl = &pTxqCtrl->queueCtrl;
	if (!pQueueCtrl->pFirst) {
		printk("aggr tx queue for cpu %d wasn't created\n", cpu);
		return;
	}
	printk("nextToProc=%d (%p), txqPending=%d\n",
		   pQueueCtrl->nextToProc,
		   MVPP2_QUEUE_DESC_PTR(pQueueCtrl, pQueueCtrl->nextToProc),
		   mvPp2AggrTxqPendDescNumGet(cpu));

	mvPp2QueueShow(pQueueCtrl, mode, 1);
}
void mvPp2IsrRegs(struct mvpp2_hw *hw, int port)
{
	int physPort;

	if (mvPp2PortCheck(port))
		return;

	physPort = MV_PPV2_PORT_PHYS(port);

	printk("\n[PPv2 ISR registers: port=%d - %s]\n", port, MVPP2_IS_PON_PORT(port) ? "PON" : "GMAC");
	mvpp2_print_reg(MVPP2_ISR_RXQ_GROUP_REG(port), "MVPP2_ISR_RXQ_GROUP_REG");
	mvpp2_print_reg(MVPP2_ISR_ENABLE_REG(port), "MVPP2_ISR_ENABLE_REG");
	mvpp2_print_reg(MVPP2_ISR_RX_TX_CAUSE_REG(physPort), "MVPP2_ISR_RX_TX_CAUSE_REG");
	mvpp2_print_reg(MVPP2_ISR_RX_TX_MASK_REG(physPort), "MVPP2_ISR_RX_TX_MASK_REG");

	mvpp2_print_reg(MVPP2_ISR_RX_ERR_CAUSE_REG(physPort), "MVPP2_ISR_RX_ERR_CAUSE_REG");
	mvpp2_print_reg(MVPP2_ISR_RX_ERR_MASK_REG(physPort), "MVPP2_ISR_RX_ERR_MASK_REG");

	if (MVPP2_IS_PON_PORT(port)) {
		mvpp2_print_reg(MVPP2_ISR_PON_TX_UNDR_CAUSE_REG, "MVPP2_ISR_PON_TX_UNDR_CAUSE_REG");
		mvpp2_print_reg(MVPP2_ISR_PON_TX_UNDR_MASK_REG, "MVPP2_ISR_PON_TX_UNDR_MASK_REG");
	} else {
		mvpp2_print_reg(MVPP2_ISR_TX_ERR_CAUSE_REG(physPort), "MVPP2_ISR_TX_ERR_CAUSE_REG");
		mvpp2_print_reg(MVPP2_ISR_TX_ERR_MASK_REG(physPort), "MVPP2_ISR_TX_ERR_MASK_REG");
	}
	mvpp2_print_reg(MVPP2_ISR_MISC_CAUSE_REG, "MVPP2_ISR_MISC_CAUSE_REG");
	mvpp2_print_reg(MVPP2_ISR_MISC_MASK_REG, "MVPP2_ISR_MISC_MASK_REG");
}


void mvPp2PhysTxqRegs(struct mvpp2_hw *hw, int txq)
{
	printk("\n[PPv2 TxQ registers: global txq=%d]\n", txq);

	if (mvPp2MaxCheck(txq, MVPP2_TXQ_TOTAL_NUM, "global txq"))
		return;

	mvpp2_write(MVPP2_TXQ_NUM_REG, txq);
	mvpp2_print_reg(MVPP2_TXQ_NUM_REG, "MVPP2_TXQ_NUM_REG");
	mvpp2_print_reg(MVPP2_TXQ_DESC_ADDR_REG, "MVPP2_TXQ_DESC_ADDR_REG");
	mvpp2_print_reg(MVPP2_TXQ_DESC_SIZE_REG, "MVPP2_TXQ_DESC_SIZE_REG");
	mvpp2_print_reg(MVPP2_TXQ_DESC_HWF_SIZE_REG, "MVPP2_TXQ_DESC_HWF_SIZE_REG");
	mvpp2_print_reg(MVPP2_TXQ_INDEX_REG, "MVPP2_TXQ_INDEX_REG");
	mvpp2_print_reg(MVPP2_TXQ_PREF_BUF_REG, "MVPP2_TXQ_PREF_BUF_REG");
	mvpp2_print_reg(MVPP2_TXQ_PENDING_REG, "MVPP2_TXQ_PENDING_REG");
	mvpp2_print_reg(MVPP2_TXQ_SENT_REG(txq), "MVPP2_TXQ_SENT_REG");
	mvpp2_print_reg(MVPP2_TXQ_INT_STATUS_REG, "MVPP2_TXQ_INT_STATUS_REG");
}

void mvPp2PortTxqRegs(struct mvpp2_hw *hw, int port, int txp, int txq)
{
	printk("\n[PPv2 TxQ registers: port=%d, txp=%d, local txq=%d]\n", port, txp, txq);

	if (mvPp2TxpCheck(port, txp))
		return;

	if (mvPp2MaxCheck(txq, MVPP2_MAX_TXQ, "local txq"))
		return;

	mvPp2PhysTxqRegs(MV_PPV2_TXQ_PHYS(port, txp, txq));
}

void mvPp2AggrTxqRegs(struct mvpp2_hw *hw, int cpu)
{
	printk("\n[PP2 Aggr TXQ registers: cpu=%d]\n", cpu);

	if (mvPp2CpuCheck(cpu))
		return;

	mvpp2_print_reg(MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu), "MVPP2_AGGR_TXQ_DESC_ADDR_REG");
	mvpp2_print_reg(MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu), "MVPP2_AGGR_TXQ_DESC_SIZE_REG");
	mvpp2_print_reg(MVPP2_AGGR_TXQ_STATUS_REG(cpu), "MVPP2_AGGR_TXQ_STATUS_REG");
	mvpp2_print_reg(MVPP2_AGGR_TXQ_INDEX_REG(cpu), "MVPP2_AGGR_TXQ_INDEX_REG");
}

void mvPp2AddrDecodeRegs(struct mvpp2_hw *hw)
{
	MV_U32 regValue;
	int win;

	/* ToDo - print Misc interrupt Cause and Mask registers */

	mvpp2_print_reg(ETH_BASE_ADDR_ENABLE_REG, "ETH_BASE_ADDR_ENABLE_REG");
	mvpp2_print_reg(ETH_TARGET_DEF_ADDR_REG, "ETH_TARGET_DEF_ADDR_REG");
	mvpp2_print_reg(ETH_TARGET_DEF_ID_REG, "ETH_TARGET_DEF_ID_REG");

	regValue = mvPp2RdReg(ETH_BASE_ADDR_ENABLE_REG);
	for (win = 0; win < ETH_MAX_DECODE_WIN; win++) {
		if ((regValue & (1 << win)) == 0)
			continue; /* window is disable */

		printk("\t win[%d]\n", win);
		mvpp2_print_reg(ETH_WIN_BASE_REG(win), "\t ETH_WIN_BASE_REG");
		mvpp2_print_reg(ETH_WIN_SIZE_REG(win), "\t ETH_WIN_SIZE_REG");
		if (win < ETH_MAX_HIGH_ADDR_REMAP_WIN)
			mvpp2_print_reg(ETH_WIN_REMAP_REG(win), "\t ETH_WIN_REMAP_REG");
	}
}


void mvPp2TxSchedRegs(struct mvpp2_hw *hw, int port, int txp)
{
	int physTxp, txq;

	physTxp = MV_PPV2_TXP_PHYS(port, txp);

	printk("\n[TXP Scheduler registers: port=%d, txp=%d, physPort=%d]\n", port, txp, physTxp);

	mvPp2WrReg(MVPP2_TXP_SCHED_PORT_INDEX_REG, physTxp);
	mvpp2_print_reg(MVPP2_TXP_SCHED_PORT_INDEX_REG, "MVPP2_TXP_SCHED_PORT_INDEX_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_Q_CMD_REG, "MVPP2_TXP_SCHED_Q_CMD_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_CMD_1_REG, "MVPP2_TXP_SCHED_CMD_1_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_FIXED_PRIO_REG, "MVPP2_TXP_SCHED_FIXED_PRIO_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_PERIOD_REG, "MVPP2_TXP_SCHED_PERIOD_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_MTU_REG, "MVPP2_TXP_SCHED_MTU_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_REFILL_REG, "MVPP2_TXP_SCHED_REFILL_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_TOKEN_SIZE_REG, "MVPP2_TXP_SCHED_TOKEN_SIZE_REG");
	mvpp2_print_reg(MVPP2_TXP_SCHED_TOKEN_CNTR_REG, "MVPP2_TXP_SCHED_TOKEN_CNTR_REG");

	for (txq = 0; txq < MVPP2_MAX_TXQ; txq++) {
		printk("\n[TxQ Scheduler registers: port=%d, txp=%d, txq=%d]\n", port, txp, txq);
		mvpp2_print_reg(MVPP2_TXQ_SCHED_REFILL_REG(txq), "MVPP2_TXQ_SCHED_REFILL_REG");
		mvpp2_print_reg(MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq), "MVPP2_TXQ_SCHED_TOKEN_SIZE_REG");
		mvpp2_print_reg(MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq), "MVPP2_TXQ_SCHED_TOKEN_CNTR_REG");
	}
}

void      mvPp2FwdSwitchRegs(struct mvpp2_hw *hw)
{
	printk("\n[FWD Switch registers]\n");

	mvpp2_print_reg(MVPP2_FWD_SWITCH_FLOW_ID_REG, "MVPP2_FWD_SWITCH_FLOW_ID_REG");
	mvpp2_print_reg(MVPP2_FWD_SWITCH_CTRL_REG, "MVPP2_FWD_SWITCH_CTRL_REG");
	mvpp2_print_reg(MVPP2_FWD_SWITCH_STATUS_REG, "MVPP2_FWD_SWITCH_STATUS_REG");
}


void mvPp2BmPoolRegs(struct mvpp2_hw *hw, int pool)
{
	if (mvPp2MaxCheck(pool, MV_BM_POOLS, "bm_pool"))
		return;

	printk("\n[BM pool registers: pool=%d]\n", pool);
	mvpp2_print_reg(MV_BM_POOL_BASE_REG(pool), "MV_BM_POOL_BASE_REG");
	mvpp2_print_reg(MV_BM_POOL_SIZE_REG(pool), "MV_BM_POOL_SIZE_REG");
	mvpp2_print_reg(MV_BM_POOL_READ_PTR_REG(pool), "MV_BM_POOL_READ_PTR_REG");
	mvpp2_print_reg(MV_BM_POOL_PTRS_NUM_REG(pool), "MV_BM_POOL_PTRS_NUM_REG");
	mvpp2_print_reg(MV_BM_BPPI_READ_PTR_REG(pool), "MV_BM_BPPI_READ_PTR_REG");
	mvpp2_print_reg(MV_BM_BPPI_PTRS_NUM_REG(pool), "MV_BM_BPPI_PTRS_NUM_REG");
	mvpp2_print_reg(MV_BM_POOL_CTRL_REG(pool), "MV_BM_POOL_CTRL_REG");
	mvpp2_print_reg(MV_BM_INTR_CAUSE_REG(pool), "MV_BM_INTR_CAUSE_REG");
	mvpp2_print_reg(MV_BM_INTR_MASK_REG(pool), "MV_BM_INTR_MASK_REG");
}

void mvPp2V0DropCntrs(struct mvpp2_hw *hw, int port)
{
	int i;

	printk("\n[Port #%d Drop counters]\n", port);
	mvpp2_print_reg(MVPP2_OVERRUN_DROP_REG(MV_PPV2_PORT_PHYS(port)), "MVPP2_OVERRUN_DROP_REG");
	mvpp2_print_reg(MVPP2_CLS_DROP_REG(MV_PPV2_PORT_PHYS(port)), "MVPP2_CLS_DROP_REG");

	if (MVPP2_IS_PON_PORT(port)) {
		for (i = 0; i < mvPp2HalData.maxTcont; i++) {
			mvpp2_print_reg2(MVPP2_V0_TX_EARLY_DROP_REG(i), "MVPP2_TX_EARLY_DROP_REG", i);
			mvpp2_print_reg2(MVPP2_V0_TX_DESC_DROP_REG(i), "MVPP2_TX_DESC_DROP_REG", i);
		}
	} else {
		i = MVPP2_MAX_TCONT + port;
		mvpp2_print_reg2(MVPP2_V0_TX_EARLY_DROP_REG(i), "MVPP2_TX_EARLY_DROP_REG", i);
		mvpp2_print_reg2(MVPP2_V0_TX_DESC_DROP_REG(i), "MVPP2_TX_DESC_DROP_REG", i);
	}
	for (i = port * CONFIG_MVPP2_RXQ; i < (port * CONFIG_MVPP2_RXQ + CONFIG_MVPP2_RXQ); i++) {
		mvpp2_print_reg2(MVPP2_V0_RX_EARLY_DROP_REG(i), "MVPP2_RX_EARLY_DROP_REG", i);
		mvpp2_print_reg2(MVPP2_V0_RX_DESC_DROP_REG(i), "MVPP2_RX_DESC_DROP_REG", i);
	}
}

void mvPp2V1DropCntrs(struct mvpp2_hw *hw, int port)
{
	int txp, phyRxq, q;
	MVPP2_PORT_CTRL *pPortCtrl = mvPp2PortHndlGet(port);
	int physPort = MV_PPV2_PORT_PHYS(port);


	printk("\n[global drop counters]\n");
	mvPp2RegPrintNonZero(MVPP2_V1_OVERFLOW_MC_DROP_REG, "MVPP2_OVERRUN_DROP_REG");

	printk("\n[Port #%d Drop counters]\n", port);
	mvPp2RegPrintNonZero(MVPP2_OVERRUN_DROP_REG(physPort), "MVPP2_OVERRUN_DROP_REG");
	mvPp2RegPrintNonZero(MVPP2_CLS_DROP_REG(physPort), "MVPP2_CLS_DROP_REG");

	for (txp = 0; txp < pPortCtrl->txpNum; txp++) {
		for (q = 0; q < MVPP2_MAX_TXQ; q++) {
			printk("\n------ [Port #%d txp #%d txq #%d counters] -----\n", port, txp, q);
			mvPp2WrReg(MVPP2_V1_CNT_IDX_REG, TX_CNT_IDX(port, txp, q));
			mvPp2RegPrintNonZero(MVPP2_V1_TX_PKT_FULLQ_DROP_REG, "MVPP2_V1_TX_PKT_FULLQ_DROP_REG");
			mvPp2RegPrintNonZero(MVPP2_V1_TX_PKT_EARLY_DROP_REG, "MVPP2_V1_TX_PKT_EARLY_DROP_REG");
			mvPp2RegPrintNonZero(MVPP2_V1_TX_PKT_BM_DROP_REG, "MVPP2_V1_TX_PKT_BM_DROP_REG");
			mvPp2RegPrintNonZero(MVPP2_V1_TX_PKT_BM_MC_DROP_REG, "MVPP2_V1_TX_PKT_BM_MC_DROP_REG");
		}
	}

	for (q = 0; q < CONFIG_MVPP2_RXQ; q++) {
		printk("\n------ [Port #%d, rxq #%d counters] -----\n", port, q);
		phyRxq = mvPp2LogicRxqToPhysRxq(port, q);
		mvPp2WrReg(MVPP2_V1_CNT_IDX_REG, phyRxq);
		mvPp2RegPrintNonZero(MVPP2_V1_RX_PKT_FULLQ_DROP_REG, "MVPP2_V1_RX_PKT_FULLQ_DROP_REG");
		mvPp2RegPrintNonZero(MVPP2_V1_RX_PKT_EARLY_DROP_REG, "MVPP2_V1_RX_PKT_EARLY_DROP_REG");
		mvPp2RegPrintNonZero(MVPP2_V1_RX_PKT_BM_DROP_REG, "MVPP2_V1_RX_PKT_BM_DROP_REG");
	}
}

void mvPp2V1TxqDbgCntrs(struct mvpp2_hw *hw, int port, int txp, int txq)
{
	printk("\n------ [Port #%d txp #%d txq #%d counters] -----\n", port, txp, txq);
	mvPp2WrReg(MVPP2_V1_CNT_IDX_REG, TX_CNT_IDX(port, txp, txq));
	mvpp2_print_reg(MVPP2_V1_TX_DESC_ENQ_REG, "MVPP2_V1_TX_DESC_ENQ_REG");
	mvpp2_print_reg(MVPP2_V1_TX_DESC_ENQ_TO_DRAM_REG, "MVPP2_V1_TX_DESC_ENQ_TO_DRAM_REG");
	mvpp2_print_reg(MVPP2_V1_TX_BUF_ENQ_TO_DRAM_REG, "MVPP2_V1_TX_BUF_ENQ_TO_DRAM_REG");
	mvpp2_print_reg(MVPP2_V1_TX_DESC_HWF_ENQ_REG, "MVPP2_V1_TX_DESC_HWF_ENQ_REG");
	mvpp2_print_reg(MVPP2_V1_TX_PKT_DQ_REG, "MVPP2_V1_TX_PKT_DQ_REG");
	mvpp2_print_reg(MVPP2_V1_TX_PKT_FULLQ_DROP_REG, "MVPP2_V1_TX_PKT_FULLQ_DROP_REG");
	mvpp2_print_reg(MVPP2_V1_TX_PKT_EARLY_DROP_REG, "MVPP2_V1_TX_PKT_EARLY_DROP_REG");
	mvpp2_print_reg(MVPP2_V1_TX_PKT_BM_DROP_REG, "MVPP2_V1_TX_PKT_BM_DROP_REG");
	mvpp2_print_reg(MVPP2_V1_TX_PKT_BM_MC_DROP_REG, "MVPP2_V1_TX_PKT_BM_MC_DROP_REG");
}
#endif

void mvPp2V1RxqDbgCntrs(struct mvpp2 *priv, int port, int rxq)
{
	struct mvpp2_port *pp_port;
	int phy_rxq;
	struct mvpp2_hw *hw = &priv->hw;

	pp_port = mvpp2_port_struct_get(priv, port);
	if (pp_port)
		phy_rxq = pp_port->first_rxq + rxq;
	else
		return;

	printk("\n------ [Port #%d, rxq #%d counters] -----\n", port, rxq);
	mvpp2_write(hw, MVPP2_CNT_IDX_REG, phy_rxq);
	mvpp2_print_reg(hw, MVPP2_RX_PKT_FULLQ_DROP_REG, "MV_PP2_RX_PKT_FULLQ_DROP_REG");
	mvpp2_print_reg(hw, MVPP2_RX_PKT_EARLY_DROP_REG, "MVPP2_V1_RX_PKT_EARLY_DROP_REG");
	mvpp2_print_reg(hw, MVPP2_RX_PKT_BM_DROP_REG, "MVPP2_V1_RX_PKT_BM_DROP_REG");
	mvpp2_print_reg(hw, MVPP2_RX_DESC_ENQ_REG, "MVPP2_V1_RX_DESC_ENQ_REG");
}
EXPORT_SYMBOL(mvPp2V1RxqDbgCntrs);

#if 0
void mvPp2TxRegs(struct mvpp2_hw *hw)
{
	printk("\n[TX general registers]\n");

	mvpp2_print_reg(MVPP2_TX_SNOOP_REG, "MVPP2_TX_SNOOP_REG");
	mvpp2_print_reg(MVPP2_TX_FIFO_THRESH_REG, "MVPP2_TX_FIFO_THRESH_REG");
	mvpp2_print_reg(MVPP2_TX_PORT_FLUSH_REG, "MVPP2_TX_PORT_FLUSH_REG");
}
#endif

void mvPp2RxFifoRegs(struct mvpp2_hw *hw, int port)
{

	printk("\n[Port #%d RX Fifo]\n", port);
	mvpp2_print_reg(hw, MVPP2_RX_DATA_FIFO_SIZE_REG(port), "MVPP2_RX_DATA_FIFO_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_RX_ATTR_FIFO_SIZE_REG(port), "MVPP2_RX_ATTR_FIFO_SIZE_REG");
	printk("\n[Global RX Fifo regs]\n");
	mvpp2_print_reg(hw, MVPP2_RX_MIN_PKT_SIZE_REG, "MVPP2_RX_MIN_PKT_SIZE_REG");
}
EXPORT_SYMBOL(mvPp2RxFifoRegs);

#if 0
/* Print status of Ethernet port */
void mvPp2PortStatus(struct mvpp2_hw *hw, int port)
{
	int i, txp, txq;
	MV_ETH_PORT_STATUS	link;
	MVPP2_PORT_CTRL 	*pPortCtrl;

	if (mvPp2PortCheck(port))
		return;

	pPortCtrl = mvPp2PortHndlGet(port);
	if (!pPortCtrl)
		return;

	printk("\n[RXQ mapping: port=%d, ctrl=%p]\n", port, pPortCtrl);
	if (pPortCtrl->pRxQueue) {
		printk("         RXQ: ");
		for (i = 0; i < pPortCtrl->rxqNum; i++)
			printk(" %4d", i);

		printk("\nphysical RXQ: ");
		for (i = 0; i < pPortCtrl->rxqNum; i++) {
			if (pPortCtrl->pRxQueue[i])
				printk(" %4d", pPortCtrl->pRxQueue[i]->rxq);
			else
				printk(" NULL");
		}
		printk("\n");
	}

	printk("\n[BM queue to Qset mapping]\n");
	if (pPortCtrl->pRxQueue) {
		printk("       RXQ: ");
		for (i = 0; i < pPortCtrl->rxqNum; i++)
			printk(" %4d", i);

		printk("\n long Qset: ");
		for (i = 0; i < pPortCtrl->rxqNum; i++)
			printk(" %4d", mvBmRxqToQsetLongGet(mvPp2LogicRxqToPhysRxq(port, i)));

		printk("\nshort Qset: ");
		for (i = 0; i < pPortCtrl->rxqNum; i++)
			printk(" %4d", mvBmRxqToQsetShortGet(mvPp2LogicRxqToPhysRxq(port, i)));

		printk("\n");
	}
	if (pPortCtrl->pTxQueue) {
		for (txp = 0; txp < pPortCtrl->txpNum; txp++) {
			printk("\nTXP %2d, TXQ:", txp);
			for (txq = 0; txq < pPortCtrl->txqNum; txq++)
				printk(" %4d", txq);

			printk("\n long Qset: ");
			for (txq = 0; txq < pPortCtrl->txqNum; txq++)
				printk(" %4d", mvBmTxqToQsetLongGet(MV_PPV2_TXQ_PHYS(port, txp, txq)));

			printk("\nshort Qset: ");
			for (txq = 0; txq < pPortCtrl->txqNum; txq++)
				printk(" %4d", mvBmTxqToQsetShortGet(MV_PPV2_TXQ_PHYS(port, txp, txq)));

			printk("\n");
		}
	}

	printk("\n[Link: port=%d, ctrl=%p]\n", port, pPortCtrl);

	if (!MVPP2_IS_PON_PORT(port)) {

		mvGmacLinkStatus(port, &link);

		if (link.linkup) {
			printk("link up");
			printk(", %s duplex", (link.duplex == MV_ETH_DUPLEX_FULL) ? "full" : "half");
			printk(", speed ");

			if (link.speed == MV_ETH_SPEED_1000)
				printk("1 Gbps\n");
			else if (link.speed == MV_ETH_SPEED_100)
				printk("100 Mbps\n");
			else
				printk("10 Mbps\n");

			printk("rxFC - %s, txFC - %s\n",
				(link.rxFc == MV_ETH_FC_DISABLE) ? "disabled" : "enabled",
				(link.txFc == MV_ETH_FC_DISABLE) ? "disabled" : "enabled");
		} else
			printk("link down\n");
	}
}
#endif

static char *mvpp2_prs_l2_info_str(unsigned int l2_info)
{
	switch (l2_info << MVPP2_PRS_RI_L2_CAST_OFFS) {
	case MVPP2_PRS_RI_L2_UCAST:
		return "Ucast";
	case MVPP2_PRS_RI_L2_MCAST:
		return "Mcast";
	case MVPP2_PRS_RI_L2_BCAST:
		return "Bcast";
	default:
		return "Unknown";
	}
	return NULL;
}

static char *mvpp2_prs_vlan_info_str(unsigned int vlan_info)
{
	switch (vlan_info << MVPP2_PRS_RI_VLAN_OFFS) {
	case MVPP2_PRS_RI_VLAN_NONE:
		return "None";
	case MVPP2_PRS_RI_VLAN_SINGLE:
		return "Single";
	case MVPP2_PRS_RI_VLAN_DOUBLE:
		return "Double";
	case MVPP2_PRS_RI_VLAN_TRIPLE:
		return "Triple";
	default:
		return "Unknown";
	}
	return NULL;
}

void mvpp2_rx_desc_print(struct mvpp2_rx_desc *desc)
{
	int i;
	u32 *words = (u32 *) desc;

	printk("RX desc - %p: ", desc);
	for (i = 0; i < 8; i++)
		printk("%8.8x ", *words++);
	printk("\n");

	printk("pkt_size=%d, L3_offs=%d, IP_hlen=%d, ",
	       desc->data_size,
	       (desc->status & MVPP2_RXD_L3_OFFSET_MASK) >> MVPP2_RXD_L3_OFFSET_OFFS,
	       (desc->status & MVPP2_RXD_IP_HLEN_MASK) >> MVPP2_RXD_IP_HLEN_OFFS);

	printk("L2=%s, ",
		mvpp2_prs_l2_info_str((desc->rsrvd_parser & MVPP2_RXD_L2_CAST_MASK) >> MVPP2_RXD_L2_CAST_OFFS));

	printk("VLAN=");
	printk("%s, ",
		mvpp2_prs_vlan_info_str((desc->rsrvd_parser & MVPP2_RXD_VLAN_INFO_MASK) >> MVPP2_RXD_VLAN_INFO_OFFS));

	printk("L3=");
	if (MVPP2_RXD_L3_IS_IP4(desc->status))
		printk("IPv4 (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
	else if (MVPP2_RXD_L3_IS_IP4_OPT(desc->status))
		printk("IPv4 Options (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
	else if (MVPP2_RXD_L3_IS_IP4_OTHER(desc->status))
		printk("IPv4 Other (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
	else if (MVPP2_RXD_L3_IS_IP6(desc->status))
		printk("IPv6, ");
	else if (MVPP2_RXD_L3_IS_IP6_EXT(desc->status))
		printk("IPv6 Ext, ");
	else
		printk("Unknown, ");

	if (desc->status & MVPP2_RXD_IP_FRAG_MASK)
		printk("Frag, ");

	printk("L4=");
	if (MVPP2_RXD_L4_IS_TCP(desc->status))
		printk("TCP (csum=%s)", (desc->status & MVPP2_RXD_L4_CHK_OK_MASK) ? "Ok" : "Bad");
	else if (MVPP2_RXD_L4_IS_UDP(desc->status))
		printk("UDP (csum=%s)", (desc->status & MVPP2_RXD_L4_CHK_OK_MASK) ? "Ok" : "Bad");
	else
		printk("Unknown");

	printk("\n");

	printk("Lookup_ID=0x%x, cpu_code=0x%x\n",
		(desc->rsrvd_parser & MVPP2_RXD_LKP_ID_MASK) >> MVPP2_RXD_LKP_ID_OFFS,
		(desc->rsrvd_parser & MVPP2_RXD_CPU_CODE_MASK) >> MVPP2_RXD_CPU_CODE_OFFS);
}

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

/* Dump memory in specific format:
 * address: X1X1X1X1 X2X2X2X2 ... X8X8X8X8
 */
void mvpp2_skb_dump(struct sk_buff *skb, int size, int access)
{
	int i, j;
	u32 memAddr;
	u32 addr = skb->head + NET_SKB_PAD;

	printk("skb=%p, buf=%p, ksize=%d\n", skb, skb->head, ksize(skb->head));

	if (access == 0)
		access = 1;

	if ((access != 4) && (access != 2) && (access != 1)) {
		printk("%d wrong access size. Access must be 1 or 2 or 4\n", access);
		return;
	}
	memAddr = MV_ALIGN_DOWN((unsigned int)addr, 4);
	size = MV_ALIGN_UP(size, 4);
	addr = (void *)MV_ALIGN_DOWN((unsigned int)addr, access);
	while (size > 0) {
		printk("%08x: ", memAddr);
		i = 0;
		/* 32 bytes in the line */
		while (i < 32) {
			if (memAddr >= (u32)addr) {
				switch (access) {
				case 1:
					printk("%02x ", MV_MEMIO8_READ(memAddr));
					break;

				case 2:
					printk("%04x ", MV_MEMIO16_READ(memAddr));
					break;

				case 4:
					printk("%08x ", MV_MEMIO32_READ(memAddr));
					break;
				}
			} else {
				for (j = 0; j < (access * 2 + 1); j++)
					printk(" ");
			}
			i += access;
			memAddr += access;
			size -= access;
			if (size <= 0)
				break;
		}
		printk("\n");
	}
}
