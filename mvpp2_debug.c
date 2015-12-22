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
#include "mvpp2_debug.h"



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

static void mvPp2RxQueueDetailedShow(struct mvpp2 *priv, struct mvpp2_rx_queue *pp_rxq)
{
	int i;
	struct mvpp2_rx_desc *rx_desc = pp_rxq->first_desc;

	for (i = 0; i < pp_rxq->size; i++) {
		printk("%3d. desc=%p, status=%08x, data_size=%4d",
			   i, rx_desc+i, rx_desc[i].status, rx_desc[i].data_size);
		if (priv->pp2_version == PPV21) {
			printk("buf_addr=%lx, buf_cookie=%p",
				   mvpp21_rxdesc_phys_addr_get(rx_desc),
				   mvpp21_rxdesc_cookie_get(rx_desc));
		}
		else {
			printk("buf_addr=%lx, buf_cookie=%p",
				   mvpp22_rxdesc_phys_addr_get(rx_desc),
				   mvpp22_rxdesc_cookie_get(rx_desc));
		}

		printk("parser_info=%03x\n", rx_desc->rsrvd_parser);
	}
}

/* Show Port/Rxq descriptors ring */
void mvPp2RxqShow(struct mvpp2 *priv, int port, int rxq, int mode)
{
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

	if (pp_rxq->first_desc == NULL) {
		printk("rxq #%d of port #%d is not initialized\n", rxq, port);
		return;
	}

	printk("\n[PPv2 RxQ show: port=%d, logical rxq=%d -> physical rxq=%d]\n",
			port, pp_rxq->log_id, pp_rxq->id);

	printk("size=%d, pkts_coal=%d, time_coal=%d\n",pp_rxq->size, pp_rxq->pkts_coal, pp_rxq->time_coal);

	printk("first_virt_addr=%p, first_dma_addr=%lx, next_rx_desc=%d, rxq_cccupied=%d, rxq_nonoccupied=%d\n",
	pp_rxq->first_desc, MVPP2_DESCQ_MEM_ALIGN(pp_rxq->descs_phys), pp_rxq->next_desc_to_proc,
	mvpp2_rxq_received(pp_port, pp_rxq->id), mvpp2_rxq_free(pp_port, pp_rxq->id));
	printk("virt_mem_area_addr=%p, dma_mem_area_addr=%lx\n", pp_rxq->desc_mem, pp_rxq->descs_phys);

	if (mode)
		mvPp2RxQueueDetailedShow(priv, pp_rxq);
}
EXPORT_SYMBOL(mvPp2RxqShow);

void mvPp2PhysRxqRegs(struct mvpp2 *pp2, int rxq)
{
	struct mvpp2_hw *hw  = &pp2->hw;

	printk("\n[PPv2 RxQ registers: global rxq=%d]\n", rxq);

	mvpp2_write(hw, MVPP2_RXQ_NUM_REG, rxq);
	mvpp2_print_reg(hw, MVPP2_RXQ_NUM_REG, "MVPP2_RXQ_NUM_REG");
	mvpp2_print_reg(hw, MVPP2_RXQ_DESC_ADDR_REG, "MVPP2_RXQ_DESC_ADDR_REG");
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


static void mvPp2TxQueueDetailedShow(struct mvpp2 *priv, void *pp_txq, bool aggr_queue)
{
	int i, j, size;
	struct mvpp2_tx_desc *tx_desc;

	if (aggr_queue) {
		size = ((struct mvpp2_aggr_tx_queue *)pp_txq)->size;
		tx_desc = ((struct mvpp2_aggr_tx_queue *)pp_txq)->first_desc;
	} else {
		size = ((struct mvpp2_tx_queue *)pp_txq)->size;
		tx_desc = ((struct mvpp2_tx_queue *)pp_txq)->first_desc;
	}

	for (i = 0; i < 16/*size*/; i++) {
		printk("%3d. desc=%p, cmd=%x, data_size=%-4d pkt_offset=%-3d, phy_txq=%d\n",
			   i, tx_desc+i, tx_desc[i].command, tx_desc[i].data_size,
			   tx_desc[i].packet_offset, tx_desc[i].phys_txq);
		if (priv->pp2_version == PPV21) {
			printk("buf_phys_addr=%08x, buf_cookie=%08x\n",
				tx_desc[i].u.pp21.buf_phys_addr, tx_desc[i].u.pp21.buf_cookie);
			printk( "hw_cmd[0]=%x, hw_cmd[1]=%x, hw_cmd[2]=%x, rsrvd1=%x\n",
				tx_desc[i].u.pp21.rsrvd_hw_cmd[0],
				tx_desc[i].u.pp21.rsrvd_hw_cmd[1],
				tx_desc[i].u.pp21.rsrvd_hw_cmd[2],
				tx_desc[i].u.pp21.rsrvd1);
		}
		else {
			printk("     rsrvd_hw_cmd1=%llx, buf_phys_addr_cmd2=%llx, buf_cookie_bm_cmd3=%llx \n",
				tx_desc[i].u.pp22.rsrvd_hw_cmd1,
				tx_desc[i].u.pp22.buf_phys_addr_hw_cmd2,
				tx_desc[i].u.pp22.buf_cookie_bm_qset_hw_cmd3);

			for(j=0;j<8;j++) {
				printk("%d:%x\n", j, *((u32 *)(tx_desc+i)+j));
			}
		}
	}
}



/* Show Port/TXQ descriptors ring */
void mvPp2TxqShow(struct mvpp2 *priv, int port, int txq, int mode)
{
	struct mvpp2_port *pp_port;
	struct mvpp2_tx_queue *pp_txq;
	struct mvpp2_txq_pcpu *txq_pcpu;
	int cpu;

	pp_port = mvpp2_port_struct_get(priv, port);

	if (pp_port == NULL) {
		printk("port #%d is not initialized\n", port);
		return;
	}

	if (mvpp2_max_check(txq, pp_port->num_tx_queues, "logical txq"))
		return;

	pp_txq = pp_port->txqs[txq];

	if (pp_txq->first_desc == NULL) {
		printk("txq #%d of port #%d is not initialized\n", txq, port);
		return;
	}

	printk("\n[PPv2 TxQ show: port=%d, logical_txq=%d]\n", port, pp_txq->log_id);

	printk("physical_txq=%d, size=%d, pkts_coal=%d \n",pp_txq->id, pp_txq->size, pp_txq->pkts_coal);

	printk("first_virt_addr=%p, first_dma_addr=%lx, next_tx_desc=%d\n",
		   pp_txq->first_desc, MVPP2_DESCQ_MEM_ALIGN(pp_txq->descs_phys), pp_txq->next_desc_to_proc);
	printk("virt_mem_area_addr=%p, dma_mem_area_addr=%lx\n", pp_txq->desc_mem, pp_txq->descs_phys);

	for_each_online_cpu(cpu) {
		txq_pcpu = per_cpu_ptr(pp_txq->pcpu, cpu);
		printk("\n[PPv2 TxQ %d cpu=%d show:\n", txq, cpu);

		printk("cpu=%d, size=%d, count=%d reserved_num=%d\n",
			txq_pcpu->cpu, txq_pcpu->size, txq_pcpu->count, txq_pcpu->reserved_num);
		printk("txq_put_index=%d, txq_get_index=%d\n",
			txq_pcpu->txq_put_index, txq_pcpu->txq_get_index);
		printk("tx_skb=%p, tx_buffs=%p\n",
			txq_pcpu->tx_skb, txq_pcpu->tx_buffs);
	}

	if (mode)
		mvPp2TxQueueDetailedShow(priv, pp_txq, 0);
}
EXPORT_SYMBOL(mvPp2TxqShow);


/* Show CPU aggregation TXQ descriptors ring */
void mvPp2AggrTxqShow(struct mvpp2 *priv, int cpu, int mode)
{

	struct mvpp2_aggr_tx_queue *aggr_queue = NULL;
	int i;

	printk("\n[PPv2 AggrTxQ: cpu=%d]\n", cpu);

	for (i=0;i<priv->num_aggr_qs;i++) {
		if (priv->aggr_txqs[i].id == cpu) {
			aggr_queue = &priv->aggr_txqs[i];
			break;
		}
	}
	if (!aggr_queue) {
		printk("aggr_txq for cpu #%d is not initialized\n", cpu);
		return;
	}

	printk("id=%d, size=%d, count=%d, next_desc=%d, pending_cntr=%d\n", aggr_queue->id,
		aggr_queue->size, aggr_queue->count,aggr_queue->next_desc_to_proc,
		mvpp2_aggr_desc_num_read(priv,cpu));

	if (mode)
		mvPp2TxQueueDetailedShow(priv, aggr_queue, 1);

}
EXPORT_SYMBOL(mvPp2AggrTxqShow);

void mvPp2PhysTxqRegs(struct mvpp2 *priv, int txq)
{
	struct mvpp2_hw *hw = &priv->hw;

	printk("\n[PPv2 TxQ registers: global txq=%d]\n", txq);

	if (mvpp2_max_check(txq, MVPP2_TXQ_TOTAL_NUM, "global txq"))
		return;

	mvpp2_write(hw, MVPP2_TXQ_NUM_REG, txq);
	mvpp2_print_reg(hw, MVPP2_TXQ_NUM_REG, "MVPP2_TXQ_NUM_REG");
	mvpp2_print_reg(hw, MVPP2_TXQ_DESC_ADDR_LOW_REG, "MVPP2_TXQ_DESC_ADDR_LOW_REG");
	if (priv->pp2_version == PPV22)
		mvpp2_print_reg(hw, MVPP22_TXQ_DESC_ADDR_HIGH_REG, "MVPP22_TXQ_DESC_ADDR_HIGH_REG");

	mvpp2_print_reg(hw, MVPP2_TXQ_DESC_SIZE_REG, "MVPP2_TXQ_DESC_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_TXQ_DESC_HWF_SIZE_REG, "MVPP2_TXQ_DESC_HWF_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_TXQ_INDEX_REG, "MVPP2_TXQ_INDEX_REG");
	mvpp2_print_reg(hw, MVPP2_TXQ_PREF_BUF_REG, "MVPP2_TXQ_PREF_BUF_REG");
	mvpp2_print_reg(hw, MVPP2_TXQ_PENDING_REG, "MVPP2_TXQ_PENDING_REG");
	mvpp2_print_reg(hw, MVPP2_TXQ_INT_STATUS_REG, "MVPP2_TXQ_INT_STATUS_REG");
	if (priv->pp2_version == PPV21)
		mvpp2_print_reg(hw, MVPP21_TXQ_SENT_REG(txq), "MVPP21_TXQ_SENT_REG");
	else
		mvpp2_print_reg(hw, MVPP22_TXQ_SENT_REG(txq), "MVPP22_TXQ_SENT_REG");
}
EXPORT_SYMBOL(mvPp2PhysTxqRegs);

void mvPp2PortTxqRegs(struct mvpp2 *priv, int port, int txq)
{
	struct mvpp2_port *pp2_port;

	pp2_port = mvpp2_port_struct_get(priv, port);

	if (mvpp2_max_check(txq, pp2_port->num_tx_queues, "port txq"))
		return;

	printk("\n[PPv2 TxQ registers: port=%d, local txq=%d]\n", port, txq);

	mvPp2PhysTxqRegs(priv, pp2_port->txqs[txq]->id);
}
EXPORT_SYMBOL(mvPp2PortTxqRegs);

void mvPp2AggrTxqRegs(struct mvpp2 *priv, int cpu)
{
	struct mvpp2_hw *hw = &priv->hw;

	printk("\n[PP2 Aggr TXQ registers: cpu=%d]\n", cpu);

	mvpp2_print_reg(hw, MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu), "MVPP2_AGGR_TXQ_DESC_ADDR_REG");
	mvpp2_print_reg(hw, MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu), "MVPP2_AGGR_TXQ_DESC_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_AGGR_TXQ_STATUS_REG(cpu), "MVPP2_AGGR_TXQ_STATUS_REG");
	mvpp2_print_reg(hw, MVPP2_AGGR_TXQ_INDEX_REG(cpu), "MVPP2_AGGR_TXQ_INDEX_REG");
}
EXPORT_SYMBOL(mvPp2AggrTxqRegs);

void mvPp2V1TxqDbgCntrs(struct mvpp2 *priv, int port, int txq)
{

	struct mvpp2_hw *hw = &priv->hw;

	printk("\n------ [Port #%d txq #%d counters] -----\n", port, txq);
	mvpp2_write(hw, MVPP2_CNT_IDX_REG, MVPP2_CNT_IDX_TX(port, txq));
	mvpp2_print_reg(hw, MVPP2_CNT_IDX_REG, "MVPP2_CNT_IDX_REG");
	mvpp2_print_reg(hw, MVPP2_TX_DESC_ENQ_REG, "MVPP2_TX_DESC_ENQ_REG");
	mvpp2_print_reg(hw, MVPP2_TX_DESC_ENQ_TO_DRAM_REG, "MVPP2_TX_DESC_ENQ_TO_DRAM_REG");
	mvpp2_print_reg(hw, MVPP2_TX_BUF_ENQ_TO_DRAM_REG, "MVPP2_TX_BUF_ENQ_TO_DRAM_REG");
	mvpp2_print_reg(hw, MVPP2_TX_DESC_HWF_ENQ_REG, "MVPP2_TX_DESC_HWF_ENQ_REG");
	mvpp2_print_reg(hw, MVPP2_TX_PKT_DQ_REG, "MVPP2_TX_PKT_DQ_REG");
	mvpp2_print_reg(hw, MVPP2_TX_PKT_FULLQ_DROP_REG, "MVPP2_TX_PKT_FULLQ_DROP_REG");
	mvpp2_print_reg(hw, MVPP2_TX_PKT_EARLY_DROP_REG, "MVPP2_TX_PKT_EARLY_DROP_REG");
	mvpp2_print_reg(hw, MVPP2_TX_PKT_BM_DROP_REG, "MVPP2_TX_PKT_BM_DROP_REG");
	mvpp2_print_reg(hw, MVPP2_TX_PKT_BM_MC_DROP_REG, "MVPP2_TX_PKT_BM_MC_DROP_REG");
}
EXPORT_SYMBOL(mvPp2V1TxqDbgCntrs);


void mvPp2TxRegs(struct mvpp2 *priv)
{
	struct mvpp2_hw *hw = &priv->hw;
	int i;
	printk("\n[TX general registers]\n");

	mvpp2_print_reg(hw, MVPP2_TX_SNOOP_REG, "MVPP2_TX_SNOOP_REG");
	if (priv->pp2_version == PPV21) {
		mvpp2_print_reg(hw, MVPP21_TX_FIFO_THRESH_REG, "MVPP21_TX_FIFO_THRESH_REG");
	} else {
		for (i=0;i<MVPP2_MAX_PORTS;i++) {
			mvpp2_print_reg(hw, MVPP22_TX_FIFO_THRESH_REG(i), "MVPP22_TX_FIFO_THRESH_REG");
		}
	}
	mvpp2_print_reg(hw, MVPP2_TX_PORT_FLUSH_REG, "MVPP2_TX_PORT_FLUSH_REG");
}
EXPORT_SYMBOL(mvPp2TxRegs);

void mvPp2TxSchedRegs(struct mvpp2 *priv, int port)
{
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);
	int physTxp, txq;

	physTxp = mvpp2_egress_port(pp2_port);

	printk("\n[TXP Scheduler registers: port=%d, physPort=%d]\n", port, physTxp);

	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, physTxp);
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, "MV_PP2_TXP_SCHED_PORT_INDEX_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_Q_CMD_REG, "MV_PP2_TXP_SCHED_Q_CMD_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_CMD_1_REG, "MV_PP2_TXP_SCHED_CMD_1_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_FIXED_PRIO_REG, "MV_PP2_TXP_SCHED_FIXED_PRIO_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_PERIOD_REG, "MV_PP2_TXP_SCHED_PERIOD_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_MTU_REG, "MV_PP2_TXP_SCHED_MTU_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_REFILL_REG, "MV_PP2_TXP_SCHED_REFILL_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, "MV_PP2_TXP_SCHED_TOKEN_SIZE_REG");
	mvpp2_print_reg(hw, MVPP2_TXP_SCHED_TOKEN_CNTR_REG, "MV_PP2_TXP_SCHED_TOKEN_CNTR_REG");

	for (txq = 0; txq < MVPP2_MAX_TXQ; txq++) {
		printk("\n[TxQ Scheduler registers: port=%d, txq=%d]\n", port, txq);
		mvpp2_print_reg(hw, MVPP2_TXQ_SCHED_REFILL_REG(txq), "MV_PP2_TXQ_SCHED_REFILL_REG");
		mvpp2_print_reg(hw, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq), "MV_PP2_TXQ_SCHED_TOKEN_SIZE_REG");
		mvpp2_print_reg(hw, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq), "MV_PP2_TXQ_SCHED_TOKEN_CNTR_REG");
	}
}
EXPORT_SYMBOL(mvPp2TxSchedRegs);

/* Calculate period and tokens accordingly with required rate and accuracy */
int mvPp2RateCalc(int rate, unsigned int accuracy, unsigned int *pPeriod, unsigned int *pTokens)
{
	/* Calculate refill tokens and period - rate [Kbps] = tokens [bits] * 1000 / period [usec] */
	/* Assume:  Tclock [MHz] / BasicRefillNoOfClocks = 1 */
	unsigned int period, tokens, calc;

	if (rate == 0) {
		/* Disable traffic from the port: tokens = 0 */
		if (pPeriod != NULL)
			*pPeriod = 1000;

		if (pTokens != NULL)
			*pTokens = 0;

		return 0;
	}

	/* Find values of "period" and "tokens" match "rate" and "accuracy" when period is minimal */
	for (period = 1; period <= 1000; period++) {
		tokens = 1;
		while (1)	{
			calc = (tokens * 1000) / period;
			if (((MV_ABS(calc - rate) * 100) / rate) <= accuracy) {
				if (pPeriod != NULL)
					*pPeriod = period;

				if (pTokens != NULL)
					*pTokens = tokens;

				return 0;
			}
			if (calc > rate)
				break;

			tokens++;
		}
	}
	return -1;
}

/* Set bandwidth limitation for TX port
 *   rate [Kbps]    - steady state TX bandwidth limitation
 */
int mvPp2TxpRateSet(struct mvpp2 *priv, int port, int rate)
{
	u32 regVal;
	unsigned int tokens, period, txPortNum, accuracy = 0;
	int status;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);

	if (port >= MVPP2_MAX_PORTS)
		return -1;

	txPortNum = mvpp2_egress_port(pp2_port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	regVal = mvpp2_read(hw, MVPP2_TXP_SCHED_PERIOD_REG);

	status = mvPp2RateCalc(rate, accuracy, &period, &tokens);
	if (status != MV_OK) {
		printk("%s: Can't provide rate of %d [Kbps] with accuracy of %d [%%]\n",
				__func__, rate, accuracy);
		return status;
	}
	if (tokens > MVPP2_TXP_REFILL_TOKENS_MAX)
		tokens = MVPP2_TXP_REFILL_TOKENS_MAX;

	if (period > MVPP2_TXP_REFILL_PERIOD_MAX)
		period = MVPP2_TXP_REFILL_PERIOD_MAX;

	regVal = mvpp2_read(hw, MVPP2_TXP_SCHED_REFILL_REG);

	regVal &= ~MVPP2_TXP_REFILL_TOKENS_ALL_MASK ;
	regVal |= MVPP2_TXP_REFILL_TOKENS_MASK(tokens);

	regVal &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	regVal |= MVPP2_TXP_REFILL_PERIOD_MASK(period);

	mvpp2_write(hw, MVPP2_TXP_SCHED_REFILL_REG, regVal);

	return 0;
}
EXPORT_SYMBOL(mvPp2TxpRateSet);

/* Set maximum burst size for TX port
 *   burst [bytes] - number of bytes to be sent with maximum possible TX rate,
 *                    before TX rate limitation will take place.
 */
int mvPp2TxpBurstSet(struct mvpp2 *priv, int port, int burst)
{
	u32 size, mtu;
	int txPortNum;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);

	if (port >= MVPP2_MAX_PORTS)
		return -1;

	txPortNum = mvpp2_egress_port(pp2_port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Calulate Token Bucket Size */
	size = 8 * burst;

	if (size > MVPP2_TXP_TOKEN_SIZE_MAX)
		size = MVPP2_TXP_TOKEN_SIZE_MAX;

	/* Token bucket size must be larger then MTU */
	mtu = mvpp2_read(hw, MVPP2_TXP_SCHED_MTU_REG);
	if (mtu > size) {
		printk("%s Error: Bucket size (%d bytes) < MTU (%d bytes)\n",
					__func__, (size / 8), (mtu / 8));
		return -1;
	}
	mvpp2_write(hw, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, size);

	return 0;
}
EXPORT_SYMBOL(mvPp2TxpBurstSet);

/* Set bandwidth limitation for TXQ
 *   rate  [Kbps]  - steady state TX rate limitation
 */
int mvPp2TxqRateSet(struct mvpp2 *priv, int port, int txq, int rate)
{
	u32		regVal;
	unsigned int	txPortNum, period, tokens, accuracy = 0;
	int	status;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);

	if (port >= MVPP2_MAX_PORTS)
		return -1;

	if (txq >= MVPP2_MAX_TXQ)
		return -1;

	status = mvPp2RateCalc(rate, accuracy, &period, &tokens);
	if (status != MV_OK) {
		printk("%s: Can't provide rate of %d [Kbps] with accuracy of %d [%%]\n",
				__func__, rate, accuracy);
		return status;
	}

	txPortNum = mvpp2_egress_port(pp2_port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	if (tokens > MVPP2_TXQ_REFILL_TOKENS_MAX)
		tokens = MVPP2_TXQ_REFILL_TOKENS_MAX;

	if (period > MVPP2_TXQ_REFILL_PERIOD_MAX)
		period = MVPP2_TXQ_REFILL_PERIOD_MAX;

	regVal = mvpp2_read(hw, MVPP2_TXQ_SCHED_REFILL_REG(txq));

	regVal &= ~MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
	regVal |= MVPP2_TXQ_REFILL_TOKENS_MASK(tokens);

	regVal &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
	regVal |= MVPP2_TXQ_REFILL_PERIOD_MASK(period);

	mvpp2_write(hw, MVPP2_TXQ_SCHED_REFILL_REG(txq), regVal);

	return 0;
}
EXPORT_SYMBOL(mvPp2TxqRateSet);

/* Set maximum burst size for TX port
 *   burst [bytes] - number of bytes to be sent with maximum possible TX rate,
 *                    before TX bandwidth limitation will take place.
 */
int mvPp2TxqBurstSet(struct mvpp2 *priv, int port, int txq, int burst)
{
	u32  size, mtu;
	int txPortNum;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);

	if (port >= MVPP2_MAX_PORTS)
		return -1;

	if (txq >= MVPP2_MAX_TXQ)
		return -1;

	txPortNum = mvpp2_egress_port(pp2_port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Calulate Tocket Bucket Size */
	size = 8 * burst;

	if (size > MVPP2_TXQ_TOKEN_SIZE_MAX)
		size = MVPP2_TXQ_TOKEN_SIZE_MAX;

	/* Tocken bucket size must be larger then MTU */
	mtu = mvpp2_read(hw, MVPP2_TXP_SCHED_MTU_REG);
	if (mtu > size) {
		printk("%s Error: Bucket size (%d bytes) < MTU (%d bytes)\n",
					__func__, (size / 8), (mtu / 8));
		return -1;
	}

	mvpp2_write(hw, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq), size);

	return 0;
}
EXPORT_SYMBOL(mvPp2TxqBurstSet);

/* Set TXQ to work in FIX priority mode */
int mvPp2TxqFixPrioSet(struct mvpp2 *priv, int port, int txq)
{
	u32 regVal;
	int txPortNum;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);

	if (port >= MVPP2_MAX_PORTS)
		return -1;

	if (txq >= MVPP2_MAX_TXQ)
		return -1;

	txPortNum = mvpp2_egress_port(pp2_port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	regVal = mvpp2_read(hw, MVPP2_TXP_SCHED_FIXED_PRIO_REG);
	regVal |= (1 << txq);
	mvpp2_write(hw, MVPP2_TXP_SCHED_FIXED_PRIO_REG, regVal);

	return MV_OK;
}
EXPORT_SYMBOL(mvPp2TxqFixPrioSet);

/* Set TXQ to work in WRR mode and set relative weight. */
/*   Weight range [1..N] */
int mvPp2TxqWrrPrioSet(struct mvpp2 *priv, int port, int txq, int weight)
{
	u32 regVal, mtu, mtu_aligned, weight_min;
	int txPortNum;
	struct mvpp2_hw *hw = &priv->hw;
	struct mvpp2_port *pp2_port = mvpp2_port_struct_get(priv, port);

	if (port >= MVPP2_MAX_PORTS)
		return -1;

	if (txq >= MVPP2_MAX_TXQ)
		return -1;

	txPortNum = mvpp2_egress_port(pp2_port);
	mvpp2_write(hw, MVPP2_TXP_SCHED_PORT_INDEX_REG, txPortNum);

	/* Weight * 256 bytes * 8 bits must be larger then MTU [bits] */
	mtu = mvpp2_read(hw, MVPP2_TXP_SCHED_MTU_REG);

	/* WA for wrong Token bucket update: Set MTU value = 3*real MTU value, now get read MTU*/
	mtu /= MV_AMPLIFY_FACTOR_MTU;
	mtu /= MV_BIT_NUM_OF_BYTE; /* move to bytes */
	mtu_aligned = MV_ALIGN_UP(mtu, MV_WRR_WEIGHT_UNIT);
	weight_min = mtu_aligned / MV_WRR_WEIGHT_UNIT;

	if ((weight < weight_min) || (weight > MVPP2_TXQ_WRR_WEIGHT_MAX)) {
		printk("%s Error: weight=%d is out of range %d...%d\n",
				__func__, weight, weight_min, MVPP2_TXQ_WRR_WEIGHT_MAX);
		return -1;
	}

	regVal = mvpp2_read(hw, MVPP2_TXQ_SCHED_WRR_REG(txq));

	regVal &= ~MVPP2_TXQ_WRR_WEIGHT_ALL_MASK;
	regVal |= MVPP2_TXQ_WRR_WEIGHT_MASK(weight);
	mvpp2_write(hw, MVPP2_TXQ_SCHED_WRR_REG(txq), regVal);

	regVal = mvpp2_read(hw, MVPP2_TXP_SCHED_FIXED_PRIO_REG);
	regVal &= ~(1 << txq);
	mvpp2_write(hw, MVPP2_TXP_SCHED_FIXED_PRIO_REG, regVal);

	return 0;
}
EXPORT_SYMBOL(mvPp2TxqWrrPrioSet);

#if 0
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

void mvpp2_rx_desc_print(struct mvpp2 *priv, struct mvpp2_rx_desc *desc)
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

	if (priv->pp2_version == PPV22) {
		printk("buf_phys_addr = 0x%llx\n", desc->u.pp22.buf_phys_addr_key_hash & DMA_BIT_MASK(40));
		printk("buf_virt_addr = 0x%llx\n", desc->u.pp22.buf_cookie_bm_qset_cls_info & DMA_BIT_MASK(40));
	}
}

/* Dump memory in specific format:
 * address: X1X1X1X1 X2X2X2X2 ... X8X8X8X8
 */
void mvpp2_skb_dump(struct sk_buff *skb, int size, int access)
{
	int i, j;
	void *addr = skb->head + NET_SKB_PAD;
	u32 memAddr = (u32)addr;

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

/* Wrap the API just for debug */
int mvpp2_wrap_cos_mode_set(struct mvpp2_port *port, enum mvpp2_cos_classifier cos_mode)
{
	return mvpp2_cos_classifier_set(port, cos_mode);
}
EXPORT_SYMBOL(mvpp2_wrap_cos_mode_set);

int mvpp2_wrap_cos_mode_get(struct mvpp2_port *port)
{
	return mvpp2_cos_classifier_get(port);
}
EXPORT_SYMBOL(mvpp2_wrap_cos_mode_get);

int mvpp2_wrap_cos_pri_map_set(struct mvpp2_port *port, int cos_pri_map)
{
	return mvpp2_cos_pri_map_set(port, cos_pri_map);
}
EXPORT_SYMBOL(mvpp2_wrap_cos_pri_map_set);

int mvpp2_wrap_cos_pri_map_get(struct mvpp2_port *port)
{
	return mvpp2_cos_pri_map_get(port);
}
EXPORT_SYMBOL(mvpp2_wrap_cos_pri_map_get);

int mvpp2_wrap_cos_dflt_value_set(struct mvpp2_port *port, int cos_value)
{
	return mvpp2_cos_default_value_set(port, cos_value);
}
EXPORT_SYMBOL(mvpp2_wrap_cos_dflt_value_set);

int mvpp2_wrap_cos_dflt_value_get(struct mvpp2_port *port)
{
	return mvpp2_cos_default_value_get(port);
}
EXPORT_SYMBOL(mvpp2_wrap_cos_dflt_value_get);

int mvpp22_wrap_rss_mode_set(struct mvpp2_port *port, int rss_mode)
{
	return mvpp22_rss_mode_set(port, rss_mode);
}
EXPORT_SYMBOL(mvpp22_wrap_rss_mode_set);

int mvpp22_wrap_rss_dflt_cpu_set(struct mvpp2_port *port, int default_cpu)
{
	return mvpp22_rss_default_cpu_set(port, default_cpu);
}
EXPORT_SYMBOL(mvpp22_wrap_rss_dflt_cpu_set);

/* mvpp2_port_bind_cpu_set
*  -- Bind the port to cpu when rss disabled.
*/
int mvpp2_port_bind_cpu_set(struct mvpp2_port *port, u8 bind_cpu)
{
	int ret = 0;
	u8 bound_cpu_first_rxq;

	if (port->priv->pp2_cfg.rss_cfg.rss_en) {
		netdev_err(port->dev, "cannot bind cpu to port when rss is enabled\n");
		return -EINVAL;
	}

	if (!(port->priv->cpu_map & (1 << bind_cpu))) {
		netdev_err(port->dev, "invalid cpu(%d)\n", bind_cpu);
		return -EINVAL;
	}

	/* Check original cpu and new cpu is same or not */
	if (bind_cpu != ((port->priv->pp2_cfg.rx_cpu_map >> (port->id * 4)) & 0xF)) {
		port->priv->pp2_cfg.rx_cpu_map &= (~(0xF << (port->id * 4)));
		port->priv->pp2_cfg.rx_cpu_map |= ((bind_cpu & 0xF) << (port->id * 4));
		bound_cpu_first_rxq = mvpp2_bound_cpu_first_rxq_calc(port);
		ret = mvpp2_cls_c2_rule_set(port, bound_cpu_first_rxq);
	}

	return ret;
}
EXPORT_SYMBOL(mvpp2_port_bind_cpu_set);

int mvpp2_cls_c2_qos_prio_set(struct mvpp2_cls_c2_qos_entry *qos, u8 pri)
{
	if (!qos)
		return -EINVAL;

	qos->data &= ~MVPP2_CLS2_QOS_TBL_PRI_MASK;
	qos->data |= (((u32)pri) << MVPP2_CLS2_QOS_TBL_PRI_OFF);
	return 0;
}
EXPORT_SYMBOL(mvpp2_cls_c2_qos_prio_set);

int mvpp2_cls_c2_qos_dscp_set(struct mvpp2_cls_c2_qos_entry *qos, u8 dscp)
{
	if (!qos)
		return -EINVAL;

	qos->data &= ~MVPP2_CLS2_QOS_TBL_DSCP_MASK;
	qos->data |= (((u32)dscp) << MVPP2_CLS2_QOS_TBL_DSCP_OFF);
	return 0;
}
EXPORT_SYMBOL(mvpp2_cls_c2_qos_dscp_set);

int mvpp2_cls_c2_qos_color_set(struct mvpp2_cls_c2_qos_entry *qos, u8 color)
{
	if (!qos)
		return -EINVAL;

	qos->data &= ~MVPP2_CLS2_QOS_TBL_COLOR_MASK;
	qos->data |= (((u32)color) << MVPP2_CLS2_QOS_TBL_COLOR_OFF);
	return 0;
}
EXPORT_SYMBOL(mvpp2_cls_c2_qos_color_set);

int mvpp2_cls_c2_queue_set(struct mvpp2_cls_c2_entry *c2, int cmd, int queue, int from)
{
	int status = 0;
	int qHigh, qLow;

	/* cmd validation in set functions */

	qHigh = (queue & MVPP2_CLS2_ACT_QOS_ATTR_QH_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QH_OFF;
	qLow = (queue & MVPP2_CLS2_ACT_QOS_ATTR_QL_MASK) >> MVPP2_CLS2_ACT_QOS_ATTR_QL_OFF;

	status |= mvpp2_cls_c2_queue_low_set(c2, cmd, qLow, from);
	status |= mvpp2_cls_c2_queue_high_set(c2, cmd, qHigh, from);

	return status;
}
EXPORT_SYMBOL(mvpp2_cls_c2_queue_set);

int mvpp2_cls_c2_mtu_set(struct mvpp2_cls_c2_entry *c2, int mtu_inx)
{
	PTR_VALIDATE(c2);
	POS_RANGE_VALIDATE(mtu_inx, (1 << MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_BITS) - 1);

	c2->sram.regs.hwf_attr &= ~MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_MASK;
	c2->sram.regs.hwf_attr |= (mtu_inx << MVPP2_CLS2_ACT_HWF_ATTR_MTUIDX_OFF);

	return MV_OK;
}
EXPORT_SYMBOL(mvpp2_cls_c2_mtu_set);

