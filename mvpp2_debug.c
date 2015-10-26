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


