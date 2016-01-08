/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/mbus.h>
#include <linux/inetdevice.h>
#include <linux/interrupt.h>
//#include <linux/mv_pp2.h>
#include <asm/setup.h>
#include <net/ip.h>
#include <net/ipv6.h>



//#include "mvOs.h"
//#include "mvDebug.h"
//#include "mvEthPhy.h"

//#include "gbe/mvPp2Gbe.h"
//#include "prs/mvPp2Prs.h"
//#include "prs/mvPp2PrsHw.h"
//#include "cls/mvPp2Classifier.h"

#include "mvPp2Common.h"
#include "mv_eth_sysfs.h"

#include "mvpp2.h"



//#include "dpi/mvPp2DpiHw.h"
//#include "wol/mvPp2Wol.h"


struct mvpp2 *sysfs_cur_priv;
struct mvpp2_hw *sysfs_cur_hw;
static struct platform_device * pp2_sysfs;


extern void mvpp2_pp2_basic_print(struct platform_device *pdev, struct mvpp2 *priv);

static int mv_pp2_sysfs_init(void)
{
	struct device *pd;
	struct device *pp2_dev;
	struct platform_device *pp2_plat_dev;
	struct mvpp2 *priv;
	char *pp2_dev_name = "f10f0000.ethernet";


	pd = bus_find_device_by_name(&platform_bus_type, NULL, "pp2");
	if (!pd) {
		pp2_sysfs = platform_device_register_simple("pp2", -1, NULL, 0);
		pd = bus_find_device_by_name(&platform_bus_type, NULL, "pp2");
	}

	if (!pd) {
		printk(KERN_ERR"%s: cannot find sysfs pp2 device\n", __func__);
		return -1;
	}
	pp2_dev = bus_find_device_by_name(&platform_bus_type, NULL, pp2_dev_name);
	if (!pp2_dev) {
		printk(KERN_ERR"%s: cannot find device %s\n", __func__, pp2_dev_name);
		return -1;
	}

	priv = dev_get_drvdata(pp2_dev);
	pp2_plat_dev = to_platform_device(pp2_dev);
	mvpp2_pp2_basic_print(pp2_plat_dev, priv);
	sysfs_cur_priv = priv;
	sysfs_cur_hw = &priv->hw;

	mv_pp2_prs_high_sysfs_init(&pd->kobj);
	mv_pp2_cls_sysfs_init(&pd->kobj);
	mv_pp2_cls2_sysfs_init(&pd->kobj);
	//mv_pp2_cls3_sysfs_init(&pd->kobj);
	//mv_pp2_cls4_sysfs_init(&pd->kobj);
	//mv_pp2_wol_sysfs_init(&pd->kobj);
	//mv_pp2_plcr_sysfs_init(&pd->kobj);
	//mv_pp2_pme_sysfs_init(&pd->kobj);
	//mv_pp2_mc_sysfs_init(&pd->kobj);

	mv_pp2_bm_sysfs_init(&pd->kobj);
	mv_pp2_rx_sysfs_init(&pd->kobj);
	mv_pp2_tx_sysfs_init(&pd->kobj);

	mv_pp2_rss_sysfs_init(&pd->kobj);

	mv_pp2_cos_sysfs_init(&pd->kobj);

	mv_pp2_debug_sysfs_init(&pd->kobj);

	mv_pp2_tx_sched_sysfs_init(&pd->kobj);
//	mv_pp2_qos_sysfs_init(gbe_kobj);
//	mv_pp2_gbe_pme_sysfs_init(gbe_kobj);
//#ifdef CONFIG_MV_PP2_HWF
//	mv_pp2_gbe_hwf_sysfs_init(gbe_kobj);
//#endif



	mv_pp2_gbe_sysfs_init(&pd->kobj);
//	mv_pp2_dbg_sysfs_init(&pd->kobj);

	return 0;
}

static void mv_pp2_sysfs_exit(void)
{
	struct device *pd;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "pp2");
	if (!pd) {
		printk(KERN_ERR"%s: cannot find pp2 device\n", __func__);
		return;
	}
//#ifdef CONFIG_MV_PP2_L2FW
//	mv_pp2_l2fw_sysfs_exit(&pd->kobj);
//#endif

//#ifdef CONFIG_MV_ETH_PP2_1
//	mv_pp2_dpi_sysfs_exit(&pd->kobj);
//#endif

//	mv_pp2_wol_sysfs_exit(&pd->kobj);
//	mv_pp2_pme_sysfs_exit(&pd->kobj);

//	mv_pp2_mc_sysfs_exit(&pd->kobj);
//	mv_pp2_cls4_sysfs_exit(&pd->kobj);
//	mv_pp2_cls3_sysfs_exit(&pd->kobj);
	mv_pp2_prs_high_sysfs_exit(&pd->kobj);
	mv_pp2_cls2_sysfs_exit(&pd->kobj);
	mv_pp2_cls_sysfs_exit(&pd->kobj);
	mv_pp2_bm_sysfs_exit(&pd->kobj);
	mv_pp2_rx_sysfs_exit(&pd->kobj);
	mv_pp2_tx_sysfs_exit(&pd->kobj);
	mv_pp2_rss_sysfs_exit(&pd->kobj);
	mv_pp2_cos_sysfs_exit(&pd->kobj);
	mv_pp2_debug_sysfs_exit(&pd->kobj);
	mv_pp2_tx_sched_sysfs_exit(&pd->kobj);
//	mv_pp2_plcr_sysfs_exit(&pd->kobj);
	mv_pp2_gbe_sysfs_exit(&pd->kobj);
	/* can't delete, we call to init/clean function from this sysfs */
	/* TODO: open this line when we delete clean/init sysfs commands*/
	/*mv_pp2_dbg_sysfs_exit(&pd->kobj);*/
	platform_device_unregister(pp2_sysfs);
}

static int __init mpp2_sysfs_module_init(void)
{
	mv_pp2_sysfs_init();

	return(0);
}

static void __exit mpp2_sysfs_module_exit(void)
{
	mv_pp2_sysfs_exit();
	return;
}

module_init(mpp2_sysfs_module_init);
module_exit(mpp2_sysfs_module_exit);


MODULE_DESCRIPTION("Marvell PPv2 Sysfs Commands- www.marvell.com");
MODULE_LICENSE("GPL v2");


