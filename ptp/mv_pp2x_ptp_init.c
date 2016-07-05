/*
* ***************************************************************************
* Copyright (C) 2015 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/



#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/mbus.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include "mv_pp2x.h"

#include <mv_tai_regs.h>
#include <mv_ptp_if.h>
#include <mv_ptp_service.h>

static int mv_pp2x_ptp_map_init(struct platform_device *pdev,
	struct mv_pp2x *priv)
{
	/* TAI and PTP should be aligned to Page-Size for dev/uio */
	struct mv_tai_ptp_map *m, map;
	void *base;
	struct resource *res;

	m = mv_tai_ptp_map_get();
	if (m->num)
		return 1; /* already initialized */

	/* TAI from .dtb */
	res = platform_get_resource_byname(pdev,
		IORESOURCE_MEM, "tai");
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	/* UIO remap must be page aligned => extend/align mapping
	 * This is safety inside the page.
	 */
	map.tai_base_pa = res->start & ~(PAGE_SIZE - 1);
	map.tai_base_va = (phys_addr_t)((u64)base  & ~(PAGE_SIZE - 1));
	map.tai_size = PAGE_SIZE;

	/* PTP is already in "mspg", but PhAddr needed */
	base = priv->hw.gop.gop_110.mspg_base;/* + 0x0800 */
	if (IS_ERR(base))
		return PTR_ERR(base);
	res = platform_get_resource_byname(pdev,
		IORESOURCE_MEM, "mspg");
	map.ptp_base_pa = res->start;
	map.ptp_base_va = (phys_addr_t)base;
	map.ptp_size = 0x1000 * MVPP2_MAX_PORTS;

	map.num = 2; /* = 1tai + 1ptp; do this last*/

	mv_tai_ptp_map_init(&map);
	return 0;
}


/* mv_pp2x_ptp_init:
 *  Called from mv_pp2x_probe() -> mv_pp2x_platform_data_get()
 */
int mv_pp2x_ptp_init(struct platform_device *pdev,
	struct mv_pp2x_port *p_port, int port)
{
	/* TAI clock init (must be after gop) */
	u32 tclk_hz = MV_PP2_TAI_CLK_FREQ_HZ;
	struct device_node *dn = pdev->dev.of_node;
	int ret;
	bool common_required;
	struct mv_pp2x *priv = p_port->priv;

	if (priv->pp2_version != PPV22)
		return -EINVAL;
	/* WORKAROUND: Skip the "f4000000.ppv22",
	 * only CPU_0 "f2000000.ppv22" shared-dev supported
	 */
	if (strcmp("f2000000.ppv22", pdev->name))
		return 0;

	ret = mv_pp2x_ptp_map_init(pdev, priv);
	if (ret < 0)
		return ret;

	common_required = !ret;
	if (common_required) {
		of_property_read_u32(dn, "clock-frequency", &tclk_hz);
		pr_info("tai clock-frequency = %u MHz\n", tclk_hz/1000000);
		mv_ptp_tclk_hz_set(tclk_hz);
		mv_tai_clock_init(pdev);
		mv_ptp_sysfs_init("pp2", NULL);
		mv_ptp_tai_tod_uio_init(pdev);
	}

	mv_pp2x_ptp_hook_init(priv, port);
	if (!mv_ptp_enable(port, true))
		pr_info("ptp port_%d enabled on netdev <%s>\n",
			port, p_port->dev->name);
	return 0;
}
