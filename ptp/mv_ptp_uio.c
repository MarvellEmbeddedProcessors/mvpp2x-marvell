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

/******************************************************************************
**
**      UIO deriver and device adding
**
** Gives direct access to TAI/PTP registers in user-space over UIO-mapping
** Complicated actions are executed over ".set = write_store_cmd":
**     write to /sys/module/mv_pp3/parameters/ts_tai_tod_uio
**
** Every XXX-UIO device requires CONFIG_UIO::uio_register_device().
** As an alternative for CONFIG_UIO=y Kernel built-in there could be
** an external-module CONFIG_UIO=m or
** tricky built-in <drivers/uio/uio.c> into this ptp selected by define
** MV_PTP_SERVICE_UIO_DEV_BUILT_IN (refer at the end of file).
**
** If no CONFIG_UIO and no MV_PTP_SERVICE_UIO_DEV_BUILT_IN enabled,
** this PTP-UIO provides empty stub function mv_ptp_tai_tod_uio_init()
*******************************************************************************
*/

/* includes */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

#if !defined(MV_PTP_SERVICE_UIO) && !defined(CONFIG_UIO)
int mv_ptp_tai_tod_uio_init(struct platform_device *shared_pdev)
{
	pr_info("ptp: run without dev/uio\n");
	return 0;
}
#else /* CONFIG_UIO */

#ifdef ARMADA_390
#include "gop/a390_mg_if.h"
#include "gop/mv_gop_if.h"
#include "gop/mv_ptp_regs.h"
#include "gop/mv_tai_regs.h"
#include "net_dev/mv_ptp_service.h"
#else
#include "mv_ptp_regs.h"
#include "mv_tai_regs.h"
#include "mv_ptp_if.h"
#include "mv_ptp_service.h"
#endif


#define TS_NAME	"ts_tai_tod"
#define TS_NAME_UIO	"ts_tai_tod_uio"

struct ts_ptp_uio {
	struct uio_info uio_info;
	/* Auxiliary parameters */
	u32 dedicated_mg_region;
	u32 dedicated_mg_region_offs;
	u32 num_maps;
};

static struct ts_ptp_uio *ts_ptp_uio;

static int read_show_cmd(char *buf, const struct kernel_param *kp)
{
	int i, offs;
	struct ts_ptp_uio *p = (void *)(*(phys_addr_t *)(kp->arg));

	if (!p)
		goto end;
	if (p->dedicated_mg_region_offs) {
		sprintf(buf, "region=%d region_offs=%x size=%lx (%s)",
			p->dedicated_mg_region, p->dedicated_mg_region_offs,
			(unsigned long)p->uio_info.mem[0].size, p->uio_info.mem[0].name);
	} else {
		offs = 0;
		for (i = 0; i < ts_ptp_uio->num_maps; i++)
			offs += sprintf(buf + offs, "%s addr=%p size=%lx\n",
				p->uio_info.mem[0].name,
				(void *)p->uio_info.mem[i].addr,
				(unsigned long)p->uio_info.mem[0].size);
	}
	/*pr_debug("%s device is used with parameters: %s\n", kp->name, buf);*/
end:
	return strlen(buf);
}

static int write_store_cmd(const char *buf, const struct kernel_param *kp)
{
	struct mv_pp3_tai_tod *ts;
	int rc;

	if (buf[0]) {
		pr_err("%s: write/store called with ASCII: <%s>\n", TS_NAME_UIO, buf);
		return 0;
	}
	ts = (struct mv_pp3_tai_tod *)buf;
	if (ts->operation == MV_TAI_GET_CAPTURE) {
		/* For DEBUG only */
		/* mv_pp3_tai_tod_op(ts->operation, ts, 0); - already in "ts" */
		mv_pp3_tai_tod_dump_util(ts);
		return 0;
	}
	/* Real operation called over "parameters/ts_tai_tod_uio" write
	 * as an alternative to the UIO direct access in user space
	 */
	rc = mv_pp3_tai_tod_op(ts->operation, ts, 0);
	return rc;
}

static const struct kernel_param_ops param_ops = {
	.get = read_show_cmd,
	.set = write_store_cmd,
};
module_param_cb(ts_tai_tod_uio, &param_ops, &ts_ptp_uio, 0644);


static int ts_ptp_uio_probe(struct platform_device *pdev)
{
	int ret, i;
	struct mv_tai_ptp_map *m;
	phys_addr_t gop_pa[2];
	phys_addr_t gop_va[2];
	u32 gop_sz[2];

	ts_ptp_uio = kzalloc(sizeof(struct ts_ptp_uio), GFP_KERNEL);
	if (!ts_ptp_uio)
		return -ENOMEM;

#ifdef MV_PP3_DEDICATED_MG_REGION
	/* Address-convert for TAI/PTP registers
	 *   TAI: 0x03180A00..0x03180B00
	 *   PTP: 0x03180800..0x03180874 ... port3:0x03183800..0x03183874
	 *   => TAI/PTP Register(offset) 0318pXXX
	 * With indirect MG address completion the final Mapping is:
	 *    0318pXXX -> REGION7(111b << 19) -> 0038pXXX
	 * TAI-access un User-space is like:
	 *    *(u32*)(gop_va + 0x00380000 + RegisterOFFS)
	 *
	 * Without dedicated region the UIO has no mmap and could be used
	 * by application only over read_show_cmd, write_store_cmd
	 */
	ts_ptp_uio->dedicated_mg_region = MV_PP3_DEDICATED_MG_REGION;
	ts_ptp_uio->dedicated_mg_region_offs = MV_PP3_DEDICATED_MG_REGION << 19;
	ts_ptp_uio->num_maps = 1;
	ret = mv_gop_addrs_size_get(&gop_va[0], &gop_pa[0], &gop_sz[0]);
	(void *)m;
#else
	m = mv_tai_ptp_map_get();
	ret = !m->tai_base_pa || !m->tai_base_va || !m->tai_size
		|| !m->ptp_base_pa || !m->ptp_base_va || !m->ptp_size;
	gop_pa[0] = m->tai_base_pa;
	gop_va[0] = m->tai_base_va;
	gop_sz[0] = m->tai_size;
	gop_pa[1] = m->ptp_base_pa;
	gop_va[1] = m->ptp_base_va;
	gop_sz[1] = m->ptp_size;
	ts_ptp_uio->num_maps = m->num;
#endif
	ts_ptp_uio->uio_info.name = TS_NAME;
	ts_ptp_uio->uio_info.version = "v1";
	ts_ptp_uio->uio_info.priv = ts_ptp_uio;

	for (i = 0; i < ts_ptp_uio->num_maps; i++) {
		ts_ptp_uio->uio_info.mem[i].name = "common";
		ts_ptp_uio->uio_info.mem[i].addr = gop_pa[i];
		ts_ptp_uio->uio_info.mem[i].internal_addr = (void *)gop_va[i];
		ts_ptp_uio->uio_info.mem[i].size = gop_sz[i];
		ts_ptp_uio->uio_info.mem[i].memtype = UIO_MEM_PHYS;
	}
	if (ts_ptp_uio->num_maps == 2) {
		ts_ptp_uio->uio_info.mem[0].name = "tai";
		ts_ptp_uio->uio_info.mem[1].name = "ptp";
	}

	if (ret) {
		pr_err("%s failed, wrong mapping\n", __func__);
		goto err;
	}
	platform_set_drvdata(pdev, ts_ptp_uio);
	if (uio_register_device(&pdev->dev, &ts_ptp_uio->uio_info)) {
		pr_err("%s: register device fails!\n", __func__);
		goto err;
	}
	return 0;

err:
	kfree(ts_ptp_uio);
	ts_ptp_uio = NULL;
	ret = 0; /* say OK to continue sys-up without this driver */
	return ret;
}

static struct platform_driver ts_tai_tod_driver = {
	.driver = {
		.name = TS_NAME,
		.owner = THIS_MODULE
	},
};


#ifdef MV_PTP_SERVICE_UIO_DEV_BUILT_IN
static int __init uio_init(void);
static void __exit uio_exit(void);
#else
#define uio_init()
#define uio_exit()
#endif

int mv_ptp_tai_tod_uio_init(struct platform_device *shared_pdev)
{
	/* This probe-init is extension of mv_pp3_shared_probe()
	 * but called in very late stage (!) as part of PTP init.
	 * It is using the shared_pdev imported from net_dev
	 */
	int rc;

	/* Could be called more than once but only 1 created */
	if (ts_ptp_uio)
		return 0;

	if (!shared_pdev)
		return 0;

	uio_init(); /* Blank ot <../drivers/uio/uio.c> */

	rc = ts_ptp_uio_probe(shared_pdev);
	if (!rc)
		rc = platform_driver_register(&ts_tai_tod_driver);

	if (rc)
		pr_err("%s: Can't register %s driver. rc=%d\n", __func__, TS_NAME, rc);
	return rc;
}

void mv_ptp_tai_tod_uio_exit(void)
{
	uio_exit();
}

#ifdef MV_PTP_SERVICE_UIO_DEV_BUILT_IN
#undef module_init
#undef module_exit
#define module_init(uio_init)	/*blank*/
#define module_exit(uio_exit)	/*blank*/
#include <../drivers/uio/uio.c>
#endif


MODULE_AUTHOR("Yan Markman");
MODULE_DESCRIPTION("UIO driver for Marvell TAI-ToD");
MODULE_LICENSE("GPL");

#endif /* MV_PTP_SERVICE_UIO / CONFIG_UIO */
