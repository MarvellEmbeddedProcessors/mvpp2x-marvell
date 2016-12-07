/*
* ***************************************************************************
* Copyright (C) 2016 Marvell International Ltd.
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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/capability.h>

#ifdef ARMADA_390
#include "common/mv_sw_if.h"
#include "gop/mv_gop_if.h"
#include "gop/mv_ptp_regs.h"
#include "gop/mv_tai_regs.h"
#include "gop/mv_ptp_if.h"
#else
#include "mv_ptp_regs.h"
#include "mv_tai_regs.h"
#include "mv_ptp_if.h"
#include "mv_ptp_service.h"
#endif

static struct platform_device *pp2_sysfs;

/* PR_HLP() just for short source-code line:     PR_HLP("An example\n");
 * instead long-line:  o += scnprintf(b+o, PAGE_SIZE-o, "An example\n");
 */
#define PR_HLP(str)	{ o += scnprintf(b+o, PAGE_SIZE-o, (str)); }

static ssize_t mv_gop_ptp_help(char *b)
{
	int o = 0;
	/* NOTE: the sysfs-show limited with PAGE_SIZE. Current help-size is about 1.44kB */
	PR_HLP("\n");
	PR_HLP("cat              tai_regs  - show TAI unit registers\n");
	PR_HLP("cat              tai_tod   - show TAI time capture values\n");
	PR_HLP("cat              tai_clock - show TAI clock status\n");
	PR_HLP("cat              ptp_netif - get netdev-port mapping\n");
	PR_HLP("echo nsec      > 1pps_out_phase -  +/-nsec update_set (DEC)\n");
	PR_HLP("echo units     > freq_offs      -  +/-HEX units\n");
	PR_HLP("echo [p]       > ptp_regs  - show PTP unit registers\n");
	PR_HLP("echo [p] [0/1] > ptp_en    - enable(1) / disable(0) PTP unit\n");
	PR_HLP("echo [p]       > ptp_reset - reset given port PTP unit\n");
	PR_HLP("     [p] - mac (port) number\n");
	PR_HLP("----\n");
	PR_HLP("echo [h] [l] [n] > tai_tod_load_value  - set TAI TOD with DECimal\n");
	PR_HLP("         [h] hig16bit sec, [l] low32bit sec, [n] - nanosec\n");
	PR_HLP("\n");

	PR_HLP("--- TAI TOD operationS (HEX parameters)---\n");
	PR_HLP("echo [o] [h] [l] [n] > tai_op\n");
	PR_HLP("         [h] high sec, [l] low sec, [n] nanosec (HEX)\n");
	PR_HLP("     [o] OPeration (HEX all parameters)\n");
	PR_HLP("   ToD time:      [h]=0 must be present\n");
	PR_HLP("     1c -increment[l+n], 1c0 -graceful inc[l+n]\n");
	PR_HLP("     dc -decrement[l+n], 1d0 -graceful dec[l+n]\n");
	PR_HLP("   FREQ:\n");
	PR_HLP("     F1c / Fdc - inc/dec by value [h]\n");
	PR_HLP("   SYNC ToD time from/to linux or Sys/kernel:\n");
	PR_HLP("     41 - from linux, 21 - to linux\n");
	PR_HLP("     45 - from Sys/kernel, 47,46 -print ToD and System time\n");
	PR_HLP("   Tai-Clock cfg:\n");
	PR_HLP("     CE1 - Clock External Increment [h] seconds\n");
	PR_HLP("     CED - Clock External Decrement [h] seconds\n");
	PR_HLP("     CEA - Clock External Absolute set [h] seconds\n");
	PR_HLP("     CEC - Clock External Check stability & counter\n");
	PR_HLP("     CE  - Clock External (restart ToD=000..0)\n");
	PR_HLP("     C1  - Clock Internal (free-running)\n");
	PR_HLP("     C0  - Clock Off\n");
	PR_HLP("   DEBUG:\n");
	PR_HLP("     deb h l n  - DEBug-op with up to 3 parameters\n");
	PR_HLP("\n");
#ifdef MV_PTP_DEBUG
	PR_HLP("echo [r] [v]     > write_tai - [v]value to [r]TAI-reg\n");
	PR_HLP("echo [r]         > read_tai\n");
	PR_HLP("echo [r] [v] {p} > write_ptp - [v]value to [r]PTP-reg\n");
	PR_HLP("echo [r]     {p} > read_ptp    [r] is offs or addr=r*p\n");
	PR_HLP("echo [p] [q]     > tx_ts_get [pORT][qUEUE 0/1]\n");
	PR_HLP("\n");
#endif
	return o;
}

static ssize_t mv_ptp_netdev_name_get_sysfs(char *buf)
{
	int port, sz = 0;
	char name_buf[16/*IFNAMSIZ*/];

	for (port = 0; port < 4; port++) {
		if (!mv_ptp_netdev_name_get(port, name_buf))
			sz += scnprintf(buf + sz, PAGE_SIZE,
				"ptp_port_%d = <%s>\n", port, name_buf);
	}
	return sz;
}

static ssize_t mv_ptp_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help_ptp"))
		off = mv_gop_ptp_help(buf);
	else if (!strcmp(name, "tai_regs"))
		mv_pp3_tai_reg_dump();
	else if (!strcmp(name, "tai_tod"))
		mv_pp3_tai_tod_dump();
	else if (!strcmp(name, "tai_clock"))
		off = mv_pp3_tai_clock_status_get_sysfs(buf);
	else if (!strcmp(name, "ptp_netif"))
		off = mv_ptp_netdev_name_get_sysfs(buf);
	else {
		off = 1;
		pr_err("%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

	return off;
}

static ssize_t mv_ptp_sysfs_tai_tod_load(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	/* echo > tai_tod_load_value */
	unsigned long flags;
	int num;
	u32 h, l, n;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	n = 0;
	num = sscanf(buf, "%d %d %d", &h, &l, &n);
	if (num < 2)
		return -EINVAL;
	local_irq_save(flags);
	mv_pp3_tai_tod_load_set(h, l, n, 0);
	local_irq_restore(flags);
	return len;
}

static ssize_t mv_ptp_sysfs_tai_op(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	/* echo > tai_op */
	unsigned long flags;
	int num;
	u32 h, l, n, op;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	h = l = n = 0;
	num = sscanf(buf, "%x %x %x %x", &op, &h, &l, &n);
	if (num < 1)
		return -EINVAL;
	if (!op)
		return -EINVAL; /* likely wrong OP */
	local_irq_save(flags);
	mv_pp3_tai_tod_load_set(h, l, n, op);
	local_irq_restore(flags);
	return len;
}

static ssize_t mv_ptp_sysfs_1pps_out_phase(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	/* echo > tai_op */
	unsigned long flags;
	int rc,  nsec;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	rc = kstrtoint(buf, 0, &nsec);
	if (rc)
		return -EINVAL;
	/* Valid nano-second range is -0.5s .. 1sec */
	if ((nsec < -499999999) || (nsec > 999999999))
		return -EINVAL;

	local_irq_save(flags);
	mv_tai_1pps_out_phase_update(nsec);
	local_irq_restore(flags);
	return len;
}

static ssize_t mv_ptp_sysfs_freq_offs(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	/* echo > tai_op */
	int num, step_input, rc;
	u32 h, l, step;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	num = sscanf(buf, "-%x", &step_input);
	if (num == 1) {
		step_input = -step_input;
	} else {
		rc = kstrtoint(buf, 16, &step_input);
		if (rc)
			return -EINVAL;
	}
	h = mv_tai_reg_read(MV_TAI_TOD_STEP_FRAC_CFG_HIGH_REG);
	l = mv_tai_reg_read(MV_TAI_TOD_STEP_FRAC_CFG_LOW_REG);
	step = ((h & 0xFFFF) << 16) | (l & 0xFFFF);

	if (!step_input)
		step = 0;
	else
		step += step_input;

	h = step >> 16;
	l = step & 0xFFFF;
	mv_tai_reg_write(MV_TAI_TOD_STEP_FRAC_CFG_LOW_REG, l);
	mv_tai_reg_write(MV_TAI_TOD_STEP_FRAC_CFG_HIGH_REG, h);
	pr_info("Frequency STEP_FRAC_CFG = %d = 0x%04x.%04x\n",
		(int)step, h, l);
	return len;
}


static ssize_t mv_ptp_sysfs_store_2hex(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	const char      *name = attr->attr.name;
	unsigned long   flags;
	int             err, num;
	unsigned int    p, v, reg, addr, p1;
	u32 ts32bit;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	/* Read port and value */
	err = p = v = p1 = 0;
	num = sscanf(buf, "%x %x %x", &p, &v, &p1);
	if (num < 1)
		return -EINVAL;

	local_irq_save(flags);

	if (!strcmp(name, "ptp_en")) {
		mv_ptp_enable(p, (v) ? true : false);
		goto done;
	}
	if (!strcmp(name, "ptp_reset")) {
		mv_pp3_ptp_reset(p);
		goto done;
	}
	if (!strcmp(name, "ptp_regs")) {
		mv_pp3_ptp_reg_dump(p);
		goto done;
	}

#if !defined MV_PTP_DEBUG
	(void)addr;
	(void)reg;
#else

	if (!strcmp(name, "tx_ts_get")) {
		if (!MV_PTP_PORT_IS_VALID(p))
			goto err;
		ts32bit = mv_ptp_egress_tx_ts_32bit_get(p, v);
		mv_ptp_ts32bit_print(ts32bit, "TX");
		goto done;
	}

	/* Reg read/write with smart 6-digit-addressing
	 * (like 0x130408 used in regs-dump) or with Offset only
	 */
	addr = p;
	if (!strcmp(name, "write_tai")) {
		if (p >= 0x1000)
			p &= 0x0fff;
		reg = mv_tai_reg_read(p);
		mv_tai_reg_write(p, v);
		pr_info("TAI 0x%x/%x: %04x -> %04x\n", addr, p, reg, mv_tai_reg_read(p));
		goto done;
	}
	if (!strcmp(name, "read_tai")) {
		if (p >= 0x1000)
			p &= 0x0fff;
		pr_info("TAI 0x%x/%x: %04x\n", addr, p, mv_tai_reg_read(p));
		goto done;
	}
	if (!strcmp(name, "write_ptp")) {
		if (!MV_PTP_PORT_IS_VALID(p1))
			goto err;
		if (p >= 0x10000)
			p &= 0x0ffff;
		if ((p >= 0x1000) && p1)
			goto err;
		p += MV_PTP_PORT_BASE(p1);
		reg = mv_ptp_reg_read(p);
		mv_ptp_reg_write(p, v);
		pr_info("PTP 0x%x/%x: %04x -> %04x\n", addr, p, reg, mv_ptp_reg_read(p));
		goto done;
	}
	if (!strcmp(name, "read_ptp")) {
		p1 = v;
		if (!MV_PTP_PORT_IS_VALID(p1))
			goto err;
		if (p >= 0x10000)
			p &= 0x0ffff;
		if ((p >= 0x1000) && p1)
			goto err;
		p += MV_PTP_PORT_BASE(p1);
		pr_info("PTP 0x%x/%x: %04x\n", addr, p, mv_ptp_reg_read(p));
		goto done;
	}
err:
#endif /* MV_PTP_DEBUG */
/*err:*/
	/* error or unknown operation */
	local_irq_restore(flags);
	pr_err("ptp: illegal sysfs operation <%s>\n", name);
	return -EINVAL;

done:
	local_irq_restore(flags);
	return len;
}

static DEVICE_ATTR(help_ptp,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_regs,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_clock,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(ptp_netif,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_tod,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_tod_load_value,	S_IWUSR, NULL, mv_ptp_sysfs_tai_tod_load);
static DEVICE_ATTR(tai_op,		S_IWUSR, NULL, mv_ptp_sysfs_tai_op);
static DEVICE_ATTR(1pps_out_phase,	S_IWUSR, NULL, mv_ptp_sysfs_1pps_out_phase);
static DEVICE_ATTR(freq_offs,	S_IWUSR, NULL, mv_ptp_sysfs_freq_offs);

static DEVICE_ATTR(ptp_regs,		S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(ptp_en,		S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(ptp_reset,		S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
#ifdef MV_PTP_DEBUG
static DEVICE_ATTR(write_tai,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(read_tai,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(write_ptp,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(read_ptp,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(tx_ts_get,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
#endif

static struct attribute *mv_gop_ptp_attrs[] = {
	&dev_attr_help_ptp.attr,
	&dev_attr_tai_regs.attr,
	&dev_attr_tai_clock.attr,
	&dev_attr_tai_tod.attr,
	&dev_attr_tai_tod_load_value.attr,
	&dev_attr_tai_op.attr,
	&dev_attr_1pps_out_phase.attr,
	&dev_attr_freq_offs.attr,
	&dev_attr_ptp_netif.attr,
	&dev_attr_ptp_regs.attr,
	&dev_attr_ptp_en.attr,
	&dev_attr_ptp_reset.attr,
#ifdef MV_PTP_DEBUG
	&dev_attr_write_tai.attr,
	&dev_attr_read_tai.attr,
	&dev_attr_write_ptp.attr,
	&dev_attr_read_ptp.attr,
	&dev_attr_tx_ts_get.attr,
#endif
	NULL
};

static struct attribute_group mv_gop_ptp_group = {
	/* .name = "ptp",  <-- from kobject_create_and_add */
	.attrs = mv_gop_ptp_attrs,
};

static bool mv_sysfs_ptp_initialized;

int mv_ptp_sysfs_init(char *dev_name, char *subdir)
{
	/* dev_name: "pp2", "pp3" ... */
	struct device *pd;
	int err;
	struct kobject *parent_kobj, *ptp_kobj;

	if (mv_sysfs_ptp_initialized)
		return 0;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, dev_name);
	if (!pd) {
		pp2_sysfs = platform_device_register_simple("pp2", -1, NULL, 0);
		pd = bus_find_device_by_name(&platform_bus_type, NULL, dev_name);
	}
	if (!pd) {
		pr_err("ptp: cannot find parent pdev <%s> for sysfs\n", dev_name);
		return -1;
	}
	parent_kobj = &pd->kobj;
	if (subdir) {
		ptp_kobj = kobject_create_and_add(subdir, parent_kobj);
		if (!ptp_kobj)
			parent_kobj = ptp_kobj;
	}

	ptp_kobj = kobject_create_and_add("ptp", parent_kobj);
	if (!ptp_kobj) {
		pr_err("ptp: cannot create sysfs <%s/ptp>\n", dev_name);
		return -ENOMEM;
	}

	err = sysfs_create_group(ptp_kobj, &mv_gop_ptp_group);
	if (err)
		pr_err("ptp: sysfs group ptp error=%d\n", err);
	else
		mv_sysfs_ptp_initialized = true;
	return err;
}

int mv_sysfs_ptp_exit(struct kobject *kobj)
{
	sysfs_remove_group(kobj, &mv_gop_ptp_group);
	if (pp2_sysfs)
		platform_device_unregister(pp2_sysfs);
	return 0;
}

