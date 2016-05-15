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
#endif


static ssize_t mv_gop_ptp_help(char *b)
{
	int o = 0;
	/* NOTE: the sysfs-show limited with PAGE_SIZE. Current help-size is about 1.43kB */
	o += scnprintf(b+o, PAGE_SIZE-o, "\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "cat              tai_regs  - show TAI unit registers\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "cat              tai_tod   - show TAI time capture values\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "cat              tai_clock - show TAI clock status\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [p]       > ptp_regs  - show PTP unit registers\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [p] [0/1] > ptp_en    - enable(1) / disable(0) PTP unit\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [p]       > ptp_reset - reset given port PTP unit\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     [p] - mac (port) number\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "----\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [h] [l] [n] > tai_tod_load_value  - set TAI TOD with DECimal\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "         [h] hig16bit sec, [l] low32bit sec, [n] - nanosec\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "\n");

	o += scnprintf(b+o, PAGE_SIZE-o, "--- TAI TOD operationS (HEX parameters)---\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [o] [h] [l] [n] > tai_op\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "         [h] high sec, [l] low sec, [n] nanosec (HEX)\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     [o] OPeration (HEX all parameters)\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "   ToD time:      [h]=0 must be present\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     1c -increment[l+n], 1c0 -graceful inc[l+n]\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     dc -decrement[l+n], 1d0 -graceful dec[l+n]\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "   FREQ:\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     F1c / Fdc - inc/dec by value [h]\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "   SYNC ToD time from/to linux or Sys/kernel:\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     41 - from linux, 21 - to linux\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     45 - from Sys/kernel, 47,46 -print ToD and System time\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "   Tai-Clock cfg:\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     CE1 - Clock External Increment [h] seconds\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     CED - Clock External Decrement [h] seconds\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     CEA - Clock External Absolute set [h] seconds\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     CEC - Clock External Check stability & counter\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     C1  - Clock Internal (free-running)\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     C0  - Clock Off\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     CEB11 - Blink led on gpio=11\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "   DEBUG:\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "     deb h l n  - DEBug-op with up to 3 parameters\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [r] [v] > reg_write_ptp  -  [v]value to [r]PTP-reg\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "echo [r] [v] > reg_write_tai  -  [v]value to [r]TAI-reg\n");
	o += scnprintf(b+o, PAGE_SIZE-o, "\n");

	return o;
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

static ssize_t mv_ptp_sysfs_store_2hex(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	const char      *name = attr->attr.name;
	unsigned long   flags;
	int             err, num;
	unsigned int    p, v;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	/* Read port and value */
	err = p = v = 0;
	num = sscanf(buf, "%x %x", &p, &v);
	if (num < 1)
		return -EINVAL;

	local_irq_save(flags);

	if (!strcmp(name, "ptp_en"))
		mv_ptp_enable(p, (v) ? true : false);
	else if (!strcmp(name, "ptp_reset"))
		mv_pp3_ptp_reset(p);
	else if (!strcmp(name, "ptp_regs"))
		mv_pp3_ptp_reg_dump(p);
	else if (!strcmp(name, "reg_write_ptp"))
		mv_ptp_reg_write(p, v);
	else if (!strcmp(name, "reg_write_tai"))
		mv_tai_reg_write(p, v);
	else
		err = 1;

	local_irq_restore(flags);

	if (err) {
		pr_err("ptp: illegal sysfs operation <%s>\n", name);
		return -EINVAL;
	}
	return len;
}

static DEVICE_ATTR(help_ptp,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_regs,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_clock,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_tod,		S_IRUSR, mv_ptp_sysfs_show, NULL);
static DEVICE_ATTR(tai_tod_load_value,	S_IWUSR, NULL, mv_ptp_sysfs_tai_tod_load);
static DEVICE_ATTR(tai_op,		S_IWUSR, NULL, mv_ptp_sysfs_tai_op);
static DEVICE_ATTR(ptp_regs,		S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(ptp_en,		S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(ptp_reset,		S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(reg_write_ptp,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);
static DEVICE_ATTR(reg_write_tai,	S_IWUSR, NULL, mv_ptp_sysfs_store_2hex);

static struct attribute *mv_gop_ptp_attrs[] = {
	&dev_attr_help_ptp.attr,
	&dev_attr_tai_regs.attr,
	&dev_attr_tai_clock.attr,
	&dev_attr_tai_tod.attr,
	&dev_attr_tai_tod_load_value.attr,
	&dev_attr_tai_op.attr,
	&dev_attr_ptp_regs.attr,
	&dev_attr_ptp_en.attr,
	&dev_attr_ptp_reset.attr,
	&dev_attr_reg_write_ptp.attr,
	&dev_attr_reg_write_tai.attr,
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
	return 0;
}

