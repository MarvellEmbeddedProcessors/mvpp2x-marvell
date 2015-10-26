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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/capability.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include "mvPp2Common.h"



static ssize_t mv_cls2_help(char *buf)
{
	int off = 0;

	off += scnprintf(buf + off, PAGE_SIZE, "cat  prio_hw_dump - dump all QoS priority tables from HW.\n");
	off += scnprintf(buf + off, PAGE_SIZE, "cat  dscp_hw_dump - dump all QoS dscp tables from HW.\n");
	off += scnprintf(buf + off, PAGE_SIZE, "cat  act_hw_dump  - dump all action table enrties from HW.\n");
	off += scnprintf(buf + off, PAGE_SIZE, "cat  hw_regs      - dump classifier C2 registers.\n");
	off += scnprintf(buf + off, PAGE_SIZE, "cat  cnt_dump     - dump all hit counters that are not zeroed.\n");

	off += scnprintf(buf + off, PAGE_SIZE, "echo               > cnt_clr_all            - clear all hit counters from action tabe.\n");
	off += scnprintf(buf + off, PAGE_SIZE, "echo idx           > cnt_read               - show hit counter for action table entry.\n");
	off += scnprintf(buf + off, PAGE_SIZE, "\n");
	return off;
}


static ssize_t mv_cls2_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "prio_hw_dump"))
		off += mvpp2_cls_c2_qos_prio_hw_dump(sysfs_cur_hw);
	else if (!strcmp(name, "dscp_hw_dump"))
		off += mvpp2_cls_c2_qos_dscp_hw_dump(sysfs_cur_hw);
	else if (!strcmp(name, "act_hw_dump"))
		off += mvpp2_cls_c2_hw_dump(sysfs_cur_hw);
	else if (!strcmp(name, "cnt_dump"))
		off += mvpp2_cls_c2_hit_cntr_dump(sysfs_cur_hw);
	else if (!strcmp(name, "hw_regs"))
		off += mvpp2_cls_c2_regs_dump(sysfs_cur_hw);
	else
		off += mv_cls2_help(buf);

	return off;
}


static ssize_t mv_cls2_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t len)
{
	const char    *name = attr->attr.name;
	unsigned int  err = 0, a = 0, b = 0, c = 0, d = 0, e = 0;
	unsigned long flags;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	sscanf(buf, "%x %x %x %x %x", &a, &b, &c, &d, &e);

	local_irq_save(flags);

	if (!strcmp(name, "cnt_clr_all"))
		mvpp2_cls_c2_hit_cntr_clear_all(sysfs_cur_hw);
	else if (!strcmp(name, "cnt_read"))
		mvpp2_cls_c2_hit_cntr_read(sysfs_cur_hw, a, NULL);
	else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	local_irq_restore(flags);

	if (err)
		printk(KERN_ERR "%s: <%s>, error %d\n", __func__, attr->attr.name, err);

	return err ? -EINVAL : len;
}


static DEVICE_ATTR(prio_hw_dump,		S_IRUSR, mv_cls2_show, NULL);
static DEVICE_ATTR(dscp_hw_dump,		S_IRUSR, mv_cls2_show, NULL);
static DEVICE_ATTR(act_hw_dump,		S_IRUSR, mv_cls2_show, NULL);
static DEVICE_ATTR(cnt_dump,		S_IRUSR, mv_cls2_show, NULL);
static DEVICE_ATTR(hw_regs,		S_IRUSR, mv_cls2_show, NULL);
static DEVICE_ATTR(help,			S_IRUSR, mv_cls2_show, NULL);

static DEVICE_ATTR(cnt_clr_all,		S_IWUSR, mv_cls2_show, mv_cls2_store);
static DEVICE_ATTR(cnt_read,		S_IWUSR, mv_cls2_show, mv_cls2_store);


static struct attribute *cls2_attrs[] = {
	&dev_attr_prio_hw_dump.attr,
	&dev_attr_dscp_hw_dump.attr,
	&dev_attr_act_hw_dump.attr,
	&dev_attr_cnt_dump.attr,
	&dev_attr_hw_regs.attr,
	&dev_attr_help.attr,
	&dev_attr_cnt_clr_all.attr,
	&dev_attr_cnt_read.attr,
	NULL
};

static struct attribute_group cls2_group = {
	.name = "cls2",
	.attrs = cls2_attrs,
};

int mv_pp2_cls2_sysfs_init(struct kobject *pp2_kobj)
{
	int err = 0;

	err = sysfs_create_group(pp2_kobj, &cls2_group);
	if (err)
		printk(KERN_INFO "sysfs group %s failed %d\n", cls2_group.name, err);

	return err;
}

int mv_pp2_cls2_sysfs_exit(struct kobject *pp2_kobj)
{
	sysfs_remove_group(pp2_kobj, &cls2_group);
	return 0;
}

