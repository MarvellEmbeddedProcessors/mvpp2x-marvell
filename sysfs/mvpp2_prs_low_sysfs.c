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


static  struct mvpp2_prs_entry pe;


static ssize_t mv_prs_low_help(char *buf)
{
	int off = 0;

	off += scnprintf(buf + off, PAGE_SIZE - off, "cat          hw_dump       - dump all valid HW entries\n");
	off += scnprintf(buf + off, PAGE_SIZE - off, "cat          hw_regs       - dump parser registers.\n");
	off += scnprintf(buf + off, PAGE_SIZE - off, "cat          hw_hits       - dump non zeroed hit counters and the associated HW entries\n");
	off += scnprintf(buf + off, PAGE_SIZE - off, "\n");


	return off;
}

static ssize_t mv_prs_low_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	if (!strcmp(name, "hw_dump"))
		mvpp2_prs_hw_dump(sysfs_cur_hw);
	else if (!strcmp(name, "hw_regs"))
		mvpp2_prs_hw_regs_dump(sysfs_cur_hw);
	else if (!strcmp(name, "hw_hits"))
		mvpp2_prs_hw_hits_dump(sysfs_cur_hw);
	else
		off += mv_prs_low_help(buf);

	return off;
}


static ssize_t mv_prs_low_store_unsigned(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t len)
{
	const char    *name = attr->attr.name;
	unsigned int  err = 0, a = 0, b = 0, c = 0;
	unsigned long flags;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	sscanf(buf, "%x %x %x", &a, &b, &c);

	local_irq_save(flags);

	if (!strcmp(name, "hw_read")) {
		pe.index = a;
		mvpp2_prs_hw_read(sysfs_cur_hw, &pe);
	}
	else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	local_irq_restore(flags);

	if (err)
		printk(KERN_ERR "%s: <%s>, error %d\n", __func__, attr->attr.name, err);

	return err ? -EINVAL : len;
}


static DEVICE_ATTR(hw_dump,		S_IRUSR, mv_prs_low_show, NULL);
static DEVICE_ATTR(help,			S_IRUSR, mv_prs_low_show, NULL);
static DEVICE_ATTR(hw_regs,		S_IRUSR, mv_prs_low_show, NULL);
static DEVICE_ATTR(hw_hits,		S_IRUSR, mv_prs_low_show, NULL);
static DEVICE_ATTR(hw_read,		S_IWUSR, mv_prs_low_show, mv_prs_low_store_unsigned);



static struct attribute *prs_low_attrs[] = {
	&dev_attr_hw_dump.attr,
	&dev_attr_hw_hits.attr,
	&dev_attr_hw_regs.attr,
	&dev_attr_hw_read.attr,
	&dev_attr_help.attr,
	NULL
};

static struct attribute_group prs_low_group = {
	.name = "debug",
	.attrs = prs_low_attrs,
};

int mv_pp2_prs_low_sysfs_init(struct kobject *pp2_kobj)
{
	int err;

	err = sysfs_create_group(pp2_kobj, &prs_low_group);
	if (err)
		pr_err("sysfs group %s failed %d\n", prs_low_group.name, err);

	return err;
}

int mv_pp2_prs_low_sysfs_exit(struct kobject *pp2_kobj)
{
	sysfs_remove_group(pp2_kobj, &prs_low_group);

	return 0;
}

