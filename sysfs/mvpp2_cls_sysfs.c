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

static struct mvpp2_cls_lookup_entry lkp_entry;
static struct mvpp2_cls_flow_entry flow_entry;


static ssize_t mv_cls_help(char *buf)
{
	int off = 0;
	off += scnprintf(buf + off, PAGE_SIZE,  "cat             lkp_hw_dump          - dump lookup ID tabel from hardware.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "cat             flow_hw_hits         - dump non zeroed hit counters  and the associated flow tabel entries from hardware.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "cat             lkp_hw_hits          - dump non zeroed hit counters and the associated lookup ID entires from hardware.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "cat             flow_hw_dump         - dump flow table from hardware.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "cat             hw_regs              - dump classifier top registers.\n");

	off += scnprintf(buf + off, PAGE_SIZE,  "\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo idx way    > lkp_hw_read        - read lookup ID table entry from HW <idx,way>.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo id         > flow_hw_read       - read flow table entry <id> from HW.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo offset val > mvpp2_reg_write    - Write mvpp2 register.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo offset     > mvpp2_reg_read     - Read mvpp2 register.\n");

	off += scnprintf(buf + off, PAGE_SIZE,  "\n");

	return off;
}


static ssize_t mv_cls_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "lkp_hw_hits"))
		mvpp2_cls_hw_lkp_hits_dump(sysfs_cur_hw);
	else if (!strcmp(name, "flow_hw_hits"))
		mvpp2_cls_hw_flow_hits_dump(sysfs_cur_hw);
	else if (!strcmp(name, "lkp_hw_dump"))
		mvpp2_cls_hw_lkp_dump(sysfs_cur_hw);
	else if (!strcmp(name, "flow_hw_dump"))
		mvpp2_cls_hw_flow_dump(sysfs_cur_hw);
	else if (!strcmp(name, "hw_regs"))
		mvpp2_cls_hw_regs_dump(sysfs_cur_hw);
	else
		off += mv_cls_help(buf);

	return off;
}



static ssize_t mv_cls_store_unsigned(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t len)
{
	const char    *name = attr->attr.name;
	unsigned int  err = 0, a = 0, b = 0, c = 0, d = 0;
	unsigned long flags;
	u32 val;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	sscanf(buf, "%x %x %x %x", &a, &b, &c, &d);

	local_irq_save(flags);

	if (!strcmp(name, "lkp_hw_read"))
		mvpp2_cls_hw_lkp_read(sysfs_cur_hw, a, b, &lkp_entry);
	else if (!strcmp(name, "flow_hw_read"))
		mvpp2_cls_hw_flow_read(sysfs_cur_hw, a, &flow_entry);
	else if (!strcmp(name, "mvpp2_reg_read")) {
		val = mvpp2_read(sysfs_cur_hw, a);
		printk("mvpp2_read(0x%x)=0x%x\n", a, val);
	}
	else if (!strcmp(name, "mvpp2_reg_write")) {
		mvpp2_write(sysfs_cur_hw, a, b);
		val = mvpp2_read(sysfs_cur_hw, a);
		printk("mvpp2_write_read(0x%x)=0x%x\n", a, val);
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



static DEVICE_ATTR(lkp_hw_dump,		S_IRUSR, mv_cls_show, NULL);
static DEVICE_ATTR(lkp_hw_hits,		S_IRUSR, mv_cls_show, NULL);
static DEVICE_ATTR(flow_hw_hits,		S_IRUSR, mv_cls_show, NULL);
static DEVICE_ATTR(flow_hw_dump,		S_IRUSR, mv_cls_show, NULL);
static DEVICE_ATTR(help,			S_IRUSR, mv_cls_show, NULL);
static DEVICE_ATTR(hw_regs,		S_IRUSR, mv_cls_show, NULL);
static DEVICE_ATTR(lkp_hw_read,		S_IWUSR, mv_cls_show, mv_cls_store_unsigned);
static DEVICE_ATTR(flow_hw_read,		S_IWUSR, mv_cls_show, mv_cls_store_unsigned);
static DEVICE_ATTR(mvpp2_reg_read,	S_IWUSR, mv_cls_show, mv_cls_store_unsigned);
static DEVICE_ATTR(mvpp2_reg_write,	S_IWUSR, mv_cls_show, mv_cls_store_unsigned);





static struct attribute *cls_attrs[] = {
	&dev_attr_lkp_hw_hits.attr,
	&dev_attr_flow_hw_hits.attr,
	&dev_attr_lkp_hw_dump.attr,
	&dev_attr_flow_hw_dump.attr,
	&dev_attr_hw_regs.attr,
	&dev_attr_lkp_hw_read.attr,
	&dev_attr_flow_hw_read.attr,
	&dev_attr_mvpp2_reg_read.attr,
	&dev_attr_mvpp2_reg_write.attr,
	&dev_attr_help.attr,
	NULL
};

static struct attribute_group cls_group = {
	.name = "cls",
	.attrs = cls_attrs,
};

int mv_pp2_cls_sysfs_init(struct kobject *pp2_kobj)
{
	int err = 0;

	err = sysfs_create_group(pp2_kobj, &cls_group);
	if (err)
		printk(KERN_INFO "sysfs group %s failed %d\n", cls_group.name, err);

	return err;
}

int mv_pp2_cls_sysfs_exit(struct kobject *pp2_kobj)
{
	sysfs_remove_group(pp2_kobj, &cls_group);

	return 0;
}
