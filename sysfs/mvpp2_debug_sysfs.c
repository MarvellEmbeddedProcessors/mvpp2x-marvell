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


static ssize_t mv_debug_help(char *buf)
{
	int off = 0;
	off += scnprintf(buf + off, PAGE_SIZE,  "echo [if_name] [cpu]     >  bind_cpu - Bind the interface to dedicated CPU\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "     NOTE: bind_cpu only valid when rss is disabled\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo offset val >mvpp2_reg_write    - Write mvpp2 register.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo offset     >mvpp2_reg_read     - Read mvpp2 register.\n");

	return off;
}


static ssize_t mv_debug_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	off += mv_debug_help(buf);

	return off;
}

static ssize_t mv_debug_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t len)
{
	const char      *name = attr->attr.name;
	int             err;
	unsigned int    b;
	char		if_name[10];
	struct net_device *netdev;
	struct mvpp2_port *port;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	/* Read port and value */
	err = b = 0;

	sscanf(buf, "%s %x", if_name, &b);

	netdev = dev_get_by_name(&init_net, if_name);
	if (!netdev) {
		printk("%s: illegal interface <%s>\n", __func__, if_name);
		return -EINVAL;
	}
	port = netdev_priv(netdev);

	if (!strcmp(name, "bind_cpu")) {
		mvpp2_port_bind_cpu_set(port, b);
	}
	else if (!strcmp(name, "debug_param")) {
		mvpp2_debug_param_set(b);
	}
	else {
		err = 1;
		printk("%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

	if (err)
		printk("%s: error %d\n", __func__, err);

	return err ? -EINVAL : len;
}

static ssize_t mv_debug_store_unsigned(struct device *dev,
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

	if (!strcmp(name, "mvpp2_reg_read")) {
		val = mvpp2_read(sysfs_cur_hw, a);
		printk("mvpp2_read(0x%x)=0x%x\n", a, val);
	} else if (!strcmp(name, "mvpp2_reg_write")) {
		mvpp2_write(sysfs_cur_hw, a, b);
		val = mvpp2_read(sysfs_cur_hw, a);
		printk("mvpp2_write_read(0x%x)=0x%x\n", a, val);
	} else {
		err = 1;
		printk(KERN_ERR "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}
	local_irq_restore(flags);

	if (err)
		printk(KERN_ERR "%s: <%s>, error %d\n", __func__, attr->attr.name, err);

	return err ? -EINVAL : len;
}

static DEVICE_ATTR(help,	S_IRUSR, mv_debug_show, NULL);
static DEVICE_ATTR(bind_cpu,	S_IWUSR, NULL, mv_debug_store);
static DEVICE_ATTR(debug_param,	S_IWUSR, NULL, mv_debug_store);
static DEVICE_ATTR(mvpp2_reg_read,	S_IWUSR, NULL, mv_debug_store_unsigned);
static DEVICE_ATTR(mvpp2_reg_write,	S_IWUSR, NULL, mv_debug_store_unsigned);

static struct attribute *debug_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_bind_cpu.attr,
	&dev_attr_debug_param.attr,
	&dev_attr_mvpp2_reg_read.attr,
	&dev_attr_mvpp2_reg_write.attr,
	NULL
};

static struct attribute_group debug_group = {
	.name = "debug",
	.attrs = debug_attrs,
};

int mv_pp2_debug_sysfs_init(struct kobject *pp2_kobj)
{
	int err = 0;

	err = sysfs_create_group(pp2_kobj, &debug_group);
	if (err)
		printk("sysfs group %s failed %d\n", debug_group.name, err);

	return err;
}

int mv_pp2_debug_sysfs_exit(struct kobject *pp2_kobj)
{
	sysfs_remove_group(pp2_kobj, &debug_group);

	return 0;
}

