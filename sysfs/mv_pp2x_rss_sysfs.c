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
#include "mv_pp2x_sysfs.h"


static ssize_t mv_rss_help(char *buf)
{
	int off = 0;
	off += scnprintf(buf + off, PAGE_SIZE,  "cat                         rss_hw_dump  - dump rxq in rss table entry from hardware.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo [if_name] [mode]    >  rss_mode     - Set the hash mode for non-frag UDP packet\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "     [mode]      - 0 - 2-Tuple\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "                 - 1 - 5-Tuple\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "echo [if_name] [cpu]     >  rss_dflt_cpu - Set cpu to handle the non-IP packet\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "\n");

	return off;
}


static ssize_t mv_rss_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "rss_hw_dump"))
		mv_pp22_rss_hw_dump(sysfs_cur_hw);
	else
		off += mv_rss_help(buf);

	return off;
}

static ssize_t mv_rss_store(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t len)
{
	const char      *name = attr->attr.name;
	int             err;
	unsigned int    b;
	char		if_name[10];
	struct net_device *netdev;
	struct mv_pp2x_port *port;

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

	if (!strcmp(name, "rss_mode")) {
		mv_pp22_wrap_rss_mode_set(port, b);
	} else if (!strcmp(name, "rss_dflt_cpu")) {
		mv_pp22_wrap_rss_dflt_cpu_set(port, b);
	} else {
		err = 1;
		printk("%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

	if (err)
		printk("%s: error %d\n", __func__, err);

	return err ? -EINVAL : len;
}



static DEVICE_ATTR(rss_hw_dump,		S_IRUSR, mv_rss_show, NULL);
static DEVICE_ATTR(help,		S_IRUSR, mv_rss_show, NULL);
static DEVICE_ATTR(rss_mode,		S_IWUSR, NULL, mv_rss_store);
static DEVICE_ATTR(rss_dflt_cpu,	S_IWUSR, NULL, mv_rss_store);

static struct attribute *rss_attrs[] = {
	&dev_attr_rss_hw_dump.attr,
	&dev_attr_help.attr,
	&dev_attr_rss_mode.attr,
	&dev_attr_rss_dflt_cpu.attr,
	NULL
};

static struct attribute_group rss_group = {
	.name = "rss",
	.attrs = rss_attrs,
};

int mv_pp2_rss_sysfs_init(struct kobject *pp2_kobj)
{
	int err = 0;

	err = sysfs_create_group(pp2_kobj, &rss_group);
	if (err)
		printk("sysfs group %s failed %d\n", rss_group.name, err);

	return err;
}

int mv_pp2_rss_sysfs_exit(struct kobject *pp2_kobj)
{
	sysfs_remove_group(pp2_kobj, &rss_group);

	return 0;
}

