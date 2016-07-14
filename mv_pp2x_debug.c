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
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>

#include "mv_pp2x.h"
#include "mv_pp2x_hw.h"
#include "mv_pp2x_debug.h"

int mv_pp2x_debug_param_set(u32 param)
{
	debug_param = param;
	return 0;
}
EXPORT_SYMBOL(mv_pp2x_debug_param_set);

int mv_pp2x_debug_param_get(void)
{
	return debug_param;
}
EXPORT_SYMBOL(mv_pp2x_debug_param_get);

/* Extra debug */
void mv_pp2x_skb_dump(struct sk_buff *skb, int size, int access)
{
	int i, j;
	void *addr = skb->head + NET_SKB_PAD;
	uintptr_t memAddr = (uintptr_t)addr;

	printk("skb=%p, buf=%p, ksize=%d\n", skb, skb->head,
			(int)ksize(skb->head));

	if (access == 0)
		access = 1;

	if ((access != 4) && (access != 2) && (access != 1)) {
		printk("%d wrong access size. Access must be 1 or 2 or 4\n",
				access);
		return;
	}
	memAddr = MV_ALIGN_DOWN((uintptr_t)addr, 4);
	size = MV_ALIGN_UP(size, 4);
	addr = (void *)MV_ALIGN_DOWN((uintptr_t)addr, access);
	while (size > 0) {
		printk("%08lx: ", memAddr);
		i = 0;
		/* 32 bytes in the line */
		while (i < 32) {
			if (memAddr >= (uintptr_t)addr) {
				switch (access) {
				case 1:
					printk("%02x ",
						MV_MEMIO8_READ(memAddr));
					break;

				case 2:
					printk("%04x ",
						MV_MEMIO16_READ(memAddr));
					break;

				case 4:
					printk("%08x ",
						MV_MEMIO32_READ(memAddr));
					break;
				}
			} else {
				for (j = 0; j < (access * 2 + 1); j++)
					printk(" ");
			}
			i += access;
			memAddr += access;
			size -= access;
			if (size <= 0)
				break;
		}
		printk("\n");
	}
}



