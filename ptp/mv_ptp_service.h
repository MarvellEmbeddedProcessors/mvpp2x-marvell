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
#ifndef __mv_ptp_h__
#define __mv_ptp_h__

void mv_ptp_hook_enable(int port, bool enable);
void mv_ptp_hook_extra_op(u32 val1, u32 val2, u32 val3);

#ifdef __KERNEL__
/* Probre/Init should be called with/after mv_ptp_enable() */
int mv_ptp_tai_tod_uio_init(struct platform_device *shared_pdev);
#endif

#endif /* __mv_ptp_h__ */