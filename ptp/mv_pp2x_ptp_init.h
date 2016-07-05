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

#ifndef _mv_pp2x_ptp_init_h_
#define _mv_pp2x_ptp_init_h_

#include "mv_pp2x.h"

#include <mv_tai_regs.h>
#include <mv_ptp_if.h>
#include <mv_ptp_service.h>

/* Called from mv_pp2x_probe() */
int mv_pp2x_ptp_init(struct platform_device *pdev,
	struct mv_pp2x_port *p_port, int port_count);
#endif /* _mv_pp2x_ptp_init_h_ */
