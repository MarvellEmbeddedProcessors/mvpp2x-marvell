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

#ifndef _mv_ptp_if_h_
#define _mv_ptp_if_h_

#define PTP_TAI_PRT_STR	"TAI/PTP"

#include <mv_tai_regs.h>

struct mv_tai_ptp_map {
	phys_addr_t tai_base_pa;
	phys_addr_t tai_base_va;
	phys_addr_t ptp_base_pa;
	phys_addr_t ptp_base_va;
	u32 tai_size;
	u32 ptp_size;
	int num;
};

extern int mv_tai_clock_external_force_modparm;

int mv_ptp_tclk_hz_set(u32 tclk_hz); /* from dtb "clock-frequency" */
u32 mv_ptp_tclk_hz_get(void);
void mv_tai_clock_init(struct platform_device *pdev, int tai_clock_external);
bool mv_pp3_tai_clock_external_init(struct platform_device *pdev);
void mv_pp3_tai_clock_external_init2(bool from_external);
void mv_pp3_tai_set_nop(void);
void mv_pp3_tai_clock_cfg_external(bool from_external);
void mv_pp3_tai_clock_disable(void);
bool mv_pp3_tai_clock_enable_get(void);
void mv_pp3_tai_clock_stable_status_set(bool on);
bool mv_pp3_tai_clock_stable_status_get(void);
u16 mv_pp3_tai_clock_in_cntr_get(u32 *accumulated);

ssize_t mv_pp3_tai_clock_status_get_sysfs(char *buf);

int mv_ptp_enable(int port, bool state);
void mv_pp3_ptp_reset(int port);
void mv_pp3_ptp_reset_all_ptp_ports(void);

int mv_pp3_tai_tod_op(enum mv_pp3_tai_ptp_op op, struct mv_pp3_tai_tod *ts,
			int synced_op);
int mv_pp3_tai_tod_op_read_captured(struct mv_pp3_tai_tod *ts, u32 *status);

void mv_pp3_tai_clock_from_external_sync(int start, u32 sec, int d_sec);

void mv_pp3_ptp_reg_dump(int port);
void mv_pp3_tai_tod_dump_util(struct mv_pp3_tai_tod *ts);
void mv_pp3_tai_tod_from_linux(struct mv_pp3_tai_tod *ts);
void mv_pp3_tai_tod_to_linux(struct mv_pp3_tai_tod *ts);
void mv_pp3_tai_reg_dump(void);
void mv_pp3_tai_tod_dump(void);
int mv_pp3_tai_tod_load_set(u32 sec_h, u32 sec_l, u32 nano, u32 frac);

int mv_tai_ptp_map_init(struct mv_tai_ptp_map *map);
struct mv_tai_ptp_map *mv_tai_ptp_map_get(void);
void mv_tai_ptp_map_print(struct mv_tai_ptp_map *map, char *str);
int mv_tai_1pps_out_phase_update(int nsec);
u32 mv_ptp_egress_tx_ts_32bit_get(int port, int queue_num);

#if !defined ARMADA_390
int __init mv_ptp_event_init_module(void);
int __exit mv_ptp_event_deinit_module(void);
#endif
int mv_ptp_sysfs_init(char *dev_name, char *subdir);
void mv_pp2x_ptp_hook_init(void *priv, int port);

#endif /* _mv_ptp_if_h_ */
