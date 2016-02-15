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

#ifndef _MV_GOP_HW_H_
#define _MV_GOP_HW_H_

#if 0
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/errno.h>
#endif

#if 0
#include <linux/if_vlan.h>
#include <linux/platform_device.h>
#include <linux/dma-direction.h>
#include <linux/spinlock.h>
#include <asm-generic/dma-mapping-broken.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/netdevice.h>

#define __ATTRIBUTE_PACKED__	__packed
#define MV_MALLOC	kmalloc

#define	MV_MAC_ADDR_SIZE	(6)
#define MV_MAC_STR_SIZE		(20)
#define MV_MH_SIZE		(2)
#define MV_CRC_SIZE		(4)
#define MV_DSCP_NUM		(64)

#define MV_MTU_MIN		(68)
/* Layer2 packet info EtherType + Double VLAN + MAC_SA + MAC_DA +
 * Marvell header
 */
#define MV_L2_HLEN		(MV_MH_SIZE + 2 * VLAN_HLEN + ETH_HLEN)

/* MTU = MRU - MV_L2_SIZE */
#define MV_MTU_MAX		((10 * 1024) - MV_L2_HLEN)

#define MV_MAC_IS_EQUAL(_mac1, _mac2)			\
	(!memcmp((_mac1), (_mac2), MV_MAC_ADDR_SIZE))

/******************************************************
 * common functions				      *
 ******************************************************/
void mv_debug_mem_dump(void *addr, int size, int access);
unsigned int mv_field_get(int offs, int bits,  unsigned int *entry);
void mv_field_set(int offs, int bits, unsigned int *entry,  unsigned int val);


/******************************************************
 * align memory allocateion                           *
 ******************************************************/
/* Macro for testing aligment. Positive if number is NOT aligned   */
#define MV_IS_NOT_ALIGN(number, align)      ((number) & ((align) - 1))

/* Macro for alignment up. For example, MV_ALIGN_UP(0x0330, 0x20) = 0x0340   */
#define MV_ALIGN_UP(number, align)                             \
(((number) & ((align) - 1)) ? (((number) + (align)) & ~((align)-1)) : (number))

/* Macro for alignment down. For example, MV_ALIGN_UP(0x0330, 0x20) = 0x0320 */
#define MV_ALIGN_DOWN(number, align) ((number) & ~((align)-1))

/* Return mask of all ones for any number of bits less than 32 */
#define MV_ALL_ONES_MASK(bits)	((1 << (bits)) - 1)

/* Check that num is power of 2 */
#define MV_IS_POWER_OF_2(num) ((num != 0) && ((num & (num - 1)) == 0))


/* QM/BM related */
#define MV_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MV_MAX(a, b) (((a) > (b)) ? (a) : (b))

#define UNIT_OF__8_BYTES  8
#define UNIT_OF_64_BYTES 64

#define MV_32_BITS		32
#define MV_40_BITS		40
#define MV_WORD_BITS		32
#define MV_BYTE_BITS		8

/* Error definitions*/
/* Error Codes */
#endif

#if 0
#define OK                                  0
#define ON                                  1
#define OFF                                 0
#endif

/* Sets the field located at the specified in data.     */
#define U32_SET_FIELD(data, mask, val)	((data) = (((data) & ~(mask)) | (val)))


/* port related */
enum mv_reset {RESET, UNRESET};

#if 0
enum mv_port_mode {
	MV_PORT_RXAUI,
	MV_PORT_XAUI,
	MV_PORT_SGMII,
	MV_PORT_SGMII2_5,
	MV_PORT_QSGMII,
	MV_PORT_RGMII
};
#endif

enum mv_port_speed {
	MV_PORT_SPEED_AN,
	MV_PORT_SPEED_10,
	MV_PORT_SPEED_100,
	MV_PORT_SPEED_1000,
	MV_PORT_SPEED_2000,
	MV_PORT_SPEED_10000
};

enum mv_port_duplex {
	MV_PORT_DUPLEX_AN,
	MV_PORT_DUPLEX_HALF,
	MV_PORT_DUPLEX_FULL
};

enum mv_port_fc {
	MV_PORT_FC_AN_NO,
	MV_PORT_FC_AN_SYM,
	MV_PORT_FC_AN_ASYM,
	MV_PORT_FC_DISABLE,
	MV_PORT_FC_ENABLE,
	MV_PORT_FC_ACTIVE
};

struct mv_port_link_status {
	int			linkup; /*flag*/
	enum mv_port_speed	speed;
	enum mv_port_duplex	duplex;
	enum mv_port_fc		rx_fc;
	enum mv_port_fc		tx_fc;
};

/* different loopback types can be configure on different levels:
 * MAC, PCS, SERDES
 */
enum mv_lb_type {
	MV_DISABLE_LB,
	MV_RX_2_TX_LB,
	MV_TX_2_RX_LB,         /* on SERDES level - analog loopback */
	MV_TX_2_RX_DIGITAL_LB  /* on SERDES level - digital loopback */
};

enum sd_media_mode {MV_RXAUI, MV_XAUI};

#if 0
/* convert one char symbol to 8 bit interger hex format */
static inline unsigned char char_to_hex(char msg)
{
	unsigned char tmp = 0;

	if ((msg >= '0') && (msg <= '9'))
		tmp = msg - '0';
	else if ((msg >= 'a') && (msg <= 'f'))
		tmp = msg - 'a' + 10;
	else if ((msg >= 'A') && (msg <= 'F'))
		tmp = msg - 'A' + 10;

	return tmp;
}

/* convert asci string of known size to 8 bit interger hex format array */
static inline void str_to_hex(char *msg, int size, unsigned char *imsg,
	int new_size)
{
	int i, j;
	unsigned char tmp;

	for (i = 0, j = 0; j < new_size; i = i + 2, j++) {
		/* build high byte nible */
		tmp = (char_to_hex(msg[i]) << 4);
		/* build low byte nible */
		tmp += char_to_hex(msg[i+1]);
		imsg[j] = tmp;
	}
}


/* convert mac address in format xx:xx:xx:xx:xx:xx to array of
* unsigned char [6]
*/
static inline void mv_mac_str2hex(const char *mac_str, u8 *mac_hex)
{
	int i;
	char tmp[3];
	u8 tmp1;

	for (i = 0; i < 6; i++) {
		tmp[0] = mac_str[(i * 3) + 0];
		tmp[1] = mac_str[(i * 3) + 1];
		tmp[2] = '\0';
		str_to_hex(tmp, 3, &tmp1, 1);
		mac_hex[i] = tmp1;
	}
}
#endif

/* pp3_gop_ctrl flags */
#define mv_gop_F_DEBUG_BIT		0
#define mv_gop_F_ATTACH_BIT		1

#define mv_gop_F_DEBUG		(1 << mv_gop_F_DEBUG_BIT)
#define mv_gop_F_ATTACH		(1 << mv_gop_F_ATTACH_BIT)

enum gop_port_flags {NOT_CREATED, CREATED, UNDER_RESET, ENABLED};

struct gop_port_ctrl {
	u32  flags;
};

#define MV_RGMII_TX_FIFO_MIN_TH		(0x41)
#define MV_SGMII_TX_FIFO_MIN_TH		(0x5)
#define MV_SGMII2_5_TX_FIFO_MIN_TH	(0xB)

static inline u32 mv_gop_gen_read(void __iomem *base, u32 offset)
{
	void *reg_ptr = base + offset;
	u32 val;

	val = readl(reg_ptr);
	return val;
}

static inline void mv_gop_gen_write(void __iomem *base, u32 offset, u32 data)
{
	void *reg_ptr = base + offset;

	writel(data, reg_ptr);
}


/* GOP port configuration functions */
int mv_gop110_port_init(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_port_reset(struct gop_hw *gop, struct mv_mac_data *mac);
void mv_gop110_port_enable(struct gop_hw *gop, struct mv_mac_data *mac);
void mv_gop110_port_disable(struct gop_hw *gop, struct mv_mac_data *mac);
void mv_gop110_port_periodic_xon_set(struct gop_hw *gop,
				     struct mv_mac_data *mac,
				     int enable);
bool mv_gop110_port_is_link_up(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_port_link_status(struct gop_hw *gop, struct mv_mac_data *mac,
			       struct mv_port_link_status *pstatus);
int mv_gop110_port_regs(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_port_events_mask(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_port_events_unmask(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_port_events_clear(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_status_show(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_speed_duplex_get(struct gop_hw *gop, struct mv_mac_data *mac,
			       enum mv_port_speed *speed,
			       enum mv_port_duplex *duplex);
int mv_gop110_speed_duplex_set(struct gop_hw *gop, struct mv_mac_data *mac,
			       enum mv_port_speed speed,
			       enum mv_port_duplex duplex);
int mv_gop110_autoneg_restart(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_fl_cfg(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_force_link_mode_set(struct gop_hw *gop, struct mv_mac_data *mac,
				  bool force_link_up,
				  bool force_link_down);
int mv_gop110_force_link_mode_get(struct gop_hw *gop, struct mv_mac_data *mac,
				  bool *force_link_up,
				  bool *force_link_down);
int mv_gop110_loopback_set(struct gop_hw *gop, struct mv_mac_data *mac,
			   bool lb);
void mv_gop_reg_print(char *reg_name, u32 reg);

/* Gig PCS Functions */
int mv_gop110_gpcs_mode_cfg(struct gop_hw *gop, int pcs_num, bool en);
int mv_gop110_gpcs_reset(struct gop_hw *gop, int pcs_num, enum mv_reset act);

/* Serdes Functions */
static inline u32 mv_gop110_serdes_read(struct gop_hw *gop, int lane_num,
					u32 offset)
{
	return(mv_gop_gen_read(gop->gop_110.serdes.base,
		lane_num * gop->gop_110.serdes.obj_size + offset));
}
static inline void mv_gop110_serdes_write(struct gop_hw *gop, int lane_num,
					  u32 offset, u32 data)
{
	mv_gop_gen_write(gop->gop_110.serdes.base,
		lane_num * gop->gop_110.serdes.obj_size + offset, data);
}

static inline void mv_gop110_serdes_print(struct gop_hw *gop, char *reg_name,
					  int lane_num, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_serdes_read(gop, lane_num, reg));
}

void mv_gop110_serdes_lane_regs_dump(struct gop_hw *gop, int lane);
void mv_gop110_serdes_init(struct gop_hw *gop, int lane,
			   enum sd_media_mode mode);
void mv_gop110_serdes_reset(struct gop_hw *gop, int lane, bool analog_reset,
			    bool core_reset, bool digital_reset);

/* XPCS Functions */

static inline u32 mv_gop110_xpcs_global_read(struct gop_hw *gop, u32 offset)
{
	return mv_gop_gen_read(gop->gop_110.xpcs_base, offset);
}
static inline void mv_gop110_xpcs_global_write(struct gop_hw *gop, u32 offset,
					       u32 data)
{
	mv_gop_gen_write(gop->gop_110.xpcs_base, offset, data);
}
static inline void mv_gop110_xpcs_global_print(struct gop_hw *gop,
					       char *reg_name, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_xpcs_global_read(gop, reg));
}

static inline u32 mv_gop110_xpcs_lane_read(struct gop_hw *gop, int lane_num,
					   u32 offset)
{
	return mv_gop_gen_read(gop->gop_110.xpcs_base, offset);
}
static inline void mv_gop110_xpcs_lane_write(struct gop_hw *gop, int lane_num,
					     u32 offset, u32 data)
{
	mv_gop_gen_write(gop->gop_110.xpcs_base, offset, data);
}
static inline void mv_gop110_xpcs_lane_print(struct gop_hw *gop,
					     char *reg_name,
					     int lane_num, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_xpcs_lane_read(gop, lane_num, reg));
}

void mv_gop110_xpcs_gl_regs_dump(struct gop_hw *gop);
void mv_gop110_xpcs_lane_regs_dump(struct gop_hw *gop, int lane);
int mv_gop110_xpcs_reset(struct gop_hw *gop, enum mv_reset reset);
int mv_gop110_xpcs_mode(struct gop_hw *gop, int num_of_lanes);

/* XLG MAC Functions */
static inline u32 mv_gop110_xlg_mac_read(struct gop_hw *gop, int mac_num,
					 u32 offset)
{
	return(mv_gop_gen_read(gop->gop_110.xlg_mac.base,
		mac_num * gop->gop_110.xlg_mac.obj_size + offset));
}
static inline void mv_gop110_xlg_mac_write(struct gop_hw *gop, int mac_num,
					   u32 offset, u32 data)
{
	mv_gop_gen_write(gop->gop_110.xlg_mac.base,
		mac_num * gop->gop_110.xlg_mac.obj_size + offset, data);
}
static inline void mv_gop110_xlg_mac_print(struct gop_hw *gop, char *reg_name,
					   int mac_num, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_xlg_mac_read(gop, mac_num, reg));
}

/* MIB MAC Functions */
static inline u32 mv_gop110_xmib_mac_read(struct gop_hw *gop, int mac_num,
					  u32 offset)
{
	return(mv_gop_gen_read(gop->gop_110.xmib.base,
		mac_num * gop->gop_110.xmib.obj_size + offset));
}
static inline void mv_gop110_xmib_mac_write(struct gop_hw *gop, int mac_num,
					    u32 offset, u32 data)
{
	mv_gop_gen_write(gop->gop_110.xmib.base,
		mac_num * gop->gop_110.xmib.obj_size + offset, data);
}
static inline void mv_gop110_xmib_mac_print(struct gop_hw *gop, char *reg_name,
					    int mac_num, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_xmib_mac_read(gop, mac_num, reg));
}

void mv_gop110_xlg_mac_regs_dump(struct gop_hw *gop, int port);
int mv_gop110_xlg_mac_reset(struct gop_hw *gop, int mac_num,
			    enum mv_reset reset);
int mv_gop110_xlg_mac_mode_cfg(struct gop_hw *gop, int mac_num,
			       int num_of_act_lanes);
int mv_gop110_xlg_mac_loopback_cfg(struct gop_hw *gop, int mac_num,
				   enum mv_lb_type type);

bool mv_gop110_xlg_mac_link_status_get(struct gop_hw *gop, int mac_num);
void mv_gop110_xlg_mac_port_enable(struct gop_hw *gop, int mac_num);
void mv_gop110_xlg_mac_port_disable(struct gop_hw *gop, int mac_num);
void mv_gop110_xlg_mac_port_periodic_xon_set(struct gop_hw *gop,
					     int mac_num,
					     int enable);
int mv_gop110_xlg_mac_link_status(struct gop_hw *gop, int mac_num,
				  struct mv_port_link_status *pstatus);
int mv_gop110_xlg_mac_max_rx_size_set(struct gop_hw *gop, int mac_num,
				      int max_rx_size);
int mv_gop110_xlg_mac_force_link_mode_set(struct gop_hw *gop, int mac_num,
					  bool force_link_up,
					  bool force_link_down);
int mv_gop110_xlg_mac_speed_duplex_set(struct gop_hw *gop, int mac_num,
				       enum mv_port_speed speed,
				       enum mv_port_duplex duplex);
int mv_gop110_xlg_mac_speed_duplex_get(struct gop_hw *gop, int mac_num,
				       enum mv_port_speed *speed,
				       enum mv_port_duplex *duplex);
int mv_gop110_xlg_mac_fc_set(struct gop_hw *gop, int mac_num,
			     enum mv_port_fc fc);
void mv_gop110_xlg_mac_fc_get(struct gop_hw *gop, int mac_num,
			      enum mv_port_fc *fc);
int mv_gop110_xlg_mac_port_link_speed_fc(struct gop_hw *gop, int mac_num,
					 enum mv_port_speed speed,
					 int force_link_up);
void mv_gop110_xlg_port_link_event_mask(struct gop_hw *gop, int mac_num);
void mv_gop110_xlg_port_external_event_unmask(struct gop_hw *gop,
					      int mac_num,
					      int bit_2_open);
void mv_gop110_xlg_port_link_event_clear(struct gop_hw *gop, int mac_num);
void mv_gop110_xlg_2_gig_mac_cfg(struct gop_hw *gop, int mac_num);

/* GMAC Functions  */
static inline u32 mv_gop110_gmac_read(struct gop_hw *gop, int mac_num,
				      u32 offset)
{
	return(mv_gop_gen_read(gop->gop_110.gmac.base,
		mac_num * gop->gop_110.gmac.obj_size + offset));
}
static inline void mv_gop110_gmac_write(struct gop_hw *gop, int mac_num,
					u32 offset, u32 data)
{
	mv_gop_gen_write(gop->gop_110.gmac.base,
		mac_num * gop->gop_110.gmac.obj_size + offset, data);
}
static inline void mv_gop110_gmac_print(struct gop_hw *gop, char *reg_name,
					int mac_num, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_gmac_read(gop, mac_num, reg));
}

void mv_gop110_register_bases_dump(struct gop_hw *gop);
void mv_gop110_gmac_regs_dump(struct gop_hw *gop, int port);
int mv_gop110_gmac_reset(struct gop_hw *gop, int mac_num,
			 enum mv_reset reset);
int mv_gop110_gmac_mode_cfg(struct gop_hw *gop, struct mv_mac_data *mac);
int mv_gop110_gmac_loopback_cfg(struct gop_hw *gop, int mac_num,
				enum mv_lb_type type);
bool mv_gop110_gmac_link_status_get(struct gop_hw *gop, int mac_num);
void mv_gop110_gmac_port_enable(struct gop_hw *gop, int mac_num);
void mv_gop110_gmac_port_disable(struct gop_hw *gop, int mac_num);
void mv_gop110_gmac_port_periodic_xon_set(struct gop_hw *gop, int mac_num,
					  int enable);
int mv_gop110_gmac_link_status(struct gop_hw *gop, int mac_num,
			       struct mv_port_link_status *pstatus);
int mv_gop110_gmac_max_rx_size_set(struct gop_hw *gop, int mac_num,
				   int max_rx_size);
int mv_gop110_gmac_force_link_mode_set(struct gop_hw *gop, int mac_num,
				       bool force_link_up,
				       bool force_link_down);
int mv_gop110_gmac_force_link_mode_get(struct gop_hw *gop, int mac_num,
				       bool *force_link_up,
				       bool *force_link_down);
int mv_gop110_gmac_speed_duplex_set(struct gop_hw *gop, int mac_num,
				    enum mv_port_speed speed,
				    enum mv_port_duplex duplex);
int mv_gop110_gmac_speed_duplex_get(struct gop_hw *gop, int mac_num,
				    enum mv_port_speed *speed,
				    enum mv_port_duplex *duplex);
int mv_gop110_gmac_fc_set(struct gop_hw *gop, int mac_num,
			  enum mv_port_fc fc);
void mv_gop110_gmac_fc_get(struct gop_hw *gop, int mac_num,
			   enum mv_port_fc *fc);
int mv_gop110_gmac_port_link_speed_fc(struct gop_hw *gop, int mac_num,
				      enum mv_port_speed speed,
				      int force_link_up);
void mv_gop110_gmac_port_link_event_mask(struct gop_hw *gop, int mac_num);
void mv_gop110_gmac_port_link_event_unmask(struct gop_hw *gop, int mac_num);
void mv_gop110_gmac_port_link_event_clear(struct gop_hw *gop, int mac_num);
int mv_gop110_gmac_port_autoneg_restart(struct gop_hw *gop, int mac_num);

/* SMI Functions  */
static inline u32 mv_gop110_smi_read(struct gop_hw *gop, u32 offset)
{
	return mv_gop_gen_read(gop->gop_110.smi_base, offset);
}
static inline void mv_gop110_smi_write(struct gop_hw *gop, u32 offset,
				       u32 data)
{
	mv_gop_gen_write(gop->gop_110.smi_base, offset, data);
}
static inline void mv_gop110_smi_print(struct gop_hw *gop, char *reg_name,
				       u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_smi_read(gop, reg));
}
/* PTP Functions  */
static inline u32 mv_gop110_ptp_read(struct gop_hw *gop, int mac_num,
				     u32 offset)
{
	return mv_gop_gen_read(gop->gop_110.ptp.base,
		mac_num * gop->gop_110.ptp.obj_size + offset);
}
static inline void mv_gop110_ptp_write(struct gop_hw *gop, int mac_num,
				       u32 offset, u32 data)
{
	mv_gop_gen_write(gop->gop_110.ptp.base,
		mac_num * gop->gop_110.ptp.obj_size + offset, data);
}
static inline void mv_gop110_ptp_print(struct gop_hw *gop, char *reg_name,
				       int mac_num, u32 reg)
{
	pr_info("  %-32s: 0x%x = 0x%08x\n", reg_name, reg,
		mv_gop110_ptp_read(gop, mac_num, reg));
}

int mv_gop110_smi_init(struct gop_hw *gop);
int mv_gop110_smi_phy_addr_cfg(struct gop_hw *gop, int port, int addr);

/* MIB Functions  */
u64 mv_gop110_mib_read64(struct gop_hw *gop, int port, unsigned int offset);
void mv_gop110_mib_counters_show(struct gop_hw *gop, int port);

/* PTP Functions */
void mv_gop110_ptp_enable(struct gop_hw *gop, int port, bool state);

#endif /* _MV_GOP_HW_H_ */
