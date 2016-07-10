/****************************************************************************
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
#include <linux/inetdevice.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <linux/list.h>
#include <linux/of_mdio.h>
#include <linux/if_vlan.h>

#include <mv_pp2x.h>
#include <mv_ptp_regs.h>
#include <mv_ptp_service.h>


/* ============  CFH-interface macros  ========================
* MVPP2 PTP info in TX-descriptor has 2 parts:
*    OFFS_0x08[11:0] contains tag1[11:0]
*    OFFS_0x14[31:8] contains tag2[35:12]
* PTP_descriptor [35:0] bits are:
*  p32[2]:[3:0]   [3:0]   Action                        /PACT
*  p32[2]:[6:4]   [6:4]   PacketFormat                  /PF
*  p32[2]:[7]     [7]     CF_WraparoundCheckEn          /WC
*  p32[2]:[9:8]   [9:8]   IngressTimestampSeconds[1:0]
*  p32[2]:[10]    [10]    Reserved
*  p32[2]:[11]    [11]    MAC TimeStamping Enable       /TSE
*		 :
*  p32[5]:[13:8]  [17:12] TimestampQueueEntryID[5:0]
*  p32[5]:[14]    [18]    TimestampQueueSelect          /QS
*  p32[5]:[15]    [19]    UDP ChecksumEn                /CUE
*  p32[5]:[23:16] [27:20] Timestamp Offset[7:0]         /TS_off
*  p32[5]:[31:24] [35:28] UDP Checksum Offset[7:0]      /CS_off
*
* Legacy: TSE, QS, TS_off, CS_off, CUE, PACT, PF, WC, DE, ETS, SEC
*/
#define MVPP2_TXD_PTP_TSE_SET(p32)	{ p32[2] |= BIT(11); }
#define MVPP2_TXD_PTP_CUE_SET(p32)	{ p32[5] |= BIT(15); }
#define MVPP2_TXD_PTP_PACT_SET(p32, pact)	{ p32[2] |= pact; }
#define MVPP2_TXD_PTP_TS_OFF_SET(p32, ts)	{ p32[5] |= (ts << 16); }
#define MVPP2_TXD_PTP_CS_OFF_SET(p32, cs)	{ p32[5] |= (cs << 24); }

#define MVPP2_TXD_PTP_LEN_GET(p32, len)	{ len = p32[1] >> 16; }
#define MVPP2_TXD_PTP_LEN_SET(p32, len)	{ p32[1] |= ((len) << 16); }

#define MVPP2_TXD_PTP_CLEAR_EXT(p32)	{ p32[2] = 0; p32[5] = 0; }
#define MVPP2_TXD_PTP_SET(txd, p32)	\
	do { \
		u32 *txd32 = (u32 *)txd; \
		txd32[1] = (txd32[1] & 0x0000ffff) | p32[1]; \
		txd32[2] = (txd32[2] & 0xfffff000) | p32[2]; \
		txd32[5] = (txd32[5] & 0x000000ff) | p32[5]; \
	} while (0)


/* GENERAL packet-info in RX-descriptor (parsing) */
#define MV_RXD_VLAN_INFO_GET(d) ((d->rsrvd_parser & MVPP2_RXD_VLAN_INFO_MASK) >> MVPP2_RXD_VLAN_INFO_OFFS)
#define MV_RXD_IPHDR_LEN_GET(d) (((d->status & MVPP2_RXD_IP_HLEN_MASK) >> MVPP2_RXD_IP_HLEN_OFFS)*sizeof(u32))
#define MV_RXD_L3_OFFS_GET(d) ((d->status & MVPP2_RXD_L3_OFFSET_MASK) >> MVPP2_RXD_L3_OFFSET_OFFS)
#define MV_RXD_L3_IS_IP4(d)	(d->status & MVPP2_RXD_L3_IP4)
#define MV_RXD_L3_IS_IP6(d)	(d->status & MVPP2_RXD_L3_IP6)
#define MV_RXD_L4_IS_UDP(d)	(d->status & MVPP2_RXD_L4_UDP)

/* Extra-offset for GENERAL packet parsing */
#define PTP_MH_ADDOFFS_RX	0 /* (MVPP2_MH_SIZE + 2) */
#define PTP_MH_ADDOFFS_TX	2 /* (MVPP2_MH_SIZE + 2) */
#define PTP_MH_ADDOFFS   	0 /* (MVPP2_MH_SIZE) */

/* ========  PTP  macros  ============================ */
#define PTP_PORT_EVENT	319 /* for PTP RX and TX */
#define PTP_PORT_SIGNAL	320 /* for PTP TX only */
/* PTP message-IDs having special TX handling */
#define PTP_SYNC	0x00
#define PTP_DELAY_REQ	0x01
#define PTP_PEER_DELAY_REQ	0x02
#define PTP_DELAY_RESP	0x09
#define PTP_ANNOUNCE	0x0b

/* PTP_HEADER_OFFS = dst_port_offs + 6 */
#define PTP_HEADER_MSG_ID_OFFS	(0)
#define PTP_HEADER_RESERVE_1BYTES_OFFS	(5)
#define PTP_HEADER_CORRECTION_FIELD_OFFS	(8) /*size 6+2bytes */
#define PTP_HEADER_RESERVE_4BYTES_OFFS	(16)
#define PTP_CORRECTION_PRIVATE_ID	0xa5

/* PTP check upon MAX-length statistically is most effective.
 * The skb->data_len is already known ZERO, so skb->len is enought
 * MAX=Announce_64bytes + MVPP2_MH_SIZE + IPv4_42 + IPv6_20 + VLAN_2 =
 *  64 + 44 + 20 + 2 = 64 + 66
 */
#define MAX_PTP_UDP_LEN	80
#define MAX_PTP_PKT_LEN	(MAX_PTP_UDP_LEN + 66)

#define PTP_TS_CS_CORRECTION_SIZE	2

/* FEATURES */
#define PTP_IGNORE_TIMESTAMPING_FLAG_FOR_DEBUG
/*#define PTP_TS_TRAFFIC_CORRECTION*/

/***  Extra debug: MV_PTP_DEBUG_HOOK ************************/
/*#define MV_PTP_DEBUG_HOOK*/
#ifdef MV_PTP_DEBUG_HOOK
#include <mv_ptp_hook_dbg.h>
#define PTP_RX_TS_PRINT(TXT, rx32bit) \
	mv_ptp_ts32bit_print(rx32bit, TXT)
#define PTP_TX_PRINT(TXT, LEN, TS_OFFS, CS_OFFS, PACT) \
	pr_info("%s: data_len=%d, ts_offs=%d cs_offs=%d, pkt_action=%d\n", \
		TXT, LEN, TS_OFFS, CS_OFFS, PACT);
#else
#define PTP_RX_TS_PRINT(TXT, rx32bit)	/**/
#define PTP_TX_PRINT(TXT, PKT_LEN, TS_OFFS, CS_OFFS, PACT)	/**/
#endif
/***  Extra debug END:MV_PTP_DEBUG_HOOK *********************/

struct ptp_stats {
	u32 tx;
	u32 tx_capture_in_queue;
	u32 rx;
};

struct mv_pp2x_ptp_desc {
	int	emac_num;
	struct ptp_stats *stats;
};


/**** Net-dev hook Utilities (used as static/inline) ******************
 *  mv_pp2_is_pkt_ptp_rx_proc()   - RX filter PTP & processing
 *  mv_pp2_is_pkt_ptp_tx_proc()   - TX filter PTP & processing
 *  mv_is_pkt_ptp_tx()            - filter PTP local-utility
 *  mv_ptp_pkt_proc_tx()          - processing local-utility
 *
 *  mv_ptp_stats_print()          - statistic (private PTP)
 *  mv_ptp_hook_enable()          - enable and statistic print
 *  mv_ptp_hook_extra_op()        - extra operations for debug
***********************************************************************/

static struct mv_pp2x *mv_ptp_priv;
static struct mv_pp2x_ptp_desc	mv_ptp_desc[MVPP2_MAX_PORTS];
static struct ptp_stats	ptp_stats[MVPP2_MAX_PORTS];

static inline void mv_ptp_stats_print(int port_num, int reset)
{
	struct ptp_stats *p = &ptp_stats[port_num];

	if (reset) {
		memset(p, 0, sizeof(struct ptp_stats));
	} else {
		/* STATS print-form optimization:
		 * MasterOnly should always have capture_in_queue=0
		 * SlaveOnly  should always have tx == capture_in_queue
		 * BoundaryClock=Master+Slave and so tx != capture_in_queue
		 */
		if ((p->tx == p->tx_capture_in_queue) || !p->tx_capture_in_queue)
			pr_info("ptp port %d stats: tx=%u ; rx=%u\n",
				port_num, p->tx, p->rx);
		else
			pr_info("ptp port %d stats: tx=%u (%u:Qcapture) ; rx=%u\n",
				port_num, p->tx, p->tx_capture_in_queue, p->rx);
	}
}


void mv_ptp_hook_extra_op(u32 val1, u32 val2, u32 val3)
{
	int i, rc = -1, clear_stats = 0;

	if (val1 == 0xc/*Clear statistic*/) {
		clear_stats = 1;
		rc = 1;
	}
	pr_info("echo deb c > [C]lear statistic %d\n", clear_stats);

	if (rc) {
		for (i = 0; i < MVPP2_MAX_PORTS; i++)
			mv_ptp_stats_print(i, clear_stats);
#ifdef PTP_TS_TRAFFIC_CORRECTION
		pr_info(" PTP_TS_TRAFFIC_CORRECTION is enabled\n");
#endif
	}
}

void mv_ptp_hook_enable(int port_num, bool enable)
{
	struct mv_pp2x_port *port_desc;

	if (!MV_PTP_PORT_IS_VALID(port_num) || !mv_ptp_priv)
		return;
	port_desc = mv_pp2x_port_struct_get_by_gop_index(mv_ptp_priv, port_num);
	if (!port_desc)
		return;
	/* New Request handling depends upon current state */
	if (!port_desc->ptp_desc) {
		/* Currently disabled */
		if (enable) {
			mv_ptp_stats_print(port_num, 1); /* new session, reset only */
			port_desc->ptp_desc = &mv_ptp_desc[port_num]; /* hook enabling */
			port_desc->ptp_desc->emac_num = port_num;
			port_desc->ptp_desc->stats = &ptp_stats[port_num];
		}
	} else {
		/* Currently enabled */
		mv_ptp_stats_print(port_num, 0); /* print out accumulated */
		if (!enable)
			port_desc->ptp_desc = NULL; /* hook disabling */
	}
}

void mv_pp2x_ptp_hook_init(void *priv, int port)
{
	struct mv_pp2x *pp2x_priv = priv;

	(void)port;
	if (pp2x_priv->pp2_version == PPV21) {
		pr_err("ERROR: pp21 (armada-375) does not support PTP\n");
		return;
	}
	if (!mv_ptp_priv)
		mv_ptp_priv = pp2x_priv;
}

int mv_ptp_netdev_name_get(int port, char *name_buf)
{
	struct mv_pp2x_port *port_desc;

	if (!MV_PTP_PORT_IS_VALID(port) || !mv_ptp_priv || !name_buf)
		return -EINVAL;
	port_desc = mv_pp2x_port_struct_get_by_gop_index(mv_ptp_priv, port);
	if (!port_desc)
		return -EINVAL;
	/* Name found, copy into given name-buffer */
	strcpy(name_buf, port_desc->dev->name);
	return 0;
}

/***************************************************************************
 **  Real-Time used utilities
 ***************************************************************************
 */
static inline int ptp_get_emac_num(struct mv_pp2x_port *port_desc)
{
	return port_desc->ptp_desc->emac_num;
}


#ifdef PTP_TS_TRAFFIC_CORRECTION
/* Under traffic long packets have transmission latency ~12us (on 1Gb link)
 * causing for PTP delay and TimeStamp-deviation 0..12us
 * Handle the case in a statistical algorithm and correct TS.
 * Use the fact that EVERY Ingress has TS-32bits in CFH
 * and pass the info to upper application.
 * The information is placed into "correction field".
 * This requires special PTP application to handle this non-standard.
 * Since the Standard correction-field is used for TransparentClock devices,
 * we need to place traffic-info only if received correction==0 and mark 0xA5
 * in 1byte-reserve to distinct standard/non-standard
 */
struct ptp_correction_field { /* correctionField not standard refill */
	u8 tx_factor;
	u8 rx_handled_burst_sz; /* Pkts handled before PTP in same napi-budget */
	u16 rx_prev_sz;
	u32 rx_prev_ts;
};

struct ptp_taffic_stats {
	int last_sz;
	u32 last_ts;
};
static struct ptp_taffic_stats ptp_taffic_rx_stats[MVPP2_MAX_PORTS];

static inline void mv_pp3_rx_traffic_stats(int emac_num, int pkt_len, u32 ts)
{
	ptp_taffic_rx_stats[emac_num].last_ts = ts;
	ptp_taffic_rx_stats[emac_num].last_sz = pkt_len;
}

static inline void mv_pp3_rx_traffic_handle(int emac_num, int pkt_len,
	u8 *ptp_data, int rcvd_pkts)
{
	struct ptp_correction_field *cf;
	struct ptp_taffic_stats *s;
	u16 *p16 = (u16 *)(ptp_data + PTP_HEADER_CORRECTION_FIELD_OFFS + 4);

	if (*p16)
		return; /* Field is not empty. Do not touch */

	ptp_data[PTP_HEADER_RESERVE_1BYTES_OFFS] = PTP_CORRECTION_PRIVATE_ID;

	cf = (void *)(ptp_data + PTP_HEADER_CORRECTION_FIELD_OFFS);
	s = &ptp_taffic_rx_stats[emac_num];
	cf->rx_handled_burst_sz = (u8)rcvd_pkts;
	cf->rx_prev_sz = (u16)s->last_sz;
	cf->rx_prev_ts = s->last_ts;
}
#else
#define mv_pp3_rx_traffic_stats(IDX, LEN, TS)
#define mv_pp3_rx_traffic_handle(IDX, LEN, DATA, RX_DONE)
#endif/*PTP_TS_TRAFFIC_CORRECTION*/


static inline void mv_pp2_is_pkt_ptp_rx_proc(struct mv_pp2x_port *port_desc,
	struct mv_pp2x_rx_desc *rx_desc, int pkt_len, u8 *pkt_data, int rcvd_pkts)
{
	int eth_tag_len, dst_port_offs, ptp_offs, ptp_hdr_offs, emac_num;
	u16 ether_type, l4_port;
	u32 ts, l3_is_ipv4;

	if (!port_desc->ptp_desc)
		return;

	emac_num = ptp_get_emac_num(port_desc);

	if (pkt_len > MAX_PTP_PKT_LEN)
		goto exit;

	if (!MV_RXD_L4_IS_UDP(rx_desc))
		goto exit;

	/* Check VLAN - needed for correct offset */
	eth_tag_len = MV_RXD_VLAN_INFO_GET(rx_desc) ? 2 : 0;

	/* Check in most-valuable ordering: port -> udpProto -> etherType */
	dst_port_offs = PTP_MH_ADDOFFS_RX + eth_tag_len +
		MV_RXD_L3_OFFS_GET(rx_desc) + MV_RXD_IPHDR_LEN_GET(rx_desc);
	l4_port = ntohs(*(u16 *)(pkt_data + dst_port_offs));
	if ((l4_port != PTP_PORT_EVENT) && (l4_port != PTP_PORT_SIGNAL))
		goto exit;

	l3_is_ipv4 = MV_RXD_L3_IS_IP4(rx_desc);
	if (!l3_is_ipv4 && !MV_RXD_L3_IS_IP6(rx_desc))
		goto exit;

	ether_type = ntohs(*(u16 *)(pkt_data + 14 + eth_tag_len));
	if (ether_type == ETH_P_1588)
		goto exit; /*ETH_P_1588=0x88F7 is not supported by upper PTP layers*/

	/* Handling PTP packet: fetch TS32bits from cfh and place into PTP header */
	ts = rx_desc->u.pp22.rsrvd_timestamp;

	ptp_hdr_offs = dst_port_offs + 6;
	if (!l3_is_ipv4)
		ptp_hdr_offs += 20; /* IPV6 header +20 bytes */

	ptp_offs = ptp_hdr_offs + PTP_HEADER_RESERVE_4BYTES_OFFS;
	memcpy(pkt_data + ptp_offs, &ts, sizeof(ts));
	PTP_RX_TS_PRINT("PTP-RX", ts);

	/*DBG_PTP_TS("ptp-rx: ts=%08x=%d.%09d\n", ts, ts >> 30, ts & 0x3fffffff);*/
	port_desc->ptp_desc->stats->rx++;
	rcvd_pkts--; /* packets received BEFORE this PTP packet as traffic-load-factor */
	mv_pp3_rx_traffic_handle(emac_num, pkt_len, pkt_data + ptp_hdr_offs, rcvd_pkts);

	return;
exit:
	mv_pp3_rx_traffic_stats(emac_num, pkt_len, cfh->tag2);
}

static inline int mv_is_pkt_ptp_tx(struct mv_pp2x_port *port_desc, struct sk_buff *skb, int *tx_ts_queue)
{
	u16 protocol, udp_port;
	int eth_tag_len;
	const int l2_hdr_len = 14;
	int ip_hdr_len; /* 20 or 40 for ipv4 or ipv6 */
	int skb_ip_len_lsb_offs;
	int skb_dst_port_offs/* UDP/L4: on offs=2 out of udpHdrLen=8 */;
	int skb_udp_len_lsb_offs;
	int skb_ptp_header_offs; /* Offset from skb-data beginning */
	int skb_ts_offs; /* OUT result: TimeStamp offset */
	const int ptp_ts_offs = 34; /* Offset from PTP-data beginning */
	u8 msg_type;
#ifndef PTP_IGNORE_TIMESTAMPING_FLAG_FOR_DEBUG
	if (!skb->sk)
		return 0;
	/* User should set for PTP/TX socket the sockopt
	 *  (SOL_SOCKET, SO_TIMESTAMPING, SOCK_TIMESTAMPING_TX_HARDWARE)
	 */
	if (!(skb->sk->sk_flags & (1 << SOCK_TIMESTAMPING_TX_HARDWARE)))
		return 0;
#endif
	if (!port_desc->ptp_desc)
		return 0;

	if (skb->len > MAX_PTP_PKT_LEN)
		return 0;

	/* Check VLAN to obtain correct offset */
	if (skb->protocol == htons(ETH_P_8021Q)) {
		eth_tag_len = 2;
		protocol = vlan_eth_hdr(skb)->h_vlan_encapsulated_proto;
	} else {
		eth_tag_len = 0;
		protocol = skb->protocol;
	}
	/* Check IP v4 vs v6 */
	switch (protocol) {
	case htons(ETH_P_IP):
		ip_hdr_len = ip_hdr(skb)->ihl * sizeof(u32);
		skb_dst_port_offs = PTP_MH_ADDOFFS_TX + eth_tag_len + l2_hdr_len + ip_hdr_len;
		udp_port = *(u16 *)(skb->data + skb_dst_port_offs);
		if ((udp_port != htons(PTP_PORT_EVENT)) && (udp_port != htons(PTP_PORT_SIGNAL)))
			return 0;
		if (ip_hdr(skb)->protocol != IPPROTO_UDP)
			return 0;
		break;

	case htons(ETH_P_IPV6):
		ip_hdr_len = 20 + 20;
		skb_dst_port_offs = PTP_MH_ADDOFFS_TX + eth_tag_len + l2_hdr_len + ip_hdr_len;
		udp_port = *(u16 *)(skb->data + skb_dst_port_offs);
		if ((udp_port != htons(PTP_PORT_EVENT)) && (udp_port != htons(PTP_PORT_SIGNAL)))
			return 0;
		break;

	default:
		/* PTP_ETHER=0x887F is not supported */
		return 0;
	}
	skb_ptp_header_offs = skb_dst_port_offs + 6;
	skb_ts_offs = skb_ptp_header_offs + ptp_ts_offs;

	/* Capture TS into Queue for TX-Slave event messages only:
	 *   DELAY_REQ=1 and PEER_DELAY_REQ=2
	 */
	msg_type = skb->data[skb_ptp_header_offs] & 0x0f;

	if (udp_port == htons(PTP_PORT_EVENT)) {
		/* Capture TS into Queue for TX-Slave event messages only:
		 *   DELAY_REQ and PEER_DELAY_REQ
		 */
		*tx_ts_queue = ((msg_type == PTP_DELAY_REQ) ||
			(msg_type == PTP_PEER_DELAY_REQ));
	} else if (msg_type == PTP_ANNOUNCE) {
		*tx_ts_queue = 0;
	} else {
		return 0;
	}
	/* PTP TX with FW TimeStamp update impacts the CheckSum.
	 * The FW does not fixes the CS but adds 2 bytes of correction-data
	 * to bring back the CS to be correct.
	 * This requires an additional 2 byte storage after TS-field.
	 * The SKB has enough storage, but the skb, UDP and IPvX length
	 * should be extended with these 2=PTP_TS_CS_CORRECTION_SIZE.
	 */
	skb_udp_len_lsb_offs = skb_dst_port_offs + 2 + 1;
	skb_ip_len_lsb_offs = PTP_MH_ADDOFFS + eth_tag_len + l2_hdr_len +
		(2 + 1)/*IPv4 TotalLen LSB*/;
	if (protocol == htons(ETH_P_IPV6))
		skb_ip_len_lsb_offs += 2;  /*IPv6 PayloadLen vs IPv4 TotalLen*/
	skb->data[skb_udp_len_lsb_offs] += PTP_TS_CS_CORRECTION_SIZE;
	skb->data[skb_ip_len_lsb_offs] += PTP_TS_CS_CORRECTION_SIZE;
	skb->len += PTP_TS_CS_CORRECTION_SIZE;

	return skb_ts_offs;
}

static inline void mv_ptp_pkt_proc_tx(struct mv_pp2x_port *port_desc,
			struct mv_pp2x_tx_desc *tx_desc, int skb_ts_offs, int tx_ts_queue)
{
	int pact, cfh_ts_offs, cfh_cs_offs, len;
	int skb_len = tx_desc->data_size;/* = skb_headlen(skb) */
	struct mv_pp2x_tx_desc l_txd = {0};
	u32 *ptxd = (u32 *)&l_txd;
	u32 *tx_desc_u32 = (u32 *)tx_desc;

	/* Convert skb offset to cfh offset (aka "TS off" field)
	 * Set PACT: 6={AddTime(to packet) + Capture(to egress queue)}
	 *   or only 4=AddTime according to the cfh-offset
	 */
	cfh_ts_offs = skb_ts_offs - PTP_MH_ADDOFFS; /* 10byte timestamp beginning */
	cfh_cs_offs = skb_len - PTP_MH_ADDOFFS_RX; /* EOPacket, hw adds CS here */
	if ((cfh_ts_offs + 10/*ts-size*/) > cfh_cs_offs)
		return; /* UDP with PTP-port but NOT ptp-packet */

	pact = (tx_ts_queue) ? 6 : 4;

	/*DBG_PTP_TS("ptp-tx: ts in_queue=%d, offs=%d\n", tx_ts_queue, cfh_ts_offs);*/
	port_desc->ptp_desc->stats->tx++;
	if (tx_ts_queue)
		port_desc->ptp_desc->stats->tx_capture_in_queue++;

	/* Access-latency to tx_desc may be bigger than local
	 * make all bit operation on local and than copy.
	 */
	MVPP2_TXD_PTP_CLEAR_EXT(ptxd);
	/* Add PTP related to TX CFH */
	/* TSE, QS, TS_off, CS_off, CUE, PACT, PF, WC, DE, ETS, SEC*/
	/*  1,   0,   76,     86,    1,   4,    0,  0,  0,  0,   0 */
	MVPP2_TXD_PTP_TSE_SET(ptxd);
	/* MV_CFH_PTP_QS_SET(0) QueueSelect=0 */
	MVPP2_TXD_PTP_TS_OFF_SET(ptxd, cfh_ts_offs); /*TS_off never including MVPP2_MH_SIZE*/
	MVPP2_TXD_PTP_CS_OFF_SET(ptxd, cfh_cs_offs); /*CS_off*/;
	MVPP2_TXD_PTP_CUE_SET(ptxd);
	MVPP2_TXD_PTP_LEN_GET(tx_desc_u32, len);
	MVPP2_TXD_PTP_LEN_SET(ptxd, len + 2); /* add 2 bytes for CS-correction */
	MVPP2_TXD_PTP_PACT_SET(ptxd, pact); /*PACT*/
	/* Final copy updates from local l_txd into real tx_desc */
	MVPP2_TXD_PTP_SET(tx_desc, ptxd);
	PTP_TX_PRINT("PTP-TX", skb_len, cfh_ts_offs, cfh_cs_offs, pact);
}


static inline void mv_pp2_is_pkt_ptp_tx_proc(struct mv_pp2x_port *port_desc,
			struct mv_pp2x_tx_desc *tx_desc, struct sk_buff *skb)
{
	int ptp_ts_offs, tx_ts_queue;

	ptp_ts_offs = mv_is_pkt_ptp_tx(port_desc, skb, &tx_ts_queue);
	if (ptp_ts_offs > 0)
		mv_ptp_pkt_proc_tx(port_desc, tx_desc, ptp_ts_offs, tx_ts_queue);
}
