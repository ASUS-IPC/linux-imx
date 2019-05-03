/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2010-2011 EIA Electronics, Kurt Van Dijck <kurt.van.dijck@eia.be>
// Copyright (c) 2017-2018 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
// Copyright (c) 2017-2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>

#ifndef _J1939_PRIV_H_
#define _J1939_PRIV_H_

#include <linux/can/j1939.h>

/* TODO: return ENETRESET on busoff. */

struct j1939_session;

enum j1939_sk_errqueue_type {
	J1939_ERRQUEUE_ACK,
	J1939_ERRQUEUE_SCHED,
	J1939_ERRQUEUE_ABORT,
};

/* j1939 devices */
struct j1939_ecu {
	struct list_head list;
	name_t name;
	u8 addr;

	/* indicates that this ecu successfully claimed @sa as its address */
	struct hrtimer ac_timer;
	struct kref kref;
	struct j1939_priv *priv;

	/* count users, to help transport protocol decide for interaction */
	int nusers;
};

struct j1939_priv {
	struct list_head ecus;
	/* local list entry in priv
	 * These allow irq (& softirq) context lookups on j1939 devices
	 * This approach (separate lists) is done as the other 2 alternatives
	 * are not easier or even wrong
	 * 1) using the pure kobject methods involves mutexes, which are not
	 *    allowed in irq context.
	 * 2) duplicating data structures would require a lot of synchronization
	 *    code
	 * usage:
	 */

	/* segments need a lock to protect the above list */
	rwlock_t lock;

	struct net_device *ndev;

	/* list of 256 ecu ptrs, that cache the claimed addresses.
	 * also protected by the above lock
	 */
	struct j1939_addr_ent {
		struct j1939_ecu *ecu;
		/* count users, to help transport protocol */
		int nusers;
	} ents[256];

	struct kref kref;

	/* protects both tp_session lists below*/
	spinlock_t tp_session_list_lock;
	struct list_head tp_sessionq;
	struct list_head tp_extsessionq;
	wait_queue_head_t tp_wait;
	unsigned int tp_max_packet_size;

	struct list_head j1939_socks;
	spinlock_t j1939_socks_lock;
};

void j1939_ecu_put(struct j1939_ecu *ecu);

/* keep the cache of what is local */
int j1939_local_ecu_get(struct j1939_priv *priv, name_t name, u8 sa);
void j1939_local_ecu_put(struct j1939_priv *priv, name_t name, u8 sa);

static inline bool j1939_address_is_unicast(u8 addr)
{
	return addr <= J1939_MAX_UNICAST_ADDR;
}

static inline bool j1939_address_is_idle(u8 addr)
{
	return addr == J1939_IDLE_ADDR;
}

static inline bool j1939_address_is_valid(u8 addr)
{
	return addr != J1939_NO_ADDR;
}

static inline bool j1939_pgn_is_pdu1(pgn_t pgn)
{
	/* ignore dp & res bits for this */
	return (pgn & 0xff00) < 0xf000;
}

/* utility to correctly unmap an ECU */
void j1939_ecu_unmap_locked(struct j1939_ecu *ecu);
void j1939_ecu_unmap(struct j1939_ecu *ecu);

u8 j1939_name_to_addr(struct j1939_priv *priv, name_t name);
struct j1939_ecu *j1939_ecu_find_by_addr_locked(struct j1939_priv *priv,
						u8 addr);
struct j1939_ecu *j1939_ecu_get_by_addr(struct j1939_priv *priv, u8 addr);
struct j1939_ecu *j1939_ecu_get_by_addr_locked(struct j1939_priv *priv,
					       u8 addr);
struct j1939_ecu *j1939_ecu_get_by_name(struct j1939_priv *priv, name_t name);
struct j1939_ecu *j1939_ecu_get_by_name_locked(struct j1939_priv *priv,
					       name_t name);

struct j1939_addr {
	name_t src_name;
	name_t dst_name;
	pgn_t pgn;

	u8 sa;
	u8 da;
};

/* control buffer of the sk_buff */
struct j1939_sk_buff_cb {
	/* j1939 clones incoming skb's.
	 * insock saves the incoming skb->sk
	 * to determine local generated packets
	 */
	struct sock *insock;

	/* Offset in bytes withing one ETP session */
	u32 offset;

	/* for tx, MSG_SYN will be used to sync on sockets */
	u32 msg_flags;

	struct j1939_addr addr;

	/* Flags for quick lookups during skb processing
	 * These are set in the receive path only
	 */
#define J1939_ECU_LOCAL	BIT(0)
	u8 src_flags;
	u8 dst_flags;

	priority_t priority;
};

static inline struct j1939_sk_buff_cb *j1939_skb_to_cb(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct j1939_sk_buff_cb) > sizeof(skb->cb));

	return (struct j1939_sk_buff_cb *)skb->cb;
}

int j1939_send_one(struct j1939_priv *priv, struct sk_buff *skb);
void j1939_sk_recv(struct j1939_priv *priv, struct sk_buff *skb);
void j1939_sk_send_multi_abort(struct j1939_priv *priv, struct sock *sk,
			       int err);
void j1939_sk_errqueue(struct j1939_session *session,
		       enum j1939_sk_errqueue_type type);

/* stack entries */
struct j1939_session *j1939_tp_send(struct j1939_priv *priv,
				    struct sk_buff *skb, size_t size);
int j1939_tp_recv(struct j1939_priv *priv, struct sk_buff *skb);
int j1939_ac_fixup(struct j1939_priv *priv, struct sk_buff *skb);
void j1939_ac_recv(struct j1939_priv *priv, struct sk_buff *skb);

/* network management */
struct j1939_ecu *j1939_ecu_create_locked(struct j1939_priv *priv, name_t name);

void j1939_ecu_timer_start(struct j1939_ecu *ecu);
void j1939_ecu_timer_cancel(struct j1939_ecu *ecu);
void j1939_ecu_unmap_all(struct j1939_priv *priv);

int j1939_netdev_start(struct net *net, struct net_device *ndev);
void j1939_netdev_stop(struct net_device *ndev);

struct j1939_priv *j1939_priv_get_by_ndev(struct net_device *ndev);
void j1939_priv_put(struct j1939_priv *priv);
void j1939_priv_get(struct j1939_priv *priv);

/* notify/alert all j1939 sockets bound to ifindex */
void j1939_sk_netdev_event(struct net_device *ndev, int error_code);
int j1939_tp_rmdev_notifier(struct j1939_priv *priv);
void j1939_tp_init(struct j1939_priv *priv);

/* decrement pending skb for a j1939 socket */
void j1939_sock_pending_del(struct sock *sk);

struct j1939_session {
	struct j1939_priv *priv;
	struct list_head list;
	struct kref kref;
	spinlock_t lock;
	struct sock *sk;

	/* ifindex, src, dst, pgn define the session block
	 * the are _never_ modified after insertion in the list
	 * this decreases locking problems a _lot_
	 */
	struct j1939_sk_buff_cb skcb;
	struct sk_buff_head skb_queue;

	/* all tx related stuff (last_txcmd, pkt.tx)
	 * is protected (modified only) with the txtimer hrtimer
	 * 'total' & 'block' are never changed,
	 * last_cmd, last & block are protected by ->lock
	 * this means that the tx may run after cts is received that should
	 * have stopped tx, but this time discrepancy is never avoided anyhow
	 */
	u8 last_cmd, last_txcmd;
	bool transmission;
	bool extd;
	unsigned int total_message_size; /* Total message size, number of bytes */
	int err;
	u32 tskey;

	/* Packets counters for a (extended) transfer session. The packet is
	 * maximal of 7 bytes. */
	struct {
		/* total - total number of packets for this session */
		unsigned int total;
		/* last - last packet of a transfer block after which responder
		 * should send ETP.CM_CTS and originator ETP.CM_DPO */
		unsigned int last;
		/* tx - number of packets send by originator node.
		 * this counter can be set back if responder node didn't
		 * received all packets send by originator. */
		unsigned int tx;
		/* done - number of packets received and confirmed by
		 * responder */
		unsigned int done;
		/* block - amount of packets expected in one block */
		unsigned int block;
		/* dpo - ETP.CM_DPO, Data Packet Offset */
		unsigned int dpo;
	} pkt;
	struct hrtimer txtimer, rxtimer;
};

void j1939_session_get(struct j1939_session *session);
void j1939_session_put(struct j1939_session *session);
void j1939_session_skb_queue(struct j1939_session *session,
			     struct sk_buff *skb);
struct j1939_session *j1939_session_get_by_skcb(struct j1939_priv *priv,
						struct j1939_sk_buff_cb *skcb,
						bool extd, bool reverse);

#define J1939_MAX_TP_PACKET_SIZE (7 * 0xff)
#define J1939_MAX_ETP_PACKET_SIZE (7 * 0x00ffffff)

#define J1939_REGULAR 0
#define J1939_EXTENDED 1

/* CAN protocol */
extern const struct can_proto j1939_can_proto;

#endif /* _J1939_PRIV_H_ */
