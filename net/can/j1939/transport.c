// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2010-2011 EIA Electronics, Kurt Van Dijck <kurt.van.dijck@eia.be>
// Copyright (c) 2017-2018 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
// Copyright (c) 2017-2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
// Copyright (c) 2018 Protonic, Robin van der Gracht <robin@protonic.nl>

#include <linux/can/skb.h>

#include "j1939-priv.h"

#define J1939_REGULAR 0
#define J1939_EXTENDED 1

#define J1939_ETP_PGN_CTL 0xc800
#define J1939_ETP_PGN_DAT 0xc700
#define J1939_TP_PGN_CTL 0xec00
#define J1939_TP_PGN_DAT 0xeb00

#define J1939_TP_CMD_RTS 0x10
#define J1939_TP_CMD_CTS 0x11
#define J1939_TP_CMD_EOMA 0x13
#define J1939_TP_CMD_BAM 0x20
#define J1939_TP_CMD_ABORT 0xff

#define J1939_ETP_CMD_RTS 0x14
#define J1939_ETP_CMD_CTS 0x15
#define J1939_ETP_CMD_DPO 0x16
#define J1939_ETP_CMD_EOMA 0x17
#define J1939_ETP_CMD_ABORT 0xff

enum j1939_xtp_abort {
	J1939_XTP_ABORT_NO_ERROR = 0,
	J1939_XTP_ABORT_BUSY = 1,
	J1939_XTP_ABORT_RESOURCE = 2,
	J1939_XTP_ABORT_TIMEOUT = 3,
	J1939_XTP_ABORT_GENERIC = 4,
	J1939_XTP_ABORT_FAULT = 5,
};

static unsigned int j1939_tp_block = 255;
static unsigned int j1939_tp_retry_ms = 20;
static unsigned int j1939_tp_packet_delay;
static unsigned int j1939_tp_padding = 1;

/* helpers */
static inline void j1939_fix_cb(struct j1939_sk_buff_cb *skcb)
{
	skcb->msg_flags &= ~MSG_SYN;
}

static inline struct list_head *j1939_sessionq(struct j1939_priv *priv,
					       bool extd)
{
	if (extd)
		return &priv->tp_extsessionq;
	else
		return &priv->tp_sessionq;
}

static inline void j1939_session_list_lock(struct j1939_priv *priv)
{
	spin_lock_bh(&priv->tp_session_list_lock);
}

static inline void j1939_session_list_unlock(struct j1939_priv *priv)
{
	spin_unlock_bh(&priv->tp_session_list_lock);
}

static void j1939_session_list_add(struct j1939_session *session)
{
	struct list_head *list = j1939_sessionq(session->priv, session->extd);

	list_add_tail(&session->list, list);
}

static void j1939_session_list_del(struct j1939_session *session)
{
	list_del_init(&session->list);
}

void j1939_session_get(struct j1939_session *session)
{
	kref_get(&session->kref);
}

static void j1939_session_destroy(struct j1939_session *session)
{
	j1939_session_list_lock(session->priv);
	j1939_session_list_del(session);
	j1939_session_list_unlock(session->priv);
	skb_queue_purge(&session->skb_queue);
	j1939_priv_put(session->priv);
	kfree(session);
}

static void __j1939_session_release(struct kref *kref)
{
	struct j1939_session *session = container_of(kref, struct j1939_session,
						     kref);

	j1939_session_destroy(session);
}

void j1939_session_put(struct j1939_session *session)
{
	kref_put(&session->kref, __j1939_session_release);
}

static void j1939_session_txtimer_cancel(struct j1939_session *session)
{
	if (hrtimer_cancel(&session->txtimer))
		j1939_session_put(session);
}

static void j1939_session_rxtimer_cancel(struct j1939_session *session)
{
	if (hrtimer_cancel(&session->rxtimer))
		j1939_session_put(session);
}

static void j1939_session_timers_cancel(struct j1939_session *session)
{
	j1939_session_txtimer_cancel(session);
	j1939_session_rxtimer_cancel(session);
}

static inline bool j1939_cb_is_broadcast(const struct j1939_sk_buff_cb *skcb)
{
	return (!skcb->addr.dst_name && (skcb->addr.da == 0xff));
}

/* transport status locking */
static inline void j1939_session_lock(struct j1939_session *session)
{
	spin_lock_bh(&session->lock);
}

static inline void j1939_session_unlock(struct j1939_session *session)
{
	spin_unlock_bh(&session->lock);
}

void j1939_session_skb_drop_old(struct j1939_session *session)
{
	struct sk_buff *do_skb;
	struct j1939_sk_buff_cb *do_skcb;
	unsigned int offset_start;
	unsigned long flags;

	if (skb_queue_len(&session->skb_queue) < 2)
		return;

	offset_start = session->pkt.done * 7;

	spin_lock_irqsave(&session->skb_queue.lock, flags);
	do_skb = skb_peek(&session->skb_queue);
	do_skcb = j1939_skb_to_cb(do_skb);

	if ((do_skcb->offset + do_skb->len) < offset_start) {
		__skb_unlink(do_skb, &session->skb_queue);
		kfree_skb(do_skb);
	}
	spin_unlock_irqrestore(&session->skb_queue.lock, flags);
}


void j1939_session_skb_queue(struct j1939_session *session,
			     struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_priv *priv = session->priv;

	j1939_ac_fixup(priv, skb);

	if (j1939_address_is_unicast(skcb->addr.da) &&
	    priv->ents[skcb->addr.da].nusers)
		skcb->dst_flags |= J1939_ECU_LOCAL;

	skcb->src_flags |= J1939_ECU_LOCAL;

	skb_queue_tail(&session->skb_queue, skb);
}

static struct sk_buff *j1939_session_skb_find(struct j1939_session *session)
{
	struct j1939_priv *priv = session->priv;
	struct sk_buff *skb = NULL;
	struct sk_buff *do_skb;
	struct j1939_sk_buff_cb *skcb, *do_skcb;
	unsigned int offset_start;
	unsigned long flags;

	offset_start = session->pkt.dpo * 7;

	spin_lock_irqsave(&session->skb_queue.lock, flags);
	skb_queue_walk(&session->skb_queue, do_skb) {

		do_skcb = j1939_skb_to_cb(do_skb);

		if (offset_start >= do_skcb->offset
		    && offset_start < (do_skcb->offset + do_skb->len)) {
			skb = do_skb;
			skcb = do_skcb;
		}
	}
	spin_unlock_irqrestore(&session->skb_queue.lock, flags);

	if (!skb)
		netdev_warn(priv->ndev, "no skb found for start: %i, queue size: %i\n",
			    offset_start,
			    skb_queue_len(&session->skb_queue));

	return skb;
}

/* see if we are receiver
 * returns 0 for broadcasts, although we will receive them
 */
static inline int j1939_tp_im_receiver(struct j1939_sk_buff_cb *skcb)
{
	return skcb->dst_flags & J1939_ECU_LOCAL;
}

/* see if we are sender */
static inline int j1939_tp_im_transmitter(struct j1939_sk_buff_cb *skcb)
{
	return skcb->src_flags & J1939_ECU_LOCAL;
}

/* see if we are involved as either receiver or transmitter */
static int j1939_tp_im_involved(struct j1939_sk_buff_cb *skcb, bool swap)
{
	if (swap)
		return j1939_tp_im_receiver(skcb);
	else
		return j1939_tp_im_transmitter(skcb);
}

static int j1939_tp_im_involved_anydir(struct j1939_sk_buff_cb *skcb)
{
	return (skcb->src_flags | skcb->dst_flags) & J1939_ECU_LOCAL;
}

/* extract pgn from flow-ctl message */
static inline pgn_t j1939_xtp_ctl_to_pgn(const u8 *dat)
{
	pgn_t pgn;

	pgn = (dat[7] << 16) | (dat[6] << 8) | (dat[5] << 0);
	if (j1939_pgn_is_pdu1(pgn))
		pgn &= 0xffff00;
	return pgn;
}

static inline unsigned int j1939_tp_ctl_to_size(const u8 *dat)
{
	return (dat[2] << 8) + (dat[1] << 0);
}

static inline unsigned int j1939_etp_ctl_to_packet(const u8 *dat)
{
	return (dat[4] << 16) | (dat[3] << 8) | (dat[2] << 0);
}

static inline unsigned int j1939_etp_ctl_to_size(const u8 *dat)
{
	return (dat[4] << 24) | (dat[3] << 16) |
		(dat[2] << 8) | (dat[1] << 0);
}

/* find existing session:
 * reverse: swap cb's src & dst
 * there is no problem with matching broadcasts, since
 * broadcasts (no dst, no da) would never call this
 * with reverse == true
 */
static bool j1939_session_match(struct j1939_session *session,
				struct j1939_sk_buff_cb *skcb, bool reverse)
{
	struct j1939_sk_buff_cb *se_skcb = &session->skcb;
	struct j1939_addr *se_addr = &se_skcb->addr;
	struct j1939_addr *sk_addr = &skcb->addr;

	if (reverse) {
		if (se_addr->src_name) {
			if (se_addr->src_name != sk_addr->dst_name)
				return false;
		} else if (se_addr->sa != sk_addr->da) {
			return false;
		}

		if (se_addr->dst_name) {
			if (se_addr->dst_name != sk_addr->src_name)
				return false;
		} else if (se_addr->da != sk_addr->sa) {
			return false;
		}
	} else {
		if (se_addr->src_name) {
			if (se_addr->src_name != sk_addr->src_name)
				return false;
		} else if (se_addr->sa != sk_addr->sa) {
			return false;
		}

		if (se_addr->dst_name) {
			if (se_addr->dst_name != sk_addr->dst_name)
				return false;
		} else if (se_addr->da != sk_addr->da) {
			return false;
		}
	}

	return true;
}

static struct
j1939_session *j1939_session_get_by_skcb_locked(struct j1939_priv *priv,
					       struct list_head *root,
					       struct j1939_sk_buff_cb *skcb,
					       bool reverse)
{
	struct j1939_session *session;

	lockdep_assert_held(&priv->tp_session_list_lock);

	list_for_each_entry(session, root, list) {
		j1939_session_get(session);
		if (j1939_session_match(session, skcb, reverse))
			return session;
		j1939_session_put(session);
	}

	return NULL;
}

static struct j1939_session *j1939_session_get_by_skcb(struct j1939_priv *priv,
						      struct j1939_sk_buff_cb *skcb,
						      bool extd, bool reverse)
{
	struct list_head *root = j1939_sessionq(priv, extd);
	struct j1939_session *session;

	j1939_session_list_lock(priv);
	session = j1939_session_get_by_skcb_locked(priv, root, skcb, reverse);
	j1939_session_list_unlock(priv);

	return session;
}

static void j1939_skbcb_swap(struct j1939_sk_buff_cb *skcb)
{
	swap(skcb->addr.dst_name, skcb->addr.src_name);
	swap(skcb->addr.da, skcb->addr.sa);
	swap(skcb->dst_flags, skcb->src_flags);
}

static struct sk_buff *j1939_tp_tx_dat_new(struct j1939_priv *priv,
					   struct j1939_sk_buff_cb *re_skcb,
					   bool extd, bool ctl,
					   bool swap_src_dst)
{
	struct sk_buff *skb;
	struct j1939_sk_buff_cb *skcb;

	skb = alloc_skb(sizeof(struct can_frame) + sizeof(struct can_skb_priv),
			GFP_ATOMIC);
	if (unlikely(!skb))
		return ERR_PTR(-ENOMEM);

	skb->dev = priv->ndev;
	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = priv->ndev->ifindex;
	/* reserve CAN header */
	skb_reserve(skb, offsetof(struct can_frame, data));

	memcpy(skb->cb, re_skcb, sizeof(skb->cb));
	skcb = j1939_skb_to_cb(skb);
	j1939_fix_cb(skcb);
	if (swap_src_dst)
		j1939_skbcb_swap(skcb);

	if (ctl) {
		if (extd)
			skcb->addr.pgn = J1939_ETP_PGN_CTL;
		else
			skcb->addr.pgn = J1939_TP_PGN_CTL;
	} else {
		if (extd)
			skcb->addr.pgn = J1939_ETP_PGN_DAT;
		else
			skcb->addr.pgn = J1939_TP_PGN_DAT;
	}

	return skb;
}

/* TP transmit packet functions */
static int j1939_tp_tx_dat(struct j1939_session *session,
			   const u8 *dat, int len)
{
	struct j1939_priv *priv = session->priv;
	struct sk_buff *skb;

	skb = j1939_tp_tx_dat_new(priv, &session->skcb, session->extd, false, false);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	skb_put_data(skb, dat, len);
	if (j1939_tp_padding && len < 8)
		memset(skb_put(skb, 8 - len), 0xff, 8 - len);

	return j1939_send_one(priv, skb);
}

static int j1939_xtp_do_tx_ctl(struct j1939_priv *priv,
			       struct j1939_sk_buff_cb *re_skcb, bool extd,
			       bool swap_src_dst, pgn_t pgn, const u8 *dat)
{
	struct sk_buff *skb;
	u8 *skdat;

	if (!j1939_tp_im_involved(re_skcb, swap_src_dst))
		return 0;

	skb = j1939_tp_tx_dat_new(priv, re_skcb, extd, true, swap_src_dst);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	skdat = skb_put(skb, 8);
	memcpy(skdat, dat, 5);
	skdat[5] = (pgn >> 0);
	skdat[6] = (pgn >> 8);
	skdat[7] = (pgn >> 16);

	return j1939_send_one(priv, skb);
}

static inline int j1939_tp_tx_ctl(struct j1939_session *session,
				  bool swap_src_dst, const u8 *dat)
{
	struct j1939_priv *priv = session->priv;

	return j1939_xtp_do_tx_ctl(priv, &session->skcb, session->extd,
				   swap_src_dst,
				   session->skcb.addr.pgn, dat);
}

static int j1939_xtp_tx_abort(struct j1939_priv *priv,
			      struct j1939_sk_buff_cb *re_skcb,
			      bool extd, bool swap_src_dst,
			      enum j1939_xtp_abort err,
			      pgn_t pgn)
{
	u8 dat[5];

	if (!j1939_tp_im_involved(re_skcb, swap_src_dst))
		return 0;

	memset(dat, 0xff, sizeof(dat));
	dat[0] = J1939_TP_CMD_ABORT;
	if (extd)
		dat[1] = J1939_XTP_ABORT_GENERIC;
	else
		dat[1] = err;
	return j1939_xtp_do_tx_ctl(priv, re_skcb, extd, swap_src_dst, pgn, dat);
}

static inline void j1939_tp_schedule_txtimer(struct j1939_session *session,
					     int msec)
{
	j1939_session_get(session);
	hrtimer_start(&session->txtimer, ms_to_ktime(msec),
		      HRTIMER_MODE_REL);
}

static inline void j1939_tp_set_rxtimeout(struct j1939_session *session,
					  int msec)
{
	j1939_session_rxtimer_cancel(session);
	j1939_session_get(session);
	hrtimer_start(&session->rxtimer, ms_to_ktime(msec),
		      HRTIMER_MODE_REL);
}

/* transmit function */
static int j1939_tp_txnext(struct j1939_session *session)
{
	u8 dat[8];
	const u8 *tpdat;
	int ret, offset, pkt_done, pkt_end;
	unsigned int pkt, len, pdelay;
	struct sk_buff *se_skb;
	struct j1939_sk_buff_cb *skcb;

	memset(dat, 0xff, sizeof(dat));

	switch (session->last_cmd) {
	case 0:
		if (!j1939_tp_im_transmitter(&session->skcb))
			break;
		dat[1] = (session->total_message_size >> 0);
		dat[2] = (session->total_message_size >> 8);
		dat[3] = session->pkt.total;
		if (session->extd) {
			dat[0] = J1939_ETP_CMD_RTS;
			dat[1] = (session->total_message_size >> 0);
			dat[2] = (session->total_message_size >> 8);
			dat[3] = (session->total_message_size >> 16);
			dat[4] = (session->total_message_size >> 24);
		} else if (j1939_cb_is_broadcast(&session->skcb)) {
			dat[0] = J1939_TP_CMD_BAM;
			/* fake cts for broadcast */
			session->pkt.tx = 0;
		} else {
			dat[0] = J1939_TP_CMD_RTS;
			dat[4] = dat[3];
		}
		if (dat[0] == session->last_txcmd)
			/* done already */
			break;
		ret = j1939_tp_tx_ctl(session, false, dat);
		if (ret < 0)
			goto failed;
		session->last_txcmd = dat[0];
		/* must lock? */
		if (dat[0] == J1939_TP_CMD_BAM)
			j1939_tp_schedule_txtimer(session, 50);
		j1939_tp_set_rxtimeout(session, 1250);
		break;
	case J1939_TP_CMD_RTS:
	case J1939_ETP_CMD_RTS: /* fallthrough */
		if (!j1939_tp_im_receiver(&session->skcb))
			break;
 tx_cts:
		ret = 0;
		len = session->pkt.total - session->pkt.done;
		len = min3(len, session->pkt.block, j1939_tp_block ?: 255);

		if (session->extd) {
			pkt = session->pkt.done + 1;
			dat[0] = J1939_ETP_CMD_CTS;
			dat[1] = len;
			dat[2] = (pkt >> 0);
			dat[3] = (pkt >> 8);
			dat[4] = (pkt >> 16);
		} else {
			dat[0] = J1939_TP_CMD_CTS;
			dat[1] = len;
			dat[2] = session->pkt.done + 1;
		}
		if (dat[0] == session->last_txcmd)
			/* done already */
			break;
		ret = j1939_tp_tx_ctl(session, true, dat);
		if (ret < 0)
			goto failed;
		if (len)
			/* only mark cts done when len is set */
			session->last_txcmd = dat[0];
		j1939_tp_set_rxtimeout(session, 1250);
		break;
	case J1939_ETP_CMD_CTS:
		if (j1939_tp_im_transmitter(&session->skcb) &&
		    session->extd &&
		    session->last_txcmd != J1939_ETP_CMD_DPO) {
			/* do dpo */
			dat[0] = J1939_ETP_CMD_DPO;
			session->pkt.dpo = session->pkt.done;
			pkt = session->pkt.dpo;
			dat[1] = session->pkt.last - session->pkt.done;
			dat[2] = (pkt >> 0);
			dat[3] = (pkt >> 8);
			dat[4] = (pkt >> 16);
			ret = j1939_tp_tx_ctl(session, false, dat);
			if (ret < 0)
				goto failed;
			session->last_txcmd = dat[0];
			j1939_tp_set_rxtimeout(session, 1250);
			session->pkt.tx = session->pkt.done;
		}
		/* fallthrough */
	case J1939_TP_CMD_CTS: /* fallthrough */
	case 0xff: /* did some data */
	case J1939_ETP_CMD_DPO: /* fallthrough */
		if ((session->extd ||
		     !j1939_cb_is_broadcast(&session->skcb)) &&
		    j1939_tp_im_receiver(&session->skcb)) {
			if (session->pkt.done >= session->pkt.total) {
				if (session->extd) {
					dat[0] = J1939_ETP_CMD_EOMA;
					dat[1] = session->total_message_size >> 0;
					dat[2] = session->total_message_size >> 8;
					dat[3] = session->total_message_size >> 16;
					dat[4] = session->total_message_size >> 24;
				} else {
					dat[0] = J1939_TP_CMD_EOMA;
					dat[1] = session->total_message_size;
					dat[2] = session->total_message_size >> 8;
					dat[3] = session->pkt.total;
				}
				if (dat[0] == session->last_txcmd)
					/* done already */
					break;
				ret = j1939_tp_tx_ctl(session, true, dat);
				if (ret < 0)
					goto failed;
				session->last_txcmd = dat[0];
				j1939_tp_set_rxtimeout(session, 1250);
				/* wait for the EOMA packet to come in */
				break;
			} else if (session->pkt.done >= session->pkt.last) {
				session->last_txcmd = 0;
				goto tx_cts;
			}
		}
	case J1939_TP_CMD_BAM: /* fallthrough */
		if (!j1939_tp_im_transmitter(&session->skcb))
			break;

		se_skb = j1939_session_skb_find(session);
		if (!se_skb)
			break;

		skcb = j1939_skb_to_cb(se_skb);
		tpdat = se_skb->data;
		ret = 0;
		pkt_done = 0;
		if (!session->extd && j1939_cb_is_broadcast(&session->skcb))
			pkt_end = session->pkt.total;
		else
			pkt_end = session->pkt.last;

		while (session->pkt.tx < pkt_end) {
			dat[0] = session->pkt.tx - session->pkt.dpo + 1;
			offset = (session->pkt.tx * 7) - skcb->offset;
			len =  se_skb->len - offset;
			if (len > 7)
				len = 7;
			memcpy(&dat[1], &tpdat[offset], len);
			ret = j1939_tp_tx_dat(session, dat, len + 1);
			if (ret < 0)
				break;
			session->last_txcmd = 0xff;
			++pkt_done;
			++session->pkt.tx;
			pdelay = j1939_cb_is_broadcast(&session->skcb) ? 50 :
				j1939_tp_packet_delay;
			if (session->pkt.tx < session->pkt.total && pdelay) {
				j1939_tp_schedule_txtimer(session, pdelay);
				break;
			}
		}
		if (pkt_done)
			j1939_tp_set_rxtimeout(session, 250);
		if (ret)
			goto failed;
		break;
	}

	return 0;

 failed:
	return ret;
}

/* timer & scheduler functions */
static enum hrtimer_restart j1939_tp_txtimer(struct hrtimer *hrtimer)
{
	struct j1939_session *session =
		container_of(hrtimer, struct j1939_session, txtimer);
	int ret;

	ret = j1939_tp_txnext(session);
	if (ret < 0)
		j1939_tp_schedule_txtimer(session, j1939_tp_retry_ms ?: 20);
	j1939_session_put(session);

	return HRTIMER_NORESTART;
}

/* session completion functions */
static void __j1939_session_drop(struct j1939_session *session)
{
	struct j1939_priv *priv = session->priv;

	if (!session->transmission)
		return;

	j1939_sock_pending_del(session->sk);
	wake_up_all(&priv->tp_wait);
}

static void j1939_session_completed(struct j1939_session *session)
{
	struct sk_buff *se_skb = j1939_session_skb_find(session);

	/* distribute among j1939 receivers */
	j1939_sk_recv(session->priv, se_skb);
	__j1939_session_drop(session);
}

static void j1939_session_cancel(struct j1939_session *session,
				 enum j1939_xtp_abort err)
{
	struct j1939_priv *priv = session->priv;

	/* do not send aborts on incoming broadcasts */
	if (err && !j1939_cb_is_broadcast(&session->skcb))
		j1939_xtp_tx_abort(priv, &session->skcb, session->extd,
				   !(session->skcb.src_flags & J1939_ECU_LOCAL),
				   err, session->skcb.addr.pgn);

	__j1939_session_drop(session);
}

static enum hrtimer_restart j1939_tp_rxtimer(struct hrtimer *hrtimer)
{
	struct j1939_session *session = container_of(hrtimer,
						     struct j1939_session,
						     rxtimer);
	struct j1939_priv *priv = session->priv;

	netdev_alert(priv->ndev, "%s: timeout\n", __func__);
	j1939_session_txtimer_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_TIMEOUT);
	j1939_session_put(session);

	return HRTIMER_NORESTART;
}

/* receive packet functions */
static void j1939_xtp_rx_bad_message_one(struct j1939_priv *priv,
					 struct sk_buff *skb, bool extd,
					 bool reverse)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	pgn_t pgn;

	pgn = j1939_xtp_ctl_to_pgn(skb->data);
	session = j1939_session_get_by_skcb(priv, skcb, extd, reverse);
	if (!session) {
		j1939_xtp_tx_abort(priv, skcb, extd, false, J1939_XTP_ABORT_FAULT, pgn);
		return;
	}

	/* FIXME: extend session match to search for PGN? In case of BOM TP.
	 * (session->skcb.addr.pgn == pgn)
	 */
	/* do not allow TP control messages on 2 pgn's */
	j1939_session_cancel(session, J1939_XTP_ABORT_FAULT);
	j1939_session_put(session);
}

/* abort packets may come in 2 directions */
static void j1939_xtp_rx_bad_message(struct j1939_priv *priv,
				     struct sk_buff *skb, bool extd)
{
	netdev_info(priv->ndev, "%s, pgn %05x\n", __func__,
		    j1939_xtp_ctl_to_pgn(skb->data));

	j1939_xtp_rx_bad_message_one(priv, skb, extd, false);
	j1939_xtp_rx_bad_message_one(priv, skb, extd, true);
}

static void j1939_xtp_rx_abort_one(struct j1939_priv *priv, struct sk_buff *skb,
				   bool extd, bool reverse)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	pgn_t pgn;

	pgn = j1939_xtp_ctl_to_pgn(skb->data);
	session = j1939_session_get_by_skcb(priv, skcb, extd, reverse);
	if (!session)
		return;
	if (session->transmission && !session->last_txcmd) {
		/* empty block:
		 * do not drop session when a transmit session did not
		 * start yet
		 */
	} else if (session->skcb.addr.pgn == pgn) {
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_NO_ERROR);
	}

	/* TODO: maybe cancel current connection
	 * as another pgn was communicated
	 */
	j1939_session_put(session);
}

/* abort packets may come in 2 directions */
static void j1939_xtp_rx_abort(struct j1939_priv *priv, struct sk_buff *skb,
			       bool extd)
{
	netdev_info(priv->ndev, "%s, %05x\n", __func__,
		    j1939_xtp_ctl_to_pgn(skb->data));

	j1939_xtp_rx_abort_one(priv, skb, extd, false);
	j1939_xtp_rx_abort_one(priv, skb, extd, true);
}

static void j1939_xtp_rx_eoma(struct j1939_session *session, struct sk_buff *skb,
			      bool extd)
{
	struct j1939_priv *priv = session->priv;
	pgn_t pgn;

	/* end of tx cycle */
	pgn = j1939_xtp_ctl_to_pgn(skb->data);

	j1939_session_timers_cancel(session);
	if (session->skcb.addr.pgn != pgn) {
		struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);

		j1939_xtp_tx_abort(priv, skcb, extd, true, J1939_XTP_ABORT_BUSY,
				   pgn);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);
	} else {
		/* transmitted without problems */
		j1939_session_completed(session);
	}
}

static void j1939_xtp_rx_cts(struct j1939_session *session, struct sk_buff *skb,
			     bool extd)
{
	struct j1939_priv *priv = session->priv;
	pgn_t pgn;
	unsigned int pkt;
	const u8 *dat;

	dat = skb->data;
	pgn = j1939_xtp_ctl_to_pgn(skb->data);

	if (session->skcb.addr.pgn != pgn) {
		struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
		/* what to do? */
		j1939_xtp_tx_abort(priv, skcb, extd, true, J1939_XTP_ABORT_BUSY,
				   pgn);
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);
		goto out_session_put;
	}

	j1939_session_lock(session);
	if (extd)
		pkt = j1939_etp_ctl_to_packet(dat);
	else
		pkt = dat[2];
	if (!pkt) {
		goto out_session_unlock;
	} else if (dat[1] > session->pkt.block /* 0xff for etp */) {
		goto out_session_unlock;
	} else {
		/* set packet counters only when not CTS(0) */
		session->pkt.done = pkt - 1;
		j1939_session_skb_drop_old(session);
		session->pkt.last = session->pkt.done + dat[1];
		if (session->pkt.last > session->pkt.total)
			/* safety measure */
			session->pkt.last = session->pkt.total;
		/* TODO: do not set tx here, do it in txtimer */
		session->pkt.tx = session->pkt.done;
	}

	session->last_cmd = dat[0];
	j1939_session_unlock(session);
	if (dat[1]) {
		j1939_tp_set_rxtimeout(session, 1250);
		if (j1939_tp_im_transmitter(&session->skcb))
			j1939_tp_schedule_txtimer(session, 0);
	} else {
		/* CTS(0) */
		j1939_tp_set_rxtimeout(session, 550);
	}
	return;

 out_session_unlock:
	j1939_session_unlock(session);
	j1939_session_timers_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_FAULT);
 out_session_put:
	j1939_session_put(session);
}

static struct j1939_session *j1939_session_new(struct j1939_priv *priv,
					       struct sk_buff *skb, size_t size)
{
	struct j1939_session *session;
	struct j1939_sk_buff_cb *skcb;

	session = kzalloc(sizeof(*session), gfp_any());
	if (!session)
		return NULL;
	INIT_LIST_HEAD(&session->list);
	spin_lock_init(&session->lock);
	kref_init(&session->kref);

	j1939_priv_get(priv);
	session->priv = priv;
	session->total_message_size = size;

	skb_queue_head_init(&session->skb_queue);
	skb_queue_tail(&session->skb_queue, skb);

	skcb = j1939_skb_to_cb(skb);
	memcpy(&session->skcb, skcb, sizeof(session->skcb));

	hrtimer_init(&session->txtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	session->txtimer.function = j1939_tp_txtimer;
	hrtimer_init(&session->rxtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	session->rxtimer.function = j1939_tp_rxtimer;

	return session;
}

static struct j1939_session *j1939_session_fresh_new(struct j1939_priv *priv,
						     int size,
						     const struct j1939_sk_buff_cb *rel_skcb,
						     pgn_t pgn)
{
	struct sk_buff *skb;
	struct j1939_sk_buff_cb *skcb;
	struct j1939_session *session;

	skb = alloc_skb(size + sizeof(struct can_skb_priv), GFP_ATOMIC);
	if (unlikely(!skb))
		return NULL;

	skb->dev = priv->ndev;
	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = priv->ndev->ifindex;
	skcb = j1939_skb_to_cb(skb);
	memcpy(skcb, rel_skcb, sizeof(*skcb));
	j1939_fix_cb(skcb);
	skcb->addr.pgn = pgn;

	session = j1939_session_new(priv, skb, skb->len);
	if (!session) {
		kfree_skb(skb);
		return NULL;
	}

	/* alloc data area */
	skb_put(skb, size);
	/* skb is recounted in j1939_session_new() */
	return session;
}

static int j1939_session_insert(struct j1939_session *session)
{
	struct j1939_priv *priv = session->priv;
	struct j1939_session *pending;
	int ret = 0;

	pending = j1939_session_get_by_skcb(priv, &session->skcb, session->extd,
					   false);
	if (pending) {
		j1939_session_put(pending);
		ret = -EAGAIN;
	} else {
		j1939_session_list_lock(priv);
		j1939_session_list_add(session);
		j1939_session_list_unlock(priv);
	}

	return ret;
}


struct j1939_session *j1939_xtp_rx_rts_new(struct j1939_priv *priv,
					   struct sk_buff *skb, bool extd)
{
	enum j1939_xtp_abort abort = J1939_XTP_ABORT_NO_ERROR;
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	const u8 *dat;
	pgn_t pgn;
	int len;

	if (j1939_tp_im_transmitter(skcb)) {
		netdev_alert(priv->ndev, "%s: I should tx (%02x %02x)\n",
			     __func__, skcb->addr.sa, skcb->addr.da);

		return NULL;
	}

	dat = skb->data;
	pgn = j1939_xtp_ctl_to_pgn(dat);

	if (extd) {
		len = j1939_etp_ctl_to_size(dat);
		if (len > J1939_MAX_ETP_PACKET_SIZE)
			abort = J1939_XTP_ABORT_FAULT;
		else if (len > priv->tp_max_packet_size)
			abort = J1939_XTP_ABORT_RESOURCE;
		else if (len <= J1939_MAX_TP_PACKET_SIZE)
			abort = J1939_XTP_ABORT_FAULT;
	} else {
		len = j1939_tp_ctl_to_size(dat);
		if (len > J1939_MAX_TP_PACKET_SIZE)
			abort = J1939_XTP_ABORT_FAULT;
		else if (len > priv->tp_max_packet_size)
			abort = J1939_XTP_ABORT_RESOURCE;
	}
	if (abort) {
		j1939_xtp_tx_abort(priv, skcb, extd, true, abort, pgn);
		return NULL;
	}

	session = j1939_session_fresh_new(priv, len, skcb, pgn);
	if (!session) {
		j1939_xtp_tx_abort(priv, skcb, extd, true,
				   J1939_XTP_ABORT_RESOURCE, pgn);
		return NULL;
	}
	session->extd = extd;

	/* initialize the control buffer: plain copy */
	session->pkt.total = (len + 6) / 7;
	session->pkt.block = 0xff;
	if (!extd) {
		if (dat[3] != session->pkt.total)
			netdev_alert(priv->ndev, "%s: strange total, %u != %u\n",
				     __func__, session->pkt.total,
				     dat[3]);
		session->pkt.total = dat[3];
		session->pkt.block = min(dat[3], dat[4]);
	}

	session->pkt.done = 0;
	session->pkt.tx = 0;

	WARN_ON_ONCE(j1939_session_insert(session));

	return session;
}

static int j1939_xtp_rx_rts_current(struct j1939_session *session,
				struct sk_buff *skb, bool extd)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_priv *priv = session->priv;
	const u8 *dat;
	pgn_t pgn;

	dat = skb->data;
	pgn = j1939_xtp_ctl_to_pgn(dat);

	if (!j1939_tp_im_transmitter(skcb)) {
		/* RTS on pending connection */
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);

		if (pgn != session->skcb.addr.pgn &&
		    dat[0] != J1939_TP_CMD_BAM)
			j1939_xtp_tx_abort(priv, skcb, extd, true,
					   J1939_XTP_ABORT_BUSY, pgn);

		return -EBUSY;
	}

	if (session->last_cmd != 0) {
		/* we received a second rts on the same connection */
		netdev_alert(priv->ndev, "%s: connection exists (%02x %02x). last cmd: %x\n",
			     __func__, skcb->addr.sa, skcb->addr.da,
			     session->last_cmd);

		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);

		return -EBUSY;
	}

	if (session->skcb.addr.sa != skcb->addr.sa ||
	    session->skcb.addr.da != skcb->addr.da)
		netdev_warn(priv->ndev, "%s: session->skcb.addr.sa=0x%02x skcb->addr.sa=0x%02x session->skcb.addr.da=0x%02x skcb->addr.da=0x%02x\n",
			    __func__,
			    session->skcb.addr.sa, skcb->addr.sa,
			    session->skcb.addr.da, skcb->addr.da);
	/* make sure 'sa' & 'da' are correct !
	 * They may be 'not filled in yet' for sending
	 * skb's, since they did not pass the Address Claim ever.
	 */
	session->skcb.addr.sa = skcb->addr.sa;
	session->skcb.addr.da = skcb->addr.da;

	return 0;
}

static void j1939_xtp_rx_dpo(struct j1939_session *session, struct sk_buff *skb)
{
	struct j1939_priv *priv = session->priv;
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	const u8 *dat = skb->data;
	pgn_t pgn;

	pgn = j1939_xtp_ctl_to_pgn(dat);

	if (session->skcb.addr.pgn != pgn) {
		netdev_info(priv->ndev, "%s: different pgn\n", __func__);
		j1939_xtp_tx_abort(priv, skcb, true, true, J1939_XTP_ABORT_BUSY,
				   pgn);
		j1939_session_timers_cancel(session);
		j1939_session_cancel(session, J1939_XTP_ABORT_BUSY);
		return;
	}

	/* transmitted without problems */
	session->pkt.dpo = j1939_etp_ctl_to_packet(skb->data);
	session->last_cmd = dat[0];
	j1939_tp_set_rxtimeout(session, 750);
}

static void j1939_xtp_rx_dat(struct j1939_priv *priv, struct sk_buff *skb,
			     bool extd)
{
	struct j1939_sk_buff_cb *skcb;
	struct j1939_session *session;
	struct sk_buff *se_skb;
	const u8 *dat;
	u8 *tpdat;
	int offset;
	int nbytes;
	bool final = false;
	bool do_cts_eoma = false;
	int packet;

	skcb = j1939_skb_to_cb(skb);
	session = j1939_session_get_by_skcb(priv, skcb, extd, false);
	if (!session) {
		netdev_info(priv->ndev, "%s: no connection found\n", __func__);
		return;
	}
	dat = skb->data;
	if (skb->len <= 1)
		/* makes no sense */
		goto out_session_cancel;

	j1939_session_lock(session);

	switch (session->last_cmd) {
	case 0xff:
		break;
	case J1939_ETP_CMD_DPO:
		if (extd)
			break;
	case J1939_TP_CMD_BAM:
	case J1939_TP_CMD_CTS:
		if (!extd)
			break;
	default:
		netdev_info(priv->ndev, "%s: last %02x\n", __func__,
			    session->last_cmd);
		goto out_session_unlock;
	}

	packet = (dat[0] - 1 + session->pkt.dpo);
	if (packet > session->pkt.total ||
	    (session->pkt.done + 1) > session->pkt.total) {
		netdev_info(priv->ndev, "%s: should have been completed\n",
			    __func__);
		goto out_session_unlock;
	}
	se_skb = j1939_session_skb_find(session);
	skcb = j1939_skb_to_cb(se_skb);
	offset = packet * 7 - skcb->offset;
	nbytes = se_skb->len - offset;
	if (nbytes > 7)
		nbytes = 7;
	if (nbytes <= 0 || (nbytes + 1) > skb->len) {
		netdev_info(priv->ndev, "%s: nbytes %i, len %i\n", __func__,
			    nbytes, skb->len);
		goto out_session_unlock;
	}

	tpdat = se_skb->data;
	memcpy(&tpdat[offset], &dat[1], nbytes);
	if (packet == session->pkt.done)
		++session->pkt.done;

	if (!extd && j1939_cb_is_broadcast(&session->skcb)) {
		if (session->pkt.done >= session->pkt.total)
			final = true;
	} else {
		/* never final, an EOMA must follow */
		if (session->pkt.done >= session->pkt.last)
			do_cts_eoma = true;
	}
	j1939_session_unlock(session);

	if (final) {
		j1939_session_completed(session);
	} else if (do_cts_eoma) {
		j1939_tp_set_rxtimeout(session, 1250);
		if (j1939_tp_im_receiver(&session->skcb))
			j1939_tp_schedule_txtimer(session, 0);
	} else {
		j1939_tp_set_rxtimeout(session, 250);
	}
	session->last_cmd = 0xff;
	j1939_session_put(session);

	return;

 out_session_unlock:
	/* unlock session (spinlock) before trying to send */
	j1939_session_unlock(session);
 out_session_cancel:
	j1939_session_timers_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_FAULT);
	j1939_session_put(session);
}

static inline int j1939_tp_tx_initial(struct j1939_session *session)
{
	int ret;

	ret = j1939_tp_txnext(session);
	/* set nonblocking for further packets */
	session->skcb.msg_flags |= MSG_DONTWAIT;

	return ret;
}

/* j1939 main intf */
struct j1939_session *j1939_tp_send(struct j1939_priv *priv,
				    struct sk_buff *skb, size_t size)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	bool extd = J1939_REGULAR;
	int ret;

	if (skcb->addr.pgn == J1939_TP_PGN_DAT ||
	    skcb->addr.pgn == J1939_TP_PGN_CTL ||
	    skcb->addr.pgn == J1939_ETP_PGN_DAT ||
	    skcb->addr.pgn == J1939_ETP_PGN_CTL)
		/* avoid conflict */
		return ERR_PTR(-EDOM);

	if (size > priv->tp_max_packet_size)
		return ERR_PTR(-EMSGSIZE);

	if (size > J1939_MAX_TP_PACKET_SIZE)
		extd = J1939_EXTENDED;

	if (extd && j1939_cb_is_broadcast(skcb))
		return ERR_PTR(-EDESTADDRREQ);

	/* fill in addresses from names */
	ret = j1939_ac_fixup(priv, skb);
	if (unlikely(ret))
		return ERR_PTR(ret);

	/* fix dst_flags, it may be used there soon */
	if (j1939_address_is_unicast(skcb->addr.da) &&
	    priv->ents[skcb->addr.da].nusers)
		skcb->dst_flags |= J1939_ECU_LOCAL;

	/* src is always local, I'm sending ... */
	skcb->src_flags |= J1939_ECU_LOCAL;

	/* prepare new session */
	session = j1939_session_new(priv, skb, size);
	if (!session)
		return ERR_PTR(-ENOMEM);

	/* skb is recounted in j1939_session_new() */
	session->sk = skb->sk;
	session->extd = extd;
	session->transmission = true;
	session->pkt.total = (size + 6) / 7;
	session->pkt.block = session->extd ? 255 :
		min(j1939_tp_block ?: 255, session->pkt.total);

	if (j1939_cb_is_broadcast(&session->skcb))
		/* set the end-packet for broadcast */
		session->pkt.last = session->pkt.total;

	/* insert into queue, but avoid collision with pending session */
	if (session->skcb.msg_flags & MSG_DONTWAIT)
		ret = j1939_session_insert(session);
	else
		ret = wait_event_interruptible(priv->tp_wait,
					       j1939_session_insert(session) == 0);
	if (ret < 0)
		goto failed;

	ret = j1939_tp_tx_initial(session);
	if (ret)
		goto failed;

	/* transmission started */
	return session;

 failed:
	j1939_session_timers_cancel(session);
	j1939_session_cancel(session, J1939_XTP_ABORT_NO_ERROR);
	j1939_session_put(session);
	return ERR_PTR(ret);
}

static void j1939_tp_cmd_recv(struct j1939_priv *priv, struct sk_buff *skb,
			      bool extd_pgn)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	struct j1939_session *session;
	bool extd = J1939_REGULAR;
	const u8 *dat = skb->data;

	switch (*dat) {
	case J1939_ETP_CMD_RTS:
		extd = J1939_EXTENDED;
	case J1939_TP_CMD_BAM: /* falltrough */
	case J1939_TP_CMD_RTS: /* falltrough */
		if (extd != extd_pgn)
			goto rx_bad_message;

		if (*dat == J1939_TP_CMD_RTS && j1939_cb_is_broadcast(skcb)) {
			netdev_alert(priv->ndev, "%s: rts without destination (%02x)\n",
				     __func__, skcb->addr.sa);
			return;
		}

		session = j1939_session_get_by_skcb(priv, skcb, extd, false);
		/* TODO: abort RTS when a similar
		 * TP is pending in the other direction
		 */
		if (session) {
			if (j1939_xtp_rx_rts_current(session, skb, extd))
				break;
		} else {
			session = j1939_xtp_rx_rts_new(priv, skb, extd);
			if (!session)
				break;
		}
		session->last_cmd = dat[0];

		j1939_tp_set_rxtimeout(session, 1250);

		if ((dat[0] != J1939_TP_CMD_BAM) &&
		    j1939_tp_im_receiver(&session->skcb))
			j1939_tp_schedule_txtimer(session, 0);

		j1939_session_put(session);
		break;

	case J1939_ETP_CMD_CTS:
		extd = J1939_EXTENDED;
	case J1939_TP_CMD_CTS: /* falltrough */
		if (extd != extd_pgn)
			goto rx_bad_message;

		session = j1939_session_get_by_skcb(priv, skcb, extd, true);
		if (!session)
			break;
		j1939_xtp_rx_cts(session, skb, extd_pgn);
		j1939_session_put(session);
		break;

	case J1939_ETP_CMD_DPO:
		if (!extd_pgn)
			goto rx_bad_message;

		session = j1939_session_get_by_skcb(priv, skcb, J1939_EXTENDED, false);
		if (!session) {
			netdev_info(priv->ndev, "%s: no connection found\n", __func__);
			break;
		}
		j1939_xtp_rx_dpo(session, skb);
		j1939_session_put(session);
		break;

	case J1939_ETP_CMD_EOMA:
		extd = J1939_EXTENDED;
	case J1939_TP_CMD_EOMA: /* falltrough */
		if (extd != extd_pgn)
			goto rx_bad_message;

		session = j1939_session_get_by_skcb(priv, skcb, extd, true);
		if (!session)
			break;
		j1939_xtp_rx_eoma(session, skb, extd_pgn);
		j1939_session_put(session);
		break;

	case J1939_ETP_CMD_ABORT: /* && J1939_TP_CMD_ABORT */
		j1939_xtp_rx_abort(priv, skb, extd_pgn);
		break;
	default:
		goto rx_bad_message;
	}

	return;

rx_bad_message:
	j1939_xtp_rx_bad_message(priv, skb, extd_pgn);
}

int j1939_tp_recv(struct j1939_priv *priv, struct sk_buff *skb)
{
	struct j1939_sk_buff_cb *skcb = j1939_skb_to_cb(skb);
	bool extd = J1939_REGULAR;

	if (!j1939_tp_im_involved_anydir(skcb))
		return 0;

	switch (skcb->addr.pgn) {
	case J1939_ETP_PGN_DAT:
		extd = J1939_EXTENDED;
	case J1939_TP_PGN_DAT: /* falltrough */
		j1939_xtp_rx_dat(priv, skb, extd);
		break;

	case J1939_ETP_PGN_CTL:
		extd = J1939_EXTENDED;
	case J1939_TP_PGN_CTL: /* falltrough */
		if (skb->len < 8) {
			j1939_xtp_rx_bad_message(priv, skb, extd);
			break;
		}

		j1939_tp_cmd_recv(priv, skb, extd);
		break;
	default:
		return 0; /* no problem */
	}
	return 1; /* "I processed the message" */
}

int j1939_tp_rmdev_notifier(struct j1939_priv *priv)
{
	struct j1939_session *session, *saved;

	j1939_session_list_lock(priv);
	list_for_each_entry_safe(session, saved,
				 &priv->tp_sessionq, list) {
		j1939_session_timers_cancel(session);
	}
	list_for_each_entry_safe(session, saved,
				 &priv->tp_extsessionq, list) {
		j1939_session_timers_cancel(session);
	}
	j1939_session_list_unlock(priv);
	return NOTIFY_DONE;
}

void j1939_tp_init(struct j1939_priv *priv)
{
	spin_lock_init(&priv->tp_session_list_lock);
	INIT_LIST_HEAD(&priv->tp_sessionq);
	INIT_LIST_HEAD(&priv->tp_extsessionq);
	init_waitqueue_head(&priv->tp_wait);
	priv->tp_max_packet_size = J1939_MAX_ETP_PACKET_SIZE;
}
