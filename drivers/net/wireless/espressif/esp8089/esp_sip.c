/*
 * Copyright (c) 2009 - 2014 Espressif System.
 *
 * Serial Interconnctor Protocol
 */

#include <linux/ieee80211.h>
#include <net/mac80211.h>
#include <net/cfg80211.h>
#include <linux/skbuff.h>
#include <linux/bitops.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sd.h>
#include <linux/completion.h>

#include "esp_mac80211.h"
#include "esp_pub.h"
#include "esp_sip.h"
#include "esp_ctrl.h"
#include "esp_sif.h"
#include "esp_debug.h"
#include "slc_host_register.h"
#include "esp_wmac.h"
#include "esp_utils.h"

extern struct completion *gl_bootup_cplx;

static int old_signal = -35;
static int avg_signal;
static int signal_loop;

struct esp_mac_prefix esp_mac_prefix_table[] = {
	{0, {0x18, 0xfe, 0x34} },
	{1, {0xac, 0xd0, 0x74} },
	{255, {0x18, 0xfe, 0x34} },
};

#define SIGNAL_COUNT  300

/* FIXME: Incomplete ternary condition */
#define TID_TO_AC(_tid) (!(_tid) || (_tid) == 3 ? WME_AC_BE : ((_tid) < 3 ? WME_AC_BK : ((_tid) < 6 ? WME_AC_VI : WME_AC_VO)))

#define esp_sip_dbg esp_dbg
struct sip_trace {
	u32 tx_data;
	u32 tx_cmd;
	u32 rx_data;
	u32 rx_evt;
	u32 rx_tx_status;
	u32 tx_out_of_credit;
	u32 tx_one_shot_overflow;
};

static struct sip_trace str;

#define STRACE_TX_DATA_INC() (str.tx_data++)
#define STRACE_TX_CMD_INC()  (str.tx_cmd++)
#define STRACE_RX_DATA_INC() (str.rx_data++)
#define STRACE_RX_EVENT_INC() (str.rx_evt++)
#define STRACE_RX_TXSTATUS_INC() (str.rx_tx_status++)
#define STRACE_TX_OUT_OF_CREDIT_INC() (str.tx_out_of_credit++)
#define STRACE_TX_ONE_SHOT_INC() (str.tx_one_shot_overflow++)

#define SIP_STOP_QUEUE_THRESHOLD 48
#define SIP_RESUME_QUEUE_THRESHOLD  12

#define SIP_MIN_DATA_PKT_LEN    (sizeof(struct esp_mac_rx_ctrl) + 24)	//24 is min 80211hdr

static void sip_recalc_credit_init(struct esp_sip *sip);

static int sip_recalc_credit_claim(struct esp_sip *sip, int force);

static void sip_recalc_credit_release(struct esp_sip *sip);

static struct sip_pkt *sip_get_ctrl_buf(struct esp_sip *sip,
					enum sip_buf_type bftype);

static void sip_reclaim_ctrl_buf(struct esp_sip *sip, struct sip_pkt *pkt,
				 enum sip_buf_type bftype);

static void sip_free_init_ctrl_buf(struct esp_sip *sip);

static int sip_pack_pkt(struct esp_sip *sip, struct sk_buff *skb,
			int *pm_state);

static struct esp_mac_rx_ctrl *sip_parse_normal_mac_ctrl(struct sk_buff *skb,
							 int *pkt_len_enc,
							 int *buf_len,
							 int *pulled_len);

static struct sk_buff *sip_parse_data_rx_info(struct esp_sip *sip,
					      struct sk_buff *skb,
					      int pkt_len_enc, int buf_len,
					      struct esp_mac_rx_ctrl *mac_ctrl,
					      int *pulled_len);

static inline void sip_rx_pkt_enqueue(struct esp_sip *sip, struct sk_buff *skb);

static void sip_update_tx_credits(struct esp_sip *sip, u16 recycled_credits);

static bool sip_rx_pkt_process(struct esp_sip *sip, struct sk_buff *skb);

static void sip_tx_status_report(struct esp_sip *sip, struct sk_buff *skb,
				 struct ieee80211_tx_info *tx_info,
				 bool success);

static bool check_ac_tid(u8 *pkt, u8 ac, u8 tid)
{
	struct ieee80211_hdr *wh = (struct ieee80211_hdr *)pkt;

	if (!ieee80211_is_data_qos(wh->frame_control) &&
	    !ieee80211_is_mgmt(wh->frame_control) &&
	    !ieee80211_is_ctl(wh->frame_control)) {
		if (tid || ac != WME_AC_BE) {
			pr_info("444 ac:%u, tid:%u\n", ac, tid);

			if (tid == 7 && ac == WME_AC_VO)
				return false;
		}

		return true;	//hack to modify non-qos null data.
	}

	return false;
}

static void sip_recalc_credit_timeout(unsigned long data)
{
	struct esp_sip *sip = (struct esp_sip *)data;

	sip_recalc_credit_claim(sip, 1);	/* recalc again */
}

static void sip_recalc_credit_init(struct esp_sip *sip)
{
	atomic_set(&sip->credit_status, RECALC_CREDIT_DISABLE);	//set it disable

	setup_timer(&sip->credit_timer, sip_recalc_credit_timeout,
		    (unsigned long)sip);
}

static int sip_recalc_credit_claim(struct esp_sip *sip, int force)
{
	int ret;

	if (atomic_read(&sip->credit_status) == RECALC_CREDIT_ENABLE && !force)
		return 1;

	atomic_set(&sip->credit_status, RECALC_CREDIT_ENABLE);
	ret = sip_send_recalc_credit(sip->epub);
	if (ret) {
		dev_err(sip->epub->dev, "sending recalc credit failed: %d",
			ret);
		return ret;
	}

	/*setup a timer for handle the abs_credit not receive */
	mod_timer(&sip->credit_timer, jiffies + msecs_to_jiffies(2000));

	return ret;
}

static void sip_recalc_credit_release(struct esp_sip *sip)
{
	if (atomic_read(&sip->credit_status) == RECALC_CREDIT_ENABLE) {
		atomic_set(&sip->credit_status, RECALC_CREDIT_DISABLE);
		del_timer_sync(&sip->credit_timer);
	} else {
		dev_dbg(sip->epub->dev, "maybe bogus credit");
	}
}

static void sip_update_tx_credits(struct esp_sip *sip, u16 recycled_credits)
{
	if (recycled_credits & 0x800) {
		atomic_set(&sip->tx_credits, (recycled_credits & 0x7ff));
		sip_recalc_credit_release(sip);
	} else {
		atomic_add(recycled_credits, &sip->tx_credits);
	}
}

void sip_trigger_txq_process(struct esp_sip *sip)
{
	if (atomic_read(&sip->tx_credits) <= sip->credit_to_reserve + SIP_CTRL_CREDIT_RESERVE ||
	    atomic_read(&sip->credit_status) == RECALC_CREDIT_ENABLE)
		return;

	if (sip_queue_may_resume(sip)) {
		/* wakeup upper queue only if we have sufficient credits */
		atomic_set(&sip->epub->txq_stopped, false);
		ieee80211_wake_queues(sip->epub->hw);
	} else if (atomic_read(&sip->epub->txq_stopped)) {
		dev_err(sip->epub->dev, "%s can't wake txq, credits: %d\n",
			__func__, atomic_read(&sip->tx_credits));
	}

	if (!skb_queue_empty(&sip->epub->txq)) {
		/* try to send out pkt already in sip queue once we have credits */
#if !defined(FPGA_TXDATA)
		if (!sif_get_ate_config())
			ieee80211_queue_work(sip->epub->hw,
					     &sip->epub->tx_work);
		else
			queue_work(sip->epub->esp_wkq, &sip->epub->tx_work);
#else
		queue_work(sip->epub->esp_wkq, &sip->epub->tx_work);
#endif
	}
}

static bool sip_ampdu_occupy_buf(struct esp_sip *sip,
				 struct esp_rx_ampdu_len *ampdu_len)
{
	return (!ampdu_len->substate ||
		esp_wmac_rxsec_error(ampdu_len->substate) ||
		(sip->dump_rpbm_err && ampdu_len->substate == RX_RPBM_ERR));
}

/* TODO: HARDCORE CLEANUP */

static bool sip_rx_pkt_process(struct esp_sip *sip, struct sk_buff *skb)
{
	struct sip_hdr *hdr;
	struct sk_buff *rskb;
	struct esp_mac_rx_ctrl *mac_ctrl, new_mac_ctrl;
	int remains_len, first_pkt_len, ret = 0, pkt_len_enc = 0, buf_len = 0,
	    pulled_len = 0, pkt_num;
	u8 *bufptr;
	bool trigger_rxq = false, have_rxabort = false, have_goodpkt = false;
	struct esp_rx_ampdu_len *ampdu_len;
	static int pkt_dropped, pkt_total;
	static u8 frame_head[16];
	static u8 frame_buf_ttl;

	if (!skb)
		return trigger_rxq;

	hdr = (struct sip_hdr *)skb->data;
	bufptr = skb->data;

	if (hdr->h_credits & SIP_CREDITS_MASK)
		sip_update_tx_credits(sip, hdr->h_credits & SIP_CREDITS_MASK);

	hdr->h_credits &= ~SIP_CREDITS_MASK;	/* clean credits in sip_hdr, prevent over-add */

	/* first pkt's length is stored in  recycled_credits first 20 bits
	 * config w3 [31:12]
	 * repair hdr->len of first pkt
	 */
	remains_len = hdr->len;
	first_pkt_len = hdr->h_credits >> 12;
	hdr->len = first_pkt_len;

	if (first_pkt_len > remains_len) {
		sip_recalc_credit_claim(sip, 0);
		show_buf((u8 *)hdr, first_pkt_len);
		ESSERT(0);
		goto _exit;
	}

	/* pkts handling, including the first pkt, should alloc new skb for each data pkt.
	 * free the original whole skb after parsing is done.
	 */
	while (remains_len) {
		if (remains_len < sizeof(struct sip_hdr)) {
			sip_recalc_credit_claim(sip, 0);
			ESSERT(0);
			show_buf((u8 *)hdr, 512);
			goto _exit;
		}

		hdr = (struct sip_hdr *)bufptr;
		if (hdr->len <= 0 || hdr->len & 3) {
			sip_recalc_credit_claim(sip, 0);
			show_buf((u8 *)hdr, 512);
			ESSERT(0);
			goto _exit;
		}

		if (unlikely(hdr->seq != sip->rxseq++)) {
			sip_recalc_credit_claim(sip, 0);
			dev_dbg(sip->epub->dev,
				"%s seq mismatch! got %u, expect %u\n",
				__func__, hdr->seq, sip->rxseq - 1);
			sip->rxseq = hdr->seq + 1;
			show_buf(bufptr, 32);
			ESSERT(0);
		}

		if (SIP_HDR_IS_CTRL(hdr)) {
			STRACE_RX_EVENT_INC();

			ret = sip_parse_events(sip, bufptr);
			skb_pull(skb, hdr->len);
		} else if (SIP_HDR_IS_DATA(hdr)) {
			STRACE_RX_DATA_INC();
			mac_ctrl = sip_parse_normal_mac_ctrl(skb, &pkt_len_enc,
							     &buf_len,
							     &pulled_len);
			rskb = sip_parse_data_rx_info(sip, skb, pkt_len_enc,
						      buf_len, mac_ctrl,
						      &pulled_len);

			if (!rskb)
				goto _move_on;

			if (likely(atomic_read(&sip->epub->wl.off) == 0)) {
				local_bh_disable();
				ieee80211_rx(sip->epub->hw, rskb);
				local_bh_enable();
			} else {
				/* still need go thro parsing as skb_pull should invoke */
				kfree_skb(rskb);
			}
		} else if (SIP_HDR_IS_AMPDU(hdr)) {
			ampdu_len = (struct esp_rx_ampdu_len *)(skb->data + hdr->len / sip->rx_blksz * sip->rx_blksz);
			dev_dbg(sip->epub->dev, "%s rx ampdu total len %u\n",
				    __func__, hdr->len);
			if (skb->data != (u8 *)hdr) {
				printk("%p %p\n", skb->data, hdr);
				show_buf(skb->data, 512);
				show_buf((u8 *)hdr, 512);
				ESSERT(0);
				goto _exit;
			}
			mac_ctrl = sip_parse_normal_mac_ctrl(skb, NULL, NULL,
							     &pulled_len);
			memcpy(&new_mac_ctrl, mac_ctrl,
			       sizeof(struct esp_mac_rx_ctrl));
			mac_ctrl = &new_mac_ctrl;
			pkt_num = mac_ctrl->ampdu_cnt;
			dev_dbg(sip->epub->dev,
				"%s %d rx ampdu %u pkts, %d pkts dumped, first len %u\n",
				__func__, __LINE__,
				(unsigned int)((hdr->len % sip->rx_blksz) /
					       sizeof(struct esp_rx_ampdu_len)),
				pkt_num, (unsigned int)ampdu_len->sublen);

			pkt_total += mac_ctrl->ampdu_cnt;
			while (pkt_num > 0) {
				dev_dbg(sip->epub->dev,
					"%s %d ampdu sub state %02x,\n",
					__func__, __LINE__,
					ampdu_len->substate);

				if (sip_ampdu_occupy_buf(sip, ampdu_len)) {	//pkt is dumped
					rskb = sip_parse_data_rx_info(sip, skb,
								      ampdu_len->sublen - FCS_LEN,
								      0,
								      mac_ctrl,
								      &pulled_len);
					if (!rskb) {
						ESSERT(0);
						goto _exit;
					}

					if (likely(atomic_read(&sip->epub->wl.off) == 0) &&
					    (ampdu_len->substate == 0 ||
					     ampdu_len->substate == RX_TKIPMIC_ERR ||
					     (sip->sendup_rpbm_pkt &&
					      ampdu_len->substate == RX_RPBM_ERR)) &&
					    (sip->rxabort_fixed ||
					     !have_rxabort)) {
						if (!have_goodpkt) {
							have_goodpkt = true;
							memcpy(frame_head,
							       rskb->data,
							       16);
							frame_head[1] &= ~0x80;
							frame_buf_ttl = 3;
						}
						local_bh_disable();
						ieee80211_rx(sip->epub->hw,
							     rskb);
						local_bh_enable();
					} else {
						kfree_skb(rskb);
					}
				} else {
					if (ampdu_len->substate == RX_ABORT) {
						u8 *a;
						have_rxabort = true;
						dev_dbg(sip->epub->dev,
							"rx abort %d %d\n",
							frame_buf_ttl, pkt_num);
						if (frame_buf_ttl &&
						    !sip->rxabort_fixed) {
							struct esp_rx_ampdu_len *next_good_ampdu_len = ampdu_len + 1;
							a = frame_head;
							while (!sip_ampdu_occupy_buf(sip, next_good_ampdu_len)) {
								if (next_good_ampdu_len > ampdu_len + pkt_num - 1)
									break;
								next_good_ampdu_len++;
							}
							if (next_good_ampdu_len <= ampdu_len + pkt_num - 1) {
								bool b0, b10, b11;
								a = skb->data;
								b0 = memcmp(frame_head + 4, skb->data + 4, 12) == 0;
								b10 = memcmp(frame_head + 10, skb->data, 6) == 0;
								b11 = memcpy(frame_head + 11, skb->data, 5) == 0;
								if (b0 && !b10 &&
								    !b11) {
									have_rxabort = false;
								} else if (!b0 &&
									   b10 &&
									   !b11) {
									skb_push(skb, 10);
									memcpy(skb->data,
									       frame_head,
									       10);
									have_rxabort = false;
									pulled_len -= 10;
								} else if (!b0 && !b10 && b11) {
									skb_push(skb, 11);
									memcpy(skb->data,
									       frame_head,
									       11);
									have_rxabort = false;
									pulled_len -= 11;
								}
							}
						}
					}
					pkt_dropped++;
					dev_dbg(sip->epub->dev,
						"%s ampdu dropped %d/%d\n",
						__func__, pkt_dropped,
						pkt_total);
				}
				pkt_num--;
				ampdu_len++;
			}
			if (frame_buf_ttl)
				frame_buf_ttl--;
			skb_pull(skb, hdr->len - pulled_len);
		} else {
			dev_err(sip->epub->dev, "unknown SIP HDR type\n");
		}

_move_on:
		if (hdr->len < remains_len)
			remains_len -= hdr->len;
		else
			break;
		bufptr += hdr->len;
	}

_exit:
	kfree_skb(skb);

	return trigger_rxq;
}

static void _sip_rxq_process(struct esp_sip *sip)
{
	struct sk_buff *skb;
	bool sendup = false;

	while ((skb = skb_dequeue(&sip->rxq)))
		if (sip_rx_pkt_process(sip, skb))
			sendup = true;

	if (sendup)
		queue_work(sip->epub->esp_wkq, &sip->epub->sendup_work);

	/* probably tx_credit is updated, try txq */
	sip_trigger_txq_process(sip);
}

void sip_rxq_process(struct work_struct *work)
{
	struct esp_sip *sip = container_of(work, struct esp_sip,
					   rx_process_work);

	if (!sip) {
		ESSERT(0);
		return;
	}

	if (unlikely(atomic_read(&sip->state) == SIP_SEND_INIT)) {
		sip_send_chip_init(sip);
		atomic_set(&sip->state, SIP_WAIT_BOOTUP);
		return;
	}

	mutex_lock(&sip->rx_mtx);
	_sip_rxq_process(sip);
	mutex_unlock(&sip->rx_mtx);
}

static inline void sip_rx_pkt_enqueue(struct esp_sip *sip, struct sk_buff *skb)
{
	skb_queue_tail(&sip->rxq, skb);
}

static u32 sip_rx_count;

int sip_rx(struct esp_pub *epub)
{
	struct sip_hdr *shdr;
	struct esp_sip *sip = epub->sip;
	struct sk_buff *first_skb, *rx_skb;
	u8 *rx_buf;
	u32 rx_blksz, first_sz;
	int err;
	u8 raw_seq;

	if (likely(sif_get_ate_config() != 1)) {
		raw_seq = sif_get_regs(epub)->intr_raw & 0xff;

		if (raw_seq != sip->to_host_seq) {
			if (raw_seq == sip->to_host_seq + 1) {	/* when last read pkt crc err, this situation may occur, but raw_seq mustn't < to_host_Seq */
				sip->to_host_seq = raw_seq;
				dev_dbg(epub->dev,
					"warn: to_host_seq reg 0x%02x, seq 0x%02x",
					raw_seq, sip->to_host_seq);
			} else {
				dev_err(epub->dev,
					"err: to_host_seq reg 0x%02x, seq 0x%02x",
					raw_seq, sip->to_host_seq);
				err = 0;
				goto _err;
			}
		}
	}

	/* first read one block out, if we luck enough, that's it
	 *
	 *  To make design as simple as possible, we allocate skb(s)
	 *  separately for each sif read operation to avoid global
	 *  read_buf_pointe access.  It coule be optimized late.
	 */

	first_sz = sif_get_regs(epub)->config_w0;
	rx_blksz = sif_get_blksz(epub);
	first_skb = __dev_alloc_skb(roundup(first_sz, rx_blksz), GFP_KERNEL);

	if (!first_skb) {
		sif_unlock_bus(epub);
		err = 0;
		goto _err;
	}

	rx_buf = skb_put(first_skb, first_sz);

	err = esp_common_read(epub, rx_buf, first_sz, ESP_SIF_NOSYNC, false);
	sip_rx_count++;
	if (unlikely(err)) {
		dev_err(epub->dev, " first read err %d %d\n", err,
			sif_get_regs(epub)->config_w0);
		kfree_skb(first_skb);
		sif_unlock_bus(epub);
		goto _err;
	}

	shdr = (struct sip_hdr *)rx_buf;
	if (SIP_HDR_IS_CTRL(shdr) && shdr->c_evtid == SIP_EVT_SLEEP) {
		atomic_set(&sip->epub->ps.state, ESP_PM_ON);
	}

	if (likely(sif_get_ate_config() != 1))
		sip->to_host_seq++;

	if (shdr->len & 3) {
		dev_err(epub->dev, "shdr->len[%d] error\n", shdr->len);
		kfree_skb(first_skb);
		sif_unlock_bus(epub);
		err = -EIO;
		goto _err;
	}

	if (shdr->len != first_sz) {
		dev_err(epub->dev, "shdr->len[%d]  first_size[%d] error\n",
			shdr->len, first_sz);
		kfree_skb(first_skb);
		sif_unlock_bus(epub);
		err = -EIO;
		goto _err;
	} else {
		sif_unlock_bus(epub);
		skb_trim(first_skb, shdr->len);
		dev_dbg(epub->dev, "first_skb only\n");

		rx_skb = first_skb;
	}

	if (atomic_read(&sip->state) == SIP_STOP) {
		kfree_skb(rx_skb);
		dev_err(epub->dev, "rx packet while sip stopped\n");
		return 0;
	}

	sip_rx_pkt_enqueue(sip, rx_skb);
	queue_work(sip->epub->esp_wkq, &sip->rx_process_work);

_err:
	return err;
}

int sip_post_init(struct esp_sip *sip, struct sip_evt_bootup2 *bevt)
{
	struct esp_pub *epub;
	u8 mac_id = bevt->mac_addr[0];
	int mac_index = 0, i;

	if (!sip) {
		ESSERT(0);
		return -EINVAL;
	}

	epub = sip->epub;

	sip->tx_aggr_write_ptr = sip->tx_aggr_buf;
	sip->tx_blksz = bevt->tx_blksz;
	sip->rx_blksz = bevt->rx_blksz;
	sip->credit_to_reserve = bevt->credit_to_reserve;
	sip->dump_rpbm_err = bevt->options & SIP_DUMP_RPBM_ERR;
	sip->rxabort_fixed = bevt->options & SIP_RXABORT_FIXED;
	sip->support_bgscan = bevt->options & SIP_SUPPORT_BGSCAN;
	sip->sendup_rpbm_pkt = 0;

	/* print out MAC addr... */
	memcpy(epub->mac_addr, bevt->mac_addr, ETH_ALEN);
	for (i = 0;
	     i < sizeof(esp_mac_prefix_table) / sizeof(struct esp_mac_prefix);
	     i++)
		if (esp_mac_prefix_table[i].mac_index == mac_id) {
			mac_index = i;
			break;
		}

	epub->mac_addr[0] = esp_mac_prefix_table[mac_index].mac_addr_prefix[0];
	epub->mac_addr[1] = esp_mac_prefix_table[mac_index].mac_addr_prefix[1];
	epub->mac_addr[2] = esp_mac_prefix_table[mac_index].mac_addr_prefix[2];

	atomic_set(&sip->noise_floor, bevt->noise_floor);

	sip_recalc_credit_init(sip);

	return 0;
}

/* write pkts in aggr buf to target memory */
static void sip_write_pkts(struct esp_sip *sip, int pm_state)
{
	int tx_aggr_len, err;
	struct sip_hdr *first_shdr;

	tx_aggr_len = sip->tx_aggr_write_ptr - sip->tx_aggr_buf;
	if (tx_aggr_len < sizeof(struct sip_hdr)) {
		dev_err(sip->epub->dev, "[tx_aggr_len] %d < sizeof(sip_hdr)\n",
			tx_aggr_len);
		ESSERT(0);
		return;
	}

	if (tx_aggr_len & 0x3) {
		ESSERT(0);
		return;
	}

	first_shdr = (struct sip_hdr *)sip->tx_aggr_buf;

	if (atomic_read(&sip->tx_credits) <= SIP_CREDITS_LOW_THRESHOLD)
		first_shdr->fc[1] |= SIP_HDR_F_NEED_CRDT_RPT;

	/* still use lock bus instead of sif_lldesc_write_sync since we want to protect several global varibles assignments */
	sif_lock_bus(sip->epub);

	err = esp_common_write(sip->epub, sip->tx_aggr_buf, tx_aggr_len,
			       ESP_SIF_NOSYNC);
	if (err)
		dev_err(sip->epub->dev, "error while writing pkts: %d\n", err);

	sip->tx_aggr_write_ptr = sip->tx_aggr_buf;
	sip->tx_tot_len = 0;

	sif_unlock_bus(sip->epub);
}

/* setup sip header and tx info, copy pkt into aggr buf */
static int sip_pack_pkt(struct esp_sip *sip, struct sk_buff *skb, int *pm_state)
{
	struct ieee80211_tx_info *itx_info;
	struct sip_hdr *shdr;
	struct ieee80211_hdr *wh;
	struct esp_vif *evif;
	struct esp_node *node;
	u32 tx_len, offset = 0;
	bool is_data = true;
	u8 sta_index;
	int alg;

	itx_info = IEEE80211_SKB_CB(skb);
	if (itx_info->flags == 0xffffffff) {
		shdr = (struct sip_hdr *)skb->data;
		is_data = false;
		tx_len = skb->len;
	} else {
		wh = (struct ieee80211_hdr *)skb->data;
		evif = (struct esp_vif *)itx_info->control.vif->drv_priv;
		/* update sip header */
		shdr = (struct sip_hdr *)sip->tx_aggr_write_ptr;

		shdr->fc[0] = 0;
		shdr->fc[1] = 0;

		if (itx_info->flags & IEEE80211_TX_CTL_AMPDU)
			SIP_HDR_SET_TYPE(shdr->fc[0], SIP_DATA_AMPDU);
		else
			SIP_HDR_SET_TYPE(shdr->fc[0], SIP_DATA);

		if (!evif->epub) {
			sip_tx_status_report(sip, skb, itx_info, false);
			atomic_dec(&sip->tx_data_pkt_queued);
			return -EINVAL;
		}

		/* make room for encrypted pkt */
		if (itx_info->control.hw_key) {
			alg = esp_cipher2alg(itx_info->control.hw_key->cipher);
			if (unlikely(alg == -1)) {
				sip_tx_status_report(sip, skb, itx_info, false);
				atomic_dec(&sip->tx_data_pkt_queued);
				return -1;
			}

			shdr->d_enc_flag = alg + 1;
			shdr->d_hw_kid = itx_info->control.hw_key->hw_key_idx |
				(evif->index << 7);
		} else {
			shdr->d_enc_flag = 0;
			shdr->d_hw_kid = evif->index << 7 | evif->index;
		}

		/* update sip tx info */
		node = esp_get_node_by_addr(sip->epub, wh->addr1);
		if (node)
			sta_index = node->index;
		else
			sta_index = ESP_PUB_MAX_STA + 1;

		SIP_HDR_SET_IFIDX(shdr->fc[0], evif->index << 3 | sta_index);
		shdr->d_p2p = itx_info->control.vif->p2p;

		if (evif->index == 1)
			shdr->d_p2p = 1;

		shdr->d_ac = skb_get_queue_mapping(skb);
		shdr->d_tid = skb->priority & IEEE80211_QOS_CTL_TAG1D_MASK;

		wh = (struct ieee80211_hdr *)skb->data;

		if (ieee80211_is_mgmt(wh->frame_control)) {
			/* addba/delba/bar may use different tid/ac */
			if (shdr->d_ac == WME_AC_VO)
				shdr->d_tid = 7;

			if (ieee80211_is_beacon(wh->frame_control)) {
				shdr->d_tid = 8;
				shdr->d_ac = 4;
			}
		}

		if (check_ac_tid(skb->data, shdr->d_ac, shdr->d_tid)) {
			shdr->d_ac = WME_AC_BE;
			shdr->d_tid = 0;
		}

		/* make sure data is start at 4 bytes aligned addr. */
		offset = roundup(sizeof(struct sip_hdr), 4);

		if (SIP_HDR_IS_AMPDU(shdr)) {
			memset(sip->tx_aggr_write_ptr + offset, 0,
			       sizeof(struct esp_tx_ampdu_entry));
			offset += roundup(sizeof(struct esp_tx_ampdu_entry), 4);
		}

		tx_len = offset + skb->len;
		shdr->len = tx_len;	/* actual len */
	}

	shdr->seq = sip->txseq++;

	/* copy skb to aggr buf */
	memcpy(sip->tx_aggr_write_ptr + offset, skb->data, skb->len);

	if (is_data) {
		spin_lock_bh(&sip->epub->tx_lock);
		sip->txdataseq = shdr->seq;
		spin_unlock_bh(&sip->epub->tx_lock);

		/* fake a tx_status and report to mac80211 stack to speed up tx, may affect
		 *  1) rate control (now it's all in target, so should be OK)
		 *  2) ps mode, mac80211 want to check ACK of ps/nulldata to see if AP is awake
		 *  3) BAR, mac80211 do BAR by checking ACK
		 *
		 *  XXX: need to adjust for 11n, e.g. report tx_status according to BA received in target
		 */
		sip_tx_status_report(sip, skb, itx_info, true);
		atomic_dec(&sip->tx_data_pkt_queued);

		STRACE_TX_DATA_INC();
	} else {
		/* check pm state here */

		/* no need to hold ctrl skb */
		sip_free_ctrl_skbuff(sip, skb);
		STRACE_TX_CMD_INC();
	}

	/* TBD: roundup here or whole aggr-buf */
	tx_len = roundup(tx_len, sip->tx_blksz);

	sip->tx_aggr_write_ptr += tx_len;
	sip->tx_tot_len += tx_len;

	return 0;
}

static void sip_tx_status_report(struct esp_sip *sip, struct sk_buff *skb,
				 struct ieee80211_tx_info *tx_info,
				 bool success)
{
	struct ieee80211_hdr *wh;
	struct esp_node *node;
	struct esp_tx_tid *tid;
	struct ieee80211_sta *sta;
	u8 tidno;

	if (!(tx_info->flags & IEEE80211_TX_CTL_AMPDU)) {
		if (likely(success))
			tx_info->flags |= IEEE80211_TX_STAT_ACK;
		else
			tx_info->flags &= ~IEEE80211_TX_STAT_ACK;

		/* manipulate rate status... */
		tx_info->status.rates[0].idx = 11;
		tx_info->status.rates[0].count = 1;
		tx_info->status.rates[0].flags = 0;
		tx_info->status.rates[1].idx = -1;
	} else {
		tx_info->flags |= IEEE80211_TX_STAT_AMPDU |
			IEEE80211_TX_STAT_ACK;
		tx_info->status.ampdu_len = 1;
		tx_info->status.ampdu_ack_len = 1;

		/* manipulate rate status... */
		tx_info->status.rates[0].idx = 7;
		tx_info->status.rates[0].count = 1;
		tx_info->status.rates[0].flags = IEEE80211_TX_RC_MCS |
			IEEE80211_TX_RC_SHORT_GI;
		tx_info->status.rates[1].idx = -1;
	}

	if (!mod_support_no_txampdu() &&
	    cfg80211_get_chandef_type(&sip->epub->hw->conf.chandef) != NL80211_CHAN_NO_HT) {
		wh = (struct ieee80211_hdr *)skb->data;

		if (ieee80211_is_data_qos(wh->frame_control) && !(IEEE80211_SKB_CB(skb)->flags & IEEE80211_TX_CTL_AMPDU)) {
			tidno = ieee80211_get_qos_ctl(wh)[0] &
				IEEE80211_QOS_CTL_TID_MASK;

			node = esp_get_node_by_addr(sip->epub, wh->addr1);
			if (!node || !node->sta)
				goto _exit;

			sta = node->sta;
			tid = &node->tid[tidno];
			spin_lock_bh(&sip->epub->tx_ampdu_lock);

			//start session
			if (!tid) {
				spin_unlock_bh(&sip->epub->tx_ampdu_lock);
				ESSERT(0);
				goto _exit;
			}

			if (tid->state == ESP_TID_STATE_INIT &&
			    TID_TO_AC(tidno) != WME_AC_VO && tid->cnt >= 10) {
				tid->state = ESP_TID_STATE_TRIGGER;
				dev_dbg(sip->epub->dev,
					"start tx ba session,addr:%pM,tid:%u\n",
					wh->addr1, tidno);
				spin_unlock_bh(&sip->epub->tx_ampdu_lock);
				ieee80211_start_tx_ba_session(sta, tidno, 0);
			} else {
				if (tid->state == ESP_TID_STATE_INIT)
					tid->cnt++;
				else
					tid->cnt = 0;

				spin_unlock_bh(&sip->epub->tx_ampdu_lock);
			}
		}
	}

_exit:
	ieee80211_tx_status(sip->epub->hw, skb);
}

/*  NB: this routine should be locked when calling
 */
void sip_txq_process(struct esp_pub *epub)
{
	struct sk_buff *skb;
	struct sip_hdr *hdr;
	struct esp_sip *sip = epub->sip;
	struct ieee80211_tx_info *itx_info;
	u32 pkt_len, tx_len = 0;
	int blknum = 0, pm_state = 0;
	bool queued_back = false, out_of_credits = false;

	while ((skb = skb_dequeue(&epub->txq))) {
		/* cmd skb->len does not include sip_hdr too */
		pkt_len = skb->len;
		itx_info = IEEE80211_SKB_CB(skb);
		if (itx_info->flags != 0xffffffff) {
			pkt_len += roundup(sizeof(struct sip_hdr), 4);
			if (itx_info->flags & IEEE80211_TX_CTL_AMPDU)
				pkt_len += roundup(sizeof(struct esp_tx_ampdu_entry),
						   4);
		}

		/* current design simply requires every sip_hdr must be at the begin of mblk, that definitely
		 * need to be optimized, e.g. calculate remain length in the previous mblk, if it larger than
		 * certain threshold (e.g, whole pkt or > 50% of pkt or 2 x sizeof(struct sip_hdr), append pkt
		 * to the previous mblk.  This might be done in sip_pack_pkt()
		 */
		pkt_len = roundup(pkt_len, sip->tx_blksz);
		blknum = pkt_len / sip->tx_blksz;

		/*
		 * FIXME: Empirical delay. Without this delay, the connection to
		 * a WiFi network crashes the kernel (sometimes at the second
		 * connection).
		 */
		udelay(2000);

		if (unlikely(atomic_read(&sip->credit_status) == RECALC_CREDIT_ENABLE)) {	/* need recalc credit */
			hdr = (struct sip_hdr *)skb->data;
			itx_info = IEEE80211_SKB_CB(skb);

			if (!(itx_info->flags == 0xffffffff &&
			      SIP_HDR_GET_TYPE(hdr->fc[0]) == SIP_CTRL &&
			      hdr->c_cmdid == SIP_CMD_RECALC_CREDIT &&
			      blknum <= atomic_read(&sip->tx_credits) - sip->credit_to_reserve)) {	/* except cmd recalc credit */
				dev_dbg(epub->dev, "recalc credits!\n");
				STRACE_TX_OUT_OF_CREDIT_INC();
				queued_back = true;
				out_of_credits = true;
				break;
			}
		} else {
			if (unlikely(blknum > (atomic_read(&sip->tx_credits) - sip->credit_to_reserve - SIP_CTRL_CREDIT_RESERVE))) {
				itx_info = IEEE80211_SKB_CB(skb);
				if (itx_info->flags == 0xffffffff) {	/* priv ctrl pkt */
					if (blknum > atomic_read(&sip->tx_credits) - sip->credit_to_reserve) {
						dev_dbg(epub->dev,
							"%s cmd pkt out of credits!\n",
							__func__);
						STRACE_TX_OUT_OF_CREDIT_INC();
						queued_back = true;
						out_of_credits = true;
						break;
					}
				} else {
					dev_dbg(epub->dev,
						"%s out of credits!\n",
						__func__);
					STRACE_TX_OUT_OF_CREDIT_INC();
					queued_back = true;
					out_of_credits = true;
					break;
				}
			}
		}

		tx_len += pkt_len;
		if (tx_len >= SIP_TX_AGGR_BUF_SIZE) {
			/* do we need to have limitation likemax 8 pkts in a row? */
			dev_dbg(epub->dev, "%s too much pkts in one shot!\n",
				__func__);
			STRACE_TX_ONE_SHOT_INC();
			tx_len -= pkt_len;
			queued_back = true;
			break;
		}

		if (sip_pack_pkt(sip, skb, &pm_state) != 0) {
			/* wrong pkt, won't send to target */
			tx_len -= pkt_len;
			continue;
		}

		atomic_sub(blknum, &sip->tx_credits);
		/*
		 * FIXME: Empirical delay. Without this delay, the connection to
		 * a WiFi network crashes the kernel (sometimes at the second
		 * connection).
		 */
		udelay(2000);

	}

	if (queued_back)
		skb_queue_head(&epub->txq, skb);

	if (atomic_read(&sip->state) == SIP_STOP
#ifdef HOST_RESET_BUG
	    || atomic_read(&epub->wl.off) == 1
#endif
	   ) {
		queued_back = 1;
		tx_len = 0;
	}

	if (tx_len)
		sip_write_pkts(sip, pm_state);

	if (queued_back && !out_of_credits)
		/* skb pending, do async process again */
		sip_trigger_txq_process(sip);
}

#ifndef NO_WMM_DUMMY
static struct esp_80211_wmm_param_element esp_wmm_param = {
	.oui = {0x00, 0x50, 0xf2},
	.oui_type = 0x02,
	.oui_subtype = 0x01,
	.version = 0x01,
	.qos_info = 0x00,
	.reserved = 0x00,
	.ac = {
		{
			.aci_aifsn = 0x03,
			.cw = 0xa4,
			.txop_limit = 0x0000,
		},
		{
			.aci_aifsn = 0x27,
			.cw = 0xa4,
			.txop_limit = 0x0000,
		},
		{
			.aci_aifsn = 0x42,
			.cw = 0x43,
			.txop_limit = 0x005e,
		},
		{
			.aci_aifsn = 0x62,
			.cw = 0x32,
			.txop_limit = 0x002f,
		},
	},
};

static int esp_add_wmm(struct sk_buff *skb)
{
	u8 *p;
	int flag = 0;
	int remain_len;
	int base_len;
	int len;
	struct ieee80211_mgmt *mgmt;
	struct ieee80211_hdr *wh;

	if (!skb)
		return -1;

	wh = (struct ieee80211_hdr *)skb->data;
	mgmt = (struct ieee80211_mgmt *)((u8 *)skb->data);

	if (ieee80211_is_assoc_resp(wh->frame_control)) {
		p = mgmt->u.assoc_resp.variable;
		base_len = (u8 *)mgmt->u.assoc_resp.variable - (u8 *)mgmt;
	} else if (ieee80211_is_reassoc_resp(wh->frame_control)) {
		p = mgmt->u.reassoc_resp.variable;
		base_len = (u8 *)mgmt->u.reassoc_resp.variable - (u8 *)mgmt;
	} else if (ieee80211_is_probe_resp(wh->frame_control)) {
		p = mgmt->u.probe_resp.variable;
		base_len = (u8 *)mgmt->u.probe_resp.variable - (u8 *)mgmt;
	} else if (ieee80211_is_beacon(wh->frame_control)) {
		p = mgmt->u.beacon.variable;
		base_len = (u8 *)mgmt->u.beacon.variable - (u8 *)mgmt;
	} else {
		return 1;
	}

	remain_len = skb->len - base_len;

	while (remain_len > 0) {
		if (*p == 0xdd && *(p + 5) == 0x02)	//wmm type
			return 0;
		else if (*p == 0x2d)	//has ht cap
			flag = 1;

		len = *(++p);
		p += (len + 1);
		remain_len -= (len + 2);
	}

	if (remain_len < 0)
		return -2;

	if (flag == 1) {
		skb_put(skb, 2 + sizeof(esp_wmm_param));

		memset(p, 0xdd, sizeof(u8));
		memset(p + 1, sizeof(esp_wmm_param), sizeof(u8));
		memcpy(p + 2, &esp_wmm_param, sizeof(esp_wmm_param));
	}

	return 0;
}
#endif				/* NO_WMM_DUMMY */

/*  parse mac_rx_ctrl and return length */
static int sip_parse_mac_rx_info(struct esp_sip *sip,
				 struct esp_mac_rx_ctrl *mac_ctrl,
				 struct sk_buff *skb)
{
	struct ieee80211_rx_status *rx_status;
	struct ieee80211_hdr *hdr;
	struct ieee80211_hdr *wh;

	rx_status = IEEE80211_SKB_RXCB(skb);
	rx_status->freq = esp_ieee2mhz(mac_ctrl->channel);
	rx_status->signal = mac_ctrl->rssi + mac_ctrl->noise_floor;	/* snr actually, need to offset noise floor e.g. -85 */

	hdr = (struct ieee80211_hdr *)skb->data;
	if (mac_ctrl->damatch0 == 1 && mac_ctrl->bssidmatch0 == 1 &&	/*match bssid and da, but beacon package contain other bssid */
	    !strncmp(hdr->addr2, sip->epub->wl.bssid, ETH_ALEN)) {	/* force match addr2 */
		if (++signal_loop >= SIGNAL_COUNT) {
			avg_signal += rx_status->signal;
			avg_signal /= SIGNAL_COUNT;
			rx_status->signal = avg_signal + 5;
			old_signal = rx_status->signal;
			signal_loop = 0;
			avg_signal = 0;
		} else {
			avg_signal += rx_status->signal;
			rx_status->signal = old_signal;
		}
	}

	rx_status->antenna = 0;	/* one antenna for now */
	rx_status->band = NL80211_BAND_2GHZ;
	rx_status->flag = RX_FLAG_DECRYPTED | RX_FLAG_MMIC_STRIPPED;
	if (mac_ctrl->sig_mode) {
		rx_status->enc_flags |= RX_ENC_FLAG_HT_GF;
		rx_status->rate_idx = mac_ctrl->MCS;
		if (mac_ctrl->SGI)
			rx_status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
	} else {
		rx_status->rate_idx = esp_wmac_rate2idx(mac_ctrl->rate);
	}

	if (mac_ctrl->rxend_state == RX_FCS_ERR)
		rx_status->flag |= RX_FLAG_FAILED_FCS_CRC;

	/* Mic error frame flag */
	if (mac_ctrl->rxend_state == RX_TKIPMIC_ERR ||
	    mac_ctrl->rxend_state == RX_CCMPMIC_ERR) {
		if (atomic_read(&sip->epub->wl.tkip_key_set) == 1) {
			rx_status->flag |= RX_FLAG_MMIC_ERROR;
			atomic_set(&sip->epub->wl.tkip_key_set, 0);
			printk("mic err\n");
		} else {
			printk("mic err discard\n");
		}
	}

	wh = (struct ieee80211_hdr *)((u8 *)skb->data);

#ifndef NO_WMM_DUMMY
	if (ieee80211_is_mgmt(wh->frame_control))
		esp_add_wmm(skb);
#endif

	/* some kernel e.g. 3.0.8 wrongly handles non-encrypted pkt like eapol */
	if (ieee80211_is_data(wh->frame_control)) {
		if (!ieee80211_has_protected(wh->frame_control))
			rx_status->flag |= RX_FLAG_IV_STRIPPED;
		else if (!atomic_read(&sip->epub->wl.ptk_cnt))
				rx_status->flag |= RX_FLAG_IV_STRIPPED;
	}

	return 0;
}

static struct esp_mac_rx_ctrl *sip_parse_normal_mac_ctrl(struct sk_buff *skb,
							 int *pkt_len_enc,
							 int *buf_len,
							 int *pulled_len)
{
	struct sip_hdr *hdr = (struct sip_hdr *)skb->data;
	struct esp_mac_rx_ctrl *mac_ctrl;
	int len_in_hdr = hdr->len;

	ESSERT(skb);
	ESSERT(skb->len > SIP_MIN_DATA_PKT_LEN);

	skb_pull(skb, sizeof(struct sip_hdr));
	*pulled_len += sizeof(struct sip_hdr);
	mac_ctrl = (struct esp_mac_rx_ctrl *)skb->data;
	if (!mac_ctrl->aggregation) {
		ESSERT(pkt_len_enc);
		ESSERT(buf_len);
		*pkt_len_enc = (mac_ctrl->sig_mode ? mac_ctrl->HT_length : mac_ctrl->legacy_length) - FCS_LEN;
		*buf_len = len_in_hdr - sizeof(struct sip_hdr) -
			sizeof(struct esp_mac_rx_ctrl);
	}

	skb_pull(skb, sizeof(struct esp_mac_rx_ctrl));
	*pulled_len += sizeof(struct esp_mac_rx_ctrl);

	return mac_ctrl;
}

/* for one MPDU (including subframe in AMPDU) */
static struct sk_buff *sip_parse_data_rx_info(struct esp_sip *sip,
					      struct sk_buff *skb,
					      int pkt_len_enc, int buf_len,
					      struct esp_mac_rx_ctrl *mac_ctrl,
					      int *pulled_len)
{
	/*   | mac_rx_ctrl | real_data_payload | ampdu_entries | */
	struct sk_buff *rskb;
	struct ieee80211_hdr *wh;
	int pkt_len, ret;

	if (mac_ctrl->aggregation) {
		wh = (struct ieee80211_hdr *)skb->data;
		pkt_len = pkt_len_enc;
		if (ieee80211_has_protected(wh->frame_control))	//ampdu, it is CCMP enc
			pkt_len -= 8;

		buf_len = roundup(pkt_len, 4);
	} else {
		pkt_len = buf_len - 3 + ((pkt_len_enc - 1) & 0x3);
	}

#ifndef NO_WMM_DUMMY
	rskb = __dev_alloc_skb(pkt_len_enc + sizeof(esp_wmm_param) + 2,
			       GFP_ATOMIC);
#else
	rskb = __dev_alloc_skb(pkt_len_enc, GFP_ATOMIC);
#endif				/* NO_WMM_DUMMY */
	if (unlikely(!rskb)) {
		dev_err(sip->epub->dev, "no mem for rskb\n");
		return NULL;
	}

	skb_put(rskb, pkt_len_enc);

	memcpy(rskb->data, skb->data, pkt_len);

	if (pkt_len_enc > pkt_len)
		memset(rskb->data + pkt_len, 0, pkt_len_enc - pkt_len);

	/* strip out current pkt, move to the next one */
	skb_pull(skb, buf_len);
	*pulled_len += buf_len;

	ret = sip_parse_mac_rx_info(sip, mac_ctrl, rskb);
	if (ret == -1 && !mac_ctrl->aggregation) {
		kfree_skb(rskb);
		return NULL;
	}

	return rskb;
}

struct esp_sip *sip_attach(struct esp_pub *epub)
{
	struct esp_sip *sip;
	struct sip_pkt *pkt;
	int i, po;

	sip = kzalloc(sizeof(*sip), GFP_KERNEL);
	if (!sip) {
		dev_dbg(epub->dev, "no mem for sip!\n");
		goto _err_sip;
	}

	po = get_order(SIP_TX_AGGR_BUF_SIZE);

	sip->tx_aggr_buf = (u8 *)__get_free_pages(GFP_ATOMIC, po);
	if (!sip->tx_aggr_buf) {
		dev_err(epub->dev, "no mem for tx_aggr_buf!\n");
		goto _err_aggr;
	}

	spin_lock_init(&sip->lock);

	INIT_LIST_HEAD(&sip->free_ctrl_txbuf);
	INIT_LIST_HEAD(&sip->free_ctrl_rxbuf);

	for (i = 0; i < SIP_CTRL_BUF_N; i++) {
		pkt = kzalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			goto _err_pkt;

		pkt->buf_begin = kzalloc(SIP_CTRL_BUF_SZ, GFP_KERNEL);
		if (!pkt->buf_begin) {
			kfree(pkt);
			pkt = NULL;
			goto _err_pkt;
		}

		pkt->buf_len = SIP_CTRL_BUF_SZ;
		pkt->buf = pkt->buf_begin;

		if (i < SIP_CTRL_TXBUF_N)
			list_add_tail(&pkt->list, &sip->free_ctrl_txbuf);
		else
			list_add_tail(&pkt->list, &sip->free_ctrl_rxbuf);
	}

	mutex_init(&sip->rx_mtx);
	skb_queue_head_init(&sip->rxq);
	INIT_WORK(&sip->rx_process_work, sip_rxq_process);

	sip->epub = epub;
	atomic_set(&sip->noise_floor, -96);

	atomic_set(&sip->state, SIP_INIT);
	atomic_set(&sip->tx_credits, 0);

	if (!sip->rawbuf) {
		sip->rawbuf = kzalloc(SIP_BOOT_BUF_SIZE, GFP_KERNEL);
		if (!sip->rawbuf) {
			dev_err(epub->dev, "no mem for rawbuf!\n");
			goto _err_pkt;
		}
	}

	atomic_set(&sip->state, SIP_PREPARE_BOOT);

	return sip;

_err_pkt:
	sip_free_init_ctrl_buf(sip);

	if (sip->tx_aggr_buf) {
		po = get_order(SIP_TX_AGGR_BUF_SIZE);
		free_pages((unsigned long)sip->tx_aggr_buf, po);
		sip->tx_aggr_buf = NULL;
	}

_err_aggr:
	kfree(sip);
	sip = NULL;

_err_sip:
	return NULL;
}

static void sip_free_init_ctrl_buf(struct esp_sip *sip)
{
	struct sip_pkt *pkt, *tpkt;

	list_for_each_entry_safe(pkt, tpkt, &sip->free_ctrl_txbuf, list) {
		list_del(&pkt->list);
		kfree(pkt->buf_begin);
		kfree(pkt);
	}

	list_for_each_entry_safe(pkt, tpkt, &sip->free_ctrl_rxbuf, list) {
		list_del(&pkt->list);
		kfree(pkt->buf_begin);
		kfree(pkt);
	}
}

void sip_detach(struct esp_sip *sip)
{
	if (!sip)
		return;

	sip_free_init_ctrl_buf(sip);

	if (atomic_read(&sip->state) == SIP_RUN) {
		sif_disable_target_interrupt(sip->epub);

		atomic_set(&sip->state, SIP_STOP);

		/* disable irq here */
		sif_disable_irq(sip->epub);
		cancel_work_sync(&sip->rx_process_work);

		skb_queue_purge(&sip->rxq);
		mutex_destroy(&sip->rx_mtx);
		cancel_work_sync(&sip->epub->sendup_work);
		skb_queue_purge(&sip->epub->rxq);

		if (test_and_clear_bit(ESP_WL_FLAG_HW_REGISTERED,
				       &sip->epub->wl.flags))
			ieee80211_unregister_hw(sip->epub->hw);

		/* cancel all worker/timer */
		cancel_work_sync(&sip->epub->tx_work);
		skb_queue_purge(&sip->epub->txq);
		skb_queue_purge(&sip->epub->txdoneq);

		free_pages((unsigned long)sip->tx_aggr_buf,
			   get_order(SIP_TX_AGGR_BUF_SIZE));
		sip->tx_aggr_buf = NULL;

		atomic_set(&sip->state, SIP_INIT);
	} else if (atomic_read(&sip->state) >= SIP_BOOT &&
		   atomic_read(&sip->state) <= SIP_WAIT_BOOTUP) {
		sif_disable_target_interrupt(sip->epub);
		atomic_set(&sip->state, SIP_STOP);
		sif_disable_irq(sip->epub);
		kfree(sip->rawbuf);

		if (atomic_read(&sip->state) == SIP_SEND_INIT) {
			cancel_work_sync(&sip->rx_process_work);
			skb_queue_purge(&sip->rxq);
			mutex_destroy(&sip->rx_mtx);
			cancel_work_sync(&sip->epub->sendup_work);
			skb_queue_purge(&sip->epub->rxq);
		}

		if (test_and_clear_bit(ESP_WL_FLAG_HW_REGISTERED,
				       &sip->epub->wl.flags))
			ieee80211_unregister_hw(sip->epub->hw);
		atomic_set(&sip->state, SIP_INIT);
	} else {
		dev_err(sip->epub->dev, "wrong state (%d) for detaching sip\n",
			atomic_read(&sip->state));
	}

	kfree(sip);
}

int sip_write_memory(struct esp_sip *sip, u32 addr, u8 *buf, u16 len)
{
	struct sip_cmd_write_memory *cmd;
	struct sip_hdr *chdr;
	u16 remains, hdrs, bufsize;
	u32 loadaddr;
	u8 *src;
	int err = 0;
	u32 *t;

	if (!sip || !sip->rawbuf) {
		ESSERT(sip);
		ESSERT(sip->rawbuf);
		return -EINVAL;
	}

	memset(sip->rawbuf, 0, SIP_BOOT_BUF_SIZE);

	chdr = (struct sip_hdr *)sip->rawbuf;
	SIP_HDR_SET_TYPE(chdr->fc[0], SIP_CTRL);
	chdr->c_cmdid = SIP_CMD_WRITE_MEMORY;

	remains = len;
	hdrs = sizeof(struct sip_hdr) + sizeof(struct sip_cmd_write_memory);

	while (remains) {
		src = &buf[len - remains];
		loadaddr = addr + (len - remains);

		if (remains < (SIP_BOOT_BUF_SIZE - hdrs)) {
			/* aligned with 4 bytes */
			bufsize = roundup(remains, 4);
			memset(sip->rawbuf + hdrs, 0, bufsize);
			remains = 0;
		} else {
			bufsize = SIP_BOOT_BUF_SIZE - hdrs;
			remains -= bufsize;
		}

		chdr->len = bufsize + hdrs;
		chdr->seq = sip->txseq++;
		cmd = (struct sip_cmd_write_memory *)(sip->rawbuf + SIP_CTRL_HDR_LEN);
		cmd->len = bufsize;
		cmd->addr = loadaddr;
		memcpy(sip->rawbuf + hdrs, src, bufsize);

		t = (u32 *)sip->rawbuf;
		err = esp_common_write(sip->epub, sip->rawbuf, chdr->len,
				       ESP_SIF_SYNC);
		if (err) {
			dev_err(sip->epub->dev, "send buffer failed\n");
			return err;
		}
		// 1ms is enough, in fact on dell-d430, need not delay at all.
		mdelay(1);
	}

	return err;
}

int sip_send_cmd(struct esp_sip *sip, int cid, u32 cmdlen, void *cmd)
{
	struct sip_hdr *chdr;
	struct sip_pkt *pkt;
	int ret;

	pkt = sip_get_ctrl_buf(sip, SIP_TX_CTRL_BUF);
	if (!pkt)
		return -ENOMEM;

	chdr = (struct sip_hdr *)pkt->buf_begin;
	chdr->len = SIP_CTRL_HDR_LEN + cmdlen;
	chdr->seq = sip->txseq++;
	chdr->c_cmdid = cid;

	if (cmd) {
		memset(pkt->buf, 0, cmdlen);
		memcpy(pkt->buf, (u8 *)cmd, cmdlen);
	}

	ret = esp_common_write(sip->epub, pkt->buf_begin, chdr->len,
			       ESP_SIF_SYNC);
	if (ret)
		dev_err(sip->epub->dev, "send cmd %d failed\n", cid);

	sip_reclaim_ctrl_buf(sip, pkt, SIP_TX_CTRL_BUF);

	/*  Hack here: reset tx/rx seq before target ram code is up... */
	if (cid == SIP_CMD_BOOTUP) {
		sip->rxseq = 0;
		sip->txseq = 0;
		sip->txdataseq = 0;
	}

	return ret;
}

struct sk_buff *sip_alloc_ctrl_skbuf(struct esp_sip *sip, u16 len, u32 cid)
{
	struct sip_hdr *si;
	struct ieee80211_tx_info *ti;
	struct sk_buff *skb;

	ESSERT(len <= sip->tx_blksz);

	/* no need to reserve space for net stack */
	skb = __dev_alloc_skb(len, GFP_KERNEL);
	if (!skb) {
		dev_err(sip->epub->dev, "no skb for ctrl!\n");
		return NULL;
	}

	skb->len = len;

	ti = IEEE80211_SKB_CB(skb);
	/* set tx_info flags to 0xffffffff to indicate sip_ctrl pkt */
	ti->flags = 0xffffffff;

	si = (struct sip_hdr *)skb->data;
	memset(si, 0, sizeof(struct sip_hdr));
	SIP_HDR_SET_TYPE(si->fc[0], SIP_CTRL);
	si->len = len;
	si->c_cmdid = cid;

	return skb;
}

void sip_free_ctrl_skbuff(struct esp_sip *sip, struct sk_buff *skb)
{
	memset(IEEE80211_SKB_CB(skb), 0, sizeof(struct ieee80211_tx_info));
	kfree_skb(skb);
}

static struct sip_pkt *sip_get_ctrl_buf(struct esp_sip *sip,
					enum sip_buf_type bftype)
{
	struct sip_pkt *pkt;
	struct list_head *bflist;
	struct sip_hdr *chdr;

	/* FIXME: Why taking spinlock to check list_empty? */
	bflist = (bftype == SIP_TX_CTRL_BUF) ? &sip->free_ctrl_txbuf : &sip->free_ctrl_rxbuf;

	spin_lock_bh(&sip->lock);

	if (list_empty(bflist)) {
		spin_unlock_bh(&sip->lock);
		return NULL;
	}

	pkt = list_first_entry(bflist, struct sip_pkt, list);
	list_del(&pkt->list);
	spin_unlock_bh(&sip->lock);

	if (bftype == SIP_TX_CTRL_BUF) {
		chdr = (struct sip_hdr *)pkt->buf_begin;
		SIP_HDR_SET_TYPE(chdr->fc[0], SIP_CTRL);
		pkt->buf = pkt->buf_begin + SIP_CTRL_HDR_LEN;
	} else {
		pkt->buf = pkt->buf_begin;
	}

	return pkt;
}

static void sip_reclaim_ctrl_buf(struct esp_sip *sip, struct sip_pkt *pkt,
				 enum sip_buf_type bftype)
{
	struct list_head *bflist;

	if (bftype == SIP_TX_CTRL_BUF)
		bflist = &sip->free_ctrl_txbuf;
	else if (bftype == SIP_RX_CTRL_BUF)
		bflist = &sip->free_ctrl_rxbuf;
	else
		return;

	pkt->buf = pkt->buf_begin;

	spin_lock_bh(&sip->lock);
	list_add_tail(&pkt->list, bflist);
	spin_unlock_bh(&sip->lock);
}

int sip_poll_bootup_event(struct esp_sip *sip)
{
	int ret = 0;

	if (gl_bootup_cplx)
		ret = wait_for_completion_timeout(gl_bootup_cplx, 2 * HZ);

	if (!ret) {
		dev_err(sip->epub->dev, "bootup event timeout\n");
		return -ETIMEDOUT;
	}

	if (!sif_get_ate_config())
		ret = esp_register_mac80211(sip->epub);

	atomic_set(&sip->state, SIP_RUN);

	return ret;
}

/* FIXME: always returning 0 ? */
int sip_poll_resetting_event(struct esp_sip *sip)
{
	unsigned int ret;

	if (gl_bootup_cplx)
		ret = wait_for_completion_timeout(gl_bootup_cplx, 10 * HZ);

	if (!ret) {
		dev_err(sip->epub->dev, "resetting event timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

bool sip_queue_need_stop(struct esp_sip *sip)
{
	return atomic_read(&sip->tx_data_pkt_queued) >= SIP_STOP_QUEUE_THRESHOLD ||
		(atomic_read(&sip->tx_credits) < 8 &&
		 atomic_read(&sip->tx_data_pkt_queued) >= SIP_STOP_QUEUE_THRESHOLD / 4 * 3);
}

bool sip_queue_may_resume(struct esp_sip *sip)
{
	return atomic_read(&sip->epub->txq_stopped) &&
		!test_bit(ESP_WL_FLAG_STOP_TXQ, &sip->epub->wl.flags) &&
		((atomic_read(&sip->tx_credits) >= 16 &&
		  atomic_read(&sip->tx_data_pkt_queued) < SIP_RESUME_QUEUE_THRESHOLD * 2) ||
		 atomic_read(&sip->tx_data_pkt_queued) < SIP_RESUME_QUEUE_THRESHOLD);
}

int sip_cmd_enqueue(struct esp_sip *sip, struct sk_buff *skb, int prior)
{
	if (!sip || !sip->epub || !skb) {
		return -EINVAL;
	}

	if (prior == ENQUEUE_PRIOR_HEAD)
		skb_queue_head(&sip->epub->txq, skb);
	else
		skb_queue_tail(&sip->epub->txq, skb);

	if (!sif_get_ate_config())
		ieee80211_queue_work(sip->epub->hw, &sip->epub->tx_work);
	else
		queue_work(sip->epub->esp_wkq, &sip->epub->tx_work);

	return 0;
}

void sip_tx_data_pkt_enqueue(struct esp_pub *epub, struct sk_buff *skb)
{
	if (!epub || !epub->sip || !skb)
		return;

	skb_queue_tail(&epub->txq, skb);
	atomic_inc(&epub->sip->tx_data_pkt_queued);

	if (sip_queue_need_stop(epub->sip))
		if (epub->hw) {
			ieee80211_stop_queues(epub->hw);
			atomic_set(&epub->txq_stopped, true);
		}
}
