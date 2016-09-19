/*
 * Copyright (c) 2010 - 2014 Espressif System.
 *
 * main routine
 */

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <net/cfg80211.h>
#include <net/mac80211.h>
#include <linux/time.h>
#include <linux/moduleparam.h>

#include "esp_pub.h"
#include "esp_sip.h"
#include "esp_sif.h"
#include "esp_debug.h"
#include "esp_file.h"
#include "esp_wl.h"

struct completion *gl_bootup_cplx;

static int esp_download_fw(struct esp_pub *epub);

static int modparam_no_txampdu;
static int modparam_no_rxampdu;

module_param_named(no_txampdu, modparam_no_txampdu, int, 0444);
MODULE_PARM_DESC(no_txampdu, "Disable tx ampdu.");

module_param_named(no_rxampdu, modparam_no_rxampdu, int, 0444);
MODULE_PARM_DESC(no_rxampdu, "Disable rx ampdu.");

static char *modparam_eagle_path = "/lib/firmware";

module_param_named(eagle_path, modparam_eagle_path, charp, 0444);
MODULE_PARM_DESC(eagle_path, "eagle path");

bool mod_support_no_txampdu(void)
{
	return modparam_no_txampdu;
}

bool mod_support_no_rxampdu(void)
{
	return modparam_no_rxampdu;
}

int esp_pub_init_all(struct esp_pub *epub)
{
	int ret;

	/* completion for bootup event poll */
	DECLARE_COMPLETION_ONSTACK(complete);

	atomic_set(&epub->ps.state, ESP_PM_OFF);

	if (epub->sdio_state == ESP_SDIO_STATE_FIRST_INIT) {
		epub->sip = sip_attach(epub);
		if (!epub->sip) {
			printk(KERN_ERR "%s sip alloc failed\n", __func__);
			return -ENOMEM;
		}
	} else {
		atomic_set(&epub->sip->state, SIP_PREPARE_BOOT);
		atomic_set(&epub->sip->tx_credits, 0);
	}

	epub->sip->to_host_seq = 0;

	ret = esp_download_fw(epub);
	if (ret) {
		printk("download firmware failed\n");
		return ret;
	}

	gl_bootup_cplx = &complete;
	epub->wait_reset = 0;
	sif_enable_irq(epub);

	if (epub->sdio_state == ESP_SDIO_STATE_SECOND_INIT ||
	    sif_get_ate_config() == 1) {
		ret = sip_poll_bootup_event(epub->sip);
	} else {
		ret = sip_poll_resetting_event(epub->sip);
		if (!ret) {
			sif_lock_bus(epub);
			sif_interrupt_target(epub, 7);
			sif_unlock_bus(epub);
		}
	}

	gl_bootup_cplx = NULL;

	if (sif_get_ate_config() == 1)
		ret = -EOPNOTSUPP;

	return ret;
}

void esp_dsr(struct esp_pub *epub)
{
	sip_rx(epub);
}

struct esp_fw_hdr {
	u8 magic;
	u8 blocks;
	u8 pad[2];
	u32 entry_addr;
} __packed;

struct esp_fw_blk_hdr {
	u32 load_addr;
	u32 data_len;
} __packed;

#define ESP_FW_NAME1 "eagle_fw_ate_config_v19.bin"
#define ESP_FW_NAME2 "eagle_fw_first_init_v19.bin"
#define ESP_FW_NAME3 "eagle_fw_second_init_v19.bin"

static int esp_download_fw(struct esp_pub *epub)
{
	const struct firmware *fw_entry;
	struct esp_fw_hdr *fhdr;
	struct esp_fw_blk_hdr *bhdr;
	struct sip_cmd_bootup bootcmd;
	u8 *fw_buf;
	u32 offset;
	int ret;
	u8 blocks;
	char *esp_fw_name;

	if (sif_get_ate_config() == 1)
		esp_fw_name = ESP_FW_NAME3;
	else
		esp_fw_name = epub->sdio_state == ESP_SDIO_STATE_FIRST_INIT ? ESP_FW_NAME1 : ESP_FW_NAME2;

	ret = request_firmware(&fw_entry, esp_fw_name, epub->dev);
	if (ret)
		return ret;

	fw_buf = kmemdup(fw_entry->data, fw_entry->size, GFP_KERNEL);

	release_firmware(fw_entry);

	if (!fw_buf)
		return -ENOMEM;

	fhdr = (struct esp_fw_hdr *)fw_buf;
	if (fhdr->magic != 0xE9) {
		printk("%s wrong firmware magic!\n", __func__);
		goto _err;
	}

	blocks = fhdr->blocks;
	offset = sizeof(struct esp_fw_hdr);

	while (blocks) {
		bhdr = (struct esp_fw_blk_hdr *)&fw_buf[offset];
		offset += sizeof(struct esp_fw_blk_hdr);

		ret = sip_write_memory(epub->sip, bhdr->load_addr,
				       &fw_buf[offset], bhdr->data_len);
		if (ret) {
			printk("failed to write firmware: %d\n", ret);
			goto _err;
		}

		blocks--;
		offset += bhdr->data_len;
	}

	/* TODO: last byte should be the checksum and skip checksum for now */

	bootcmd.boot_addr = fhdr->entry_addr;
	ret = sip_send_cmd(epub->sip, SIP_CMD_BOOTUP,
			   sizeof(struct sip_cmd_bootup), &bootcmd);
	if (ret)
		goto _err;

_err:
	kfree(fw_buf);

	return ret;
}

MODULE_FIRMWARE(ESP_FW_NAME1);
MODULE_FIRMWARE(ESP_FW_NAME2);
MODULE_FIRMWARE(ESP_FW_NAME3);
