/*
 * Copyright (c) 2009 - 2014 Espressif System.
 *   IO interface
 *    - sdio/spi common i/f driver
 *    - target sdio hal
 */

#include <linux/mmc/sdio_func.h>
#include "esp_sif.h"
#include "slc_host_register.h"
#include "esp_debug.h"

int esp_common_read(struct esp_pub *epub, u8 *buf, u32 len, int sync,
		    bool noround)
{
	if (sync)
		return sif_lldesc_read_sync(epub, buf, len);

	return sif_lldesc_read_raw(epub, buf, len, noround);
}

int esp_common_write(struct esp_pub *epub, u8 *buf, u32 len, int sync)
{
	if (sync)
		return sif_lldesc_write_sync(epub, buf, len);

	return sif_lldesc_write_raw(epub, buf, len);
}

int esp_common_read_with_addr(struct esp_pub *epub, u32 addr, u8 *buf, u32 len,
			      int sync)
{
	if (sync)
		return sif_io_sync(epub, addr, buf, len, SIF_FROM_DEVICE |
				   SIF_SYNC | SIF_BYTE_BASIS | SIF_INC_ADDR);

	return sif_io_raw(epub, addr, buf, len, SIF_FROM_DEVICE |
			  SIF_BYTE_BASIS | SIF_INC_ADDR);
}

int esp_common_write_with_addr(struct esp_pub *epub, u32 addr, u8 *buf,
			       u32 len, int sync)
{
	if (sync)
		return sif_io_sync(epub, addr, buf, len, SIF_TO_DEVICE |
				   SIF_SYNC | SIF_BYTE_BASIS | SIF_INC_ADDR);

	return sif_io_raw(epub, addr, buf, len, SIF_TO_DEVICE | SIF_BYTE_BASIS |
			  SIF_INC_ADDR);
}

int esp_common_readbyte_with_addr(struct esp_pub *epub, u32 addr, u8 *buf,
				  int sync)
{
	int res;

	if (sync)
		sif_lock_bus(epub);

	*buf = sdio_io_readb(epub, addr, &res);

	if (sync)
		sif_unlock_bus(epub);

	return res;
}

int esp_common_writebyte_with_addr(struct esp_pub *epub, u32 addr, u8 buf,
				   int sync)
{
	int res;

	if (sync)
		sif_lock_bus(epub);

	sdio_io_writeb(epub, buf, addr, &res);

	if (sync)
		sif_unlock_bus(epub);

	return res;
}

enum _SDIO_INTR_MODE {
	SDIO_INTR_IB = 0,
	SDIO_INTR_OOB_TOGGLE,
	SDIO_INTR_OOB_HIGH_LEVEL,
	SDIO_INTR_OOB_LOW_LEVEL,
};

#define GEN_GPIO_SEL(_gpio_num, _sel_func, _intr_mode, _offset) (((_offset) << 9) | ((_intr_mode) << 7) | ((_sel_func) << 4) | (_gpio_num))
//bit[3:0] = gpio num, 2
//bit[6:4] = gpio sel func, 0
//bit[8:7] = gpio intr mode, SDIO_INTR_OOB_TOGGLE
//bit[15:9] = register offset, 0x38

u16 gpio_sel_sets[17] = {
	GEN_GPIO_SEL(0, 0, SDIO_INTR_OOB_TOGGLE, 0x34),	//GPIO0
	GEN_GPIO_SEL(1, 3, SDIO_INTR_OOB_TOGGLE, 0x18),	//U0TXD
	GEN_GPIO_SEL(2, 0, SDIO_INTR_OOB_TOGGLE, 0x38),	//GPIO2
	GEN_GPIO_SEL(3, 3, SDIO_INTR_OOB_TOGGLE, 0x14),	//U0RXD
	GEN_GPIO_SEL(4, 0, SDIO_INTR_OOB_TOGGLE, 0x3C),	//GPIO4
	GEN_GPIO_SEL(5, 0, SDIO_INTR_OOB_TOGGLE, 0x40),	//GPIO5
	GEN_GPIO_SEL(6, 3, SDIO_INTR_OOB_TOGGLE, 0x1C),	//SD_CLK
	GEN_GPIO_SEL(7, 3, SDIO_INTR_OOB_TOGGLE, 0x20),	//SD_DATA0
	GEN_GPIO_SEL(8, 3, SDIO_INTR_OOB_TOGGLE, 0x24),	//SD_DATA1
	GEN_GPIO_SEL(9, 3, SDIO_INTR_OOB_TOGGLE, 0x28),	//SD_DATA2
	GEN_GPIO_SEL(10, 3, SDIO_INTR_OOB_TOGGLE, 0x2C),	//SD_DATA3
	GEN_GPIO_SEL(11, 3, SDIO_INTR_OOB_TOGGLE, 0x30),	//SD_CMD
	GEN_GPIO_SEL(12, 3, SDIO_INTR_OOB_TOGGLE, 0x04),	//MTDI
	GEN_GPIO_SEL(13, 3, SDIO_INTR_OOB_TOGGLE, 0x08),	//MTCK
	GEN_GPIO_SEL(14, 3, SDIO_INTR_OOB_TOGGLE, 0x0C),	//MTMS
	GEN_GPIO_SEL(15, 3, SDIO_INTR_OOB_TOGGLE, 0x10),	//MTDO
	//pls do not change sel before, if you want to change intr mode,change the one blow
	//GEN_GPIO_SEL(2, 0, SDIO_INTR_OOB_TOGGLE, 0x38)
	GEN_GPIO_SEL(2, 0, SDIO_INTR_OOB_LOW_LEVEL, 0x38)
};

int sif_interrupt_target(struct esp_pub *epub, u8 index)
{
	u8 low_byte = BIT(index);
	return esp_common_writebyte_with_addr(epub, SLC_HOST_CONF_W4 + 2,
					      low_byte, ESP_SIF_NOSYNC);
}

void check_target_id(struct esp_pub *epub)
{
	u32 date;
	int err, i;
	u16 gpio_sel;
	u8 low_byte = 0, high_byte = 0, byte2 = 0, byte3 = 0;

	EPUB_CTRL_CHECK(epub, _err);

	sif_lock_bus(epub);

	for (i = 0; i < 4; i++) {
		err = esp_common_readbyte_with_addr(epub, SLC_HOST_DATE + i,
						    (u8 *)&date + i,
						    ESP_SIF_NOSYNC);
		err = esp_common_readbyte_with_addr(epub, SLC_HOST_ID + i,
						    (u8 *)&EPUB_TO_CTRL(epub)->target_id + i,
						    ESP_SIF_NOSYNC);
	}

	sif_unlock_bus(epub);

	switch (EPUB_TO_CTRL(epub)->target_id) {
	case 0x100:
		EPUB_TO_CTRL(epub)->slc_window_end_addr = 0x20000;
		break;
	case 0x600:
		EPUB_TO_CTRL(epub)->slc_window_end_addr = 0x20000 - 0x800;

		if (sif_get_bt_config() == 1 && sif_get_rst_config() != 1) {
			u8 gpio_num = sif_get_wakeup_gpio_config();
			gpio_sel = gpio_sel_sets[gpio_num];
			byte2 = gpio_sel;
			byte3 = gpio_sel >> 8;
		}
		sif_lock_bus(epub);
		err = esp_common_writebyte_with_addr(epub, SLC_HOST_CONF_W1,
						     low_byte,
						     ESP_SIF_NOSYNC);
		err = esp_common_writebyte_with_addr(epub, SLC_HOST_CONF_W1 + 1,
						     high_byte,
						     ESP_SIF_NOSYNC);
		err = esp_common_writebyte_with_addr(epub, SLC_HOST_CONF_W1 + 2,
						     byte2,
						     ESP_SIF_NOSYNC);
		err = esp_common_writebyte_with_addr(epub, SLC_HOST_CONF_W1 + 3,
						     byte3,
						     ESP_SIF_NOSYNC);
		sif_unlock_bus(epub);
		break;
	default:
		EPUB_TO_CTRL(epub)->slc_window_end_addr = 0x20000;
		break;
	}
_err:
	return;
}

u32 sif_get_blksz(struct esp_pub *epub)
{
	EPUB_CTRL_CHECK(epub, _err);

	return EPUB_TO_CTRL(epub)->slc_blk_sz;

_err:
	return 512;
}

void sif_dsr(struct sdio_func *func)
{
	struct esp_sdio_ctrl *sctrl = sdio_get_drvdata(func);
	struct slc_host_regs *regs = &sctrl->slc_regs;
	int ret;

	sdio_release_host(sctrl->func);
	sif_lock_bus(sctrl->epub);
	memset(regs, 0x0, sizeof(struct slc_host_regs));

	ret = esp_common_read_with_addr(sctrl->epub, REG_SLC_HOST_BASE + 8,
					(u8 *)regs,
					sizeof(struct slc_host_regs),
					ESP_SIF_NOSYNC);

	if (regs->intr_raw & SLC_HOST_RX_ST && !ret)
		esp_dsr(sctrl->epub);
	else
		sif_unlock_bus(sctrl->epub);

	/* FIXME: missing unlock_bus? */
	sdio_claim_host(func);
	atomic_set(&sctrl->irq_handling, 0);
}

struct slc_host_regs *sif_get_regs(struct esp_pub *epub)
{
	EPUB_CTRL_CHECK(epub, _err);

	return &EPUB_TO_CTRL(epub)->slc_regs;

_err:
	return NULL;
}

void sif_disable_target_interrupt(struct esp_pub *epub)
{
	EPUB_FUNC_CHECK(epub, _exit);
	sif_lock_bus(epub);
#ifdef HOST_RESET_BUG
	mdelay(10);
#endif
	memset(EPUB_TO_CTRL(epub)->dma_buffer, 0x00, sizeof(u32));
	esp_common_write_with_addr(epub, SLC_HOST_INT_ENA,
				   EPUB_TO_CTRL(epub)->dma_buffer, sizeof(u32),
				   ESP_SIF_NOSYNC);
#ifdef HOST_RESET_BUG
	mdelay(10);
#endif

	sif_unlock_bus(epub);

	mdelay(1);

	sif_lock_bus(epub);
	sif_interrupt_target(epub, 7);
	sif_unlock_bus(epub);

_exit:
	return;
}

static int bt_config;
void sif_record_bt_config(int value)
{
	bt_config = value;
}

int sif_get_bt_config(void)
{
	return bt_config;
}

static int rst_config;
void sif_record_rst_config(int value)
{
	rst_config = value;
}

int sif_get_rst_config(void)
{
	return rst_config;
}

static int ate_test;
void sif_record_ate_config(int value)
{
	ate_test = value;
}

int sif_get_ate_config(void)
{
	return ate_test;
}

static int wakeup_gpio = 12;
void sif_record_wakeup_gpio_config(int value)
{
	wakeup_gpio = value;
}

int sif_get_wakeup_gpio_config(void)
{
	return wakeup_gpio;
}
