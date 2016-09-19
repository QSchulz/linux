/*
 * Copyright (c) 2010 -2014 Espressif System.
 *
 *   file operation in kernel space
 *
 */

#ifndef _ESP_FILE_H_
#define _ESP_FILE_H_

#include <linux/firmware.h>

#define CONF_ATTR_LEN		24
#define MAX_ATTR_NUM		24
#define MAX_FIX_ATTR_NUM	16

struct esp_init_table_elem {
	char attr[CONF_ATTR_LEN];
	int offset;
	short value;
};

int request_init_conf(struct device *dev);
void fix_init_data(u8 *init_data_buf, int buf_size);

#endif				/* _ESP_FILE_H_ */
