/*
 * Copyright (c) 2011-2014 Espressif System.
 *
 * esp debug interface
 *  - debugfs
 *  - debug level control
 */

#include <linux/types.h>
#include <linux/kernel.h>

#include <net/mac80211.h>
#include "sip2_common.h"

#include "esp_debug.h"

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_ESP8089_DEBUG_FS)

static struct dentry *esp_debugfs_root;

static ssize_t esp_debugfs_read(struct file *filp, char __user *buffer,
				size_t count, loff_t *ppos)
{
	if (*ppos >= 32)
		return 0;

	if (*ppos + count > 32)
		count = 32 - *ppos;

	if (copy_to_user(buffer, filp->private_data + *ppos, count))
		return -EFAULT;

	*ppos += count;

	return count;
}

static ssize_t esp_debugfs_write(struct file *filp, const char __user *buffer,
				 size_t count, loff_t *ppos)
{
	if (*ppos >= 32)
		return 0;

	if (*ppos + count > 32)
		count = 32 - *ppos;

	if (copy_from_user(filp->private_data + *ppos, buffer, count))
		return -EFAULT;

	*ppos += count;

	return count;
}

const struct file_operations esp_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = esp_debugfs_read,
	.write = esp_debugfs_write,
};

struct dentry *esp_dump_var(const char *name, struct dentry *parent,
			    void *value, enum esp_type type)
{
	struct dentry *rc = NULL;
	umode_t mode = 0644;

	if (!esp_debugfs_root)
		return NULL;

	if (!parent)
		parent = esp_debugfs_root;

	switch (type) {
	case ESP_U8:
		rc = debugfs_create_u8(name, mode, parent, (u8 *)value);
		break;
	case ESP_U16:
		rc = debugfs_create_u16(name, mode, parent, (u16 *)value);
		break;
	case ESP_U32:
		rc = debugfs_create_u32(name, mode, parent, (u32 *)value);
		break;
	case ESP_U64:
		rc = debugfs_create_u64(name, mode, parent, (u64 *)value);
		break;
	case ESP_BOOL:
		rc = debugfs_create_bool(name, mode, parent, (bool *)value);
		break;
	default:		//32
		rc = debugfs_create_u32(name, mode, parent, (u32 *)value);
	}

	if (!rc)
		goto _fail;

	return rc;

_fail:
	debugfs_remove_recursive(esp_debugfs_root);
	esp_debugfs_root = NULL;
	printk("%s failed, debugfs root removed; var name: %s\n", __func__,
	       name);
	return NULL;
}

struct dentry *esp_dump_array(const char *name, struct dentry *parent,
			      struct debugfs_blob_wrapper *blob)
{
	struct dentry *rc;
	umode_t mode = 0644;

	if (!esp_debugfs_root)
		return NULL;

	if (!parent)
		parent = esp_debugfs_root;

	rc = debugfs_create_blob(name, mode, parent, blob);
	if (!rc)
		goto _fail;

	return rc;

_fail:
	debugfs_remove_recursive(esp_debugfs_root);
	esp_debugfs_root = NULL;
	printk("%s failed, debugfs root removed; var name: %s\n", __func__,
	       name);
	return NULL;
}

struct dentry *esp_dump(const char *name, struct dentry *parent,
			void *data, int size)
{
	struct dentry *rc;
	umode_t mode = 0644;

	if (!esp_debugfs_root)
		return NULL;

	if (!parent)
		parent = esp_debugfs_root;

	rc = debugfs_create_file(name, mode, parent, data, &esp_debugfs_fops);
	if (!rc)
		goto _fail;

	return rc;

_fail:
	debugfs_remove_recursive(esp_debugfs_root);
	esp_debugfs_root = NULL;
	printk("%s failed, debugfs root removed; var name: %s\n", __func__,
	       name);
	return NULL;
}

struct dentry *esp_debugfs_add_sub_dir(const char *name)
{
	struct dentry *sub_dir;

	sub_dir = debugfs_create_dir(name, esp_debugfs_root);
	if (!sub_dir)
		goto _fail;

	return sub_dir;

_fail:
	debugfs_remove_recursive(esp_debugfs_root);
	esp_debugfs_root = NULL;
	printk("%s failed, debugfs root removed; dir name: %s\n", __func__,
	       name);
	return NULL;
}

int esp_debugfs_init(void)
{
	esp_debugfs_root = debugfs_create_dir("esp_debug", NULL);

	if (IS_ERR_OR_NULL(esp_debugfs_root))
		return -ENOENT;

	return 0;
}

void esp_debugfs_exit(void)
{
	debugfs_remove_recursive(esp_debugfs_root);
}

#else

inline struct dentry *esp_dump_var(const char *name, struct dentry *parent,
				   void *value, enum esp_type type)
{
	return NULL;
}

inline struct dentry *esp_dump_array(const char *name, struct dentry *parent,
				     struct debugfs_blob_wrapper *blob)
{
	return NULL;
}

inline struct dentry *esp_dump(const char *name, struct dentry *parent,
			       void *data, int size)
{
	return NULL;
}

struct dentry *esp_debugfs_add_sub_dir(const char *name)
{
	return NULL;
}

inline int esp_debugfs_init(void)
{
	return -EPERM;
}

inline void esp_debugfs_exit(void)
{
}

#endif

/* FIXME: What's the actual usecase? */
void show_buf(u8 *buf, u32 len)
{
	int i = 0, j;

	printk(KERN_INFO "\n++++++++++++++++show rbuf+++++++++++++++\n");
	for (i = 0; i < (len / 16); i++) {
		j = i * 16;
		printk(KERN_INFO
		       "0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
		       buf[j], buf[j + 1], buf[j + 2], buf[j + 3],
		       buf[j + 4], buf[j + 5], buf[j + 6], buf[j + 7],
		       buf[j + 8], buf[j + 9], buf[j + 10], buf[j + 11],
		       buf[j + 12], buf[j + 13], buf[j + 14], buf[j + 15]);
	}
	printk(KERN_INFO "\n++++++++++++++++++++++++++++++++++++++++\n");
}
