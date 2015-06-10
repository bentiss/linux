/*
 * Copyright (c) 2011, 2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kconfig.h>
#include <linux/lockdep.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"

#define SMB_PROTOCOL_VERSION_ADDRESS	0xfd
#define SMB_MAX_COUNT			32
#define RMI_SMB2_MAP_SIZE		8 /* 8 entry of 4 bytes each */
#define RMI_SMB2_MAP_FLAGS_WE		0x01

#define BUFFER_SIZE_INCREMENT		32

struct mapping_table_entry {
	union {
		struct {
			u16 rmiaddr;
			u8 readcount;
			u8 flags;
		};
		u8 entry[4];
	};
};

struct rmi_smb_xport {
	struct rmi_transport_dev xport;
	struct i2c_client *client;
	bool pdata_created;

	struct mutex page_mutex;
	int page;
	u8 table_index;
	struct mutex mappingtable_mutex;
	struct mapping_table_entry mapping_table[RMI_SMB2_MAP_SIZE];

	u8 *tx_buf;
	size_t tx_buf_size;
};

/*SMB block write - wrapper over ic2_smb_write_block */
static int smb_block_write(struct rmi_transport_dev *xport,
			      u8 commandcode, const void *buf, size_t len)
{
	struct rmi_smb_xport *rmi_smb =
		container_of(xport, struct rmi_smb_xport, xport);
	struct i2c_client *client = rmi_smb->client;
	int tx_size = len + 1;
	int retval;

	if (!rmi_smb->tx_buf || rmi_smb->tx_buf_size < tx_size) {
		if (rmi_smb->tx_buf)
			devm_kfree(&client->dev, rmi_smb->tx_buf);
		rmi_smb->tx_buf_size = tx_size + BUFFER_SIZE_INCREMENT;
		rmi_smb->tx_buf = devm_kzalloc(&client->dev,
					       rmi_smb->tx_buf_size,
					       GFP_KERNEL);
		if (!rmi_smb->tx_buf) {
			rmi_smb->tx_buf_size = 0;
			retval = -ENOMEM;
			goto exit;
		}
	}

	rmi_smb->tx_buf[0] = commandcode & 0xff;
	memcpy(rmi_smb->tx_buf + 1, buf, len);

	retval = i2c_smbus_write_block_data(client, commandcode,
					    tx_size, rmi_smb->tx_buf);

exit:
	dev_dbg(&client->dev,
		"write %zd bytes at %#04x: %d (%*ph)\n",
		len, commandcode, retval, (int)len, buf);

	xport->stats.tx_count++;
	if (retval)
		xport->stats.tx_errs++;
	else
		xport->stats.tx_bytes += len;

	return retval;
}

/* The function to get command code for smbus operations and keeps
records to the driver mapping table */
static int rmi_smb_get_command_code(struct rmi_transport_dev *xport,
		u16 rmiaddr, int bytecount, bool isread, u8 *commandcode)
{
	struct rmi_smb_xport *rmi_smb =
		container_of(xport, struct rmi_smb_xport, xport);
	struct i2c_client *client = rmi_smb->client;
	int i;
	int retval;
	struct mapping_table_entry *mapping_data = NULL;
	u8 test_read[16];

	pr_err("%s rmiaddr: %#06x, bytecount: %d, isread: %s %s:%d\n", __func__,
		rmiaddr,
		bytecount,
		isread ? "true" : "false",
		__FILE__, __LINE__);
	mutex_lock(&rmi_smb->mappingtable_mutex);
	for (i = 0; i < RMI_SMB2_MAP_SIZE; i++) {
		if (rmi_smb->mapping_table[i].rmiaddr == rmiaddr) {
			if (isread) {
				if (rmi_smb->mapping_table[i].readcount
							== bytecount) {
					*commandcode = i;
					retval = 0;
					goto exit;
				}
			} else {
				if (rmi_smb->mapping_table[i].flags &
							RMI_SMB2_MAP_FLAGS_WE) {
					*commandcode = i;
					retval = 0;
					goto exit;
				}
			}
		}
	}
	i = rmi_smb->table_index;
	rmi_smb->table_index = (i + 1) % RMI_SMB2_MAP_SIZE;

	mapping_data = kzalloc(sizeof(struct mapping_table_entry), GFP_KERNEL);
	if (!mapping_data) {
		retval = -ENOMEM;
		goto exit;
	}
	/* constructs mapping table data entry. 4 bytes each entry */
	mapping_data->rmiaddr = rmiaddr;
	mapping_data->readcount = bytecount;
	mapping_data->flags = RMI_SMB2_MAP_FLAGS_WE; /* enable write */

	retval = smb_block_write(xport, i + 0x80, mapping_data,
				 sizeof(struct mapping_table_entry));

	if (retval < 0) {
		/* if not written to device mapping table */
		/* clear the driver mapping table records */
		rmi_smb->mapping_table[i].rmiaddr = 0x0000;
		rmi_smb->mapping_table[i].readcount = 0;
		rmi_smb->mapping_table[i].flags = 0;
		pr_err("%s  %s:%d\n", __func__, __FILE__, __LINE__);
		goto exit;
	}
	/* save to the driver level mapping table */
	rmi_smb->mapping_table[i].rmiaddr = rmiaddr;
	rmi_smb->mapping_table[i].readcount = bytecount;
	rmi_smb->mapping_table[i].flags = RMI_SMB2_MAP_FLAGS_WE;
	*commandcode = i;

	memset(test_read, 0, sizeof(test_read));
	retval = i2c_smbus_read_block_data(client, i + 0x80, test_read);
	if (retval >= 0)
		pr_err("%s %04x: %*ph %s:%d\n", __func__, i, (int)sizeof(test_read), test_read, __FILE__, __LINE__);
	else
		pr_err("%s  %04x: retval: %d %s:%d\n", __func__, i, retval, __FILE__, __LINE__);

exit:
	kfree(mapping_data);
	mutex_unlock(&rmi_smb->mappingtable_mutex);
	pr_err("%s commandcode: %#04x %s:%d\n", __func__, *commandcode, __FILE__, __LINE__);

	return retval;
}

static int rmi_smb_write_block(struct rmi_transport_dev *xport, u16 rmiaddr,
				const void *databuff, size_t len)
{
	int retval = 0;
	u8 commandcode;
	struct rmi_smb_xport *rmi_smb =
		container_of(xport, struct rmi_smb_xport, xport);

	mutex_lock(&rmi_smb->page_mutex);

	while (len > 0) {  /* while more than 32 bytes */
		/* break into 32 butes chunks to write */
		/* get command code */
		int block_len = min((int)len, SMB_MAX_COUNT);
		retval = rmi_smb_get_command_code(xport, rmiaddr, block_len,
			false, &commandcode);
		if (retval < 0)
			goto exit;

		pr_err("%s rmiaddr: %#06x commandcode: %#04x %s:%d\n", __func__, rmiaddr, commandcode, __FILE__, __LINE__);

		/* write to smb device */
		retval = smb_block_write(xport, commandcode,
					    databuff, block_len);
		if (retval < 0)
			goto exit;

		/* prepare to write next block of bytes */
		len -= SMB_MAX_COUNT;
		databuff += SMB_MAX_COUNT;
		rmiaddr += SMB_MAX_COUNT;
	}
exit:
	mutex_unlock(&rmi_smb->page_mutex);
	return retval;
}

/*SMB block read - wrapper over ic2_smb_read_block */
static int smb_block_read(struct rmi_transport_dev *xport,
			     u8 commandcode, void *buf, size_t len)
{
	struct rmi_smb_xport *rmi_smb =
		container_of(xport, struct rmi_smb_xport, xport);
	struct i2c_client *client = rmi_smb->client;
	int retval;

	retval = i2c_smbus_read_block_data(client, commandcode, buf);
	xport->stats.rx_count++;
	xport->stats.rx_bytes += len;
	if (retval < 0) {
		xport->stats.rx_errs++;
		return retval;
	}

	return retval;
}

static int rmi_smb_read_block(struct rmi_transport_dev *xport, u16 rmiaddr,
			      void *databuff, size_t len)
{
	struct rmi_smb_xport *rmi_smb =
		container_of(xport, struct rmi_smb_xport, xport);
	int retval;
	u8 commandcode;
	int cur_len = (int)len;
	u8 *inputbuff = databuff;

	mutex_lock(&rmi_smb->page_mutex);
	memset(databuff, 0, len);

	while (cur_len > 0) {
		/* break into 32 bytes chunks to write */
		/* get command code */
		int block_len = min(cur_len, SMB_MAX_COUNT);

		retval = rmi_smb_get_command_code(xport, rmiaddr, block_len,
						  false, &commandcode);
		if (retval < 0)
			goto exit;

		pr_err("%s rmiaddr: %#06x commandcode: %#04x %s:%d\n", __func__, rmiaddr, commandcode, __FILE__, __LINE__);
		/* read to smb device */
		retval = smb_block_read(xport, commandcode,
					databuff, block_len);
		if (retval < 0)
			goto exit;

		/* prepare to read next block of bytes */
		cur_len -= SMB_MAX_COUNT;
		databuff += SMB_MAX_COUNT;
		rmiaddr += SMB_MAX_COUNT;
	}

	retval = 0;

exit:
	pr_err("%s retval: %d : %*ph (len: %d) %s:%d\n", __func__, retval, (int)len, inputbuff, (int)len, __FILE__, __LINE__);

	mutex_unlock(&rmi_smb->page_mutex);
	return retval;
}

static const struct rmi_transport_ops rmi_smb_ops = {
	.write_block	= rmi_smb_write_block,
	.read_block	= rmi_smb_read_block,
};

static int rmi_smb_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct rmi_device_platform_data *pdata = dev_get_platdata(&client->dev);
	struct rmi_smb_xport *rmi_smb;
	int retval;
	int smbus_version;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA)) {
		dev_err(&client->dev,
			"adapter does not support required functionality.\n");
		return -ENODEV;
	}

	rmi_smb = devm_kzalloc(&client->dev, sizeof(struct rmi_smb_xport),
				GFP_KERNEL);
	if (!rmi_smb)
		return -ENOMEM;

	if (!pdata) {
		dev_info(&client->dev, "no platform data, allocating one\n");
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata),
				     GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		pdata->sensor_name = "Synaptics SMBus";
		rmi_smb->pdata_created = true;

		client->dev.platform_data = pdata;
	}

	dev_dbg(&client->dev, "Probing %s at %#02x (GPIO %d).\n",
		pdata->sensor_name ? pdata->sensor_name : "-no name-",
		client->addr, pdata->attn_gpio);

	if (pdata->gpio_config) {
		retval = pdata->gpio_config(pdata->gpio_data, true);
		if (retval < 0) {
			dev_err(&client->dev, "Failed to configure GPIOs, code: %d.\n",
				retval);
			return retval;
		}
	}

	rmi_smb->client = client;
	mutex_init(&rmi_smb->page_mutex);
	mutex_init(&rmi_smb->mappingtable_mutex);

	rmi_smb->xport.dev = &client->dev;
	rmi_smb->xport.proto_name = "smb2";
	rmi_smb->xport.ops = &rmi_smb_ops;

	/* Check if for SMBus new version device by reading version byte. */
	retval = i2c_smbus_read_byte_data(client, SMB_PROTOCOL_VERSION_ADDRESS);
	if (retval < 0) {
		dev_err(&client->dev, "failed to get SMBus version number!\n");
		return retval;
	}
	smbus_version = retval + 1;
	dev_dbg(&client->dev, "Smbus version is %d", smbus_version);

	if (smbus_version != 2) {
		dev_err(&client->dev, "Unrecognized SMB version %d.\n",
				smbus_version);
		retval = -ENODEV;
		goto err_gpio;
	}

	retval = rmi_register_transport_device(&rmi_smb->xport);
	if (retval) {
		dev_err(&client->dev, "Failed to register transport driver at 0x%.2X.\n",
			client->addr);
		goto err_gpio;
	}

	i2c_set_clientdata(client, rmi_smb);

	dev_info(&client->dev, "registered rmi smb driver at %#04x.\n",
			client->addr);
	return 0;

err_gpio:
	if (pdata->gpio_config)
		pdata->gpio_config(pdata->gpio_data, false);
	if (rmi_smb->pdata_created)
		client->dev.platform_data = NULL;

	return retval;
}

static int rmi_smb_remove(struct i2c_client *client)
{
	const struct rmi_device_platform_data *pdata =
				dev_get_platdata(&client->dev);
	struct rmi_smb_xport *rmi_smb = i2c_get_clientdata(client);

	rmi_unregister_transport_device(&rmi_smb->xport);

	if (rmi_smb->pdata_created)
		client->dev.platform_data = NULL;
	if (pdata && pdata->gpio_config)
		pdata->gpio_config(pdata->gpio_data, false);

	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ "rmi_smbus", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_smb_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_smbus"
	},
	.id_table	= rmi_id,
	.probe		= rmi_smb_probe,
	.remove		= rmi_smb_remove,
};

module_i2c_driver(rmi_smb_driver);

MODULE_AUTHOR("Allie Xiong <axiong@synaptics.com>");
MODULE_DESCRIPTION("RMI SMBus driver");
MODULE_LICENSE("GPL");
