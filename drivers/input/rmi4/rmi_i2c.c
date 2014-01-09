/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"

#define BUFFER_SIZE_INCREMENT 32

/**
 * struct rmi_i2c_data - stores information for i2c communication
 *
 * @page_mutex: Locks current page to avoid changing pages in unexpected ways.
 * @page: Keeps track of the current virtual page
 * @xport: Pointer to the transport interface
 *
 * @tx_buf: Buffer used for transmitting data to the sensor over i2c.
 * @tx_buf_size: Size of the buffer
 */
struct rmi_i2c_data {
	struct mutex page_mutex;
	int page;
	struct rmi_transport_dev *xport;

	u8 *tx_buf;
	int tx_buf_size;
};

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

static char *xport_proto_name = "i2c";

/*
 * rmi_set_page - Set RMI page
 * @xport: The pointer to the rmi_transport_dev struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the transport
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_transport_dev *xport, u8 page)
{
	struct i2c_client *client = to_i2c_client(xport->dev);
	struct rmi_i2c_data *data = xport->data;
	u8 txbuf[2] = {RMI_PAGE_SELECT_REGISTER, page};
	int retval;

	dev_dbg(&client->dev, "writes 3 bytes: %02x %02x\n",
			txbuf[0], txbuf[1]);
	xport->info.tx_count++;
	xport->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		xport->info.tx_errs++;
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, retval);
		return (retval < 0) ? retval : -EIO;
	}
	data->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_transport_dev *xport, u16 addr,
			       const void *buf, const int len)
{
	struct i2c_client *client = to_i2c_client(xport->dev);
	struct rmi_i2c_data *data = xport->data;
	int retval;
	int tx_size = len + 1;

	mutex_lock(&data->page_mutex);

	if (!data->tx_buf || data->tx_buf_size < tx_size) {
		if (data->tx_buf)
			devm_kfree(&client->dev, data->tx_buf);
		data->tx_buf_size = tx_size + BUFFER_SIZE_INCREMENT;
		data->tx_buf = devm_kzalloc(&client->dev, data->tx_buf_size,
					    GFP_KERNEL);
		if (!data->tx_buf) {
			data->tx_buf_size = 0;
			retval = -ENOMEM;
			goto exit;
		}
	}
	data->tx_buf[0] = addr & 0xff;
	memcpy(data->tx_buf + 1, buf, len);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(xport, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

	dev_dbg(&client->dev,
		"writes %d bytes at %#06x: %*ph\n", len, addr, len, buf);

	xport->info.tx_count++;
	xport->info.tx_bytes += tx_size;
	retval = i2c_master_send(client, data->tx_buf, tx_size);
	if (retval < 0)
		xport->info.tx_errs++;
	else
		retval--; /* don't count the address byte */

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}


static int rmi_i2c_read_block(struct rmi_transport_dev *xport, u16 addr,
			      void *buf, const int len)
{
	struct i2c_client *client = to_i2c_client(xport->dev);
	struct rmi_i2c_data *data = xport->data;
	u8 txbuf[1] = {addr & 0xff};
	int retval;

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		retval = rmi_set_page(xport, RMI_I2C_PAGE(addr));
		if (retval < 0)
			goto exit;
	}

	dev_dbg(&client->dev, "writes 1 bytes: %02x\n", txbuf[0]);

	xport->info.tx_count++;
	xport->info.tx_bytes += sizeof(txbuf);
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval != sizeof(txbuf)) {
		xport->info.tx_errs++;
		retval = (retval < 0) ? retval : -EIO;
		goto exit;
	}

	retval = i2c_master_recv(client, buf, len);

	xport->info.rx_count++;
	xport->info.rx_bytes += len;
	if (retval < 0)
		xport->info.rx_errs++;
	else
		dev_dbg(&client->dev,
			"read %d bytes at %#06x: %*ph\n",
			len, addr, len, buf);

exit:
	mutex_unlock(&data->page_mutex);
	return retval;
}

static int rmi_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	const struct rmi_device_platform_data *pdata = dev_get_platdata(&client->dev);
	struct rmi_transport_dev *xport;
	struct rmi_i2c_data *data;
	int retval;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}
	dev_dbg(&client->dev, "Probing %s at %#02x (GPIO %d).\n",
		pdata->sensor_name ? pdata->sensor_name : "-no name-",
		client->addr, pdata->attn_gpio);

	retval = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!retval) {
		dev_err(&client->dev, "i2c_check_functionality error %d.\n",
			retval);
		return retval;
	}

	if (pdata->gpio_config) {
		retval = pdata->gpio_config(pdata->gpio_data, true);
		if (retval < 0) {
			dev_err(&client->dev, "Failed to configure GPIOs, code: %d.\n",
				retval);
			return retval;
		}
	}

	xport = devm_kzalloc(&client->dev, sizeof(struct rmi_transport_dev),
				GFP_KERNEL);

	if (!xport)
		return -ENOMEM;

	data = devm_kzalloc(&client->dev, sizeof(struct rmi_i2c_data),
				GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->xport = xport;

	xport->data = data;
	xport->dev = &client->dev;

	xport->write_block = rmi_i2c_write_block;
	xport->read_block = rmi_i2c_read_block;
	xport->info.proto = xport_proto_name;

	mutex_init(&data->page_mutex);

	/*
	 * Setting the page to zero will (a) make sure the PSR is in a
	 * known state, and (b) make sure we can talk to the device.
	 */
	retval = rmi_set_page(xport, 0);
	if (retval) {
		dev_err(&client->dev, "Failed to set page select to 0.\n");
		return retval;
	}

	retval = rmi_register_transport_device(xport);
	if (retval) {
		dev_err(&client->dev, "Failed to register transport driver at 0x%.2X.\n",
			client->addr);
		goto err_gpio;
	}
	i2c_set_clientdata(client, xport);

	dev_info(&client->dev, "registered rmi i2c driver at %#04x.\n",
			client->addr);
	return 0;

err_gpio:
	if (pdata->gpio_config)
		pdata->gpio_config(pdata->gpio_data, false);
	return retval;
}

static int rmi_i2c_remove(struct i2c_client *client)
{
	struct rmi_transport_dev *xport = i2c_get_clientdata(client);
	struct rmi_device_platform_data *pdata = dev_get_platdata(&client->dev);

	rmi_unregister_transport_device(xport);

	if (pdata->gpio_config)
		pdata->gpio_config(&pdata->gpio_data, false);

	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ "rmi_i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_i2c"
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= rmi_i2c_remove,
};

module_i2c_driver(rmi_i2c_driver);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com>");
MODULE_DESCRIPTION("RMI I2C driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
