/*
 * Copyright (C) 2015 Red Hat
 * Copyright (C) 2015 Stephen Chandler Paul <thatslyude@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/serio.h>
#include "rmi_driver.h"

#define RMI_F03_RX_DATA_OFB		0x01
#define RMI_F03_OB_SIZE			2

#define RMI_F03_OB_OFFSET		2
#define RMI_F03_OB_DATA_OFFSET		1
#define RMI_F03_OB_FLAG_TIMEOUT		(1 << 6)
#define RMI_F03_OB_FLAG_PARITY		(1 << 7)

#define RMI_F03_DEVICE_COUNT		0x07
#define RMI_F03_BYTES_PER_DEVICE_MASK	0x70
#define RMI_F03_BYTES_PER_DEVICE_SHIFT	4
#define RMI_F03_QUEUE_LENGTH		0x0F

struct f03_data {
	struct rmi_function *fn;

	struct serio *serio;

	u8 device_count;
	u8 rx_queue_length;
};

static int rmi_f03_pt_write(struct serio *id, unsigned char val)
{
	struct f03_data *f03 = id->port_data;
	int rc;

	dev_dbg(&f03->fn->dev, "Wrote %hhx to PS/2 passthrough address", val);

	rc = rmi_write(f03->fn->rmi_dev, f03->fn->fd.data_base_addr, val);
	if (rc) {
		dev_err(&f03->fn->dev,
			"%s: Failed to write to F03 TX register.\n", __func__);
		return rc;
	}

	return 0;
}

static inline int rmi_f03_initialize(struct rmi_function *fn)
{
	struct f03_data *f03;
	struct device *dev = &fn->dev;
	int rc;
	u8 bytes_per_device;
	u8 query1;
	size_t query2_len;

	rc = rmi_read(fn->rmi_dev, fn->fd.query_base_addr, &query1);
	if (rc) {
		dev_err(dev, "Failed to read query register.\n");
		return rc;
	}

	f03 = devm_kzalloc(dev, sizeof(struct f03_data), GFP_KERNEL);
	if (!f03) {
		dev_err(dev, "Failed to allocate f03_data.\n");
		return -ENOMEM;
	}

	f03->device_count = query1 & RMI_F03_DEVICE_COUNT;
	bytes_per_device = (query1 & RMI_F03_BYTES_PER_DEVICE_MASK) >>
		RMI_F03_BYTES_PER_DEVICE_SHIFT;

	query2_len = f03->device_count * bytes_per_device;

	/*
	 * The first generation of image sensors don't have a second part to
	 * their f03 query, as such we have to set some of these values manually
	 */
	if (query2_len < 1) {
		f03->device_count = 1;
		f03->rx_queue_length = 7;
	} else {
		u8 query2[query2_len];

		rc = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr + 1,
				    query2, query2_len);
		if (rc) {
			dev_err(dev, "Failed to read second set of query registers.\n");
			return rc;
		}

		f03->rx_queue_length = query2[0] & RMI_F03_QUEUE_LENGTH;
	}

	f03->fn = fn;

	dev_set_drvdata(dev, f03);

	return f03->device_count;
}

static inline int rmi_f03_register_pt(struct rmi_function *fn)
{
	struct f03_data *f03 = dev_get_drvdata(&fn->dev);
	struct serio *serio = kzalloc(sizeof(struct serio), GFP_KERNEL);

	if (!serio)
		return -ENOMEM;

	serio->id.type = SERIO_RMI_PSTHRU;
	serio->write = rmi_f03_pt_write;
	serio->port_data = f03;

	strlcpy(serio->name, "Synaptics RMI4 PS2 pass-through",
		sizeof(serio->name));
	strlcpy(serio->phys, "synaptics-rmi4-pt/serio1",
		sizeof(serio->phys));

	f03->serio = serio;

	serio_register_port(serio);

	return 0;
}

static int rmi_f03_probe(struct rmi_function *fn)
{
	int rc;

	rc = rmi_f03_initialize(fn);
	if (rc < 0)
		return rc;

	dev_dbg(&fn->dev, "%d devices on PS/2 passthrough", rc);

	rc = rmi_f03_register_pt(fn);
	if (rc)
		return rc;

	return 0;
}

static int rmi_f03_config(struct rmi_function *fn)
{
	fn->rmi_dev->driver->set_irq_bits(fn->rmi_dev, fn->irq_mask);

	return 0;
}

static int rmi_f03_attention(struct rmi_function *fn, unsigned long *irq_bits)
{
	struct f03_data *f03 = dev_get_drvdata(&fn->dev);
	u16 data_addr = fn->fd.data_base_addr;
	const u8 ob_len = f03->rx_queue_length * RMI_F03_OB_SIZE;
	u8 obs[ob_len];
	u8 ob_status;
	u8 ob_data;
	unsigned int serio_flags;
	int i;
	int retval;

	/* Grab all of the data registers, and check them for data */
	retval = rmi_read_block(fn->rmi_dev, data_addr + RMI_F03_OB_OFFSET,
				&obs, ob_len);
	if (retval) {
		dev_err(&fn->dev, "%s: Failed to read F03 output buffers.\n",
			__func__);
		serio_interrupt(f03->serio, 0, SERIO_TIMEOUT);
		return retval;
	}

	for (i = 0; i < ob_len; i += RMI_F03_OB_SIZE) {
		ob_status = obs[i];
		ob_data = obs[i + RMI_F03_OB_DATA_OFFSET];
		serio_flags = 0;

		if (!(ob_status & RMI_F03_RX_DATA_OFB))
			continue;

		if (ob_status & RMI_F03_OB_FLAG_TIMEOUT)
			serio_flags |= SERIO_TIMEOUT;
		if (ob_status & RMI_F03_OB_FLAG_PARITY)
			serio_flags |= SERIO_PARITY;

		dev_dbg(&fn->dev,
			"%s: Received %hhx from PS2 guest T: %c P: %c\n", __func__,
			ob_data,
			serio_flags & SERIO_TIMEOUT ?  'Y' : 'N',
			serio_flags & SERIO_PARITY ? 'Y' : 'N');

		serio_interrupt(f03->serio, ob_data, serio_flags);
	}

	return 0;
}

static void rmi_f03_remove(struct rmi_function *fn)
{
	struct f03_data *f03 = dev_get_drvdata(&fn->dev);

	if (f03->serio)
		serio_unregister_port(f03->serio);
}

static struct rmi_function_handler rmi_f03_handler = {
	.driver = {
		.name = "rmi_f03",
	},
	.func = 0x03,
	.probe = rmi_f03_probe,
	.config = rmi_f03_config,
	.attention = rmi_f03_attention,
	.remove = rmi_f03_remove,
};

int __init rmi_register_f03_handler(void)
{
	return rmi_register_function_handler(&rmi_f03_handler);
}

void rmi_unregister_f03_handler(void)
{
	rmi_unregister_function_handler(&rmi_f03_handler);
}

MODULE_AUTHOR("Stephen Chandler Paul <thatslyude@gmail.com>");
MODULE_DESCRIPTION("RMI F03 module");
MODULE_LICENSE("GPL");
