/*
 * Copyright (c) 2016 Red Hat, Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/libps2.h>
#include <linux/rmi.h>
#include <linux/serio.h>
#include <linux/slab.h>

struct psmouse;

#include "../mouse/synaptics.h"

#define DRIVER_DESC	"RMI4 PS/2 driver"

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@redhat.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

struct rmi_ps2_platform_data {
	struct i2c_board_info i2c_info;
	struct rmi_device_platform_data smbus_pdata;
	struct rmi_f03_data f03_pdata;
	struct rmi_f30_data f30_pdata;
	struct rmi_2d_sensor_platform_data rmi_2d_sensor_pdata;
};

struct rmi_ps2 {
	struct ps2dev ps2dev;
	struct i2c_client *smbus_client;
	struct notifier_block i2c_notifier;
	struct rmi_ps2_platform_data pdata;

	unsigned long int model_id;		/* Model-ID */
	unsigned long int firmware_id;		/* Firmware-ID */
	unsigned long int board_id;		/* Board-ID */
	unsigned long int capabilities;		/* Capabilities */
	unsigned long int ext_cap;		/* Extended Capabilities */
	unsigned long int ext_cap_0c;		/* Ext Caps from 0x0c query */
	unsigned long int ext_cap_10;		/* Ext Caps from 0x10 query */
	unsigned long int identity;		/* Identification */
};

static const struct rmi_2d_sensor_platform_data rmi_smbus_2d_sensor_data = {
	.sensor_type = rmi_sensor_touchpad,
	.axis_align.flip_y = true,
	.kernel_tracking = true, /* to prevent cursors jumps */
};

static int rmi_serio_create_intertouch(struct rmi_ps2 *rmi_ps2,
				       struct i2c_adapter *adap)
{
	struct rmi_ps2_platform_data *pdata = &rmi_ps2->pdata;
	if (rmi_ps2->smbus_client)
		return -EINVAL;

	memset(pdata, 0, sizeof(*pdata));

	pdata->i2c_info.addr = 0x2c;
	pdata->i2c_info.platform_data = &pdata->smbus_pdata;
	strlcpy(pdata->i2c_info.type, "rmi4_smbus", I2C_NAME_SIZE);

	pdata->smbus_pdata.sensor_pdata = &pdata->rmi_2d_sensor_pdata;
	pdata->smbus_pdata.f03_data = &pdata->f03_pdata;
	pdata->smbus_pdata.f30_data = &pdata->f30_pdata;

	pdata->f03_pdata.parent = rmi_ps2->ps2dev.serio;

	pdata->rmi_2d_sensor_pdata = rmi_smbus_2d_sensor_data;

	pdata->f30_pdata.trackstick_buttons =
			!!SYN_CAP_EXT_BUTTONS_STICK(rmi_ps2->ext_cap_10);

	pdata->rmi_2d_sensor_pdata.topbuttonpad =
			synaptics_is_topbuttonpad(rmi_ps2->ps2dev.serio) &&
			!SYN_CAP_EXT_BUTTONS_STICK(rmi_ps2->ext_cap_10);

	rmi_ps2->smbus_client = i2c_new_device(adap, &pdata->i2c_info);

	return 0;
}

static int rmi_serio_attach_i2c_device(struct device *dev, void *data)
{
	struct rmi_ps2 *rmi_ps2 = data;
	struct i2c_adapter *adap;

	if (dev->type != &i2c_adapter_type)
		return 0;

	adap = to_i2c_adapter(dev);

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
		return 0;

	if (rmi_ps2->smbus_client)
		return 0;

	if (rmi_serio_create_intertouch(rmi_ps2, adap))
		pr_err("%s Can't create SMBus device, exists already %s:%d\n",
			__func__, __FILE__, __LINE__);

	pr_debug("rmi_serio: adapter [%s] registered\n", adap->name);
	return 0;
}

static int rmi_serio_detach_i2c_device(struct device *dev,
				       struct rmi_ps2 *rmi_ps2)
{
	struct i2c_client *client;

	if (dev->type == &i2c_adapter_type)
		return 0;

	client = to_i2c_client(dev);
	if (client == rmi_ps2->smbus_client)
		rmi_ps2->smbus_client = NULL;

	pr_debug("rmi_serio: client [%s] unregistered\n", client->name);
	return 0;
}

static int rmi_serio_notifier_call(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct device *dev = data;
	struct rmi_ps2 *rmi_ps2;

	rmi_ps2 = container_of(nb, struct rmi_ps2, i2c_notifier);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return rmi_serio_attach_i2c_device(dev, rmi_ps2);
	case BUS_NOTIFY_DEL_DEVICE:
		return rmi_serio_detach_i2c_device(dev, rmi_ps2);
	}

	return 0;
}

/*
 * Send a command to the synaptics touchpad by special commands
 */
static inline int rmi_serio_send_cmd(struct rmi_ps2 *rmi_ps2, unsigned char c,
				     unsigned char *param)
{
	return synaptics_send_ps2_cmd(&rmi_ps2->ps2dev, c, param);
}

static irqreturn_t rmi_serio_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
	struct rmi_ps2 *rmi_ps2 = serio_get_drvdata(serio);

	if (unlikely((flags & SERIO_TIMEOUT) ||
		     (flags & SERIO_PARITY))) {
		ps2_cmd_aborted(&rmi_ps2->ps2dev);
		goto out;
	}

	if (unlikely(rmi_ps2->ps2dev.flags & PS2_FLAG_ACK))
		if  (ps2_handle_ack(&rmi_ps2->ps2dev, data))
			goto out;

	if (unlikely(rmi_ps2->ps2dev.flags & PS2_FLAG_CMD))
		if  (ps2_handle_response(&rmi_ps2->ps2dev, data))
			goto out;

out:
	return IRQ_HANDLED;
}

#define RMI_SERIO_READ_REGISTER(r, query, storage)			\
	{								\
		if (rmi_serio_send_cmd(r, query, buf))			\
			return -EIO;					\
		(r)->storage = (buf[0]<<16) | (buf[1]<<8) | buf[2];	\
	}

static int rmi_serio_ps2_init(struct rmi_ps2 *rmi_ps2)
{
	unsigned char buf[3];

	if (ps2_reset(&rmi_ps2->ps2dev))
		return -EIO;

	RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_IDENTIFY, identity);

	if (!SYN_ID_IS_SYNAPTICS(rmi_ps2->identity))
		return -EINVAL;

	RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_MODEL, model_id);
	RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_FIRMWARE_ID, firmware_id);
	RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_CAPABILITIES, capabilities);

	if (!SYN_CAP_EXTENDED(rmi_ps2->capabilities))
		rmi_ps2->capabilities = 0;

	if (SYN_EXT_CAP_REQUESTS(rmi_ps2->capabilities) >= 1) {
		RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_EXT_CAPAB, ext_cap);

		if (SYN_CAP_MULTI_BUTTON_NO(rmi_ps2->ext_cap) > 8)
			rmi_ps2->ext_cap &= 0xff0fff;
	}

	if (SYN_EXT_CAP_REQUESTS(rmi_ps2->capabilities) >= 4)
		RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_EXT_CAPAB_0C,
					ext_cap_0c);

	/* firmwares prior 7.5 have no board_id encoded */
	if (SYN_ID_FULL(rmi_ps2->identity) >= 0x705) {
		if (rmi_serio_send_cmd(rmi_ps2, SYN_QUE_MODES, buf))
			return -EIO;

		rmi_ps2->board_id = ((buf[0] & 0xfc) << 6) | buf[1];

		if (SYN_MEXT_CAP_BIT(buf[0]))
			RMI_SERIO_READ_REGISTER(rmi_ps2, SYN_QUE_MEXT_CAPAB_10,
						ext_cap_10);
	}

	return 0;
}

static int rmi_serio_connect(struct serio *serio, struct serio_driver *drv)
{
	struct rmi_ps2 *rmi_ps2;
	int error;

	if (!synaptics_use_intertouch(serio))
		return -ENODEV;

	rmi_ps2 = devm_kzalloc(&serio->dev, sizeof(struct rmi_ps2), GFP_KERNEL);
	if (!rmi_ps2)
		return -ENOMEM;

	rmi_ps2->i2c_notifier.notifier_call = rmi_serio_notifier_call;

	ps2_init(&rmi_ps2->ps2dev, serio);

	error = serio_open(serio, drv);
	if (error)
		return error;

	serio_set_drvdata(serio, rmi_ps2);

	error = rmi_serio_ps2_init(rmi_ps2);
	if (error)
		goto err_clear_drvdata;

	/* Keep track of adapters which will be added or removed later */
	error = bus_register_notifier(&i2c_bus_type, &rmi_ps2->i2c_notifier);
	if (error)
		goto err_clear_drvdata;

	/* Bind to already existing adapters right away */
	i2c_for_each_dev(rmi_ps2, rmi_serio_attach_i2c_device);

	return 0;

err_clear_drvdata:
	serio_set_drvdata(serio, NULL);
	serio_close(serio);
	return error;
}

static int rmi_serio_reconnect(struct serio *serio)
{
	struct rmi_ps2 *rmi_ps2 = serio_get_drvdata(serio);
	struct device_driver *driver;

	if (ps2_reset(&rmi_ps2->ps2dev))
		return -EIO;

	if (rmi_ps2->smbus_client && rmi_ps2->smbus_client->dev.driver) {
		/*
		 * The PS/2 port has been reset and the device is in an
		 * unknown state. Send a resume command.
		 */
		driver = rmi_ps2->smbus_client->dev.driver;
		if (driver->resume)
			driver->resume(&rmi_ps2->smbus_client->dev);
	}

	return 0;
}

static void rmi_serio_disconnect(struct serio *serio)
{
	struct rmi_ps2 *rmi_ps2;

	rmi_ps2 = serio_get_drvdata(serio);

	if (rmi_ps2->smbus_client)
		i2c_unregister_device(rmi_ps2->smbus_client);

	bus_unregister_notifier(&i2c_bus_type, &rmi_ps2->i2c_notifier);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
}

static struct serio_device_id rmi_serio_ids[] = {
	{
		.type	= SERIO_ANY,
		.proto	= SERIO_ANY,
		.id	= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, rmi_serio_ids);

static struct serio_driver rmi_serio_drv = {
	.driver		= {
		.name	= "rmi4_ps2",
	},
	.description	= DRIVER_DESC,
	.id_table	= rmi_serio_ids,
	.interrupt	= rmi_serio_interrupt,
	.connect	= rmi_serio_connect,
	.reconnect	= rmi_serio_reconnect,
	.disconnect	= rmi_serio_disconnect,
};

module_serio_driver(rmi_serio_drv);
