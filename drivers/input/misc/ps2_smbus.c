/*
 * Copyright (c) 2017 Red Hat, Inc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/rmi.h>
#include <linux/serio.h>
#include <linux/slab.h>

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@redhat.com>");
MODULE_DESCRIPTION("Platform PS/2 - SMBus bridge driver");
MODULE_LICENSE("GPL");

static struct workqueue_struct *kps2smbus_wq;
DEFINE_MUTEX(ps2smbus_mutex);

enum ps2smbus_type {
	PS2SMBUS_SYNAPTICS_RMI4,
};

struct ps2smbus {
	struct i2c_client *smbus_client;
	struct notifier_block i2c_notifier;
	enum ps2smbus_type type;
	void *pdata;
};

enum ps2smbus_event_type {
	PS2SMBUS_REGISTER_DEVICE,
	PS2SMBUS_UNREGISTER_DEVICE,
};

struct ps2smbus_work {
	struct work_struct work;
	enum ps2smbus_event_type type;
	struct ps2smbus *ps2smbus;
	struct i2c_adapter *adap;
};

static void ps2smbus_create_rmi4(struct ps2smbus *ps2smbus,
				 struct i2c_adapter *adap)
{
	const struct i2c_board_info i2c_info = {
		I2C_BOARD_INFO("rmi4_smbus", 0x2c),
		.platform_data = ps2smbus->pdata,
		.flags = I2C_CLIENT_HOST_NOTIFY,
	};

	ps2smbus->smbus_client = i2c_new_device(adap, &i2c_info);
}

static void ps2smbus_worker(struct work_struct *work)
{
	struct ps2smbus_work *ps2smbus_work;
	struct i2c_client *client;

	ps2smbus_work = container_of(work, struct ps2smbus_work, work);
	client = ps2smbus_work->ps2smbus->smbus_client;

	mutex_lock(&ps2smbus_mutex);

	switch (ps2smbus_work->type) {
	case PS2SMBUS_REGISTER_DEVICE:
		if (ps2smbus_work->ps2smbus->type == PS2SMBUS_SYNAPTICS_RMI4)
			ps2smbus_create_rmi4(ps2smbus_work->ps2smbus,
					     ps2smbus_work->adap);
		break;
	case PS2SMBUS_UNREGISTER_DEVICE:
		if (client)
			i2c_unregister_device(client);
		break;
	}

	kfree(ps2smbus_work);

	mutex_unlock(&ps2smbus_mutex);
}

static int ps2smbus_schedule_work(enum ps2smbus_event_type type,
				  struct ps2smbus *ps2smbus,
				  struct i2c_adapter *adap)
{
	struct ps2smbus_work *ps2smbus_work;

	ps2smbus_work = kzalloc(sizeof(*ps2smbus_work), GFP_KERNEL);
	if (!ps2smbus_work)
		return -ENOMEM;

	ps2smbus_work->type = type;
	ps2smbus_work->ps2smbus = ps2smbus;
	ps2smbus_work->adap = adap;

	INIT_WORK(&ps2smbus_work->work, ps2smbus_worker);

	queue_work(kps2smbus_wq, &ps2smbus_work->work);

	return 0;
}

static int ps2smbus_attach_i2c_device(struct device *dev, void *data)
{
	struct ps2smbus *ps2smbus = data;
	struct i2c_adapter *adap;

	if (dev->type != &i2c_adapter_type)
		return 0;

	adap = to_i2c_adapter(dev);

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
		return 0;

	if (ps2smbus->smbus_client)
		return 0;

	ps2smbus_schedule_work(PS2SMBUS_REGISTER_DEVICE, ps2smbus, adap);

	pr_debug("ps2smbus: adapter [%s] registered\n", adap->name);
	return 0;
}

static int ps2smbus_detach_i2c_device(struct device *dev,
				      struct ps2smbus *ps2smbus)
{
	struct i2c_client *client;

	if (dev->type == &i2c_adapter_type)
		return 0;

	mutex_lock(&ps2smbus_mutex);

	client = to_i2c_client(dev);
	if (client == ps2smbus->smbus_client)
		ps2smbus->smbus_client = NULL;

	mutex_unlock(&ps2smbus_mutex);

	pr_debug("ps2smbus: client [%s] unregistered\n", client->name);
	return 0;
}

static int ps2smbus_notifier_call(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct device *dev = data;
	struct ps2smbus *ps2smbus;

	ps2smbus = container_of(nb, struct ps2smbus, i2c_notifier);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return ps2smbus_attach_i2c_device(dev, ps2smbus);
	case BUS_NOTIFY_DEL_DEVICE:
		return ps2smbus_detach_i2c_device(dev, ps2smbus);
	}

	return 0;
}

static int ps2smbus_probe(struct platform_device *pdev)
{
	struct rmi_device_platform_data *rmi_pdata = pdev->dev.platform_data;
	struct serio *parent;
	struct ps2smbus *ps2smbus;
	int error;

	ps2smbus = devm_kzalloc(&pdev->dev, sizeof(struct ps2smbus),
				GFP_KERNEL);
	if (!ps2smbus)
		return -ENOMEM;

	ps2smbus->i2c_notifier.notifier_call = ps2smbus_notifier_call;
	ps2smbus->pdata = pdev->dev.platform_data;
	ps2smbus->type = pdev->id_entry->driver_data;
	if (pdev->dev.parent) {
		parent = to_serio_port(pdev->dev.parent);
		if (ps2smbus->type == PS2SMBUS_SYNAPTICS_RMI4)
			rmi_pdata->parent = parent;
	}

	/* Keep track of adapters which will be added or removed later */
	error = bus_register_notifier(&i2c_bus_type, &ps2smbus->i2c_notifier);
	if (error)
		return error;

	/* Bind to already existing adapters right away */
	i2c_for_each_dev(ps2smbus, ps2smbus_attach_i2c_device);

	platform_set_drvdata(pdev, ps2smbus);

	return 0;
}

static int ps2smbus_remove(struct platform_device *pdev)
{
	struct ps2smbus *ps2smbus = platform_get_drvdata(pdev);

	bus_unregister_notifier(&i2c_bus_type, &ps2smbus->i2c_notifier);

	if (ps2smbus->smbus_client)
		ps2smbus_schedule_work(PS2SMBUS_UNREGISTER_DEVICE, ps2smbus,
				       NULL);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct platform_device_id ps2smbus_id_table[] = {
	{ .name = "rmi4", .driver_data = PS2SMBUS_SYNAPTICS_RMI4 },
	{ }
};
MODULE_DEVICE_TABLE(platform, ps2smbus_id_table);

static struct platform_driver ps2smbus_drv = {
	.driver		= {
		.name	= "ps2smbus",
	},
	.probe		= ps2smbus_probe,
	.remove		= ps2smbus_remove,
	.id_table	= ps2smbus_id_table,
};

static int __init ps2smbus_init(void)
{
	int err;

	kps2smbus_wq = alloc_ordered_workqueue("kps2smbusd", WQ_MEM_RECLAIM);
	if (!kps2smbus_wq) {
		pr_err("failed to create kps2smbusd workqueue\n");
		return -ENOMEM;
	}

	err = platform_driver_register(&ps2smbus_drv);
	if (err)
		destroy_workqueue(kps2smbus_wq);

	return err;
}

static void __exit ps2smbus_exit(void)
{
	platform_driver_unregister(&ps2smbus_drv);
	destroy_workqueue(kps2smbus_wq);
}

module_init(ps2smbus_init);
module_exit(ps2smbus_exit);
