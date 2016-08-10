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
#include <linux/platform_device.h>
#include <linux/rmi.h>
#include <linux/slab.h>

#define DRIVER_DESC	"RMI4 Platform PS/2 - SMBus bridge driver"

MODULE_AUTHOR("Benjamin Tissoires <benjamin.tissoires@redhat.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static struct workqueue_struct *krmi_wq;
DEFINE_MUTEX(rmi_mutex);

struct rmi_pltf {
	struct i2c_client *smbus_client;
	struct notifier_block i2c_notifier;
	struct rmi_device_platform_data *pdata;
};

enum rmi_event_type {
	RMI_REGISTER_DEVICE,
	RMI_UNREGISTER_DEVICE,
};

struct rmi_work {
	struct work_struct work;
	enum rmi_event_type type;
	struct rmi_pltf *rmi;
	struct i2c_adapter *adap;
};

static void rmi_create_intertouch(struct rmi_pltf *rmi_pltf,
				  struct i2c_adapter *adap)
{
	const struct i2c_board_info i2c_info = {
		I2C_BOARD_INFO("rmi4_smbus", 0x2c),
		.platform_data = rmi_pltf->pdata,
	};

	rmi_pltf->smbus_client = i2c_new_device(adap, &i2c_info);
}

static void rmi_worker(struct work_struct *work)
{
	struct rmi_work *rmi_work = container_of(work, struct rmi_work, work);

	mutex_lock(&rmi_mutex);

	switch (rmi_work->type) {
	case RMI_REGISTER_DEVICE:
		rmi_create_intertouch(rmi_work->rmi, rmi_work->adap);
		break;
	case RMI_UNREGISTER_DEVICE:
		if (rmi_work->rmi->smbus_client)
			i2c_unregister_device(rmi_work->rmi->smbus_client);
		break;
	};

	kfree(rmi_work);

	mutex_unlock(&rmi_mutex);
}

static int rmi_schedule_work(enum rmi_event_type type,
			     struct rmi_pltf *rmi,
			     struct i2c_adapter *adap)
{
	struct rmi_work *rmi_work = kzalloc(sizeof(*rmi_work), GFP_KERNEL);

	if (!rmi_work)
		return -ENOMEM;

	rmi_work->type = type;
	rmi_work->rmi = rmi;
	rmi_work->adap = adap;

	INIT_WORK(&rmi_work->work, rmi_worker);

	queue_work(krmi_wq, &rmi_work->work);

	return 0;
}

static int rmi_attach_i2c_device(struct device *dev, void *data)
{
	struct rmi_pltf *rmi_pltf = data;
	struct i2c_adapter *adap;

	if (dev->type != &i2c_adapter_type)
		return 0;

	adap = to_i2c_adapter(dev);

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
		return 0;

	if (rmi_pltf->smbus_client)
		return 0;

	rmi_schedule_work(RMI_REGISTER_DEVICE, rmi_pltf, adap);

	pr_debug("rmi_platform: adapter [%s] registered\n", adap->name);
	return 0;
}

static int rmi_detach_i2c_device(struct device *dev, struct rmi_pltf *rmi_pltf)
{
	struct i2c_client *client;

	if (dev->type == &i2c_adapter_type)
		return 0;

	mutex_lock(&rmi_mutex);

	client = to_i2c_client(dev);
	if (client == rmi_pltf->smbus_client)
		rmi_pltf->smbus_client = NULL;

	mutex_unlock(&rmi_mutex);

	pr_debug("rmi_platform: client [%s] unregistered\n", client->name);
	return 0;
}

static int rmi_notifier_call(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct device *dev = data;
	struct rmi_pltf *rmi_pltf;

	rmi_pltf = container_of(nb, struct rmi_pltf, i2c_notifier);

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return rmi_attach_i2c_device(dev, rmi_pltf);
	case BUS_NOTIFY_DEL_DEVICE:
		return rmi_detach_i2c_device(dev, rmi_pltf);
	}

	return 0;
}

static int rmi_probe(struct platform_device *pdev)
{
	struct rmi_device_platform_data *pdata = pdev->dev.platform_data;
	struct rmi_pltf *rmi_pltf;
	int error;

	rmi_pltf = devm_kzalloc(&pdev->dev, sizeof(struct rmi_pltf),
				GFP_KERNEL);
	if (!rmi_pltf)
		return -ENOMEM;

	rmi_pltf->i2c_notifier.notifier_call = rmi_notifier_call;

	rmi_pltf->pdata = pdata;

	/* Keep track of adapters which will be added or removed later */
	error = bus_register_notifier(&i2c_bus_type, &rmi_pltf->i2c_notifier);
	if (error)
		return error;

	/* Bind to already existing adapters right away */
	i2c_for_each_dev(rmi_pltf, rmi_attach_i2c_device);

	platform_set_drvdata(pdev, rmi_pltf);

	return 0;
}

static int rmi_remove(struct platform_device *pdev)
{
	struct rmi_pltf *rmi_pltf = platform_get_drvdata(pdev);

	bus_unregister_notifier(&i2c_bus_type, &rmi_pltf->i2c_notifier);

	if (rmi_pltf->smbus_client)
		rmi_schedule_work(RMI_UNREGISTER_DEVICE, rmi_pltf, NULL);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct platform_device_id rmi_id_table[] = {
	{ .name = "rmi4" },
	{ }
};
MODULE_DEVICE_TABLE(platform, rmi_id_table);

static struct platform_driver rmi_drv = {
	.driver		= {
		.name	= "rmi4",
	},
	.probe		= rmi_probe,
	.remove		= rmi_remove,
	.id_table	= rmi_id_table,
};

static int __init rmi_init(void)
{
	int err;

	krmi_wq = create_singlethread_workqueue("krmid");
	if (!krmi_wq) {
		pr_err("failed to create krmid workqueue\n");
		return -ENOMEM;
	}

	err = platform_driver_register(&rmi_drv);
	if (err)
		destroy_workqueue(krmi_wq);

	return err;
}

static void __exit rmi_exit(void)
{
	platform_driver_unregister(&rmi_drv);
	destroy_workqueue(krmi_wq);
}

module_init(rmi_init);
module_exit(rmi_exit);
