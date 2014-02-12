/*
 * Copyright (c) 2011-2014 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "rmi_driver.h"

#define RMI_PRODUCT_ID_LENGTH    10
#define RMI_PRODUCT_INFO_LENGTH   2

#define RMI_DATE_CODE_LENGTH      3

#define PRODUCT_ID_OFFSET 0x10
#define PRODUCT_INFO_OFFSET 0x1E


/* Force a firmware reset of the sensor */
#define RMI_F01_CMD_DEVICE_RESET	1

/* Various F01_RMI_QueryX bits */

#define RMI_F01_QRY1_CUSTOM_MAP		(1 << 0)
#define RMI_F01_QRY1_NON_COMPLIANT	(1 << 1)
#define RMI_F01_QRY1_HAS_LTS		(1 << 2)
#define RMI_F01_QRY1_HAS_SENSOR_ID	(1 << 3)
#define RMI_F01_QRY1_HAS_CHARGER_INP	(1 << 4)
#define RMI_F01_QRY1_HAS_ADJ_DOZE	(1 << 5)
#define RMI_F01_QRY1_HAS_ADJ_DOZE_HOFF	(1 << 6)
#define RMI_F01_QRY1_HAS_PROPS_2	(1 << 7)

#define RMI_F01_QRY5_YEAR_MASK		0x1f
#define RMI_F01_QRY6_MONTH_MASK		0x0f
#define RMI_F01_QRY7_DAY_MASK		0x1f

#define RMI_F01_QRY2_PRODINFO_MASK	0x7f

#define RMI_F01_BASIC_QUERY_LEN		21 /* From Query 00 through 20 */

struct f01_basic_properties {
	u8 manufacturer_id;
	bool has_lts;
	bool has_adjustable_doze;
	bool has_adjustable_doze_holdoff;
	char dom[11]; /* YYYY/MM/DD + '\0' */
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
	u16 productinfo;
};

/* F01 device status bits */

/* Most recent device status event */
#define RMI_F01_STATUS_CODE(status)		((status) & 0x0f)
/* The device has lost its configuration for some reason. */
#define RMI_F01_STATUS_UNCONFIGURED(status)	(!!((status) & 0x80))

/* Control register bits */

/*
 * Sleep mode controls power management on the device and affects all
 * functions of the device.
 */
#define RMI_F01_CTRL0_SLEEP_MODE_MASK	0x03

#define RMI_SLEEP_MODE_NORMAL		0x00
#define RMI_SLEEP_MODE_SENSOR_SLEEP	0x01
#define RMI_SLEEP_MODE_RESERVED0	0x02
#define RMI_SLEEP_MODE_RESERVED1	0x03

#define RMI_IS_VALID_SLEEPMODE(mode) \
	(mode >= RMI_SLEEP_MODE_NORMAL && mode <= RMI_SLEEP_MODE_RESERVED1)

/*
 * This bit disables whatever sleep mode may be selected by the sleep_mode
 * field and forces the device to run at full power without sleeping.
 */
#define RMI_F01_CRTL0_NOSLEEP_BIT	(1 << 2)

/*
 * When this bit is set, the touch controller employs a noise-filtering
 * algorithm designed for use with a connected battery charger.
 */
#define RMI_F01_CRTL0_CHARGER_BIT	(1 << 5)

/*
 * Sets the report rate for the device. The effect of this setting is
 * highly product dependent. Check the spec sheet for your particular
 * touch sensor.
 */
#define RMI_F01_CRTL0_REPORTRATE_BIT	(1 << 6)

/*
 * Written by the host as an indicator that the device has been
 * successfully configured.
 */
#define RMI_F01_CRTL0_CONFIGURED_BIT	(1 << 7)

/**
 * @ctrl0 - see the bit definitions above.
 * @interrupt_enable - A mask of per-function interrupts on the touch sensor.
 * @doze_interval - controls the interval between checks for finger presence
 * when the touch sensor is in doze mode, in units of 10ms.
 * @wakeup_threshold - controls the capacitance threshold at which the touch
 * sensor will decide to wake up from that low power state.
 * @doze_holdoff - controls how long the touch sensor waits after the last
 * finger lifts before entering the doze state, in units of 100ms.
 */
struct f01_device_control {
	u8 ctrl0;
	u8 *interrupt_enable;
	u8 doze_interval;
	u8 wakeup_threshold;
	u8 doze_holdoff;
};

struct f01_data {
	struct f01_basic_properties properties;

	struct f01_device_control device_control;
	struct mutex control_mutex;

	u8 device_status;

	u16 interrupt_enable_addr;
	u16 doze_interval_addr;
	u16 wakeup_threshold_addr;
	u16 doze_holdoff_addr;
	int irq_count;
	int num_of_irq_regs;

#ifdef CONFIG_PM_SLEEP
	bool suspended;
	bool old_nosleep;
#endif
};

static int rmi_f01_alloc_memory(struct rmi_function *fn,
				int num_of_irq_regs)
{
	struct f01_data *f01;

	f01 = devm_kzalloc(&fn->dev, sizeof(struct f01_data), GFP_KERNEL);
	if (!f01) {
		dev_err(&fn->dev, "Failed to allocate fn_01_data.\n");
		return -ENOMEM;
	}

	f01->device_control.interrupt_enable = devm_kzalloc(&fn->dev,
			sizeof(u8) * (num_of_irq_regs),
			GFP_KERNEL);
	if (!f01->device_control.interrupt_enable) {
		dev_err(&fn->dev, "Failed to allocate interrupt enable.\n");
		return -ENOMEM;
	}
	fn->data = f01;

	return 0;
}

static int rmi_f01_read_properties(struct rmi_device *rmi_dev,
				   u16 query_base_addr,
				   struct f01_basic_properties *props)
{
	u8 basic_query[RMI_F01_BASIC_QUERY_LEN];
	int error;

	error = rmi_read_block(rmi_dev, query_base_addr,
			       basic_query, sizeof(basic_query));
	if (error < 0) {
		dev_err(&rmi_dev->dev, "Failed to read device query registers.\n");
		return error;
	}

	/* Now parse what we got */
	props->manufacturer_id = basic_query[0];

	props->has_lts = basic_query[1] & RMI_F01_QRY1_HAS_LTS;
	props->has_adjustable_doze =
			basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE;
	props->has_adjustable_doze_holdoff =
			basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE_HOFF;

	snprintf(props->dom, sizeof(props->dom), "20%02d/%02d/%02d",
		 basic_query[5] & RMI_F01_QRY5_YEAR_MASK,
		 basic_query[6] & RMI_F01_QRY6_MONTH_MASK,
		 basic_query[7] & RMI_F01_QRY7_DAY_MASK);

	memcpy(props->product_id, &basic_query[11],
		RMI_PRODUCT_ID_LENGTH);
	props->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

	props->productinfo =
			((basic_query[2] & RMI_F01_QRY2_PRODINFO_MASK) << 7) |
			(basic_query[3] & RMI_F01_QRY2_PRODINFO_MASK);

	return 0;
}

static int rmi_f01_initialize(struct rmi_function *fn)
{
	u8 temp;
	int error;
	u16 ctrl_base_addr;
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	struct f01_data *data = fn->data;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	mutex_init(&data->control_mutex);

	/*
	 * Set the configured bit and (optionally) other important stuff
	 * in the device control register.
	 */
	ctrl_base_addr = fn->fd.control_base_addr;
	error = rmi_read_block(rmi_dev, fn->fd.control_base_addr,
			&data->device_control.ctrl0,
			sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read F01 control.\n");
		return error;
	}
	switch (pdata->power_management.nosleep) {
	case RMI_F01_NOSLEEP_DEFAULT:
		break;
	case RMI_F01_NOSLEEP_OFF:
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;
		break;
	case RMI_F01_NOSLEEP_ON:
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
		break;
	}

	/*
	 * Sleep mode might be set as a hangover from a system crash or
	 * reboot without power cycle.  If so, clear it so the sensor
	 * is certain to function.
	 */
	if ((data->device_control.ctrl0 & RMI_F01_CTRL0_SLEEP_MODE_MASK) !=
			RMI_SLEEP_MODE_NORMAL) {
		dev_warn(&fn->dev,
			 "WARNING: Non-zero sleep mode found. Clearing...\n");
		data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	}

	data->device_control.ctrl0 |= RMI_F01_CRTL0_CONFIGURED_BIT;

	error = rmi_write_block(rmi_dev, fn->fd.control_base_addr,
				&data->device_control.ctrl0,
				sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to write F01 control.\n");
		return error;
	}

	data->irq_count = driver_data->irq_count;
	data->num_of_irq_regs = driver_data->num_of_irq_regs;
	ctrl_base_addr += sizeof(u8);

	data->interrupt_enable_addr = ctrl_base_addr;
	error = rmi_read_block(rmi_dev, ctrl_base_addr,
				data->device_control.interrupt_enable,
				sizeof(u8) * (data->num_of_irq_regs));
	if (error < 0) {
		dev_err(&fn->dev,
			"Failed to read F01 control interrupt enable register.\n");
		return error;
	}

	ctrl_base_addr += data->num_of_irq_regs;

	/* dummy read in order to clear irqs */
	error = rmi_read(rmi_dev, fn->fd.data_base_addr + 1, &temp);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read Interrupt Status.\n");
		return error;
	}

	error = rmi_f01_read_properties(rmi_dev, fn->fd.query_base_addr,
					&data->properties);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read F01 properties.\n");
		return error;
	}
	dev_info(&fn->dev, "found RMI device, manufacturer: %s, product: %s\n",
		 data->properties.manufacturer_id == 1 ?
							"Synaptics" : "unknown",
		 data->properties.product_id);

	/* read control register */
	if (data->properties.has_adjustable_doze) {
		data->doze_interval_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_interval) {
			data->device_control.doze_interval =
				pdata->power_management.doze_interval;
			error = rmi_write(rmi_dev, data->doze_interval_addr,
					data->device_control.doze_interval);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to configure F01 doze interval register.\n");
				return error;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_interval_addr,
					&data->device_control.doze_interval);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 doze interval register.\n");
				return error;
			}
		}

		data->wakeup_threshold_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.wakeup_threshold) {
			data->device_control.wakeup_threshold =
				pdata->power_management.wakeup_threshold;
			error = rmi_write(rmi_dev, data->wakeup_threshold_addr,
					data->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to configure F01 wakeup threshold register.\n");
				return error;
			}
		} else {
			error = rmi_read(rmi_dev, data->wakeup_threshold_addr,
					&data->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 wakeup threshold register.\n");
				return error;
			}
		}
	}

	if (data->properties.has_adjustable_doze_holdoff) {
		data->doze_holdoff_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_holdoff) {
			data->device_control.doze_holdoff =
				pdata->power_management.doze_holdoff;
			error = rmi_write(rmi_dev, data->doze_holdoff_addr,
					data->device_control.doze_holdoff);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to configure F01 doze holdoff register.\n");
				return error;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_holdoff_addr,
					&data->device_control.doze_holdoff);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 doze holdoff register.\n");
				return error;
			}
		}
	}

	error = rmi_read_block(rmi_dev, fn->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read device status.\n");
		return error;
	}

	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_err(&fn->dev,
			"Device was reset during configuration process, status: %#02x!\n",
			RMI_F01_STATUS_CODE(data->device_status));
		return -EINVAL;
	}

	return 0;
}

static int rmi_f01_config(struct rmi_function *fn)
{
	struct f01_data *data = fn->data;
	int retval;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
				 sizeof(data->device_control.ctrl0));
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to write device_control.reg.\n");
		return retval;
	}

	retval = rmi_write_block(fn->rmi_dev, data->interrupt_enable_addr,
				 data->device_control.interrupt_enable,
				 sizeof(u8) * data->num_of_irq_regs);

	if (retval < 0) {
		dev_err(&fn->dev, "Failed to write interrupt enable.\n");
		return retval;
	}
	if (data->properties.has_lts) {
		retval = rmi_write_block(fn->rmi_dev, data->doze_interval_addr,
					 &data->device_control.doze_interval,
					 sizeof(u8));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write doze interval.\n");
			return retval;
		}
	}

	if (data->properties.has_adjustable_doze) {
		retval = rmi_write_block(fn->rmi_dev,
					 data->wakeup_threshold_addr,
					 &data->device_control.wakeup_threshold,
					 sizeof(u8));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write wakeup threshold.\n");
			return retval;
		}
	}

	if (data->properties.has_adjustable_doze_holdoff) {
		retval = rmi_write_block(fn->rmi_dev, data->doze_holdoff_addr,
					 &data->device_control.doze_holdoff,
					 sizeof(u8));
		if (retval < 0) {
			dev_err(&fn->dev, "Failed to write doze holdoff.\n");
			return retval;
		}
	}
	return 0;
}

static int rmi_f01_probe(struct rmi_function *fn)
{
	struct rmi_driver_data *driver_data =
			dev_get_drvdata(&fn->rmi_dev->dev);
	int error;

	error = rmi_f01_alloc_memory(fn, driver_data->num_of_irq_regs);
	if (error)
		return error;

	error = rmi_f01_initialize(fn);
	if (error)
		return error;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rmi_f01_suspend(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f01_data *data = fn->data;
	int error;

	data->old_nosleep = data->device_control.ctrl0 &
					RMI_F01_CRTL0_NOSLEEP_BIT;
	data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= RMI_SLEEP_MODE_SENSOR_SLEEP;

	error = rmi_write_block(rmi_dev,
				fn->fd.control_base_addr,
				&data->device_control.ctrl0,
				sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to write sleep mode. Code: %d.\n",
			error);
		if (data->old_nosleep)
			data->device_control.ctrl0 |=
					RMI_F01_CRTL0_NOSLEEP_BIT;
		data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
		data->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;
		return error;
	}

	return 0;
}

static int rmi_f01_resume(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f01_data *data = fn->data;
	int error;

	if (data->old_nosleep)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;

	error = rmi_write_block(rmi_dev, fn->fd.control_base_addr,
				&data->device_control.ctrl0,
				sizeof(data->device_control.ctrl0));
	if (error < 0) {
		dev_err(&fn->dev,
			"Failed to restore normal operation. Code: %d.\n",
			error);
		return error;
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(rmi_f01_pm_ops, rmi_f01_suspend, rmi_f01_resume);

static int rmi_f01_attention(struct rmi_function *fn,
			     unsigned long *irq_bits)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct f01_data *data = fn->data;
	int retval;

	retval = rmi_read_block(rmi_dev, fn->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (retval < 0) {
		dev_err(&fn->dev, "Failed to read device status, code: %d.\n",
			retval);
		return retval;
	}

	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_warn(&fn->dev, "Device reset detected.\n");
		retval = rmi_dev->driver->reset_handler(rmi_dev);
		if (retval < 0)
			return retval;
	}
	return 0;
}

static struct rmi_function_handler rmi_f01_handler = {
	.driver = {
		.name	= "rmi_f01",
		.pm	= &rmi_f01_pm_ops,
		/*
		 * Do not allow user unbinding F01 as it is critical
		 * function.
		 */
		.suppress_bind_attrs = true,
	},
	.func		= 0x01,
	.probe		= rmi_f01_probe,
	.config		= rmi_f01_config,
	.attention	= rmi_f01_attention,
};

int __init rmi_register_f01_handler(void)
{
	return rmi_register_function_handler(&rmi_f01_handler);
}

void rmi_unregister_f01_handler(void)
{
	rmi_unregister_function_handler(&rmi_f01_handler);
}
