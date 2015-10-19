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
 * @doze_interval - controls the interval between checks for finger presence
 * when the touch sensor is in doze mode, in units of 10ms.
 * @wakeup_threshold - controls the capacitance threshold at which the touch
 * sensor will decide to wake up from that low power state.
 * @doze_holdoff - controls how long the touch sensor waits after the last
 * finger lifts before entering the doze state, in units of 100ms.
 */
struct f01_device_control {
	u8 ctrl0;
	u8 doze_interval;
	u8 wakeup_threshold;
	u8 doze_holdoff;
};

struct f01_data {
	struct f01_basic_properties properties;

	struct f01_device_control device_control;

	u16 doze_interval_addr;
	u16 wakeup_threshold_addr;
	u16 doze_holdoff_addr;

#ifdef CONFIG_PM_SLEEP
	bool suspended;
	bool old_nosleep;
#endif

	unsigned int num_of_irq_regs;
};

static int rmi_f01_read_properties(struct rmi_device *rmi_dev,
				   u16 query_base_addr,
				   struct f01_basic_properties *props)
{
	u8 basic_query[RMI_F01_BASIC_QUERY_LEN];
	int error;

	error = rmi_read_block(rmi_dev, query_base_addr,
			       basic_query, sizeof(basic_query));
	if (error) {
		dev_err(&rmi_dev->dev,
			"Failed to read device query registers: %d\n", error);
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

static int rmi_f01_probe(struct rmi_function *fn)
{
	struct rmi_device *rmi_dev = fn->rmi_dev;
	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	const struct rmi_device_platform_data *pdata = rmi_get_platform_data(rmi_dev);
	struct f01_data *f01;
	int error;
	u16 ctrl_base_addr = fn->fd.control_base_addr;
	u8 device_status;
	u8 temp;

	f01 = devm_kzalloc(&fn->dev, sizeof(struct f01_data), GFP_KERNEL);
	if (!f01) {
		dev_err(&fn->dev, "Failed to allocate fn01_data.\n");
		return -ENOMEM;
	}

	f01->num_of_irq_regs = driver_data->num_of_irq_regs;

	/*
	 * Set the configured bit and (optionally) other important stuff
	 * in the device control register.
	 */

	error = rmi_read(rmi_dev, fn->fd.control_base_addr,
			 &f01->device_control.ctrl0);
	if (error) {
		dev_err(&fn->dev, "Failed to read F01 control: %d\n", error);
		return error;
	}

	switch (pdata->power_management.nosleep) {
	case RMI_F01_NOSLEEP_DEFAULT:
		break;
	case RMI_F01_NOSLEEP_OFF:
		f01->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;
		break;
	case RMI_F01_NOSLEEP_ON:
		f01->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
		break;
	}

	/*
	 * Sleep mode might be set as a hangover from a system crash or
	 * reboot without power cycle.  If so, clear it so the sensor
	 * is certain to function.
	 */
	if ((f01->device_control.ctrl0 & RMI_F01_CTRL0_SLEEP_MODE_MASK) !=
			RMI_SLEEP_MODE_NORMAL) {
		dev_warn(&fn->dev,
			 "WARNING: Non-zero sleep mode found. Clearing...\n");
		f01->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	}

	f01->device_control.ctrl0 |= RMI_F01_CRTL0_CONFIGURED_BIT;

	error = rmi_write(rmi_dev, fn->fd.control_base_addr,
			  f01->device_control.ctrl0);
	if (error) {
		dev_err(&fn->dev, "Failed to write F01 control: %d\n", error);
		return error;
	}

	/* Dummy read in order to clear irqs */
	error = rmi_read(rmi_dev, fn->fd.data_base_addr + 1, &temp);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read Interrupt Status.\n");
		return error;
	}

	error = rmi_f01_read_properties(rmi_dev, fn->fd.query_base_addr,
					&f01->properties);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read F01 properties.\n");
		return error;
	}

	dev_info(&fn->dev, "found RMI device, manufacturer: %s, product: %s\n",
		 f01->properties.manufacturer_id == 1 ? "Synaptics" : "unknown",
		 f01->properties.product_id);

	/* Advance to interrupt control registers, then skip over them. */
	ctrl_base_addr++;
	ctrl_base_addr += f01->num_of_irq_regs;

	/* read control register */
	if (f01->properties.has_adjustable_doze) {
		f01->doze_interval_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_interval) {
			f01->device_control.doze_interval =
				pdata->power_management.doze_interval;
			error = rmi_write(rmi_dev, f01->doze_interval_addr,
					  f01->device_control.doze_interval);
			if (error) {
				dev_err(&fn->dev,
					"Failed to configure F01 doze interval register: %d\n",
					error);
				return error;
			}
		} else {
			error = rmi_read(rmi_dev, f01->doze_interval_addr,
					 &f01->device_control.doze_interval);
			if (error) {
				dev_err(&fn->dev,
					"Failed to read F01 doze interval register: %d\n",
					error);
				return error;
			}
		}

		f01->wakeup_threshold_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.wakeup_threshold) {
			f01->device_control.wakeup_threshold =
				pdata->power_management.wakeup_threshold;
			error = rmi_write(rmi_dev, f01->wakeup_threshold_addr,
					  f01->device_control.wakeup_threshold);
			if (error) {
				dev_err(&fn->dev,
					"Failed to configure F01 wakeup threshold register: %d\n",
					error);
				return error;
			}
		} else {
			error = rmi_read(rmi_dev, f01->wakeup_threshold_addr,
					 &f01->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn->dev,
					"Failed to read F01 wakeup threshold register: %d\n",
					error);
				return error;
			}
		}
	}

	if (f01->properties.has_lts)
		ctrl_base_addr++;

	if (f01->properties.has_adjustable_doze_holdoff) {
		f01->doze_holdoff_addr = ctrl_base_addr;
		ctrl_base_addr++;

		if (pdata->power_management.doze_holdoff) {
			f01->device_control.doze_holdoff =
				pdata->power_management.doze_holdoff;
			error = rmi_write(rmi_dev, f01->doze_holdoff_addr,
					  f01->device_control.doze_holdoff);
			if (error) {
				dev_err(&fn->dev,
					"Failed to configure F01 doze holdoff register: %d\n",
					error);
				return error;
			}
		} else {
			error = rmi_read(rmi_dev, f01->doze_holdoff_addr,
					 &f01->device_control.doze_holdoff);
			if (error) {
				dev_err(&fn->dev,
					"Failed to read F01 doze holdoff register: %d\n",
					error);
				return error;
			}
		}
	}

	error = rmi_read(rmi_dev, fn->fd.data_base_addr, &device_status);
	if (error < 0) {
		dev_err(&fn->dev,
			"Failed to read device status: %d\n", error);
		return error;
	}

	if (RMI_F01_STATUS_UNCONFIGURED(device_status)) {
		dev_err(&fn->dev,
			"Device was reset during configuration process, status: %#02x!\n",
			RMI_F01_STATUS_CODE(device_status));
		return -EINVAL;
	}

	dev_set_drvdata(&fn->dev, f01);

	return 0;
}

static int rmi_f01_config(struct rmi_function *fn)
{
	struct f01_data *f01 = dev_get_drvdata(&fn->dev);
	int error;

	error = rmi_write(fn->rmi_dev, fn->fd.control_base_addr,
			  f01->device_control.ctrl0);
	if (error) {
		dev_err(&fn->dev,
			"Failed to write device_control register: %d\n", error);
		return error;
	}

	if (f01->properties.has_adjustable_doze) {
		error = rmi_write(fn->rmi_dev, f01->doze_interval_addr,
				  f01->device_control.doze_interval);
		if (error) {
			dev_err(&fn->dev,
				"Failed to write doze interval: %d\n", error);
			return error;
		}

		error = rmi_write_block(fn->rmi_dev,
					 f01->wakeup_threshold_addr,
					 &f01->device_control.wakeup_threshold,
					 sizeof(u8));
		if (error) {
			dev_err(&fn->dev,
				"Failed to write wakeup threshold: %d\n",
				error);
			return error;
		}
	}

	if (f01->properties.has_adjustable_doze_holdoff) {
		error = rmi_write(fn->rmi_dev, f01->doze_holdoff_addr,
				  f01->device_control.doze_holdoff);
		if (error) {
			dev_err(&fn->dev,
				"Failed to write doze holdoff: %d\n", error);
			return error;
		}
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rmi_f01_suspend(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *f01 = dev_get_drvdata(&fn->dev);
	int error;

	f01->old_nosleep =
		f01->device_control.ctrl0 & RMI_F01_CRTL0_NOSLEEP_BIT;
	f01->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;

	f01->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	f01->device_control.ctrl0 |= RMI_SLEEP_MODE_SENSOR_SLEEP;

	error = rmi_write(fn->rmi_dev, fn->fd.control_base_addr,
			  f01->device_control.ctrl0);
	if (error) {
		dev_err(&fn->dev, "Failed to write sleep mode: %d.\n", error);
		if (f01->old_nosleep)
			f01->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
		f01->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
		f01->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;
		return error;
	}

	return 0;
}

static int rmi_f01_resume(struct device *dev)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *f01 = dev_get_drvdata(&fn->dev);
	int error;

	if (f01->old_nosleep)
		f01->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;

	f01->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	f01->device_control.ctrl0 |= RMI_SLEEP_MODE_NORMAL;

	error = rmi_write(fn->rmi_dev, fn->fd.control_base_addr,
			  f01->device_control.ctrl0);
	if (error) {
		dev_err(&fn->dev,
			"Failed to restore normal operation: %d.\n", error);
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
	int error;
	u8 device_status;

	error = rmi_read(rmi_dev, fn->fd.data_base_addr, &device_status);
	if (error) {
		dev_err(&fn->dev,
			"Failed to read device status: %d.\n", error);
		return error;
	}

	if (RMI_F01_STATUS_UNCONFIGURED(device_status)) {
		dev_warn(&fn->dev, "Device reset detected.\n");
		error = rmi_dev->driver->reset_handler(rmi_dev);
		if (error) {
			dev_err(&fn->dev, "Device reset failed: %d\n", error);
			return error;
		}
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
