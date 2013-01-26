/*
 * Copyright (c) 2011-2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
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
	char dom[9]; /* YYYYMMDD + '\0' */
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
	u16 productinfo;
};

/* F01 device status bits */

/* Most recent device status event */
#define RMI_F01_STATUS_CODE(status)		((status) & 0x0f)
/* Indicates that flash programming is enabled (bootloader mode). */
#define RMI_F01_STATUS_BOOTLOADER(status)	(!!((status) & 0x40))
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

#ifdef CONFIG_PM
	bool suspended;
	bool old_nosleep;
#endif

#ifdef CONFIG_RMI4_DEBUG
	struct dentry *debugfs_interrupt_enable;
#endif
};

#ifdef CONFIG_RMI4_DEBUG
struct f01_debugfs_data {
	bool done;
	struct rmi_function *fn;
};

static int f01_debug_open(struct inode *inodep, struct file *filp)
{
	struct f01_debugfs_data *data;
	struct rmi_function *fn = inodep->i_private;

	data = kzalloc(sizeof(struct f01_debugfs_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->fn = fn;
	filp->private_data = data;
	return 0;
}

static int f01_debug_release(struct inode *inodep, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static ssize_t interrupt_enable_read(struct file *filp, char __user *buffer,
				     size_t size, loff_t *offset) {
	int i;
	int len;
	int total_len = 0;
	char local_buf[size]; // FIXME!!!! XXX arbitrary size array on stack
	char *current_buf = local_buf;
	struct f01_debugfs_data *data = filp->private_data;
	struct f01_data *f01 = data->fn->data;

	if (data->done)
		return 0;

	data->done = 1;

	/* loop through each irq value and copy its
	 * string representation into buf */
	for (i = 0; i < f01->irq_count; i++) {
		int irq_reg;
		int irq_shift;
		int interrupt_enable;

		irq_reg = i / 8;
		irq_shift = i % 8;
		interrupt_enable =
		    ((f01->device_control.interrupt_enable[irq_reg]
			>> irq_shift) & 0x01);

		/* get next irq value and write it to buf */
		len = snprintf(current_buf, size - total_len,
			"%u ", interrupt_enable);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(&data->fn->dev, "Failed to build interrupt_enable buffer, code = %d.\n",
						len);
			return snprintf(local_buf, size, "unknown\n");
		}
	}
	len = snprintf(current_buf, size - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(&data->fn->dev, "%s: Failed to append carriage return.\n",
			 __func__);

	if (copy_to_user(buffer, local_buf, total_len))
		return -EFAULT;

	return total_len;
}

static ssize_t interrupt_enable_write(struct file *filp,
		const char __user *buffer, size_t size, loff_t *offset) {
	int retval;
	char buf[size];
	char *local_buf = buf;
	int i;
	int irq_count = 0;
	int irq_reg = 0;
	struct f01_debugfs_data *data = filp->private_data;
	struct f01_data *f01 = data->fn->data;

	retval = copy_from_user(buf, buffer, size);
	if (retval)
		return -EFAULT;

	for (i = 0; i < f01->irq_count && *local_buf != 0;
	     i++, local_buf += 2) {
		int irq_shift;
		int interrupt_enable;
		int result;

		irq_reg = i / 8;
		irq_shift = i % 8;

		/* get next interrupt mapping value and store and bump up to
		 * point to next item in local_buf */
		result = sscanf(local_buf, "%u", &interrupt_enable);
		if ((result != 1) ||
			(interrupt_enable != 0 && interrupt_enable != 1)) {
			dev_err(&data->fn->dev, "Interrupt enable[%d] is not a valid value 0x%x.\n",
				i, interrupt_enable);
			return -EINVAL;
		}
		if (interrupt_enable == 0) {
			f01->device_control.interrupt_enable[irq_reg] &=
				(1 << irq_shift) ^ 0xFF;
		} else
			f01->device_control.interrupt_enable[irq_reg] |=
				(1 << irq_shift);
		irq_count++;
	}

	/* Make sure the irq count matches */
	if (irq_count != f01->irq_count) {
		dev_err(&data->fn->dev, "Interrupt enable count of %d doesn't match device count of %d.\n",
			 irq_count, f01->irq_count);
		return -EINVAL;
	}

	/* write back to the control register */
	retval = rmi_write_block(data->fn->rmi_dev, f01->interrupt_enable_addr,
			f01->device_control.interrupt_enable,
			f01->num_of_irq_regs);
	if (retval < 0) {
		dev_err(&data->fn->dev, "Could not write interrupt_enable mask to %#06x\n",
			f01->interrupt_enable_addr);
		return retval;
	}

	return size;
}

static const struct file_operations interrupt_enable_fops = {
	.owner = THIS_MODULE,
	.open = f01_debug_open,
	.release = f01_debug_release,
	.read = interrupt_enable_read,
	.write = interrupt_enable_write,
};

static int setup_debugfs(struct rmi_function *fn)
{
	struct f01_data *data = fn->data;

	if (!fn->debugfs_root)
		return -ENODEV;

	data->debugfs_interrupt_enable = debugfs_create_file("interrupt_enable",
		RMI_RW_ATTR, fn->debugfs_root, fn, &interrupt_enable_fops);
	if (!data->debugfs_interrupt_enable)
		dev_warn(&fn->dev,
			 "Failed to create debugfs interrupt_enable.\n");

	return 0;
}

static void teardown_debugfs(struct f01_data *f01)
{
	if (f01->debugfs_interrupt_enable)
		debugfs_remove(f01->debugfs_interrupt_enable);
}

#else

static inline int setup_debugfs(struct rmi_function *fn)
{
	return 0;
}

static inline void teardown_debugfs(struct f01_data *f01)
{
}

#endif

static ssize_t rmi_fn_01_productinfo_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%04x\n",
			data->properties.productinfo);
}

static ssize_t rmi_fn_01_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%s\n", data->properties.product_id);
}

static ssize_t rmi_fn_01_manufacturer_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n",
			data->properties.manufacturer_id);
}

static ssize_t rmi_fn_01_datecode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%s\n", data->properties.dom);
}

static ssize_t rmi_fn_01_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	unsigned int reset;
	int error;

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;
	if (reset < 0 || reset > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (reset) {
		/* Command register always reads as 0, so just use a local. */
		u8 command = RMI_F01_CMD_DEVICE_RESET;

		error = rmi_write_block(fn->rmi_dev, fn->fd.command_base_addr,
					 &command, sizeof(command));
		if (error < 0) {
			dev_err(dev, "Failed to issue reset command, code = %d.",
				error);
			return error;
		}
	}

	return count;
}

static ssize_t rmi_fn_01_sleepmode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = data->device_control.ctrl0 &
					RMI_F01_CTRL0_SLEEP_MODE_MASK;

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_sleepmode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || !RMI_IS_VALID_SLEEPMODE(new_value)) {
		dev_err(dev, "%s: Invalid sleep mode %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	dev_dbg(dev, "Setting sleep mode to %ld.", new_value);

	data->device_control.ctrl0 &= ~RMI_F01_CTRL0_SLEEP_MODE_MASK;
	data->device_control.ctrl0 |= new_value;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
			&data->device_control.ctrl0,
			sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write sleep mode, code %d.\n", retval);

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_nosleep_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = !!(data->device_control.ctrl0 &
					RMI_F01_CRTL0_NOSLEEP_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_nosleep_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid nosleep bit %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	if (new_value)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_NOSLEEP_BIT;
	else
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_NOSLEEP_BIT;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
				 sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write nosleep bit.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_chargerinput_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = !!(data->device_control.ctrl0 &
					RMI_F01_CRTL0_CHARGER_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_chargerinput_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid chargerinput bit %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	if (new_value)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_CHARGER_BIT;
	else
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_CHARGER_BIT;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
				 sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write chargerinput bit.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_reportrate_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	int value = !!(data->device_control.ctrl0 &
				RMI_F01_CRTL0_REPORTRATE_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_reportrate_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid reportrate bit %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	if (new_value)
		data->device_control.ctrl0 |= RMI_F01_CRTL0_REPORTRATE_BIT;
	else
		data->device_control.ctrl0 &= ~RMI_F01_CRTL0_REPORTRATE_BIT;

	retval = rmi_write_block(fn->rmi_dev, fn->fd.control_base_addr,
				 &data->device_control.ctrl0,
				 sizeof(data->device_control.ctrl0));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write reportrate bit.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_interrupt_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	int i, len, total_len = 0;
	char *current_buf = buf;

	/* loop through each irq value and copy its
	 * string representation into buf */
	for (i = 0; i < data->irq_count; i++) {
		int irq_reg;
		int irq_shift;
		int interrupt_enable;

		irq_reg = i / 8;
		irq_shift = i % 8;
		interrupt_enable =
		    ((data->device_control.interrupt_enable[irq_reg]
			>> irq_shift) & 0x01);

		/* get next irq value and write it to buf */
		len = snprintf(current_buf, PAGE_SIZE - total_len,
			"%u ", interrupt_enable);
		/* bump up ptr to next location in buf if the
		 * snprintf was valid.  Otherwise issue an error
		 * and return. */
		if (len > 0) {
			current_buf += len;
			total_len += len;
		} else {
			dev_err(dev, "Failed to build interrupt_enable buffer, code = %d.\n",
						len);
			return snprintf(buf, PAGE_SIZE, "unknown\n");
		}
	}
	len = snprintf(current_buf, PAGE_SIZE - total_len, "\n");
	if (len > 0)
		total_len += len;
	else
		dev_warn(dev, "%s: Failed to append carriage return.\n",
			 __func__);
	return total_len;

}

static ssize_t rmi_fn_01_doze_interval_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.doze_interval);

}

static ssize_t rmi_fn_01_doze_interval_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;
	u16 ctrl_base_addr;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid doze interval %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	data->device_control.doze_interval = new_value;
	ctrl_base_addr = fn->fd.control_base_addr + sizeof(u8) +
			(sizeof(u8)*(data->num_of_irq_regs));
	dev_dbg(dev, "doze_interval store address %x, value %d",
		ctrl_base_addr, data->device_control.doze_interval);

	retval = rmi_write_block(fn->rmi_dev, data->doze_interval_addr,
			&data->device_control.doze_interval,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write doze interval.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_wakeup_threshold_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.wakeup_threshold);
}

static ssize_t rmi_fn_01_wakeup_threshold_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid wakeup threshold %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	data->device_control.doze_interval = new_value;
	retval = rmi_write_block(fn->rmi_dev, data->wakeup_threshold_addr,
			&data->device_control.wakeup_threshold,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write wakeup threshold.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_doze_holdoff_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			data->device_control.doze_holdoff);

}

static ssize_t rmi_fn_01_doze_holdoff_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value > 255) {
		dev_err(dev, "%s: Invalid doze holdoff %s.", __func__, buf);
		return -EINVAL;
	}

	retval = mutex_lock_interruptible(&data->control_mutex);
	if (retval)
		return retval;

	data->device_control.doze_interval = new_value;
	retval = rmi_write_block(fn->rmi_dev, data->doze_holdoff_addr,
			&data->device_control.doze_holdoff,
			sizeof(u8));
	if (retval >= 0)
		retval = count;
	else
		dev_err(dev, "Failed to write doze holdoff.\n");

	mutex_unlock(&data->control_mutex);
	return retval;
}

static ssize_t rmi_fn_01_configured_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	unsigned int value = !!(data->device_control.ctrl0 &
					RMI_F01_CRTL0_CONFIGURED_BIT);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t rmi_fn_01_unconfigured_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			RMI_F01_STATUS_UNCONFIGURED(data->device_status));
}

static ssize_t rmi_fn_01_flashprog_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			RMI_F01_STATUS_BOOTLOADER(data->device_status));
}

static ssize_t rmi_fn_01_statuscode_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n",
			RMI_F01_STATUS_CODE(data->device_status));
}

#define RMI_F01_ATTR(_name)			\
	DEVICE_ATTR(_name, RMI_RW_ATTR,		\
		    rmi_fn_01_##_name##_show,	\
		    rmi_fn_01_##_name##_store)

#define RMI_F01_RO_ATTR(_name)			\
	DEVICE_ATTR(_name, RMI_RO_ATTR,		\
		    rmi_fn_01_##_name##_show,	\
		    NULL)

#define RMI_F01_WO_ATTR(_name)			\
	DEVICE_ATTR(_name, RMI_RO_ATTR,		\
		    NULL,			\
		    rmi_fn_01_##_name##_store)


static RMI_F01_RO_ATTR(productinfo);
static RMI_F01_RO_ATTR(productid);
static RMI_F01_RO_ATTR(manufacturer);
static RMI_F01_RO_ATTR(datecode);

/* Control register access */
static RMI_F01_ATTR(sleepmode);
static RMI_F01_ATTR(nosleep);
static RMI_F01_ATTR(chargerinput);
static RMI_F01_ATTR(reportrate);

/*
 * We don't want arbitrary callers changing the interrupt enable mask,
 * so it's read only.
 */
static RMI_F01_RO_ATTR(interrupt_enable);
static RMI_F01_ATTR(doze_interval);
static RMI_F01_ATTR(wakeup_threshold);
static RMI_F01_ATTR(doze_holdoff);

/*
 * We make 'configured' RO, since the driver uses that to look for
 * resets.  We don't want someone faking us out by changing that
 * bit.
 */
static RMI_F01_RO_ATTR(configured);

/* Command register access. */
static RMI_F01_WO_ATTR(reset);

/* Status register access. */
static RMI_F01_RO_ATTR(unconfigured);
static RMI_F01_RO_ATTR(flashprog);
static RMI_F01_RO_ATTR(statuscode);

static struct attribute *rmi_fn_01_attrs[] = {
	&dev_attr_productinfo.attr,
	&dev_attr_productid.attr,
	&dev_attr_manufacturer.attr,
	&dev_attr_datecode.attr,
	&dev_attr_sleepmode.attr,
	&dev_attr_nosleep.attr,
	&dev_attr_chargerinput.attr,
	&dev_attr_reportrate.attr,
	&dev_attr_interrupt_enable.attr,
	&dev_attr_doze_interval.attr,
	&dev_attr_wakeup_threshold.attr,
	&dev_attr_doze_holdoff.attr,
	&dev_attr_configured.attr,
	&dev_attr_reset.attr,
	&dev_attr_unconfigured.attr,
	&dev_attr_flashprog.attr,
	&dev_attr_statuscode.attr,
	NULL
};

static umode_t rmi_fn_01_attr_visible(struct kobject *kobj,
				      struct attribute *attr, int n)
{
	struct device *dev = kobj_to_dev(kobj);
	struct rmi_function *fn = to_rmi_function(dev);
	struct f01_data *data = fn->data;
	umode_t mode = attr->mode;

	if (attr == &dev_attr_doze_interval.attr) {
		if (!data->properties.has_lts)
			mode = 0;
	} else if (attr == &dev_attr_wakeup_threshold.attr) {
		if (!data->properties.has_adjustable_doze)
			mode = 0;
	} else if (attr == &dev_attr_doze_holdoff.attr) {
		if (!data->properties.has_adjustable_doze_holdoff)
			mode = 0;
	}

	return mode;
}

static struct attribute_group rmi_fn_01_attr_group = {
	.is_visible	= rmi_fn_01_attr_visible,
	.attrs		= rmi_fn_01_attrs,
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
			sizeof(u8)*(num_of_irq_regs),
			GFP_KERNEL);
	if (!f01->device_control.interrupt_enable) {
		dev_err(&fn->dev, "Failed to allocate interrupt enable.\n");
		return -ENOMEM;
	}
	fn->data = f01;

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
	u8 basic_query[RMI_F01_BASIC_QUERY_LEN];

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
		goto error_exit;
	}

	ctrl_base_addr += data->num_of_irq_regs;

	/* dummy read in order to clear irqs */
	error = rmi_read(rmi_dev, fn->fd.data_base_addr + 1, &temp);
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read Interrupt Status.\n");
		return error;
	}

	error = rmi_read_block(rmi_dev, fn->fd.query_base_addr,
			       basic_query, sizeof(basic_query));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read device query registers.\n");
		return error;
	}

	/* Now parse what we got */
	data->properties.manufacturer_id = basic_query[0];

	data->properties.has_lts = basic_query[1] & RMI_F01_QRY1_HAS_LTS;
	data->properties.has_adjustable_doze =
			basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE;
	data->properties.has_adjustable_doze_holdoff =
			basic_query[1] & RMI_F01_QRY1_HAS_ADJ_DOZE_HOFF;

	snprintf(data->properties.dom, sizeof(data->properties.dom),
		 "20%02x%02x%02x",
		 basic_query[5] & RMI_F01_QRY5_YEAR_MASK,
		 basic_query[6] & RMI_F01_QRY6_MONTH_MASK,
		 basic_query[7] & RMI_F01_QRY7_DAY_MASK);

	memcpy(data->properties.product_id, &basic_query[11],
		RMI_PRODUCT_ID_LENGTH);
	data->properties.product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

	data->properties.productinfo =
			((basic_query[2] & RMI_F01_QRY2_PRODINFO_MASK) << 7) |
			(basic_query[3] & RMI_F01_QRY2_PRODINFO_MASK);

	dev_info(&fn->dev, "found RMI device, manufacturer: %s, product: %s\n",
		 data->properties.manufacturer_id == 1 ?
							"synaptics" : "unknown",
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
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_interval_addr,
					&data->device_control.doze_interval);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 doze interval register.\n");
				goto error_exit;
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
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->wakeup_threshold_addr,
					&data->device_control.wakeup_threshold);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 wakeup threshold register.\n");
				goto error_exit;
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
				goto error_exit;
			}
		} else {
			error = rmi_read(rmi_dev, data->doze_holdoff_addr,
					&data->device_control.doze_holdoff);
			if (error < 0) {
				dev_err(&fn->dev, "Failed to read F01 doze holdoff register.\n");
				goto error_exit;
			}
		}
	}

	error = rmi_read_block(rmi_dev, fn->fd.data_base_addr,
		&data->device_status, sizeof(data->device_status));
	if (error < 0) {
		dev_err(&fn->dev, "Failed to read device status.\n");
		goto error_exit;
	}

	driver_data->f01_bootloader_mode =
			RMI_F01_STATUS_BOOTLOADER(data->device_status);
	if (driver_data->f01_bootloader_mode)
		dev_warn(&rmi_dev->dev,
			 "WARNING: RMI4 device is in bootloader mode!\n");


	if (RMI_F01_STATUS_UNCONFIGURED(data->device_status)) {
		dev_err(&fn->dev,
			"Device was reset during configuration process, status: %#02x!\n",
			RMI_F01_STATUS_CODE(data->device_status));
		error = -EINVAL;
		goto error_exit;
	}

	error = setup_debugfs(fn);
	if (error)
		dev_warn(&fn->dev, "Failed to setup debugfs, error: %d.\n",
			 error);

	return 0;

 error_exit:
	kfree(data);
	return error;
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

	error = sysfs_create_group(&fn->dev.kobj, &rmi_fn_01_attr_group);
	if (error)
		return error;

	return 0;
}

static void rmi_f01_remove(struct rmi_function *fn)
{
	teardown_debugfs(fn->data);
	sysfs_remove_group(&fn->dev.kobj, &rmi_fn_01_attr_group);
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
	.remove		= rmi_f01_remove,
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
