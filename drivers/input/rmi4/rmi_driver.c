/*
 * Copyright (c) 2011-2013 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This driver provides the core support for a single RMI4-based device.
 *
 * The RMI4 specification can be found here (URL split for line length):
 *
 * http://www.synaptics.com/sites/default/files/
 *      511-000136-01-Rev-E-RMI4%20Intrfacing%20Guide.pdf
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/kconfig.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <uapi/linux/input.h>
#include "rmi_bus.h"
#include "rmi_driver.h"

#define HAS_NONSTANDARD_PDT_MASK 0x40
#define RMI4_MAX_PAGE 0xff
#define RMI4_PAGE_SIZE 0x100

#define RMI_DEVICE_RESET_CMD	0x01
#define DEFAULT_RESET_DELAY_MS	100

#define DEFAULT_POLL_INTERVAL_MS	13

#define IRQ_DEBUG(data) (IS_ENABLED(CONFIG_RMI4_DEBUG) && data->irq_debug)

static irqreturn_t rmi_irq_thread(int irq, void *p)
{
	struct rmi_phys_device *phys = p;
	struct rmi_device *rmi_dev = phys->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;
	struct rmi_device_platform_data *pdata = phys->dev->platform_data;
	struct rmi_driver_data *data;

	data = dev_get_drvdata(&rmi_dev->dev);

	if (IRQ_DEBUG(data))
		dev_dbg(phys->dev, "ATTN gpio, value: %d.\n",
				gpio_get_value(pdata->attn_gpio));

	if (gpio_get_value(pdata->attn_gpio) == pdata->attn_polarity) {
		data->attn_count++;
		if (driver && driver->irq_handler && rmi_dev)
			driver->irq_handler(rmi_dev, irq);
	}

	return IRQ_HANDLED;
}

static int process_interrupt_requests(struct rmi_device *rmi_dev);

static void rmi_poll_work(struct work_struct *work)
{
	struct rmi_driver_data *data =
			container_of(work, struct rmi_driver_data, poll_work);
	struct rmi_device *rmi_dev = data->rmi_dev;

	process_interrupt_requests(rmi_dev);
}

/* This is the timer function for polling - it simply has to schedule work
 * and restart the timer. */
static enum hrtimer_restart rmi_poll_timer(struct hrtimer *timer)
{
	struct rmi_driver_data *data =
			container_of(timer, struct rmi_driver_data, poll_timer);

	if (!data->enabled)
		return HRTIMER_NORESTART;
	if (!work_pending(&data->poll_work))
		schedule_work(&data->poll_work);
	hrtimer_start(&data->poll_timer, data->poll_interval, HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static int enable_polling(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	dev_dbg(&rmi_dev->dev, "Polling enabled.\n");
	INIT_WORK(&data->poll_work, rmi_poll_work);
	hrtimer_init(&data->poll_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->poll_timer.function = rmi_poll_timer;
	hrtimer_start(&data->poll_timer, data->poll_interval, HRTIMER_MODE_REL);

	return 0;
}

static void disable_polling(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	dev_dbg(&rmi_dev->dev, "Polling disabled.\n");
	hrtimer_cancel(&data->poll_timer);
	cancel_work_sync(&data->poll_work);
}

static void disable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	if (!data->enabled)
		return;

	if (!data->irq)
		disable_polling(rmi_dev);

	if (rmi_dev->phys->disable_device)
		rmi_dev->phys->disable_device(rmi_dev->phys);

	if (data->irq) {
		disable_irq(data->irq);
		free_irq(data->irq, rmi_dev->phys);
	}

	data->enabled = false;
}

static int enable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_phys_device *rmi_phys;
	int retval = 0;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	if (data->enabled)
		return 0;

	if (rmi_dev->phys->enable_device) {
		retval = rmi_dev->phys->enable_device(rmi_dev->phys);
		if (retval)
			return retval;
	}

	rmi_phys = rmi_dev->phys;
	if (data->irq) {
		retval = request_threaded_irq(data->irq,
				rmi_phys->hard_irq ? rmi_phys->hard_irq : NULL,
				rmi_phys->irq_thread ?
					rmi_phys->irq_thread : rmi_irq_thread,
				data->irq_flags,
				dev_name(&rmi_dev->dev), rmi_phys);
		if (retval)
			return retval;
	} else {
		retval = enable_polling(rmi_dev);
		if (retval < 0)
			return retval;
	}

	data->enabled = true;

	if (!pdata->level_triggered &&
		    gpio_get_value(pdata->attn_gpio) == pdata->attn_polarity)
		retval = process_interrupt_requests(rmi_dev);

	return retval;
}

static void rmi_free_function_list(struct rmi_device *rmi_dev)
{
	struct rmi_function *fn, *tmp;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	data->f01_container = NULL;

	/* Doing it in the reverse order so F01 will be removed last */
	list_for_each_entry_safe_reverse(fn, tmp,
					 &data->function_list, node) {
		list_del(&fn->node);
		rmi_unregister_function(fn);
	}
}

static int reset_one_function(struct rmi_function *fn)
{
	struct rmi_function_handler *fh;
	int retval = 0;

	if (!fn || !fn->dev.driver)
		return 0;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fh->reset) {
		retval = fh->reset(fn);
		if (retval < 0)
			dev_err(&fn->dev, "Reset failed with code %d.\n",
				retval);
	}

	return retval;
}

static int configure_one_function(struct rmi_function *fn)
{
	struct rmi_function_handler *fh;
	int retval = 0;

	if (!fn || !fn->dev.driver)
		return 0;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fh->config) {
		retval = fh->config(fn);
		if (retval < 0)
			dev_err(&fn->dev, "Config failed with code %d.\n",
				retval);
	}

	return retval;
}

static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *entry;
	int retval;

	list_for_each_entry(entry, &data->function_list, node) {
		retval = reset_one_function(entry);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function *entry;
	int retval;

	list_for_each_entry(entry, &data->function_list, node) {
		retval = configure_one_function(entry);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static void process_one_interrupt(struct rmi_function *fn,
		unsigned long *irq_status, struct rmi_driver_data *data)
{
	struct rmi_function_handler *fh;
	DECLARE_BITMAP(irq_bits, data->num_of_irq_regs);

	if (!fn || !fn->dev.driver)
		return;

	fh = to_rmi_function_handler(fn->dev.driver);
	if (fn->irq_mask && fh->attention) {
		bitmap_and(irq_bits, irq_status, fn->irq_mask,
				data->irq_count);
		if (!bitmap_empty(irq_bits, data->irq_count))
			fh->attention(fn, irq_bits);
	}
}

static int process_interrupt_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;
	struct rmi_function *entry;
	int error;

	error = rmi_read_block(rmi_dev,
				data->f01_container->fd.data_base_addr + 1,
				data->irq_status, data->num_of_irq_regs);
	if (error < 0) {
		dev_err(dev, "Failed to read irqs, code=%d\n", error);
		return error;
	}

	mutex_lock(&data->irq_mutex);
	bitmap_and(data->irq_status, data->irq_status, data->current_irq_mask,
	       data->irq_count);
	/* At this point, irq_status has all bits that are set in the
	 * interrupt status register and are enabled.
	 */
	mutex_unlock(&data->irq_mutex);

	/* It would be nice to be able to use irq_chip to handle these
	 * nested IRQs.  Unfortunately, most of the current customers for
	 * this driver are using older kernels (3.0.x) that don't support
	 * the features required for that.  Once they've shifted to more
	 * recent kernels (say, 3.3 and higher), this should be switched to
	 * use irq_chip.
	 */
	list_for_each_entry(entry, &data->function_list, node) {
		if (entry->irq_mask)
			process_one_interrupt(entry, data->irq_status,
					      data);
	}

	return 0;
}

/**
 * rmi_driver_set_input_params - set input device id and other data.
 *
 * @rmi_dev: Pointer to an RMI device
 * @input: Pointer to input device
 *
 */
static int rmi_driver_set_input_params(struct rmi_device *rmi_dev,
				struct input_dev *input)
{
	// FIXME: set up parent
	input->name = SYNAPTICS_INPUT_DEVICE_NAME;
	input->id.vendor  = SYNAPTICS_VENDOR_ID;
	input->id.bustype = BUS_RMI;
	return 0;
}

/**
 * This pair of functions allows functions like function 54 to request to have
 * other interrupts disabled until the restore function is called. Only one
 * store happens at a time.
 */
static int rmi_driver_irq_save(struct rmi_device *rmi_dev,
				unsigned long *new_ints)
{
	int retval = 0;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);
	if (!data->irq_stored) {
		/* Save current enabled interrupts */
		retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to read enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		/*
		 * Disable every interrupt except for function 54
		 * TODO:Will also want to not disable function 1-like functions.
		 * No need to take care of this now, since there's no good way
		 * to identify them.
		 */
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				new_ints, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to change enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		bitmap_copy(data->current_irq_mask, new_ints, data->irq_count);
		data->irq_stored = true;
	} else {
		retval = -ENOSPC; /* No space to store IRQs.*/
		dev_err(dev, "Attempted to save IRQs when already stored!");
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_irq_restore(struct rmi_device *rmi_dev)
{
	int retval = 0;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);

	if (data->irq_stored) {
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to write enabled interupts!",
								__func__);
			goto error_unlock;
		}
		memcpy(data->current_irq_mask, data->irq_mask_store,
					data->num_of_irq_regs * sizeof(u8));
		data->irq_stored = false;
	} else {
		retval = -EINVAL;
		dev_err(dev, "%s: Attempted to restore values when not stored!",
			__func__);
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_irq_handler(struct rmi_device *rmi_dev, int irq)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	might_sleep();
	/* Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container) {
		dev_dbg(&rmi_dev->dev,
			 "Not ready to handle interrupts yet!\n");
		return 0;
	}

	return process_interrupt_requests(rmi_dev);
}

static int rmi_driver_reset_handler(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	int error;

	/* Can get called before the driver is fully ready to deal with
	 * this situation.
	 */
	if (!data || !data->f01_container) {
		dev_warn(&rmi_dev->dev,
			 "Not ready to handle reset yet!\n");
		return 0;
	}

	error = rmi_driver_process_reset_requests(rmi_dev);
	if (error < 0)
		return error;


	error = rmi_driver_process_config_requests(rmi_dev);
	if (error < 0)
		return error;

	if (data->irq_stored) {
		error = rmi_driver_irq_restore(rmi_dev);
		if (error < 0)
			return error;
	}

	return 0;
}

/*
 * Construct a function's IRQ mask. This should be called once and stored.
 */
int rmi_driver_irq_get_mask(struct rmi_device *rmi_dev,
		struct rmi_function *fn) {
	int i;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);

	/* call devm_kcalloc when it will be defined in kernel in future */
	fn->irq_mask = devm_kzalloc(&rmi_dev->dev,
			BITS_TO_LONGS(data->irq_count)*sizeof(unsigned long),
			GFP_KERNEL);

	if (fn->irq_mask) {
		for (i = 0; i < fn->num_of_irqs; i++)
			set_bit(fn->irq_pos+i, fn->irq_mask);
		return 0;
	} else
		return -ENOMEM;
}

static void rmi_driver_copy_pdt_to_fd(struct pdt_entry *pdt,
				      struct rmi_function_descriptor *fd,
				      u16 page_start)
{
	fd->query_base_addr = pdt->query_base_addr + page_start;
	fd->command_base_addr = pdt->command_base_addr + page_start;
	fd->control_base_addr = pdt->control_base_addr + page_start;
	fd->data_base_addr = pdt->data_base_addr + page_start;
	fd->function_number = pdt->function_number;
	fd->interrupt_source_count = pdt->interrupt_source_count;
	fd->function_version = pdt->function_version;
}

static int create_function(struct rmi_device *rmi_dev,
				     struct pdt_entry *pdt,
				     int *current_irq_count,
				     u16 page_start)
{
	struct device *dev = &rmi_dev->dev;
	struct rmi_driver_data *data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);
	struct rmi_function *fn;
	int error;

	dev_dbg(dev, "Initializing F%02X for %s.\n",
		pdt->function_number, pdata->sensor_name);

	fn = kzalloc(sizeof(struct rmi_function), GFP_KERNEL);
	if (!fn) {
		dev_err(dev, "Failed to allocate memory for F%02X\n",
			pdt->function_number);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&fn->node);

	fn->rmi_dev = rmi_dev;
	fn->num_of_irqs = pdt->interrupt_source_count;

	fn->irq_pos = *current_irq_count;
	*current_irq_count += fn->num_of_irqs;

	rmi_driver_copy_pdt_to_fd(pdt, &fn->fd, page_start);

	error = rmi_register_function(fn);
	if (error)
		goto err_free_mem;

	list_add_tail(&fn->node, &data->function_list);

	return 0;

err_free_mem:
	kfree(fn);
	return error;
}

/*
 * Scan the PDT for F01 so we can force a reset before anything else
 * is done.  This forces the sensor into a known state, and also
 * forces application of any pending updates from reflashing the
 * firmware or configuration.
 *
 * At this time, we also reflash the device if (a) in kernel reflashing is
 * enabled, and (b) the reflash module decides it requires reflashing.
 *
 * We have to do this before actually building the PDT because the reflash
 * might cause various registers to move around.
 */
static int reset_and_reflash(struct rmi_device *rmi_dev)
{
	struct pdt_entry pdt_entry;
	int page;
	struct device *dev = &rmi_dev->dev;
	bool done = false;
	bool has_f01 = false;
	int i;
	int retval;
	const struct rmi_device_platform_data *pdata =
			to_rmi_platform_data(rmi_dev);

	dev_dbg(dev, "Initial reset.\n");

	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, &pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				dev_err(dev, "Read PDT entry at %#06x failed, code = %d.\n",
						i, retval);
				return retval;
			}

			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;
			done = false;

			if (pdt_entry.function_number == 0x01) {
				u16 cmd_addr = page_start +
					pdt_entry.command_base_addr;
				u8 cmd_buf = RMI_DEVICE_RESET_CMD;
				retval = rmi_write_block(rmi_dev, cmd_addr,
						&cmd_buf, 1);
				if (retval < 0) {
					dev_err(dev, "Initial reset failed. Code = %d.\n",
						retval);
					return retval;
				}
				mdelay(pdata->reset_delay_ms);
				done = true;
				has_f01 = true;
				break;
			}
		}
	}

	if (!has_f01) {
		dev_warn(dev, "WARNING: Failed to find F01 for initial reset.\n");
		return -ENODEV;
	}

	return 0;
}

static int rmi_scan_pdt(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data;
	struct pdt_entry pdt_entry;
	int page;
	struct device *dev = &rmi_dev->dev;
	int irq_count = 0;
	bool done = false;
	int i;
	int retval;

	dev_dbg(dev, "Scanning PDT...\n");

	data = dev_get_drvdata(&rmi_dev->dev);
	mutex_lock(&data->pdt_mutex);

	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, &pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				dev_err(dev, "Read of PDT entry at %#06x failed.\n",
					i);
				goto error_exit;
			}

			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;

			dev_dbg(dev, "Found F%02X on page %#04x\n",
					pdt_entry.function_number, page);
			done = false;

			// XXX need to make sure we create F01 first...
			retval = create_function(rmi_dev,
					&pdt_entry, &irq_count, page_start);

			if (retval)
				goto error_exit;
		}
		done = done || data->f01_bootloader_mode;
	}
	data->irq_count = irq_count;
	data->num_of_irq_regs = (irq_count + 7) / 8;
	dev_dbg(dev, "%s: Done with PDT scan.\n", __func__);
	retval = 0;

error_exit:
	mutex_unlock(&data->pdt_mutex);
	return retval;
}

#if 0
// XXX is this crap needed with F01 always present?
static int f01_notifier_call(struct notifier_block *nb,
				unsigned long action, void *data)
{
	struct device *dev = data;
	struct rmi_function *fn;

	if (!rmi_is_function_device(dev))
		return 0;

	fn = to_rmi_function(dev);
	if (fn->fd.function_number != 0x01)
		return 0;

	switch (action) {
	case BUS_NOTIFY_BOUND_DRIVER:
		dev_dbg(dev, "%s: F01 driver bound.\n", __func__);
		enable_sensor(fn->rmi_dev);
		break;
	case BUS_NOTIFY_UNBIND_DRIVER:
		dev_dbg(dev, "%s: F01 driver going away.\n", __func__);
		disable_sensor(fn->rmi_dev);
		break;
	}
	return 0;
}

static struct notifier_block rmi_bus_notifier = {
	.notifier_call = f01_notifier_call,
};
#endif

#ifdef CONFIG_PM_SLEEP
static int rmi_driver_suspend(struct device *dev)
{
	struct rmi_driver_data *data;
	int retval = 0;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	data = dev_get_drvdata(&rmi_dev->dev);

	mutex_lock(&data->suspend_mutex);

	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval)
			goto exit;
	}

	disable_sensor(rmi_dev);

#if 0
	/** Do it backwards so F01 comes last. */
	list_for_each_entry_reverse(entry, &data->function_list, node)
		if (suspend_one_device(entry) < 0)
			goto exit;
#endif

	if (data->post_suspend)
		retval = data->post_suspend(data->pm_data);

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

static int rmi_driver_resume(struct device *dev)
{
	struct rmi_driver_data *data;
	int retval = 0;
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	data = dev_get_drvdata(&rmi_dev->dev);
	mutex_lock(&data->suspend_mutex);

	if (data->pre_resume) {
		retval = data->pre_resume(data->pm_data);
		if (retval)
			goto exit;
	}

#if 0
	/** Do it forwards, so F01 comes first. */
	list_for_each_entry(entry, &data->function_list, node) {
		if (resume_one_device(entry) < 0)
			goto exit;
	}
#endif

	retval = enable_sensor(rmi_dev);
	if (retval)
		goto exit;


	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	data->suspended = false;
exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(rmi_driver_pm, rmi_driver_suspend, rmi_driver_resume);

static int rmi_driver_remove(struct device *dev)
{
	struct rmi_device *rmi_dev = to_rmi_device(dev);

	disable_sensor(rmi_dev);
	rmi_free_function_list(rmi_dev);

	return 0;
}

static int rmi_driver_probe(struct device *dev)
{
	struct rmi_driver *rmi_driver;
	struct rmi_driver_data *data = NULL;
	struct rmi_function *fn;
	struct rmi_device_platform_data *pdata;
	int retval = 0;
	struct rmi_device *rmi_dev;

	dev_dbg(dev, "%s: Starting probe.\n", __func__);

	if (!rmi_is_physical_device(dev)) {
		dev_dbg(dev, "Not a sensor device.\n");
		return -ENODEV;
	}

	rmi_dev = to_rmi_device(dev);
	rmi_driver = to_rmi_driver(dev->driver);
	rmi_dev->driver = rmi_driver;

	pdata = to_rmi_platform_data(rmi_dev);

	data = devm_kzalloc(dev, sizeof(struct rmi_driver_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: Failed to allocate driver data.\n", __func__);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&data->function_list);
	data->rmi_dev = rmi_dev;
	dev_set_drvdata(&rmi_dev->dev, data);
	mutex_init(&data->pdt_mutex);

	/* Right before a warm boot, the sensor might be in some unusual state,
	 * such as F54 diagnostics, or F34 bootloader mode.  In order to clear
	 * the sensor to a known state, we issue a initial reset to clear any
	 * previous settings and force it into normal operation.
	 *
	 * For a number of reasons, this initial reset may fail to return
	 * within the specified time, but we'll still be able to bring up the
	 * driver normally after that failure.  This occurs most commonly in
	 * a cold boot situation (where then firmware takes longer to come up
	 * than from a warm boot) and the reset_delay_ms in the platform data
	 * has been set too short to accomodate that.  Since the sensor will
	 * eventually come up and be usable, we don't want to just fail here
	 * and leave the customer's device unusable.  So we warn them, and
	 * continue processing.
	 */
	if (!pdata->reset_delay_ms)
		pdata->reset_delay_ms = DEFAULT_RESET_DELAY_MS;
	retval = reset_and_reflash(rmi_dev);
	if (retval)
		dev_warn(dev, "RMI initial reset failed! Continuing in spite of this.\n");

	retval = rmi_scan_pdt(rmi_dev);
	if (retval) {
		dev_err(dev, "PDT scan for %s failed with code %d.\n",
			pdata->sensor_name, retval);
		goto err_free_data;
	}

	if (!data->f01_container) {
		dev_err(dev, "missing F01 container!\n");
		retval = -EINVAL;
		goto err_free_data;
	}

	list_for_each_entry(fn, &data->function_list, node) {
		retval = rmi_driver_irq_get_mask(rmi_dev, fn);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to create irq_mask.\n",
				__func__);
			goto err_free_data;
		}
	}

	retval = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION, &data->pdt_props);
	if (retval < 0) {
		/* we'll print out a warning and continue since
		 * failure to get the PDT properties is not a cause to fail
		 */
		dev_warn(dev, "Could not read PDT properties from %#06x. Assuming 0x00.\n",
			 PDT_PROPERTIES_LOCATION);
	}

	mutex_init(&data->irq_mutex);
	data->irq_status = devm_kzalloc(dev,
		BITS_TO_LONGS(data->irq_count)*sizeof(unsigned long),
		GFP_KERNEL);
	if (!data->irq_status) {
		dev_err(dev, "Failed to allocate irq_status.\n");
		retval = -ENOMEM;
		goto err_free_data;
	}

	data->current_irq_mask = devm_kzalloc(dev,
				data->num_of_irq_regs,
				GFP_KERNEL);
	if (!data->current_irq_mask) {
		dev_err(dev, "Failed to allocate current_irq_mask.\n");
		retval = -ENOMEM;
		goto err_free_data;
	}

	retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->current_irq_mask, data->num_of_irq_regs);
	if (retval < 0) {
		dev_err(dev, "%s: Failed to read current IRQ mask.\n",
			__func__);
		goto err_free_data;
	}

	data->irq_mask_store = devm_kzalloc(dev,
		BITS_TO_LONGS(data->irq_count)*sizeof(unsigned long),
		GFP_KERNEL);
	if (!data->irq_mask_store) {
		dev_err(dev, "Failed to allocate mask store.\n");
		retval = -ENOMEM;
		goto err_free_data;
	}
	if (IS_ENABLED(CONFIG_PM)) {
		data->pm_data = pdata->pm_data;
		data->pre_suspend = pdata->pre_suspend;
		data->post_suspend = pdata->post_suspend;
		data->pre_resume = pdata->pre_resume;
		data->post_resume = pdata->post_resume;

		mutex_init(&data->suspend_mutex);
	}

	if (pdata->attn_gpio) {
		data->irq = gpio_to_irq(pdata->attn_gpio);
		if (pdata->level_triggered) {
			data->irq_flags = IRQF_ONESHOT |
				((pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH)
				? IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW);
		} else {
			data->irq_flags =
				(pdata->attn_polarity == RMI_ATTN_ACTIVE_HIGH)
				? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
		}
	} else
		data->poll_interval = ktime_set(0,
			(pdata->poll_interval_ms ? pdata->poll_interval_ms :
			DEFAULT_POLL_INTERVAL_MS) * 1000);

	if (data->f01_container->dev.driver) {
		/* Driver already bound, so enable ATTN now. */
		enable_sensor(rmi_dev);
	}

	if (IS_ENABLED(CONFIG_RMI4_DEV) && pdata->attn_gpio) {
		retval = gpio_export(pdata->attn_gpio, false);
		if (retval) {
			dev_warn(dev, "WARNING: Failed to export ATTN gpio!\n");
			retval = 0;
		} else {
			retval = gpio_export_link(dev,
						  "attn", pdata->attn_gpio);
			if (retval) {
				dev_warn(dev,
					"WARNING: Failed to symlink ATTN gpio!\n");
				retval = 0;
			} else {
				dev_info(dev, "Exported ATTN GPIO %d.",
					pdata->attn_gpio);
			}
		}
	}

	return 0;

 err_free_data:
	return retval;
}

struct rmi_driver rmi_physical_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi_physical",
		.bus	= &rmi_bus_type,
		.pm	= &rmi_driver_pm,
		.probe = rmi_driver_probe,
		.remove = rmi_driver_remove,
	},
	.irq_handler = rmi_driver_irq_handler,
	.reset_handler = rmi_driver_reset_handler,
	.store_irq_mask = rmi_driver_irq_save,
	.restore_irq_mask = rmi_driver_irq_restore,
	.set_input_params = rmi_driver_set_input_params,
};

bool rmi_is_physical_driver(struct device_driver *drv)
{
	return drv == &rmi_physical_driver.driver;
}

int __init rmi_register_physical_driver(void)
{
	int error;

	error = driver_register(&rmi_physical_driver.driver);
	if (error) {
		pr_err("%s: driver register failed, code=%d.\n", __func__,
		       error);
		return error;
	}

	return 0;
}

void __exit rmi_unregister_physical_driver(void)
{
	driver_unregister(&rmi_physical_driver.driver);
}
