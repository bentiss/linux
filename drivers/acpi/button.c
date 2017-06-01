/*
 *  button.c - ACPI Button Driver
 *
 *  Copyright (C) 2001, 2002 Andy Grover <andrew.grover@intel.com>
 *  Copyright (C) 2001, 2002 Paul Diefenbaugh <paul.s.diefenbaugh@intel.com>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt) "ACPI: button: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <acpi/button.h>

#define PREFIX "ACPI: "

#define ACPI_BUTTON_CLASS		"button"
#define ACPI_BUTTON_FILE_INFO		"info"
#define ACPI_BUTTON_FILE_STATE		"state"
#define ACPI_BUTTON_TYPE_UNKNOWN	0x00
#define ACPI_BUTTON_NOTIFY_STATUS	0x80

#define ACPI_BUTTON_SUBCLASS_POWER	"power"
#define ACPI_BUTTON_HID_POWER		"PNP0C0C"
#define ACPI_BUTTON_DEVICE_NAME_POWER	"Power Button"
#define ACPI_BUTTON_TYPE_POWER		0x01

#define ACPI_BUTTON_SUBCLASS_SLEEP	"sleep"
#define ACPI_BUTTON_HID_SLEEP		"PNP0C0E"
#define ACPI_BUTTON_DEVICE_NAME_SLEEP	"Sleep Button"
#define ACPI_BUTTON_TYPE_SLEEP		0x03

#define ACPI_BUTTON_SUBCLASS_LID	"lid"
#define ACPI_BUTTON_HID_LID		"PNP0C0D"
#define ACPI_BUTTON_DEVICE_NAME_LID	"Lid Switch"
#define ACPI_BUTTON_TYPE_LID		0x05

#define ACPI_BUTTON_LID_INIT_IGNORE	0x00
#define ACPI_BUTTON_LID_INIT_OPEN	0x01
#define ACPI_BUTTON_LID_INIT_METHOD	0x02

#define _COMPONENT		ACPI_BUTTON_COMPONENT
ACPI_MODULE_NAME("button");

MODULE_AUTHOR("Paul Diefenbaugh");
MODULE_DESCRIPTION("ACPI Button Driver");
MODULE_LICENSE("GPL");

static const struct acpi_device_id button_device_ids[] = {
	{ACPI_BUTTON_HID_LID,    0},
	{ACPI_BUTTON_HID_SLEEP,  0},
	{ACPI_BUTTON_HID_SLEEPF, 0},
	{ACPI_BUTTON_HID_POWER,  0},
	{ACPI_BUTTON_HID_POWERF, 0},
	{"", 0},
};
MODULE_DEVICE_TABLE(acpi, button_device_ids);

static int acpi_button_add(struct acpi_device *device);
static int acpi_button_remove(struct acpi_device *device);
static void acpi_button_notify(struct acpi_device *device, u32 event);
static int acpi_button_add_input(struct acpi_device *device);
static int acpi_lid_update_reliable(struct acpi_device *device);

#ifdef CONFIG_PM_SLEEP
static int acpi_button_suspend(struct device *dev);
static int acpi_button_resume(struct device *dev);
#else
#define acpi_button_suspend NULL
#define acpi_button_resume NULL
#endif
static SIMPLE_DEV_PM_OPS(acpi_button_pm, acpi_button_suspend, acpi_button_resume);

static struct acpi_driver acpi_button_driver = {
	.name = "button",
	.class = ACPI_BUTTON_CLASS,
	.ids = button_device_ids,
	.ops = {
		.add = acpi_button_add,
		.remove = acpi_button_remove,
		.notify = acpi_button_notify,
	},
	.drv.pm = &acpi_button_pm,
};

struct acpi_button {
	unsigned int type;
	struct input_dev *input;
	char phys[32];			/* for input device */
	unsigned long pushed;
	bool suspended;
};

static DEFINE_MUTEX(button_input_lock);

static BLOCKING_NOTIFIER_HEAD(acpi_lid_notifier);
static struct acpi_device *lid_device;
static u8 lid_init_state = ACPI_BUTTON_LID_INIT_METHOD;

static unsigned long lid_report_interval __read_mostly = 500;
module_param(lid_report_interval, ulong, 0644);
MODULE_PARM_DESC(lid_report_interval, "Interval (ms) between lid key events");

static bool lid_reliable = true;

static int param_set_lid_reliable(const char *val,
				  const struct kernel_param *kp)
{
	bool prev_lid_reliable = lid_reliable;
	int ret;

	mutex_lock(&button_input_lock);

	ret = param_set_bool(val, kp);
	if (ret) {
		mutex_unlock(&button_input_lock);
		return ret;
	}

	/*
	 * prevent a loop when we show up the device to userspace because
	 * of an acpi notification, and userspace immediately removes it
	 * by marking it as unreliable when this was already known.
	 */
	if (lid_device && prev_lid_reliable != lid_reliable) {
		ret = acpi_lid_update_reliable(lid_device);
		if (ret)
			lid_reliable = prev_lid_reliable;
	}

	mutex_unlock(&button_input_lock);
	return ret;
}

static const struct kernel_param_ops lid_reliable_ops = {
	.get = param_get_bool,
	.set = param_set_lid_reliable,
};
module_param_cb(lid_reliable, &lid_reliable_ops, &lid_reliable, 0644);
MODULE_PARM_DESC(lid_reliable, "Is the LID switch reliable (true|false)?");

/* --------------------------------------------------------------------------
                              FS Interface (/proc)
   -------------------------------------------------------------------------- */

static struct proc_dir_entry *acpi_button_dir;
static struct proc_dir_entry *acpi_lid_dir;

static int acpi_lid_evaluate_state(struct acpi_device *device)
{
	unsigned long long lid_state;
	acpi_status status;

	status = acpi_evaluate_integer(device->handle, "_LID", NULL, &lid_state);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	return lid_state ? 1 : 0;
}

static int acpi_lid_notify_state(struct acpi_device *device, int state)
{
	struct acpi_button *button = acpi_driver_data(device);

	/* button_input_lock must be held */

	if (!button->input)
		return 0;

	/*
	 * If the lid is unreliable, always send an "open" event before any
	 * other. The input layer will filter out the extra open if required,
	 * and it will force the close event to be sent.
	 */
	if (!lid_reliable)
		input_report_switch(button->input, SW_LID, 0);

	input_report_switch(button->input, SW_LID, !state);
	input_sync(button->input);

	if (state)
		acpi_pm_wakeup_event(&device->dev);

	return 0;
}

/*
 * Pass incoming event to all connected clients.
 */
static void acpi_button_lid_events(struct input_handle *handle,
				   const struct input_value *vals,
				   unsigned int count)
{
	const struct input_value *v;
	int state = -1;
	int ret;

	for (v = vals; v != vals + count; v++) {
		switch (v->type) {
		case EV_SYN:
			if (v->code == SYN_REPORT && state >= 0) {
				ret = blocking_notifier_call_chain(&acpi_lid_notifier,
								state,
								lid_device);
				if (ret == NOTIFY_DONE)
					ret = blocking_notifier_call_chain(&acpi_lid_notifier,
								state,
								lid_device);
				if (ret == NOTIFY_DONE || ret == NOTIFY_OK) {
					/*
					 * It is also regarded as success if
					 * the notifier_chain returns NOTIFY_OK
					 * or NOTIFY_DONE.
					 */
					ret = 0;
				}
			}
			break;
		case EV_SW:
			if (v->code == SW_LID)
				state = !v->value;
			break;
		}
	}
}

static int acpi_button_lid_connect(struct input_handler *handler,
				   struct input_dev *dev,
				   const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "acpi-button-lid";

	error = input_register_handle(handle);
	if (error) {
		dev_err(&lid_device->dev, "Error installing input handle\n");
		goto err_free;
	}

	error = input_open_device(handle);
	if (error) {
		dev_err(&lid_device->dev, "Failed to open input device\n");
		goto err_unregister;
	}

	return 0;

 err_unregister:
	input_unregister_handle(handle);
 err_free:
	kfree(handle);
	return error;
}

static void acpi_button_lid_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

bool acpi_button_lid_match(struct input_handler *handler,
			   struct input_dev *dev)
{
	struct acpi_button *button;

	if (!lid_device)
		return false;

	button = acpi_driver_data(lid_device);

	if (!button || dev != button->input)
		return false;

	return true;
}

static const struct input_device_id acpi_button_lid_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_SW) },
		.swbit = { BIT_MASK(SW_LID) },
	},
	{ },
};

MODULE_DEVICE_TABLE(input, acpi_button_lid_ids);

static struct input_handler acpi_button_lid_handler = {
	.match		= acpi_button_lid_match,
	.connect	= acpi_button_lid_connect,
	.disconnect	= acpi_button_lid_disconnect,
	.events		= acpi_button_lid_events,
	.name		= "acpi-lid-callchain",
	.id_table	= acpi_button_lid_ids,
};


static int acpi_button_state_seq_show(struct seq_file *seq, void *offset)
{
	struct acpi_device *device = seq->private;
	int state;

	state = acpi_lid_evaluate_state(device);
	seq_printf(seq, "state:      %s\n",
		   state < 0 ? "unsupported" : (state ? "open" : "closed"));
	return 0;
}

static int acpi_button_state_open_fs(struct inode *inode, struct file *file)
{
	return single_open(file, acpi_button_state_seq_show, PDE_DATA(inode));
}

static const struct file_operations acpi_button_state_fops = {
	.owner = THIS_MODULE,
	.open = acpi_button_state_open_fs,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int acpi_button_add_fs(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);
	struct proc_dir_entry *entry = NULL;
	int ret = 0;

	/* procfs I/F for ACPI lid device only */
	if (button->type != ACPI_BUTTON_TYPE_LID)
		return 0;

	if (acpi_button_dir || acpi_lid_dir) {
		printk(KERN_ERR PREFIX "More than one Lid device found!\n");
		return -EEXIST;
	}

	/* create /proc/acpi/button */
	acpi_button_dir = proc_mkdir(ACPI_BUTTON_CLASS, acpi_root_dir);
	if (!acpi_button_dir)
		return -ENODEV;

	/* create /proc/acpi/button/lid */
	acpi_lid_dir = proc_mkdir(ACPI_BUTTON_SUBCLASS_LID, acpi_button_dir);
	if (!acpi_lid_dir) {
		ret = -ENODEV;
		goto remove_button_dir;
	}

	/* create /proc/acpi/button/lid/LID/ */
	acpi_device_dir(device) = proc_mkdir(acpi_device_bid(device), acpi_lid_dir);
	if (!acpi_device_dir(device)) {
		ret = -ENODEV;
		goto remove_lid_dir;
	}

	/* create /proc/acpi/button/lid/LID/state */
	entry = proc_create_data(ACPI_BUTTON_FILE_STATE,
				 S_IRUGO, acpi_device_dir(device),
				 &acpi_button_state_fops, device);
	if (!entry) {
		ret = -ENODEV;
		goto remove_dev_dir;
	}

done:
	return ret;

remove_dev_dir:
	remove_proc_entry(acpi_device_bid(device),
			  acpi_lid_dir);
	acpi_device_dir(device) = NULL;
remove_lid_dir:
	remove_proc_entry(ACPI_BUTTON_SUBCLASS_LID, acpi_button_dir);
	acpi_lid_dir = NULL;
remove_button_dir:
	remove_proc_entry(ACPI_BUTTON_CLASS, acpi_root_dir);
	acpi_button_dir = NULL;
	goto done;
}

static int acpi_button_remove_fs(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);

	if (button->type != ACPI_BUTTON_TYPE_LID)
		return 0;

	remove_proc_entry(ACPI_BUTTON_FILE_STATE,
			  acpi_device_dir(device));
	remove_proc_entry(acpi_device_bid(device),
			  acpi_lid_dir);
	acpi_device_dir(device) = NULL;
	remove_proc_entry(ACPI_BUTTON_SUBCLASS_LID, acpi_button_dir);
	acpi_lid_dir = NULL;
	remove_proc_entry(ACPI_BUTTON_CLASS, acpi_root_dir);
	acpi_button_dir = NULL;

	return 0;
}

/* --------------------------------------------------------------------------
                                Driver Interface
   -------------------------------------------------------------------------- */
int acpi_lid_notifier_register(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&acpi_lid_notifier, nb);
}
EXPORT_SYMBOL(acpi_lid_notifier_register);

int acpi_lid_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&acpi_lid_notifier, nb);
}
EXPORT_SYMBOL(acpi_lid_notifier_unregister);

int acpi_lid_open(void)
{
	if (!lid_device)
		return -ENODEV;

	return acpi_lid_evaluate_state(lid_device);
}
EXPORT_SYMBOL(acpi_lid_open);

static int acpi_lid_update_state(struct acpi_device *device)
{
	int state;

	state = acpi_lid_evaluate_state(device);
	if (state < 0)
		return state;

	return acpi_lid_notify_state(device, state);
}

static int acpi_lid_notify(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);
	int ret;

	mutex_lock(&button_input_lock);
	if (!button->input)
		acpi_button_add_input(device);
	ret = acpi_lid_update_state(device);
	mutex_unlock(&button_input_lock);


	return ret;
}

static void acpi_lid_initialize_state(struct acpi_device *device)
{
	switch (lid_init_state) {
	case ACPI_BUTTON_LID_INIT_OPEN:
		(void)acpi_lid_notify_state(device, 1);
		break;
	case ACPI_BUTTON_LID_INIT_METHOD:
		(void)acpi_lid_update_state(device);
		break;
	case ACPI_BUTTON_LID_INIT_IGNORE:
	default:
		break;
	}
}

static void acpi_button_notify(struct acpi_device *device, u32 event)
{
	struct acpi_button *button = acpi_driver_data(device);
	struct input_dev *input;

	switch (event) {
	case ACPI_FIXED_HARDWARE_EVENT:
		event = ACPI_BUTTON_NOTIFY_STATUS;
		/* fall through */
	case ACPI_BUTTON_NOTIFY_STATUS:
		input = button->input;
		if (button->type == ACPI_BUTTON_TYPE_LID) {
			acpi_lid_notify(device);
		} else {
			int keycode;

			acpi_pm_wakeup_event(&device->dev);
			if (button->suspended)
				break;

			keycode = test_bit(KEY_SLEEP, input->keybit) ?
						KEY_SLEEP : KEY_POWER;
			input_report_key(input, keycode, 1);
			input_sync(input);
			input_report_key(input, keycode, 0);
			input_sync(input);

			acpi_bus_generate_netlink_event(
					device->pnp.device_class,
					dev_name(&device->dev),
					event, ++button->pushed);
		}
		break;
	default:
		ACPI_DEBUG_PRINT((ACPI_DB_INFO,
				  "Unsupported event [0x%x]\n", event));
		break;
	}
}

#ifdef CONFIG_PM_SLEEP
static int acpi_button_suspend(struct device *dev)
{
	struct acpi_device *device = to_acpi_device(dev);
	struct acpi_button *button = acpi_driver_data(device);

	button->suspended = true;

	if (button->type == ACPI_BUTTON_TYPE_LID) {
		/*
		 * If lid is marked unreliable, this will have the effect
		 * of unregistering the LID input node
		 */
		mutex_lock(&button_input_lock);
		acpi_lid_update_reliable(device);
		mutex_unlock(&button_input_lock);
	}
	return 0;
}

static int acpi_button_resume(struct device *dev)
{
	struct acpi_device *device = to_acpi_device(dev);
	struct acpi_button *button = acpi_driver_data(device);

	button->suspended = false;
	if (button->type == ACPI_BUTTON_TYPE_LID) {
		/*
		 * If lid is marked reliable, this will have the effect
		 * of registering a new LID input node if none was there
		 */
		mutex_lock(&button_input_lock);
		acpi_lid_update_reliable(device);
		acpi_lid_initialize_state(device);
		mutex_unlock(&button_input_lock);
	}

	return 0;
}
#endif

static void acpi_button_remove_input(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);

	/* button_input_lock must be held */
	input_unregister_device(button->input);
	button->input = NULL;
}

static int acpi_button_add_input(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);
	struct input_dev *input;
	int error;

	/* button_input_lock must be held */

	button->input = input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto err;
	}

	input->name = acpi_device_name(device);
	input->phys = button->phys;
	input->id.bustype = BUS_HOST;
	input->id.product = button->type;
	input->dev.parent = &device->dev;

	switch (button->type) {
	case ACPI_BUTTON_TYPE_POWER:
		input_set_capability(input, EV_KEY, KEY_POWER);
		break;

	case ACPI_BUTTON_TYPE_SLEEP:
		input_set_capability(input, EV_KEY, KEY_SLEEP);
		break;

	case ACPI_BUTTON_TYPE_LID:
		input_set_capability(input, EV_SW, SW_LID);
		break;
	}

	error = input_register_device(input);
	if (error)
		goto err;

	return 0;

 err:
	input_free_device(input);
	button->input = NULL;
	return error;
}

static int acpi_lid_update_reliable(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(lid_device);
	int error;

	/* button_input_lock must be held */

	if (lid_reliable && !button->input) {
		error = acpi_button_add_input(device);
		if (error)
			return error;

		error = acpi_lid_update_state(device);
		if (error) {
			acpi_button_remove_input(device);
			return error;
		}
	}

	if (!lid_reliable && button->input)
		acpi_button_remove_input(device);

	return 0;
}

static int acpi_button_add(struct acpi_device *device)
{
	struct acpi_button *button;
	const char *hid = acpi_device_hid(device);
	char *name, *class;
	int error;

	button = kzalloc(sizeof(struct acpi_button), GFP_KERNEL);
	if (!button)
		return -ENOMEM;

	device->driver_data = button;

	name = acpi_device_name(device);
	class = acpi_device_class(device);

	if (!strcmp(hid, ACPI_BUTTON_HID_POWER) ||
	    !strcmp(hid, ACPI_BUTTON_HID_POWERF)) {
		button->type = ACPI_BUTTON_TYPE_POWER;
		strcpy(name, ACPI_BUTTON_DEVICE_NAME_POWER);
		sprintf(class, "%s/%s",
			ACPI_BUTTON_CLASS, ACPI_BUTTON_SUBCLASS_POWER);
	} else if (!strcmp(hid, ACPI_BUTTON_HID_SLEEP) ||
		   !strcmp(hid, ACPI_BUTTON_HID_SLEEPF)) {
		button->type = ACPI_BUTTON_TYPE_SLEEP;
		strcpy(name, ACPI_BUTTON_DEVICE_NAME_SLEEP);
		sprintf(class, "%s/%s",
			ACPI_BUTTON_CLASS, ACPI_BUTTON_SUBCLASS_SLEEP);
	} else if (!strcmp(hid, ACPI_BUTTON_HID_LID)) {
		button->type = ACPI_BUTTON_TYPE_LID;
		strcpy(name, ACPI_BUTTON_DEVICE_NAME_LID);
		sprintf(class, "%s/%s",
			ACPI_BUTTON_CLASS, ACPI_BUTTON_SUBCLASS_LID);
	} else {
		printk(KERN_ERR PREFIX "Unsupported hid [%s]\n", hid);
		error = -ENODEV;
		goto err_free_button;
	}

	error = acpi_button_add_fs(device);
	if (error)
		goto err_free_button;

	snprintf(button->phys, sizeof(button->phys), "%s/button/input0", hid);

	if (button->type == ACPI_BUTTON_TYPE_LID) {
		/*
		 * This assumes there's only one lid device, or if there are
		 * more we only care about the last one...
		 */
		lid_device = device;
	}

	if (lid_reliable || button->type != ACPI_BUTTON_TYPE_LID) {
		error = acpi_button_add_input(device);
		if (error)
			goto err_remove_fs;

		if (button->type == ACPI_BUTTON_TYPE_LID) {
			mutex_lock(&button_input_lock);
			acpi_lid_initialize_state(device);
			mutex_unlock(&button_input_lock);
		}
	}

	device_init_wakeup(&device->dev, true);
	printk(KERN_INFO PREFIX "%s [%s]\n", name, acpi_device_bid(device));
	return 0;

 err_remove_fs:
	acpi_button_remove_fs(device);
 err_free_button:
	kfree(button);
	return error;
}

static int acpi_button_remove(struct acpi_device *device)
{
	struct acpi_button *button = acpi_driver_data(device);

	acpi_button_remove_fs(device);
	acpi_button_remove_input(device);
	kfree(button);
	return 0;
}

static int param_set_lid_init_state(const char *val, struct kernel_param *kp)
{
	int result = 0;

	if (!strncmp(val, "open", sizeof("open") - 1)) {
		lid_init_state = ACPI_BUTTON_LID_INIT_OPEN;
		pr_info("Notify initial lid state as open\n");
	} else if (!strncmp(val, "method", sizeof("method") - 1)) {
		lid_init_state = ACPI_BUTTON_LID_INIT_METHOD;
		pr_info("Notify initial lid state with _LID return value\n");
	} else if (!strncmp(val, "ignore", sizeof("ignore") - 1)) {
		lid_init_state = ACPI_BUTTON_LID_INIT_IGNORE;
		pr_info("Do not notify initial lid state\n");
	} else
		result = -EINVAL;
	return result;
}

static int param_get_lid_init_state(char *buffer, struct kernel_param *kp)
{
	switch (lid_init_state) {
	case ACPI_BUTTON_LID_INIT_OPEN:
		return sprintf(buffer, "open");
	case ACPI_BUTTON_LID_INIT_METHOD:
		return sprintf(buffer, "method");
	case ACPI_BUTTON_LID_INIT_IGNORE:
		return sprintf(buffer, "ignore");
	default:
		return sprintf(buffer, "invalid");
	}
	return 0;
}

module_param_call(lid_init_state,
		  param_set_lid_init_state, param_get_lid_init_state,
		  NULL, 0644);
MODULE_PARM_DESC(lid_init_state, "Behavior for reporting LID initial state");

static int __init acpi_button_init(void)
{
	int error;

	error = input_register_handler(&acpi_button_lid_handler);
	if (error)
		return error;

	return acpi_bus_register_driver(&acpi_button_driver);
}

static void __exit acpi_button_exit(void)
{
	acpi_bus_unregister_driver(&acpi_button_driver);
	input_unregister_handler(&acpi_button_lid_handler);
}

module_init(acpi_button_init);
module_exit(acpi_button_exit);
