/*
 *  Bluetooth Wacom Tablet support
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2006-2007 Jiri Kosina
 *  Copyright (c) 2008 Jiri Slaby <jirislaby@gmail.com>
 *  Copyright (c) 2006 Andrew Zabolotny <zap@homelink.ru>
 *  Copyright (c) 2009 Bastien Nocera <hadess@hadess.net>
 *  Copyright (c) 2011 Przemysław Firszt <przemo@firszt.eu>
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <asm/unaligned.h>

#include "hid-ids.h"

#define PAD_DEVICE_ID	0x0F

#define WAC_CMD_LED_CONTROL     0x20
#define WAC_CMD_ICON_START_STOP     0x21
#define WAC_CMD_ICON_TRANSFER       0x26

#define HID_UP_OFFSET			0xbe000000
#define HID_MASK_UP_OFFSET		0xff000000
#define HID_MASK_USAGE_OFFSET		0x0000ff00

#define HID_UP_WACOM			0x00be0000
#define HID_WAC_GLOBAL_SUB_REPORT_ID	0x00be0001
#define HID_WAC_LOCAL_SUB_REPORT_ID	0x00be0002
#define HID_WAC_SERIAL			0x00be0003
#define HID_WAC_TOOL_ID			0x00be0004

struct wacom_data {
	__u16 tool;
	__u16 butstate;
	__u8 whlstate;
	__u8 features;
	__u32 id;
	__u32 serial;
	unsigned char high_speed;
	__u8 battery_capacity;
	__u8 power_raw;
	__u8 ps_connected;
	__u8 bat_charging;
	struct power_supply battery;
	struct power_supply ac;
	__u8 led_selector;
	struct led_classdev *leds[4];
	struct input_dev *input;
	s32 x, y, distance, p, x_tilt, y_tilt, in_range, hserial, tool_id;
	s32 tool_type, prev_tool_type;
	s32 eraser;
	s32 z, z_max;
	s32 pad_id;
	__s16 inputmode;	/* InputMode HID feature, -1 if non-existent */
	__s16 inputmode_index;	/* InputMode HID feature index in the report */
};

/*percent of battery capacity for Graphire
  8th value means AC online and show 100% capacity */
static unsigned short batcap_gr[8] = { 1, 15, 25, 35, 50, 70, 100, 100 };
/*percent of battery capacity for Intuos4 WL, AC has a separate bit*/
static unsigned short batcap_i4[8] = { 1, 15, 30, 45, 60, 70, 85, 100 };

static enum power_supply_property wacom_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_STATUS,
};

static enum power_supply_property wacom_ac_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_SCOPE,
};

static void wacom_scramble(__u8 *image)
{
	__u16 mask;
	__u16 s1;
	__u16 s2;
	__u16 r1 ;
	__u16 r2 ;
	__u16 r;
	__u8 buf[256];
	int i, w, x, y, z;

	for (x = 0; x < 32; x++) {
		for (y = 0; y < 8; y++)
			buf[(8 * x) + (7 - y)] = image[(8 * x) + y];
	}

	/* Change 76543210 into GECA6420 as required by Intuos4 WL
	 *        HGFEDCBA      HFDB7531
	 */
	for (x = 0; x < 4; x++) {
		for (y = 0; y < 4; y++) {
			for (z = 0; z < 8; z++) {
				mask = 0x0001;
				r1 = 0;
				r2 = 0;
				i = (x << 6) + (y << 4) + z;
				s1 = buf[i];
				s2 = buf[i+8];
				for (w = 0; w < 8; w++) {
					r1 |= (s1 & mask);
					r2 |= (s2 & mask);
					s1 <<= 1;
					s2 <<= 1;
					mask <<= 2;
				}
				r = r1 | (r2 << 1);
				i = (x << 6) + (y << 4) + (z << 1);
				image[i] = 0xFF & r;
				image[i+1] = (0xFF00 & r) >> 8;
			}
		}
	}
}

static void wacom_set_image(struct hid_device *hdev, const char *image,
						__u8 icon_no)
{
	__u8 rep_data[68];
	__u8 p[256];
	int ret, i, j;

	for (i = 0; i < 256; i++)
		p[i] = image[i];

	rep_data[0] = WAC_CMD_ICON_START_STOP;
	rep_data[1] = 0;
	ret = hid_hw_raw_request(hdev, rep_data[0], rep_data, 2,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret < 0)
		goto err;

	rep_data[0] = WAC_CMD_ICON_TRANSFER;
	rep_data[1] = icon_no & 0x07;

	wacom_scramble(p);

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 64; j++)
			rep_data[j + 3] = p[(i << 6) + j];

		rep_data[2] = i;
		ret = hid_hw_raw_request(hdev, rep_data[0], rep_data, 67,
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	}

	rep_data[0] = WAC_CMD_ICON_START_STOP;
	rep_data[1] = 0;

	ret = hid_hw_raw_request(hdev, rep_data[0], rep_data, 2,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);

err:
	return;
}

static void wacom_leds_set_brightness(struct led_classdev *led_dev,
						enum led_brightness value)
{
	struct device *dev = led_dev->dev->parent;
	struct hid_device *hdev;
	struct wacom_data *wdata;
	unsigned char *buf;
	__u8 led = 0;
	int i;

	hdev = container_of(dev, struct hid_device, dev);
	wdata = hid_get_drvdata(hdev);
	for (i = 0; i < 4; ++i) {
		if (wdata->leds[i] == led_dev)
			wdata->led_selector = i;
	}

	led = wdata->led_selector | 0x04;
	buf = kzalloc(9, GFP_KERNEL);
	if (buf) {
		buf[0] = WAC_CMD_LED_CONTROL;
		buf[1] = led;
		buf[2] = value >> 2;
		buf[3] = value;
		/* use fixed brightness for OLEDs */
		buf[4] = 0x08;
		hid_hw_raw_request(hdev, buf[0], buf, 9, HID_FEATURE_REPORT,
				   HID_REQ_SET_REPORT);
		kfree(buf);
	}

	return;
}

static enum led_brightness wacom_leds_get_brightness(struct led_classdev *led_dev)
{
	struct wacom_data *wdata;
	struct device *dev = led_dev->dev->parent;
	int value = 0;
	int i;

	wdata = hid_get_drvdata(container_of(dev, struct hid_device, dev));

	for (i = 0; i < 4; ++i) {
		if (wdata->leds[i] == led_dev) {
			value = wdata->leds[i]->brightness;
			break;
		}
	}

	return value;
}


static int wacom_initialize_leds(struct hid_device *hdev)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	struct led_classdev *led;
	struct device *dev = &hdev->dev;
	size_t namesz = strlen(dev_name(dev)) + 12;
	char *name;
	int i, ret;

	wdata->led_selector = 0;

	for (i = 0; i < 4; i++) {
		led = kzalloc(sizeof(struct led_classdev) + namesz, GFP_KERNEL);
		if (!led) {
			hid_warn(hdev,
				 "can't allocate memory for LED selector\n");
			ret = -ENOMEM;
			goto err;
		}

		name = (void *)&led[1];
		snprintf(name, namesz, "%s:selector:%d", dev_name(dev), i);
		led->name = name;
		led->brightness = 0;
		led->max_brightness = 127;
		led->brightness_get = wacom_leds_get_brightness;
		led->brightness_set = wacom_leds_set_brightness;

		wdata->leds[i] = led;

		ret = led_classdev_register(dev, wdata->leds[i]);

		if (ret) {
			wdata->leds[i] = NULL;
			kfree(led);
			hid_warn(hdev, "can't register LED\n");
			goto err;
		}
	}

err:
	return ret;
}

static void wacom_destroy_leds(struct hid_device *hdev)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	struct led_classdev *led;
	int i;

	for (i = 0; i < 4; ++i) {
		if (wdata->leds[i]) {
			led = wdata->leds[i];
			wdata->leds[i] = NULL;
			led_classdev_unregister(led);
			kfree(led);
		}
	}

}

static int wacom_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct wacom_data *wdata = container_of(psy,
					struct wacom_data, battery);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = wdata->battery_capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (wdata->bat_charging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			if (wdata->battery_capacity == 100 && wdata->ps_connected)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int wacom_ac_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct wacom_data *wdata = container_of(psy, struct wacom_data, ac);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		/* fall through */
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = wdata->ps_connected;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void wacom_set_features(struct hid_device *hdev, u8 speed)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	struct hid_report *r;
	struct hid_report_enum *re;
	int limit, ret;
	__u8 rep_data[2];

	if (wdata->inputmode >= 0) {
		re = &(hdev->report_enum[HID_FEATURE_REPORT]);
		r = re->report_id_hash[wdata->inputmode];
		if (r) {
			r->field[0]->value[wdata->inputmode_index] = 2;
			hid_hw_request(hdev, r, HID_REQ_SET_REPORT);
		}
	}

	switch (hdev->product) {
	case USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH:
		rep_data[0] = 0x03 ; rep_data[1] = 0x00;
		limit = 3;
		do {
			ret = hid_hw_raw_request(hdev, rep_data[0], rep_data, 2,
					HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
		} while (ret < 0 && limit-- > 0);

		if (ret >= 0) {
			if (speed == 0)
				rep_data[0] = 0x05;
			else
				rep_data[0] = 0x06;

			rep_data[1] = 0x00;
			limit = 3;
			do {
				ret = hid_hw_raw_request(hdev, rep_data[0],
					rep_data, 2, HID_FEATURE_REPORT,
					HID_REQ_SET_REPORT);
			} while (ret < 0 && limit-- > 0);

			if (ret >= 0) {
				wdata->high_speed = speed;
				return;
			}
		}

		/*
		 * Note that if the raw queries fail, it's not a hard failure
		 * and it is safe to continue
		 */
		hid_warn(hdev, "failed to poke device, command %d, err %d\n",
			 rep_data[0], ret);
		break;
	case USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH:
		if (speed == 1)
			wdata->features &= ~0x20;
		else
			wdata->features |= 0x20;

		rep_data[0] = 0x03;
		rep_data[1] = wdata->features;

		ret = hid_hw_raw_request(hdev, rep_data[0], rep_data, 2,
				HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
		if (ret >= 0)
			wdata->high_speed = speed;
		break;
	}

	return;
}

static ssize_t wacom_show_speed(struct device *dev,
				struct device_attribute
				*attr, char *buf)
{
	struct wacom_data *wdata = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%i\n", wdata->high_speed);
}

static ssize_t wacom_store_speed(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct hid_device *hdev = container_of(dev, struct hid_device, dev);
	int new_speed;

	if (sscanf(buf, "%1d", &new_speed ) != 1)
		return -EINVAL;

	if (new_speed == 0 || new_speed == 1) {
		wacom_set_features(hdev, new_speed);
		return strnlen(buf, PAGE_SIZE);
	} else
		return -EINVAL;
}

static DEVICE_ATTR(speed, S_IRUGO | S_IWUSR | S_IWGRP,
		wacom_show_speed, wacom_store_speed);

#define WACOM_STORE(OLED_ID)						\
static ssize_t wacom_oled##OLED_ID##_store(struct device *dev,		\
				struct device_attribute *attr,		\
				const char *buf, size_t count)		\
{									\
	struct hid_device *hdev = container_of(dev, struct hid_device,	\
				dev);					\
									\
	if (count != 256)						\
		return -EINVAL;						\
									\
	wacom_set_image(hdev, buf, OLED_ID);				\
									\
	return count;							\
}									\
									\
static DEVICE_ATTR(oled##OLED_ID##_img, S_IWUSR | S_IWGRP, NULL,	\
				wacom_oled##OLED_ID##_store)

WACOM_STORE(0);
WACOM_STORE(1);
WACOM_STORE(2);
WACOM_STORE(3);
WACOM_STORE(4);
WACOM_STORE(5);
WACOM_STORE(6);
WACOM_STORE(7);

static int wacom_gr_parse_report(struct hid_device *hdev,
			struct wacom_data *wdata,
			struct input_dev *input, unsigned char *data)
{
	int tool, x, y, rw;

	tool = 0;
	/* Get X & Y positions */
	x = le16_to_cpu(*(__le16 *) &data[2]);
	y = le16_to_cpu(*(__le16 *) &data[4]);

	/* Get current tool identifier */
	if (data[1] & 0x90) { /* If pen is in the in/active area */
		switch ((data[1] >> 5) & 3) {
		case 0:	/* Pen */
			tool = BTN_TOOL_PEN;
			break;

		case 1: /* Rubber */
			tool = BTN_TOOL_RUBBER;
			break;

		case 2: /* Mouse with wheel */
		case 3: /* Mouse without wheel */
			tool = BTN_TOOL_MOUSE;
			break;
		}

		/* Reset tool if out of active tablet area */
		if (!(data[1] & 0x10))
			tool = 0;
	}

	/* If tool changed, notify input subsystem */
	if (wdata->tool != tool) {
		if (wdata->tool) {
			/* Completely reset old tool state */
			if (wdata->tool == BTN_TOOL_MOUSE) {
				input_report_key(input, BTN_LEFT, 0);
				input_report_key(input, BTN_RIGHT, 0);
				input_report_key(input, BTN_MIDDLE, 0);
				input_report_abs(input, ABS_DISTANCE,
					input_abs_get_max(input, ABS_DISTANCE));
			} else {
				input_report_key(input, BTN_TOUCH, 0);
				input_report_key(input, BTN_STYLUS, 0);
				input_report_key(input, BTN_STYLUS2, 0);
				input_report_abs(input, ABS_PRESSURE, 0);
			}
			input_report_key(input, wdata->tool, 0);
			input_sync(input);
		}
		wdata->tool = tool;
		if (tool)
			input_report_key(input, tool, 1);
	}

	if (tool) {
		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);

		switch ((data[1] >> 5) & 3) {
		case 2: /* Mouse with wheel */
			input_report_key(input, BTN_MIDDLE, data[1] & 0x04);
			rw = (data[6] & 0x01) ? -1 :
				(data[6] & 0x02) ? 1 : 0;
			input_report_rel(input, REL_WHEEL, rw);
			/* fall through */

		case 3: /* Mouse without wheel */
			input_report_key(input, BTN_LEFT, data[1] & 0x01);
			input_report_key(input, BTN_RIGHT, data[1] & 0x02);
			/* Compute distance between mouse and tablet */
			rw = 44 - (data[6] >> 2);
			if (rw < 0)
				rw = 0;
			else if (rw > 31)
				rw = 31;
			input_report_abs(input, ABS_DISTANCE, rw);
			break;

		default:
			input_report_abs(input, ABS_PRESSURE,
					data[6] | (((__u16) (data[1] & 0x08)) << 5));
			input_report_key(input, BTN_TOUCH, data[1] & 0x01);
			input_report_key(input, BTN_STYLUS, data[1] & 0x02);
			input_report_key(input, BTN_STYLUS2, (tool == BTN_TOOL_PEN) && data[1] & 0x04);
			break;
		}

		input_sync(input);
	}

	/* Report the state of the two buttons at the top of the tablet
	 * as two extra fingerpad keys (buttons 4 & 5). */
	rw = data[7] & 0x03;
	if (rw != wdata->butstate) {
		wdata->butstate = rw;
		input_report_key(input, BTN_0, rw & 0x02);
		input_report_key(input, BTN_1, rw & 0x01);
		input_report_key(input, BTN_TOOL_FINGER, 0xf0);
		input_event(input, EV_MSC, MSC_SERIAL, 0xf0);
		input_sync(input);
	}

	/* Store current battery capacity and power supply state*/
	rw = (data[7] >> 2 & 0x07);
	if (rw != wdata->power_raw) {
		wdata->power_raw = rw;
		wdata->battery_capacity = batcap_gr[rw];
		if (rw == 7)
			wdata->ps_connected = 1;
		else
			wdata->ps_connected = 0;
	}
	return 1;
}

static void wacom_i4_parse_button_report(struct wacom_data *wdata,
			struct input_dev *input, unsigned char *data)
{
	__u16 new_butstate;
	__u8 new_whlstate;
	__u8 sync = 0;

	new_whlstate = data[1];
	if (new_whlstate != wdata->whlstate) {
		wdata->whlstate = new_whlstate;
		if (new_whlstate & 0x80) {
			input_report_key(input, BTN_TOUCH, 1);
			input_report_abs(input, ABS_WHEEL, (new_whlstate & 0x7f));
			input_report_key(input, BTN_TOOL_FINGER, 1);
		} else {
			input_report_key(input, BTN_TOUCH, 0);
			input_report_abs(input, ABS_WHEEL, 0);
			input_report_key(input, BTN_TOOL_FINGER, 0);
		}
		sync = 1;
	}

	new_butstate = (data[3] << 1) | (data[2] & 0x01);
	if (new_butstate != wdata->butstate) {
		wdata->butstate = new_butstate;
		input_report_key(input, BTN_0, new_butstate & 0x001);
		input_report_key(input, BTN_1, new_butstate & 0x002);
		input_report_key(input, BTN_2, new_butstate & 0x004);
		input_report_key(input, BTN_3, new_butstate & 0x008);
		input_report_key(input, BTN_4, new_butstate & 0x010);
		input_report_key(input, BTN_5, new_butstate & 0x020);
		input_report_key(input, BTN_6, new_butstate & 0x040);
		input_report_key(input, BTN_7, new_butstate & 0x080);
		input_report_key(input, BTN_8, new_butstate & 0x100);
		input_report_key(input, BTN_TOOL_FINGER, 1);
		sync = 1;
	}

	if (sync) {
		input_report_abs(input, ABS_MISC, PAD_DEVICE_ID);
		input_event(input, EV_MSC, MSC_SERIAL, 0xffffffff);
		input_sync(input);
	}
}

static void wacom_i4_parse_pen_report(struct wacom_data *wdata,
			struct input_dev *input, unsigned char *data)
{
	__u16 x, y, pressure;
	__u8 distance;
	__u8 tilt_x, tilt_y;

	switch (data[1]) {
	case 0x80: /* Out of proximity report */
		input_report_key(input, BTN_TOUCH, 0);
		input_report_abs(input, ABS_PRESSURE, 0);
		input_report_key(input, BTN_STYLUS, 0);
		input_report_key(input, BTN_STYLUS2, 0);
		input_report_key(input, wdata->tool, 0);
		input_report_abs(input, ABS_MISC, 0);
		input_event(input, EV_MSC, MSC_SERIAL, wdata->serial);
		wdata->tool = 0;
		input_sync(input);
		break;
	case 0xC2: /* Tool report */
		wdata->id = ((data[2] << 4) | (data[3] >> 4) |
			((data[7] & 0x0f) << 20) |
			((data[8] & 0xf0) << 12));
		wdata->serial = ((data[3] & 0x0f) << 28) +
				(data[4] << 20) + (data[5] << 12) +
				(data[6] << 4) + (data[7] >> 4);

		switch (wdata->id) {
		case 0x100802:
			wdata->tool = BTN_TOOL_PEN;
			break;
		case 0x10080A:
			wdata->tool = BTN_TOOL_RUBBER;
			break;
		}
		break;
	default: /* Position/pressure report */
		x = data[2] << 9 | data[3] << 1 | ((data[9] & 0x02) >> 1);
		y = data[4] << 9 | data[5] << 1 | (data[9] & 0x01);
		pressure = (data[6] << 3) | ((data[7] & 0xC0) >> 5)
			| (data[1] & 0x01);
		distance = (data[9] >> 2) & 0x3f;
		tilt_x = ((data[7] << 1) & 0x7e) | (data[8] >> 7);
		tilt_y = data[8] & 0x7f;

		input_report_key(input, BTN_TOUCH, pressure > 1);

		input_report_key(input, BTN_STYLUS, data[1] & 0x02);
		input_report_key(input, BTN_STYLUS2, data[1] & 0x04);
		input_report_key(input, wdata->tool, 1);
		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);
		input_report_abs(input, ABS_PRESSURE, pressure);
		input_report_abs(input, ABS_DISTANCE, distance);
		input_report_abs(input, ABS_TILT_X, tilt_x);
		input_report_abs(input, ABS_TILT_Y, tilt_y);
		input_report_abs(input, ABS_MISC, wdata->id);
		input_event(input, EV_MSC, MSC_SERIAL, wdata->serial);
		input_report_key(input, wdata->tool, 1);
		input_sync(input);
		break;
	}

	return;
}

static void wacom_i4_parse_report(struct hid_device *hdev,
			struct wacom_data *wdata,
			struct input_dev *input, unsigned char *data)
{
	switch (data[0]) {
	case 0x00: /* Empty report */
		break;
	case 0x02: /* Pen report */
		wacom_i4_parse_pen_report(wdata, input, data);
		break;
	case 0x03: /* Features Report */
		wdata->features = data[2];
		break;
	case 0x0C: /* Button report */
		wacom_i4_parse_button_report(wdata, input, data);
		break;
	default:
		hid_err(hdev, "Unknown report: %d,%d\n", data[0], data[1]);
		break;
	}
}

static __u32 extract(const struct hid_device *hid, __u8 *report,
		     unsigned offset, unsigned n)
{
	u64 x;

	if (n > 32)
		hid_warn(hid, "extract() called with n (%d) > 32!\n", n);

	report += offset >> 3;  /* adjust byte index */
	offset &= 7;            /* now only need bit offset into one byte */
	x = get_unaligned_le64(report);
	x = (x >> offset) & ((1ULL << n) - 1);  /* extract bit field */
	return (u32) x;
}

static int wacom_raw_event(struct hid_device *hdev, struct hid_report *report,
		u8 *raw_data, int size)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	struct hid_input *hidinput;
	struct input_dev *input;
	unsigned char *data = (unsigned char *) raw_data;
	int i;
	__u8 power_raw;

	if (!(hdev->claimed & HID_CLAIMED_INPUT))
		return 0;

	for (i = 0; i < report->maxfield; i++) {
		struct hid_field *field = report->field[i];
		unsigned usage = field->usage->hid;
		unsigned offset = field->report_offset;
		unsigned rsize = field->report_size;
		if (usage == HID_WAC_GLOBAL_SUB_REPORT_ID) {
			u8 value = (u8) extract(hdev, raw_data + 1, offset,
						rsize);
			raw_data[0] = value;
		}
	}

	if ((hdev->product != USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) &&
	    (hdev->product != USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH))
		return 1;

	hidinput = list_entry(hdev->inputs.next, struct hid_input, list);
	input = hidinput->input;

	switch (hdev->product) {
	case USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH:
		if (data[0] == 0x03) {
			return wacom_gr_parse_report(hdev, wdata, input, data);
		} else {
			hid_err(hdev, "Unknown report: %d,%d size:%d\n",
					data[0], data[1], size);
			return 0;
		}
		break;
	case USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH:
		i = 1;

		switch (data[0]) {
		case 0x04:
			wacom_i4_parse_report(hdev, wdata, input, data + i);
			i += 10;
			/* fall through */
		case 0x03:
			wacom_i4_parse_report(hdev, wdata, input, data + i);
			i += 10;
			wacom_i4_parse_report(hdev, wdata, input, data + i);
			power_raw = data[i+10];
			if (power_raw != wdata->power_raw) {
				wdata->power_raw = power_raw;
				wdata->battery_capacity = batcap_i4[power_raw & 0x07];
				wdata->bat_charging = (power_raw & 0x08) ? 1 : 0;
				wdata->ps_connected = (power_raw & 0x10) ? 1 : 0;
			}

			break;
		default:
			hid_err(hdev, "Unknown report: %d,%d size:%d\n",
					data[0], data[1], size);
			return 0;
		}
	}
	return 1;
}

static int wacom_get_tool_type(int tool_id)
{
	int tool_type;

	switch (tool_id) {
	case 0x812: /* Inking pen */
	case 0x801: /* Intuos3 Inking pen */
	case 0x120802: /* Intuos4/5 Inking Pen */
	case 0x012:
		tool_type = BTN_TOOL_PENCIL;
		break;

	case 0x822: /* Pen */
	case 0x842:
	case 0x852:
	case 0x823: /* Intuos3 Grip Pen */
	case 0x813: /* Intuos3 Classic Pen */
	case 0x885: /* Intuos3 Marker Pen */
	case 0x802: /* Intuos4/5 13HD/24HD General Pen */
	case 0x804: /* Intuos4/5 13HD/24HD Marker Pen */
	case 0x022:
	case 0x100804: /* Intuos4/5 13HD/24HD Art Pen */
	case 0x140802: /* Intuos4/5 13HD/24HD Classic Pen */
	case 0x160802: /* Cintiq 13HD Pro Pen */
	case 0x180802: /* DTH2242 Pen */
	case 0x100802: /* Intuos4/5 13HD/24HD General Pen */
		tool_type = BTN_TOOL_PEN;
		break;

	case 0x832: /* Stroke pen */
	case 0x032:
		tool_type = BTN_TOOL_BRUSH;
		break;

	case 0x007: /* Mouse 4D and 2D */
	case 0x09c:
	case 0x094:
	case 0x017: /* Intuos3 2D Mouse */
	case 0x806: /* Intuos4 Mouse */
		tool_type = BTN_TOOL_MOUSE;
		break;

	case 0x096: /* Lens cursor */
	case 0x097: /* Intuos3 Lens cursor */
	case 0x006: /* Intuos4 Lens cursor */
		tool_type = BTN_TOOL_LENS;
		break;

	case 0x82a: /* Eraser */
	case 0x85a:
	case 0x91a:
	case 0xd1a:
	case 0x0fa:
	case 0x82b: /* Intuos3 Grip Pen Eraser */
	case 0x81b: /* Intuos3 Classic Pen Eraser */
	case 0x91b: /* Intuos3 Airbrush Eraser */
	case 0x80c: /* Intuos4/5 13HD/24HD Marker Pen Eraser */
	case 0x80a: /* Intuos4/5 13HD/24HD General Pen Eraser */
	case 0x90a: /* Intuos4/5 13HD/24HD Airbrush Eraser */
	case 0x14080a: /* Intuos4/5 13HD/24HD Classic Pen Eraser */
	case 0x10090a: /* Intuos4/5 13HD/24HD Airbrush Eraser */
	case 0x10080c: /* Intuos4/5 13HD/24HD Art Pen Eraser */
	case 0x16080a: /* Cintiq 13HD Pro Pen Eraser */
	case 0x18080a: /* DTH2242 Eraser */
	case 0x10080a: /* Intuos4/5 13HD/24HD General Pen Eraser */
		tool_type = BTN_TOOL_RUBBER;
		break;

	case 0xd12:
	case 0x912:
	case 0x112:
	case 0x913: /* Intuos3 Airbrush */
	case 0x902: /* Intuos4/5 13HD/24HD Airbrush */
	case 0x100902: /* Intuos4/5 13HD/24HD Airbrush */
		tool_type = BTN_TOOL_AIRBRUSH;
		break;

	default: /* Unknown tool */
		tool_type = BTN_TOOL_PEN;
		break;
	}
	return tool_type;
}

static s32 wacom_replace_bits(s32 data, s32 bits, unsigned shift, unsigned size)
{
	u32 mask = GENMASK(size + shift - 1, shift);
	data &= ~mask;
	data |=  bits << shift;
	return data;
}

static int wacom_input_event(struct hid_device *hdev, struct hid_field *field,
				struct hid_usage *usage, __s32 value)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	unsigned usage_code = usage->hid;
	unsigned code;
	unsigned shift = 0;
	unsigned size = field->report_size;

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH))
		return 0;

	if (!(hdev->claimed & HID_CLAIMED_INPUT))
		return -1;

	if ((usage_code & HID_MASK_UP_OFFSET) &&
	    ((usage_code & HID_MASK_UP_OFFSET) != HID_UP_OFFSET))
		return 0;

	/* remove leading 0xbe and included offset */
	usage_code &= ~(HID_MASK_UP_OFFSET | HID_MASK_USAGE_OFFSET);

	shift = (usage->hid & HID_MASK_USAGE_OFFSET) >> 8;
	switch (usage_code & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:
		switch (usage_code) {
		case HID_GD_X:
			wdata->x = wacom_replace_bits(wdata->x, value, shift, size);
			break;
		case HID_GD_Y:
			wdata->y = wacom_replace_bits(wdata->y, value, shift, size);
			break;
		case HID_GD_Z:
			wdata->z = wacom_replace_bits(wdata->z, value, shift, size);
			wdata->distance = wdata->z_max - wdata->z;
			break;
		}
		break;
	case HID_UP_DIGITIZER:
		switch (usage_code) {
		case HID_DG_TIPPRESSURE:
			wdata->p = wacom_replace_bits(wdata->p, value, shift, size);
			break;
		case HID_DG_X_TILT:
			wdata->x_tilt = wacom_replace_bits(wdata->x_tilt, value, shift,
							   size);
			break;
		case HID_DG_Y_TILT:
			wdata->y_tilt = wacom_replace_bits(wdata->y_tilt, value, shift,
							   size);
			break;
		case HID_DG_ALTITUDE:
			wdata->distance = wacom_replace_bits(wdata->distance, value, shift, size);
			break;
		case HID_DG_INRANGE:
			wdata->in_range = wacom_replace_bits(wdata->in_range, value,
							     shift, size);
			break;
		case HID_DG_ERASER:
			wdata->eraser = wacom_replace_bits(wdata->eraser, value,
							     shift, size);
			wdata->tool_type = wdata->eraser ? BTN_TOOL_RUBBER :
							   BTN_TOOL_PEN;
			break;
		}
		break;
	case HID_UP_WACOM:
		switch (usage_code) {
		case HID_WAC_SERIAL:
			wdata->hserial = wacom_replace_bits(wdata->hserial, value,
							    shift, size);
			break;
		case HID_WAC_TOOL_ID:
			wdata->tool_id = wacom_replace_bits(wdata->tool_id, value,
							    shift, size);
			wdata->tool_type = wacom_get_tool_type(wdata->tool_id);
			break;
		}
		break;
	case HID_UP_BUTTON:
		code = usage->hid & HID_USAGE;
		if (!(usage->hid & HID_MASK_UP_OFFSET))
			code = BTN_0 + code - 1;
		input_report_key(wdata->input, code, value);
		if (field->report->id == 0x0c && value && code > 0xff)
			wdata->pad_id = PAD_DEVICE_ID;
		break;
	}
	return 1;
}

static void wacom_input_report(struct hid_device *hdev,
		struct hid_report *report)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH))
		return;

	if (!(hdev->claimed & HID_CLAIMED_INPUT))
		return;

	if (!wdata->x) {
		/* looks like we received a pad event */
		input_report_abs(wdata->input, ABS_MISC, wdata->pad_id);
		if (!wdata->tool_id)
			/* Cintiqs send "in" reports without x/y */
			input_event(wdata->input, EV_MSC, MSC_SERIAL,
					wdata->hserial);
		wdata->pad_id = 0;
		return;
	}

	if (!wdata->in_range) {
		wdata->x = 0;
		wdata->y = 0;
		wdata->z = 0;
		wdata->distance = 0;
		wdata->p = 0;
		wdata->x_tilt = 0;
		wdata->y_tilt = 0;
		wdata->tool_id = 0;
		wdata->p = 0;
	}

	input_report_abs(wdata->input, ABS_X, wdata->x);
	input_report_abs(wdata->input, ABS_Y, wdata->y);
	input_report_abs(wdata->input, ABS_DISTANCE, wdata->distance);
	input_report_abs(wdata->input, ABS_PRESSURE, wdata->p);
	input_report_abs(wdata->input, ABS_TILT_X, wdata->x_tilt);
	input_report_abs(wdata->input, ABS_TILT_Y, wdata->y_tilt);
	input_report_abs(wdata->input, ABS_MISC, wdata->tool_id);

	if (wdata->prev_tool_type != wdata->tool_type)
		input_report_key(wdata->input, wdata->prev_tool_type, 0);
	wdata->prev_tool_type = wdata->tool_type;
	input_report_key(wdata->input, wdata->tool_type, !!wdata->in_range);

	input_report_key(wdata->input, BTN_TOUCH, wdata->p > 1);

	input_event(wdata->input, EV_MSC, MSC_SERIAL, wdata->hserial);
	if (!wdata->in_range)
		wdata->hserial = -1;
}

static void set_abs(struct input_dev *input, unsigned int code,
		struct hid_field *field, int fuzz)
{
	int fmin = field->logical_minimum;
	int fmax = field->logical_maximum;
	input_set_abs_params(input, code, fmin, fmax, fuzz, 0);
	input_abs_set_res(input, code, hidinput_calc_abs_res(field, code));
}

static void wacom_feature_mapping(struct hid_device *hdev,
		struct hid_field *field, struct hid_usage *usage)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);

	switch (usage->hid) {
	case HID_DG_INPUTMODE:
		/* Ignore if value index is out of bounds. */
		if (usage->usage_index >= field->report_count) {
			dev_err(&hdev->dev, "HID_DG_INPUTMODE out of range\n");
			break;
		}

		wdata->inputmode = field->report->id;
		wdata->inputmode_index = usage->usage_index;

		break;
	}
}

static int wacom_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	unsigned usage_code = usage->hid;
	unsigned code;

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH))
		return 0;

	if ((usage_code & HID_MASK_UP_OFFSET) &&
	    ((usage_code & HID_MASK_UP_OFFSET) != HID_UP_OFFSET))
		return 0;

	/* remove leading 0xbe and included offset */
	usage_code &= ~(HID_MASK_UP_OFFSET | HID_MASK_USAGE_OFFSET);

	switch (usage_code & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:
		switch (usage_code) {
		case HID_GD_X:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_X);
			set_abs(hi->input, ABS_X, field, 4);
			return 1;
		case HID_GD_Y:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_Y);
			set_abs(hi->input, ABS_Y, field, 4);
			return 1;
		case HID_GD_Z:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_DISTANCE);
			set_abs(hi->input, ABS_DISTANCE, field, 0);
			wdata->z_max = field->logical_maximum;
			return 1;
		case HID_GD_RX:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_RX);
			set_abs(hi->input, ABS_RX, field, 0);
			return 1;
		case HID_GD_RY:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_RY);
			set_abs(hi->input, ABS_RY, field, 0);
			return 1;
		}
		return 1;

	case HID_UP_DIGITIZER:
		switch (usage_code) {
		case HID_DG_TIPPRESSURE:
			hid_map_usage(hi, usage, bit, max, EV_ABS,
					ABS_PRESSURE);
			set_abs(hi->input, ABS_PRESSURE, field, 0);
			input_set_capability(hi->input, EV_KEY, BTN_TOUCH);
			return 1;
		case HID_DG_X_TILT:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_TILT_X);
			set_abs(hi->input, ABS_TILT_X, field, 0);
			return 1;
		case HID_DG_Y_TILT:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_TILT_Y);
			set_abs(hi->input, ABS_TILT_Y, field, 0);
			return 1;
		case HID_DG_ALTITUDE:
			hid_map_usage(hi, usage, bit, max, EV_ABS,
					ABS_DISTANCE);
			set_abs(hi->input, ABS_DISTANCE, field, 0);
			return 1;
		case HID_DG_ERASER:
			hid_map_usage(hi, usage, bit, max, EV_KEY, BTN_TOOL_RUBBER);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_RUBBER);
			hid_map_usage(hi, usage, bit, max, EV_KEY, BTN_TOOL_PEN);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_PEN);
			return 1;
		}
		return 1;

	case HID_UP_BUTTON:
		code = usage->hid & HID_USAGE;
		if (!(usage->hid & HID_MASK_UP_OFFSET))
			code = BTN_0 + code - 1;
		hid_map_usage(hi, usage, bit, max, EV_KEY, code);
		input_set_capability(hi->input, EV_KEY, code);
		return 1;

	case HID_UP_WACOM:
		switch (usage_code) {
		case HID_WAC_SERIAL:
			hid_map_usage(hi, usage, bit, max, EV_MSC, MSC_SERIAL);
			input_set_capability(hi->input, EV_MSC, MSC_SERIAL);
			return 1;
		case HID_WAC_TOOL_ID:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MISC);
			input_set_abs_params(hi->input, ABS_MISC, 0, 0, 0, 0);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_PEN);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_RUBBER);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_BRUSH);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_PENCIL);
			input_set_capability(hi->input, EV_KEY, BTN_TOOL_AIRBRUSH);
			return 1;
		}
		return 1;
	}

	return 1;
}

static int wacom_input_mapped(struct hid_device *hdev, struct hid_input *hi,
	struct hid_field *field, struct hid_usage *usage, unsigned long **bit,
								int *max)
{
	struct input_dev *input = hi->input;

	if (usage->type == EV_KEY ||
	    usage->type == EV_ABS ||
	    usage->type == EV_MSC)
		set_bit(usage->type, hi->input->evbit);

	if ((hdev->product != USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) &&
	    (hdev->product != USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH))
		return -1;

	__set_bit(INPUT_PROP_POINTER, input->propbit);

	/* Basics */
	input->evbit[0] |= BIT(EV_KEY) | BIT(EV_ABS) | BIT(EV_REL);

	__set_bit(REL_WHEEL, input->relbit);

	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);
	__set_bit(BTN_LEFT, input->keybit);
	__set_bit(BTN_RIGHT, input->keybit);
	__set_bit(BTN_MIDDLE, input->keybit);

	/* Pad */
	input_set_capability(input, EV_MSC, MSC_SERIAL);

	__set_bit(BTN_0, input->keybit);
	__set_bit(BTN_1, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);

	/* Distance, rubber and mouse */
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_TOOL_MOUSE, input->keybit);

	switch (hdev->product) {
	case USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH:
		input_set_abs_params(input, ABS_X, 0, 16704, 4, 0);
		input_set_abs_params(input, ABS_Y, 0, 12064, 4, 0);
		input_set_abs_params(input, ABS_PRESSURE, 0, 511, 0, 0);
		input_set_abs_params(input, ABS_DISTANCE, 0, 32, 0, 0);
		break;
	case USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH:
		__set_bit(ABS_WHEEL, input->absbit);
		__set_bit(ABS_MISC, input->absbit);
		__set_bit(BTN_2, input->keybit);
		__set_bit(BTN_3, input->keybit);
		__set_bit(BTN_4, input->keybit);
		__set_bit(BTN_5, input->keybit);
		__set_bit(BTN_6, input->keybit);
		__set_bit(BTN_7, input->keybit);
		__set_bit(BTN_8, input->keybit);
		input_set_abs_params(input, ABS_WHEEL, 0, 71, 0, 0);
		input_set_abs_params(input, ABS_X, 0, 40640, 4, 0);
		input_set_abs_params(input, ABS_Y, 0, 25400, 4, 0);
		input_set_abs_params(input, ABS_PRESSURE, 0, 2047, 0, 0);
		input_set_abs_params(input, ABS_DISTANCE, 0, 63, 0, 0);
		input_set_abs_params(input, ABS_TILT_X, 0, 127, 0, 0);
		input_set_abs_params(input, ABS_TILT_Y, 0, 127, 0, 0);
		break;
	}

	return 0;
}

static void wacom_input_configured(struct hid_device *hdev,
		struct hid_input *hi)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);
	wdata->input = hi->input;

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH))
		return;

	__set_bit(ABS_WHEEL, wdata->input->absbit);
	input_set_abs_params(wdata->input, ABS_WHEEL, 0, 1023, 0, 0);
	__set_bit(ABS_Z, wdata->input->absbit);
	input_set_abs_params(wdata->input, ABS_Z, -900, 899, 0, 0);

	__set_bit(INPUT_PROP_DIRECT, wdata->input->propbit);
}

static int wacom_probe(struct hid_device *hdev,
		const struct hid_device_id *id)
{
	struct wacom_data *wdata;
	int ret;

	wdata = kzalloc(sizeof(*wdata), GFP_KERNEL);
	if (wdata == NULL) {
		hid_err(hdev, "can't alloc wacom descriptor\n");
		return -ENOMEM;
	}

	wdata->inputmode = -1;

	hid_set_drvdata(hdev, wdata);

	/* Parse the HID report now */
	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err_free;
	}

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH)) {
		ret = device_create_file(&hdev->dev, &dev_attr_speed);
		if (ret)
			hid_warn(hdev,
				 "can't create sysfs speed attribute err: %d\n",
				 ret);

#define OLED_INIT(OLED_ID)						\
	do {								\
		ret = device_create_file(&hdev->dev,			\
				&dev_attr_oled##OLED_ID##_img);		\
		if (ret)						\
			hid_warn(hdev,					\
			 "can't create sysfs oled attribute, err: %d\n", ret);\
	} while (0)

		OLED_INIT(0);
		OLED_INIT(1);
		OLED_INIT(2);
		OLED_INIT(3);
		OLED_INIT(4);
		OLED_INIT(5);
		OLED_INIT(6);
		OLED_INIT(7);
	}

	wdata->features = 0;
	wacom_set_features(hdev, 1);

	if (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH) {
		sprintf(hdev->name, "%s", "Wacom Intuos4 WL");
		ret = wacom_initialize_leds(hdev);
		if (ret)
			hid_warn(hdev,
				 "can't create led attribute, err: %d\n", ret);
	}

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH)) {
		wdata->battery.properties = wacom_battery_props;
		wdata->battery.num_properties = ARRAY_SIZE(wacom_battery_props);
		wdata->battery.get_property = wacom_battery_get_property;
		wdata->battery.name = "wacom_battery";
		wdata->battery.type = POWER_SUPPLY_TYPE_BATTERY;
		wdata->battery.use_for_apm = 0;


		ret = power_supply_register(&hdev->dev, &wdata->battery);
		if (ret) {
			hid_err(hdev,
			      "can't create sysfs battery attribute, err: %d\n",
			      ret);
			goto err_battery;
		}

		power_supply_powers(&wdata->battery, &hdev->dev);

		wdata->ac.properties = wacom_ac_props;
		wdata->ac.num_properties = ARRAY_SIZE(wacom_ac_props);
		wdata->ac.get_property = wacom_ac_get_property;
		wdata->ac.name = "wacom_ac";
		wdata->ac.type = POWER_SUPPLY_TYPE_MAINS;
		wdata->ac.use_for_apm = 0;

		ret = power_supply_register(&hdev->dev, &wdata->ac);
		if (ret) {
			hid_err(hdev,
				"can't create ac battery attribute, err: %d\n",
				ret);
			goto err_ac;
		}

		power_supply_powers(&wdata->ac, &hdev->dev);
	}
	return 0;

err_ac:
	power_supply_unregister(&wdata->battery);
err_battery:
	wacom_destroy_leds(hdev);
	device_remove_file(&hdev->dev, &dev_attr_oled0_img);
	device_remove_file(&hdev->dev, &dev_attr_oled1_img);
	device_remove_file(&hdev->dev, &dev_attr_oled2_img);
	device_remove_file(&hdev->dev, &dev_attr_oled3_img);
	device_remove_file(&hdev->dev, &dev_attr_oled4_img);
	device_remove_file(&hdev->dev, &dev_attr_oled5_img);
	device_remove_file(&hdev->dev, &dev_attr_oled6_img);
	device_remove_file(&hdev->dev, &dev_attr_oled7_img);
	device_remove_file(&hdev->dev, &dev_attr_speed);
	hid_hw_stop(hdev);
err_free:
	kfree(wdata);
	return ret;
}

static void wacom_remove(struct hid_device *hdev)
{
	struct wacom_data *wdata = hid_get_drvdata(hdev);

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH)) {
		wacom_destroy_leds(hdev);
		device_remove_file(&hdev->dev, &dev_attr_oled0_img);
		device_remove_file(&hdev->dev, &dev_attr_oled1_img);
		device_remove_file(&hdev->dev, &dev_attr_oled2_img);
		device_remove_file(&hdev->dev, &dev_attr_oled3_img);
		device_remove_file(&hdev->dev, &dev_attr_oled4_img);
		device_remove_file(&hdev->dev, &dev_attr_oled5_img);
		device_remove_file(&hdev->dev, &dev_attr_oled6_img);
		device_remove_file(&hdev->dev, &dev_attr_oled7_img);
		device_remove_file(&hdev->dev, &dev_attr_speed);
	}

	hid_hw_stop(hdev);

	if ((hdev->product == USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) ||
	    (hdev->product == USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH)) {
		power_supply_unregister(&wdata->battery);
		power_supply_unregister(&wdata->ac);
	}
	kfree(hid_get_drvdata(hdev));
}

static const struct hid_device_id wacom_devices[] = {
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_WACOM, USB_DEVICE_ID_WACOM_GRAPHIRE_BLUETOOTH) },
	{ HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_WACOM, USB_DEVICE_ID_WACOM_INTUOS4_BLUETOOTH) },
	{ HID_DEVICE(HID_BUS_ANY, HID_GROUP_WACOM, HID_ANY_ID, HID_ANY_ID) },

	{ }
};
MODULE_DEVICE_TABLE(hid, wacom_devices);

static struct hid_driver wacom_driver = {
	.name = "wacom",
	.id_table = wacom_devices,
	.probe = wacom_probe,
	.remove = wacom_remove,
	.raw_event = wacom_raw_event,
	.event = wacom_input_event,
	.report = wacom_input_report,
	.feature_mapping = wacom_feature_mapping,
	.input_mapping = wacom_input_mapping,
	.input_mapped = wacom_input_mapped,
	.input_configured = wacom_input_configured,
};
module_hid_driver(wacom_driver);

MODULE_DESCRIPTION("Driver for Wacom Graphire Bluetooth and Wacom Intuos4 WL");
MODULE_LICENSE("GPL");
