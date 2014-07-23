/*
 * Copyright (c) 2011-2014 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_DRIVER_H
#define _RMI_DRIVER_H

#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include "rmi_bus.h"

#define RMI_DRIVER_VERSION "1.6"

#define SYNAPTICS_INPUT_DEVICE_NAME "Synaptics RMI4 Touch Sensor"
#define SYNAPTICS_VENDOR_ID 0x06cb

#define GROUP(_attrs) { \
	.attrs = _attrs,  \
}

#define PDT_PROPERTIES_LOCATION 0x00EF
#define BSR_LOCATION 0x00FE

#define RMI_PDT_PROPS_HAS_BSR 0x02

struct rmi_driver_data {
	struct list_head function_list;

	struct rmi_device *rmi_dev;

	struct rmi_function *f01_container;
	bool f01_bootloader_mode;

	u32 attn_count;
	u32 irq_debug;	/* Should be bool, but debugfs wants u32 */
	bool gpio_held;
	int irq;
	int irq_flags;
	int num_of_irq_regs;
	int irq_count;
	unsigned long *irq_status;
	unsigned long *fn_irq_bits;
	unsigned long *current_irq_mask;
	unsigned long *new_irq_mask;
	struct mutex irq_mutex;

	/* Following are used when polling. */
	struct hrtimer poll_timer;
	struct work_struct poll_work;
	ktime_t poll_interval;

	u8 pdt_props;
	u8 bsr;

	bool enabled;
#ifdef CONFIG_PM_SLEEP
	bool suspended;
	struct mutex suspend_mutex;

	void *pm_data;
	int (*pre_suspend) (const void *pm_data);
	int (*post_suspend) (const void *pm_data);
	int (*pre_resume) (const void *pm_data);
	int (*post_resume) (const void *pm_data);
#endif

#ifdef CONFIG_RMI4_DEBUG
	struct dentry *debugfs_delay;
	struct dentry *debugfs_xport;
	struct dentry *debugfs_reg_ctl;
	struct dentry *debugfs_reg;
	struct dentry *debugfs_irq;
	struct dentry *debugfs_attn_count;
	u16 reg_debug_addr;
	u8 reg_debug_size;
#endif

	void *data;
};

#define RMI_PDT_ENTRY_SIZE 6
#define RMI_PDT_FUNCTION_VERSION_MASK   0x60
#define RMI_PDT_INT_SOURCE_COUNT_MASK   0x07

#define PDT_START_SCAN_LOCATION 0x00e9
#define PDT_END_SCAN_LOCATION	0x0005
#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)

struct pdt_entry {
	u16 page_start;
	u8 query_base_addr;
	u8 command_base_addr;
	u8 control_base_addr;
	u8 data_base_addr;
	u8 interrupt_source_count;
	u8 function_version;
	u8 function_number;
};

int rmi_read_pdt_entry(struct rmi_device *rmi_dev, struct pdt_entry *entry,
			u16 pdt_address);

bool rmi_is_physical_driver(struct device_driver *);
int rmi_register_physical_driver(void);
void rmi_unregister_physical_driver(void);

int rmi_register_f01_handler(void);
void rmi_unregister_f01_handler(void);

#endif
