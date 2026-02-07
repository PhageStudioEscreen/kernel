/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef UART_HID_H
#define UART_HID_H

#include <linux/serdev.h>

/**
 * struct uarthid_ops - Ops provided to the core.
 *
 * @power_up: do sequencing to power up the device.
 * @power_down: do sequencing to power down the device.
 * @shutdown_tail: called at the end of shutdown.
 */
struct uarthid_ops {
    int (*power_up)(struct uarthid_ops *ops);
    void (*power_down)(struct uarthid_ops *ops);
};

int  uart_hid_core_probe(struct serdev_device *serdev, struct uarthid_ops *ops,
                         u16 hid_descriptor_address, u32 baudrate, u32 quirks);
void uart_hid_core_remove(struct serdev_device *serdev);

extern const struct dev_pm_ops uart_hid_core_pm;

#endif
