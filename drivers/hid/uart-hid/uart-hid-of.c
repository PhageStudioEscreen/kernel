/*
 * HID over UART Open Firmware Subclass
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code was forked out of the core code, which was partly based on
 * "USB HID support for Linux":
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007-2008 Oliver Neukum
 *  Copyright (c) 2006-2010 Jiri Kosina
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/serdev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#include "uart-hid.h"

struct uart_hid_of {
    struct uarthid_ops ops;

    struct serdev_device      *serdev;
    struct regulator_bulk_data supplies[2];
    int                        post_power_delay_ms;
};

static int uart_hid_of_power_up(struct uarthid_ops *ops)
{
    struct uart_hid_of *uhid_of = container_of(ops, struct uart_hid_of, ops);
    struct device      *dev     = &uhid_of->serdev->dev;
    int                 ret;

    ret = regulator_bulk_enable(ARRAY_SIZE(uhid_of->supplies),
                                uhid_of->supplies);
    if (ret) {
        dev_warn(dev, "Failed to enable supplies: %d\n", ret);
        return ret;
    }

    if (uhid_of->post_power_delay_ms)
        msleep(uhid_of->post_power_delay_ms);

    return 0;
}

static void uart_hid_of_power_down(struct uarthid_ops *ops)
{
    struct uart_hid_of *uhid_of = container_of(ops, struct uart_hid_of, ops);

    regulator_bulk_disable(ARRAY_SIZE(uhid_of->supplies),
                           uhid_of->supplies);
}

static int uart_hid_of_probe(struct serdev_device *serdev)
{
    struct device      *dev = &serdev->dev;
    struct uart_hid_of *uhid_of;
    u16                 hid_descriptor_address;
    u32                 baudrate = 115200;
    u32                 quirks   = 0;
    int                 ret;
    u32                 val;

    uhid_of = devm_kzalloc(&serdev->dev, sizeof(*uhid_of), GFP_KERNEL);
    if (!uhid_of)
        return -ENOMEM;

    uhid_of->serdev              = serdev;
    uhid_of->ops.power_up        = uart_hid_of_power_up;
    uhid_of->ops.power_down      = uart_hid_of_power_down;
    uhid_of->post_power_delay_ms = 0;

    ret = of_property_read_u32(dev->of_node, "hid-descr-addr", &val);
    if (ret) {
        dev_err(&serdev->dev, "hid register address not provided\n");
        return -ENODEV;
    }
    if (val >> 16) {
        dev_err(&serdev->dev, "bad hid register address: 0x%08x\n",
                val);
        return -EINVAL;
    }
    hid_descriptor_address = val;

    ret = of_property_read_u32(dev->of_node, "baudrate", &val);
    if (ret) {
        dev_warn(&serdev->dev, "baudrate not provided, use default %u\n", baudrate);
    } else {
        baudrate = val;
    }

    if (!device_property_read_u32(&serdev->dev, "post-power-on-delay-ms",
                                  &val))
        uhid_of->post_power_delay_ms = val;

    uhid_of->supplies[0].supply = "vdd";
    uhid_of->supplies[1].supply = "vddl";
    ret                         = devm_regulator_bulk_get(&serdev->dev,
                                                          ARRAY_SIZE(uhid_of->supplies),
                                                          uhid_of->supplies);
    if (ret) {
        dev_err(&serdev->dev, "Failed to request supplies: %d\n", ret);
        return ret;
    }

    dev_info(&serdev->dev, "new device with baudrate %u\n", baudrate);

    return uart_hid_core_probe(serdev, &uhid_of->ops,
                               hid_descriptor_address, baudrate, quirks);
}

#ifdef CONFIG_OF
static const struct of_device_id uart_hid_of_match[] = {
    { .compatible = "pgs,hid-over-uart" },
    {},
};
MODULE_DEVICE_TABLE(of, uart_hid_of_match);
#endif

static struct serdev_device_driver uart_hid_of_driver = {
    .driver = {
        .name           = "uart_hid_of",
        .pm             = &uart_hid_core_pm,
        .probe_type     = PROBE_PREFER_ASYNCHRONOUS,
        .of_match_table = of_match_ptr(uart_hid_of_match),
    },

    .probe  = uart_hid_of_probe,
    .remove = uart_hid_core_remove,
};

module_serdev_device_driver(uart_hid_of_driver);

MODULE_DESCRIPTION("HID over UART OF driver");
MODULE_AUTHOR("Egahp <2687434412@qq.com>");
MODULE_LICENSE("GPL v2");
