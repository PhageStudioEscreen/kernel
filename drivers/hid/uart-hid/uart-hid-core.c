/*
 * HID over UART protocol implementation
 *
 * Copyright (c) 2025 Egahp
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc
 *
 * This code is partly based on "USB HID support for Linux":
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

#include <linux/module.h>
#include <linux/serdev.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/hid.h>
#include <linux/mutex.h>
#include <asm/unaligned.h>

#include "../hid-ids.h"
#include "uart-hid.h"

/* Magic Header*/
#define UART_HID_MAGIC_INREP  0x50455249 /* 'I' 'R' 'E' 'P' */
#define UART_HID_MAGIC_OUTREP 0x5045524F /* 'O' 'R' 'E' 'P' */
#define UART_HID_MAGIC_INDAT  0x54414449 /* 'I' 'D' 'A' 'T' */
#define UART_HID_MAGIC_OUTDAT 0x5441444F /* 'O' 'D' 'A' 'T' */

/* Command opcodes */
#define UART_HID_OPCODE_RESET        0x01
#define UART_HID_OPCODE_GET_REPORT   0x02
#define UART_HID_OPCODE_SET_REPORT   0x03
#define UART_HID_OPCODE_GET_IDLE     0x04
#define UART_HID_OPCODE_SET_IDLE     0x05
#define UART_HID_OPCODE_GET_PROTOCOL 0x06
#define UART_HID_OPCODE_SET_PROTOCOL 0x07
#define UART_HID_OPCODE_SET_POWER    0x08
#define UART_HID_OPCODE_GET_WORK     0x09
#define UART_HID_OPCODE_SET_WORK     0x0A

/* Report Type */
#define UART_HID_RTYPE_INPUT   1
#define UART_HID_RTYPE_OUTPUT  2
#define UART_HID_RTYPE_FEATURE 3

/* flags */
#define UART_HID_WORKING    0
#define UART_HID_RX_PENDING 1

/* power state */
#define UART_HID_PWR_ON    0x00
#define UART_HID_PWR_SLEEP 0x01

/* work state */
#define UART_HID_WORK_START 0x00
#define UART_HID_WORK_STOP  0x01

#define UART_HID_TIMEOUT_AUTO 0x00000000

/* debug option */
static bool debug;
module_param(debug, bool, 0444);
MODULE_PARM_DESC(debug, "print a lot of debug information");

#define uart_hid_dbg(uhid, fmt, arg...)                       \
    do {                                                      \
        if (true)                                             \
            dev_printk(KERN_DEBUG, &(uhid)->serdev->dev, fmt, \
                       ##arg);                                \
    } while (0)

struct uart_hid_desc {
    __le16 wHIDDescRegister;
    __le16 wHIDDescLength;
    __le16 bcdVersion;
    __le16 wReportDescLength;
    __le16 wReportDescRegister;
    __le16 wInputRegister;
    __le16 wMaxInputLength;
    __le16 wOutputRegister;
    __le16 wMaxOutputLength;
    __le16 wCommandRegister;
    __le16 wDataRegister;
    __le16 wVendorID;
    __le16 wProductID;
    __le16 wVersionID;
    __le32 reserved;
} __packed;

/* The main device structure */
struct uart_hid {
    struct serdev_device *serdev; /* serial device */
    struct hid_device    *hid;    /* pointer to corresponding HID dev */
    struct uart_hid_desc  hdesc;  /* the HID Descriptor */

    size_t irepsize; /* input report buffer size */
    size_t idatsize; /* input data buffer size */
    size_t odatsize; /* output data/report buffer size */
    size_t irawsize; /* input raw buffer size */

    u8 *irepbuf; /* input report buffer */
    u8 *idatbuf; /* input data buffer */
    u8 *odatbuf; /* output data/report buffer */
    u8 *irawbuf; /* input raw buffer */

    u8 *recvbuf;  /* receive buffer */
    u16 recvsize; /* actual receive size */
    u16 wantsize; /* want receive size */

    unsigned long flags;    /* device flags */
    unsigned long quirks;   /* Various quirks */
    u32           baudrate; /* current baudrate */

    wait_queue_head_t   wait; /* For waiting the interrupt */
    struct mutex        reset_lock;
    struct uarthid_ops *ops;
};

static const struct uart_hid_quirks {
    __u16 idVendor;
    __u16 idProduct;
    __u32 quirks;
} uart_hid_quirks[] = {
    { 0, 0 },
};

/**
 * uart_hid_lookup_quirk: return any quirks associated with a UART HID device
 * @idVendor: the 16-bit vendor ID
 * @idProduct: the 16-bit product ID
 *
 * Returns: a u32 quirks value.
 */
static u32 uart_hid_lookup_quirk(const u16 idVendor, const u16 idProduct)
{
    u32 quirks = 0;
    int n;

    for (n = 0; uart_hid_quirks[n].idVendor; n++)
        if (uart_hid_quirks[n].idVendor == idVendor &&
            (uart_hid_quirks[n].idProduct == (__u16)HID_ANY_ID ||
             uart_hid_quirks[n].idProduct == idProduct))
            quirks = uart_hid_quirks[n].quirks;

    return quirks;
}

/**
 *  uart_hid_xfer - Transfer data to/from a UART HID device
 * @uhid: the uart hid device
 * @send_buf: buffer containing data to send
 * @send_len: length of data to send
 * @recv_buf: buffer to store received data
 * @recv_len: length of data to receive
 * @timeout: operation timeout, 0: auto, other: timeout in ms
 */
static int uart_hid_xfer(struct uart_hid *uhid,
                         u8 *send_buf, int send_len,
                         u8 *recv_buf, int recv_len, u32 timeout)
{
    struct serdev_device *serdev = uhid->serdev;
    u32                   timeout_actual;
    int                   ret;

    /* if need recv, set bit before send */
    if (recv_len) {
        /* set wantsize */
        uhid->recvsize = 0;
        uhid->wantsize = recv_len;
        if (recv_len < (sizeof(__le32) + sizeof(__le16))) {
            return -EINVAL;
        }
        set_bit(UART_HID_RX_PENDING, &uhid->flags);
    }

    if (send_len) {
        uart_hid_dbg(uhid, "%s: send %*ph\n", __func__, send_len,
                     send_buf);

        ret = serdev_device_write(serdev, send_buf, send_len,
                                  MAX_SCHEDULE_TIMEOUT);

        if (ret != send_len) {
            clear_bit(UART_HID_RX_PENDING, &uhid->flags);
            return ret < 0 ? ret : -EIO;
        }
    }

    if (recv_len) {
        if (timeout == UART_HID_TIMEOUT_AUTO) {
            u32 xfer_time  = (recv_len * 10 * 1000 / uhid->baudrate);
            timeout_actual = xfer_time < 25 ? 50 : 50 + xfer_time;
        } else {
            timeout_actual = timeout;
        }

        uart_hid_dbg(uhid, "%s: read %d byte, timeout %u ms...\n",
                     __func__, recv_len, timeout_actual);

        if (!wait_event_timeout(
                uhid->wait,
                !test_bit(UART_HID_RX_PENDING, &uhid->flags),
                msecs_to_jiffies(timeout_actual))) {
            clear_bit(UART_HID_RX_PENDING, &uhid->flags);
            uart_hid_dbg(uhid, "%s: read timeout\n", __func__);
            return -EIO;
        }

        if (uhid->recvsize != uhid->wantsize) {
            /* dMagic + 0x00 + 0x00  indicates that
               the device's function is not implemented */
            if ((uhid->recvsize == (sizeof(__le32) + sizeof(__le16))) &&
                (le16_to_cpup((__le16 *)(uhid->recvbuf + sizeof(__le32))) == 0x0000)) {
                return -ENOSYS;
            } else {
                return -EIO;
            }
        }

        memcpy(recv_buf, uhid->recvbuf, uhid->recvsize);
    }

    return 0;
}

static int uart_hid_fetch_reg(struct uart_hid *uhid, __le16 reg,
                              void *buf, size_t len)
{
    int    ret;
    size_t odatlen = 0;
    size_t irawlen = len + sizeof(__le32);

    /* dMagic */
    *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTDAT);
    odatlen += sizeof(__le32);

    /* wReg */
    *(__le16 *)(uhid->odatbuf + odatlen) = reg;
    odatlen += sizeof(__le16);

    ret = uart_hid_xfer(uhid, uhid->odatbuf, odatlen,
                        uhid->irawbuf, irawlen, UART_HID_TIMEOUT_AUTO);
    if (ret) {
        dev_err(&uhid->serdev->dev,
                "failed to fetch reg from device: %d\n", ret);
        return ret;
    }

    /* remove dMagic */
    memcpy(buf, (uhid->irawbuf + sizeof(__le32)), len);
    return 0;
}

static size_t uart_hid_encode_command(u8 *buf, u8 opcode,
                                      int report_type, int report_id)
{
    size_t length = 0;

    if (report_id < 0x0F) {
        buf[length++] = report_type << 4 | report_id;
        buf[length++] = opcode;
    } else {
        buf[length++] = report_type << 4 | 0x0F;
        buf[length++] = opcode;
        buf[length++] = report_id;
    }

    return length;
}

static int uart_hid_get_report(struct uart_hid *uhid,
                               u8 report_type, u8 report_id, u8 *recv_buf, size_t recv_len)
{
    int    ret;
    size_t odatlen = 0;
    size_t idatlen = sizeof(__le32) + sizeof(__le16) + recv_len;
    size_t ret_count;

    uart_hid_dbg(uhid, "%s\n", __func__);

    /* dMagic */
    *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTDAT);
    odatlen += sizeof(__le32);

    /* wReg */
    *(__le16 *)(uhid->odatbuf + odatlen) = uhid->hdesc.wCommandRegister;
    odatlen += sizeof(__le16);

    /* wCommand + [bReportId] */
    odatlen += uart_hid_encode_command(uhid->odatbuf + odatlen,
                                       UART_HID_OPCODE_GET_REPORT,
                                       report_type, report_id);

    /* wReg */
    put_unaligned_le16(le16_to_cpu(uhid->hdesc.wDataRegister),
                       uhid->odatbuf + odatlen);
    odatlen += sizeof(__le16);

    ret = uart_hid_xfer(uhid, uhid->odatbuf, odatlen, uhid->idatbuf,
                        idatlen, UART_HID_TIMEOUT_AUTO);
    if (ret) {
        dev_err(&uhid->serdev->dev,
                "failed to get report from device: %d\n", ret);
        return ret;
    }

    /* wLength */
    ret_count = le16_to_cpup((__le16 *)(uhid->idatbuf + sizeof(__le32)));
    if (ret_count != (sizeof(__le16) + recv_len)) {
        return -EIO;
    }

    /* remove dMagic + wLength */
    memcpy(recv_buf, uhid->idatbuf + sizeof(__le32) + sizeof(__le16), recv_len);

    if (report_id && recv_len && recv_buf[0] != report_id) {
        dev_err(&uhid->serdev->dev,
                "device returned incorrect report (%d vs %d expected)\n",
                recv_buf[0], report_id);
        return -EINVAL;
    }

    return recv_len;
}

static size_t uart_hid_format_report(u8 *buf, const u8 *data,
                                     size_t size)
{
    size_t length = sizeof(__le16); /* reserve space to store size */

    memcpy(buf + length, data, size);
    length += size;

    /* Store overall size in the beginning of the buffer */
    put_unaligned_le16(length, buf);

    return length;
}

/**
 * uart_hid_set_or_output_report: forward an incoming report to the device
 * @uhid: the uart hid device
 * @report_type: 0x03 for HID_FEATURE_REPORT ; 0x02 for HID_OUTPUT_REPORT
 * @report_id: the report ID
 * @buf: the actual data to transfer, without the report ID
 * @data_len: size of buf
 * @do_set: true: use SET_REPORT HID command, false: send plain OUTPUT report
 */
static int uart_hid_set_or_output_report(struct uart_hid *uhid, u8 report_type,
                                         u8 report_id, const u8 *buf,
                                         size_t data_len, bool do_set)
{
    int    ret;
    size_t odatlen = sizeof(__le32) + /* dMagic */
                     sizeof(__le16) + /* wReg */
                     sizeof(__le16) + /* wCommand */
                     sizeof(u8) +     /* bReportId */
                     sizeof(__le16) + /* wReg */
                     sizeof(__le16) + /* wLength */
                     data_len;

    uart_hid_dbg(uhid, "uart_hid_%s_report\n", do_set ? "set" : "output");

    if (odatlen > uhid->odatsize)
        return -EINVAL;

    /* output report is disabled when wMaxOutputLength equal 0 */
    if (!do_set && (le16_to_cpu(uhid->hdesc.wMaxOutputLength) == 0))
        return -ENOSYS;

    if (!do_set && ((data_len + sizeof(__le16)) >
                    le16_to_cpu(uhid->hdesc.wMaxOutputLength)))
        return -EINVAL;

    odatlen = 0;

    if (do_set) {
        /* dMagic */
        *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTDAT);
        odatlen += sizeof(__le32);

        /* wReg */
        *(__le16 *)(uhid->odatbuf + odatlen) = uhid->hdesc.wCommandRegister;
        odatlen += sizeof(__le16);

        /* wCommand + [bReportId] */
        odatlen += uart_hid_encode_command(uhid->odatbuf + odatlen,
                                           UART_HID_OPCODE_SET_REPORT,
                                           report_type, report_id);

        /* wReg */
        put_unaligned_le16(le16_to_cpu(uhid->hdesc.wDataRegister),
                           uhid->odatbuf + odatlen);
        odatlen += sizeof(__le16);
    } else {
        /* dMagic */
        *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTREP);
        odatlen += sizeof(__le32);

        /* wReg */
        *(__le16 *)(uhid->odatbuf + odatlen) = uhid->hdesc.wOutputRegister;
        odatlen += sizeof(__le16);
    }

    odatlen += uart_hid_format_report(uhid->odatbuf + odatlen, buf, data_len);

    ret = uart_hid_xfer(uhid, uhid->odatbuf, odatlen, NULL, 0, UART_HID_TIMEOUT_AUTO);
    if (ret) {
        dev_err(&uhid->serdev->dev,
                "failed to %s report to device: %d\n", do_set ? "set" : "output", ret);
        return ret;
    }

    return data_len;
}

static int uart_hid_set_power_command(struct uart_hid *uhid, int power_state)
{
    size_t odatlen = 0;

    /* dMagic */
    *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTDAT);
    odatlen += sizeof(__le32);

    /* wReg */
    *(__le16 *)(uhid->odatbuf + odatlen) = uhid->hdesc.wCommandRegister;
    odatlen += sizeof(__le16);

    /* wCommand + [bReportId] */
    odatlen += uart_hid_encode_command(uhid->odatbuf + odatlen,
                                       UART_HID_OPCODE_SET_POWER,
                                       0, power_state);

    return uart_hid_xfer(uhid, uhid->odatbuf, odatlen, NULL, 0, UART_HID_TIMEOUT_AUTO);
}

static int uart_hid_set_power(struct uart_hid *uhid, int power_state)
{
    int ret;

    uart_hid_dbg(uhid, "%s: %s\n", __func__,
                 power_state == UART_HID_PWR_ON ? "power on" : "power sleep");

    ret = uart_hid_set_power_command(uhid, power_state);
    if (ret)
        dev_err(&uhid->serdev->dev,
                "failed to change power setting.\n");

    /*
     * The HID over UART specification states that if a DEVICE needs time
     * after the PWR_ON request, it should utilise CLOCK stretching.
     * However, it has been observered that the Windows driver provides a
     * 1ms sleep between the PWR_ON and RESET requests.
     * According to Goodix Windows even waits 60 ms after (other?)
     * PWR_ON requests. Testing has confirmed that several devices
     * will not work properly without a delay after a PWR_ON request.
     */
    if (!ret && power_state == UART_HID_PWR_ON)
        msleep(60);

    return ret;
}

static size_t uart_hid_format_work(u8 *buf, u32 baudrate)
{
    size_t length = sizeof(__le16); /* reserve space to store size */

    /* store 4byte baudrate */
    put_unaligned_le32(cpu_to_le32(baudrate), buf + length);
    length += sizeof(__le32);

    /* Store overall size in the beginning of the buffer */
    put_unaligned_le16(length, buf);

    return length;
}

static int uart_hid_set_work_command(struct uart_hid *uhid, int work_state,
                                     u32 baudrate)
{
    size_t odatlen = 0;

    /* dMagic */
    *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTDAT);
    odatlen += sizeof(__le32);

    /* wReg */
    *(__le16 *)(uhid->odatbuf + odatlen) = uhid->hdesc.wCommandRegister;
    odatlen += sizeof(__le16);

    /* wCommand + [bReportId] */
    odatlen += uart_hid_encode_command(uhid->odatbuf + odatlen,
                                       UART_HID_OPCODE_SET_WORK,
                                       0, work_state);

    /* wReg */
    put_unaligned_le16(le16_to_cpu(uhid->hdesc.wDataRegister),
                       uhid->odatbuf + odatlen);
    odatlen += sizeof(__le16);

    odatlen += uart_hid_format_work(uhid->odatbuf + odatlen, baudrate);

    return uart_hid_xfer(uhid, uhid->odatbuf, odatlen, NULL, 0, UART_HID_TIMEOUT_AUTO);
}

static int uart_hid_set_work(struct uart_hid *uhid, int work_state)
{
    int ret;

    uart_hid_dbg(uhid, "%s: %s %u\n", __func__,
                 work_state == UART_HID_WORK_START ? "start" : "stop", uhid->baudrate);

    mutex_lock(&uhid->reset_lock);

    ret = uart_hid_set_work_command(uhid, work_state, uhid->baudrate);
    if (ret) {
        dev_err(&uhid->serdev->dev, "failed to set_work.\n");
        goto out_unlock;
    }

    ret = serdev_device_set_baudrate(uhid->serdev, uhid->baudrate);
    if (ret != uhid->baudrate) {
        dev_err(&uhid->serdev->dev, "failed to set baudrate %u\n", uhid->baudrate);
        goto out_unlock;
    }

out_unlock:
    mutex_unlock(&uhid->reset_lock);
    return ret;
}

static int uart_hid_execute_reset(struct uart_hid *uhid)
{
    int    ret;
    size_t odatlen = 0;
    size_t irawlen = sizeof(__le32) + sizeof(__le16);

    uart_hid_dbg(uhid, "resetting...\n");

    /* dMagic */
    *(__le32 *)(uhid->odatbuf + odatlen) = cpu_to_le32(UART_HID_MAGIC_OUTDAT);
    odatlen += sizeof(__le32);

    /* wReg */
    *(__le16 *)(uhid->odatbuf + odatlen) = uhid->hdesc.wCommandRegister;
    odatlen += sizeof(__le16);

    /* wCommand + [bReportId] */
    odatlen += uart_hid_encode_command(uhid->odatbuf + odatlen,
                                       UART_HID_OPCODE_RESET,
                                       0, 0);

    ret = uart_hid_xfer(uhid, uhid->odatbuf, odatlen, uhid->irawbuf, irawlen, 3000);
    if (ret || (le16_to_cpup((__le16 *)(uhid->irawbuf + sizeof(__le32))) != 0x0000)) {
        dev_err(&uhid->serdev->dev, "failed to reset device.\n");
        ret = ret < 0 ? ret : -EIO;
    }

    return ret;
}

static int uart_hid_hwreset(struct uart_hid *uhid)
{
    int ret;

    uart_hid_dbg(uhid, "%s\n", __func__);

    /*
     * This prevents sending feature reports while the device is
     * being reset. Otherwise we may lose the reset complete
     * interrupt.
     */
    mutex_lock(&uhid->reset_lock);

    ret = uart_hid_set_power(uhid, UART_HID_PWR_ON);
    if (ret)
        goto out_unlock;

    ret = uart_hid_execute_reset(uhid);
    if (ret) {
        uart_hid_set_power(uhid, UART_HID_PWR_SLEEP);
        goto out_unlock;
    }

    ret = uart_hid_set_power(uhid, UART_HID_PWR_ON);
    if (ret)
        goto out_unlock;

    serdev_device_set_baudrate(uhid->serdev, uhid->baudrate);

out_unlock:
    mutex_unlock(&uhid->reset_lock);
    return ret;
}

static int uart_hid_recv(struct serdev_device *serdev, const u8 *data,
                         size_t count)
{
    struct uart_hid *uhid = serdev_device_get_drvdata(serdev);
    u32              used = 0;
    u32              dMagic;
    u16              wLength;

next_report:

    /* dMagic + wLength */
    if (count < sizeof(__le32) + sizeof(__le16)) {
        return used;
    }

    /* copy for properly aligned */
    memcpy(uhid->irepbuf, data, sizeof(__le32) + sizeof(__le16));

    dMagic  = le32_to_cpup((__le32 *)uhid->irepbuf);
    wLength = le16_to_cpup((__le16 *)(uhid->irepbuf + sizeof(__le32)));

    if (dMagic == UART_HID_MAGIC_INDAT) {
        if (test_bit(UART_HID_RX_PENDING, &uhid->flags)) {
            u16 wantsize = uhid->wantsize;

            /* Only the fetch report descriptor does not have a length field,
               but the first two bytes will not be zero. */
            if (wLength == 0x0000) {
                wantsize = sizeof(__le32) + sizeof(__le16);
            }

            if (wantsize > count) {
                uart_hid_dbg(uhid, "%s: read (%d/%d) continue\n", __func__, count, wantsize);
                return used;
            }

            uart_hid_dbg(uhid, "%s: read (%d/%d) done\n", __func__, count, wantsize);

            uhid->recvsize = wantsize;
            memcpy(uhid->recvbuf, data, wantsize);

            uart_hid_dbg(uhid, "%s: read %*ph\n", __func__, uhid->recvsize, uhid->recvbuf);

            clear_bit(UART_HID_RX_PENDING, &uhid->flags);
            wake_up(&uhid->wait);

            /* next report */
            used += wantsize;
            data += wantsize;
            count -= wantsize;
            goto next_report;
        }
    } else if (dMagic == UART_HID_MAGIC_INREP) {
        u16 insize = sizeof(__le32) + wLength;

        if (wLength == 0x0000) {
            dev_warn(&uhid->serdev->dev, "%s: zero input report\n", __func__);
            used += insize;
            data += insize;
            count -= insize;
            goto next_report;
        }

        if (wLength < 2) {
            dev_err(&uhid->serdev->dev,
                    "%s: length underflow (%d/%d)\n",
                    __func__, insize, (sizeof(__le32) + sizeof(__le16)));
            goto drop;
        }

        if (insize > uhid->irepsize) {
            dev_err(&uhid->serdev->dev,
                    "%s: length overflow (%d/%d)\n",
                    __func__, insize, uhid->irepsize);
            goto drop;
        }

        if (insize > count) {
            dev_warn(&uhid->serdev->dev,
                     "%s: input (%d/%d) continue\n", __func__,
                     count, insize);
            return used;
        }

        uart_hid_dbg(uhid, "%s: input (%d/%d) done\n", __func__, count, insize);

        memcpy(uhid->irepbuf, data, insize);

        if (test_bit(UART_HID_WORKING, &uhid->flags)) {
            if (uhid->hid->group != HID_GROUP_RMI)
                pm_wakeup_event(&uhid->serdev->dev, 0);

            hid_input_report(uhid->hid, HID_INPUT_REPORT,
                             uhid->irepbuf + sizeof(__le32) + sizeof(__le16),
                             wLength - sizeof(__le16), 1);
        } else {
            uart_hid_dbg(uhid, "%s: input (%d/%d) drop", __func__, count, insize);
        }

        uart_hid_dbg(uhid, "%s: input %*ph\n", __func__, insize, uhid->irepbuf);

        used += insize;
        data += insize;
        count -= insize;
        goto next_report;
    }

drop:
    /* drop just 1 byte to find the next packet */
    used++;
    data++;
    count--;

    goto next_report;
}

/*
 * Traverse the supplied list of reports and find the longest
 */
static void uart_hid_find_max_report(struct hid_device *hid, unsigned int type,
                                     unsigned int *max)
{
    struct hid_report *report;
    unsigned int       size;

    /* We should not rely on wMaxInputLength, as some devices may set it to
     * a wrong length. */
    list_for_each_entry(report, &hid->report_enum[type].report_list, list)
    {
        /* align up ((nbit - 1) / 8 + 1) + report_id ? 1 : 0 */
        size = ((report->size - 1) >> 3) + 1 + hid->report_enum[type].numbered;
        if (*max < size)
            *max = size;
    }
}

static void uart_hid_free_buffers(struct uart_hid *uhid)
{
    if (uhid->irepbuf)
        kfree(uhid->irepbuf);

    if (uhid->idatbuf)
        kfree(uhid->idatbuf);

    if (uhid->odatbuf)
        kfree(uhid->odatbuf);

    if (uhid->irawbuf)
        kfree(uhid->irawbuf);

    if (uhid->recvbuf)
        kfree(uhid->recvbuf);

    uhid->irepbuf = NULL;
    uhid->idatbuf = NULL;
    uhid->odatbuf = NULL;
    uhid->irawbuf = NULL;
    uhid->recvbuf = NULL;

    uhid->irepsize = 0;
    uhid->idatsize = 0;
    uhid->odatsize = 0;
    uhid->irawsize = 0;
}

static int uart_hid_alloc_buffers(struct uart_hid *uhid, size_t irepsize, size_t idatsize, size_t odatsize)
{
    uhid->irepsize = sizeof(__le32) + sizeof(__le16) + irepsize;
    uhid->idatsize = sizeof(__le32) + sizeof(__le16) + idatsize;
    uhid->odatsize = sizeof(__le32) + /* dMagic */
                     sizeof(__le16) + /* wReg */
                     sizeof(__le16) + /* wCommand */
                     sizeof(u8) +     /* bReportId */
                     sizeof(__le16) + /* wReg */
                     sizeof(__le16) + /* wLength */
                     odatsize;
    uhid->irawsize = HID_MAX_DESCRIPTOR_SIZE;

    uhid->irepbuf = kzalloc(uhid->irepsize, GFP_KERNEL);
    uhid->idatbuf = kzalloc(uhid->idatsize, GFP_KERNEL);
    uhid->odatbuf = kzalloc(uhid->odatsize, GFP_KERNEL);
    uhid->irawbuf = kzalloc(uhid->irawsize, GFP_KERNEL);

    uhid->recvbuf = kzalloc(HID_MAX_DESCRIPTOR_SIZE, GFP_KERNEL);

    if (!uhid->irepbuf || !uhid->idatbuf || !uhid->odatbuf ||
        !uhid->recvbuf || !uhid->irawbuf) {
        uart_hid_free_buffers(uhid);
        return -ENOMEM;
    }

    return 0;
}

static int uart_hid_get_raw_report(struct hid_device *hid,
                                   u8 report_type, u8 report_id,
                                   u8 *buf, size_t count)
{
    struct serdev_device *serdev = hid->driver_data;
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);
    int                   ret;

    if (report_type == HID_OUTPUT_REPORT)
        return -EINVAL;

    /* Byte 0 is the report_id. Report data starts at byte 1.*/
    buf[0] = report_id;
    if (report_id == 0x0) {
        /* Offset the return buffer by 1, so that the report ID
           will remain in byte 0. */
        buf++;
        count--;
    }

    ret = uart_hid_get_report(
        uhid,
        report_type == HID_FEATURE_REPORT ? UART_HID_RTYPE_FEATURE : UART_HID_RTYPE_INPUT,
        report_id, buf, count);

    /* count also the report id */
    if (ret > 0 && (report_id == 0))
        ret++;

    return ret;
}

static int uart_hid_set_raw_report(struct hid_device *hid,
                                   u8 report_type, u8 report_id,
                                   const u8 *buf, size_t count, bool do_set)
{
    struct serdev_device *serdev = hid->driver_data;
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);
    int                   ret;

    if (report_type == HID_INPUT_REPORT)
        return -EINVAL;

    /* Byte 0 is the report_id. Report data starts at byte 1.*/
    if (report_id == 0x00) {
        /* Don't send the Report ID */
        buf++;
        count--;
    }

    mutex_lock(&uhid->reset_lock);

    ret = uart_hid_set_or_output_report(
        uhid,
        report_type == HID_FEATURE_REPORT ? UART_HID_RTYPE_FEATURE : UART_HID_RTYPE_OUTPUT,
        report_id, buf, count, do_set);

    mutex_unlock(&uhid->reset_lock);

    /* count also the report id */
    if (ret > 0 && (report_id == 0))
        ret++;

    return ret;
}

static int uart_hid_output_report(struct hid_device *hid, u8 *buf, size_t count)
{
    return uart_hid_set_raw_report(hid, HID_OUTPUT_REPORT, buf[0], buf, count, false);
}

static int uart_hid_raw_request(struct hid_device *hid, unsigned char reportnum,
                                __u8 *buf, size_t len, unsigned char rtype,
                                int reqtype)
{
    switch (reqtype) {
        case HID_REQ_GET_REPORT:
            return uart_hid_get_raw_report(hid, rtype, reportnum, buf, len);
        case HID_REQ_SET_REPORT:
            return uart_hid_set_raw_report(hid, rtype, reportnum, buf, len, true);
        default:
            return -EIO;
    }
}

static int uart_hid_parse(struct hid_device *hid)
{
    struct serdev_device *serdev = hid->driver_data;
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);
    struct uart_hid_desc *hdesc  = &uhid->hdesc;
    unsigned int          rsize;
    char                 *rdesc;
    int                   ret;
    int                   tries = 3;
    char                 *use_override;

    uart_hid_dbg(uhid, "%s\n", __func__);

    rsize = le16_to_cpu(hdesc->wReportDescLength);
    if (!rsize || rsize > HID_MAX_DESCRIPTOR_SIZE) {
        dbg_hid("weird size of report descriptor (%u)\n", rsize);
        return -EINVAL;
    }

    do {
        ret = uart_hid_hwreset(uhid);
        if (ret)
            msleep(1000);
    } while (tries-- > 0 && ret);

    if (ret)
        return ret;

    use_override = NULL;

    if (use_override) {
        rdesc = use_override;
        uart_hid_dbg(uhid, "using a hid report descriptor override\n");
    } else {
        rdesc = kzalloc(rsize, GFP_KERNEL);

        if (!rdesc) {
            dbg_hid("couldn't allocate rdesc memory\n");
            return -ENOMEM;
        }

        uart_hid_dbg(uhid, "fetching the hid report descriptor\n");

        ret = uart_hid_fetch_reg(uhid,
                                 uhid->hdesc.wReportDescRegister,
                                 rdesc, rsize);
        if (ret) {
            hid_err(hid, "reading report descriptor failed\n");
            kfree(rdesc);
            return -EIO;
        }
    }

    uart_hid_dbg(uhid, "report descriptor: %*ph\n", rsize, rdesc);

    ret = hid_parse_report(hid, rdesc, rsize);
    if (!use_override)
        kfree(rdesc);

    if (ret) {
        dbg_hid("parsing report descriptor failed\n");
        return ret;
    }

    return 0;
}

static int uart_hid_start(struct hid_device *hid)
{
    struct serdev_device *serdev = hid->driver_data;
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);
    int                   ret;
    unsigned int          irepsize;
    unsigned int          idatsize;
    unsigned int          odatsize;

    irepsize = HID_MIN_BUFFER_SIZE;
    uart_hid_find_max_report(hid, HID_INPUT_REPORT, &irepsize);
    if (irepsize > HID_MAX_BUFFER_SIZE)
        irepsize = HID_MAX_BUFFER_SIZE;

    idatsize = HID_MIN_BUFFER_SIZE;
    uart_hid_find_max_report(hid, HID_INPUT_REPORT, &idatsize);
    uart_hid_find_max_report(hid, HID_FEATURE_REPORT, &idatsize);
    if (idatsize > HID_MAX_BUFFER_SIZE)
        idatsize = HID_MAX_BUFFER_SIZE;

    odatsize = HID_MIN_BUFFER_SIZE;
    uart_hid_find_max_report(hid, HID_OUTPUT_REPORT, &odatsize);
    uart_hid_find_max_report(hid, HID_FEATURE_REPORT, &odatsize);
    if (odatsize > HID_MAX_BUFFER_SIZE)
        odatsize = HID_MAX_BUFFER_SIZE;

    uart_hid_free_buffers(uhid);
    ret = uart_hid_alloc_buffers(uhid, irepsize, idatsize, odatsize);
    if (ret)
        return ret;

    return 0;
}

static void uart_hid_stop(struct hid_device *hid)
{
    hid->claimed = 0;
}

static int uart_hid_open(struct hid_device *hid)
{
    struct serdev_device *serdev = hid->driver_data;
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);

    uart_hid_set_work(uhid, UART_HID_WORK_START);

    set_bit(UART_HID_WORKING, &uhid->flags);
    return 0;
}

static void uart_hid_close(struct hid_device *hid)
{
    struct serdev_device *serdev = hid->driver_data;
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);

    clear_bit(UART_HID_WORKING, &uhid->flags);

    uart_hid_set_work(uhid, UART_HID_WORK_STOP);
}

struct hid_ll_driver uart_hid_ll_driver = {
    .parse         = uart_hid_parse,
    .start         = uart_hid_start,
    .stop          = uart_hid_stop,
    .open          = uart_hid_open,
    .close         = uart_hid_close,
    .output_report = uart_hid_output_report,
    .raw_request   = uart_hid_raw_request,
};
EXPORT_SYMBOL_GPL(uart_hid_ll_driver);

static const struct serdev_device_ops uart_hid_serdev_ops = {
    .receive_buf  = uart_hid_recv,
    .write_wakeup = serdev_device_write_wakeup,
};

static int uart_hid_fetch_hid_descriptor(struct uart_hid *uhid)
{
    struct uart_hid_desc *hdesc = &uhid->hdesc;
    unsigned int          dsize;
    int                   ret;

    /* uart hid fetch using a fixed descriptor size (30 bytes) */
    uart_hid_dbg(uhid, "fetching the hid descriptor\n");
    ret = uart_hid_fetch_reg(uhid,
                             uhid->hdesc.wHIDDescRegister,
                             &uhid->hdesc.wHIDDescLength,
                             (sizeof(uhid->hdesc) - sizeof(__le16)));
    if (ret) {
        dev_err(&uhid->serdev->dev,
                "failed to fetch hid descriptor: %d\n",
                ret);
        return -ENODEV;
    }

    /* Validate the length of HID descriptor, the 4 first bytes:
     * bytes 0-1 -> length
     * bytes 2-3 -> bcdVersion (has to be 1.00) */
    /* check bcdVersion == 1.0 */
    if (le16_to_cpu(hdesc->bcdVersion) != 0x0100) {
        dev_err(&uhid->serdev->dev,
                "unexpected hid descriptor bcdVersion (0x%04hx)\n",
                le16_to_cpu(hdesc->bcdVersion));
        return -ENODEV;
    }

    /* Descriptor length should be 30 bytes as per the specification */
    dsize = le16_to_cpu(hdesc->wHIDDescLength);
    if (dsize != (sizeof(struct uart_hid_desc) - sizeof(__le16))) {
        dev_err(&uhid->serdev->dev,
                "weird size of hid descriptor (%u)\n", dsize);
        return -ENODEV;
    }
    uart_hid_dbg(uhid, "hid descriptor: %*ph\n", dsize, &uhid->hdesc);
    return 0;
}

static int uart_hid_core_power_up(struct uart_hid *uhid)
{
    if (!uhid->ops->power_up)
        return 0;

    return uhid->ops->power_up(uhid->ops);
}

static void uart_hid_core_power_down(struct uart_hid *uhid)
{
    if (!uhid->ops->power_down)
        return;

    uhid->ops->power_down(uhid->ops);
}

int uart_hid_core_probe(struct serdev_device *serdev, struct uarthid_ops *ops,
                        u16 hid_descriptor_address, u32 baudrate, u32 quirks)
{
    int                ret;
    struct uart_hid   *uhid;
    struct hid_device *hid;

    dbg_hid("hid probe called for serial %d\n", serdev->nr);

    uhid = devm_kzalloc(&serdev->dev, sizeof(*uhid), GFP_KERNEL);
    if (!uhid)
        return -ENOMEM;

    uhid->ops = ops;

    ret = uart_hid_core_power_up(uhid);
    if (ret)
        return ret;

    serdev_device_set_drvdata(serdev, uhid);

    uhid->serdev = serdev;

    uhid->hdesc.wHIDDescRegister = cpu_to_le16(hid_descriptor_address);

    init_waitqueue_head(&uhid->wait);
    mutex_init(&uhid->reset_lock);

    /* we need to allocate the command buffer without knowing the maximum
     * size of the reports. Let's use HID_MIN_BUFFER_SIZE, then we do the
     * real computation later. */
    ret = uart_hid_alloc_buffers(uhid,
                                 HID_MIN_BUFFER_SIZE,
                                 HID_MIN_BUFFER_SIZE,
                                 HID_MIN_BUFFER_SIZE);
    if (ret < 0)
        goto err_powered;

    device_enable_async_suspend(&serdev->dev);

    serdev_device_set_client_ops(serdev, &uart_hid_serdev_ops);
    ret = serdev_device_open(serdev);
    if (ret) {
        dev_err(&serdev->dev, "unable to open device\n");
        goto err_powered;
    }

    uhid->baudrate = baudrate;
    ret            = serdev_device_set_baudrate(serdev, uhid->baudrate);
    if (ret != uhid->baudrate) {
        dev_err(&serdev->dev, "failed to set baudrate %u\n", uhid->baudrate);
        ret = -EINVAL;
        goto err_serdev;
    }

    serdev_device_set_flow_control(serdev, false);

    ret = uart_hid_fetch_hid_descriptor(uhid);
    if (ret)
        goto err_serdev;

    hid = hid_allocate_device();
    if (IS_ERR(hid)) {
        ret = PTR_ERR(hid);
        goto err_serdev;
    }

    uhid->hid = hid;

    hid->driver_data = serdev;
    hid->ll_driver   = &uart_hid_ll_driver;
    hid->dev.parent  = &serdev->dev;
    hid->bus         = BUS_RS232;
    hid->version     = le16_to_cpu(uhid->hdesc.wVersionID);
    hid->vendor      = le16_to_cpu(uhid->hdesc.wVendorID);
    hid->product     = le16_to_cpu(uhid->hdesc.wProductID);

    hid->initial_quirks = quirks;

    snprintf(hid->name, sizeof(hid->name), "%s %04X:%04X",
             dev_name(&serdev->dev), (u16)hid->vendor, (u16)hid->product);
    strscpy(hid->phys, dev_name(&serdev->dev), sizeof(hid->phys));

    uhid->quirks = uart_hid_lookup_quirk(hid->vendor, hid->product);

    ret = hid_add_device(hid);
    if (ret) {
        if (ret != -ENODEV)
            hid_err(serdev, "can't add hid device: %d\n", ret);
        goto err_mem_free;
    }

    return 0;

err_mem_free:
    hid_destroy_device(hid);

err_serdev:
    serdev_device_close(serdev);

err_powered:
    uart_hid_core_power_down(uhid);
    uart_hid_free_buffers(uhid);
    return ret;
}
EXPORT_SYMBOL_GPL(uart_hid_core_probe);

void uart_hid_core_remove(struct serdev_device *serdev)
{
    struct uart_hid   *uhid = serdev_device_get_drvdata(serdev);
    struct hid_device *hid  = uhid->hid;

    hid_destroy_device(hid);
    serdev_device_close(serdev);

    uart_hid_free_buffers(uhid);

    uart_hid_core_power_down(uhid);
}
EXPORT_SYMBOL_GPL(uart_hid_core_remove);

#ifdef CONFIG_PM_SLEEP
static int uart_hid_core_suspend(struct device *dev)
{
    struct serdev_device *serdev = to_serdev_device(dev);
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);
    struct hid_device    *hid    = uhid->hid;
    int                   ret;

    ret = hid_driver_suspend(hid, PMSG_SUSPEND);
    if (ret < 0)
        return ret;

    /* Save some power */
    uart_hid_set_power(uhid, UART_HID_PWR_SLEEP);

    if (device_may_wakeup(&serdev->dev)) {
        hid_warn(hid, "Not supported remote wakeup\n");
    } else {
        /* Currently power down, unable to wake up from serdev */
        uart_hid_core_power_down(uhid);
    }

    return 0;
}

static int uart_hid_core_resume(struct device *dev)
{
    int                   ret;
    struct serdev_device *serdev = to_serdev_device(dev);
    struct uart_hid      *uhid   = serdev_device_get_drvdata(serdev);
    struct hid_device    *hid    = uhid->hid;

    if (device_may_wakeup(&serdev->dev)) {
        /* Resume device */
        ret = uart_hid_set_power(uhid, UART_HID_PWR_ON);
    } else {
        uart_hid_core_power_up(uhid);
        ret = uart_hid_hwreset(uhid);
    }

    if (ret)
        return ret;

    return hid_driver_reset_resume(hid);
}
#endif

const struct dev_pm_ops uart_hid_core_pm = {
    SET_SYSTEM_SLEEP_PM_OPS(uart_hid_core_suspend, uart_hid_core_resume)
};
EXPORT_SYMBOL_GPL(uart_hid_core_pm);

MODULE_DESCRIPTION("HID over UART core driver");
MODULE_AUTHOR("Egahp <2687434412@qq.com>");
MODULE_LICENSE("GPL");
