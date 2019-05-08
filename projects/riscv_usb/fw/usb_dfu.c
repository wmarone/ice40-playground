/*
 * usb_dfu.c
 *
 * Copyright (C) 2019 Sylvain Munaut
 * All rights reserved.
 *
 * LGPL v3+, see LICENSE.lgpl3
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "spi.h"
#include "usb.h"
#include "usb_dfu.h"


#define DFU_POLL_MS		250


#define USB_RT_DFU_DETACH	((0 << 8) | 0x21)
#define USB_RT_DFU_DNLOAD	((1 << 8) | 0x21)
#define USB_RT_DFU_UPLOAD	((2 << 8) | 0xa1)
#define USB_RT_DFU_GET_STATUS	((3 << 8) | 0xa1)
#define USB_RT_DFU_CLR_STATUS	((4 << 8) | 0x21)
#define USB_RT_DFU_GETSTATE	((5 << 8) | 0xa1)
#define USB_RT_DFU_ABORT	((6 << 8) | 0x21)


enum dfu_state {
	appIDLE = 0,
	appDETACH,
	dfuIDLE,
	dfuDNLOAD_SYNC,
	dfuDNBUSY,
	dfuDNLOAD_IDLE,
	dfuMANIFEST_SYNC,
	dfuMANIFEST,
	dfuMANIFEST_WAIT_RESET,
	dfuUPLOAD_IDLE,
	dfuERROR
};

enum dfu_status {
	OK = 0,
	errTARGET,
	errFILE,
	errWRITE,
	errERASE,
	errCHECK_ERASED,
	errPROG,
	errVERIFY,
	errADDRESS,
	errNOTDONE,
	errFIRMWARE,
	errVENDOR,
	errUSBR,
	errPOR,
	errUNKNOWN,
	errSTALLEDPKT,
};

struct {
	enum dfu_state state;
	enum dfu_status status;

	uint8_t tick;
	uint8_t alt;

	uint8_t buf[4096] __attribute__((aligned(4)));

	struct {
		uint32_t addr_prog;
		uint32_t addr_erase;

		int op_ofs;
		int op_len;

		enum {
			FL_IDLE = 0,
			FL_ERASE,
			FL_PROGRAM,
		} op;
	} flash;
} g_dfu;


static void
_dfu_tick(void)
{
	/* Rate limit to once every 10 ms */
	if (g_dfu.tick++ < 10)
		return;

	g_dfu.tick = 0;

	/* Ready ? */
	if ((g_dfu.flash.op == FL_PROGRAM) && (g_dfu.state != dfuDNBUSY))
		return;

	/* Anything to do ? Is flash ready ? */
	if ((g_dfu.flash.op == FL_IDLE) || (flash_read_sr() & 1))
		return;

	/* Erase */
	if (g_dfu.flash.op == FL_ERASE) {
		/* Done ? */
		if (g_dfu.flash.addr_erase >= (g_dfu.flash.addr_prog + g_dfu.flash.op_len)) {
			/* Yes, move to programming */
			g_dfu.flash.op = FL_PROGRAM;
		} else{
			/* No, issue the next command */
			flash_write_enable();
			flash_sector_erase(g_dfu.flash.addr_erase);
			g_dfu.flash.addr_erase += 4096;
		}
	}

	/* Programming */
	if (g_dfu.flash.op == FL_PROGRAM) {
		/* Done ? */
		if (g_dfu.flash.op_ofs == g_dfu.flash.op_len) {
			/* Yes ! */
			g_dfu.flash.op = FL_IDLE;
			g_dfu.state = dfuDNLOAD_SYNC;
			g_dfu.flash.addr_prog += g_dfu.flash.op_len;
		} else {
			/* Max len */
			unsigned l = g_dfu.flash.op_len - g_dfu.flash.op_ofs;
			unsigned pl = 256 - ((g_dfu.flash.addr_prog + g_dfu.flash.op_ofs) & 0xff);
			if (l > pl)
				l = pl;

			/* Write page */
			flash_write_enable();
			flash_page_program(&g_dfu.buf[g_dfu.flash.op_ofs], g_dfu.flash.addr_prog + g_dfu.flash.op_ofs, l);

			/* Next page */
			g_dfu.flash.op_ofs += l;
		}
	}
}

static bool
_dfu_dnload_done_cb(struct usb_xfer *xfer)
{
	g_dfu.state = dfuDNBUSY;
	return true;
}

static enum usb_fnd_resp
_dfu_ctrl_req(struct usb_ctrl_req *req, struct usb_xfer *xfer)
{
	switch (req->wRequestAndType)
	{
	case USB_RT_DFU_DNLOAD:
		if (req->wLength) {
			/* Setup buffer for data */
			xfer->len     = req->wLength;
			xfer->data    = g_dfu.buf;
			xfer->cb_done = _dfu_dnload_done_cb;

			/* State update */
			g_dfu.state = dfuDNLOAD_SYNC;

			/* Prepare flash */
			g_dfu.flash.op_ofs = 0;
			g_dfu.flash.op_len = req->wLength;
			g_dfu.flash.op     = FL_ERASE;
		} else {
			/* Last xfer */
			g_dfu.state = dfuIDLE;
		}
		break;

	case USB_RT_DFU_GET_STATUS:
		/* Update state */
		if (g_dfu.state == dfuDNLOAD_SYNC) {
			g_dfu.state = dfuDNLOAD_IDLE;
		} else if (g_dfu.state == dfuMANIFEST_SYNC) {
			g_dfu.state = dfuMANIFEST;
		}

		/* Return data */
		xfer->data[0] = g_dfu.status;
		xfer->data[1] = (DFU_POLL_MS >>  0) & 0xff;
		xfer->data[2] = (DFU_POLL_MS >>  8) & 0xff;
		xfer->data[3] = (DFU_POLL_MS >> 16) & 0xff;
		xfer->data[4] = g_dfu.state;
		xfer->data[5] = 0;
		break;

	case USB_RT_DFU_CLR_STATUS:
		break;

	case USB_RT_DFU_GETSTATE:
		xfer->data[0] = g_dfu.state;
		break;

	case USB_RT_DFU_ABORT:
		break;

	default:
		return false;
	}

	return true;
}

static enum usb_fnd_resp
_dfu_set_intf(const struct usb_intf_desc *base, const struct usb_intf_desc *sel)
{
	uint32_t addr;

	if ((sel->bInterfaceClass != 0xfe) ||
	    (sel->bInterfaceSubClass != 0x01) ||
	    (sel->bInterfaceProtocol != 0x02))
		return USB_FND_CONTINUE;

	g_dfu.state = dfuIDLE;
	g_dfu.alt = sel->bAlternateSetting;

	addr = (512 + (g_dfu.alt * 128)) * 1024;
	g_dfu.flash.addr_prog  = addr;
	g_dfu.flash.addr_erase = addr;

	return USB_FND_SUCCESS;
}

static enum usb_fnd_resp
_dfu_get_intf(const struct usb_intf_desc *base, uint8_t *alt)
{
	if ((base->bInterfaceClass != 0xfe) ||
	    (base->bInterfaceSubClass != 0x01) ||
	    (base->bInterfaceProtocol != 0x02))
		return USB_FND_CONTINUE;

	*alt = g_dfu.alt;

	return USB_FND_SUCCESS;
}


static struct usb_fn_drv _dfu_drv = {
	.sof		= _dfu_tick,
	.ctrl_req	= _dfu_ctrl_req,
	.set_intf	= _dfu_set_intf,
	.get_intf	= _dfu_get_intf,
};


void
usb_dfu_init(void)
{
	memset(&g_dfu, 0x00, sizeof(g_dfu));
	usb_register_function_driver(&_dfu_drv);
}
