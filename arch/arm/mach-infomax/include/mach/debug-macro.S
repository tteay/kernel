/*
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/serial_reg.h>

//#include <mach/hardware.h>
#include <mach/iomap.h>

#define UART_SHIFT  2

/*
		.macro	addruart, rp, rv, tmp
		ldreq	\rp, =0xf8009300
		ldrne	\rv, =0xfc009300
		.endm
*/
	.macro	addruart, rp, rv, tmp
	/* If we move the address using MMU, use this. */
	ldr	\rp,	  = 0xffea0000 @ MMU off, physical address
	ldr	\rv,	  = IO_ADDRESS(0xffea0000) @ MMU on, virtual address
	orr	\rp, \rp, #0x00008300
	orr	\rv, \rv, #0x00008300
	.endm

	.macro	senduart,rd,rx
	str \rd, [\rx, #UART_TX << UART_SHIFT]
	.endm
	
	.macro	busyuart,rd,rx
1002:		ldr \rd, [\rx, #UART_LSR << UART_SHIFT]
	and \rd, \rd, #UART_LSR_TEMT | UART_LSR_THRE
	teq \rd, #UART_LSR_TEMT | UART_LSR_THRE
	bne 1002b
	.endm

	/* The UART's don't have any flow control IO's wired up. */
	.macro	waituart,rd,rx
	.endm

//#include <debug/pl01x.S>
