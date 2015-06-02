#ifndef __MACH_UNCOMPRESS_H
#define __MACH_UNCOMPRESS_H

#include <linux/serial_reg.h>
#include <mach/hardware.h>

static volatile unsigned long * const UART = (unsigned long *)0xFFEA8300;

/*
 * The following code assumes the serial port has already been
 * initialized by the bootloader.  If you didn't setup a port in
 * your bootloader then nothing will appear (which might be desired).
 */
static inline void putc(char c)
{
	while (!(UART[UART_LSR] & UART_LSR_THRE))
		barrier();
	UART[UART_TX] = c;
}

static inline void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#endif /* __MACH_UNCOMPRESS_H */
