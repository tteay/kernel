#include <linux/serial_8250.h>
//for test
#include <asm/setup.h>

#define SERIAL_FLAGS	(UPF_BOOT_AUTOCONF | UPF_SKIP_TEST |UPF_IOREMAP | UPF_SHARE_IRQ)
/* infomax uart's clock is different with standard 8250/16550,
 * just set .uartclk as (max_baud_rate * 16)
 */
//#define SERIAL_MAX_BAUD		4000000
//#define SERIAL_CLK		(SERIAL_MAX_BAUD * 16)
#define SERIAL_CLK		(26000000)


static struct plat_serial8250_port im9868_sio_data[] = {
	[0] = {
		.mapbase	= (unsigned long)(0xFFEA8100),
		.irq		= INT_AP_UART_CTL,
		.flags		= SERIAL_FLAGS,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= SERIAL_CLK,
	},
	[1] = {
		.mapbase	= (unsigned long)(0xFFEA8200),
		.irq		= INT_AP_UART_CTL,
		.flags		= SERIAL_FLAGS,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= SERIAL_CLK,
	},
	[2] = {
		.mapbase	= (unsigned long)(0xFFEA8300),
		.irq		= INT_AP_UART_CTL,
		.flags		= SERIAL_FLAGS,
		.iotype		= UPIO_MEM32,
		.regshift	= 2,
		.uartclk	= SERIAL_CLK,
	},
	[3] = {
		/* can't be removed, this is the end flag */
	},
};

static struct platform_device im9868_sio = {
	.name	= "serial8250",
	.id	= PLAT8250_DEV_PLATFORM,
	.dev	= {
		.platform_data	= &im9868_sio_data,
	},
};

static struct platform_device *im9868_devices[] __initdata = {
	&im9868_sio,
};

void __init im9868_machine_init(void)
{
	early_print("%s\n",__func__);
	platform_add_devices(im9868_devices, ARRAY_SIZE(im9868_devices));
	early_print("%s\n",__func__);
}

