#include <asm/system.h>

#include <linux/irqchip/arm-gic.h>
#include <mach/iomap.h>
#include <asm/io.h>
//for test
#include <asm/setup.h>
#include <asm/sizes.h>

void __init im9868_init_irq(void)
{
	early_print("%s\n",__func__);

	gic_init(0, 32, ioremap(0xFFEF1000, SZ_4K),
		 ioremap(0xFFEF2000, SZ_4K));
	//gic_init(0, 32, IOMEM(INFOMAX_CA7_GIC_BASE + 0X1000),
	//	 IOMEM(INFOMAX_CA7_GIC_BASE + 0X2000));
	early_print("%s\n",__func__);
}

