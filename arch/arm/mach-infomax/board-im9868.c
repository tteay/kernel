#include <linux/kernel.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/mach/irq.h>

#include "common.h"

MACHINE_START(IM9868_EVB_V1, "Infomax iM9868 Platform")
	//.atag_offset	= 0x100,
	.map_io = im9868_map_io,
	//.reserve = infomax_reserve,
	.init_irq = im9868_init_irq,
	//.handle_irq = gic_handle_irq,
	.init_machine = im9868_machine_init,
	.init_time = im9868_timer_init,
	//.init_early =im9868_init_early,
	//.restart = im9868_restart,
MACHINE_END
