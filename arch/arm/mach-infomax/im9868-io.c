#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/page.h>

#include <asm/mach/map.h>
#include <mach/iomap.h>
#include <asm/system.h>
//for test
#include <asm/setup.h>

#define INFOMAX_CHIP_DEVICE(name, chip) { \
		.virtual = (unsigned long) INFOMAX_##name##_BASE, \
		.pfn = __phys_to_pfn(chip##_##name##_PHYS), \
		.length = chip##_##name##_SIZE, \
		.type = MT_DEVICE, \
	 }

#define IM9868_DEVICE(name) INFOMAX_CHIP_DEVICE(name, IM9868)

static void __init im9868_map_common_io(struct map_desc *io_desc, int size)
{
	BUG_ON(!size);
	iotable_init(io_desc, size);
}

static struct map_desc __initdata im9868_io_desc[]  = {
	INFOMAX_CHIP_DEVICE(GP_TIMER, IM9868),
	IM9868_DEVICE(CA7_GIC),
};
void __init im9868_map_io(void)
{
	early_print("%s\n",__func__);
	debug_ll_io_init();
	early_print("%s\n",__func__);
	im9868_map_common_io(im9868_io_desc, ARRAY_SIZE(im9868_io_desc));
	early_print("%s\n",__func__);
}
