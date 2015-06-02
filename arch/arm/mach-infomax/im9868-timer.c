/* arch/arm/plat-infomax/timer.c
 * GP timer for Infomax im9815
 *
 * Copyright (C) 2010 Infomax Communication Co., LTD.
 * Author: Allen Chuang <allen.chuang@infomax.com.tw>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/profile.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#include <mach/iomap.h>
//for test
#include <asm/setup.h>

#define C7GPT_GATE_CTL_REG	(INFOMAX_GP_TIMER_BASE)
#define C7GPT_T0_CNT_REG		(INFOMAX_GP_TIMER_BASE + 0X4)
#define C7GPT_T1_CNT_REG		(INFOMAX_GP_TIMER_BASE + 0X8)
#define C7GPT_CTL_STA_REG		(INFOMAX_GP_TIMER_BASE + 0XC)


/*
 * ---------------------------------------------------------------------------
 * GP timer
 * ---------------------------------------------------------------------------
 */

/*
 *
 * GP Timer 0 ... count down to zero, interrupt, reload
 *
 */
static int timer_set_next_event(unsigned long cycles,
				struct clock_event_device *unused)
{
	//BUG_ON(!cycles);
	writel(cycles, C7GPT_T0_CNT_REG);
	return 0;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *clk)
{
	unsigned long val;

	switch (mode) {
	case CLOCK_EVT_MODE_RESUME:
		printk("CLOCK_EVT_MODE_RESUME!\n");
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		printk("CLOCK_EVT_MODE_PERIODIC\n");
		writel(9999 * 26, C7GPT_T0_CNT_REG);
	
		val = readl(C7GPT_CTL_STA_REG);
		val |= 1<<8;//gp timer0 mode periodic
		writel(val, C7GPT_CTL_STA_REG);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		printk("CLOCK_EVT_MODE_ONESHOT\n");
		val = readl(C7GPT_CTL_STA_REG);
		val &= ~(1<<8);//gp timer0 mode oneshot
		writel(val, C7GPT_CTL_STA_REG);
		break;

	case CLOCK_EVT_MODE_UNUSED:
		printk("CLOCK_EVT_MODE_UNUSED\n");
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
		printk("CLOCK_EVT_MODE_SHUTDOWN\n");
		break;
	}
}

static struct clock_event_device clkenv = {
	.name = "GP Timer 0",
	.shift = 6,//10
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = timer_set_mode,
	.set_next_event = timer_set_next_event,
};

/*
 * IRQ handler for the timer
 */
static irqreturn_t im9868_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *env = &clkenv;
	unsigned int gp_timer_status;
	//early_print("%s %d\n",__func__,__LINE__);

	gp_timer_status = readl(C7GPT_CTL_STA_REG);
	writel(gp_timer_status, C7GPT_CTL_STA_REG);
	env->event_handler(env);

	return IRQ_HANDLED;
}

static struct irqaction im9868_timer_irq = {
	.name		= "im9868 GP Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL |
                IRQF_SHARED,
	.handler	= im9868_timer_interrupt,
};

static __init void im9868_clockevents_init(void)
{
	unsigned long rate = 26000000;
	unsigned long val;

	setup_irq(INT_GPT, &im9868_timer_irq);
	
	writel(readl(C7GPT_GATE_CTL_REG)&(~1<<0), C7GPT_GATE_CTL_REG);//free run gpt0

	writel((9999 * (rate / 1000000)), C7GPT_T0_CNT_REG);//26M/26*(9999+1)=100 counts/s
	early_print("%s 0x%x",__func__, 9999 * (rate / 1000000));

	val = readl(C7GPT_CTL_STA_REG);
	val |= 1<<0;//clear gp timer0 interrupt
	val |= 1<<4;//gp timer0 interrupt enable
	val |= 1<<8;//gp timer0 mode periodic
	writel(val, C7GPT_CTL_STA_REG);


	clkenv.mult = div_sc(rate, NSEC_PER_SEC, clkenv.shift);
	clkenv.max_delta_ns =
	    clockevent_delta2ns(0xFFFFFFFE, &clkenv);
	clkenv.min_delta_ns = clockevent_delta2ns(999*26, &clkenv);
	clkenv.cpumask = cpumask_of(0);

	clockevents_register_device(&clkenv);
}

/*
 *
 * GP Timer 1 ... free running 22-bit clock source and scheduler clock
 *
 */

static cycle_t im9868_get_cycles(struct clocksource *cs)
{
	return ~readl(C7GPT_T1_CNT_REG);
}


static struct clocksource clocksource_im9868 = {
	.name	= "im9868 GP Timer 1",
	.rating	= 300,
	.read	= im9868_get_cycles,
	.mask	= CLOCKSOURCE_MASK(32),
	.shift	= 26,//20
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init im9868_clocksource_init(void)
{
	unsigned long rate = 26000000;
	unsigned long val;

	writel(readl(C7GPT_GATE_CTL_REG)&(~1<<1), C7GPT_GATE_CTL_REG);//free run gpt1

	writel(0xFFFFFFFE, C7GPT_T1_CNT_REG);
	val = readl(C7GPT_CTL_STA_REG);
	val |= 1<<1;//clear gp timer1 interrupt
	val &= ~(1<<5);//gp timer1 interrupt disable
	val |= 1<<9;//gp timer1 mode periodic
	writel(val, C7GPT_CTL_STA_REG);

	clocksource_im9868.mult =
	    clocksource_hz2mult(rate, clocksource_im9868.shift);

	if (clocksource_register(&clocksource_im9868))
		printk("%s : Error registering gp-timer clocksource!\n", __FUNCTION__);

	return 0;
}

void __init im9868_timer_init(void)
{
	printk(KERN_INFO "im9868_timer_init enter\n");
	early_print("%s\n",__func__);

	im9868_clockevents_init();
	im9868_clocksource_init();
	enable_irq(INT_GPT);
	printk(KERN_INFO "im9868_timer_init leave\n");
	
	early_print("%s\n gate:0x%x,count0:0x%x,count1:0x%x,ctl_sta:0x%x\n",__func__,readl(C7GPT_GATE_CTL_REG),readl(C7GPT_T0_CNT_REG),readl(C7GPT_T1_CNT_REG),readl(C7GPT_CTL_STA_REG));
}

