/* linux/arch/arm/plat-s3c24xx/pwm-clock.c
 *
 * Copyright (c) 2007 Simtec Electronics
 * Copyright (c) 2007, 2008 Ben Dooks
 *	Ben Dooks <ben-linux@fluff.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>

#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <asm/plat-s3c24xx/clock.h>
#include <asm/plat-s3c24xx/cpu.h>

#include <asm/plat-s3c/regs-timer.h>

/* Each of the timers 0 through 5 go through the following
 * clock tree, with the inputs depending on the timers.
 *
 * pclk ---- [ prescaler 0 ] -+---> timer 0
 *			      +---> timer 1
 *
 * pclk ---- [ prescaler 1 ] -+---> timer 2
 *			      +---> timer 3
 *			      \---> timer 4
 *
 * Which are fed into the timers as so:
 *
 * prescaled 0 ---- [ div 2,4,8,16 ] ---\
 *				       [mux] -> timer 0
 * tclk 0 ------------------------------/
 *
 * prescaled 0 ---- [ div 2,4,8,16 ] ---\
 *				       [mux] -> timer 1
 * tclk 0 ------------------------------/
 *
 *
 * prescaled 1 ---- [ div 2,4,8,16 ] ---\
 *				       [mux] -> timer 2
 * tclk 1 ------------------------------/
 *
 * prescaled 1 ---- [ div 2,4,8,16 ] ---\
 *				       [mux] -> timer 3
 * tclk 1 ------------------------------/
 *
 * prescaled 1 ---- [ div 2,4,8, 16 ] --\
 *				       [mux] -> timer 4
 * tclk 1 ------------------------------/
 *
 * Since the mux and the divider are tied together in the
 * same register space, it is impossible to set the parent
 * and the rate at the same time. To avoid this, we add an
 * intermediate 'prescaled-and-divided' clock to select
 * as the parent for the timer input clock called tdiv.
 *
 * prescaled clk --> pwm-tdiv ---\
 *                             [ mux ] --> timer X
 * tclk -------------------------/
*/

static unsigned long clk_pwm_scaler_getrate(struct clk *clk)
{
	unsigned long tcfg0 = __raw_readl(S3C2410_TCFG0);

	if (clk->id == 1) {
		tcfg0 &= S3C2410_TCFG_PRESCALER1_MASK;
		tcfg0 >>= S3C2410_TCFG_PRESCALER1_SHIFT;
	} else {
		tcfg0 &= S3C2410_TCFG_PRESCALER0_MASK;
	}

	return clk_get_rate(clk->parent) / (tcfg0 + 1);
}

/* TODO - add set rate calls. */

static struct clk clk_timer_scaler[] = {
	[0]	= {
		.name		= "pwm-scaler0",
		.id		= -1,
		.get_rate	= clk_pwm_scaler_getrate,
	},
	[1]	= {
		.name		= "pwm-scaler1",
		.id		= -1,
		.get_rate	= clk_pwm_scaler_getrate,
	},
};

static struct clk clk_timer_tclk[] = {
	[0]	= {
		.name		= "pwm-tclk0",
		.id		= -1,
	},
	[1]	= {
		.name		= "pwm-tclk1",
		.id		= -1,
	},
};

struct pwm_tdiv_clk {
	struct clk	clk;
	unsigned int	divisor;
};

static inline struct pwm_tdiv_clk *to_tdiv(struct clk *clk)
{
	return container_of(clk, struct pwm_tdiv_clk, clk);
}

static inline unsigned long tcfg_to_divisor(unsigned long tcfg1)
{
	return 1 << (1 + tcfg1);
}

static unsigned long clk_pwm_tdiv_get_rate(struct clk *clk)
{
	unsigned long tcfg1 = __raw_readl(S3C2410_TCFG1);
	unsigned int divisor;

	tcfg1 >>= S3C2410_TCFG1_SHIFT(clk->id);
	tcfg1 &= S3C2410_TCFG1_MUX_MASK;

	if (tcfg1 == S3C2410_TCFG1_MUX_TCLK)
		divisor = to_tdiv(clk)->divisor;
	else
		divisor = tcfg_to_divisor(tcfg1);

	return clk_get_rate(clk->parent) / divisor;
}

static unsigned long clk_pwm_tdiv_round_rate(struct clk *clk,
					     unsigned long rate)
{
	unsigned long parent_rate;
	unsigned long divisor;

	parent_rate = clk_get_rate(clk->parent);
	divisor = parent_rate / rate;

	if (divisor <= 2)
		divisor = 2;
	else if (divisor <= 4)
		divisor = 4;
	else if (divisor <= 8)
		divisor = 8;
	else
		divisor = 16;

	return parent_rate / divisor;
}

static unsigned long clk_pwm_tdiv_bits(struct pwm_tdiv_clk *divclk)
{
	unsigned long bits;

	switch (divclk->divisor) {
	case 2:
		bits = S3C2410_TCFG1_MUX_DIV2;
		break;
	case 4:
		bits = S3C2410_TCFG1_MUX_DIV4;
		break;
	case 8:
		bits = S3C2410_TCFG1_MUX_DIV8;
		break;
	case 16:
	default:
		bits = S3C2410_TCFG1_MUX_DIV16;
		break;
	}

	return bits;
}

static void clk_pwm_tdiv_update(struct pwm_tdiv_clk *divclk)
{
	unsigned long tcfg1 = __raw_readl(S3C2410_TCFG1);
	unsigned long bits = clk_pwm_tdiv_bits(divclk);
	unsigned long flags;
	unsigned long shift =  S3C2410_TCFG1_SHIFT(divclk->clk.id);

	local_irq_save(flags);

	tcfg1 = __raw_readl(S3C2410_TCFG1);
	tcfg1 &= ~(S3C2410_TCFG1_MUX_MASK << shift);
	tcfg1 |= bits << shift;
	__raw_writel(tcfg1, S3C2410_TCFG1);

	local_irq_restore(flags);
}

static int clk_pwm_tdiv_set_rate(struct clk *clk, unsigned long rate)
{
	struct pwm_tdiv_clk *divclk = to_tdiv(clk);
	unsigned long tcfg1 = __raw_readl(S3C2410_TCFG1);
	unsigned long parent_rate = clk_get_rate(clk->parent);
	unsigned long divisor;

	tcfg1 >>= S3C2410_TCFG1_SHIFT(clk->id);
	tcfg1 &= S3C2410_TCFG1_MUX_MASK;

	rate = clk_round_rate(clk, rate);
	divisor = parent_rate / rate;

	if (divisor > 16)
		return -EINVAL;

	divclk->divisor = divisor;

	/* Update the current MUX settings if we are currently
	 * selected as the clock source for this clock. */

	if (tcfg1 != S3C2410_TCFG1_MUX_TCLK)
		clk_pwm_tdiv_update(divclk);

	return 0;
}

static struct pwm_tdiv_clk clk_timer_tdiv[] = {
	[0]	= {
		.clk	= {
			.name		= "pwm-tdiv",
			.parent		= &clk_timer_scaler[0],
			.get_rate	= clk_pwm_tdiv_get_rate,
			.set_rate	= clk_pwm_tdiv_set_rate,
			.round_rate	= clk_pwm_tdiv_round_rate,
		},
	},
	[1]	= {
		.clk	= {
			.name		= "pwm-tdiv",
			.parent		= &clk_timer_scaler[0],
			.get_rate	= clk_pwm_tdiv_get_rate,
			.set_rate	= clk_pwm_tdiv_set_rate,
			.round_rate	= clk_pwm_tdiv_round_rate,
		}
	},
	[2]	= {
		.clk	= {
			.name		= "pwm-tdiv",
			.parent		= &clk_timer_scaler[1],
			.get_rate	= clk_pwm_tdiv_get_rate,
			.set_rate	= clk_pwm_tdiv_set_rate,
			.round_rate	= clk_pwm_tdiv_round_rate,
		},
	},
	[3]	= {
		.clk	= {
			.name		= "pwm-tdiv",
			.parent		= &clk_timer_scaler[1],
			.get_rate	= clk_pwm_tdiv_get_rate,
			.set_rate	= clk_pwm_tdiv_set_rate,
			.round_rate	= clk_pwm_tdiv_round_rate,
		},
	},
	[4]	= {
		.clk	= {
			.name		= "pwm-tdiv",
			.parent		= &clk_timer_scaler[1],
			.get_rate	= clk_pwm_tdiv_get_rate,
			.set_rate	= clk_pwm_tdiv_set_rate,
			.round_rate	= clk_pwm_tdiv_round_rate,
		},
	},
};

static int __init clk_pwm_tdiv_register(unsigned int id)
{
	struct pwm_tdiv_clk *divclk = &clk_timer_tdiv[id];
	unsigned long tcfg1 = __raw_readl(S3C2410_TCFG1);

	tcfg1 >>= S3C2410_TCFG1_SHIFT(id);
	tcfg1 &= S3C2410_TCFG1_MUX_MASK;

	divclk->clk.id = id;
	divclk->divisor = tcfg_to_divisor(tcfg1);

	return s3c24xx_register_clock(&divclk->clk);
}

static inline struct clk *s3c24xx_pwmclk_tclk(unsigned int id)
{
	return (id >= 2) ? &clk_timer_tclk[1] : &clk_timer_tclk[0];
}

static inline struct clk *s3c24xx_pwmclk_tdiv(unsigned int id)
{
	return &clk_timer_tdiv[id].clk;
}

static int clk_pwm_tin_set_parent(struct clk *clk, struct clk *parent)
{
	unsigned int id = clk->id;
	unsigned long tcfg1;
	unsigned long flags;
	unsigned long bits;
	unsigned long shift = S3C2410_TCFG1_SHIFT(id);

	if (parent == s3c24xx_pwmclk_tclk(id))
		bits = S3C2410_TCFG1_MUX_TCLK << shift;
	else if (parent == s3c24xx_pwmclk_tdiv(id))
		/*bits = clk_pwm_tdiv_bits(to_tdiv(clk)) << shift;*/
		bits = clk_pwm_tdiv_bits(to_tdiv(parent)) << shift; 
	else
		return -EINVAL;

	clk->parent = parent;

	local_irq_save(flags);

	tcfg1 = __raw_readl(S3C2410_TCFG1);
	tcfg1 &= ~(S3C2410_TCFG1_MUX_MASK << shift);
	__raw_writel(tcfg1 | bits, S3C2410_TCFG1);

	local_irq_restore(flags);

	return 0;
}

static struct clk clk_tin[] = {
	[0]	= {
		.name		= "pwm-tin",
		.id		= 0,
		.set_parent	= clk_pwm_tin_set_parent,
	},
	[1]	= {
		.name		= "pwm-tin",
		.id		= 1,
		.set_parent	= clk_pwm_tin_set_parent,
	},
	[2]	= {
		.name		= "pwm-tin",
		.id		= 2,
		.set_parent	= clk_pwm_tin_set_parent,
	},
	[3]	= {
		.name		= "pwm-tin",
		.id		= 3,
		.set_parent	= clk_pwm_tin_set_parent,
	},
	[4]	= {
		.name		= "pwm-tin",
		.id		= 4,
		.set_parent	= clk_pwm_tin_set_parent,
	},
};

static __init int clk_pwm_tin_register(struct clk *pwm)
{
	unsigned long tcfg1 = __raw_readl(S3C2410_TCFG1);
	unsigned int id = pwm->id;

	struct clk *parent;
	int ret;

	ret = s3c24xx_register_clock(pwm);
	if (ret < 0)
		return ret;

	tcfg1 >>= S3C2410_TCFG1_SHIFT(id);
	tcfg1 &= S3C2410_TCFG1_MUX_MASK;

	if (tcfg1 == S3C2410_TCFG1_MUX_TCLK)
		parent = s3c24xx_pwmclk_tclk(id);
	else
		parent = s3c24xx_pwmclk_tdiv(id);

	return clk_set_parent(pwm, parent);
}

static __init int s3c24xx_pwmclk_init(void)
{
	struct clk *clk_timers;
	unsigned int clk;
	int ret;

	clk_timers = clk_get(NULL, "timers");
	if (IS_ERR(clk_timers)) {
		printk(KERN_ERR "%s: no parent clock\n", __func__);
		return -EINVAL;
	}

	for (clk = 0; clk < ARRAY_SIZE(clk_timer_scaler); clk++) {
		clk_timer_scaler[clk].parent = clk_timers;
		ret = s3c24xx_register_clock(&clk_timer_scaler[clk]);
		if (ret < 0) {
			printk(KERN_ERR "error adding pwm scaler%d clock\n", clk);
			goto err;
		}
	}

	for (clk = 0; clk < ARRAY_SIZE(clk_timer_tclk); clk++) {
		ret = s3c24xx_register_clock(&clk_timer_tclk[clk]);
		if (ret < 0) {
			printk(KERN_ERR "error adding pww tclk%d\n", clk);
			goto err;
		}
	}

	for (clk = 0; clk < ARRAY_SIZE(clk_timer_tdiv); clk++) {
		ret = clk_pwm_tdiv_register(clk);
		if (ret < 0) {
			printk(KERN_ERR "error adding pwm%d tdiv clock\n", clk);
			goto err;
		}
	}

	for (clk = 0; clk < ARRAY_SIZE(clk_tin); clk++) {
		ret = clk_pwm_tin_register(&clk_tin[clk]);
		if (ret < 0) {
			printk(KERN_ERR "error adding pwm%d tin clock\n", clk);
			goto err;
		}
	}

	return 0;

 err:
	return ret;
}

arch_initcall(s3c24xx_pwmclk_init);
