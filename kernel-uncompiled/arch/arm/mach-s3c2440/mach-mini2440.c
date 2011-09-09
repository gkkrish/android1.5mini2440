/* linux/arch/arm/mach-s3c2440/mach-smdk2440.c
 *
 * Copyright (c) 2004,2005 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * http://www.fluff.org/ben/smdk2440/
 *
 * Thanks to Dimity Andric and TomTom for the loan of an SMDK2440.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/plat-s3c/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>

#include <mach/idle.h>
#include <mach/fb.h>

#include <asm/plat-s3c24xx/s3c2410.h>
#include <asm/plat-s3c24xx/s3c2440.h>
#include <asm/plat-s3c24xx/clock.h>
#include <asm/plat-s3c24xx/devs.h>
#include <asm/plat-s3c24xx/cpu.h>

#include <asm/plat-s3c24xx/common-smdk.h>
#include <sound/s3c24xx_uda134x.h>

#include <mach/regs-clock.h>
#include <asm/plat-s3c24xx/udc.h>




static struct map_desc smdk2440_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, 
	{0xf0d00000, 0x55000000, SZ_1M, MT_DEVICE},  
};

#define UCON S3C2410_UCON_DEFAULT | S3C2410_UCON_UCLK
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg smdk2440_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x03,
		.ufcon	     = 0x51,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = 0x3c5,
		.ulcon	     = 0x43,
		.ufcon	     = 0x51,
	}
};

/* LCD driver info */
/*
static struct s3c2410fb_display smdk2440_lcd_cfg __initdata = {

	.lcdcon5 = S3C2410_LCDCON5_FRM565 | 
		S3C2410_LCDCON5_INVVLINE | 
		S3C2410_LCDCON5_INVVFRAME | 
		S3C2410_LCDCON5_PWREN | 
		S3C2410_LCDCON5_HWSWP, 
		.type = S3C2410_LCDCON1_TFT, 
		.width = 240, 
		.height = 320, 
		.pixclock = 174757, 
		.xres = 240, 
		.yres = 320, 
		.bpp = 16, 
		.left_margin = 6, 
		.right_margin = 37, 
		.hsync_len = 6, 
		.upper_margin = 4, 
		.lower_margin = 6, 
		.vsync_len = 2, 

};

static struct s3c2410fb_mach_info smdk2440_fb_info __initdata = {
	.displays = &smdk2440_lcd_cfg, 
	.num_displays = 1, 
	.default_display = 0, 
	.gpccon = 0xaa955699, 
	.gpccon_mask = 0xffc003cc, 
	.gpcup = 0x0000ffff, 
	.gpcup_mask = 0xffffffff, 
	.gpdcon = 0xaa95aaa1, 
	.gpdcon_mask = 0xffc0fff0, 
	.gpdup = 0x0000faff, 
	.gpdup_mask = 0xffffffff, 
	.lpcsel = 0xf82, 

};
*/

#if defined(CONFIG_FB_S3C2410_N240320)

#define LCD_WIDTH 240
#define LCD_HEIGHT 320
#define LCD_PIXCLOCK 100000

#define LCD_RIGHT_MARGIN 36
#define LCD_LEFT_MARGIN 19
#define LCD_HSYNC_LEN 5

#define LCD_UPPER_MARGIN 1
#define LCD_LOWER_MARGIN 5
#define LCD_VSYNC_LEN 1

#elif defined(CONFIG_FB_S3C2410_T240320)
#define LCD_WIDTH 320
#define LCD_HEIGHT 240
#define LCD_PIXCLOCK 170000

#define LCD_RIGHT_MARGIN 68
#define LCD_LEFT_MARGIN 4
#define LCD_HSYNC_LEN 5

#define LCD_UPPER_MARGIN 10
#define LCD_LOWER_MARGIN 4
#define LCD_VSYNC_LEN 1

#elif defined(CONFIG_FB_S3C2410_TFT800480)
#define LCD_WIDTH 800
#define LCD_HEIGHT 480
#define LCD_PIXCLOCK 40000

#define LCD_RIGHT_MARGIN 67
#define LCD_LEFT_MARGIN 40
#define LCD_HSYNC_LEN 31

#define LCD_UPPER_MARGIN 25
#define LCD_LOWER_MARGIN 5
#define LCD_VSYNC_LEN 1

#elif defined(CONFIG_FB_S3C2410_VGA1024768)
#define LCD_WIDTH 1024
#define LCD_HEIGHT 768
#define LCD_PIXCLOCK 80000

#define LCD_RIGHT_MARGIN 15
#define LCD_LEFT_MARGIN 199
#define LCD_HSYNC_LEN 15

#define LCD_UPPER_MARGIN 1
#define LCD_LOWER_MARGIN 1
#define LCD_VSYNC_LEN 1
#define LCD_CON5 (S3C2410_LCDCON5_FRM565 | S3C2410_LCDCON5_HWSWP)

#elif defined(CONFIG_FB_S3C2410_T320240)
#define LCD_WIDTH 320
#define LCD_HEIGHT 240
#define LCD_PIXCLOCK 170000

#define LCD_RIGHT_MARGIN 68
#define LCD_LEFT_MARGIN 20
#define LCD_HSYNC_LEN 31

#define LCD_UPPER_MARGIN 12
#define LCD_LOWER_MARGIN 12
#define LCD_VSYNC_LEN 2

#endif

#if defined (LCD_WIDTH)

static struct s3c2410fb_display mini2440_lcd_cfg __initdata = {

#if !defined (LCD_CON5)
	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_INVVLINE |
			  S3C2410_LCDCON5_INVVFRAME |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,
#else
	.lcdcon5	= LCD_CON5,
#endif

	.type		= S3C2410_LCDCON1_TFT,

	.width		= LCD_WIDTH,
	.height		= LCD_HEIGHT,

	.pixclock	= LCD_PIXCLOCK,
	.xres		= LCD_WIDTH,
	.yres		= LCD_HEIGHT,
	.bpp		= 16,
	.left_margin	= LCD_LEFT_MARGIN + 1,
	.right_margin	= LCD_RIGHT_MARGIN + 1,
	.hsync_len	= LCD_HSYNC_LEN + 1,
	.upper_margin	= LCD_UPPER_MARGIN + 1,
	.lower_margin	= LCD_LOWER_MARGIN + 1,
	.vsync_len	= LCD_VSYNC_LEN + 1,
};


static struct s3c2410fb_mach_info mini2440_fb_info __initdata = {
	.displays	= &mini2440_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

	.gpccon =       0xaa955699,
	.gpccon_mask =  0xffc003cc,
	.gpcup =        0x0000ffff,
	.gpcup_mask =   0xffffffff,

	.gpdcon =       0xaa95aaa1,
	.gpdcon_mask =  0xffc0fff0,
	.gpdup =        0x0000faff,
	.gpdup_mask =   0xffffffff,


	.lpcsel		= 0xf82,
};

#endif

//----------------------------------------------------------------------------------

static struct s3c24xx_uda134x_platform_data s3c24xx_uda134x_data = {
	.l3_clk = S3C2410_GPB4,
	.l3_data = S3C2410_GPB3,
	.l3_mode = S3C2410_GPB2,
	.model = UDA134X_UDA1341,
};

static struct platform_device s3c24xx_uda134x = {
	.name = "s3c24xx_uda134x",
	.dev = {
		.platform_data    = &s3c24xx_uda134x_data,
	}
};

static struct platform_device shine_audio = {
	.name = "s3c2410-iis",
	.dev = {
		.platform_data    = NULL,
	}
};

static struct platform_device *smdk2440_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_iis,
	&s3c_device_rtc,
	&s3c_device_dm9k,
	&s3c_device_sdi,
	//&s3c24xx_uda134x,
	&shine_audio,
	&s3c_device_usbgadget,
};

static void __init smdk2440_map_io(void)
{
	s3c24xx_init_io(smdk2440_iodesc, ARRAY_SIZE(smdk2440_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdk2440_uartcfgs, ARRAY_SIZE(smdk2440_uartcfgs));
}

static void __init smdk2440_machine_init(void)
{
	//s3c24xx_fb_set_platdata(&smdk2440_fb_info);
	#if defined (LCD_WIDTH)
	s3c24xx_fb_set_platdata(&mini2440_fb_info);
	#endif

	platform_add_devices(smdk2440_devices, ARRAY_SIZE(smdk2440_devices));
	smdk_machine_init();

}

MACHINE_START(S3C2440, "FriendlyARM Mini2440 development board")
	/* Maintainer: Ben Dooks <ben@fluff.org> */
	.phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= smdk2440_map_io,
	.init_machine	= smdk2440_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
