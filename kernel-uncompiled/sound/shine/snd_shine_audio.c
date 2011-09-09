/* 
**
** Copyright (C) 2009 ShineLK, Inc.
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
*/

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/kernel.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <mach/hardware.h>

#include <linux/io.h>
#include <asm/dma.h>

#include <asm/plat-s3c24xx/regs-iis.h>

#include <mach/regs-gpio.h>
#include <mach/audio.h>
#include <mach/dma.h>

#include "bitfield.h"
#include "kuda1341.h"


#define PAUSE {msleep(3000);}
#define AUDIO_RATE_DEFAULT 44100
#define S_CLOCK_FREQ 384

#define GPIO_L3CLOCK_CFG            (S3C2410_GPB4_OUTP | S3C2410_GPB_PUPDIS(4))
#define GPIO_L3DATA_CFG             (S3C2410_GPB3_OUTP | S3C2410_GPB_PUPDIS(3))
#define GPIO_L3MODE_CFG             (S3C2410_GPB2_OUTP | S3C2410_GPB_PUPDIS(2))
#define GPIO_L3CLOCK		    S3C2410_GPB4
#define GPIO_L3DATA		    S3C2410_GPB3
#define GPIO_L3MODE		    S3C2410_GPB2

//------------test dma----------------
static char *user_data;
//static unsigned long target = 0x55000010;
static unsigned long target = 0x50000000;
//static int target = S3C24XX_PA_LCD;
static char *virt_target;
static char *dmabuf;
static dma_addr_t dmaphys;
static int dmasize = 2 * 1024;
static int channel;
char *iis_regs;
//-------------------------------------


struct shine_audio{
	char *name;
	int id;
};
static struct s3c2410_dma_client shine_audio_read_client = {
	.name = "audio_read_client"
};

static struct shine_audio *audio_data;
static atomic_t open_count = ATOMIC_INIT(0);

//对传输线L3地址的初始化设置 
static void uda1341_l3_address(u8 data) 
{ 
	int i; 
	int flags; 
 
	local_irq_save(flags); 
 
	s3c2410_gpio_setpin(GPIO_L3MODE, 0); 
	s3c2410_gpio_setpin(GPIO_L3DATA, 0); 
	s3c2410_gpio_setpin(GPIO_L3CLOCK, 1); 
	udelay(1); 
	 
	for (i = 0; i < 8; i++) { 
		if (data & 0x1) { 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 0); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3DATA, 1); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 1); 
			udelay(1); 
		} else { 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 0); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3DATA, 0); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 1); 
			udelay(1); 
		} 
		data >>= 1; 
	} 
 
	s3c2410_gpio_setpin(GPIO_L3MODE, 1); 
	udelay(1); 
	local_irq_restore(flags); 
} 
//对传输线L3数据的设置 
static void uda1341_l3_data(u8 data) 
{ 
	int i; 
	int flags; 
 
	local_irq_save(flags); 
 
	s3c2410_gpio_setpin(GPIO_L3MODE, 1); 
	udelay(1); 
 
	s3c2410_gpio_setpin(GPIO_L3MODE, 0); 
	udelay(1); 
	s3c2410_gpio_setpin(GPIO_L3MODE, 1); 
 
	for (i = 0; i < 8; i++) { 
		if (data & 0x1) { 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 0); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3DATA, 1); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 1); 
			udelay(1); 
		} else { 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 0); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3DATA, 0); 
			udelay(1); 
			s3c2410_gpio_setpin(GPIO_L3CLOCK, 1); 
			udelay(1); 
		} 
 
		data >>= 1; 
	} 
 
	s3c2410_gpio_setpin(GPIO_L3MODE, 1); 
	s3c2410_gpio_setpin(GPIO_L3MODE, 0); 
	udelay(1); 
	s3c2410_gpio_setpin(GPIO_L3MODE, 1); 
	local_irq_restore(flags); 
} 


static int
iispsr_value(int s_bit_clock,int sample_rate)
{
    int             i;
    unsigned long   fact0 = clk_get_rate(clk_get(NULL, "pclk")) / s_bit_clock;
    unsigned long   r0_sample_rate,
                    r1_sample_rate = 0,
        r2_sample_rate;
    int             prescaler = 0;

    printk("requested sample_rate = %d\n", sample_rate);

    for (i = 1; i < 32; i++)
    {
        r1_sample_rate = fact0 / i;
        if (r1_sample_rate < sample_rate)
            break;
    }

    r0_sample_rate = fact0 / (i + 1);
    r2_sample_rate = fact0 / (i - 1);

    printk("calculated (%d-1) freq = %ld, error = %d\n", i + 1, r0_sample_rate, abs(r0_sample_rate - sample_rate));
    printk("calculated (%d-1) freq = %ld, error = %d\n", i, r1_sample_rate, abs(r1_sample_rate - sample_rate));
    printk("calculated (%d-1) freq = %ld, error = %d\n", i - 1, r2_sample_rate, abs(r2_sample_rate - sample_rate));

    prescaler = i;
    if (abs(r0_sample_rate - sample_rate) < abs(r1_sample_rate - sample_rate))
        prescaler = i + 1;
    if (abs(r2_sample_rate - sample_rate) < abs(r1_sample_rate - sample_rate))
        prescaler = i - 1;

    prescaler = max_t(int, 0, (prescaler - 1));

    printk("selected prescale value = %d, freq = %ld, error = %d\n", prescaler, fact0 / (prescaler + 1), abs((fact0 / (prescaler + 1)) - sample_rate));

    return prescaler;
}


static int init_iis_bus_tx(void){
	
	u32 reg;
	
	iis_regs = ioremap(S3C2410_PA_IIS, 0x100);

	

	/*Config iis regs */
	writel(0,iis_regs + S3C2410_IISCON);
	writel(0,iis_regs + S3C2410_IISMOD);
	writel(0,iis_regs + S3C2410_IISFCON);
	writel(0,iis_regs + S3C2410_IISPSR);

	writel((IISPSR_A(iispsr_value(S_CLOCK_FREQ,AUDIO_RATE_DEFAULT)) 
		| IISPSR_B(iispsr_value(S_CLOCK_FREQ,AUDIO_RATE_DEFAULT))),iis_regs + S3C2410_IISPSR);
	
	writel((S3C2410_IISCON_TXIDLE 
		| S3C2410_IISCON_PSCEN)
		,iis_regs + S3C2410_IISCON);
	
	writel((S3C2410_IISMOD_MASTER         /* Master mode */
    		| S3C2410_IISMOD_TXMODE         /* Transmit */
  		| S3C2410_IISMOD_LR_LLOW       /* Low for left channel */
  		| S3C2410_IISMOD_MSB       /* MSB-justified format */
  		| S3C2410_IISMOD_16BIT         /* Serial data bit/channel is 16 bit */
  		| S3C2410_IISMOD_384FS      /* Master clock freq = 384 fs */
  		| S3C2410_IISMOD_32FS)    /* 32 fs */
		,iis_regs + S3C2410_IISMOD);
	
	writel((S3C2410_IISFCON_TXNOR      /* Transmit FIFO access mode: DMA */
  		| S3C2410_IISFCON_TXENABLE)      /* Transmit FIFO enable */
		,iis_regs + S3C2410_IISFCON);
	
	reg = readl(iis_regs + S3C2410_IISCON);
	printk("iiscon=%08x\n",reg);
	reg |= S3C2410_IISCON_IISEN;
	writel(reg,iis_regs + S3C2410_IISCON);
	reg = readl(iis_regs + S3C2410_IISCON);
	printk("iiscon=%08x\n",reg);

	printk("iis init finish!\n");
	return 0;
}

static void init_uda1341(void){
	u32 uda1341_volume,uda1341_boost,uda_sampling,flags;

	uda1341_volume = 62 - ((DEF_VOLUME * 61) / 100);
 	uda1341_boost = 0;
   	uda_sampling = DATA2_DEEMP_NONE;
 	uda_sampling &= ~(DATA2_MUTE); 

	local_irq_save(flags); 
 	s3c2410_gpio_setpin(GPIO_L3MODE, 1);
 	s3c2410_gpio_setpin(GPIO_L3CLOCK, 1); 
 	local_irq_restore(flags); 

	uda1341_l3_address(UDA1341_REG_STATUS);
        uda1341_l3_data(STAT0_SC_384FS | STAT0_IF_MSB);     // set 384 system clock, MSB
        uda1341_l3_data(STAT1 | STAT1_DAC_GAIN | STAT1_ADC_GAIN | STAT1_ADC_ON | STAT1_DAC_ON); 

	uda1341_l3_address(UDA1341_REG_DATA0);
 	uda1341_l3_data(DATA0 |DATA0_VOLUME(uda1341_volume));  // maximum volume
 	uda1341_l3_data(DATA1 |DATA1_BASS(uda1341_boost)| DATA1_TREBLE(0));
        uda1341_l3_data(DATA2 | uda_sampling);
 	uda1341_l3_data(EXTADDR(EXT2));
 	uda1341_l3_data(EXTDATA(EXT2_MIC_GAIN(0x6)) | EXT2_MIXMODE_CH1); 

	printk("uda1341 init finish!\n");
}



static ssize_t shine_audio_read(struct file *fp, char __user *buf,
							size_t count, loff_t *pos)
{
	u32 reg;
	int i;
	
	printk("shine audio read!\n");
	printk("data->name=%s,data->id=%d\n",audio_data->name,audio_data->id);

		reg = readl(iis_regs + S3C2410_IISCON);
		printk("iiscon=%08x\n",reg);
		reg = readl(iis_regs + S3C2410_IISFCON);
		printk("iiscon=%08x\n",reg);
		//writel(0x00000000,iis_regs + S3C2410_IISFIFO);
		writel(0xffffffff,iis_regs + S3C2410_IISFIFO + 32 * i);
		msleep(1000);	
		reg = readl(iis_regs + S3C2410_IISCON);
		printk("iiscon=%08x\n",reg);
		reg = readl(iis_regs + S3C2410_IISFCON);
		printk("iiscon=%08x\n",reg);

	PAUSE
	return 0;
}

static ssize_t shine_audio_write(struct file *fp, const char __user *buf,
							 size_t count, loff_t *pos)
{
	printk("shine audio write!\n");
	PAUSE
	return 0;
}

static int shine_audio_open(struct inode *ip, struct file *fp)
{
	printk("shine audio open!\n");
	//---------------------------test iis-------------------------------
	init_iis_bus_tx();
	init_uda1341();
	//------------------------------------------------------------------
	PAUSE
	return 0;
}

static int shine_audio_release(struct inode *ip, struct file* fp)
{
	printk("shine audio release!\n");
	PAUSE
	return 0;
}
	   
static int shine_audio_ioctl(struct inode* ip, struct file* fp, unsigned int cmd, unsigned long arg)
{
	printk("shine audio ioctl!\n");
	PAUSE
	return 0;
}

static irqreturn_t shine_audio_interrupt(int irq, void *dev_id)
{
	printk("shine audio interrupt!\n");
	PAUSE
	return IRQ_HANDLED;
}

/* file operations for /dev/eac */
static struct file_operations shine_audio_fops = {
	.owner = THIS_MODULE,
	.read = shine_audio_read,
	.write = shine_audio_write,
	.open = shine_audio_open,
	.release = shine_audio_release,
   	.ioctl = shine_audio_ioctl,

};

static struct miscdevice shine_audio_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "eac",
	.fops = &shine_audio_fops,
};


static int shine_audio_probe(struct platform_device *pdev){
	int ret;
	struct shine_audio *data;
	
	printk("shine audio probe!\n");
	data = kzalloc(sizeof(data),GFP_KERNEL);
	if(data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	data->name = "shinelk";
	data->id = 1987;

	//---------------------------test dma--------------------------
	user_data = kmalloc(1024 * 50,GFP_KERNEL);
	memset(user_data,17,1024 * 50);
	//-------------------------------------------------------------
	
	if((ret = misc_register(&shine_audio_device))) 
	{
		printk("misc_register returned %d in shine_audio_init\n", ret);
		goto err_misc_register_failed;
	}

	/* Configure the I2S pins in correct mode */
	s3c2410_gpio_cfgpin(S3C2410_GPE0, S3C2410_GPE0_I2SLRCK);
	s3c2410_gpio_cfgpin(S3C2410_GPE1, S3C2410_GPE1_I2SSCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE2, S3C2410_GPE2_CDCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE3, S3C2410_GPE3_I2SSDI);
	s3c2410_gpio_cfgpin(S3C2410_GPE4, S3C2410_GPE4_I2SSDO);

	/* Config L3 pins */
	s3c2410_gpio_cfgpin(GPIO_L3CLOCK,GPIO_L3CLOCK_CFG);
	s3c2410_gpio_cfgpin(GPIO_L3DATA,GPIO_L3DATA_CFG);
	s3c2410_gpio_cfgpin(GPIO_L3MODE,GPIO_L3MODE_CFG);
	
	audio_data = data;	
	

err_data_alloc_failed:
err_misc_register_failed:
failed:
	//PAUSE
	return ret;
}

static int shine_audio_remove(struct platform_device *pdev){
	printk("shine audio remove!\n");
	PAUSE
	return 0;
}

static struct platform_driver shine_audio_driver = {
	.probe = shine_audio_probe,
	.remove = shine_audio_remove,
	.driver = {
		.name = "shine_audio"
	}
};

static int __init shine_audio_init(void){
	int ret;
	ret = platform_driver_register(&shine_audio_driver);
	if (ret < 0)
	{
		printk("platform_driver_register returned %d\n", ret);
		return ret;
	}
	
	printk("shine audio driver init!\n");
	//PAUSE
	return ret;
}

static void __exit shine_audio_exit(void){
	printk("shine audio driver exit!\n");
	platform_driver_unregister(&shine_audio_driver);
	PAUSE
}

module_init(shine_audio_init);
module_exit(shine_audio_exit);

MODULE_AUTHOR("ShineLK, Inc.");
MODULE_DESCRIPTION("ShineLK Audio Driver for android");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
