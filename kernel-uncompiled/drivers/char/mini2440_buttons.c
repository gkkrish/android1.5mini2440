#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <linux/input.h>

#define DEVICE_NAME     "buttons"

	/*
	
key 158 BACK WAKE_DROPPED
key 230 SOFT_RIGHT WAKE
key 60 SOFT_RIGHT WAKE
key 107 ENDCALL WAKE_DROPPED
key 62 ENDCALL WAKE_DROPPED
key 229 MENU WAKE_DROPPED
key 139 MENU WAKE_DROPPED
key 59 MENU WAKE_DROPPED
key 127 SEARCH WAKE_DROPPED
key 217 SEARCH WAKE_DROPPED
key 228 POUND
key 227 STAR
key 231 CALL WAKE_DROPPED
key 61 CALL WAKE_DROPPED
key 232 DPAD_CENTER WAKE_DROPPED
key 108 DPAD_DOWN WAKE_DROPPED
key 103 DPAD_UP WAKE_DROPPED
key 102 HOME WAKE
key 105 DPAD_LEFT WAKE_DROPPED
key 106 DPAD_RIGHT WAKE_DROPPED
key 115 VOLUME_UP
key 114 VOLUME_DOWN
key 116 POWER WAKE
key 212 CAMERA

5 3 4
2 1 0

	*/

static volatile int key_events[] = {75,74,73,71,72,70};

static	struct input_dev *dev;

struct button_irq_desc {
    int irq;
    int pin;
    int pin_setting;
    int number;
    char *name;	
};

#if !defined (CONFIG_QQ2440_BUTTONS)
static struct button_irq_desc button_irqs [] = {
    {IRQ_EINT8 , S3C2410_GPG0 ,  S3C2410_GPG0_EINT8  , 0, "KEY0"},
    {IRQ_EINT11, S3C2410_GPG3 ,  S3C2410_GPG3_EINT11 , 1, "KEY1"},
    {IRQ_EINT13, S3C2410_GPG5 ,  S3C2410_GPG5_EINT13 , 2, "KEY2"},
    {IRQ_EINT14, S3C2410_GPG6 ,  S3C2410_GPG6_EINT14 , 3, "KEY3"},
    {IRQ_EINT15, S3C2410_GPG7 ,  S3C2410_GPG7_EINT15 , 4, "KEY4"},
    {IRQ_EINT19, S3C2410_GPG11,  S3C2410_GPG11_EINT19, 5, "KEY5"},
};
#else /* means QQ */
static struct button_irq_desc button_irqs [] = {
    {IRQ_EINT19, S3C2410_GPG11, S3C2410_GPG11_EINT19, 0, "KEY0"},
    {IRQ_EINT11, S3C2410_GPG3,  S3C2410_GPG3_EINT11,  1, "KEY1"},
    {IRQ_EINT2,  S3C2410_GPF2,  S3C2410_GPF2_EINT2,   2, "KEY2"},
    {IRQ_EINT0,  S3C2410_GPF0,  S3C2410_GPF0_EINT0,   3, "KEY3"},
    {       -1,            -1,                 -1,    4, "KEY4"},
    {       -1,            -1,                 -1,    5, "KEY5"},
};
#endif
static volatile char key_values [] = {'0', '0', '0', '0', '0', '0'};

static DECLARE_WAIT_QUEUE_HEAD(button_waitq);

static volatile int ev_press = 0;


static irqreturn_t buttons_interrupt(int irq, void *dev_id)
{
    struct button_irq_desc *button_irqs = (struct button_irq_desc *)dev_id;
    int down;

    // udelay(0);
    down = !s3c2410_gpio_getpin(button_irqs->pin);

    if (down != (key_values[button_irqs->number] & 1)) { // Changed

	key_values[button_irqs->number] = '0' + down;
	
        ev_press = 1;
	if(down){
		//printk("btn %d : down",button_irqs->number);
		input_report_key(dev, key_events[button_irqs->number], 1);
	}
	else{
		//printk("btn %d : up",button_irqs->number);
		input_report_key(dev, key_events[button_irqs->number], 0);
	}
    }
    
    return IRQ_RETVAL(IRQ_HANDLED);
}


static int s3c24xx_buttons_open()
{
    int i;
    int err = 0;
    
    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++) {
	if (button_irqs[i].irq < 0) {
		continue;
	}
        err = request_irq(button_irqs[i].irq, buttons_interrupt, IRQ_TYPE_EDGE_BOTH, 
                          button_irqs[i].name, (void *)&button_irqs[i]);
        if (err)
            break;
    }

    if (err) {
        i--;
        for (; i >= 0; i--) {
	    if (button_irqs[i].irq < 0) {
		continue;
	    }
	    disable_irq(button_irqs[i].irq);
            free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
        }
        return -EBUSY;
    }

    ev_press = 1;
    
    return 0;
}


static int s3c24xx_buttons_close()
{
    int i;
    
    for (i = 0; i < sizeof(button_irqs)/sizeof(button_irqs[0]); i++) {
	if (button_irqs[i].irq < 0) {
	    continue;
	}
	free_irq(button_irqs[i].irq, (void *)&button_irqs[i]);
    }

    return 0;
}

static int __init dev_init(void)
{
	struct input_dev *input_dev;	
	int ret;	
	
	/* Initialise input stuff */
	input_dev = input_allocate_device();
	if (!input_dev) {
		printk(KERN_ERR "Unable to allocate the input device !!\n");
		return -ENOMEM;
	}

	dev = input_dev;
	dev->evbit[0] = BIT(EV_SYN) | BIT(EV_KEY);	

	/*dev->keybit[BITS_TO_LONGS(102)-1] = BIT_MASK(102) | BIT_MASK(107) | BIT_MASK(114) | BIT_MASK(115) | BIT_MASK(116);
	dev->keybit[BITS_TO_LONGS(158)-1] = BIT_MASK(158);
	dev->keybit[BITS_TO_LONGS(230)-1] = BIT_MASK(229) | BIT_MASK(230) | BIT_MASK(231) | BIT_MASK(232);*/

	dev->keybit[BITS_TO_LONGS(70)-1] = BIT_MASK(70) | BIT_MASK(71) | BIT_MASK(72) | BIT_MASK(73) | BIT_MASK(74) | BIT_MASK(75);
	

	dev->name = DEVICE_NAME;
	dev->phys = "button";
	dev->id.bustype = BUS_RS232;
	dev->id.vendor = 0xABCD;
	dev->id.product = 0xDCBA;
	dev->id.version = 1.0;
	
	s3c24xx_buttons_open();	
	
	/* All went ok, so register to the input system */
	ret = input_register_device(dev);

	printk (DEVICE_NAME"\tinitialized\n");

	return ret;
}

static void __exit dev_exit(void)
{
	s3c24xx_buttons_close();
	input_unregister_device(dev);
}

module_init(dev_init);
module_exit(dev_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FriendlyARM Inc.");
