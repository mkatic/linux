/*
 *  Touchscreen driver for Sharp SL-C7xx and SL-Cxx00 models
 *
 *  Copyright (c) 2004-2005 Richard Purdie
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/sort.h>
#include <linux/gpio.h>
#include <linux/pxa2xx_ssp.h>
#include <linux/spinlock.h>

#define PWR_MODE_ACTIVE		0
#define PWR_MODE_SUSPEND	1

#define X_AXIS_MAX		3830
#define X_AXIS_MIN		150
#define Y_AXIS_MAX		3830
#define Y_AXIS_MIN		190
#define PRESSURE_MIN		0
#define PRESSURE_MAX		15000
#define TIMEOUT 100000

static DEFINE_SPINLOCK(corgi_ssp_lock);

struct ts_event {
	short pressure;
	short x;
	short y;
};

struct corgi_ts {
	struct input_dev *input;
	struct timer_list timer;
	struct ts_event tc;
	int pendown;
	int power_mode;
	int irq_gpio;
	struct corgits_machinfo *machinfo;
};

struct ssp_device *ssp;

#ifdef CONFIG_PXA25x
#define CCNT(a)		asm volatile ("mrc p14, 0, %0, C1, C0, 0" : "=r"(a))
#define PMNC_GET(x)	asm volatile ("mrc p14, 0, %0, C0, C0, 0" : "=r"(x))
#define PMNC_SET(x)	asm volatile ("mcr p14, 0, %0, C0, C0, 0" : : "r"(x))
#endif
#ifdef CONFIG_PXA27x
#define CCNT(a)		asm volatile ("mrc p14, 0, %0, C1, C1, 0" : "=r"(a))
#define PMNC_GET(x)	asm volatile ("mrc p14, 0, %0, C0, C1, 0" : "=r"(x))
#define PMNC_SET(x)	asm volatile ("mcr p14, 0, %0, C0, C1, 0" : : "r"(x))
#endif

/* ADS7846 Touch Screen Controller bit definitions */
#define ADSCTRL_PD0		(1u << 0)	/* PD0 */
#define ADSCTRL_PD1		(1u << 1)	/* PD1 */
#define ADSCTRL_DFR		(1u << 2)	/* SER/DFR */
#define ADSCTRL_MOD		(1u << 3)	/* Mode */
#define ADSCTRL_ADR_SH	4	/* Address setting */
#define ADSCTRL_STS		(1u << 7)	/* Start Bit */

int ssp_read_word(u32 *data);
int ssp_write_word(u32 data);
unsigned long ts_ssp_putget(ulong data);

/* External Functions */
extern unsigned int get_clk_frequency_khz(int info);

//~ static void ts_interrupt_main(struct corgi_ts *corgi_ts, int isTimer)
//~ {
	//~ /* GPIO hardcoded for now... */
	//~ if ((GPLR(14) & GPIO_bit(14)) == 0) {
		//~ /* Disable Interrupt */
		//~ disable_irq(corgi_ts->irq_gpio);
		//~ if (read_xydata(corgi_ts)) {
			//~ corgi_ts->pendown = 1;
			//~ new_data(corgi_ts);
		//~ }
		//~ mod_timer(&corgi_ts->timer, jiffies + HZ / 100);
	//~ } 
	//~ 
	//~ else 
	//~ {
		//~ if (corgi_ts->pendown == 1 || corgi_ts->pendown == 2) {
			//~ mod_timer(&corgi_ts->timer, jiffies + HZ / 100);
			//~ corgi_ts->pendown++;
			//~ return;
		//~ }
//~ 
		//~ if (corgi_ts->pendown) {
			//~ corgi_ts->tc.pressure = 0;
			//~ new_data(corgi_ts);
		//~ }
//~ 
		//~ /* Enable Falling Edge */
		//~ enable_irq(corgi_ts->irq_gpio);
		//~ corgi_ts->pendown = 0;
	//~ }
//~ }



static void corgi_ts_timer(unsigned long data)
{
	struct corgi_ts *corgits_data = (struct corgi_ts *) data;

	//~ ts_interrupt_main(corgits_data, 1);
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

#ifdef CONFIG_PM
static int corgits_suspend(struct platform_device *dev, pm_message_t state)
{
	struct corgi_ts *corgi_ts = platform_get_drvdata(dev);

	if (corgi_ts->pendown) {
		del_timer_sync(&corgi_ts->timer);
		corgi_ts->tc.pressure = 0;
		//~ new_data(corgi_ts);
		corgi_ts->pendown = 0;
	}
	corgi_ts->power_mode = PWR_MODE_SUSPEND;

	ts_ssp_putget((1u << ADSCTRL_ADR_SH) | ADSCTRL_STS);

	return 0;
}

static int corgits_resume(struct platform_device *dev)
{
	struct corgi_ts *corgi_ts = platform_get_drvdata(dev);

	ts_ssp_putget((4u << ADSCTRL_ADR_SH) | ADSCTRL_STS);
	corgi_ts->power_mode = PWR_MODE_ACTIVE;

	return 0;
}
#else
#define corgits_suspend		NULL
#define corgits_resume		NULL
#endif

static int __init corgits_probe(struct platform_device *pdev)
{
	struct corgi_ts *corgi_ts;
	struct input_dev *input_dev;
	int err = -ENOMEM;
	int ret;
	
	corgi_ts = kzalloc(sizeof(struct corgi_ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!corgi_ts || !input_dev)
		goto fail1;

	platform_set_drvdata(pdev, corgi_ts);

/*	corgi_ts->machinfo = pdev->dev.platform_data; */
	corgi_ts->irq_gpio = platform_get_irq(pdev, 0);

	if (corgi_ts->irq_gpio < 0) {
		err = -ENODEV;
		goto fail1;
	}
	
	ssp = pxa_ssp_request (1, "corgi-ts");
	
	if (ssp == NULL)
		goto fail1;
	
	corgi_ts->input = input_dev;

	init_timer(&corgi_ts->timer);
	corgi_ts->timer.data = (unsigned long) corgi_ts;
	corgi_ts->timer.function = corgi_ts_timer; 

	input_dev->name = "Corgi Touchscreen";
	input_dev->phys = "corgits/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, PRESSURE_MIN, PRESSURE_MAX, 0, 0);

	/* gpio value hardcoded for now (11), since the macro for irq->gpio conversion
	 * is gone in the current kernel */
	 
	ret = gpio_request(11, "TS_INT");
	
	if (ret != 0) {
		printk(KERN_ALERT "corgi_ts:gpio_request failed.\n");
		goto fail1;
	}
	gpio_direction_input(11);

	/* Initiaize ADS7846 Difference Reference mode */
	ts_ssp_putget((1u << ADSCTRL_ADR_SH) | ADSCTRL_STS);
	mdelay(5);
	ts_ssp_putget((3u << ADSCTRL_ADR_SH) | ADSCTRL_STS);
	mdelay(5);
	ts_ssp_putget((4u << ADSCTRL_ADR_SH) | ADSCTRL_STS);
	mdelay(5);
	ts_ssp_putget((5u << ADSCTRL_ADR_SH) | ADSCTRL_STS);
	mdelay(5);

	if (request_irq(corgi_ts->irq_gpio, ts_interrupt, IRQF_DISABLED | IRQF_TRIGGER_FALLING, "Touchscreen", corgi_ts)) {
		printk(KERN_ALERT "corgi_ts: request_irq failed.\n");
		err = -EBUSY;
		goto fail1;
	}

	err = input_register_device(corgi_ts->input);
	if (err)
		goto fail2;

	corgi_ts->power_mode = PWR_MODE_ACTIVE;

	return 0;

 fail2:	free_irq(corgi_ts->irq_gpio, corgi_ts);
 fail1:	input_free_device(input_dev);
	kfree(corgi_ts);
	return err;
}

static int corgits_remove(struct platform_device *pdev)
{

	struct corgi_ts *corgi_ts = platform_get_drvdata(pdev);

	free_irq(corgi_ts->irq_gpio, corgi_ts);
	del_timer_sync(&corgi_ts->timer);
/*	corgi_ts->machinfo->put_hsync(); */
	input_unregister_device(corgi_ts->input);
	kfree(corgi_ts);
	return 0;

}

static struct platform_driver corgits_driver = {
	.probe		= corgits_probe,
	.remove		= corgits_remove,
	.suspend	= corgits_suspend,
	.resume		= corgits_resume,
	.driver		= {
		.name	= "corgi-ts",
	},
};

static int __devinit corgits_init(void)
{
	return platform_driver_register(&corgits_driver);
}

static void __exit corgits_exit(void)
{
	platform_driver_unregister(&corgits_driver);
}

unsigned long ts_ssp_putget(ulong data)
{
	unsigned long flag;
	u32 ret = 0;
/* GPIO hardcoded for now */
	spin_lock_irqsave(&corgi_ssp_lock, flag);
	GPCR(14) = GPIO_bit(14);

	ssp_write_word(data);
 	ssp_read_word(&ret);

	GPSR(14) = GPIO_bit(14);
	spin_unlock_irqrestore(&corgi_ssp_lock, flag);

	return ret;
}

int ssp_write_word(u32 data)
{
	int timeout = TIMEOUT;

	while (!(__raw_readl(ssp->mmio_base + SSSR) & SSSR_TNF)) {
	        if (!--timeout)
	        	return -ETIMEDOUT;
		cpu_relax();
	}

	__raw_writel(data, ssp->mmio_base + SSDR);

	return 0;
}

int ssp_read_word(u32 *data)
{
	int timeout = TIMEOUT;

	while (!(__raw_readl(ssp->mmio_base + SSSR) & SSSR_RNE)) {
	        if (!--timeout)
	        	return -ETIMEDOUT;
		cpu_relax();
	}

	*data = __raw_readl(ssp->mmio_base + SSDR);
	printk(KERN_ALERT "ssp_read_word %d\n", *data);
	return 0;
}

module_init(corgits_init);
module_exit(corgits_exit);

MODULE_AUTHOR("Richard Purdie <rpurdie@rpsys.net>");
MODULE_DESCRIPTION("Corgi TouchScreen Driver");
MODULE_LICENSE("GPL");
