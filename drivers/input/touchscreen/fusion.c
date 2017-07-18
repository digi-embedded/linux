/*
 *  "Fusion"  touchscreen driver
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/gpio.h>

#include "fusion.h"

#define DRV_NAME		"fusion"


static struct fusion_data fusion;

static unsigned short normal_i2c[] = { FUSION_I2C_SLAVE_ADDR, I2C_CLIENT_END };

//I2C_CLIENT_INSMOD;

static int fusion_write_u8(u8 addr, u8 data)
{
	return i2c_smbus_write_byte_data(fusion.client, addr, data);
}

static int fusion_read_u8(u8 addr)
{
	return i2c_smbus_read_byte_data(fusion.client, addr);
}

static int fusion_read_block(u8 addr, u8 len, u8 *data)
{
#if 0
	/* When i2c_smbus_read_i2c_block_data() takes a block length parameter, we can do
	 * this. lm-sensors lists hints this has been fixed, but I can't tell whether it
	 * was or will be merged upstream. */

	return i2c_smbus_read_i2c_block_data(&fusion.client, addr, data);
#else
	u8 msgbuf0[1] = { addr };
	u16 slave = fusion.client->addr;
	u16 flags = fusion.client->flags;
	struct i2c_msg msg[2] = { { slave, flags, 1, msgbuf0 },
				  { slave, flags | I2C_M_RD, len, data }
	};

	return i2c_transfer(fusion.client->adapter, msg, ARRAY_SIZE(msg));
#endif
}


static int fusion_register_input(void)
{
	int ret;
	struct input_dev *dev;

	dev = fusion.input = input_allocate_device();
	if (dev == NULL)
		return -ENOMEM;

	dev->name = fusion.client->name;

	set_bit(EV_KEY, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);

	if(fusion.multitouch){
		input_set_abs_params(dev, ABS_MT_POSITION_X, 0,
			fusion.info.xres-1, 0, 0);
		input_set_abs_params(dev, ABS_MT_POSITION_Y, 0,
			fusion.info.yres-1, 0, 0);
		input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	}
	else {
		set_bit(ABS_X, dev->absbit);
		set_bit(ABS_Y, dev->absbit);
		set_bit(ABS_PRESSURE, dev->absbit);
		input_set_abs_params(dev, ABS_X, 0, fusion.info.xres-1, 0, 0);
		input_set_abs_params(dev, ABS_Y, 0, fusion.info.yres-1, 0, 0);
		input_set_abs_params(dev, ABS_PRESSURE, 0, 1 ,0, 0);
	}

	ret = input_register_device(dev);
	if (ret < 0)
		goto bail1;

	return 0;

bail1:
	input_free_device(dev);
	return ret;
}

#define WC_RETRY_COUNT 		3
static int fusion_write_complete(void)
{
	int ret, i;

	for(i=0; i<WC_RETRY_COUNT; i++)
	{
		ret = fusion_write_u8(FUSION_SCAN_COMPLETE, 0);
		if(ret == 0)
			break;
		else
			dev_err(&fusion.client->dev, "Write complete failed(%d): %d\n", i, ret);
	}

	return ret;
}

#define DATA_START	FUSION_DATA_INFO
#define	DATA_END	FUSION_SEC_TIDTS
#define DATA_LEN	(DATA_END - DATA_START + 1)
#define DATA_OFF(x)	((x) - DATA_START)

static int fusion_read_sensor(void)
{
	int ret;
	u8 data[DATA_LEN];

#define DATA(x) (data[DATA_OFF(x)])
	/* To ensure data coherency, read the sensor with a single transaction. */
	ret = fusion_read_block(DATA_START, DATA_LEN, data);
	if (ret < 0) {
		dev_err(&fusion.client->dev,
			"Read block failed: %d\n", ret);
		/* Clear fusion interrupt */
		fusion_write_complete();

		return ret;
	}

	fusion.f_num = DATA(FUSION_DATA_INFO)&0x03;

	fusion.y1 = DATA(FUSION_POS_X1_HI) << 8;
	fusion.y1 |= DATA(FUSION_POS_X1_LO);
	fusion.x1 = DATA(FUSION_POS_Y1_HI) << 8;
	fusion.x1 |= DATA(FUSION_POS_Y1_LO);
	fusion.z1 = DATA(FUSION_FIR_PRESS);
	fusion.tip1 = DATA(FUSION_FIR_TIDTS)&0x0f;
	fusion.tid1 = (DATA(FUSION_FIR_TIDTS)&0xf0)>>4;


	fusion.y2 = DATA(FUSION_POS_X2_HI) << 8;
	fusion.y2 |= DATA(FUSION_POS_X2_LO);
	fusion.x2 = DATA(FUSION_POS_Y2_HI) << 8;
	fusion.x2 |= DATA(FUSION_POS_Y2_LO);
	fusion.z2 = DATA(FUSION_SEC_PRESS);
	fusion.tip2 = DATA(FUSION_SEC_TIDTS)&0x0f;
	fusion.tid2 =(DATA(FUSION_SEC_TIDTS)&0xf0)>>4;

#undef DATA
	/* Clear fusion interrupt */
	return fusion_write_complete();
}

#define val_cut_max(x, max, reverse)	\
do					\
{					\
	if(x > max)			\
		x = max;		\
	if(reverse)			\
		x = (max) - (x);	\
}					\
while(0)

static void fusion_wq(struct work_struct *work)
{
	struct input_dev *dev = fusion.input;
	int save_points = 0;
	int x1 = 0, y1 = 0, z1 = 0, x2 = 0, y2 = 0, z2 = 0;

	if (fusion_read_sensor() < 0)
		goto out;

	dev_dbg(&fusion.client->dev,"tip1, tid1, x1, y1, z1 (%x,%x,%d,%d,%d); tip2, tid2, x2, y2, z2 (%x,%x,%d,%d,%d)\n",
		fusion.tip1, fusion.tid1, fusion.x1, fusion.y1, fusion.z1,
		fusion.tip2, fusion.tid2, fusion.x2, fusion.y2, fusion.z2);

	val_cut_max(fusion.x1, fusion.info.xres-1, fusion.info.xy_reverse);
	val_cut_max(fusion.y1, fusion.info.yres-1, fusion.info.xy_reverse);
	val_cut_max(fusion.x2, fusion.info.xres-1, fusion.info.xy_reverse);
	val_cut_max(fusion.y2, fusion.info.yres-1, fusion.info.xy_reverse);

	if(fusion.tip1 == 1)
	{
		if(fusion.tid1 == 1)
		{
			/* first point */
			x1 = fusion.x1;
			y1 = fusion.y1;
			z1 = fusion.z1;
			save_points |= FUSION_SAVE_PT1;
		}
		else if(fusion.tid1 == 2)
		{
			/* second point ABS_DISTANCE second point pressure, BTN_2 second point touch */
			x2 = fusion.x1;
			y2 = fusion.y1;
			z2 = fusion.z1;
			save_points |= FUSION_SAVE_PT2;
		}
	}

	if(fusion.tip2 == 1)
	{
		if(fusion.tid2 == 2)
		{
			/* second point ABS_DISTANCE second point pressure, BTN_2 second point touch */
			x2 = fusion.x2;
			y2 = fusion.y2;
			z2 = fusion.z2;
			save_points |= FUSION_SAVE_PT2;
		}
		else if(fusion.tid2 == 1)/* maybe this will never happen */
		{
			/* first point */
			x1 = fusion.x2;
			y1 = fusion.y2;
			z1 = fusion.z2;
			save_points |= FUSION_SAVE_PT1;
		}
	}

	if( fusion.multitouch) {
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, z1);
		input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(dev, ABS_MT_POSITION_X, x1);
		input_report_abs(dev, ABS_MT_POSITION_Y, y1);
		input_mt_sync(dev);
		input_report_abs(dev, ABS_MT_TOUCH_MAJOR, z2);
		input_report_abs(dev, ABS_MT_WIDTH_MAJOR, 2);
		input_report_abs(dev, ABS_MT_POSITION_X, x2);
		input_report_abs(dev, ABS_MT_POSITION_Y, y2);
		input_mt_sync(dev);
	}
	else {
		input_report_key(dev, BTN_TOUCH, fusion.tip1);
		input_report_abs(dev, ABS_PRESSURE, z1);
		if(fusion.tip1){
			input_report_abs(dev, ABS_X, x1);
			input_report_abs(dev, ABS_Y, y1);
		}
	}

	input_sync(dev);

out:
	enable_irq(fusion.client->irq);

}
static DECLARE_WORK(fusion_work, fusion_wq);

static irqreturn_t fusion_interrupt(int irq, void *dev_id)
{
	disable_irq_nosync(fusion.client->irq);

	queue_work(fusion.workq, &fusion_work);
	return IRQ_HANDLED;
}

const static u8* g_ver_product[4] = {
	"10Z8", "70Z7", "43Z6", ""
};

static int fusion_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	u8 ver_product, ver_id;
	struct device_node *np = i2c->dev.of_node;
	u32 version;

	if (!np)
                return -ENODEV;

	if(!i2c->irq)
	{
		dev_err(&i2c->dev, "fusion irq < 0 \n");
		ret = -ENOMEM;
		goto bail1;
	}

	/* Attach the I2C client */
	fusion.client =  i2c;
	i2c_set_clientdata(i2c, &fusion);

	printk(KERN_INFO "Fusion :Touchscreen registered with bus id (%d) with slave address 0x%x\n",
			i2c_adapter_id(fusion.client->adapter),	fusion.client->addr);

	fusion.multitouch = 0;
	if (of_property_read_bool(np, "touchrev,is-multitouch"))
		fusion.multitouch = 1;

	/* Read out a lot of registers */
	ret = fusion_read_u8(FUSION_VIESION_INFO_LO);
	if (ret < 0) {
		dev_err(&i2c->dev, "query failed: %d\n", ret);
		goto bail1;
	}
	ver_product = (((u8)ret) & 0xc0) >> 6;
	version = (10 + ((((u32)ret)&0x30) >> 4)) * 100000;
	version += (((u32)ret)&0xf) * 1000;
	/* Read out a lot of registers */
	ret = fusion_read_u8(FUSION_VIESION_INFO);
		if (ret < 0) {
		dev_err(&i2c->dev, "query failed: %d\n", ret);
		goto bail1;
	}
	ver_id = ((u8)(ret) & 0x6) >> 1;
	version += ((((u32)ret) & 0xf8) >> 3) * 10;
	version += (((u32)ret) & 0x1) + 1; /* 0 is build 1, 1 is build 2 */
	printk(KERN_INFO "Fusion version product %s(%d)\n", g_ver_product[ver_product] ,ver_product);
	printk(KERN_INFO "Fusion version id %s(%d)\n", ver_id ? "1.4" : "1.0", ver_id);
	printk(KERN_INFO "Fusion version series (%d)\n", version);

	switch(ver_product)
	{
	case FUSION_VIESION_07: /* 7 inch */
		fusion.info.xres = FUSION07_XMAX;
		fusion.info.yres = FUSION07_YMAX;
		fusion.info.xy_reverse = FUSION07_REV;
		break;
	case FUSION_VIESION_43: /* 4.3 inch */
		fusion.info.xres = FUSION43_XMAX;
		fusion.info.yres = FUSION43_YMAX;
		fusion.info.xy_reverse = FUSION43_REV;
		break;
	default: /* FUSION_VIESION_10 10 inch */
		fusion.info.xres = FUSION10_XMAX;
		fusion.info.yres = FUSION10_YMAX;
		fusion.info.xy_reverse = FUSION10_REV;
		break;
	}

	/* Register the input device. */
	ret = fusion_register_input();
	if (ret < 0) {
		dev_err(&i2c->dev, "can't register input: %d\n", ret);
		goto bail1;
	}

	/* Create a worker thread */
	fusion.workq = create_singlethread_workqueue(DRV_NAME);
	if (fusion.workq == NULL) {
		dev_err(&i2c->dev, "can't create work queue\n");
		ret = -ENOMEM;
		goto bail2;
	}


	/* Register for the interrupt and enable it. Our handler will
	*  start getting invoked after this call. */
	ret = request_irq(i2c->irq, fusion_interrupt, IRQF_TRIGGER_RISING,
	i2c->name, &fusion);
	if (ret < 0) {
		dev_err(&i2c->dev, "can't get irq %d: %d\n", i2c->irq, ret);
		goto bail3;
	}
	/* clear the irq first */
	ret = fusion_write_u8(FUSION_SCAN_COMPLETE, 0);
	if (ret < 0) {
		dev_err(&i2c->dev, "Clear irq failed: %d\n", ret);
		goto bail4;
	}

	return 0;

	bail4:
	free_irq(i2c->irq, &fusion);

	bail3:
	destroy_workqueue(fusion.workq);
	fusion.workq = NULL;


	bail2:
	input_unregister_device(fusion.input);
	bail1:

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int fusion_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);
	flush_workqueue(fusion.workq);

	return 0;
}

static int fusion_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* Give the controller time to come up */
	schedule_timeout_uninterruptible(100);
	enable_irq(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(fusion_dev_pm_ops, fusion_suspend, fusion_resume);

static int fusion_remove(struct i2c_client *i2c)
{
	destroy_workqueue(fusion.workq);
	free_irq(i2c->irq, &fusion);
	input_unregister_device(fusion.input);
	i2c_set_clientdata(i2c, NULL);

	printk(KERN_INFO "Fusion driver removed\n");

	return 0;
}

static struct i2c_device_id fusion_id[] = {
	{"fusion", 0},
	{},
};

static const struct of_device_id fusion_dt_ids[] = {
        {
                .compatible = "touchrev,fusion-touch",
        }, {
                /* sentinel */
        }
};
MODULE_DEVICE_TABLE(of, fusion_dt_ids);

static struct i2c_driver fusion_i2c_drv = {
	.driver = {
		.name		= DRV_NAME,
		.of_match_table = fusion_dt_ids,
		.pm		= &fusion_dev_pm_ops,
	},
	.probe          = fusion_probe,
	.remove         = fusion_remove,
	.id_table       = fusion_id,
	.address_list   = normal_i2c,
};


static int __init fusion_init( void )
{
	int ret;

	memset(&fusion, 0, sizeof(fusion));

	/* Probe for Fusion on I2C. */
	ret = i2c_add_driver(&fusion_i2c_drv);
	if (ret < 0) {
		printk(KERN_ERR  "fusion_init can't add i2c driver: %d\n", ret);
	}

	return ret;
}

static void __exit fusion_exit( void )
{
	i2c_del_driver(&fusion_i2c_drv);
}
module_init(fusion_init);
module_exit(fusion_exit);

MODULE_DESCRIPTION("Fusion Touchscreen Driver");
MODULE_LICENSE("GPL");

