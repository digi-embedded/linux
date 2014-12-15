/*
 * ad9389.c
 *
 * Copyright 2010 - Digi International, Inc. All Rights Reserved.
 *
 * Based on ad9889.c driver from Armadeus:
 * Copyright (C) 2009 Armadeus Systems <nicolas.colombain@armadeus.com>
 * And also on mxcfb_sii9022.c from Pegatron:
 * Copyright 2009 Pegatron Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <video/ad9389.h>

#define HPD_INT			0x80
#define MSEN_INT		0x40
#define	VS_INT			0x20
#define	AUDIO_FIFO_FULL_INT	0x10
#define	ITU656_ERR_INT		0x08
#define EDID_RDY_INT		0x04
#define EDID_LENGTH		256
#define DRV_NAME		"ad9389"


#define DEBUG
#define I2C_DBG			0x0001
#define EDID_DBG		0x0002
#define REGS_DBG		0x0004
#define SCREEN_DBG		0x0008

//static int debug = I2C_DBG | EDID_DBG | REGS_DBG | SCREEN_DBG;
static int debug = 0;

struct ad9389_dev *pad9389;


static inline int ad9389_read_reg(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static inline int ad9389_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static inline int ad9389_update_reg(struct i2c_client *client, u8 reg,
				    u8 mask, u8 val)
{
	u8 regval = i2c_smbus_read_byte_data(client, reg);
	regval &= ~mask;
	regval |= val;
	return ad9389_write_reg(client, reg, regval);
}

static void ad9389_set_av_mute(struct i2c_client *client, int mute)
{
	if (mute) {
		ad9389_update_reg(client, 0x45, 0x40, 0x40);
		ad9389_update_reg(client, 0x45, 0x80, 0);
	} else {
		ad9389_update_reg(client, 0x45, 0x40, 0x0);
		ad9389_update_reg(client, 0x45, 0x80, 0x80);
	}
}

static void ad9389_set_power_down(struct i2c_client *client, int powerd)
{
	ad9389_update_reg(client, 0x41, 0x40, powerd ? 0x40 : 0);
}

static int ad9389_disp_connected(struct i2c_client *client)
{
	return (ad9389_read_reg(client, 0x42) & 0x40) != 0;
}

static void ad9389_enable_i2s_ch(struct i2c_client *client, u8 enable, u8 ch)
{
	u8 mask;

	if (ch > 3)
		return;

	mask = 0x04 << ch;
	ad9389_update_reg(client, 0x0c, mask, enable ? mask : 0);
}

static int ad9389_is_enabled_i2s_ch(struct i2c_client *client, u8 ch)
{
	u8 mask;

	if (ch > 3)
		return -EINVAL;

	mask = 0x04 << ch;
	return (i2c_smbus_read_byte_data(client, 0x0c) & mask) != 0;
}

static void ad9389_set_i2s_sf(struct i2c_client *client, int sample_freq)
{
	ad9389_update_reg(client, 0x15, 0xf0, sample_freq << 4);
}

static void ad9389_set_hdmi_mode(struct i2c_client *client, int hdmi)
{
	ad9389_update_reg(client, 0xaf, 0x02, hdmi ? 0x02 : 0);
}

static void ad9389_set_if_cc(struct i2c_client *client, int chcnt)
{
	ad9389_update_reg(client, 0x50, 0xe0, chcnt << 5);
}

static void ad9389_set_spk_map(struct i2c_client *client, int map)
{
	ad9389_write_reg(client, 0x51, map);
}

static void ad9389_set_N(struct i2c_client *client, int N_val)
{
	ad9389_write_reg(client, 0x1, N_val >> 16);
	ad9389_write_reg(client, 0x2, N_val >> 8);
	ad9389_write_reg(client, 0x3, N_val & 0xff);
}

static int ad9389_get_N(struct i2c_client *client)
{
	int N_val;

	N_val = (ad9389_read_reg(client, 0x1) & 0x0f) << 16;
	N_val |= (ad9389_read_reg(client, 0x2) << 8);
	N_val |= ad9389_read_reg(client, 0x3);

	return N_val;
}

#ifdef DEBUG
static void ad9389_dump_edid(u8 *edid)
{
	int i;

	if (!(debug & EDID_DBG))
		return;

	printk("\nEDID data:\n");
	for (i = 0; i < EDID_LENGTH; i++) {
		if (i % 8 == 0)
			printk("\n");
		printk("%02x ", edid[i]);
	}
	printk("\n");
}

static void ad9389_dump_regs(struct i2c_client *client)
{
	int i;

	if (!(debug & REGS_DBG))
		return;

	printk("\nAD9389 regs:\n");
	for (i = 0; i < EDID_LENGTH; i++) {
		if (i % 8 == 0)
			printk("\n%03x: ", i);
		printk("%02x ", ad9389_read_reg(client, i));
	}
	printk(KERN_DEBUG "\n");
}

static void fb_dump_modeline( struct fb_videomode *modedb, int num)
{
	struct fb_videomode *mode;
	int i;

	if (!(debug & SCREEN_DBG))
		return;

	pr_debug("Monitor/TV supported modelines:\n");

	for (i = 0; i < num; i++) {
		mode = &modedb[i];

		printk("   \"%dx%d%s%d\" %lu.%02lu ",
			mode->xres, mode->yres,
			(mode->vmode & FB_VMODE_INTERLACED) ? "i@" : "@",
				mode->refresh,
			(PICOS2KHZ(mode->pixclock) * 1000UL)/1000000,
			(PICOS2KHZ(mode->pixclock) ) % 1000);
		printk("%d %d %d %d ",
			mode->xres,
			mode->xres + mode->right_margin,
			mode->xres + mode->right_margin + mode->hsync_len,
			mode->xres + mode->right_margin +
				mode->hsync_len + mode->left_margin );
		printk("%d %d %d %d ",
			mode->yres,
			mode->yres + mode->lower_margin,
			mode->yres + mode->lower_margin + mode->vsync_len,
			mode->yres + mode->lower_margin +
				mode->vsync_len + mode->upper_margin );
		printk("%shsync %svsync\n",
			(mode->sync & FB_SYNC_HOR_HIGH_ACT) ? "+" : "-",
			(mode->sync & FB_SYNC_VERT_HIGH_ACT) ? "+" : "-" );
	}
}
#else
static void ad9389_dump_edid(u8 *edid) {}
static void ad9389_dump_regs(struct i2c_client *client) {}
static void fb_dump_modeline( struct fb_videomode *modedb, int num) {}
#endif

static int ad9389_read_edid(struct i2c_client *client, u8 *edid)
{
	union i2c_smbus_data data;
	struct ad9389_pdata *config = client->dev.platform_data;
	struct ad9389_dev *ad9389 = i2c_get_clientdata(client);
	u8 *pd;
	int status, i;

	for (i = 0, pd = edid; i < EDID_LENGTH/I2C_SMBUS_BLOCK_MAX; i++,
			pd += I2C_SMBUS_BLOCK_MAX) {
		data.block[0] = I2C_SMBUS_BLOCK_MAX;
		status = i2c_smbus_xfer(ad9389->edid_ram->adapter,
					config->edid_addr,
					ad9389->edid_ram->flags,
					I2C_SMBUS_READ, i*I2C_SMBUS_BLOCK_MAX,
					I2C_SMBUS_I2C_BLOCK_DATA, &data);
		if (status < 0)
			return status;
		memcpy(pd, &data.block[1], data.block[0]);
	}

	return 0;
}

static int ad9389_parse_edid(struct fb_var_screeninfo *einfo, u8 *edid,
		int *dvi)
{
	int ret;

	if (einfo == NULL || edid == NULL || dvi == NULL)
		return -EINVAL;

	if (edid[1] == 0x00)
		return -ENOENT;

	/* Assume dvi if no CEA extension */
	*dvi = 1;
	if (edid[126] > 0) {
		/* CEA extensions */
		if (edid[128] == 0x02 && edid[131] & 0x40) {
			*dvi = 0;
		}
	}

	ret = fb_parse_edid(edid, einfo);
	if (ret)
		return -ret;

	/* This is valid for version 1.3 of the EDID */
	if ((edid[18] == 1) && (edid[19] == 3)) {
		einfo->height = edid[21] * 10;
		einfo->width = edid[22] * 10;
	}

	return 0;
}

static void ad9389_audio_setup(struct ad9389_dev *ad9389)
{
	struct i2c_client *client = ad9389->client;

	/* disable I2S channels */
	ad9389_enable_i2s_ch(client, 0, 0);
	ad9389_enable_i2s_ch(client, 0, 1);
	ad9389_enable_i2s_ch(client, 0, 2);
	ad9389_enable_i2s_ch(client, 0, 3);

	/* Set sample freq, currently hardcoded to 44KHz */
	ad9389_set_i2s_sf(client, 0);

	/* By default, enable i2s ch0. Can be modified through the sysfs */
	ad9389_set_N(client, 6272);
	ad9389_set_if_cc(client, 1);
	ad9389_set_spk_map(client, 0);
	ad9389_enable_i2s_ch(client, 1, 0);
	ad9389_set_av_mute(client, 0);
}


static void ad9389_fb_init(struct fb_info *info)
{
	struct ad9389_dev *ad9389 = pad9389;
	struct i2c_client *client = ad9389->client;
	struct ad9389_pdata *pdata = client->dev.platform_data;
	static struct fb_var_screeninfo var;
	int ret = 0;

	dev_dbg(info->dev, "%s\n", __func__);

	if (!ad9389_disp_connected(client)) {
		dev_dbg(info->dev, "%s, display disconnected\n", __func__);
		ad9389_set_power_down(client, 1);
		if(pdata->disp_disconnected)
			pdata->disp_disconnected(ad9389);
		return;
	}

	if(pdata->disp_connected)
		pdata->disp_connected(ad9389);

	dev_dbg(info->dev, "%s, display connected\n", __func__);
	memset(&var, 0, sizeof(var));

	/* Disable Power down and set mute to on */
	ad9389_set_power_down(client, 0);
	ad9389_set_av_mute(client, 1);

	/* set static reserved registers*/
	ad9389_write_reg(client,0x0a, 0x01);
	ad9389_write_reg(client, 0x98, 0x03);
	ad9389_write_reg(client, 0x9C, 0x38);

	/* Write magic numbers */
	ad9389_write_reg(client, 0xA2, 0x87);
	ad9389_write_reg(client, 0xA3, 0x87);

	/* set capture edge */
	ad9389_write_reg(client, 0xba, 0x60);
	ad9389_write_reg(client, 0x47, 0x80);

	mdelay(50);

	ret = ad9389_read_edid(ad9389->client, ad9389->edid_data);
	if (!ret) {
		ad9389_dump_edid(ad9389->edid_data);
		ret = ad9389_parse_edid(&var, ad9389->edid_data, &ad9389->dvi);
		if (!ret) {
			if (!ad9389->dvi) {
				ad9389_set_hdmi_mode(client, 1);
				/* FIXME audio setup should be done once we know
				 * the pixclock */
				ad9389_audio_setup(ad9389);
			}

			fb_edid_to_monspecs(ad9389->edid_data, &info->monspecs);
			if (info->monspecs.modedb_len) {
				fb_dump_modeline(info->monspecs.modedb,
						info->monspecs.modedb_len);
				if (pdata->vmode_to_modelist)
					pdata->vmode_to_modelist(
							info->monspecs.modedb,
							info->monspecs.modedb_len,
							 &info->modelist, &var);
				else
					fb_videomode_to_modelist(
							info->monspecs.modedb,
							info->monspecs.modedb_len,
							&info->modelist);
			}
		}
	} else {
		/* TODO */
		dev_warn(&client->dev, "NO EDID information found, using default mode?\n");
	}

	ad9389_dump_regs(client);

	if (pdata->vmode_to_var)
		pdata->vmode_to_var(ad9389, &var);

	var.activate = FB_ACTIVATE_ALL;
	acquire_console_sem();
	info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	memset((char *)info->screen_base, 0, info->fix.smem_len);
	fb_blank(info, FB_BLANK_UNBLANK);
	fb_prepare_logo(info, 0);
	fb_show_logo(info, 0);
	release_console_sem();
}

int ad9389_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	return 0;
}

static void ad9389_timer(unsigned long handle)
{
	struct ad9389_dev *ad9389 = (void *)handle;
	struct i2c_client *client =  ad9389->client;

	/* Just enable again the interrupt and stop the timer */
	dev_dbg(&client->dev, "%s, enabling interrupts\n", __func__);

	mutex_lock(&ad9389->irq_lock);
	enable_irq(client->irq);
	del_timer(&ad9389->timer);
	mutex_unlock(&ad9389->irq_lock);
}

static void ad9389_work(struct work_struct *work)
{
	struct ad9389_dev *ad9389 = container_of(work, struct ad9389_dev, work);
	struct i2c_client *client =  ad9389->client;
	struct ad9389_pdata *pdata = client->dev.platform_data;
	unsigned char irq_reg1;
	unsigned char irq_reg2;

	dev_dbg(&client->dev, "%s\n", __func__);

	mutex_lock(&ad9389->irq_lock);

	/* Disable interrupt */
	disable_irq(client->irq);

	irq_reg1 = ad9389_read_reg(client, 0x96);
	irq_reg2 = ad9389_read_reg(client, 0x97);

	while ((irq_reg1 & 0xc4) | (irq_reg2 & 0xc0)) {

		dev_dbg(&client->dev, "IRQ register %02x/%02x\n",
			irq_reg1, irq_reg2);

		/* hot plug detections interrupt? */
		if (irq_reg1 & HPD_INT) {
			dev_dbg(&client->dev, "HPD irq\n");
			ad9389_fb_init(ad9389->fbi);
		}

		/* check for EDID ready flag, then call EDID Handler */
		if (irq_reg1 & EDID_RDY_INT) {
			dev_dbg(&client->dev, "EDID_RDY_INT\n");
		}

		/* ack and check again */
		ad9389_write_reg(client, 0x96, irq_reg1);
		ad9389_write_reg(client, 0x97, irq_reg2);

		irq_reg1 = ad9389_read_reg(client, 0x96);
		irq_reg2 = ad9389_read_reg(client, 0x97);
	}

	/* Fire debouncing timer */
	mod_timer(&ad9389->timer, jiffies +
			msecs_to_jiffies(pdata->debounce_ms));

	mutex_unlock(&ad9389->irq_lock);
}

static irqreturn_t ad9389_handler(int irq, void *dev_id)
{
	struct ad9389_dev *dev = (struct ad9389_dev *) dev_id;

	dev_dbg(&dev->client->dev, "%s\n", __func__);
	schedule_work(&dev->work);

	return IRQ_HANDLED;
}

static ssize_t ad9389_show_mute(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int read, ret = 0;

	read = ad9389_read_reg(client, 0x45);
	if (read & 0x40)
		ret = snprintf(buf, PAGE_SIZE, "on");
	else if (read & 0x80)
	  	ret = snprintf(buf, PAGE_SIZE, "off");

	return ret;
}

static ssize_t ad9389_store_mute(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int mute;

	if (!strcmp(buf, "on"))
	      mute = 1;
	else if (!strcmp(buf, "off"))
	      mute = 0;
	else
	      return 0;

	ad9389_set_av_mute(client, mute);

	return count;
}

static ssize_t ad9389_show_N_param(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int N_val;

	N_val = ad9389_get_N(client);

	return snprintf(buf, PAGE_SIZE, "%d", N_val);
}

static ssize_t ad9389_store_N_param(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long N_val;

	N_val = simple_strtoul(buf, NULL, 10);
	ad9389_set_N(client, N_val);

	return count;
}

#define	ad9389_show_i2s_ch(num)						\
static ssize_t ad9389_show_i2s_ch##num(struct device *dev,		\
				   struct device_attribute *attr,	\
				   char *buf)				\
{									\
	struct i2c_client *client = to_i2c_client(dev);			\
	int read, ret = 0;						\
	read = ad9389_is_enabled_i2s_ch(client, num);			\
	if (read < 0)							\
		ret = snprintf(buf, PAGE_SIZE, "error");		\
	else if (read == 1)						\
		ret = snprintf(buf, PAGE_SIZE, "on");			\
	else 								\
	  	ret = snprintf(buf, PAGE_SIZE, "off");			\
	return ret;							\
}

#define	ad9389_store_i2s_ch(num)					\
static ssize_t ad9389_store_i2s_ch##num(struct device *dev,		\
				    struct device_attribute *attr,	\
				    const char *buf, size_t count)	\
{									\
	struct i2c_client *client = to_i2c_client(dev);			\
	int enable;							\
	if (!strcmp(buf, "on"))						\
		enable = 1;						\
	else if (!strcmp(buf, "off"))					\
		enable = 0;						\
	else								\
		return 0;						\
	ad9389_enable_i2s_ch(client, enable, num);			\
	return count;							\
}

#define audio_ch(num)			\
	static DEVICE_ATTR(i2s_ch##num, S_IWUSR | S_IRUGO, \
			   ad9389_show_i2s_ch##num, ad9389_store_i2s_ch##num)

static DEVICE_ATTR(mute, S_IWUSR | S_IRUGO, ad9389_show_mute, ad9389_store_mute);
static DEVICE_ATTR(N_param, S_IWUSR | S_IRUGO, ad9389_show_N_param,
		   ad9389_store_N_param);

ad9389_show_i2s_ch(0)
ad9389_show_i2s_ch(1)
ad9389_show_i2s_ch(2)
ad9389_show_i2s_ch(3)
ad9389_store_i2s_ch(0)
ad9389_store_i2s_ch(1)
ad9389_store_i2s_ch(2)
ad9389_store_i2s_ch(3)
audio_ch(0);
audio_ch(1);
audio_ch(2);
audio_ch(3);

static struct attribute *ad9389_attributes[] = {
	&dev_attr_mute.attr,
	&dev_attr_i2s_ch0.attr,
	&dev_attr_i2s_ch1.attr,
	&dev_attr_i2s_ch2.attr,
	&dev_attr_i2s_ch3.attr,
	&dev_attr_N_param.attr,
	NULL
};

static const struct attribute_group ad9389_attr_group = {
	.attrs = ad9389_attributes,
};

static struct notifier_block nb = {
	.notifier_call = ad9389_fb_event,
};

static int ad9389_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
	struct ad9389_pdata *pdata = client->dev.platform_data;
	struct ad9389_dev *ad9389;
	int ret = -EINVAL;

	/* Sanity checks */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	if (!pdata) {
		printk(KERN_ERR DRV_NAME ": Platform data not supplied\n");
		return -ENOENT;
	}

	if (!client->irq) {
		dev_err(&client->dev, ": Invalid irq value\n");
		return -ENOENT;
	}

	ad9389 = kzalloc(sizeof(struct ad9389_dev), GFP_KERNEL);
	if (ad9389 == NULL)
		return -ENOMEM;

	ad9389->edid_data = kmalloc(EDID_LENGTH, GFP_KERNEL);
	if (ad9389->edid_data == NULL) {
		ret = -ENOMEM;
		goto err_edid_alloc;
	}

	pad9389 = ad9389;
	ad9389->client = client;
	i2c_set_clientdata(client, ad9389);

	INIT_WORK(&ad9389->work, ad9389_work);
	mutex_init(&ad9389->irq_lock);
	mutex_lock(&ad9389->irq_lock);

	/* platform specific initialization (gpio, irq...) */
	if (pdata->hw_init)
		pdata->hw_init(ad9389);

	ret = request_irq(client->irq, ad9389_handler,
			  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, DRV_NAME,
			  ad9389);
	if (ret < 0) {
		dev_err(&client->dev, "Could not allocate IRQ (n %d)\n",
				client->irq);
		goto err_irq;
	}

	/**
	 * There is no good way to detect if the chip is present.
	 * We assume that it's present because somebody answered (ack) on the
	 * device address...
	 */
	ret = ad9389_read_reg(client, 0x00);
	if (ret < 0) {
		dev_err(&client->dev, "i2c transfer error, (device present?)\n");
		ret = -ENODEV;
		goto err_presence;
	}

	ad9389->chiprev = (u8)ret;
	ad9389->edid_ram = i2c_new_dummy(client->adapter, pdata->edid_addr);
	if (!ad9389->edid_ram) {
		dev_err(&client->dev, "can't add i2c device at 0x%x\n",
				pdata->edid_addr);
		goto err_presence;
	}

	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &ad9389_attr_group);
	if (ret)
		goto err_sysfs_file;

	ad9389->fbi = registered_fb[pdata->fbidx];
	fb_register_client(&nb);

	/* Ack any active interrupt and enable irqs */
	ad9389_write_reg(client, 0x94, 0x84);
	ad9389_write_reg(client, 0x95, 0xc3);
	ad9389_write_reg(client, 0x96, 0x84);
	ad9389_write_reg(client, 0x97, 0xc3);

	mutex_unlock(&ad9389->irq_lock);

	setup_timer(&ad9389->timer, ad9389_timer, (unsigned long)ad9389);

	dev_info(&client->dev, "device detected at address 0x%x, chip revision 0x%02x\n",
	ad9389_fb_init(registered_fb[pdata->fbidx]);

	       client->addr << 1, ad9389->chiprev);

	return 0;

err_sysfs_file:
	i2c_unregister_device(ad9389->edid_ram);
err_presence:
	free_irq(client->irq, ad9389);
err_irq:
	flush_scheduled_work();
	if (pdata->hw_deinit)
		pdata->hw_deinit(ad9389);
	kfree(ad9389->edid_data);
err_edid_alloc:
	kfree(ad9389);
	pad9389 = NULL;
	return ret;
}

static int ad9389_remove(struct i2c_client *client)
{
	struct ad9389_pdata *pdata = client->dev.platform_data;
	struct ad9389_dev *ad9389 = i2c_get_clientdata(client);

	free_irq(client->irq, ad9389);
	del_timer_sync(&ad9389->timer);
	flush_scheduled_work();
	sysfs_remove_group(&client->dev.kobj, &ad9389_attr_group);
	i2c_unregister_device(ad9389->edid_ram);
	fb_unregister_client(&nb);
	kfree(ad9389->edid_data);
	kfree(ad9389);
	pad9389 = NULL;

	if (pdata->hw_deinit)
		pdata->hw_deinit(ad9389);

	return 0;
}

#ifdef CONFIG_PM
static int ad9389_suspend(struct i2c_client *client, pm_message_t state)
{
	dev_dbg(&client->dev, "PM suspend\n");
	ad9389_set_power_down(client, 1);

	return 0;
}

static int ad9389_resume(struct i2c_client *client)
{
	struct ad9389_dev *ad9389 = pad9389;
	int ret;
	static struct fb_var_screeninfo var;

	dev_dbg(&client->dev, "PM resume\n");

	/* Disable Power down and set mute to on */
	ad9389_set_power_down(client, 0);
	ad9389_set_av_mute(client, 1);

	/* set static reserved registers*/
	ad9389_write_reg(client,0x0a, 0x01);
	ad9389_write_reg(client, 0x98, 0x03);
	ad9389_write_reg(client, 0x9C, 0x38);

	/* Write magic numbers */
	ad9389_write_reg(client, 0xA2, 0x87);
	ad9389_write_reg(client, 0xA3, 0x87);

	/* set capture edge */
	ad9389_write_reg(client, 0xba, 0x60);
	ad9389_write_reg(client, 0x47, 0x80);

	mdelay(50);

	ret = ad9389_read_edid(ad9389->client, ad9389->edid_data);
	if (!ret) {
		ad9389_dump_edid(ad9389->edid_data);
		ret = ad9389_parse_edid(&var, ad9389->edid_data, &ad9389->dvi);
		if (!ret) {
			if (!ad9389->dvi) {
				ad9389_set_hdmi_mode(client, 1);
				/* FIXME audio setup should be done once we know
				 * the pixclock */
				ad9389_audio_setup(ad9389);
			}
		}
	}
	return 0;
}
#else
#define ad9389_suspend	NULL
#define ad9389_resume	NULL
#endif


static struct i2c_device_id ad9389_id[] = {
	{ "ad9389", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ad9389_id);

static struct i2c_driver ad9389_driver = {
	.driver = {
		.name = "ad9389",
	},
	.probe		= ad9389_probe,
	.remove		= ad9389_remove,
	.suspend	= ad9389_suspend,
	.resume		= ad9389_resume,
	.id_table	= ad9389_id,

};

static int __init ad9389_init(void)
{
	return i2c_add_driver(&ad9389_driver);
}

static void __exit ad9389_exit(void)
{
	i2c_del_driver(&ad9389_driver);
}

module_init(ad9389_init);
module_exit(ad9389_exit);

MODULE_DESCRIPTION("AD9389 hdmi/dvi driver");
MODULE_AUTHOR("Digi International Inc.");
MODULE_LICENSE("GPL");
