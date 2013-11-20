/* da9063-i2c.c - I2C device access for DA9063
 * Copyright (C) 2013  Dialog Semiconductor Ltd.
 * Copyright (C) 2013  Digi International Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>

#include <linux/mfd/core.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/registers.h>


int da9063_read_device(struct da9063 *da9063, u8 reg,
	int bytes, u8 *dest)
{
	struct i2c_client *i2c = da9063->i2c;
	struct i2c_msg xfer[2];
	int ret;

	xfer[0].addr = i2c->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg;

	xfer[1].addr = i2c->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = bytes;
	xfer[1].buf = dest;

	ret = i2c_transfer(i2c->adapter, xfer, 2);
	if (ret < 0)
		return ret;
	if (ret != 2)
		return -EIO;

	return 0;
}

int da9063_write_device(struct da9063 *da9063, u8 reg,
	int bytes, const u8 *src)
{
	int ret;
	u8 * buf = kzalloc( (bytes+1), GFP_KERNEL );

	if( NULL == buf )
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&buf[1], src, bytes);

	ret = i2c_master_send(da9063->i2c, buf, bytes + 1);

	kfree( buf );

	if (ret != bytes + 1)
		return -EIO;

	return 0;
}

static int da9063_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct da9063 *da9063;

	if( i2c->adapter->dev.of_node == NULL ){
		return -EPROBE_DEFER;
	}

	da9063 = devm_kzalloc(&i2c->dev, sizeof(struct da9063), GFP_KERNEL);
	if (da9063 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, da9063);
	da9063->dev = &i2c->dev;
	da9063->i2c = i2c;

	return da9063_device_init(da9063, i2c->irq);
}

static int da9063_i2c_remove(struct i2c_client *i2c)
{
	struct da9063 *da9063 = i2c_get_clientdata(i2c);

	da9063_device_exit(da9063);

	return 0;
}

static const struct i2c_device_id da9063_i2c_id[] = {
	{"da9063", DA9063_ID},
	{},
};
MODULE_DEVICE_TABLE(i2c, da9063_i2c_id);


#ifdef CONFIG_OF
static const struct of_device_id dialog_dt_ids[] = {
        { .compatible = "dlg,da9063", .data = &da9063_i2c_id[0] },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dialog_dt_ids);
#endif

static struct i2c_driver da9063_i2c_driver = {
	.driver = {
		.name = "da9063",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = dialog_dt_ids,
#endif
	},
	.probe    = da9063_i2c_probe,
	.remove   = da9063_i2c_remove,
	.id_table = da9063_i2c_id,
};

static int __init da9063_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&da9063_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register da9063 I2C driver\n");

	return ret;
}
subsys_initcall(da9063_i2c_init);

static void __exit da9063_i2c_exit(void)
{
	i2c_del_driver(&da9063_i2c_driver);
}
module_exit(da9063_i2c_exit);
