/*
 * TI LM3695 Backlight Driver
 *
 *			Copyright (C) 2013 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/i2c/lm3695.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>

/* Registers */
#define LM3695_GENERAL_PURPOSE	0x10
#define LM3695_BOOST_FREQ_SHIFT	5
#define LM3695_OVP_SHIFT		4
#define LM3695_STRING_SHIFT		3
#define LM3695_ENABLE_BRT_RW	BIT(2)
#define LM3695_DISABLE_RAMP		BIT(1)
#define LM3695_CHIP_ENABLE		BIT(0)

#define LM3695_RAMP_RATE		0x12

#define LM3695_BRT_MSB			0x14
#define LM3695_BRT_LSB			0x13

#define DEFAULT_BL_NAME			"lcd-backlight"
#define MAX_BRIGHTNESS			255

#define ldev_to_led(c)       container_of(c, struct lm3695, ldev)


enum lm3695_state {
        LM3695_OFF,
        LM3695_ON,
};

struct lm3695_device_config {
	enum lm3695_boost_freq boost_freq;
	enum lm3695_ovp ovp;
	enum lm3695_string_mode string;
	enum lm3695_ramp_rate ramp;
};

struct lm3695 {
	struct i2c_client *client;
	struct backlight_device *bl;
	struct led_classdev ldev;
	struct device *dev;
	struct lm3695_platform_data *pdata;
	struct delayed_work switchoff_work;
	int en_gpio;
	int gpio_enabled;
	enum lm3695_state status;
	struct mutex status_lock;
};

static struct lm3695_device_config lm3695_default_cfg = {
	.boost_freq = LM3695_BOOST_FREQ_443KHZ,
	.ovp = LM3695_OVP_16V,
	.string = LM3695_DUAL_STRINGS,
	.ramp = LM3695_RAMP_500us,
};

static int lm3695_init_device(struct lm3695 *);
static void lm3695_gpio_free(struct lm3695 *);

static int lm3695_write_byte(struct lm3695 *lm, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(lm->client, reg, data);
}

static ssize_t lm3695_dim_brightness_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lm3695 *lm = ldev_to_led(dev_get_drvdata(dev));
	return lm->ldev.brightness;
}

static void lm3695_switchoff_work(struct work_struct *work)
{
	struct lm3695 *lm = container_of(work, struct lm3695, switchoff_work.work);
	lm3695_gpio_free(lm);
}
static ssize_t lm3695_dim_brightness_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size) {
	struct lm3695 *lm = ldev_to_led(dev_get_drvdata(dev));
	u8 brt;
	u16 brt_offset = 0;
	int dim_brightness,i,retVal=0;
	i = sscanf(buf, "%d", &dim_brightness);

	/*
	 * Update LSB and MSB for the 11bit resolution
	 * In the range of 18-1530
	 */
	brt = min_t(int, dim_brightness, MAX_BRIGHTNESS);
	//If LED is turned on, enable the chip and init the regisers
	mutex_lock(&lm->status_lock);
	if (!lm->status && brt) {
		cancel_delayed_work_sync(&lm->switchoff_work);
		lm3695_init_device(lm);
	}
	lm->ldev.brightness = brt;
	//Adding an offset of 120 increase brightness at low values
	//1764 = Max CurrentVal 1905(16.24mA) - Min CurrentVal 141(.07mA)
	//252  = number of divisions
	if (brt) {
		brt_offset = (brt * 1764/252) + 120;
	}
	//Only 3 bits on LSB
	lm3695_write_byte(lm, LM3695_BRT_LSB, (brt_offset & 0x07));
	retVal = lm3695_write_byte(lm, LM3695_BRT_MSB, (brt_offset & 0x7F8)>>0x03);
	if (!brt && lm->status) {
		schedule_delayed_work(&lm->switchoff_work,msecs_to_jiffies(1000));
		lm->status = LM3695_OFF;
	}
	mutex_unlock(&lm->status_lock);
	return retVal;
}

#define LM3695_ATTR(_name)  \
	__ATTR(_name, 0644, lm3695_##_name##_show, lm3695_##_name##_store)

static struct device_attribute lm3695_led_attributes[] =
{
	LM3695_ATTR(dim_brightness),
	__ATTR_NULL
};

static int lm3695_gpio_request(struct lm3695 *lm, int gpio)
{
	int ret = 0;

        if (!lm->gpio_enabled) {
		ret = gpio_request(gpio, "lm3695_hwen");
		if (ret) {
			dev_err(lm->dev, "GPIO:%d request err: %d\n", gpio, ret);
			return ret;
		}
	}

	ret = gpio_direction_output(gpio, 1);
	if (ret) {
		dev_err(lm->dev, "GPIO:%d ouput err: %d\n", gpio, ret);
		gpio_free(gpio);
		return ret;
	}
        lm->gpio_enabled = 1;
	lm->en_gpio = gpio;
	lm->status  = LM3695_ON;
	return 0;
}

static void lm3695_gpio_free(struct lm3695 *lm)
{
	if (lm->en_gpio) {
		gpio_direction_output(lm->en_gpio, 0);
		gpio_free(lm->en_gpio);
		lm->gpio_enabled = 0;
	}
}

static int lm3695_init_device(struct lm3695 *lm)
{
	struct lm3695_platform_data *pdata = lm->pdata;
	struct lm3695_device_config *cfg = &lm3695_default_cfg;
	u8 val = 0;
	int ret;

	/*
	 * Sequence of device initialization
	 *
	 *  1) Control the GPIO for the HWEN pin
	 *  2) Update Ramp Rate Register
	 *  3) Set Chip Enable bit with GP register
	 *  4) 600us delay
	 */

	if (pdata && pdata->en_gpio) {
		ret = lm3695_gpio_request(lm, pdata->en_gpio);
		if (ret)
			return ret;
	}

	if (pdata) {
		cfg->boost_freq = pdata->boost_freq;
		cfg->ovp = pdata->ovp;
		cfg->string = pdata->string;
		cfg->ramp = pdata->ramp;
	}

	/* Update the RAMP RATE register */
	ret = lm3695_write_byte(lm, LM3695_RAMP_RATE, cfg->ramp);
	if (ret)
		goto err;

	/* Update the GENERAL PURPOSE register */
	val = (cfg->boost_freq << LM3695_BOOST_FREQ_SHIFT) |
		(cfg->ovp << LM3695_OVP_SHIFT) |
		(cfg->string << LM3695_STRING_SHIFT);

	if (pdata && pdata->disable_ramp)
		val |= LM3695_DISABLE_RAMP;

	val |= LM3695_ENABLE_BRT_RW | LM3695_CHIP_ENABLE;

	ret = lm3695_write_byte(lm, LM3695_GENERAL_PURPOSE, val);
	if (ret)
		goto err;

	mdelay(1);
	return 0;

err:
	lm3695_gpio_free(lm);
	return ret;
}

static void lm3695_set_led_brightness(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct lm3695 *lm = ldev_to_led(led_cdev);
	u8 brt;
	u16 brt_offset = 0;
	/*
	 * Update LSB and MSB for the 11bit resolution
	 * In the range of 18-1530
	 */
	brt = min_t(int, value, MAX_BRIGHTNESS);
	//If LED is turned on, enable the chip and init the regisers
	mutex_lock(&lm->status_lock);
	if (!lm->status && brt) {
		cancel_delayed_work_sync(&lm->switchoff_work);
		lm3695_init_device(lm);
	}
	//Adding an offset of 120 increase brightness at low values
	//1764 = Max CurrentVal 1905(16.24mA) - Min CurrentVal 141(.07mA)
	//252  = number of divisions
	if (brt) {
		brt_offset = (brt * 1764/252) + 120;
	}
	//Only 3 bits on LSB
	lm3695_write_byte(lm, LM3695_BRT_LSB, (brt_offset & 0x07));
	lm3695_write_byte(lm, LM3695_BRT_MSB, (brt_offset & 0x7F8)>>0x03);
	if (!brt && lm->status) {
		schedule_delayed_work(&lm->switchoff_work,msecs_to_jiffies(1000));
		lm->status = LM3695_OFF;
	}
	mutex_unlock(&lm->status_lock);
}
#if 0 //This function may not be used
static int lm3695_get_brightness(struct led_classdev *led_cdev)
{
	struct lm3695 *lm = ldev_to_led(led_cdev);
	return lm->ldev.brightness;
}
#endif
static int device_add_attributes(struct device *dev,
				 struct device_attribute *attrs)
{
	int error = 0;
	int i;

	if (attrs) {
		for (i = 0; attr_name(attrs[i]); i++) {
			error = device_create_file(dev, &attrs[i]);
			if (error)
				break;
		}
		if (error)
			while (--i >= 0)
				device_remove_file(dev, &attrs[i]);
	}
	return error;
}

static void device_remove_attributes(struct device *dev,
				     struct device_attribute *attrs)
{
	int i;

	if (attrs)
		for (i = 0; attr_name(attrs[i]); i++)
			device_remove_file(dev, &attrs[i]);
}

static int lm3695_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lm3695 *lm;
	struct lm3695_platform_data *pdata = cl->dev.platform_data;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	lm = devm_kzalloc(&cl->dev, sizeof(struct lm3695), GFP_KERNEL);
	if (!lm)
		return -ENOMEM;

	lm->client = cl;
	lm->dev = &cl->dev;
	lm->pdata = pdata;
	i2c_set_clientdata(cl, lm);

	//Need to init the device to check for device 
	ret = lm3695_init_device(lm);
	if (ret) {
		dev_err(lm->dev, "failed to init device: %d", ret);
		goto err_dev;
	}
	//No error, assume chip is active.
	lm->ldev.name = DEFAULT_BL_NAME; //TODO Need to set it from platform file
	lm->ldev.brightness = LED_OFF;
	lm->ldev.brightness_set = lm3695_set_led_brightness;
	lm->ldev.blink_set = NULL;
	lm->status = LM3695_OFF;
	lm->gpio_enabled = 0;

	ret = led_classdev_register(&cl->dev, &lm->ldev);
	if (ret < 0) {
		dev_err(&cl->dev,
				"couldn't register LED %s\n",
				pdata->name);
		goto err_bl;
	}
	mutex_init(&lm->status_lock);
	lm3695_gpio_free(lm);
	INIT_DELAYED_WORK_DEFERRABLE(&lm->switchoff_work, lm3695_switchoff_work);
	ret = device_add_attributes(lm->ldev.dev, lm3695_led_attributes);
	if (ret) {
		dev_err(&cl->dev,"couldn't ADD LED attr %s\n",pdata->name);
		goto err_attr;
	}
	//Default should be LOW power mode.Set HWEN gpio low.
	//This will save power till LED is turned on.
	dev_info(&cl->dev,"Front light\n");
	return 0;
err_attr:
	led_classdev_unregister(&lm->ldev);
err_bl:
	lm3695_gpio_free(lm);
err_dev:
	devm_kfree(&cl->dev, lm);
	i2c_set_clientdata(cl, NULL);
	return ret;
}

static int lm3695_remove(struct i2c_client *cl)
{
	struct lm3695 *lm = i2c_get_clientdata(cl);

	device_remove_attributes(lm->ldev.dev, lm3695_led_attributes);
	cancel_delayed_work_sync(&lm->switchoff_work);
	led_classdev_unregister(&lm->ldev);
	lm3695_gpio_free(lm);
	devm_kfree(&cl->dev, lm);
	i2c_set_clientdata(cl, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int lm3695_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lm3695 *lm = i2c_get_clientdata(client);
	dev_info(&client->dev,"Front light Suspend\n");
	lm3695_gpio_free(lm);
	return 0;
}

static int lm3695_resume(struct i2c_client *client)
{
	struct lm3695 *lm = i2c_get_clientdata(client);
	dev_info(&client->dev,"Front light resume\n");
	return  lm3695_init_device(lm);
}
#endif //CONFIG_PM

static const struct i2c_device_id lm3695_ids[] = {
	{"lm3695", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3695_ids);

static struct i2c_driver lm3695_driver = {
	.driver = {
		.name = "lm3695",
	},
	.probe = lm3695_probe,
	.remove = __devexit_p(lm3695_remove),
	.id_table = lm3695_ids,
#ifdef CONFIG_PM
	.suspend = lm3695_suspend,
	.resume = lm3695_resume,
#endif
};

static int __init lm3695_init(void)
{
	return i2c_add_driver(&lm3695_driver);
}

static void __exit lm3695_exit(void)
{
	i2c_del_driver(&lm3695_driver);
}

module_init(lm3695_init);
module_exit(lm3695_exit);

MODULE_DESCRIPTION("Texas Instruments LM3695 Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>");
MODULE_LICENSE("GPL");
