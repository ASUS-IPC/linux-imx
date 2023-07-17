#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

static int reset_gpio, detect_gpio;
static int  pciehub_en_gpio;
static int  pciehub_rst_gpio;
static int trigger_flag=0;

static const struct of_device_id of_platform_reset_match[] = {
	{ .compatible = "blizzard-platform-reset", },
	{},
};
MODULE_DEVICE_TABLE(of, of_platform_reset_match);

void platform_reset_trigger(void)
{
	if( trigger_flag == 0 ) {
		trigger_flag=1;
		printk("Trigger platform_reset\n");
		gpio_set_value_cansleep(reset_gpio, 0);
		mdelay(20);
		gpio_set_value_cansleep(reset_gpio, 1);
	} else {
		printk("Bypass platform_reset\n");
	}
}
EXPORT_SYMBOL_GPL(platform_reset_trigger);

static int platform_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int detect_pin;
	int ret;

	printk(KERN_INFO "platform_reset_probe\n");

	reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	if (!gpio_is_valid(reset_gpio) && (reset_gpio == ERR_PTR(-EPROBE_DEFER))) {
		printk("platform_reset_probe reset_gpio EPROBE_DEFER\n");
		return -EPROBE_DEFER;
	} else if (!gpio_is_valid(reset_gpio)) {
		printk("No reset-gpio pin available in gpio-rst=%d\n", reset_gpio);
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, reset_gpio, GPIOF_OUT_INIT_HIGH, "Platform reset");
		if (ret < 0) {
			printk("Failed to request reset gpio: %d\n", ret);
			return ret;
		}
		/*
		//the first reset is  trigger at uboot
		printk("reset-gpio request success\n");
		mdelay(100);
		platform_reset_trigger();
		*/
	}

	detect_gpio = of_get_named_gpio(dev->of_node, "detect-gpio", 0);
	if (!gpio_is_valid(detect_gpio) && (detect_gpio == ERR_PTR(-EPROBE_DEFER))) {
		printk("platform_reset_probe  detect_gpio EPROBE_DEFER\n");
		return -EPROBE_DEFER;
	} else if (!gpio_is_valid(detect_gpio)) {
		printk("No detect-gpio pin available in pcie_detect, , suppose the device is evt\n");
	} else {
		ret = devm_gpio_request_one(dev, detect_gpio, GPIOF_DIR_IN, "DETECT_GPIO");
		if (ret < 0) {
			printk("Failed to request detect_gpio gpio: %d\n", ret);
			return ret;
		}
	}

	pciehub_en_gpio = of_get_named_gpio(dev->of_node, "pciehub_en-gpio", 0);
	if (!gpio_is_valid(pciehub_en_gpio) && (pciehub_en_gpio == ERR_PTR(-EPROBE_DEFER))) {
		printk("platform_reset_probe pciehub_en_gpio EPROBE_DEFER\n");
		return -EPROBE_DEFER;
	} else if (!gpio_is_valid(pciehub_en_gpio)) {
		printk("No pciehub_en-gpio pin available in pciehub_en-gpio=%d, suppose the device is evt\n", pciehub_en_gpio);
	} else {
		ret = devm_gpio_request_one(dev, pciehub_en_gpio, GPIOF_OUT_INIT_LOW, "Pciehub enable");
		if (ret < 0) {
			printk("Failed to request pciehub_en gpio: %d\n", ret);
			return ret;
		}
	}

	pciehub_rst_gpio = of_get_named_gpio(dev->of_node, "pciehub_rst-gpio", 0);
	if (!gpio_is_valid(pciehub_rst_gpio) && (pciehub_rst_gpio == ERR_PTR(-EPROBE_DEFER))) {
		printk("platform_reset_probe pciehub_rst_gpio EPROBE_DEFER\n");
		return -EPROBE_DEFER;
	} else if (!gpio_is_valid(pciehub_rst_gpio)) {
		printk("No pciehub_rst-gpio pin available in pciehub_rst-gpio=%d, suppose the device is evt\n", pciehub_rst_gpio);
	} else {
		ret = devm_gpio_request_one(dev, pciehub_rst_gpio, GPIOF_OUT_INIT_LOW, "Pciehub reset");
		if (ret < 0) {
			printk("Failed to request pciehub_rst gpio: %d\n", ret);
			return ret;
		}
	}

	if (gpio_is_valid(detect_gpio)) {
		detect_pin = gpio_get_value(detect_gpio);
		printk("platform_reset_probe detect_pin=%d\n", detect_pin);
	} else {
		detect_pin = 1;
		printk("No detect_pin, set detect_pin to high\n");
	}

	if (!detect_pin && gpio_is_valid(pciehub_en_gpio)
					&& gpio_is_valid(pciehub_rst_gpio)) {
		printk("platform_reset_probe: powering on pciehub");
		gpio_set_value_cansleep(pciehub_en_gpio, 0); //disable
		gpio_set_value_cansleep(pciehub_rst_gpio, 0); //to low
		mdelay(10);
		gpio_set_value_cansleep(pciehub_en_gpio, 1); //enable
		mdelay(100);
		gpio_set_value_cansleep(pciehub_rst_gpio, 1); //to default
	}

	if (detect_pin && gpio_is_valid(pciehub_en_gpio)) {
		printk("platform_reset_probe: powering off pciehub");
		gpio_set_value_cansleep(pciehub_en_gpio, 0); //disable
		gpio_set_value_cansleep(pciehub_rst_gpio, 0); //to low
	}

	return 0;
}

static int platform_reset_remove(struct platform_device *pdev)
{
	gpio_free(reset_gpio);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int platform_reset_suspend(struct device *dev)
{
	printk("platform_reset_suspend, clear flag\n");
	trigger_flag=0;

	return 0;
}

static int platform_reset_resume(struct device *dev)
{
	int detect_pin;

	printk("platform_reset_resume\n");
	platform_reset_trigger();

	if (gpio_is_valid(detect_gpio)) {
		detect_pin = gpio_get_value(detect_gpio);
		printk("platform_reset_resume detect_pin=%d\n", detect_pin);
	}

	if (!detect_pin && gpio_is_valid(pciehub_rst_gpio)) {
		printk("platform_reset_resume:PCIEHUB reset ");
		gpio_set_value_cansleep(pciehub_rst_gpio, 0); //active reset
		mdelay(20);
		gpio_set_value_cansleep(pciehub_rst_gpio, 1); //to default
	}

	return 0;
}
#endif

static const struct dev_pm_ops platform_reset_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(platform_reset_suspend, platform_reset_resume)
};

static struct platform_driver platform_reset_driver = {
	.probe		= platform_reset_probe,
	.remove		= platform_reset_remove,
	.driver = {
		.name	= "blizzard-platform-reset",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_platform_reset_match),
#endif
		.pm	=&platform_reset_pm_ops,
	},
};

module_platform_driver(platform_reset_driver);
MODULE_LICENSE("GPL");
