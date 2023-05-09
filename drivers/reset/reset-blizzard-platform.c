#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

static int reset_gpio;
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
	int ret;

	reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	if (!gpio_is_valid(reset_gpio)) {
		printk("No reset-gpio pin available in gpio-rst\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, reset_gpio, GPIOF_OUT_INIT_LOW, "Platform reset");
		if (ret < 0) {
			printk("Failed to request reset gpio: %d\n", ret);
			return ret;
		}
		printk("reset-gpio request success\n");
		mdelay(100);
		platform_reset_trigger();
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
	printk("platform_reset_resume\n");
	platform_reset_trigger();
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
