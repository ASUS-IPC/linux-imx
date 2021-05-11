#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

static int reset_gpio;

static const struct of_device_id of_platform_reset_match[] = {
	{ .compatible = "platform-reset", },
	{},
};
MODULE_DEVICE_TABLE(of, of_platform_reset_match);

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
		printk("delay 100ms\n");
		mdelay(100);
		gpio_set_value_cansleep(reset_gpio, 0);
		mdelay(20);
		gpio_set_value_cansleep(reset_gpio, 1);

	}
	return 0;
}

static int platform_reset_remove(struct platform_device *pdev)
{
	gpio_free(reset_gpio);

	return 0;
}

static struct platform_driver platform_reset_driver = {
	.probe		= platform_reset_probe,
	.remove		= platform_reset_remove,
	.driver = {
		.name	= "platform-reset",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_platform_reset_match),
#endif
	},
};

module_platform_driver(platform_reset_driver);
