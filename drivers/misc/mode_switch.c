#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>

#define MODE_SWITCH_CLASS_NAME "mode_switch"

static int sel1_0_gpio;
static int sel1_1_gpio;
static int sel1_2_gpio;
static struct mutex mode_mux_lock;

struct class *mode_switch_class = NULL;

static ssize_t mode_switch_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	int sel1_0, sel1_1, sel1_2;

	if (gpio_is_valid(sel1_0_gpio) && gpio_is_valid(sel1_1_gpio) &&
		gpio_is_valid(sel1_2_gpio)) {

		mutex_lock(&mode_mux_lock);
		sel1_0 = gpio_get_value(sel1_0_gpio);
		sel1_1 = gpio_get_value(sel1_1_gpio);
		sel1_2 = gpio_get_value(sel1_2_gpio);
		mutex_unlock(&mode_mux_lock);

		if (sel1_0 == 1 && sel1_1 == 0 && sel1_2 == 1) {
			return sprintf(buf, "STB\n");
		} else if (sel1_0 == 0 && sel1_1 == 0 && sel1_2 == 0) {
			return sprintf(buf, "STG\n");
		} else if (sel1_0 == 0 && sel1_1 == 1 && sel1_2 == 1) {
			return sprintf(buf, "VTD\n");
		} else {
			return sprintf(buf, "Unknow mode\n");
		}
	}

	return sprintf(buf, "gpio error\n");
}

static ssize_t mode_switch_store(struct class *class,
	struct class_attribute *attr, const char *buf, size_t count)
{
	int ret;
	char *temp;

	//printk(KERN_INFO "mode_switch_store buf %s\n", buf);
	temp = strim(buf);
	//printk(KERN_INFO "mode_switch_store temp %s, length %d\n", temp, strlen(temp));

	if (gpio_is_valid(sel1_0_gpio) && gpio_is_valid(sel1_1_gpio) &&
		gpio_is_valid(sel1_2_gpio)) {
		if (!strcasecmp("STB", temp)) {
			mutex_lock(&mode_mux_lock);
			gpio_set_value(sel1_0_gpio, 1);
			gpio_set_value(sel1_1_gpio, 0);
			gpio_set_value(sel1_2_gpio, 1);
			mutex_unlock(&mode_mux_lock);
			printk(KERN_INFO "mode switch to STB\n");
		} else if (!strcasecmp("STG", temp)) {
			mutex_lock(&mode_mux_lock);
			gpio_set_value(sel1_0_gpio, 0);
			gpio_set_value(sel1_1_gpio, 0);
			gpio_set_value(sel1_2_gpio, 0);
			mutex_unlock(&mode_mux_lock);
			printk(KERN_INFO "mode switch to STG\n");
		} else if (!strcasecmp("VTD", temp)){
			mutex_lock(&mode_mux_lock);
			gpio_set_value(sel1_0_gpio, 0);
			gpio_set_value(sel1_1_gpio, 1);
			gpio_set_value(sel1_2_gpio, 1);
			mutex_unlock(&mode_mux_lock);
			printk(KERN_INFO "mode switch to VTD\n");
		} else {
			printk(KERN_INFO "Unknow mode\n");
		}
	} else {
		printk(KERN_INFO "mode_switch_store gpio error !!!\n");
	}
	return count;
}

static const struct class_attribute mode_class_attr =
	__ATTR(mode, 0664, mode_switch_show, mode_switch_store);

static const struct of_device_id of_model_switch_match[] = {
	{ .compatible = "mode-switch", },
	{},
};
MODULE_DEVICE_TABLE(of, of_model_switch_match);

static int mode_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	printk(KERN_INFO "mode_switch_probe\n");

	mode_switch_class = class_create(THIS_MODULE, MODE_SWITCH_CLASS_NAME);
	if (IS_ERR(mode_switch_class)) {
		ret = PTR_ERR(mode_switch_class);
		printk(KERN_ERR "mode_switch_probe: can't register mode_switch class\n");
		goto err;
	}

	ret = class_create_file(mode_switch_class, &mode_class_attr);
	if (ret) {
		printk(KERN_ERR "mode_switch_probe: can't create sysfs mode_class_attr\n");
		goto err_class;
	}

	dev->class = mode_switch_class;

	mutex_init(&mode_mux_lock);

	sel1_0_gpio = of_get_named_gpio(dev->of_node, "sel1_0", 0);
	sel1_1_gpio = of_get_named_gpio(dev->of_node, "sel1_1", 0);
	sel1_2_gpio = of_get_named_gpio(dev->of_node, "sel1_2", 0);

	if (!gpio_is_valid(sel1_0_gpio)) {
		printk(KERN_INFO "No sel1_0_gpio pin available in sel1_0_gpio=%d\n", sel1_0_gpio);
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, sel1_0_gpio, GPIOF_OUT_INIT_HIGH, "sel1_0_gpio");
		if (ret < 0) {
			printk(KERN_INFO "Failed to request sel1_0_gpio: %d\n", ret);
			return ret;
		}

		ret = devm_gpio_request_one(dev, sel1_1_gpio, GPIOF_OUT_INIT_LOW, "sel1_1_gpio");
		if (ret < 0) {
			printk(KERN_INFO "Failed to request sel1_1_gpio: %d\n", ret);
			return ret;
		}

		ret = devm_gpio_request_one(dev, sel1_2_gpio, GPIOF_OUT_INIT_HIGH, "sel1_2_gpio");
		if (ret < 0) {
			printk(KERN_INFO "Failed to request sel1_2_gpio: %d\n", ret);
			return ret;
		}
	}
	return 0;

err_class:
	class_destroy(mode_switch_class);
err:
	return ret;
}

static int mode_switch_remove(struct platform_device *pdev)
{
	if (gpio_is_valid(sel1_0_gpio) && gpio_is_valid(sel1_1_gpio) &&
		gpio_is_valid(sel1_2_gpio)) {

		gpio_free(sel1_0_gpio);
		gpio_free(sel1_1_gpio);
		gpio_free(sel1_2_gpio);
	}

	if (mode_switch_class != NULL) {
		class_remove_file(mode_switch_class, &mode_class_attr);
		class_destroy(mode_switch_class);
	}

	return 0;
}

static struct platform_driver mode_switch_driver = {
	.probe		= mode_switch_probe,
	.remove		= mode_switch_remove,
	.driver = {
		.name	= MODE_SWITCH_CLASS_NAME,
		.of_match_table = of_match_ptr(of_model_switch_match),
	},
};

static int __init model_switch_init(void)
{
	return platform_driver_register(&mode_switch_driver);
}

static void __exit model_switch_exit(void)
{
	platform_driver_unregister(&mode_switch_driver);
}

late_initcall(model_switch_init);
module_exit(model_switch_exit);

MODULE_LICENSE("GPL");
