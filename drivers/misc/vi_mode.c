#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>

static int soc_vi_mode0, soc_vi_mode1, soc_vi_mode2, soc_vi_mode3;
static int soc_vi_mode4, soc_vi_mode5, soc_vi_mode6, soc_vi_mode7;
	

static const struct of_device_id of_gpio_vimode_match[] = {
	{ .compatible = "gpio-vimode", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_vimode_match);


static int vimode_show(struct seq_file *m, void *v)
{
	int mode0, mode1, mode2, mode3,
		mode4, mode5, mode6, mode7;
       char vbuffer[64] = {'\n'};
	char ibuffer[64] = {'\n'};

	sprintf(vbuffer, "%s", "voltage mode:");
	sprintf(ibuffer, "%s", "current mode:");
	mode0 = gpio_get_value(soc_vi_mode0);
	mode1 = gpio_get_value(soc_vi_mode1);
	mode2 = gpio_get_value(soc_vi_mode2);
	mode3 = gpio_get_value(soc_vi_mode3);
	mode4 = gpio_get_value(soc_vi_mode4);
	mode5 = gpio_get_value(soc_vi_mode5);
	mode6 = gpio_get_value(soc_vi_mode6);
	mode7 = gpio_get_value(soc_vi_mode7);

	if (mode0) {
		strcat(ibuffer, " 0,");
	} else {
		strcat(vbuffer, " 0,");
	}

	if (mode1) {
		strcat(ibuffer, " 1,");
	} else {
		strcat(vbuffer, " 1,");
	}

	if (mode2) {
		strcat(ibuffer, " 2,");
	} else {
		strcat(vbuffer, " 2,");
	}
	if (mode3) {
		strcat(ibuffer, " 3,");
	} else {
		strcat(vbuffer, " 3,");
	}
	if (mode4) {
		strcat(ibuffer, " 4,");
	} else {
		strcat(vbuffer, " 4,");
	}
	if (mode5) {
		strcat(ibuffer, " 5,");
	} else {
		strcat(vbuffer, " 5,");
	}
	if (mode6) {
		strcat(ibuffer, " 6,");
	} else {
		strcat(vbuffer, " 6,");
	}

	if (mode7) {
		strcat(ibuffer, " 7,");
	} else {
		strcat(vbuffer, " 7,");
	}
	printk("vimode_show voltage=%s\n", vbuffer);
	printk("vimode_show current=%s\n", ibuffer);
	strcat(vbuffer, ibuffer);
	
	seq_printf(m, "%s\n", vbuffer);
	return 0;
}

static int vimode_open(struct inode *inode, struct file *file)
{
	return single_open(file, vimode_show, NULL);
}


static struct file_operations vimode_ops = {
	.owner	= THIS_MODULE,
	.open	= vimode_open,
	.read	= seq_read,
};


static int gpio_vimode_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	struct proc_dir_entry* file;

	printk("gpio_vimode_probe+\n");
	soc_vi_mode0 = of_get_named_gpio(dev->of_node, "soc_vi_mode0-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode0)) {
		printk("No soc_vi_mode0 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode0, GPIOF_DIR_IN, "GPIO_VI_MODE0");
		if (ret < 0) {
			printk("Failed to request VI_MODE0 gpio: %d\n", ret);
			return ret;
		}
	}

	soc_vi_mode1 = of_get_named_gpio(dev->of_node, "soc_vi_mode1-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode1)) {
		printk("No soc_vi_mode1 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode1, GPIOF_DIR_IN, "GPIO_VI_MODE1");
		if (ret < 0) {
			printk("Failed to request VI_MODE1 gpio: %d\n", ret);
			return ret;
		}
	}

	soc_vi_mode2 = of_get_named_gpio(dev->of_node, "soc_vi_mode2-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode2)) {
		printk("No soc_vi_mode2 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode2, GPIOF_DIR_IN, "GPIO_VI_MODE2");
		if (ret < 0) {
			printk("Failed to request VI_MODE2 gpio: %d\n", ret);
			return ret;
		}
	}
	
	soc_vi_mode3 = of_get_named_gpio(dev->of_node, "soc_vi_mode3-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode3)) {
		printk("No soc_vi_mode3 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode3, GPIOF_DIR_IN, "GPIO_VI_MODE3");
		if (ret < 0) {
			printk("Failed to request VI_MODE3 gpio: %d\n", ret);
			return ret;
		}
	}

	soc_vi_mode4 = of_get_named_gpio(dev->of_node, "soc_vi_mode4-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode4)) {
		printk("No soc_vi_mode4 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode4, GPIOF_DIR_IN, "GPIO_VI_MODE4");
		if (ret < 0) {
			printk("Failed to request VI_MODE4 gpio: %d\n", ret);
			return ret;
		}
	}

	soc_vi_mode5 = of_get_named_gpio(dev->of_node, "soc_vi_mode5-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode5)) {
		printk("No soc_vi_mode5 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode5, GPIOF_DIR_IN, "GPIO_VI_MODE5");
		if (ret < 0) {
			printk("Failed to request VI_MODE5 gpio: %d\n", ret);
			return ret;
		}
	}

	soc_vi_mode6 = of_get_named_gpio(dev->of_node, "soc_vi_mode6-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode6)) {
		printk("No soc_vi_mode6 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode6, GPIOF_DIR_IN, "GPIO_VI_MODE6");
		if (ret < 0) {
			printk("Failed to request VI_MODE6 gpio: %d\n", ret);
			return ret;
		}
	}

	soc_vi_mode7 = of_get_named_gpio(dev->of_node, "soc_vi_mode7-gpios", 0);
	if (!gpio_is_valid(soc_vi_mode7)) {
		printk("No soc_vi_mode7 pin available in gpio-vimode\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, soc_vi_mode7, GPIOF_DIR_IN, "GPIO_VI_MODE7");
		if (ret < 0) {
			printk("Failed to request VI_MODE7 gpio: %d\n", ret);
			return ret;
		}
	}

	file = proc_create("VI_mode", 0444, NULL, &vimode_ops);
	if (!file)
		return -ENOMEM;

	printk("gpio_vimode_probe-\n");
	return 0;
}

static int gpio_vimode_remove(struct platform_device *pdev)
{
	gpio_free(soc_vi_mode0);
	gpio_free(soc_vi_mode1);
	gpio_free(soc_vi_mode2);
	gpio_free(soc_vi_mode3);
	gpio_free(soc_vi_mode4);
	gpio_free(soc_vi_mode5);
	gpio_free(soc_vi_mode6);
	gpio_free(soc_vi_mode7);

	return 0;
}

static struct platform_driver vimode_driver = {
	.probe		= gpio_vimode_probe,
	.remove		= gpio_vimode_remove,
	.driver = {
		.name	= "vimode",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_gpio_vimode_match),
#endif
	},
};

module_platform_driver(vimode_driver);
