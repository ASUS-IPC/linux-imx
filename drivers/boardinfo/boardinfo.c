#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

static const char *boardinfo;
static int id0_gpio, id1_gpio;
static int sku0_gpio, sku1_gpio, sku2_gpio;

static const struct of_device_id of_gpio_hwid_match[] = {
	{ .compatible = "gpio-hwid", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_hwid_match);

static int info_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", boardinfo);
	return 0;
}

static int ver_show(struct seq_file *m, void *v)
{
	int id0, id1;
	int hwid;
	char *boardver;

	id0 = gpio_get_value(id0_gpio);
	id1 = gpio_get_value(id1_gpio);

	hwid = (id1 << 1) + id0;

	switch (hwid) {
		case 0:
			boardver = "1.00";
			break;
		case 1:
			boardver = "1.01";
			break;
		case 2:
			boardver = "1.02";
			break;
		case 3:
			boardver = "2.00";
			break;
		default:
			boardver = "unknown";
			break;
	}

	seq_printf(m, "%s\n", boardver);
	return 0;
}

static int sku_show(struct seq_file *m, void *v)
{
	int id0, id1, id2;
	int skuid;

	id0 = gpio_get_value(sku0_gpio);
	id1 = gpio_get_value(sku1_gpio);
	id2 = gpio_get_value(sku2_gpio);

	skuid = (id2 << 2) + (id1 << 1) + id0;

	seq_printf(m, "%d\n", skuid);
	return 0;
}

static int info_open(struct inode *inode, struct file *file)
{
	return single_open(file, info_show, NULL);
}

static int ver_open(struct inode *inode, struct file *file)
{
	return single_open(file, ver_show, NULL);
}

static int sku_open(struct inode *inode, struct file *file)
{
	return single_open(file, sku_show, NULL);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops boardinfo_ops = {
	.proc_open = info_open,
	.proc_read = seq_read,
};

static const struct proc_ops boardver_ops = {
	.proc_open = ver_open,
	.proc_read = seq_read,
};

static const struct proc_ops boardsku_ops = {
	.proc_open = sku_open,
	.proc_read = seq_read,
};
#else
static struct file_operations boardinfo_ops = {
	.owner	= THIS_MODULE,
	.open	= info_open,
	.read	= seq_read,
};

static struct file_operations boardver_ops = {
	.owner	= THIS_MODULE,
	.open	= ver_open,
	.read	= seq_read,
};

static struct file_operations boardsku_ops = {
	.owner  = THIS_MODULE,
	.open   = sku_open,
	.read   = seq_read,
};
#endif

static int gpio_hwid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	struct proc_dir_entry* file;

	if (device_property_read_string(dev, "boardinfo", &boardinfo))
		boardinfo = "unknow";

	id0_gpio = of_get_named_gpio(dev->of_node, "id0-gpios", 0);
	if (!gpio_is_valid(id0_gpio)) {
		printk("No id0-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, id0_gpio, GPIOF_DIR_IN, "GPIO_ID0");
		if (ret < 0) {
			printk("Failed to request ID0 gpio: %d\n", ret);
			return ret;
		}
	}

	id1_gpio = of_get_named_gpio(dev->of_node, "id1-gpios", 0);
	if (!gpio_is_valid(id1_gpio)) {
		printk("No id1-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, id1_gpio, GPIOF_DIR_IN, "GPIO_ID1");
		if (ret < 0) {
			printk("Failed to request ID1 gpio: %d\n", ret);
			return ret;
		}
	}

	sku0_gpio = of_get_named_gpio(dev->of_node, "sku0-gpios", 0);
	if (!gpio_is_valid(sku0_gpio)) {
		printk("No sku0-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, sku0_gpio, GPIOF_DIR_IN, "GPIO_SKU0");
		if (ret < 0) {
			printk("Failed to request SKU0 gpio: %d\n", ret);
			return ret;
		}
	}

	sku1_gpio = of_get_named_gpio(dev->of_node, "sku1-gpios", 0);
	if (!gpio_is_valid(sku1_gpio)) {
		printk("No sku1-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, sku1_gpio, GPIOF_DIR_IN, "GPIO_SKU1");
		if (ret < 0) {
			printk("Failed to request SKU1 gpio: %d\n", ret);
			return ret;
		}
	}

	sku2_gpio = of_get_named_gpio(dev->of_node, "sku2-gpios", 0);
	if (!gpio_is_valid(sku2_gpio)) {
		printk("No sku2-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, sku2_gpio, GPIOF_DIR_IN, "GPIO_SKU2");
		if (ret < 0) {
			printk("Failed to request SKU2 gpio: %d\n", ret);
			return ret;
		}
	}

	printk("boardinfo = %s\n", boardinfo);

	file = proc_create("boardinfo", 0444, NULL, &boardinfo_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("boardver", 0444, NULL, &boardver_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("sku", 0444, NULL, &boardsku_ops);
	if (!file)
		return -ENOMEM;

	return 0;
}

static int gpio_hwid_remove(struct platform_device *pdev)
{
	gpio_free(id0_gpio);
	gpio_free(id1_gpio);
	gpio_free(sku0_gpio);
	gpio_free(sku1_gpio);
	gpio_free(sku2_gpio);

	return 0;
}

/* PE100A return 0, PV100A return 1, IM-A return 2, IM-B return 3 */
int boardinfo_show(void) {
	if (strcmp(boardinfo, "PE100A") == 0)
		return 0;
	else if (strcmp(boardinfo, "PV100A") == 0)
		return 1;
	else if (strcmp(boardinfo, "IM-A") == 0)
		return 2;
	else if (strcmp(boardinfo, "IM-B") == 0)
		return 3;
	else
		return -1;
}
EXPORT_SYMBOL_GPL(boardinfo_show);

static struct platform_driver boardinfo_driver = {
	.probe		= gpio_hwid_probe,
	.remove		= gpio_hwid_remove,
	.driver = {
		.name	= "boardinfo",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_gpio_hwid_match),
#endif
	},
};

module_platform_driver(boardinfo_driver);
