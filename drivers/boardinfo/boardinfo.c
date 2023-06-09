#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/version.h>

static const char *boardinfo;
static int pcbid = -1, skuid = -1, iobd = -1;

static int pcb0_gpio = 0, pcb1_gpio = 0, pcb2_gpio = 0;
static int sku0_gpio = 0, sku1_gpio = 0, sku2_gpio = 0, sku3_gpio = 0;
static int iobd0_gpio = 0, iobd1_gpio = 0;

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
	char *boardver;

	switch (pcbid) {
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
	seq_printf(m, "%d\n", skuid);
	return 0;
}

static int iobd_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", iobd);
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

static int iobd_open(struct inode *inode, struct file *file)
{
	return single_open(file, iobd_show, NULL);
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

static const struct proc_ops boardio_ops = {
	.proc_open = iobd_open,
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

static struct file_operations boardio_ops = {
	.owner	= THIS_MODULE,
	.open	= iobd_open,
	.read	= seq_read,
};
#endif

static int gpio_hwid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	struct proc_dir_entry* file;

	int id0, id1, id2, id3;

	if (device_property_read_string(dev, "boardinfo", &boardinfo))
		boardinfo = "unknow";

	pcb0_gpio = of_get_named_gpio(dev->of_node, "pcb0-gpios", 0);
	if (!gpio_is_valid(pcb0_gpio) && (pcb0_gpio == ERR_PTR(-EPROBE_DEFER))) {
		printk("gpio_hwid_probe EPROBE_DEFER\n");
		return -EPROBE_DEFER;
	} else if (!gpio_is_valid(pcb0_gpio)) {
		printk("No pcb0-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, pcb0_gpio, GPIOF_DIR_IN, "GPIO_PCB0");
		if (ret < 0) {
			printk("Failed to request PCB0 gpio: %d\n", ret);
			return ret;
		}
	}

	pcb1_gpio = of_get_named_gpio(dev->of_node, "pcb1-gpios", 0);
	if (!gpio_is_valid(pcb1_gpio)) {
		printk("No pcb1-gpio pin available in gpio-hwid\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, pcb1_gpio, GPIOF_DIR_IN, "GPIO_PCB1");
		if (ret < 0) {
			printk("Failed to request PCB1 gpio: %d\n", ret);
			return ret;
		}
	}

	id0 = gpio_get_value(pcb0_gpio);
	id1 = gpio_get_value(pcb1_gpio);
	id2 = 0;

	if (strcmp(boardinfo, "BLIZZARD") == 0) {
		pcb2_gpio = of_get_named_gpio(dev->of_node, "pcb2-gpios", 0);
		if (!gpio_is_valid(pcb2_gpio)) {
			printk("No pcb2-gpio pin available in gpio-hwid\n");
			return -ENODEV;
		} else {
			ret = devm_gpio_request_one(dev, pcb2_gpio, GPIOF_DIR_IN, "GPIO_PCB2");
			if (ret < 0) {
				printk("Failed to request PCB2 gpio: %d\n", ret);
				return ret;
			}
		}

		id2 = gpio_get_value(pcb2_gpio);
	}

	pcbid = (id2 << 2) + (id1 << 1) + id0;

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

	id0 = gpio_get_value(sku0_gpio);
	id1 = gpio_get_value(sku1_gpio);
	id2 = gpio_get_value(sku2_gpio);
	id3 = 0;

	if (strcmp(boardinfo, "PE100A") == 0 || strcmp(boardinfo, "IMX8P-IM-A") == 0) {
		sku3_gpio = of_get_named_gpio(dev->of_node, "sku3-gpios", 0);
		if (!gpio_is_valid(sku3_gpio)) {
			printk("No sku3-gpio pin available in gpio-hwid\n");
			return -ENODEV;
		} else {
			ret = devm_gpio_request_one(dev, sku3_gpio, GPIOF_DIR_IN, "GPIO_SKU3");
			if (ret < 0) {
				printk("Failed to request SKU3 gpio: %d\n", ret);
				return ret;
			}
		}

		id3 = gpio_get_value(sku3_gpio);

		if (id3 == 1) {
			if (strcmp(boardinfo, "PE100A") == 0)
				boardinfo = "PE100A V3.0";
			else if (strcmp(boardinfo, "IMX8P-IM-A") == 0)
				boardinfo = "IMX8P-IM-A R2.0";
		}
	}

	skuid = (id3 << 3) + (id2 << 2) + (id1 << 1) + id0;

	if (strcmp(boardinfo, "PE100A") == 0) {
		iobd0_gpio = of_get_named_gpio(dev->of_node, "iobd0-gpios", 0);
		if (!gpio_is_valid(iobd0_gpio)) {
			printk("No iobd0-gpio pin available in gpio-hwid\n");
			return -ENODEV;
		} else {
			ret = devm_gpio_request_one(dev, iobd0_gpio, GPIOF_DIR_IN, "GPIO_IOBD0");
			if (ret < 0) {
				printk("Failed to request IOBD0 gpio: %d\n", ret);
				return ret;
			}
		}

		iobd1_gpio = of_get_named_gpio(dev->of_node, "iobd1-gpios", 0);
		if (!gpio_is_valid(iobd1_gpio)) {
			printk("No iobd1-gpio pin available in gpio-hwid\n");
			return -ENODEV;
		} else {
			ret = devm_gpio_request_one(dev, iobd1_gpio, GPIOF_DIR_IN, "GPIO_IOBD1");
			if (ret < 0) {
				printk("Failed to request IOBD1 gpio: %d\n", ret);
				return ret;
			}
		}

		id0 = gpio_get_value(iobd0_gpio);
		id1 = gpio_get_value(iobd1_gpio);

		iobd = (id1 << 1) + id0;
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

	if (strcmp(boardinfo, "PE100A") == 0) {
		file = proc_create("ioboard", 0444, NULL, &boardio_ops);
		if (!file)
			return -ENOMEM;
	}

	return 0;
}

static int gpio_hwid_remove(struct platform_device *pdev)
{
	gpio_free(pcb0_gpio);
	gpio_free(pcb1_gpio);
	gpio_free(sku0_gpio);
	gpio_free(sku1_gpio);
	gpio_free(sku2_gpio);

	if (strcmp(boardinfo, "BLIZZARD") == 0)
		gpio_free(pcb2_gpio);

	if (strcmp(boardinfo, "PE100A") == 0 || strcmp(boardinfo, "IMX8P-IM-A") == 0)
		gpio_free(sku3_gpio);

	if (strcmp(boardinfo, "PE100A") == 0) {
		gpio_free(iobd0_gpio);
		gpio_free(iobd1_gpio);
	}

	return 0;
}

/* PE100A return 0, PV100A return 1, IMX8P-IM-A return 2, IMX8P-IM-B return 3 */
int boardinfo_show(void) {
	if (strcmp(boardinfo, "PE100A") == 0)
		return 0;
	else if (strcmp(boardinfo, "PV100A") == 0)
		return 1;
	else if (strcmp(boardinfo, "IMX8P-IM-A") == 0)
		return 2;
	else if (strcmp(boardinfo, "BLIZZARD") == 0 || strcmp(boardinfo, "BLIZZARD-EVT") == 0)
		return 3;
	else
		return -1;
}
EXPORT_SYMBOL_GPL(boardinfo_show);

int get_boardver(void)
{
	return pcbid;
}
EXPORT_SYMBOL_GPL(get_boardver);

int get_boardsku(void)
{
	return skuid;
}
EXPORT_SYMBOL_GPL(get_boardsku);

int get_boardio(void)
{
	return iobd;
}
EXPORT_SYMBOL_GPL(get_boardio);

static struct platform_driver boardinfo_driver = {
	.driver = {
		.name	= "boardinfo",
		.of_match_table = of_match_ptr(of_gpio_hwid_match),
	},
	.probe 		= gpio_hwid_probe,
	.remove 	= gpio_hwid_remove,
};

module_platform_driver(boardinfo_driver);
