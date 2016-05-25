/****************************************************************************************
File: hall.c
History:
Aka.jiang    2013-12-14    Init    aka.jiang@hotmail.com
****************************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


#define HALL_NAME "cover"

//#define HALL_DEBUG

#define HALL_DEBUG_MASK_IRQ 1
#define HALL_DEBUG_MASK_SYS (1 << 1)


#define HALL_DEBUG_ON

#ifdef HALL_DEBUG_ON
    #define HALL_DEBUG_TAG                  "[HALL] "
    #define HALL_DEBUG_FUN(f)               pr_err(HALL_DEBUG_TAG"%s\n", __func__)
    #define HALL_DEBUG_ERR(fmt, args...)    pr_err(HALL_DEBUG_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
    #define HALL_DEBUG_LOG(fmt, args...)    pr_err(HALL_DEBUG_TAG fmt, ##args)
#else
    #define HALL_DEBUG_TAG
    #define HALL_DEBUG_FUN(f)
    #define HALL_DEBUG_ERR(fmt, args...)
    #define HALL_DEBUG_LOG(fmt, args...)
#endif 

struct hall_info {
	struct input_dev	*idev;
        struct device_node 	*irq_node;
	struct work_struct	work;
	unsigned int		irq;
	unsigned int		irq_gpio;
	unsigned int		sw_code;
	const char *name;
	unsigned int debug_mask;
};


static void hall_work(struct work_struct *work)
{
    struct hall_info *info = container_of(work, struct hall_info, work);
    int state;

    state = !!gpio_get_value(info->irq_gpio);

    HALL_DEBUG_LOG("state = %d", state);
    input_report_switch(info->idev, info->sw_code, state);
    input_sync(info->idev);
    enable_irq(info->irq);
}

static irqreturn_t hall_irq_handler(int irq, void *data)
{

    struct hall_info *info = data;

    disable_irq_nosync(info->irq);
    schedule_work(&info->work);

    return IRQ_HANDLED;
}


static ssize_t hall_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct hall_info *info = input_get_drvdata(input);
	int state;

	state = !!gpio_get_value(info->irq_gpio);

	return sprintf(buf, "%d\n", state);
}


static DEVICE_ATTR(status, S_IRUGO,
		hall_status_show, NULL);

static struct attribute *hall_attributes[] = {
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group hall_attribute_group = {
	.attrs = hall_attributes
};

int hall_setup_eint(struct hall_info *info)
{
    int ret = 0;
    u32 ints[2] = {0};

    info->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, hall-eint");
    of_property_read_u32_array(info->irq_node, "debounce", ints, ARRAY_SIZE(ints));
    info->irq_gpio = ints[0];
		
    info->irq = irq_of_parse_and_map(info->irq_node, 0);
    HALL_DEBUG_LOG("irq = %d, irq_gpio = %d\n", info->irq, info->irq_gpio);
    if (!info->irq) {
        HALL_DEBUG_ERR("irq_of_parse_and_map fail!!\n");
        return -EINVAL;
    }

    ret = request_irq(info->irq, hall_irq_handler, 
		//IRQF_TRIGGER_LOW | IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		info->name, info);

    if (ret != 0) {
        HALL_DEBUG_ERR("IRQ LINE NOT AVAILABLE!!\n");
        return -EINVAL;
    }
//    enable_irq(info->irq);

    return ret;
}


static int hall_probe(struct platform_device *pdev)
{
    struct hall_info *info;
    struct input_dev *input;
    char pyhs_str[50];
    int ret = 0;
    
    HALL_DEBUG_LOG("Enter!");
    info = kzalloc(sizeof(struct hall_info), GFP_KERNEL);
    input = input_allocate_device();
    if (!info || !input) {
        ret = -ENOMEM;
        goto err_free_mem;
    }

    info->idev = input;
    info->sw_code = 12;
    info->name = HALL_NAME;
    input->name = info->name;
    
    sprintf(pyhs_str, "%s/input0", input->name);
    input->phys = pyhs_str;
    input_set_capability(input, EV_SW, info->sw_code);

    HALL_DEBUG_LOG("sw_code = 0x%x\n", info->sw_code);
    HALL_DEBUG_LOG("name = %s", info->name);
    info->debug_mask = 0;

    ret = hall_setup_eint(info);
    if (ret < 0) {
        goto err_free_mem;
    }

    ret = input_register_device(info->idev);
    if (ret) {
        HALL_DEBUG_ERR("Can't register input device: %d\n", ret);
        goto err_free_mem;
    }

    input_set_drvdata(info->idev, info);
    platform_set_drvdata(pdev, info);

    ret = sysfs_create_group(&info->idev->dev.kobj, &hall_attribute_group);
    if (ret < 0){
        HALL_DEBUG_ERR("Can't create sysfs: %d\n", ret);
        goto err_sysfs;
    }

    INIT_WORK(&info->work, hall_work);
    device_init_wakeup(&pdev->dev, 1);

//    g_num = 1;
    HALL_DEBUG_LOG("OK!");
    return 0;


err_sysfs:
    input_unregister_device(info->idev);
err_free_mem:
    input_free_device(input);
    kfree(info);

    HALL_DEBUG_LOG("Fail ret = %d!\n", ret);
    return ret;

}

static struct of_device_id hall_of_match[] = {
	{ .compatible = "hall-cover", },
	{ },
};
MODULE_DEVICE_TABLE(of, hall_of_match);

static int hall_remove(struct platform_device *pdev)
{
	struct hall_info *info = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	input_unregister_device(info->idev);
	cancel_work_sync(&info->work);
	kfree(info);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hall_suspend(struct device *dev)
{

	return 0;
}

static int hall_resume(struct device *dev)
{

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(hall_pm_ops, hall_suspend, hall_resume);

static struct platform_driver hall_driver = {
	.probe		= hall_probe,
	.remove		= hall_remove,
	.driver		= {
		.name	= "hall_driver",
		.owner	= THIS_MODULE,
		.pm	= &hall_pm_ops,
        .of_match_table = hall_of_match,
	},
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}

late_initcall(hall_init);
module_exit(hall_exit);

MODULE_DESCRIPTION("Prowave Hall driver");
MODULE_AUTHOR("Aka.Jiang <aka.jiang@hotmail.com>");
MODULE_LICENSE("GPL");

