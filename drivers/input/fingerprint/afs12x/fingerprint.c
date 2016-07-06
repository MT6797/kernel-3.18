#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/compat.h>
#include <linux/platform_device.h>



struct pinctrl *ma_finger_pinctrl;
struct pinctrl_state *ma_finger_power_on, *ma_finger_power_off,*ma_finger_reset_high,*ma_finger_reset_low,*ma_finger_spi1_mi_as_spi1_mi,*ma_finger_spi1_mi_as_gpio,
*ma_finger_spi1_mo_as_spi1_mo,*ma_finger_spi1_mo_as_gpio,*ma_finger_spi1_clk_as_spi1_clk,*ma_finger_spi1_clk_as_gpio,*ma_finger_spi1_cs_as_spi1_cs,*ma_finger_spi1_cs_as_gpio,
*ma_finger_int_as_int;

int hct_finger_get_gpio_info(struct platform_device *pdev)
{
	struct device_node *node;
	int ret;
	node = of_find_compatible_node(NULL, NULL, "mediatek,ma_finger");
	printk("node.name %s full name %s",node->name,node->full_name);


		ma_finger_pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(ma_finger_pinctrl)) {
			ret = PTR_ERR(ma_finger_pinctrl);
			dev_err(&pdev->dev, "ma_finger cannot find pinctrl\n");
				return ret;
		}

    printk("[%s] ma_finger_pinctrl+++++++++++++++++\n",pdev->name);

	ma_finger_power_on = pinctrl_lookup_state(ma_finger_pinctrl, "finger_power_en1");
	if (IS_ERR(ma_finger_power_on)) {
		ret = PTR_ERR(ma_finger_power_on);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_power_on!\n");
		return ret;
	}
	ma_finger_power_off = pinctrl_lookup_state(ma_finger_pinctrl, "finger_power_en0");
	if (IS_ERR(ma_finger_power_off)) {
		ret = PTR_ERR(ma_finger_power_off);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_power_off!\n");
		return ret;
	}
	ma_finger_reset_high = pinctrl_lookup_state(ma_finger_pinctrl, "finger_reset_en1");
	if (IS_ERR(ma_finger_reset_high)) {
		ret = PTR_ERR(ma_finger_reset_high);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_reset_high!\n");
		return ret;
	}
	ma_finger_reset_low = pinctrl_lookup_state(ma_finger_pinctrl, "finger_reset_en0");
	if (IS_ERR(ma_finger_reset_low)) {
		ret = PTR_ERR(ma_finger_reset_low);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_reset_low!\n");
		return ret;
	}
	ma_finger_spi1_mi_as_spi1_mi = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_mi_as_spi1_mi");
	if (IS_ERR(ma_finger_spi1_mi_as_spi1_mi)) {
		ret = PTR_ERR(ma_finger_spi1_mi_as_spi1_mi);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_mi_as_spi1_mi!\n");
		return ret;
	}
	ma_finger_spi1_mi_as_gpio = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_mi_as_gpio");
	if (IS_ERR(ma_finger_spi1_mi_as_gpio)) {
		ret = PTR_ERR(ma_finger_spi1_mi_as_gpio);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_mi_as_gpio!\n");
		return ret;
	}
	ma_finger_spi1_mo_as_spi1_mo = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_mo_as_spi1_mo");
	if (IS_ERR(ma_finger_spi1_mo_as_spi1_mo)) {
		ret = PTR_ERR(ma_finger_spi1_mo_as_spi1_mo);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_mo_as_spi1_mo!\n");
		return ret;
	}
	ma_finger_spi1_mo_as_gpio = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_mo_as_gpio");
	if (IS_ERR(ma_finger_spi1_mo_as_gpio)) {
		ret = PTR_ERR(ma_finger_spi1_mo_as_gpio);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_mo_as_gpio!\n");
		return ret;
	}
	ma_finger_spi1_clk_as_spi1_clk = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_clk_as_spi1_clk");
	if (IS_ERR(ma_finger_spi1_clk_as_spi1_clk)) {
		ret = PTR_ERR(ma_finger_spi1_clk_as_spi1_clk);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_clk_as_spi1_clk!\n");
		return ret;
	}
	ma_finger_spi1_clk_as_gpio = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_clk_as_gpio");
	if (IS_ERR(ma_finger_spi1_clk_as_gpio)) {
		ret = PTR_ERR(ma_finger_spi1_clk_as_gpio);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_clk_as_gpio!\n");
		return ret;
	}
	ma_finger_spi1_cs_as_spi1_cs = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_cs_as_spi1_cs");
	if (IS_ERR(ma_finger_spi1_cs_as_spi1_cs)) {
		ret = PTR_ERR(ma_finger_spi1_cs_as_spi1_cs);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_cs_as_spi1_cs!\n");
		return ret;
	}
	ma_finger_spi1_cs_as_gpio = pinctrl_lookup_state(ma_finger_pinctrl, "finger_spi1_cs_as_gpio");
	if (IS_ERR(ma_finger_spi1_cs_as_gpio)) {
		ret = PTR_ERR(ma_finger_spi1_cs_as_gpio);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_cs_as_gpio!\n");
		return ret;
	}
	ma_finger_int_as_int = pinctrl_lookup_state(ma_finger_pinctrl, "finger_int_as_int");
	if (IS_ERR(ma_finger_int_as_int)) {
		ret = PTR_ERR(ma_finger_int_as_int);
		dev_err(&pdev->dev, " Cannot find ma_finger pinctrl ma_finger_spi1_cs_as_gpio!\n");
		return ret;
	}
	printk("ma_finger get gpio info ok--------");
	return 0;
}

int hct_finger_set_power(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_power_off);
		break;
		case 1 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_power_on);
		break;
		}
	return 0;
}

int hct_finger_set_reset(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_reset_low);
		break;
		case 1 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_reset_high);
		break;
		}
	return 0;
}

int hct_finger_set_eint(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			return -1;
		break;
		case 1 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_int_as_int);
		break;
		}
	return 0;
}

int hct_finger_set_spi_mode(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_clk_as_gpio);
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_cs_as_gpio);
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_mi_as_gpio);
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_mo_as_gpio);
		break;
		case 1 : 		
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_clk_as_spi1_clk);
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_cs_as_spi1_cs);
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_mi_as_spi1_mi);
			pinctrl_select_state(ma_finger_pinctrl, ma_finger_spi1_mo_as_spi1_mo);
		break;
		}
	return 0;
}



