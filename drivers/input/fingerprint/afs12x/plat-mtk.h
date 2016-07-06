/* MicroArray Fingerprint
 * plat-mtk.h
 * date: 2015-08-20
 * version: v2.0
 * Author: czl
 */

#ifndef PLAT_MTK_H
#define PLAT_MTK_H

#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/of_irq.h>
#include "mt_spi.h"
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif  
#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/cma.h>
#include <linux/debugfs.h>
#include <linux/stat.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/memblock.h>
#include <asm/page.h>
#include <asm-generic/memory_model.h>
#include <mt-plat/mt_lpae.h> /* DMA */
#include <mt-plat/aee.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include "mt_spi_hal.h"
#include <mt_spi.h>
#include <linux/gpio.h>



extern int hct_finger_get_gpio_info(struct platform_device *pdev);
extern int hct_finger_set_power(int cmd);
extern int hct_finger_set_reset(int cmd);
extern int hct_finger_set_spi_mode(int cmd);
extern int hct_finger_set_eint(int cmd);
extern int mas_probe(struct spi_device *spi);
extern int mas_remove(struct spi_device *spi);
extern int mas_plat_probe(struct platform_device *pdev);
extern int mas_plat_remove(struct platform_device *pdev);

#define GPIO_FPS_RESET_PIN	 256//90//(GPIO90 | 0x80000000)

#define P_GPIO_FPS_PWR_PIN  140//89
#ifdef CONFIG_OF
static const struct of_device_id fp_of_match[] = {
	{.compatible = "mediatek, fp-eint",},
	{},
};
#endif

#ifdef CONFIG_OF
static struct of_device_id sof_match[] = {
	{ .compatible = "mediatek,hct_finger", },
	{}
};
MODULE_DEVICE_TABLE(of, sof_match);
#endif

struct spi_device_id sdev_id = {"madev", 0};
struct spi_driver sdrv = {
	.driver = {
		.name =	"madev",
		.owner = THIS_MODULE,
	},
	.probe = mas_probe,
	.remove = mas_remove,
	.id_table = &sdev_id,
};

static struct platform_driver spdrv = {
	.probe	  = mas_plat_probe,
	.remove	 = mas_plat_remove,
	.driver = {
		.name  = "madev",
		.owner = THIS_MODULE,			
#ifdef CONFIG_OF
		.of_match_table = sof_match,
#endif
	}
};

struct mt_chip_conf smt_conf = {
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 10, // 10--6m 15--4m 20--3m 30--2m [ 60--1m 120--0.5m  300--0.2m]
	.low_time = 10,
	.cs_idletime = 10,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 5,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

struct spi_board_info smt_info[] __initdata = {
	[0] = {
		.modalias = "madev",
		.max_speed_hz = SPI_SPEED,
		.bus_num = 1,//0,
		.chip_select = 1,//0,
		.mode = SPI_MODE_0,
		.controller_data = &smt_conf
	}
};

void plat_set_gpio(struct platform_device *plat) {	
	int ret;
	ret = gpio_request(GPIO_FPS_RESET_PIN, "ars_rst");
	if (ret)
		printk("[ars3022]  : gpio_request (%d)fail\n", GPIO_FPS_RESET_PIN);

	ret = gpio_direction_output(GPIO_FPS_RESET_PIN, 0);
	if (ret)
		printk("[ars3022]  : gpio_direction_output (%d)fail\n",GPIO_FPS_RESET_PIN);

	gpio_set_value(GPIO_FPS_RESET_PIN, 1);
	msleep(20);

	ret = gpio_request(P_GPIO_FPS_PWR_PIN, "ars_pwr");
	if (ret)
		printk("[ars3022]  : gpio_request (%d)fail\n", P_GPIO_FPS_PWR_PIN);

	ret = gpio_direction_output(P_GPIO_FPS_PWR_PIN, 0);
	if (ret)
		printk("[ars3022]gpio_direction_output (%d)fail\n",P_GPIO_FPS_PWR_PIN);

	gpio_set_value(P_GPIO_FPS_PWR_PIN, 1);
	/*hct_finger_get_gpio_info(plat);
	hct_finger_set_power(1);
	hct_finger_set_reset(1);
	hct_finger_set_spi_mode(1);
	hct_finger_set_eint(1);*/
}

int plat_register_driver(void) {
	int ret;

	printd("%s: start\n", __func__);
	
	spi_register_board_info(smt_info, ARRAY_SIZE(smt_info));
	ret = spi_register_driver(&sdrv);
	if(ret==0) {
		ret = platform_driver_register(&spdrv);
	}

	printd("%s: end.\n", __func__);

	return ret;
}

void plat_unregister_driver(void) {
	spi_unregister_driver(&sdrv);
}

/* MTK电源开关
 * @power 1:开，0：关
 * @return 0成功，-1失败
 */
int plat_power(int power) {
	int ret = 0;

	// 仅PMU管理电源
//	if(power) {
//		ret = hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_3000, MODE_NAME);
//	} else {
//		ret = hwPowerDown(MT6323_POWER_LDO_VGP1, MODE_NAME);
//	}
	return (ret==1)? 0: -1;
}

void plat_set_speed(int speed) {
	int val, num;

	num = speed/1000000;
	num = num>0? num: 1;
	val = 60/num;
	smt_conf.high_time = val;
	smt_conf.low_time = val;
	smt_conf.cs_idletime = val;	
}

void plat_tansfer(struct spi_device *spi, int len) {
	static int mode = -1;
	int tmp = len>32? DMA_TRANSFER: FIFO_TRANSFER;
	//int tmp = FIFO_TRANSFER;

	//printd("%s: start\n", __func__);

	if(tmp!=mode) {
		struct mt_chip_conf *conf = (struct mt_chip_conf *) spi->controller_data;
		conf->com_mod = tmp;
		spi_setup(spi);
		mode = tmp;
	}

	//printd("%s: end.\n", __func__);
}

void plat_enable_irq(struct spi_device *spi, u8 flag) {
	static int state = -1;

	//printd("%s: start\n", __func__);

	if (state != flag) {
		if (flag) {
			printd("%s: enable_irq.\n", __func__);
			enable_irq(spi->irq);
		} else {
			printd("%s: disable_irq.\n", __func__);
			disable_irq_nosync(spi->irq);
		}
		state = flag;
	} 

	//printd("%s: end.\n", __func__);
}

int plat_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
        const char *name, void *dev) {	
	struct device_node *node = NULL;
	//const char*tname = "mediatek,fp-eint";
	//node = of_find_compatible_node(NULL,NULL, "mediatek,fp-eint");
	node = of_find_matching_node(node, fp_of_match);
	irq = irq_of_parse_and_map(node, 0);	
	printk("eeeeeeeeeeeeeeeee plat_request_irq irq = %d\n",irq);
	return request_irq(irq, handler, flags, "fp-eint", dev);
}

#endif



