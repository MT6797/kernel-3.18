/* MicroArray Fingerprint
 * plat-general.h
 * date: 2015-08-15
 * version: v2.1
 * Author: czl
 */

#ifndef PLAT_GENERAL_H
#define PLAT_GENERAL_H

extern int mas_probe(struct spi_device *spi);
extern int mas_remove(struct spi_device *spi);

struct spi_driver sdrv = {
	.driver = {
		.name = "madev",
		.owner = THIS_MODULE,
	},
	.probe = mas_probe,
	.remove = mas_remove,
};

void plat_set_speed(int speed) {
	
}

int plat_register_driver(void) {
	return spi_register_driver(&sdrv);
}

void plat_unregister_driver(void) {
	spi_unregister_driver(&sdrv);
}

/* 电源开关
 * @power 1:开，0：关
 * @return 0成功，-1失败
 */
int plat_power(int power) {
	return 0;
}

int plat_tansfer(struct spi_device *spi, int len) {
	return 0;
}

void plat_enable_irq(struct spi_device *spi, int flag) {
	static int state = -1;

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
}

int plat_request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
	const char *name, void *dev) {

	return request_irq(irq, handler, flags, name, dev);
}

#endif



