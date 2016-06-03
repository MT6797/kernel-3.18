/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/******************************************************************************\
|*                                                                            *|
|*  et320.spi.c                                                               *|
|*  Date: 2016/02/25                                                          *|
|*  Version: 0.9.0.0                                                          *|
|*  Revise Date:  2016/02/25                                                  *|                                                           
|*  Copyright (C) 2007-2016 Egis Technology Inc.                              *|
|*                                                                            *|
\******************************************************************************/


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


#include "et320.h"
///////////////////////
#ifndef SPI_PACKET_SIZE
#define SPI_PACKET_SIZE 	 0x400
#define SPI_PACKET_COUNT    0x100
#define SPI_FIFO_SIZE 32
#endif
////////////////////////
#ifdef FP_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...)	pr_err(fmt, ## args)
#else
/* Don't do anything in release builds */
#define DEBUG_PRINT(fmt, args...)
#endif

#define LGE_TEST 1
/* MTK mt6975_evb reset & interrupt pin */
#define GPIO_FPS_RESET_PIN	 256//90//(GPIO90 | 0x80000000)
//#define CUST_EINT_FPS_EINT_NUM	CUST_EINT_FP_NUM	//5
//#define TRIGGER_FPS_GPIO_PIN	GPIO_FP_EINT_PIN	//(GPIO5 | 0x80000000)

#define P_GPIO_FPS_PWR_PIN  140//89

#define FPC_BTP_SPI_CLOCK_SPEED			8*1000*1000
#define MASS_READ_TRANSMITION_SIZE 2304

#define EGIS_MASS_READ_SEGMENT_COUNT 10

unsigned int fp_detect_irq = 0;
#ifdef CONFIG_OF
static const struct of_device_id fp_of_match[] = {
	{.compatible = "mediatek, fp-eint",},
	{},
};
#endif

#define EDGE_TRIGGER_FALLING    0x0
#define	EDGE_TRIGGER_RAISING    0x1
#define	LEVEL_TRIGGER_LOW       0x2
#define	LEVEL_TRIGGER_HIGH      0x3

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 4096;
struct device *esfp0_dev;
/*-----------------MTK spi configure-----------------*/
static struct mt_chip_conf spi_conf =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,
	.low_time = 4,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
#ifdef TRANSFER_MODE_DMA
	.com_mod = DMA_TRANSFER,
#else
	.com_mod = FIFO_TRANSFER,
#endif
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
/*
static struct mt_chip_conf spi_conf_fifo =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,
	.low_time = 4,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
*/
static struct mt_chip_conf spi_conf_dma =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,
	.low_time = 4,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

/*--------------------------- Data Transfer -----------------------------*/
#if 1
#define USE_SPI1_4GB_TEST (1)

#if USE_SPI1_4GB_TEST
static dma_addr_t SpiDmaBufTx_pa;
static dma_addr_t SpiDmaBufRx_pa;
static char *spi_tx_local_buf;
static char *spi_rx_local_buf;

static int reserve_memory_spi_fn(struct reserved_mem *rmem)
{
	printk(" 11111name: %s, base: 0x%llx, size: 0x%llx\n", rmem->name,
			(unsigned long long)rmem->base, (unsigned long long)rmem->size);
	BUG_ON(rmem->size < 0x8000);
	SpiDmaBufTx_pa = rmem->base;
	SpiDmaBufRx_pa = rmem->base+0x4000;
	return 0;
}
RESERVEDMEM_OF_DECLARE(reserve_memory_test, "mediatek,spi-reserve-memory", reserve_memory_spi_fn);

#endif

static int spi_setup_xfer(struct spi_transfer *xfer)
{
	//u8 tx_buffer = 0x01;
		/* map physical addr to virtual addr */
		if (NULL == spi_tx_local_buf) {
			spi_tx_local_buf = (char *)ioremap_nocache(SpiDmaBufTx_pa, 0x4000);
			if (!spi_tx_local_buf) {
				DEBUG_PRINT("SPI Failed to dma_alloc_coherent()\n");
				return -ENOMEM;
			}
		}
		if (NULL == spi_rx_local_buf) {
			spi_rx_local_buf = (char *)ioremap_nocache(SpiDmaBufRx_pa, 0x4000);
			if (!spi_rx_local_buf) {
				DEBUG_PRINT("SPI Failed to dma_alloc_coherent()\n");
				return -ENOMEM;
			}
		}
		xfer->tx_buf = spi_tx_local_buf;
		xfer->rx_buf = spi_rx_local_buf;
		xfer->tx_dma = SpiDmaBufTx_pa;
		xfer->rx_dma = SpiDmaBufRx_pa;
	return 0;

}
#endif
//static struct spi_transfer msg_xfer[EGIS_MASS_READ_SEGMENT_COUNT];
int fp_mass_read(struct fp_data *fp, u8 addr, u8 *buf, int read_len)
{
	ssize_t status;
	struct spi_device *spi;
	struct spi_message m;
	//u8 write_addr[2];
	u8 write_addr[] = {FP_WRITE_ADDRESS, addr, 0x00, 0x00};
	u8 read_addr[] = {FP_READ_DATA, 0x00, 0x00, 0x00};
	/* Set start address */
	u8 *read_data = buf;
#if 1
	struct spi_transfer t_set_addr = {
		.tx_buf = NULL,
		.len = 2,
	};
#endif

	/* Write and read data in one data query section */
	struct spi_transfer t_read_data = {
		.tx_buf = NULL,
		.rx_buf = NULL,
		.len = read_len,
	};

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);

	spi = fp->spi;

	spi_message_init(&m);
	spi_setup_xfer(&t_set_addr);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_addr);
	spi_message_add_tail(&t_set_addr, &m);
	status = spi_sync(spi, &m);

	//spi_message_add_tail(&t_set_addr, &m);
	spi_message_init(&m);
	spi_setup_xfer(&t_read_data);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, read_addr);
	spi_message_add_tail(&t_read_data, &m);
	status = spi_sync(spi, &m);

	memcpy(read_data, spi_rx_local_buf, read_len);

	return status;
}

/*
 * Read io register
 */
int fp_io_read_register(struct fp_data *fp, u8 *addr, u8 *buf)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;
	int read_len = 1;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 read_value[] = {FP_READ_DATA, 0x00};
	u8 val, addrval;
	u8 read_addr[] = {FP_READ_DATA, 0x00, 0x00, 0x00};

	u8 result[] = {0xFF, 0xFF};
	struct spi_transfer t_set_addr = {
	
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	if (copy_from_user(&addrval, (const u8 __user *) (uintptr_t) addr
		, read_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}

	DEBUG_PRINT("%s read_len = %d\n", __func__, read_len);
	write_addr[1] = addrval;
	printk("wwwwwwwwwww %s",__func__);

	spi = fp->spi;
	spi_message_init(&m);
	spi_setup_xfer(&t_set_addr);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_addr);
	spi_message_add_tail(&t_set_addr, &m);
	spi_sync(spi, &m);

	//spi_message_add_tail(&t_set_addr, &m);
	spi_message_init(&m);
	spi_setup_xfer(&t);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, read_addr);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);

	memcpy(result, spi_rx_local_buf, 2);

	if (status < 0) {
		pr_err("%s read data error status = %ld\n"
				, __func__, status);
		return status;
	}

	val = result[1];

	DEBUG_PRINT("%s Read add_val = %x buf = 0x%x\n", __func__,
			addrval, val);

	if (copy_to_user((u8 __user *) (uintptr_t) buf, &val, read_len)) {
		pr_err("%s buffer copy_to_user fail status", __func__);
		status = -EFAULT;
		return status;
	}

	return status;
}

/*
 * Write data to register
 */
int fp_io_write_register(struct fp_data *fp, u8 *buf)
{
	ssize_t status = 0;
	struct spi_device *spi;
	int write_len = 2;
	struct spi_message m;

	u8 write_addr[] = {FP_WRITE_ADDRESS, 0x00};
	u8 write_value[] = {FP_WRITE_DATA, 0x00};
	u8 val[2];

	struct spi_transfer t1 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_value,
		.len = 2,
	};

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) buf
		, write_len)) {
		pr_err("%s buffer copy_from_user fail", __func__);
		status = -EFAULT;
		return status;
	}
	spi = fp->spi;
	spi_message_init(&m);
	write_addr[1] = val[0];
	/*Set value*/
	write_value[1] = val[1];
	spi_setup_xfer(&t1);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_addr);
	spi_message_add_tail(&t1, &m);
	spi_sync(spi, &m);

	//spi_message_add_tail(&t_set_addr, &m);
	spi_message_init(&m);
	spi_setup_xfer(&t2);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_value);
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);

	if (status < 0) {
		pr_err("%s write data error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_read_register(struct fp_data *fp, u8 addr, u8 *buf)
{
	ssize_t status;
	struct spi_device *spi;
	struct spi_message m;

	/*Set address*/
	u8 write_addr[] = {FP_WRITE_ADDRESS, addr};
	u8 read_addr[] = {FP_READ_DATA, 0x00, 0x00, 0x00};
	u8 read_value[] = {FP_READ_DATA, 0x00};
	u8 result[] = {0xFF, 0xFF};

	struct spi_transfer t1 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.speed_hz = FPC_BTP_SPI_CLOCK_SPEED,
		.tx_buf = read_value,
		.rx_buf = result,
		.len = 2,
	};

	spi = fp->spi;
	spi_message_init(&m);
	spi_setup_xfer(&t1);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_addr);
	spi_message_add_tail(&t1, &m);
	spi_sync(spi, &m);

	//spi_message_add_tail(&t_set_addr, &m);
	spi_message_init(&m);
	spi_setup_xfer(&t2);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, read_addr);
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);

	memcpy(result, spi_rx_local_buf, 2);
	
	if (status == 0) {
		*buf = result[1];
		DEBUG_PRINT("fp_read_register address = %x result = %x %x\n"
					, addr, result[0], result[1]);
	} else
		pr_err("%s read data error status = %ld\n"
				, __func__, status);

	return status;
}

int fp_set_single_register(struct fp_data *fp, u8 addr, u8 val)
{
	ssize_t status = 0;
	struct spi_device *spi;
	struct spi_message m;

	u8 write_addr[] = {FP_WRITE_ADDRESS, addr};
	u8 write_value[] = {FP_WRITE_DATA, val};

	struct spi_transfer t1 = {
		.tx_buf = write_addr,
		.len = 2,
	};
	struct spi_transfer t2 = {
		.tx_buf = write_value,
		.len = 2,
	};

	spi = fp->spi;
	spi_setup_xfer(&t1);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_addr);
	spi_message_add_tail(&t1, &m);
	spi_sync(spi, &m);

	//spi_message_add_tail(&t_set_addr, &m);
	spi_message_init(&m);
	spi_setup_xfer(&t2);
	memset(spi_tx_local_buf, 0, 4);
	strcpy(spi_tx_local_buf, write_value);
	spi_message_add_tail(&t2, &m);
	status = spi_sync(spi, &m);

	status = spi_sync(spi, &m);
	if (status < 0) {
		pr_err("%s write data error status = %ld\n",
				__func__, status);
		return status;
	}

	return status;
}

int fp_io_get_one_image(
	struct fp_data *fp,
	u8 *buf,
	u8 *image_buf
	)
{
	uint8_t read_val,
			*tx_buf = (uint8_t *)buf,
			*work_buf = NULL,
			val[2];
	int32_t status;
	uint32_t frame_not_ready_count = 0, read_count;
	u32 spi_transfer_len;

	DEBUG_PRINT("%s\n", __func__);

	if (copy_from_user(val, (const u8 __user *) (uintptr_t) tx_buf, 2)) {
		pr_err("%s buffer copy_from_user fail\n", __func__);
		status = -EFAULT;
		goto end;
	}

	/* total pixel , width * hight */
	read_count = val[0] * val[1];

	while (1) {
		status = fp_read_register
				(fp, FSTATUS_FP_ADDR, &read_val);
		if (status < 0)
			goto end;

		if (read_val & FRAME_READY_MASK)
			break;

		if (frame_not_ready_count >= 250) {
			pr_err("frame_not_ready_count = %d\n",
					frame_not_ready_count);
			break;
		}
		frame_not_ready_count++;
	}

	spi_transfer_len = (read_count + 3) > SPI_PACKET_SIZE ? (read_count + 3 + SPI_PACKET_SIZE - 1) / SPI_PACKET_SIZE * SPI_PACKET_SIZE : read_count + 3;

	
	work_buf = kzalloc(spi_transfer_len, GFP_KERNEL);
	if (work_buf == NULL) {
		status = -ENOMEM;
		goto end;
	}
	status = fp_mass_read(fp, FDATA_FP_ADDR, work_buf, spi_transfer_len);
	if (status < 0) {
		pr_err("%s call fp_mass_read error status = %d\n"
				, __func__, status);
		goto end;
	}

	if (copy_to_user((u8 __user *) (uintptr_t) image_buf,
		work_buf+3, read_count)) {
		pr_err("buffer copy_to_user fail status = %d\n", status);
		status = -EFAULT;
	}
end:
	kfree(work_buf);
	return status;
}

/* ------------------------------ Interrupt -----------------------------*/

/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD  10
#define FP_DETECTION_THRESHOLD	10

struct interrupt_desc fps_ints;
/*
 * FPS interrupt table
 */
//struct interrupt_desc fps_ints={TRIGGER_FPS_GPIO_PIN , 0, "BUT0" , 0};
// struct interrupt_desc fps_ints[] = {
// #if LGE_TEST
// 	{GPIO8 , 0, "BUT0" , 0} /* TINY4412CON15 XEINT12 pin */
// #else
// 	{GPIO8 , 0, "BUT0" , 0} /* TINY4412CON15 XEINT12 pin */
// #endif
// };
//int fps_ints_size = ARRAY_SIZE(fps_ints);

static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);

/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */

void interrupt_timer_routine(
	unsigned long _data
)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;
	pr_info("FPS interrupt count = %d" , bdata->int_count);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		pr_info("FPS triggered !!!!!!!\n");
	} else
		pr_info("FPS not triggered !!!!!!!\n");
	//bdata->int_count = 0;
		 fps_ints.int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*
 *	FUNCTION NAME.
 *		fp_interrupt_edge
 *		fp_interrupt_level
 *
 *	FUNCTIONAL DESCRIPTION.
 *		finger print interrupt callback routine
 *		fp_interrupt_edge : for edge trigger
 *		fp_interrupt_level : for level trigger
 *
 */

static irqreturn_t fp_interrupt_edge(int irq, void *dev_id)
{
	if (!fps_ints.int_count)
		mod_timer(&fps_ints.timer,jiffies + msecs_to_jiffies(fps_ints.detect_period));
	fps_ints.int_count++;
	return IRQ_HANDLED;
}
/*
static irqreturn_t fp_interrupt_level(int irq, void *dev_id)
{
	fps_ints.finger_on = 1;
	//mt_eint_mask(CUST_EINT_FPS_EINT_NUM);
	wake_up_interruptible(&interrupt_waitq);
	disable_irq_nosync(fp_detect_irq);
	return IRQ_HANDLED;
}*/

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		int_mode - determine trigger mode
 *			EDGE_TRIGGER_FALLING    0x0
 *			EDGE_TRIGGER_RAISING    0x1
 *			LEVEL_TRIGGER_LOW       0x2
 *			LEVEL_TRIGGER_HIGH      0x3
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Init(int int_mode, int detect_period, int detect_threshold)
{
        //int i, irq;
        int ret = 0;
        struct device_node *node = NULL;
 
        pr_info("FP %s int_mode = %d detect_period = %d detect_threshold = %d\n",
                                __func__,
                                int_mode,
                                detect_period,
                                detect_threshold);
 
        if((detect_period>0) && (detect_threshold>0)){
                fps_ints.detect_period = detect_period;
                fps_ints.detect_threshold = detect_threshold;
        }
        else{
                fps_ints.detect_period = detect_period;
                fps_ints.detect_threshold = detect_threshold;
        }
        fps_ints.int_count = 0;
        fps_ints.finger_on = 0;
 
        if(fps_ints.drdy_irq_flag == DRDY_IRQ_DISABLE){
 
        node = of_find_matching_node(node, fp_of_match);
 
        if (node){
                fp_detect_irq = irq_of_parse_and_map(node, 0);
                //printk("fp_irq number %d\n", fp_detect_irq);
                        ret = request_irq(fp_detect_irq,
                                fp_interrupt_edge, IRQF_TRIGGER_RISING,
                                "fp_detect-eint", NULL);
                if (ret > 0){
                        printk("fingerprint request_irq IRQ LINE NOT AVAILABLE!.");
                }
        }else{
                printk("fingerprint request_irq can not find fp eint device node!.");
        }
 
        enable_irq(fp_detect_irq);
        fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
        }
 
        return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(void)
{

	if(fps_ints.drdy_irq_flag == DRDY_IRQ_ENABLE){
		//mt_eint_mask(CUST_EINT_FPS_EINT_NUM);
		disable_irq_nosync(fp_detect_irq);
		del_timer_sync(&fps_ints.timer);
		fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	}
	return 0;

}

/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
)
{
	unsigned int mask = 0;
	//int i = 0;
	pr_info("%s\n", __func__);

	fps_ints.int_count = 0;
	poll_wait(file, &interrupt_waitq, wait);
	if (fps_ints.finger_on) {
		mask |= POLLIN | POLLRDNORM;
		fps_ints.finger_on = 0;
	}
	return mask;
}

void fps_interrupt_abort(void)
{
	fps_ints.finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

/*-------------------------------------------------------------------------*/

static void fp_reset(void)
{
	pr_info("%s\n", __func__);
	//mt_set_gpio_out(GPIO_FPS_RESET_PIN, GPIO_OUT_ZERO);
	gpio_set_value(GPIO_FPS_RESET_PIN, 0);
	msleep(30);
	//mt_set_gpio_out(GPIO_FPS_RESET_PIN, GPIO_OUT_ONE);
	gpio_set_value(GPIO_FPS_RESET_PIN, 1);
	msleep(20);
}

static void fp_reset_set(int enable)
{
	pr_info("%s\n", __func__);
	pr_info("%s enable %d\n", __func__, enable);
	if (enable == 0) {
		//mt_set_gpio_out(GPIO_FPS_RESET_PIN, GPIO_OUT_ZERO);
		gpio_set_value(GPIO_FPS_RESET_PIN, 0);
		msleep(30);
	} else {
		//mt_set_gpio_out(GPIO_FPS_RESET_PIN, GPIO_OUT_ONE);
		gpio_set_value(GPIO_FPS_RESET_PIN, 1);
		msleep(20);
	}
}

static ssize_t fp_read(struct file *filp,
						char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static ssize_t fp_write(struct file *filp,
						const char __user *buf,
						size_t count,
						loff_t *f_pos)
{
	/*Implement by vendor if needed*/
	return 0;
}

static long fp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	struct fp_data *fp;
	struct spi_device *spi;
	u32 tmp;
	struct egis_ioc_transfer *ioc = NULL;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != EGIS_IOC_MAGIC) {
		pr_err("%s _IOC_TYPE(cmd) != EGIS_IOC_MAGIC", __func__);
		return -ENOTTY;
	}

	/*
	 * Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg,
						_IOC_SIZE(cmd));
	if (err) {
		pr_err("%s err", __func__);
		return -EFAULT;
	}

	/*
	 * guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	fp = filp->private_data;
	spin_lock_irq(&fp->spi_lock);
	spi = spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);

	if (spi == NULL) {
		pr_err("%s spi == NULL", __func__);
		return -ESHUTDOWN;
	}

	mutex_lock(&fp->buf_lock);

	/* segmented and/or full-duplex I/O request */
	if (_IOC_NR(cmd) != _IOC_NR(EGIS_IOC_MESSAGE(0))
					|| _IOC_DIR(cmd) != _IOC_WRITE) {
		retval = -ENOTTY;
		goto out;
	}

	tmp = _IOC_SIZE(cmd);
	if ((tmp == 0) || (tmp % sizeof(struct egis_ioc_transfer)) != 0) {
		retval = -EINVAL;
		goto out;
	}

	/* copy into scratch area */
	ioc = kmalloc(tmp, GFP_KERNEL);
	if (ioc == NULL) {
		retval = -ENOMEM;
		goto out;
	}
	if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
		retval = -EFAULT;
		goto out;
	}

	/*
	 * Read register
	 * tx_buf include register address will be read
	 */
	if (ioc->opcode == FP_REGISTER_READ) {
		u8 *address = ioc->tx_buf;
		u8 *result = ioc->rx_buf;
		DEBUG_PRINT("fp FP_REGISTER_READ\n");

		retval = fp_io_read_register(fp, address, result);
		if (retval < 0)	{
			pr_err("%s FP_REGISTER_READ error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Write data to register
	 * tx_buf includes address and value will be wrote
	 */
	if (ioc->opcode == FP_REGISTER_WRITE) {
		u8 *buf = ioc->tx_buf;
		DEBUG_PRINT("fp FP_REGISTER_WRITE");

		retval = fp_io_write_register(fp, buf);
		if (retval < 0) {
			pr_err("%s FP_REGISTER_WRITE error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	/*
	 * Get one frame data from sensor
	 */
	if (ioc->opcode == FP_GET_ONE_IMG) {
		u8 *buf = ioc->tx_buf;
		u8 *image_buf = ioc->rx_buf;
		//DEBUG_PRINT("fp FP_GET_ONE_IMG\n");

        spi->controller_data = (void *) &spi_conf_dma;          // mtk configuration
        spi_setup(spi);

		retval = fp_io_get_one_image(fp, buf, image_buf);
		if (retval < 0) {
			pr_err("%s FP_GET_ONE_IMG error retval = %d\n"
			, __func__, retval);
			goto out;
		}
	}

	if (ioc->opcode == FP_SENSOR_RESET)
		fp_reset();

	if (ioc->opcode == FP_RESET_SET) {
		pr_info("%s FP_SENSOR_RESET\n", __func__);
		pr_info("%s status = %d\n", __func__, ioc->len);
		fp_reset_set(ioc->len);
	}

	if (ioc->opcode == FP_SET_SPI_CLOCK) {
		__u32 current_speed = spi->max_speed_hz;
		pr_info("%s FP_SET_SPI_CLOCK\n", __func__);
		pr_info("%s speed_hz = %d\n", __func__, ioc->speed_hz);
		pr_info("%s current_speed = %d\n", __func__, current_speed);

		spi->max_speed_hz = ioc->speed_hz;
		retval = spi_setup(spi);
		if (retval < 0) {
			pr_err("%s spi_setup error %d\n", __func__, retval);
			spi->max_speed_hz = current_speed;
		}
		pr_info("%s spi speed_hz = %d\n", __func__, spi->max_speed_hz);
	}

	if (ioc->opcode == FP_POWER_ONOFF)
		pr_info("power control status = %d\n", ioc->len);

	/*
	 * Trigger inital routine
	 */
	if (ioc->opcode == INT_TRIGGER_INIT) {
		pr_info(">>> fp Trigger function init\n");
		retval = Interrupt_Init(
				(int)ioc->pad[0],
				(int)ioc->pad[1],
				(int)ioc->pad[2]);
		pr_info("trigger init = %d\n", retval);
	}

	/*
	 * trigger
	 */
	if (ioc->opcode == INT_TRIGGER_CLOSE) {
		pr_info("<<< fp Trigger function close\n");
		retval = Interrupt_Free();
		pr_info("trigger close = %d\n", retval);
	}

	/*
	 * read interrupt status
	 */
	if (ioc->opcode == INT_TRIGGER_ABORT)
		fps_interrupt_abort();

out:
	if (ioc != NULL)
		kfree(ioc);

	mutex_unlock(&fp->buf_lock);
	spi_dev_put(spi);
	if (retval < 0)
		pr_err("%s retval = %d", __func__, retval);
	return retval;
}

#ifdef CONFIG_COMPAT
static long fp_compat_ioctl(struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	return fp_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define fp_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int fp_open(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;
	int			status = -ENXIO;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);

	list_for_each_entry(fp, &device_list, device_entry)
	{
		if (fp->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (fp->buffer == NULL) {
			fp->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (fp->buffer == NULL) {
				dev_dbg(&fp->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		if (status == 0) {
			fp->users++;
			filp->private_data = fp;
			nonseekable_open(inode, filp);
		}
	} else
		pr_debug("fp: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;
}

static int fp_release(struct inode *inode, struct file *filp)
{
	struct fp_data *fp;

	DEBUG_PRINT("%s\n", __func__);
	mutex_lock(&device_list_lock);
	fp = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	fp->users--;
	if (fp->users == 0) {
		int	dofree;

		kfree(fp->buffer);
		fp->buffer = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&fp->spi_lock);
		dofree = (fp->spi == NULL);
		spin_unlock_irq(&fp->spi_lock);

		if (dofree)
			kfree(fp);
	}
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct file_operations fp_fops = {
	.owner = THIS_MODULE,
	.write = fp_write,
	.read = fp_read,
	.unlocked_ioctl = fp_ioctl,
	.compat_ioctl = fp_compat_ioctl,
	.open = fp_open,
	.release = fp_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll
};

/*-------------------------------------------------------------------------*/

static struct class *fp_class;
u8 result[] = {0xFF, 0xFF};
u8 result1[] = {0xFF, 0xFF};
static struct kobject *et320_kobj=NULL;
static ssize_t fp_show_readimage(struct device *ddri,struct device_attribute *attr,char *buf)
{	
	return sprintf(buf,"ET310:%x %x\n",result[1],result1[1]);
}
static DEVICE_ATTR(readimage,S_IWUSR|S_IRUGO,fp_show_readimage,NULL);
static int fp_is_connected(struct fp_data *fp)
{	
	int ret,len = 2;	//char buf,buf1;	
	ret = fp_mass_read(fp, 0x11, result, len);	
	ret = fp_mass_read(fp, 0x13, result1, len);	
	if (ret < 0) 
	{		
	pr_err("%s call fp_mass_read error status = %d\n"				, __func__, ret);		
	return 1;	
	}	
	if((result[1] == 0x3f) && (0x8f == result1[1]))		
		return 0;	
	else		
		return 1;	
}

/*-------------------------------------------------------------------------*/

static int __init fp_probe(struct spi_device *spi)
{
	struct fp_data *fp;
	int status;
	unsigned long minor;
	//int i;
	int ret;
	
	struct spi_device *spi_initialize;

	DEBUG_PRINT("%s initial\n", __func__);

	/* Allocate driver data */
	fp = kzalloc(sizeof(*fp), GFP_KERNEL);
	if (fp == NULL)
		return -ENOMEM;

	/* Initialize the driver data */
	fp->spi = spi;

	//hwPowerOn(MT6331_POWER_LDO_VMCH, VOL_3300,"ET320_3V3");	//mtk 3.3V power-on
	spi->controller_data = (void *) &spi_conf;          // mtk configuration
    spi->max_speed_hz = FPC_BTP_SPI_CLOCK_SPEED;
    spi_setup(spi);
	spin_lock_init(&fp->spi_lock);
	mutex_init(&fp->buf_lock);

	INIT_LIST_HEAD(&fp->device_entry);
	et320_kobj = kobject_create_and_add("et320_sysfs",NULL);	
	status = sysfs_create_file(et320_kobj,&dev_attr_readimage.attr);

	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

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
	//mt_set_gpio_dir(GPIO_FPS_RESET_PIN, GPIO_DIR_OUT);
	spin_lock_irq(&fp->spi_lock);
	spi_initialize= spi_dev_get(fp->spi);
	spin_unlock_irq(&fp->spi_lock);	

	spi_initialize->mode = SPI_MODE_3;
	spi_initialize->bits_per_word = 8;

	spi_setup(spi_initialize);

	fp_reset();
	if(fp_is_connected(fp))
	{
		DEBUG_PRINT("%s : initialize error %d\n", __func__, status);
		goto exit_fp_probe;
	}
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		fp->devt = MKDEV(FP_MAJOR, minor);
		dev = device_create(fp_class, &spi->dev, fp->devt,
					fp, "esfp0");
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else{
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&fp->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	if (status == 0)
		spi_set_drvdata(spi, fp);
	else
		kfree(fp);

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	
	setup_timer(&fps_ints.timer, interrupt_timer_routine, (unsigned long)&fps_ints);
	add_timer(&fps_ints.timer);

	DEBUG_PRINT("%s : initialize success %d\n", __func__, status);
	Interrupt_Init(0,20,10);
	return status;
exit_fp_probe:
		kfree(fp);
		return -1;

}

static int fp_remove(struct spi_device *spi)
{
	struct fp_data *fp = spi_get_drvdata(spi);
	DEBUG_PRINT("%s\n", __func__);

	//hwPowerDown(MT6331_POWER_LDO_VMCH,"ET320_3V3");	// mtk 3.3v power-off

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&fp->spi_lock);
	fp->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&fp->spi_lock);

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&fp->device_entry);
	device_destroy(fp_class, fp->devt);
	clear_bit(MINOR(fp->devt), minors);
	if (fp->users == 0)
		kfree(fp);
	mutex_unlock(&device_list_lock);

	return 0;
}

struct spi_device_id et320_id_table = {"et320", 0};

static struct spi_driver fp_spi_driver = {

	.driver = {
		.name = "et320",
		.bus	= &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = fp_probe,
	.remove = fp_remove,
	.id_table = &et320_id_table,

};

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
			.modalias="et320",
			.bus_num = 1,
			.chip_select=0,
			.mode = SPI_MODE_3,
		},
};

/*-------------------------------------------------------------------------*/

static int __init fp_init(void)
{
	int status;
	DEBUG_PRINT("%s\n", __func__);

	/*
	 * Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(FP_MAJOR, "et320", &fp_fops);
	if (status < 0) {
		DEBUG_PRINT("%s : cannot register chrdev : %d\n", __func__, status);
		return status;
	}
	
	fp_class = class_create(THIS_MODULE, "fp");
	if (IS_ERR(fp_class)) {
		DEBUG_PRINT("%s : cannot create class\n", __func__);
		unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
		return PTR_ERR(fp_class);
	}

	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
	status = spi_register_driver(&fp_spi_driver);
	if (status < 0) {
		DEBUG_PRINT("%s : cannot spi register : %d\n", __func__, status);
		class_destroy(fp_class);
		unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
	}

	return status;
}
module_init(fp_init);

static void __exit fp_exit(void)
{
	DEBUG_PRINT("%s\n", __func__);
	gpio_free(GPIO_FPS_RESET_PIN);
	spi_unregister_driver(&fp_spi_driver);
	class_destroy(fp_class);
	unregister_chrdev(FP_MAJOR, fp_spi_driver.driver.name);
}
module_exit(fp_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("SPI Interface for ET320");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
