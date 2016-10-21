/*
 * Copyright (C) 2016 BetterLife.Co.Ltd. All rights  reserved.
* 
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/signal.h>
#include <linux/ctype.h>
#include <linux/wakelock.h>

#include <linux/jiffies.h>


#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/delay.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include "xz_sensor.h"

#ifndef CONFIG_ARCH_MSM
#define ARCH_MTK_BTL
#else
#undef ARCH_MTK_BTL
#endif
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/of_gpio.h>
#if defined(ARCH_MTK_BTL)
#include <mach/irqs.h>
#include <mt_spi.h>
#include <mt_gpio.h>
#include <mach/emi_mpu.h>
#include <mach/mt_clkmgr.h>
#include <mach/gpio_const.h>
#include "mt_spi_hal.h"
//#include <cust_eint.h>
//#include <cust_gpio_usage.h>
#endif

#define SPI_DRV_NAME	"bl229x"

#define bl229x_height  96
#define bl229x_width   112
#define bl229x_image_size (bl229x_height * bl229x_width)
#define BL229X_SPI_CLOCK_SPEED 12*1000*1000//6*1000*1000//10*1000*1000
#define DMA_TRANSFER_SIZE (11264)

#define READIMAGE_BUF_SIZE	(12288)
#define BL229X_IOCTL_MAGIC_NO			0xFC

#define INIT_BL229X				        _IO(BL229X_IOCTL_MAGIC_NO, 0)
#define BL229X_GETIMAGE			        _IOW(BL229X_IOCTL_MAGIC_NO, 1, u32)
#define BL229X_INITERRUPT_MODE		    _IOW(BL229X_IOCTL_MAGIC_NO, 2, u32)
#define BL229X_CONTRAST_ADJUST          _IOW(BL229X_IOCTL_MAGIC_NO, 3, u32)
#define BL229X_CONTRAST_ADJUST2		    _IOW(BL229X_IOCTL_MAGIC_NO, 3, u8)

#define BL229X_POWERDOWN_MODE1			_IO (BL229X_IOCTL_MAGIC_NO, 4)
#define BL229X_POWERDOWN_MODE2			_IO (BL229X_IOCTL_MAGIC_NO, 5)

#define BL229X_INTERRUPT_FLAGS1         _IOW(BL229X_IOCTL_MAGIC_NO, 4,  u32)
#define BL229X_INTERRUPT_FLAGS2         _IOW(BL229X_IOCTL_MAGIC_NO, 5,  u32)
#define BL229X_MULTIFUNCTIONAL_KEYCODE	_IOW(BL229X_IOCTL_MAGIC_NO, 6,  u32)
#define BL229X_TEST_MODE	            _IOWR(BL229X_IOCTL_MAGIC_NO, 7, u32)
#define BL229X_GET_ID	                _IOWR(BL229X_IOCTL_MAGIC_NO, 9, u32)
#define BL229X_INIT_ARGS	            _IOWR(BL229X_IOCTL_MAGIC_NO, 11, u32)
#define BL229X_GAIN_ADJUST              _IOWR(BL229X_IOCTL_MAGIC_NO, 12, u32)
//#define BL229X_GET_PRESURE              _IOWR(BL229X_IOCTL_MAGIC_NO, 13, u32)
#define BL229X_ENBACKLIGHT           	_IOW(BL229X_IOCTL_MAGIC_NO, 13, u32)
#define BL229X_ISBACKLIGHT           	_IOWR(BL229X_IOCTL_MAGIC_NO, 14, u32)




#define CHIP_ID_LOW		                (0x83)
#define CHIP_ID_HIGH	                (0x51)
#define GPIO_OUT_ZERO	                (0)
#define GPIO_OUT_ONE	                (1)


#define CHIP_VERSION                    (3)


#define BTL_DEBUG(fmt,arg...)          do{\
	if(bl229x_log)\
	printk("<btl-dbg>[%s:%d]"fmt"\n",__func__, __LINE__, ##arg);\
}while(0)

	
#define DRIVER_DEBUG            (1)
#define FP_DET_BY_FD            (0)
#define FP_DET_BY_NAV           (1)

#define INT_MO
#define FP_DETECT_METHOD        (FP_DET_BY_FD) //0 FD:mode, 1: NAV MODE


#define RESET_PIN_FAILED	(1)
#define SPI_PIN_FAILED		(2)
#define INT_PIN_FAILED		(3)

#define PRESS_MAX_TIMEOUT_COUNT	    (50)
#define ESD_RECOVERY_RETRY_TIMES    (3)

#define REPORT_DELAY_TIME              (20)

// sensor parameter
#define SENSOR_DEFAULT_GAIN_V1	       (0xB6)	//(0xB6)	//B6
#define SENSOR_DEFAULT_GAIN_V2	       (0xB6)	//(0xB6)	//B6
#define SENSOR_DEFAULT_GAIN_V3		   (0xB6)	
// AGC
#define SENSOR_DEFAULT_CONTRAST_V1     (60)
#define SENSOR_DEFAULT_CONTRAST_V2     (60)
#define SENSOR_DEFAULT_CONTRAST_V3     (60)//(0x90)//(80)//(0x90)	
//#define SENSOR_DEFAULT_CONTRAST (0x70)
#define SENSOR_DEFAULT_DACN_V3		   (0xff)	// DACN (N should be larger than P)

#define REPORT_KEY_DEBOUNCE_TIME    (HZ/10)

#define INIT_PARAM_SIZE              22


/*----------------------------------------------------------------------------*/

typedef enum {
    SENSOR_T_MODE_IDLE = 0,
    SENSOR_T_MODE_FP,
    SENSOR_T_MODE_CAP,
    SENSOR_T_MODE_NAV,
    SENSOR_T_MODE_INI,
    SENSOR_T_MODE_RESET,
} sensor_testmode_t;


typedef enum {
    INT_MODE_NONE     = 0,
    INT_MODE_KEY  = 1
} mode_type;


//module_param(bl229x_log, int, 00664);

struct bl229x_data {
    struct spi_device *spi;
    u8 *image_buf;
	u8 *imagetxcmd;
	u8 *imagerxpix;
    struct semaphore mutex;
	struct semaphore handler_mutex;
    u32 reset_gpio;
    u32 irq_gpio;
    u32 irq_num;
    u32 power_en_gpio;
	u8  interrupt_mode; // 0: none interrupt; 1: interrupt
	u8  is_frame_ready;
	u32 contrast;
	u32 agc; 
	s32 report_key;
	s32 report_delay;
	s32 reset;
	u8  opened;
	u8  fp_detect_method;
	
	unsigned long report_timeout;
	
    struct pinctrl *pinctrl1;
    //struct pinctrl_state *spi_pins_default;
    struct pinctrl_state *power_en_output0;
    struct pinctrl_state *power_en_output1;
    struct pinctrl_state *rst_output0;
    struct pinctrl_state *rst_output1;
    struct pinctrl_state *int_default;


	struct mutex  spi_lock;
    struct notifier_block fb_notify;
};

#if defined(ARCH_MTK_BTL)
static struct mt_chip_conf spi_conf= {

    .setuptime = 10,
    .holdtime = 10,
    .high_time = 8, //此处决定slk的频率
    .low_time =  8,
    .cs_idletime = 20, //10,
    //.ulthgh_thrsh = 0,

    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,  //\CFȴ\AB\B8\DFλ
    .tx_mlsb = 1,

    .tx_endian = 0, //tx_endian \B1\EDʾ\B4\F3\B6\CBģʽ
    .rx_endian = 0,

    .com_mod = DMA_TRANSFER,
    .pause = 1,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,


};




  
/*----------------------------------------------------------------------------*/


/*
static struct spi_board_info spi_board_bl229x[] __initdata = {
	[0] = {
		.modalias= SPI_DRV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.max_speed_hz = BL229X_SPI_CLOCK_SPEED,
	},
};
*/

#endif


struct fingerprintd_params_t{
 u8 def_contrast;                   //contrast's default for captruing image
 u8 def_ck_fingerup_contrast;       //contrast's value for checking if finger up
 u8 def_ck_fingerup_frames_e;       //waiting frames of checking if finger up for enroll
 u8 def_ck_fingerup_frames_m;       //waiting frames of checking if finger up for match	
 u8 def_match_failed_times;         //max failed times for match
 u8 def_enroll_try_times;           //tried times for balance of wet and dry when enroll
 u8 def_match_try_times;            //tried times for balance of wet and dry when match
 u8 def_intensity_threshold;        //intensity's threshold
 u8 def_contrast_high_value;        // high threshold of contrast
 u8 def_contrast_low_value;         // low threshold of contrast
 u8 def_match_quality_score_threshold; //score's threshold of quality, when match
 u8 def_match_quality_area_threshold;  //area's threshold of quality, when match
 u8 def_enroll_quality_score_threshold;//score's threshold of quality, when enroll
 u8 def_enroll_quality_area_threshold; //area's threshold of quality, when match
 u8 def_shortkey_disable;              //short key switch 
 u8 def_rate;                          // far 1 - 16() 
 u8 def_max_samples;                   // max sample for a finger
 u8 def_debug_enable;
 u8 def_contrast_direction;
 u8 reserved2;
};

const struct fingerprintd_params_t fingerprintdParams = {
  .def_contrast = 90,
  .def_ck_fingerup_contrast = 60,
  .def_ck_fingerup_frames_e = 15,
  .def_ck_fingerup_frames_m = 10, 	
  .def_match_failed_times = 5,
  .def_enroll_try_times = 5,
  .def_match_try_times = 3,
  .def_intensity_threshold = 10,
  .def_contrast_high_value = 100,
  .def_contrast_low_value = 60,
  .def_match_quality_score_threshold  = 50,
  .def_match_quality_area_threshold   = 13,
  .def_enroll_quality_score_threshold = 50,
  .def_enroll_quality_area_threshold  = 18,
  .def_shortkey_disable = 0,
  .def_rate = 8,
  .def_max_samples = 10,
  .def_debug_enable = DRIVER_DEBUG,
#if (CHIP_VERSION == 3)
  .def_contrast_direction = 1,
#else 
  .def_contrast_direction = 0,
#endif 
  .reserved2 = 0,
};

struct wake_lock fp_suspend_lock;

/*----------------------------------------------------------------------------*/

//driver init
static int  mt_spi_init(void);
static void mt_spi_exit(void);
static int  bl229x_probe(struct spi_device *spi);
static int  bl229x_open(struct inode *inode, struct file *file);
static ssize_t bl229x_write(struct file *file, const char *buff,size_t count, loff_t *ppos);
static ssize_t bl229x_read(struct file *file, char *buff,size_t count, loff_t *ppos);
static long    bl229x_ioctl(struct file *filp, unsigned int cmd,unsigned long arg) ;
static int     bl229x_release(struct inode *inode, struct file *file);
static int     fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data);
static int bl229x_mmap(struct file *filp, struct vm_area_struct *vma);

//spi func and dev init spi cmd
static int spi_send_cmd(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen);

static int  bl229x_dev_init(struct bl229x_data *spidev);
static int  bl229x_read_image(struct bl229x_data *bl229x,u32 timeout);
static int  bl229x_dev_interrupt_init(struct bl229x_data *bl229x, int navOrFd);
static int  bl229x_contrast_init(struct bl229x_data *bl229x,unsigned long arg);

//hardware init gpio and irq, dma
//static int bl229x_eint_gpio_init(struct bl229x_data *bl229x);
irqreturn_t bl229x_eint_handler(int irq,void *data);
static int bl229x_create_inputdev(void);
static int mtspi_set_dma_en(int mode);
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable);
static int bl229x_gpio_select_and_init(struct bl229x_data *bl229x);
static int bl229x_parse_dt(struct device *dev,struct bl229x_data *pdata);
static int bl229x_reset_for_esd(struct bl229x_data *bl229x);
static int bl229x_suspend(struct device *dev);
static int bl229x_resume(struct device *dev);
static int bl229x_async_fasync(int fd,struct file *filp,int mode);
static int bl229x_set_testmode(struct bl229x_data *bl229x, u8* params, u8 size);
static int bl229x_read_chipid(struct bl229x_data *bl229x);
static int is_connected(struct bl229x_data *bl229x);
/*----------------------------------------------------------------------------*/
static atomic_t suspended;
static struct bl229x_data *g_bl229x= NULL;
static struct input_dev *bl229x_inputdev = NULL;
static struct kobject *bl229x_kobj=NULL;
static u8 bl229x_log = DRIVER_DEBUG;
static int inttrupt_enabled;
static u8 reg_rsp[256];
static u8 spicmd_rsp[256];

//Asynchronous notification struct
#ifndef SYS_ANDROID_L
static struct fasync_struct *async_queue;
#endif

static DECLARE_WAIT_QUEUE_HEAD(frame_waiter);


/*----------------------------------------------------------------------------*/

static void mt_eint_mask(unsigned int eint_num)
{
    disable_irq_nosync(eint_num);  
}


/*----------------------------------------------------------------------------*/

static void mt_eint_unmask(unsigned int eint_num)
{
    enable_irq(eint_num);  
}


/*----------------------------------------------------------------------------*/
static void exchange_white_black(u8 *dst,u8 *src,int len)
{
    int i = 0;
    for( ; i < len; i++) {
        *(dst + i) = 0xff & (0xff - *(src + i));
    }
}


/*----------------------------------------------------------------------------*/

static void spi_io_set_mode(int enable)
{
#if defined(ARCH_MTK_BTL)
    //pinctrl_select_state(g_bl229x->pinctrl1, g_bl229x->spi_pins_default);
#endif
}


/*----------------------------------------------------------------------------*/

// ѡ\D4\F1\B9\A4\D7\F7\D3\EB\C4\C7\D6\D6ģʽ
static int mtspi_set_dma_en(int mode)
{
#if defined(ARCH_MTK_BTL)
    struct mt_chip_conf* spi_par;
    spi_par = &spi_conf;
    if (!spi_par) {
        return -1;
    }
    if (1 == mode) {
        if (spi_par->com_mod == DMA_TRANSFER) {
            return 0;
        }
        spi_par->com_mod = DMA_TRANSFER;
    } else {
        if (spi_par->com_mod == FIFO_TRANSFER) {
            return 0;
        }
        spi_par->com_mod = FIFO_TRANSFER;
    }

    spi_setup(g_bl229x->spi);
#endif
    return 0;
}

/*--------------------------- Data Transfer -----------------------------*/
#define USE_SPI1_4GB_TEST (1)

#if USE_SPI1_4GB_TEST
//static dma_addr_t SpiDmaBufTx_pa;
//static dma_addr_t SpiDmaBufRx_pa;
static char *spi_tx_local_buf;
static char *spi_rx_local_buf;
/*
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
*/
static int spi_setup_xfer(struct spi_transfer *xfer)
{
	//u8 tx_buffer = 0x01;
		/* map physical addr to virtual addr */
		if (NULL == spi_tx_local_buf) {
			spi_tx_local_buf = (char *)ioremap_nocache(SpiDmaBufTx_pa, 0x4000);
			if (!spi_tx_local_buf) {
				BTL_DEBUG("SPI Failed to dma_alloc_coherent()\n");
				return -ENOMEM;
			}
		}
		if (NULL == spi_rx_local_buf) {
			spi_rx_local_buf = (char *)ioremap_nocache(SpiDmaBufRx_pa, 0x4000);
			if (!spi_rx_local_buf) {
				BTL_DEBUG("SPI Failed to dma_alloc_coherent()\n");
				return -ENOMEM;
			}
		}
		//xfer->tx_buf = spi_tx_local_buf;
		//xfer->rx_buf = spi_rx_local_buf;
		//xfer->tx_dma = SpiDmaBufTx_pa;
		//xfer->rx_dma = SpiDmaBufRx_pa;
	return 0;

}
#endif
/*----------------------------------------------------------------------------*/
static int spi_send_cmd(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen)
{
    int ret=0;
    struct spi_message m;
    struct spi_transfer t = {
        .cs_change = 0,
        .delay_usecs = 5,
        .speed_hz = BL229X_SPI_CLOCK_SPEED,
        .tx_buf = tx,
        .rx_buf = rx,
        .len = spilen,
        .tx_dma = SpiDmaBufTx_pa,
        .rx_dma = SpiDmaBufRx_pa,
        .bits_per_word = 0,
    };
	
	mutex_lock(&bl229x->spi_lock);

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    ret= spi_sync(bl229x->spi,&m);

	mutex_unlock(&bl229x->spi_lock);
    return ret;
}


/*----------------------------------------------------------------------------*/
static __attribute__((unused)) int spi_send_cmd_fifo(struct bl229x_data *bl229x,u8 *tx,u8 *rx,u16 spilen)
{
    int ret=0;
    struct spi_message m;
    struct spi_transfer t = {
        .cs_change = 0,
        .delay_usecs = 5,
        .speed_hz = BL229X_SPI_CLOCK_SPEED,
        .tx_buf = tx,
        .rx_buf = rx,
        .len = spilen,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 0,
    };
    mtspi_set_dma_en(0);  // fifo ģʽ
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    ret= spi_sync(bl229x->spi,&m);
    return ret;
}


//-------------------------------------------------------------------------------------------------
static u8 bl229x_spi_read_reg(u8 nRegID)
{
    u8 nAddr;
    //u8 data_tx[4];
    //u8 data_rx[4];

    nAddr = nRegID << 1;
    nAddr &= 0x7F;

    spi_tx_local_buf[0] = nAddr;
    spi_tx_local_buf[1] = 0xff;
    
    spi_send_cmd(g_bl229x,spi_tx_local_buf,spi_rx_local_buf,2);
    return spi_rx_local_buf[1];
}


/*----------------------------------------------------------------------------*/
static u8 bl229x_spi_write_reg(u8 nRegID, u8 value)
{
    u8 nAddr;
    //u8 data_tx[4];
    //u8 data_rx[4];

    nAddr = nRegID << 1;
    nAddr |= 0x80;

    spi_tx_local_buf[0] = nAddr;
    spi_tx_local_buf[1] = value;
   
    spi_send_cmd(g_bl229x, spi_tx_local_buf, spi_rx_local_buf, 2);
    return spi_rx_local_buf[1];
}


/*----------------------------------------------------------------------------*/
static u8 getSensorInterruptStatus(struct bl229x_data *bl229x)
{
    u8 unStatus = 0;

    unStatus = bl229x_spi_read_reg(REGA_INTR_STATUS);
   
    BTL_DEBUG("nStatus=%2x",unStatus);
 
    return unStatus;
}


/*----------------------------------------------------------------------------*/
void hexdump(const unsigned char *buf, const int num)
{
    int i;
    for(i = 0; i < num; i++) {
        printk("%02X ", buf[i]);
        if ((i+1)%8 == 0)
            printk("\n");
    }
    printk("\n");
    return;
}


/*----------------------------------------------------------------------------*/
static __attribute__((unused)) u8*  spi_read_frame(struct bl229x_data *bl229x)
{
    u8 nAddr;

    nAddr = REGA_FINGER_CAP << 1;
    nAddr &= 0x7F;

    memset(bl229x->image_buf, 0xff, READIMAGE_BUF_SIZE);
    memset(bl229x->imagetxcmd, 0x66, READIMAGE_BUF_SIZE);
    bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
    bl229x_spi_write_reg(REGA_RX_DACP_LOW, 0xd0);
    bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_CAP);
    msleep(50);
    bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_PRINT);

    bl229x->imagetxcmd[0] = nAddr;
    mtspi_set_dma_en(1);
    spi_send_cmd(bl229x, bl229x->imagetxcmd, bl229x->image_buf, DMA_TRANSFER_SIZE);
    //hexdump(imagebuf,bl229x_image_size);
    mtspi_set_dma_en(0);
    return (bl229x->image_buf);
}

/*----------------------------------------------------------------------------*/
static int spi_read_frame_now(struct bl229x_data *bl229x)
{
    u8 nAddr;
    u8 data_tx[4];
    u8 data_rx[4];

    BTL_DEBUG("%s++\n",__func__);
	if (down_interruptible(&bl229x->handler_mutex))
		return -ERESTARTSYS;	
    memset(bl229x->image_buf, 0xff, READIMAGE_BUF_SIZE);
    memset(bl229x->imagetxcmd, 0xff, READIMAGE_BUF_SIZE);
    memset(bl229x->imagerxpix, 0xff, READIMAGE_BUF_SIZE);

    //spi_write_reg(REGA_HOST_CMD, MODE_FG_PRINT);
    nAddr = REGA_HOST_CMD << 1;
    nAddr |= 0x80;
    data_tx[0] = nAddr;
    data_tx[1] = MODE_FG_PRINT;
    spi_send_cmd(g_bl229x,data_tx,data_rx,2);

	
    nAddr = REGA_FINGER_CAP << 1;
    nAddr &= 0x7F;
    spi_tx_local_buf[0] = nAddr;//bl229x->imagetxcmd[0] = nAddr;
    
    mtspi_set_dma_en(1);
    //spi_send_cmd(bl229x, imagetxcmd, imagebuf, DMA_TRANSFER_SIZE);
    //spi_send_cmd(bl229x,  bl229x->imagetxcmd,  bl229x->imagerxpix, READIMAGE_BUF_SIZE);
    spi_send_cmd(bl229x,  spi_tx_local_buf,  spi_rx_local_buf, READIMAGE_BUF_SIZE);
    //hexdump(imagerxpix,112);
    exchange_white_black(bl229x->image_buf + 2,spi_rx_local_buf + 2, DMA_TRANSFER_SIZE - 2);//omit 2 dummy bytes
    //memcpy(imagebuf, imagerxpix, READIMAGE_BUF_SIZE);

    mtspi_set_dma_en(0);

	up(&bl229x->handler_mutex);
	BTL_DEBUG("%s--\n",__func__);
    return 0;
}


/*----------------------------------------------------------------------------*/
static __attribute__((unused)) u8* spi_read_frame_int(struct bl229x_data *bl229x)
{
    u8 nAddr;

    nAddr = REGA_FINGER_CAP << 1;
    nAddr &= 0x7F;

    memset(bl229x->image_buf, 0xff, READIMAGE_BUF_SIZE);
    memset(bl229x->imagetxcmd, 0x66, READIMAGE_BUF_SIZE);
    bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_PRINT);

    bl229x->imagetxcmd[0] = nAddr;
    mtspi_set_dma_en(1);
    spi_send_cmd(bl229x,  bl229x->imagetxcmd,  bl229x->image_buf, DMA_TRANSFER_SIZE);
    mtspi_set_dma_en(0);
    return ( bl229x->image_buf);
}

/*----------------------------------------------------------------------------*/
static int hw_reset(struct bl229x_data *bl229x)
{
    //test reset pin
    u32 pin_val= -1;
    bl229x_power_on(bl229x, 0);
    msleep(50);
    pin_val = gpio_get_value(bl229x->reset_gpio);
    if(GPIO_OUT_ZERO != pin_val)
        return -RESET_PIN_FAILED;
    BTL_DEBUG("%s rst pin_val=%d\n",__func__,pin_val);
    bl229x_power_on(bl229x, 1);
    pin_val = gpio_get_value(bl229x->reset_gpio);
    if(GPIO_OUT_ONE != pin_val)
        return -RESET_PIN_FAILED;
    BTL_DEBUG("%s rst pin_val=%d\n",__func__,pin_val);
    msleep(100);
    return 0;
}


/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_agc(struct device *ddri,struct device_attribute *attr,char *buf)
{
    return sprintf(buf,"agc var=%x  agc reg=%x\n",g_bl229x->contrast, bl229x_spi_read_reg(REGA_RX_DACP_LOW));
}


static ssize_t bl229x_store_agc(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    u8 nAddr;
    u8 data_tx[4];
    u8 data_rx[4];

    char *next;
    BTL_DEBUG("[bl229x]%s:\n\n", __func__);
    g_bl229x->contrast = simple_strtoul(buf, &next, 16);
    
    nAddr = REGA_RX_DACP_LOW << 1;
    nAddr |= 0x80;
    data_tx[0] = nAddr;
    data_tx[1] = g_bl229x->contrast;
    spi_send_cmd(g_bl229x,data_tx,data_rx,2);

    return size;
}
static DEVICE_ATTR(agc,0664,bl229x_show_agc,bl229x_store_agc);

/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_interrupt_mode(struct device *ddri,struct device_attribute *attr,char *buf)
{
    bl229x_dev_interrupt_init(g_bl229x,FP_DET_BY_FD);
    return sprintf(buf, "sky register int%d=%x rst%d =%x\n",g_bl229x->irq_gpio,gpio_get_value(g_bl229x->irq_gpio),g_bl229x->reset_gpio,gpio_get_value(g_bl229x->reset_gpio));
}

static ssize_t bl229x_store_interrupt_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    inttrupt_enabled = simple_strtoul(buf, &next, 10);
#if (1)
    if(inttrupt_enabled)
        mt_eint_unmask(g_bl229x->irq_num);
    else
        mt_eint_mask(g_bl229x->irq_num);
#else
    if(inttrupt_enabled)
        enable_irq(g_bl229x->spi->irq);
    else
        disable_irq_nosync(g_bl229x->spi->irq);
#endif
    return size;
}
static DEVICE_ATTR(interrupt,0664,bl229x_show_interrupt_mode,bl229x_store_interrupt_mode);

/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_readimage(struct device *ddri,struct device_attribute *attr,char *buf)
{
    bl229x_read_image(g_bl229x,100);
    return 0;
}

static DEVICE_ATTR(readimage,S_IWUSR|S_IRUGO,bl229x_show_readimage,NULL);

/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_reg(struct device *ddri,struct device_attribute *attr,char *buf)
{
    int count = 0;
    int buflen = 0;
	int param_size = reg_rsp[0];
    for(count = 0; count < param_size; count++) {
        buflen += sprintf(buf + buflen, "rsp[%d]=%x\n", count, reg_rsp[count+1]);
    }

    return buflen;
}

static ssize_t bl229x_store_reg(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    int count=0;
    u8 param[256];
	u8 w = 0;
	u8 tempBuf[128];
	u8 param_size;


    memset(tempBuf,0,128); 
	memcpy(tempBuf,buf,size);
	BTL_DEBUG("string:%s \n",tempBuf);


	w =  simple_strtoul(tempBuf, &next, 16);
	BTL_DEBUG("w:%d \n",w);
	BTL_DEBUG("next=%s \n ",next);
    while(!isxdigit(*next) && *next != 0)next++;
	if (*next == 0) {
		BTL_DEBUG("format error \n");
		reg_rsp[0] = 0;
		return 1; 
	}
	param_size = simple_strtoul(next, &next, 16);
    BTL_DEBUG("reg len=%d \n",param_size);
	
	for (count = 0; count < param_size; count++){	
      while(!isxdigit(*next) && *next != 0) 
	  	  next++;
	  if (*next == 0) break;

	  param[count] = simple_strtoul(next, &next, 16);
      BTL_DEBUG("param[%d]=%x \n ",count,param[count]);
	  BTL_DEBUG("next=%s \n ",next);
	}
	
	if (count != param_size){
	    BTL_DEBUG("format error -2  \n");
		reg_rsp[0] = 0;
		return 1; 
	}

	param[0] = param[0] << 1;
	if (w)
	   param[0] |= 0x80;
	else 
	   param[0] &= 0x7f;

	for(count = 0; count < param_size; count++) {
        BTL_DEBUG("param[%d]=%x\n", count,param[count]);
    }

    reg_rsp[0] = param_size;

	spi_send_cmd(g_bl229x, param, &reg_rsp[1], count);

    for(count = 0; count < param_size; count++) {
        BTL_DEBUG("rsp[%d]=%x\n", count, reg_rsp[count+1]);
    }
  
    return size;
}
static DEVICE_ATTR(reg, 0664, bl229x_show_reg, bl229x_store_reg);

static ssize_t bl229x_show_spicmd(struct device *ddri,struct device_attribute *attr,char *buf)
{
	int count = 0;
	int buflen = 0;
	int param_len = spicmd_rsp[0];
	for(count = 0; count < param_len; count++)
	{
		buflen += sprintf(buf + buflen, "rsp[%d]=%x\n", count, spicmd_rsp[count]);
	}

	return buflen;
}

static ssize_t bl229x_store_spicmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *next;
	int count=0;
	u8 param[256];
	int param_len = 0;

	param_len = simple_strtoul(buf, &next, 16);
	BTL_DEBUG("\n\n spicmd len=%d\n\n ",param_len);
	while(*next == '.')
	{
		param[count] = simple_strtoul(next+1, &next, 16);
		BTL_DEBUG("\nsky test param[%d]=%x \n ",count,param[count]);
		++count;
	}
	spi_send_cmd(g_bl229x, param, spicmd_rsp, count);

	for(count = 0; count < param_len; count++)
	{
		BTL_DEBUG("rsp[%d]=%x\n", count, spicmd_rsp[count]);
	}
	spicmd_rsp[0] = param_len;
	return size;
}
static DEVICE_ATTR(spicmd, 0664, bl229x_show_spicmd, bl229x_store_spicmd);


/*----------------------------------------------------------------------------*/
static ssize_t bl229x_show_key_interrupt(struct device *ddri,struct device_attribute *attr,char *buf)
{
    return sprintf(buf, "\nkey_interrupt=%d\n", g_bl229x->report_key );
}

static ssize_t bl229x_store_key_interrupt(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    g_bl229x->report_key  = simple_strtoul(buf, &next, 10);
    return size;
}

static DEVICE_ATTR(key_interrupt, 0664, bl229x_show_key_interrupt, bl229x_store_key_interrupt);

/*----------------------------------------------------------------------------*/
int g_bl229x_enbacklight = 1;
static ssize_t bl229x_show_enbacklight(struct device *ddri,struct device_attribute *attr,char *buf)
{
    //return sprintf(buf, "\nenbacklight=%d\n", g_bl229x->enbacklight);
    return sprintf(buf, "\nenbacklight=%d\n", g_bl229x_enbacklight);
}

static ssize_t bl229x_store_enbacklight(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    //g_bl229x->enbacklight = simple_strtoul(buf, &next, 10);
    
    g_bl229x_enbacklight = simple_strtoul(buf, &next, 10);
    BTL_DEBUG("g_bl229x_enbacklight = %d",g_bl229x_enbacklight);
    return size;
}

static DEVICE_ATTR(enbacklight, 0664, bl229x_show_enbacklight, bl229x_store_enbacklight);

EXPORT_SYMBOL(g_bl229x_enbacklight)
/*----------------------------------------------------------------------------*/
static int m_is_conneted = 0;
static ssize_t bl229x_show_report_delay(struct device *ddri,struct device_attribute *attr,char *buf)
{
    return sprintf(buf, "\nreport_delay=%d is_conneted=%d\n", g_bl229x->report_delay, m_is_conneted);
}

static ssize_t bl229x_store_report_delay(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    g_bl229x->report_delay = simple_strtoul(buf, &next, 10);
    return size;
}

static DEVICE_ATTR(report_delay, 0664, bl229x_show_report_delay, bl229x_store_report_delay);

/*----------------------------------------------------------------------------*/
static int  bl229x_dev_selftest(struct bl229x_data *bl229x)
{
    //复位信号
    u32 pin_val= -1;
    int chip_id = 0;
	
    BTL_DEBUG("[bl229x]%s:\n\n", __func__);
    //test reset pin
    hw_reset(bl229x);
    //test spi pins
    chip_id = bl229x_read_chipid(g_bl229x);
    if(chip_id < 0)
        return -SPI_PIN_FAILED;
    //---------------------------------------------
	bl229x_spi_write_reg(REGA_HOST_CMD,MODE_IDLE);

	bl229x_spi_write_reg(REGA_HOST_CMD,MODE_FG_CAP);

    msleep(10);
    pin_val = gpio_get_value(g_bl229x->irq_gpio);
    BTL_DEBUG("%s int pin_val=%d\n",__func__,pin_val);
    if(GPIO_OUT_ONE != pin_val)
        return -INT_PIN_FAILED;

    bl229x_dev_init(g_bl229x);
    bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);

    return 0;
}

static ssize_t bl229x_show_selftest(struct device *ddri,struct device_attribute *attr,char *buf)
{
    int ret = 0;
    mt_eint_mask(g_bl229x->irq_num);
    ret = bl229x_dev_selftest(g_bl229x);
    mt_eint_unmask(g_bl229x->irq_num);
    return sprintf(buf, "\nselftest=%d interrupt_mode_flag=%d\n", ret,g_bl229x->interrupt_mode);
}

static ssize_t bl229x_store_selftest(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}

static DEVICE_ATTR(selftest, 0664, bl229x_show_selftest, bl229x_store_selftest);


/*----------------------------------------------------------------------------*/
static struct device_attribute *bl229x_attr_list[] = {
    &dev_attr_agc,
    &dev_attr_interrupt,
    &dev_attr_readimage,
    &dev_attr_reg,
    &dev_attr_spicmd,
    &dev_attr_key_interrupt,
    &dev_attr_report_delay,
    &dev_attr_enbacklight,
    &dev_attr_selftest
};


/*----------------------------------------------------------------------------*/
static int bl229x_async_fasync(int fd,struct file *filp,int mode)
{
    BTL_DEBUG("\n\n");
    return fasync_helper(fd,filp,mode,&async_queue);
}


/*----------------------------------------------------------------------------*/
//\D2첽\C9ϱ\A8\BA\AF\CA\FD\BDӿ\DA
static void bl229x_async_Report(void)
{
    //Send signal to user space,POLL_IN is enable write
    BTL_DEBUG("\n\n");
    if (async_queue) {
        BTL_DEBUG("bl229x kill_fasync\n ");
        kill_fasync(&async_queue,SIGIO,POLL_IN);
    }
}


/*----------------------------------------------------------------------------*/
static int bl229x_read_chipid(struct bl229x_data *bl229x)
{
    int val_low = 0;
    int val_high = 0;
    int chip_id = 0;

     u8  old_value = 0;

     BTL_DEBUG("  ++\n");

     bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

     old_value = bl229x_spi_read_reg(REGA_VERSION_RD_EN);

     bl229x_spi_write_reg(REGA_VERSION_RD_EN, old_value|0x80);

    msleep(5);

    val_low = bl229x_spi_read_reg(0x10);//id reg low
    BTL_DEBUG("val_low=%x",val_low);
    if(CHIP_ID_LOW != val_low)
        return -SPI_PIN_FAILED;

    val_high = bl229x_spi_read_reg(0x11);//id reg high
    BTL_DEBUG("val_high=%x",val_high);
    if(CHIP_ID_HIGH != val_high)
        return -SPI_PIN_FAILED;

    chip_id =(val_high << 8) | (val_low & 0xff);
    BTL_DEBUG("chip_id=%x",chip_id);

     bl229x_spi_write_reg(REGA_VERSION_RD_EN, old_value);
     return chip_id;
}



/*----------------------------------------------------------------------------*/
static int bl229x_reset_for_esd(struct bl229x_data *bl229x)
{
    int recovery_count = 3;
	u8 cur_contrast = 0;
    int ret = 0;
    BTL_DEBUG("  ++\n");

	
	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	
	cur_contrast = bl229x_spi_read_reg(REGA_RX_DACP_LOW);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,cur_contrast);
	if (bl229x_spi_read_reg(REGA_RX_DACP_LOW) == cur_contrast && cur_contrast != 0){
		if (gpio_get_value(g_bl229x->irq_gpio) == 0){
           BTL_DEBUG("int gpio is low");
		   up(&bl229x->handler_mutex);
		   return 1;
		}	    
	}

    BTL_DEBUG("ret: %d,count = %d\n",ret,recovery_count);
   
	bl229x->reset = 1;

    while((ret <= 0) && recovery_count--) {
        BTL_DEBUG("hw_reset\n");
        hw_reset(bl229x);
        ret = bl229x_read_chipid(bl229x);
        BTL_DEBUG("recovey_from_esd_failed  recovery_count=%d chip_id=%x\n", recovery_count, ret);
    }
	up(&bl229x->handler_mutex);
    bl229x_dev_init(bl229x);
    bl229x_dev_interrupt_init(bl229x,g_bl229x->fp_detect_method);
    
    bl229x->reset = 0;
    BTL_DEBUG("  --\n");
    return 0;
}



/*----------------------------------------------------------------------------*/
static void fps_create_attributes(struct device *dev)
{
    int num = (int)(sizeof(bl229x_attr_list)/sizeof(bl229x_attr_list[0]));
    for (; num > 0;)
        device_create_file(dev, bl229x_attr_list[--num]);
}

static int fps_sysfs_init(void)
{
    int ret;
    bl229x_kobj = kobject_create_and_add("bl229x_sysfs",NULL);
    if(bl229x_kobj == NULL) {
        BTL_DEBUG("%s  subsystem_register failed\n",__func__);
        ret = -ENOMEM;
        return ret;
    }

    ret = sysfs_create_file(bl229x_kobj,&dev_attr_agc.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_interrupt.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_readimage.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_reg.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_spicmd.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_key_interrupt.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_report_delay.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_enbacklight.attr);
    ret = sysfs_create_file(bl229x_kobj,&dev_attr_selftest.attr);

    if(ret) {
        BTL_DEBUG("%s sysfs_create_file failed\n",__func__);
    }

	kobject_uevent(bl229x_kobj, KOBJ_ADD);
    return ret;
}

#if (CHIP_VERSION == 3)
// chip rev1 and driver ic rev0
int bl229x_dev_init(struct bl229x_data *bl229x)
{
	// 1 reg from 0x1d to 0x28, write 0x0, do not effect read 0x28 int status.
	int i = 0;
	u8 nVal = 0;

	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	for (i = 0x1D; i <= 0x28; i++)
	{
		bl229x_spi_write_reg(i, SENSOR_DEFAULT_DACN_V3);
	}

	// 2. FD threshold[7:0]: 0x11 00 
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_LOW, 0x00);

	// FD threshold[13:8]: 0x10
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, 0x01);

	// task: 3.1 decrement FD DT interval, 0x0A - 70ms, default clk - 0x06
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_LOW, 0x00);
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_HIGH, 0x06);

	// 3.2 32K clock change from 18K to 205K
	//BL_WriteSingleCmd(REGA_VERSION_RD_EN, 0x08);
	//BL_WriteSingleCmd(REGA_VERSION_RD_EN, 0x0F);

	// 3.4 32M fast clock changed to (v3.3, x65)->3.1M, (v2.8, x75)->3.5M
	//bl229x_spi_write_reg(0x39, 0x75);

	// 3.5 
	bl229x_spi_write_reg(REGA_NAVI_FRM1_LOW, 0x11);

	// 4 task 1: Frame num to 1 to test
	bl229x_spi_write_reg(0x17, 0x2C);	// prepare to read REGA_FRAME_NUM
	nVal = bl229x_spi_read_reg(REGA_FRAME_NUM);
	nVal &= 0xE7;	// b'11100111  
	bl229x_spi_write_reg(REGA_FRAME_NUM, nVal);

	// 5. 3 scan rows, close 1,3, set reg 0x2D bit[4] to 1, (pd_rx\u5bc4\u5b58\u5668)
	//BL_WriteSingleCmd(0x2D, 0xF0);
	bl229x_spi_write_reg(0x2D, 0xF0);
	
	// 5.2 driver驱动电压，
	bl229x_spi_write_reg(0x32, 0x49);	
	bl229x_spi_write_reg(0x33, 0x92);
	bl229x_spi_write_reg(0x35, 0x52);
    // driver 驱动电压 6.5v，默认 x79->v7.5
	bl229x_spi_write_reg(0x3d, 0x73);
	//bl229x_spi_write_reg(0xe0, 0x80);

	// 6 set gain, contrast
	
    bl229x_spi_write_reg(REGA_GC_STAGE,SENSOR_DEFAULT_GAIN_V3);

	bl229x_spi_write_reg(REGA_RX_DACP_LOW,SENSOR_DEFAULT_CONTRAST_V3);

	up(&bl229x->handler_mutex);
	
	return 0;
}
#elif  (CHIP_VERSION == 2)
static int  bl229x_dev_init(struct bl229x_data *bl229x)
{
	// 1 reg from 0x1d to 0x28, write 0x38, do not effect read 0x28 int status.
	int i = 0;
	uint8_t nVal = 0;

	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	for (i = 0x1D; i <= 0x28; i++)
	{
		bl229x_spi_write_reg(i, 0x38);
		//BL_WriteSingleCmd(i, 0x20);
	}

	// 2. FD threshold[7:0]: 0x11 00 
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_LOW, 0x00);

	// FD threshold[13:8]: 0x12 0x10
	//BL_WriteSingleCmd(REGA_FINGER_TD_THRED_HIGH, 0x10);
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, 0x01);

	// 3.1 decrement FD DT interval, 0x0A - 70ms
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_LOW, 0x00);
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_HIGH, 0x18);

	// 3.2 32K clock change from 18K to 205K
	bl229x_spi_write_reg(REGA_VERSION_RD_EN, 0x0F);
	
	// 3.4 32M fast clock change to (v3.3, x65)->50M, 
	bl229x_spi_write_reg(0x39, 0x75);
	
	// 3.5 \u52a0\u5feb\u6a21\u62df\u7535\u8def\u5f00\u542f
	bl229x_spi_write_reg(REGA_NAVI_FRM1_LOW, 0x11);

	// 6 set gain, contrast
    bl229x_spi_write_reg(REGA_GC_STAGE,SENSOR_DEFAULT_GAIN_V2);

	bl229x_spi_write_reg(REGA_RX_DACP_LOW,SENSOR_DEFAULT_CONTRAST_V2);

	// 4 task 1: Frame num to 1 to test
	bl229x_spi_write_reg(0x17, 0x2C);	// prepare to read REGA_FRAME_NUM
	nVal = bl229x_spi_read_reg(REGA_FRAME_NUM);
	nVal &= 0xE7;	// b'11100111  
	bl229x_spi_write_reg(REGA_FRAME_NUM, nVal);

	// 5.1 image background reversing, set reg 0x0E bit[0] to 1
	nVal = bl229x_spi_read_reg(REGA_NAVI_FRM7_LOW);
	nVal &= 0xFE;
	nVal |= 0x01;
	bl229x_spi_write_reg(REGA_NAVI_FRM7_LOW, nVal);	

	// 5.2 3 scan rows, close 1,3, set reg 0x2D bit[4] to 1, (pd_rx\u5bc4\u5b58\u5668)
	//BL_WriteSingleCmd(0x2D, 0xF0);

	up(&bl229x->handler_mutex);

	return 0;
}
#elif (CHIP_VERSION == 1)
/*----------------------------------------------------------------------------*/
static int  bl229x_dev_init(struct bl229x_data *bl229x)
{
    int i = 0;
	u8 val;

    BTL_DEBUG("  ++\n\n");

	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;
 
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

    // 1 reg from 1d to 27, write 0x20
    for (i = 0x1D; i <= 0x28; i++) {
       bl229x_spi_write_reg(i, 0);
    }

    // 2 set finger detect threold
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_LOW, 0); 
	bl229x_spi_write_reg(REGA_FINGER_TD_THRED_HIGH, 0x01);


    // 3 set FD DT interval 
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_LOW, 0); 
	bl229x_spi_write_reg(REGA_FINGER_DT_INTERVAL_HIGH, 0x0a);
	

	// 4 Navi DT internal 
	bl229x_spi_write_reg(REGA_TIME_INTERVAL_LOW, 0); 
	bl229x_spi_write_reg(REGA_TIME_INTERVAL_HIGH, 0x0a);

   
    // 5 32K clock change from 18K to 205K
	bl229x_spi_write_reg(REGA_VERSION_RD_EN, 0x0f);

    // 6 32M fast clock change to 60M	
	bl229x_spi_write_reg(0x39, 0x75);

 
	bl229x_spi_write_reg(REGA_NAVI_FRM1_LOW, 0x11);


	bl229x_spi_write_reg(REGA_GC_STAGE,SENSOR_DEFAULT_GAIN_V1);


	bl229x_spi_write_reg(REGA_RX_DACP_LOW,SENSOR_DEFAULT_CONTRAST_V1);
	
	bl229x_spi_write_reg(0x17, 0x2C);	// prepare to read REGA_FRAME_NUM
	val = bl229x_spi_read_reg(REGA_FRAME_NUM);
	val &= 0xE7;	// b'11100111  
	//val &= 0xEF;	// b'11101111  
	bl229x_spi_write_reg(REGA_FRAME_NUM, val);

	up(&bl229x->handler_mutex);

    BTL_DEBUG("  --\n");
    return 0;
}
#else
static int  bl229x_dev_init(struct bl229x_data *bl229x)
{
    return 0;
}
#endif 

/*----------------------------------------------------------------------------*/
static int bl229x_contrast_init(struct bl229x_data *bl229x,unsigned long arg)
{
	u8 cur_contrast = 0; 

    BTL_DEBUG("  ++\n");

	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;
 
	g_bl229x->interrupt_mode = INT_MODE_NONE;

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);

	bl229x_spi_write_reg(REGA_RX_DACP_LOW, arg);

	cur_contrast = bl229x_spi_read_reg(REGA_RX_DACP_LOW);

    BTL_DEBUG("last_agc=%x data_rx[1]=%x", (s32)arg, cur_contrast);
    if(cur_contrast != arg) 
		printk("%s,error !\n",__func__);     

	BTL_DEBUG("mode:%d \n",g_bl229x->interrupt_mode);


	up(&bl229x->handler_mutex);
	
    BTL_DEBUG("  --\n");
    return 0;
}


/*----------------------------------------------------------------------------*/
static int bl229x_gain_init(struct bl229x_data *bl229x,unsigned long arg)
{
	u8 cur_gain = (u8) arg;

    BTL_DEBUG("  ++\n");
	
	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;
 
	g_bl229x->interrupt_mode = INT_MODE_NONE;

	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);


	bl229x_spi_write_reg(REGA_GC_STAGE,cur_gain);

	up(&bl229x->handler_mutex);
	
    BTL_DEBUG("  --\n");
    return 0;
}

/*----------------------------------------------------------------------------*/
static int  bl229x_dev_interrupt_init(struct bl229x_data *bl229x, int navOrfp)
{

    u8 val = 0;
	
    BTL_DEBUG("  ++\n");

	if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);	
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	
	val = bl229x_spi_read_reg(REGA_HOST_CMD);
	if (val != MODE_IDLE){
		BTL_DEBUG("err0 %d\n",val);
		bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	}

	bl229x_spi_write_reg(REGA_RX_DACP_LOW, bl229x->contrast);

	val = bl229x_spi_read_reg(REGA_RX_DACP_LOW);
	if (val != bl229x->contrast){
		BTL_DEBUG("err1 %d\n",val);
		bl229x_spi_write_reg(REGA_RX_DACP_LOW, bl229x->contrast);
	}

	bl229x->interrupt_mode = INT_MODE_KEY;

    if (navOrfp)
	    bl229x_spi_write_reg(REGA_HOST_CMD, MODE_NAVI);
	else
        bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_DT);

	up(&bl229x->handler_mutex);
    BTL_DEBUG("  --\n");
    return 0;
}



/*----------------------------------------------------------------------------*/
static int bl229x_read_image(struct bl229x_data *bl229x,u32 timeout)
{
    BTL_DEBUG("  ++\n");

    if (down_interruptible(&bl229x->handler_mutex))
        return -ERESTARTSYS;
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
  
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_FG_CAP);

	up(&bl229x->handler_mutex);

	BTL_DEBUG("  --\n");
    return 0;
}


/*----------------------------------------------------------------------------*/
static int bl229x_set_testmode(struct bl229x_data *bl229x, u8 * params, u8 size)
{
	int testType;
	int i;
	int workmode;

    BTL_DEBUG("  ++\n");

	
	for (i = 0; i < size; i++)
		 BTL_DEBUG("%d\n",params[i]);

	
	testType = params[0];
	switch (testType){
        case 1:  // mode switch
            workmode = params[1];
			if (workmode == MODE_IDLE)
				bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
			else if (workmode == MODE_FG_DT)
				bl229x_dev_interrupt_init(bl229x,FP_DET_BY_FD);
			else if (workmode == MODE_FG_CAP)
				bl229x_read_image(bl229x,10);
			else if (workmode == MODE_NAVI)
				bl229x_dev_interrupt_init(bl229x,FP_DET_BY_NAV);
			break;
		case 2:  // init action 
		    if (params[1] == 1)
		       bl229x_dev_init(bl229x);
			else if (params[1] == 2){
				bl229x_dev_init(bl229x);
				bl229x_dev_interrupt_init(bl229x,0);
			}else if (params[1] == 2){
				bl229x_dev_init(bl229x);
				bl229x_dev_interrupt_init(bl229x,1);
			}
			break;
		case 3:  // enable or disabel interrupt
		    if (params[1] == 1)
				enable_irq(bl229x->irq_num);  
			else 
				disable_irq_nosync(bl229x->irq_num);
			break;
		case 4:  //reset
		    if (params[1] == 1){
		       hw_reset(bl229x);
			   bl229x_dev_init(bl229x);
		    }else if (params[1] == 2){
			   hw_reset(bl229x);
			   bl229x_dev_init(bl229x);
			   bl229x_dev_interrupt_init(bl229x,0);
			}else if (params[1] == 3){
			   hw_reset(bl229x);
			   bl229x_dev_init(bl229x);
			   bl229x_dev_interrupt_init(bl229x,1);
			}
			break;
		default:
			break;

	}

    BTL_DEBUG("  --\n");
    return 0;
}

/*----------------------------------------------------------------------------*/
irqreturn_t bl229x_eint_handler(int irq,void *data)
{
	u8 intStatus = 0;
	
	BTL_DEBUG("  ++\n");
    
	wake_lock_timeout(&fp_suspend_lock, HZ/2);
	BTL_DEBUG("mode:%d \n",g_bl229x->interrupt_mode);
    if (g_bl229x->fp_detect_method == FP_DET_BY_FD) 
		intStatus = 2;
	else if (g_bl229x->fp_detect_method == FP_DET_BY_NAV) 
		intStatus = 4;

	if (getSensorInterruptStatus(g_bl229x) != intStatus) {
	     if (bl229x_reset_for_esd(g_bl229x) == 1){
			bl229x_dev_init(g_bl229x);
	        bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);			 
		 }
       
	     BTL_DEBUG("  ++--\n");
         return IRQ_HANDLED;
	}
	spi_read_frame_now(g_bl229x);
	g_bl229x->is_frame_ready = 1;

    if (g_bl229x->interrupt_mode == INT_MODE_KEY) {       
	    g_bl229x->interrupt_mode = INT_MODE_NONE;

		//BTL_DEBUG("%u,%u,%u\n",jiffies,g_bl229x->report_timeout,REPORT_KEY_DEBOUNCE_TIME);
		if (time_after_eq(jiffies, g_bl229x->report_timeout)){
		   if (g_bl229x->report_key != 0){
                input_report_key(bl229x_inputdev,g_bl229x->report_key ,1);
                input_sync(bl229x_inputdev);

                input_report_key(bl229x_inputdev,g_bl229x->report_key ,0);
                input_sync(bl229x_inputdev);
		   }  
		   g_bl229x->report_timeout = jiffies + REPORT_KEY_DEBOUNCE_TIME; //500 ms
		   BTL_DEBUG("report power key %d\n",g_bl229x->report_key );
		}else 
           BTL_DEBUG("ignore key\n");
#ifndef SYS_ANDROID_L
        bl229x_async_Report();
#endif
		//kobject_uevent_env(&g_bl229x->spi->dev.kobj, KOBJ_CHANGE,"UEVENT=bl229x_irq");
        bl229x_dev_interrupt_init(g_bl229x,g_bl229x->fp_detect_method);
        msleep(g_bl229x->report_delay);
    } else {
        BTL_DEBUG("is_frame_ready:%d\n", g_bl229x->is_frame_ready);
       // if (g_bl229x->is_frame_ready == 0 ) {
        //    g_bl229x->is_frame_ready = 1;
        //    wake_up_interruptible(&frame_waiter);
        //}
        wake_up_interruptible(&frame_waiter);
    }
	
	BTL_DEBUG("  --\n");
	
    return IRQ_HANDLED;
}


/* -------------------------------------------------------------------- */
#ifdef CONFIG_ARCH_MSM
static int bl229x_parse_dt(struct device *dev,
                           struct bl229x_data *pdata)
{

    dev_err(dev, "bl229x_parse_dt\n");
    pdata->reset_gpio = of_get_named_gpio_flags(dev->of_node,
                        "fingerprint,rst-gpio", 0, NULL);
    if (!gpio_is_valid(pdata->reset_gpio))
        return -EINVAL;
    gpio_direction_output(pdata->reset_gpio, 1);

    pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
                      "fingerprint,touch-int-gpio", 0, NULL);
    if (!gpio_is_valid(pdata->irq_gpio))
        return -EINVAL;
    gpio_direction_input(pdata->irq_gpio);

    pdata->power_en_gpio = of_get_named_gpio_flags(dev->of_node,
                           "fingerprint,en-gpio", 0, NULL);
    if (!gpio_is_valid(pdata->power_en_gpio))
        return -EINVAL;
    gpio_direction_output(pdata->power_en_gpio, 1);

    dev_err(dev, "bl229x_parse_dt out\n");

    return 0;
}
#else

static int bl229x_parse_dt(struct device *dev,
                           struct bl229x_data *pdata)
{
    int ret;
    struct pinctrl *pinctrl1 = pdata->pinctrl1;
    //struct pinctrl_state *spi_pins_default = pdata->spi_pins_default;
    struct pinctrl_state *power_en_output0 = pdata->power_en_output0;
    struct pinctrl_state *power_en_output1 = pdata->power_en_output1;
    struct pinctrl_state *rst_output0 = pdata->rst_output0;
    struct pinctrl_state *rst_output1 = pdata->rst_output1;
    struct pinctrl_state *int_default = pdata->int_default;

    BTL_DEBUG("bl229x_pinctrl+++++++++++++++++\n");
    pinctrl1 = devm_pinctrl_get(dev);
    if (IS_ERR(pinctrl1)) {
        ret = PTR_ERR(pinctrl1);
        dev_err(dev, "fwq Cannot find bl229x pinctrl1!\n");
        return ret;
    }
    /*
    spi_pins_default = pinctrl_lookup_state(pinctrl1, "spi0_default");
    if (IS_ERR(pdata->spi_pins_default)) {
        ret = PTR_ERR(pdata->spi_pins_default);
        dev_err(dev, "fwq Cannot find bl229x pinctrl default %d!\n", ret);
    }
    */
    rst_output1 = pinctrl_lookup_state(pinctrl1, "rst_output1");
    if (IS_ERR(rst_output1)) {
        ret = PTR_ERR(rst_output1);
        dev_err(dev, "fwq Cannot find bl229x pinctrl rst_output1!\n");
    }
    rst_output0 = pinctrl_lookup_state(pinctrl1, "rst_output0");
    if (IS_ERR(rst_output0)) {
        ret = PTR_ERR(rst_output0);
        dev_err(dev, "fwq Cannot find bl229x pinctrl rst_output0!\n");
    }
    power_en_output1 = pinctrl_lookup_state(pinctrl1, "power_en_output1");
    if (IS_ERR(power_en_output1)) {
        ret = PTR_ERR(power_en_output1);
        dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output1!\n");
    }
    power_en_output0 = pinctrl_lookup_state(pinctrl1, "power_en_output0");
    if (IS_ERR(power_en_output0)) {
        ret = PTR_ERR(power_en_output0);
        dev_err(dev, "fwq Cannot find bl229x pinctrl power_en_output0!\n");
    }
    int_default = pinctrl_lookup_state(pinctrl1, "int_default");
    if (IS_ERR(int_default)) {
        ret = PTR_ERR(int_default);
        dev_err(dev, "fwq Cannot find bl229x pinctrl int_default!\n");
    }

    pdata->reset_gpio = of_get_named_gpio_flags(dev->of_node,
                        "fingerprint,rst-gpio", 0, NULL);
    pdata->irq_gpio = of_get_named_gpio_flags(dev->of_node,
                      "fingerprint,touch-int-gpio", 0, NULL);
    pdata->power_en_gpio = of_get_named_gpio_flags(dev->of_node,
                           "fingerprint,en-gpio", 0, NULL);

    pdata->pinctrl1 = pinctrl1;
    //pdata->spi_pins_default = spi_pins_default;
    pdata->power_en_output0 = power_en_output0;
    pdata->power_en_output1 = power_en_output1;
    pdata->rst_output0 = rst_output0;
    pdata->rst_output1 = rst_output1;
    pdata->int_default = int_default;

    //pinctrl_select_state(pinctrl1, spi_pins_default);
    pinctrl_select_state(pinctrl1, rst_output1);
    pinctrl_select_state(pinctrl1, power_en_output1);
    pinctrl_select_state(pinctrl1, int_default);

    BTL_DEBUG("bl229x_pinctrl----------\n");
    return 0;
}

#endif


/*----------------------------------------------------------------------------*/
//\B5\E7Դ\BF\AA\B9\D8AVDD\A3\A82.6V-3.6V\A3\A9\A3\ACDVDD\A3\A81.8V\A3\A9\A3\ACIOVDD\A3\A81.8V or 2.8V\A3\A9,RST/SHUTDOWN pull high
//ESD recovery have to power off, AVDD must under control
static int bl229x_power_on(struct bl229x_data *bl229x,bool enable)
{
#if defined(ARCH_MTK_BTL)
    if(enable) {
        if (!IS_ERR(bl229x->rst_output1)) {
            pinctrl_select_state(bl229x->pinctrl1, bl229x->rst_output1);
        }
        if (!IS_ERR(bl229x->power_en_output1)) {
            pinctrl_select_state(bl229x->pinctrl1, bl229x->power_en_output1);
        }
    } else {
        if (!IS_ERR(bl229x->rst_output0)) {
            pinctrl_select_state(bl229x->pinctrl1, bl229x->rst_output0);
        }
        if (!IS_ERR(bl229x->power_en_output0)) {
            pinctrl_select_state(bl229x->pinctrl1, bl229x->power_en_output0);
        }
    }
#else
    if(enable) {
        gpio_direction_output(bl229x->power_en_gpio, 1);
        gpio_direction_output(bl229x->reset_gpio, 1);
    } else {
        gpio_direction_output(bl229x->reset_gpio, 0);
        gpio_direction_output(bl229x->power_en_gpio, 0);
    }

#endif
    return 0;
}


/*----------------------------------------------------------------------------*/
static int bl229x_gpio_select_and_init(struct bl229x_data *bl229x)
{
    int error = 0;
#if defined(ARCH_MTK_BTL)
    bl229x_parse_dt(&bl229x->spi->dev, bl229x);
#elif defined(CONFIG_ARCH_MSM)
    bl229x_parse_dt(&bl229x->spi->dev, bl229x);
    if (gpio_is_valid(bl229x->reset_gpio)) {
        error = gpio_request(bl229x->reset_gpio, "FINGERPRINT_RST");
        gpio_direction_output(bl229x->reset_gpio, 1);
    }
    if (gpio_is_valid(bl229x->power_en_gpio)) {
        error = gpio_request(bl229x->power_en_gpio, "FINGERPRINT_3V3_EN");
        dev_err(&bl229x->spi->dev, "power_en_gpio\n");
    }
    if (gpio_is_valid(bl229x->irq_gpio)) {
        error = gpio_request(bl229x->irq_gpio, "FINGERPRINT-IRQ");
        if (error) {
            dev_err(&bl229x->spi->dev, "unable to request GPIO %d\n",
                    bl229x->irq_gpio);
            goto err;
        }
        error = gpio_direction_input(bl229x->irq_gpio);
        if (error) {
            dev_err(&bl229x->spi->dev, "set_direction for irq gpio failed\n");
            goto err;
        }
    }
err:
#endif
    return error;
}


/*----------------------------------------------------------------------------*/
// ע\B2\E1\D6ж\CF\C9豸
static int bl229x_create_inputdev(void)
{
    bl229x_inputdev = input_allocate_device();
    if (!bl229x_inputdev) {
        BTL_DEBUG("bl229x_inputdev create faile!\n");
        return -ENOMEM;
    }
    __set_bit(EV_KEY,bl229x_inputdev->evbit);
    __set_bit(KEY_F10,bl229x_inputdev->keybit);		//68
    __set_bit(KEY_F11,bl229x_inputdev->keybit);		//88
    __set_bit(KEY_F12,bl229x_inputdev->keybit);		//88
    __set_bit(KEY_CAMERA,bl229x_inputdev->keybit);	//212
    __set_bit(KEY_POWER,bl229x_inputdev->keybit);	//116
    __set_bit(KEY_PHONE,bl229x_inputdev->keybit);  //call 169
    __set_bit(KEY_BACK,bl229x_inputdev->keybit);  //call 158

    __set_bit(KEY_F1,bl229x_inputdev->keybit);	//69
    __set_bit(KEY_F2,bl229x_inputdev->keybit);	//60
    __set_bit(KEY_F3,bl229x_inputdev->keybit);	//61
    __set_bit(KEY_F4,bl229x_inputdev->keybit);	//62
    __set_bit(KEY_F5,bl229x_inputdev->keybit);	//63
    __set_bit(KEY_F6,bl229x_inputdev->keybit);	//64
    __set_bit(KEY_F7,bl229x_inputdev->keybit);	//65
    __set_bit(KEY_F8,bl229x_inputdev->keybit);	//66
    __set_bit(KEY_F9,bl229x_inputdev->keybit);	//67

    bl229x_inputdev->id.bustype = BUS_HOST;
    bl229x_inputdev->name = "bl229x_inputdev";
    if (input_register_device(bl229x_inputdev)) {
        printk("%s, register inputdev failed\n", __func__);
        input_free_device(bl229x_inputdev);
        return -ENOMEM;
    }

    return 0;
}


/* -------------------------------------------------------------------- */
static long bl229x_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
    struct bl229x_data *bl229x = filp->private_data;
    struct spi_device *spi;
    int error=0;
    u32 user_regval = 0;
	u32 chipid;
	u8  dataBuf[64];
	//int value;
	u32 bl229x_enbacklight = 0;

	BTL_DEBUG("%s\n",__func__);

    if (_IOC_DIR(cmd) & _IOC_READ)
        error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        error = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (error) {
        BTL_DEBUG("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

	if (bl229x->reset){
		BTL_DEBUG("chip is in reseting\n");
		return -EBUSY;
	}
	
    spi = spi_dev_get(bl229x->spi) ;
    if (down_interruptible(&bl229x->mutex))
        return -ENOTTY;

    switch (cmd) {
    case INIT_BL229X:
        BTL_DEBUG("INIT_BL229X \n");
        error= bl229x_dev_init(bl229x);
        break;
    case BL229X_GETIMAGE:
        BTL_DEBUG("BL229X_GETIMAGE \n");
        error = bl229x_read_image(bl229x,arg);
        break;
    case BL229X_INITERRUPT_MODE:
        BTL_DEBUG("BL229X_INITERRUPT_MODE \n");
        error = bl229x_dev_interrupt_init(bl229x,bl229x->fp_detect_method);
        break;
    case BL229X_CONTRAST_ADJUST: 
    case BL229X_CONTRAST_ADJUST2: 	
        BTL_DEBUG("BL229X_CONTRAST_ADJUST1 \n");
        error= bl229x_contrast_init(bl229x,arg);
        break;
    case BL229X_POWERDOWN_MODE1:
    case BL229X_POWERDOWN_MODE2:
        BTL_DEBUG("BL229X_POWERDOWN_MODE1 \n");
   
        error = bl229x_dev_interrupt_init(bl229x,bl229x->fp_detect_method);
        mt_eint_unmask(g_bl229x->irq_num);
        break;
    case BL229X_INTERRUPT_FLAGS1:
    case BL229X_INTERRUPT_FLAGS2:
        BTL_DEBUG("BL229X_INTERRUPT_FLAGS1 \n");
        user_regval = gpio_get_value(bl229x->irq_gpio);
        if (copy_to_user((void __user*)arg, &user_regval, sizeof(user_regval)) != 0) {
            error = -EFAULT;
        }
        break;
    case BL229X_MULTIFUNCTIONAL_KEYCODE:
        g_bl229x->report_key  = (int)arg;
        break;		
    case BL229X_TEST_MODE:
	    if (copy_from_user(dataBuf,(void __user*)arg,32) != 0 ){
		   BTL_DEBUG("ERROR: BL229X_TEST_MODE\n");
		   error = -EFAULT;
		   break;
		}
        bl229x_set_testmode(bl229x,dataBuf, 32);
        break;
	case BL229X_GAIN_ADJUST:
		bl229x_gain_init(bl229x,arg);
		break;
	case BL229X_GET_ID:
		chipid = bl229x_read_chipid(bl229x);
		if (copy_to_user((void __user*)arg,&chipid,sizeof(u32)*1) != 0 ){
		   error = -EFAULT;
		}
		break;
	case BL229X_INIT_ARGS:
		if (copy_to_user((void __user*)arg,&fingerprintdParams,sizeof(fingerprintdParams)) != 0 ){
		   error = -EFAULT;
		}
		break;
	case BL229X_ENBACKLIGHT:
		BTL_DEBUG("BL229X_ENBACKLIGHT arg:%d\n", (int)arg);
		g_bl229x_enbacklight = (int)arg;
		break;
	case BL229X_ISBACKLIGHT:
		BTL_DEBUG("BL229X_ISBACKLIGHT\n");
		bl229x_enbacklight = g_bl229x_enbacklight;
		if (copy_to_user((void __user*)arg,&bl229x_enbacklight,sizeof(u32)*1) != 0 ){
		   error = -EFAULT;
		}
		break;
    default:
        error = -ENOTTY;
        break;

    }

    up(&bl229x->mutex);
    return error;

}

/*----------------------------------------------------------------------------*/
static int bl229x_open(struct inode *inode, struct file *file)
{
    struct bl229x_data *bl229x;
	BTL_DEBUG("  ++\n");
	bl229x = g_bl229x;
	bl229x->opened++;
    spi_io_set_mode(1);
    if (down_interruptible(&bl229x->mutex))
        return -ERESTARTSYS;
    file->private_data = bl229x;
    up(&bl229x->mutex);

    BTL_DEBUG("  --\n");
    return 0;
}


/* -------------------------------------------------------------------- */
//д\B2\D9\D7\F7\A3\ACָ\CEƴ\AB\B8в\BB\D0\E8Ҫ\B6\D4\C6\E4\BD\F8\D0\D0д\B2\D9\D7\F7 \A1\A3\B9\CAֱ\BDӷ\B5\BB\D8 \B2\D9\D7\F7
static ssize_t bl229x_write(struct file *file, const char *buff,size_t count, loff_t *ppos)
{
    return -ENOMEM;
}

/* -------------------------------------------------------------------- */
// \B6\C1\B2\D9\D7\F7
static ssize_t bl229x_read(struct file *file, char  *buff,size_t count, loff_t *ppos)
{
    int ret=0;
    int timeout;
//    struct bl229x_data *bl229x = file->private_data;
    ssize_t status = 0;

    BTL_DEBUG("  ++\n");


	BTL_DEBUG("mode:%d \n",g_bl229x->interrupt_mode);
    //wait_event_interruptible(frame_waiter, is_frame_ready !=0);
    timeout = wait_event_interruptible_timeout(frame_waiter, g_bl229x->is_frame_ready !=0, 50);

    BTL_DEBUG("timeout:%d, is_frame_ready : %d\n\n",timeout,g_bl229x->is_frame_ready);

    if (timeout == 0 && g_bl229x->is_frame_ready == 0) {
        BTL_DEBUG("read timeout\n\n");
        return -EFAULT;
    }
	
    BTL_DEBUG("is_frame_ready=%d\n\n",g_bl229x->is_frame_ready);

    //if(g_bl229x->is_frame_ready == 1) {
    //    spi_read_frame_now(bl229x);
        //is_frame_ready = 0;
   // }
    BTL_DEBUG("copy_to_user \n");
    ret = copy_to_user(buff, g_bl229x->image_buf + 2, count); //skip
    if (ret) {
        status = -EFAULT;
    }

    g_bl229x->is_frame_ready = 0;

    BTL_DEBUG("status: %d \n", (int)status);
    BTL_DEBUG("  --\n");
    return status;
}


/* -------------------------------------------------------------------- */
static int bl229x_release(struct inode *inode, struct file *file)
{
    int status = 0 ;
    struct bl229x_data *bl229x = file->private_data;

    BTL_DEBUG("  ++\n");
	if (bl229x->opened == 0) return status;
#ifndef SYS_ANDROID_L
    bl229x_async_fasync(-1, file, 0);
#endif

	bl229x->opened--;
    BTL_DEBUG("  --\n");
    return status;
}
/* -------------------------------------------------------------------- */
static int bl229x_mmap(struct file *filp, struct vm_area_struct *vma){
	struct bl229x_data *bl229x = filp->private_data; /*获得设备结构体指针*/
    BTL_DEBUG("  ++\n");
	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_RESERVED;


	if (remap_pfn_range(vma,vma->vm_start,virt_to_phys(bl229x->image_buf)>>PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot))
	  return  -EAGAIN;
    BTL_DEBUG("  --\n");		    
	return 0;
}
/* -------------------------------------------------------------------- */
static int bl229x_suspend (struct device *dev)
{

    struct bl229x_data *bl229x = dev_get_drvdata(dev);

    dev_err (&bl229x->spi->dev,"[bl229x]%s\n", __func__);
    atomic_set(&suspended, 1);
	BTL_DEBUG("\n");
    return 0;
}

/* -------------------------------------------------------------------- */
static int bl229x_resume (struct device *dev)
{
    struct bl229x_data *bl229x = dev_get_drvdata(dev);
	
    dev_err (&bl229x->spi->dev,"[bl229x]%s\n", __func__);
    atomic_set(&suspended, 0);
	BTL_DEBUG("\n");
    return 0;
}


/*----------------------------------------------------------------------------*/
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;

    int *blank;


    if (evdata && evdata->data && event == FB_EVENT_BLANK )//&&

    {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK)
            bl229x_resume(&g_bl229x->spi->dev);
        else if (*blank == FB_BLANK_POWERDOWN)
            bl229x_suspend(&g_bl229x->spi->dev);
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
static const struct file_operations bl229x_fops = {
    .owner = THIS_MODULE,
    .open  = bl229x_open,
    .write = bl229x_write,
    .read  = bl229x_read,
    .mmap  = bl229x_mmap,
    .release = bl229x_release,
    .unlocked_ioctl = bl229x_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = bl229x_ioctl,
#endif
#ifndef SYS_ANDROID_L
    .fasync = bl229x_async_fasync,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice bl229x_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = SPI_DRV_NAME,
    .fops = &bl229x_fops,
};
static int is_connected(struct bl229x_data *bl229x)
{
	int cur_contrast = 100;
	bl229x_spi_write_reg(REGA_HOST_CMD, MODE_IDLE);
	bl229x_spi_write_reg(REGA_RX_DACP_LOW,cur_contrast);
	BTL_DEBUG("write %d",cur_contrast);
	m_is_conneted = bl229x_spi_read_reg(REGA_RX_DACP_LOW);
	BTL_DEBUG("readback %d",m_is_conneted);
	if(cur_contrast == m_is_conneted)
	{
		BTL_DEBUG("ok %d",bl229x_spi_read_reg(REGA_RX_DACP_LOW));
		return 1;
	}
	BTL_DEBUG("failed %d",bl229x_spi_read_reg(REGA_RX_DACP_LOW));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int  bl229x_probe(struct spi_device *spi)
{
    struct bl229x_data *bl229x = NULL;
    int err = 0;
	
    BTL_DEBUG("  ++\n\n");

	if(finger_is_connect == 1){
		BTL_DEBUG("%s: another fingerprint is connect\n", __func__);
		return -1;
 	}

    bl229x = kzalloc(sizeof(struct bl229x_data),GFP_KERNEL);
    if (!bl229x) {
        return -ENOMEM;
    }
	g_bl229x = bl229x;
    bl229x->image_buf = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));
    bl229x->imagetxcmd = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));;
    bl229x->imagerxpix = (u8*)__get_free_pages(GFP_KERNEL,get_order(READIMAGE_BUF_SIZE));;
   
    memset(bl229x->image_buf,0x00,get_order(READIMAGE_BUF_SIZE));
    memset(bl229x->imagetxcmd,0x00,get_order(READIMAGE_BUF_SIZE));
    memset(bl229x->imagerxpix,0x00,get_order(READIMAGE_BUF_SIZE));
    spi_setup_xfer(NULL);
    if (!bl229x->image_buf) {
        return -ENOMEM;
    }
    spi_set_drvdata(spi,bl229x);


    BTL_DEBUG("step-1\n");

    bl229x_create_inputdev();    
   
    bl229x->spi = spi;
    bl229x->spi->bits_per_word = 8;
    bl229x->spi->mode = SPI_MODE_0;
#if defined(ARCH_MTK_BTL)
    bl229x->spi->controller_data = (void*)&spi_conf;
#endif
    spi_setup(bl229x->spi);

    bl229x_gpio_select_and_init(bl229x);

    //bl229x_power_on(bl229x,1);
	hw_reset(bl229x);

    sema_init(&bl229x->mutex, 1);
	sema_init(&bl229x->handler_mutex, 1);
	mutex_init(&bl229x->spi_lock);
   
   	if(!is_connected(bl229x))
   	{
   		return -1;
   	}
    err = misc_register(&bl229x_misc_device);
    if(err) {
        BTL_DEBUG("bl229x_misc_device register failed\n");
        goto exit_misc_device_register_failed;
    }

	BTL_DEBUG("step-2:%d\n",spi->irq);

    
    //  IRQF_TRIGGER_HIGH | IRQF_ONESHOT 
    err = request_threaded_irq(spi->irq, NULL, bl229x_eint_handler, IRQ_TYPE_EDGE_RISING | IRQF_ONESHOT, SPI_DRV_NAME, bl229x);
	

    bl229x->irq_num = spi->irq;

#if (CHIP_VERSION == 3)
	bl229x->contrast = SENSOR_DEFAULT_CONTRAST_V3;
#elif (CHIP_VERSION == 2)
	bl229x->contrast = SENSOR_DEFAULT_CONTRAST_V2;
#else 
	bl229x->contrast = SENSOR_DEFAULT_CONTRAST_V1;
#endif 

	bl229x->interrupt_mode = INT_MODE_NONE;

	bl229x->report_key = KEY_F10;

	bl229x->report_delay = REPORT_DELAY_TIME;

	bl229x->is_frame_ready = 0;

	bl229x->opened = 0;

	bl229x->fp_detect_method = FP_DETECT_METHOD;

	bl229x->report_timeout = 0;

	atomic_set(&suspended, 0);

    //spi dma or fifo mode
    mtspi_set_dma_en(0);

    //debug \B5\F7\CAԽڵ\E3
    fps_sysfs_init();
   

    fps_create_attributes(&spi->dev);
    //ע\B2\E1˯\C3߻\BD\D0Ѻ\AF\CA\FD
   
    bl229x->fb_notify.notifier_call = fb_notifier_callback;

    err = fb_register_client(&bl229x->fb_notify);

     if (err)
       dev_err(&bl229x->spi->dev, "Unable to register fb_notifier: %d\n",
                err);

    bl229x_dev_init(bl229x);
    bl229x_dev_interrupt_init(bl229x,bl229x->fp_detect_method);


	//kobject_uevent(&spi->dev.kobj, KOBJ_ADD);

	
	wake_lock_init(&fp_suspend_lock, WAKE_LOCK_SUSPEND, "fp_wakelock");

	finger_is_connect = 1;

    BTL_DEBUG("  --\n");
    return 0;

exit_misc_device_register_failed:
    kfree(bl229x);
	printk("%s,probe-error!!!!\n\n",__func__);
    return -1;
}

/*----------------------------------------------------------------------------*/

static struct of_device_id bl229x_match_table[] = {
    {.compatible = "blestech,BL229X",},
    {},
};

/*----------------------------------------------------------------------------*/
static const struct dev_pm_ops bl229x_pm = {
    .suspend = bl229x_suspend,
    .resume = bl229x_resume
};


static struct spi_driver bl229x_driver = {
    .driver = {
        .name	= SPI_DRV_NAME,
        .bus	= &spi_bus_type,
        .owner	= THIS_MODULE,
        .of_match_table = bl229x_match_table,
        //.pm = &bl229x_pm,
    },
    .probe	= bl229x_probe,
};


/*----------------------------------------------------------------------------*/
static int  mt_spi_init(void)
{
    int ret=0;
    BTL_DEBUG("%s",__func__);
    //ret=spi_register_board_info(spi_board_bl229x,ARRAY_SIZE(spi_board_bl229x));
    ret=spi_register_driver(&bl229x_driver);

    return ret;
}


static void  mt_spi_exit(void)
{
    spi_unregister_driver(&bl229x_driver);
}

module_init(mt_spi_init);
module_exit(mt_spi_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("BetterLife BetterLife@blestech.com");
MODULE_DESCRIPTION("BL2x9x fingerprint sensor driver.");

