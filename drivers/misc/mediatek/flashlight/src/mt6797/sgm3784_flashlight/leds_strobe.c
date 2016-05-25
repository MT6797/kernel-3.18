//SGM3784

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/leds.h>


//#define USE_NEW_DRIVER		//add by lijin 2015.5.20
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_HT_strobe"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_ERROR(fmt, arg...)       pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x60


static struct work_struct workTimeOut;

#define GPIO_LED_EN  		90//8
#define GPIO_LED_STROBE  	91//9
#define GPIO_LED_GPIO  		255//10

#define SGM3784_REG_ENABLE      0x0F
#define SGM3784_REG_TIMING      0x02
#define SGM3784_REG_FLASH_LED1  0x06
#define SGM3784_REG_TORCH_LED1  0x08
#define SGM3784_REG_FLASH_LED2  0x09
#define SGM3784_REG_TORCH_LED2  0x0B


/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

int g_Reg_First_write = 0;	//add by lijin 2015.5.13

static struct i2c_client *SGM3784_i2c_client = NULL;

struct SGM3784_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct SGM3784_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct SGM3784_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/
/*
static int SGM3784_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/

static int SGM3784_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int SGM3784_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}




static int SGM3784_chip_init(struct SGM3784_chip_data *chip)
{


	return 0;
}

static int SGM3784_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct SGM3784_chip_data *chip;
	struct SGM3784_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("SGM3784_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "SGM3784 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct SGM3784_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("SGM3784 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct SGM3784_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(SGM3784_chip_init(chip)<0)
		goto err_chip_init;

	SGM3784_i2c_client = client;
	PK_DBG("SGM3784 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("SGM3784 probe is failed \n");
	return -ENODEV;
}

static int SGM3784_remove(struct i2c_client *client)
{
	struct SGM3784_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define SGM3784_NAME "leds-SGM3784"
#define I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR  0x30
static const struct i2c_device_id SGM3784_id[] = {
	{SGM3784_NAME, 0},
	{}
};
#ifdef CONFIG_OF
static const struct of_device_id SGM3784_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif
static struct i2c_driver SGM3784_i2c_driver = {
	.driver = {
		.name  = SGM3784_NAME,
#ifdef CONFIG_OF
		.of_match_table = SGM3784_of_match,
#endif
	},
	.probe	= SGM3784_probe,
	.remove   = SGM3784_remove,
	.id_table = SGM3784_id,
};

//struct SGM3784_platform_data SGM3784_pdata = {0, 0, 0, 0, 0};
//static struct i2c_board_info __initdata i2c_SGM3784={ I2C_BOARD_INFO(SGM3784_NAME, I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR), 
//							.platform_data = &SGM3784_pdata,};

static int __init SGM3784_init(void)
{
	printk("SGM3784_init\n");
	//i2c_register_board_info(2, &i2c_SGM3784, 1);
	//i2c_register_board_info(1, &i2c_SGM3784, 1);


	return i2c_add_driver(&SGM3784_i2c_driver);
}

static void __exit SGM3784_exit(void)
{
	i2c_del_driver(&SGM3784_i2c_driver);
}


module_init(SGM3784_init);
module_exit(SGM3784_exit);

MODULE_DESCRIPTION("Flash driver for SGM3784");
MODULE_AUTHOR("LiJin <lijin@malatamobile.com>");
MODULE_LICENSE("GPL v2");



int readReg(int reg)
{
#if 1
    int val;
    val = SGM3784_read_reg(SGM3784_i2c_client, reg);
    return (int)val;
#else	

    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    PK_DBG("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
#endif	
}

int writeReg(int reg, int val)
{
#if 1
    int err;
    //struct SGM3784_chip_data *chip = i2c_get_clientdata(client);
    PK_DBG("writeReg  ---1--- reg=0x%x,g_Reg_First_write=%d\n",reg,g_Reg_First_write);
    if(0)//(reg == 0x02)
    {
	if(g_Reg_First_write == 0)
        {
    	    err = SGM3784_write_reg(SGM3784_i2c_client, reg, val);
	    PK_DBG("writeReg ---2--- reg=0x%x,g_Reg_First_write=%d\n",reg,g_Reg_First_write); 
	    //mutex_lock(&chip->lock);
            g_Reg_First_write = 1;
	    //mutex_unlock(&chip->lock);
	    return (int)val;
        }
        return 0;
    }
    else
    {
    	    err = SGM3784_write_reg(SGM3784_i2c_client, reg, val);
	    return (int)val;
    }
#else	

    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
    iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);

   return 0;
#endif   
}

#define e_DutyNum 16
#define TORCHDUTYNUM 2
static int isMovieMode[e_DutyNum+1][e_DutyNum+1] = 
{ 
        {-1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//duty1=-1,duty2=-1,0,1
  	{1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//duty1=0, duty2=-1,0,1
  	{1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},//duty1=1, duty2=-1,0,1
  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

//static int torchLED1Reg[e_DutyNum+1] = {0,63,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//93,187

#if defined(USE_NEW_DRIVER)
static int torchLED1Reg[e_DutyNum] = {3,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//56,111,167,204,259,300     //56,111
static int torchLED2Reg[e_DutyNum] = {3,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//56,111,167,204,259,300	//56,111
#else
static int torchLED1Reg[e_DutyNum+1] = {0,3,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//56,111,//167,204,259,300
static int torchLED2Reg[e_DutyNum+1] = {0,3,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//56,111,//167,204,259,300
#endif


//static int flashLED1Reg[e_DutyNum+1] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,111,119,127};//93,187,280,374,468,562,655,749,843,937,1030,1124,1218,1312,1405,1499

#if defined(USE_NEW_DRIVER)
static int flashLED1Reg[e_DutyNum] = {3,6,8,10,13,16,19,22,25,27,30,32,35,38,41,43};//50,111,150,200,250,300,352,407,463,500,555,592,648,703,759 ,796  ,//851,907,944,1086
static int flashLED2Reg[e_DutyNum] = {3,6,8,10,13,16,19,22,25,27,30,32,35,38,41,43};//50,111,150,200,250,300,352,407,463,500,555,592,648,703,759 ,796  ,//851,907,944,1086
#else
static int flashLED1Reg[e_DutyNum+1] = {0,3,6,8,10,13,16,19,22,25,27,30,32,35,38,41,43};//50,100,150,200,250,300,352,407,463,500,555,592,648,703,759 ,796  ,//851,907,944,1086
static int flashLED2Reg[e_DutyNum+1] = {0,3,6,8,10,13,16,19,22,25,27,30,32,35,38,41,43};//50,100,150,200,250,300,352,407,463,500,555,592,648,703,759 ,796  ,//851,907,944,1086
#endif

#if defined(USE_NEW_DRIVER)
static int isMovieModeLED1[e_DutyNum] = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int isMovieModeLED2[e_DutyNum] = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif

int m_duty1=0;
int m_duty2=0;
int LED1Closeflag = 0;
int LED2Closeflag = 0;

int FlashIc_Enable(void)
{
  gpio_set_value(GPIO_LED_EN,1);
	PK_DBG("FlashIc_Enable!\n");
	return 0;
}

int FlashIc_Disable(void)
{
    gpio_set_value(GPIO_LED_EN,0);
	PK_DBG("FlashIc_Disable!\n");
	return 0;
}


int flashEnable_SGM3784_1(void)
{
	//int temp;
	return 0;
}
int flashDisable_SGM3784_1(void)
{
	//int temp;
    return 0;
}

int setDuty_SGM3784_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	
	return 0;
}



int flashEnable_SGM3784_2(void)
{
	//int temp;

	//PK_DBG("flashEnable_SGM3784_2\n");
	PK_DBG("---lijin---flashEnable_SGM3784_2 LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		writeReg(SGM3784_REG_ENABLE, 0x00);//close
		g_Reg_First_write = 0;
		//FlashIc_Disable();			
	}
	else if(LED1Closeflag == 1)	//led1 close
	{
		#if defined(USE_NEW_DRIVER)
		if(isMovieModeLED2[m_duty2] == 1)
		#else
		if(isMovieMode[0][m_duty2+1] == 1)
		#endif
		{
			//writeReg(0x0F, 0x00);
			 gpio_set_value(GPIO_LED_STROBE,0);
			writeReg(0x01, 0xea);	//Assist light mode with continuous LED current //0xea
		
		  writeReg(0x02, 0xd8);	//0xff  //0xdf
			writeReg(0x03, 0x0e);	//0X48  //0x0e
			writeReg(0x08, 0x03);	//led1 flash = 0
			writeReg(0x0b, 0x00);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led2 on
			 gpio_set_value(GPIO_LED_GPIO,1);
			PK_DBG("flashEnable_SGM3784_2 ---1--- LED1Closeflag = 1  led2 torch mode start\n");
		}

		else
		{
			//writeReg(0x0F, 0x00);
			gpio_set_value(GPIO_LED_GPIO,0);
			writeReg(0x01, 0xeb);	//flash mode 电平触发 高触发 硬触发  //0XFB  //0XEB
		
			writeReg(0x02, 0xd8);	//0XEF  //0XDF
			writeReg(0x03, 0x0e);	//0X48  //0X0E
			writeReg(0x06, 0x08);	//led1 flash=0
			writeReg(0x09, 0x00);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led2 on
			 gpio_set_value(GPIO_LED_STROBE,1);
			PK_DBG("flashEnable_SGM3784_2 ---2--- LED1Closeflag = 1  led2 flash mode start\n");
		}
	}

	else if(LED2Closeflag == 1)	//led2 close

	{
		#if defined(USE_NEW_DRIVER)
		if(isMovieModeLED1[m_duty1] == 1)
		#else
		if(isMovieMode[m_duty1+1][0] == 1)
		#endif
		{
			//writeReg(0x0F, 0x00);
			 gpio_set_value(GPIO_LED_STROBE,0);
			writeReg(0x01, 0xea);	//Assist light mode with continuous LED current //0xea
		
		  writeReg(0x02, 0xd8);	//0xff  //0xdf
			writeReg(0x03, 0x0e);	//0X48  //0x0e
			writeReg(0x08, 0x00);	//led1 flash = 0
			writeReg(0x0b, 0x03);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led2 on
			gpio_set_value(GPIO_LED_GPIO,1);
			PK_DBG("flashEnable_SGM3784_2 ---3--- LED2Closeflag = 1  led1 torch mode start\n");
		}			
		else
		{
			//writeReg(0x0F, 0x00);
			gpio_set_value(GPIO_LED_GPIO,0);;
			writeReg(0x01, 0xeb);	//flash mode 电平触发 高触发 硬触发  //0XFB  //0XEB
		
			writeReg(0x02, 0xd8);	//0XEF  //0XDF
			writeReg(0x03, 0x0e);	//0X48  //0X0E
			writeReg(0x06, 0x00);	//led1 flash=0
			writeReg(0x09, 0x08);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1 on
			 gpio_set_value(GPIO_LED_STROBE,1);
			PK_DBG("flashEnable_SGM3784_2 ---4--- LED2Closeflag = 1  led1 flash mode start\n");
		}

	}
	else
	{
		#if defined(USE_NEW_DRIVER)
		if((isMovieModeLED1[m_duty1] == 1)&&(isMovieModeLED2[m_duty2] == 1))
		#else
		if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
		#endif
		{
			//writeReg(0x0F, 0x00);
			 gpio_set_value(GPIO_LED_STROBE,0);
			writeReg(0x01, 0xea);	//Assist light mode with continuous LED current //0xea
		
		  writeReg(0x02, 0xd8);	//0xff  //0xdf
			writeReg(0x03, 0x0e);	//0X48  //0x0e
			writeReg(0x08, 0x3c);	//led1 flash = 0
			writeReg(0x0b, 0x08);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1,led2 on
			gpio_set_value(GPIO_LED_GPIO,1);
			PK_DBG("flashEnable_SGM3784_2  ---5--- led1,led2 torch mode start\n");
		}

		else
		{
			//writeReg(0x0F, 0x00);
			gpio_set_value(GPIO_LED_GPIO,0);;
			writeReg(0x01, 0xeb);	//flash mode 电平触发 高触发 硬触发  //0XFB  //0XEB
		
			writeReg(0x02, 0xd8);	//0XEF  //0XDF
			writeReg(0x03, 0x0e);	//0X48  //0X0E
			writeReg(0x06, 0x3c);	//led1 flash=0
			writeReg(0x09, 0x08);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1,led2 on
			 gpio_set_value(GPIO_LED_STROBE,1);
			PK_DBG("flashEnable_SGM3784_2  ---6--- led1,led2 flash mode start t\n");
		}
	}
	//PK_DBG("---lijin---flashEnable_SGM3784_2 m_duty1=%d,m_duty2=%d,LED1_TORCH=%d,LED1_FLASH=%d,LED2_TORCH=%d,LED2_FLASH=%d\n",m_duty1,m_duty2,readReg(0x08),readReg(0x06),readReg(0x0b),readReg(0x09));
	return 0;
}

int flashDisable_SGM3784_2(void)
{
	int temp;

	//PK_DBG("flashDisable_SGM3784_2\n");
	PK_DBG("---lijin---flashDisable_SGM3784_2 LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		writeReg(SGM3784_REG_ENABLE, 0x00);//close
		g_Reg_First_write = 0;
		//FlashIc_Disable();			
	}
	else if(LED1Closeflag == 1)		//led1 close
	{
		PK_DBG("flashDisable_SGM3784_2 LED1Closeflag = 1 \n");
		//PK_DBG("flashDisable_SGM3784_2 isMovieMode[0][m_duty2+1]=%d \n",isMovieMode[0][m_duty2+1]);
		#if defined(USE_NEW_DRIVER)
		if(isMovieModeLED2[m_duty2] == 1)
		#else
		if(isMovieMode[0][m_duty2+1] == 1)
		#endif
		{
			//writeReg(0x0F, 0x00);
			 gpio_set_value(GPIO_LED_STROBE,0);
			writeReg(0x01, 0xea);	////Assist light mode with continuous LED current //0xea
			#if 1
			temp = readReg(0x02);
			if(temp != 0xFF)
			{
				writeReg(0x02, 0xFF);	
			}
			#else
		        writeReg(0x02, 0xFF);	//0xff  //0xdf
			#endif
			writeReg(0x03, 0x48);	//0X48  //0x0e
			writeReg(0x06, 0x00);	//led1 flash=0
			writeReg(0x08, 0x00);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led2 on
			gpio_set_value(GPIO_LED_GPIO,1);
			PK_DBG("flashDisable_SGM3784_2 ---1--- LED1Closeflag = 1  led2 torch mode start temp=0x%x\n",temp);
		}

		else
		{
			//writeReg(0x0F, 0x00);
			gpio_set_value(GPIO_LED_GPIO,0);
			writeReg(0x01, 0xFB);	//flash mode 电平触发 高触发 硬触发  //0XFB  //0XEB
			#if 1
			temp = readReg(0x02);
			if(temp != 0xCF)
			{
				writeReg(0x02, 0xCF);	
			}
			#else
			writeReg(0x02, 0xCF);	//0XEF  //0XDF
			#endif
			writeReg(0x03, 0x48);	//0X48  //0X0E
			writeReg(0x06, 0x00);	//led1 flash=0
			writeReg(0x08, 0x00);	//led1 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led2 on
			 gpio_set_value(GPIO_LED_STROBE,1);
			PK_DBG("flashDisable_SGM3784_2  ---2--- LED1Closeflag = 1  led2 flash mode start temp=0x%x\n",temp);
		};
	}
	else if(LED2Closeflag == 1)		//led2 close
	{
		PK_DBG("flashDisable_SGM3784_2 LED2Closeflag = 1 \n");
		//PK_DBG("flashDisable_SGM3784_2 isMovieMode[m_duty1+1][0]=%d \n",isMovieMode[m_duty1+1][0]);
		#if defined(USE_NEW_DRIVER)
		if(isMovieModeLED1[m_duty1] == 1)
		#else
		if(isMovieMode[m_duty1+1][0] == 1)
		#endif
		{
			//writeReg(0x0F, 0x00);
			 gpio_set_value(GPIO_LED_STROBE,0);
			writeReg(0x01, 0xea);	//Assist light mode with continuous LED current //0xea
			#if 1
			temp = readReg(0x02);
			if(temp != 0xFF)
			{
				writeReg(0x02, 0xFF);	
			}
			#else
		        writeReg(0x02, 0xFF);	//0xff  //0xdf
			#endif
			writeReg(0x03, 0x48);	//0X48  //0x0e
			writeReg(0x09, 0x00);	//led2 flash=0
			writeReg(0x0b, 0x00);	//led2 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1 on
			gpio_set_value(GPIO_LED_GPIO,1);
			PK_DBG("flashDisable_SGM3784_2  ---3--- LED2Closeflag = 1  led1 torch mode start temp=0x%x\n",temp);
		}

		else
		{
			//writeReg(0x0F, 0x00);
			gpio_set_value(GPIO_LED_GPIO,0);
			writeReg(0x01, 0xFB);	//flash mode 电平触发 高触发 硬触发  //0XFB  //0XEB
			#if 1
			temp = readReg(0x02);
			if(temp != 0xCF)
			{
				writeReg(0x02, 0xCF);	
			}
			#else
			writeReg(0x02, 0xCF);	//0XEF  //0XDF
			#endif
			writeReg(0x03, 0x48);	//0X48  //0X0E
			writeReg(0x09, 0x00);	//led2 flash=0
			writeReg(0x0b, 0x00);	//led2 torch =0
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1 on
			 gpio_set_value(GPIO_LED_STROBE,1);
			PK_DBG("flashDisable_SGM3784_2  ---4--- LED2Closeflag = 1  led1 flash mode start temp=0x%x\n",temp);
		}

	}
	else
	{
		#if defined(USE_NEW_DRIVER)
		if((isMovieModeLED1[m_duty1] == 1)&&(isMovieModeLED2[m_duty2] == 1))
		#else
		if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
		#endif
		{
			//writeReg(0x0F, 0x00);
			 gpio_set_value(GPIO_LED_STROBE,0);
			writeReg(0x01, 0xea);	//Assist light mode with continuous LED current //0xea
			#if 1
			temp = readReg(0x02);
			if(temp != 0xFF)
			{
				writeReg(0x02, 0xFF);	
			}
			#else
		        writeReg(0x02, 0xFF);	//0xff  //0xdf
			#endif
			writeReg(0x03, 0x48);	//0X48  //0x0e
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1,led2 on
			gpio_set_value(GPIO_LED_GPIO,1);
			PK_DBG("flashDisable_SGM3784_2  ---5--- led1,led2 torch mode start temp=0x%x\n",temp);
		}

		else
		{
			//writeReg(0x0F, 0x00);
			gpio_set_value(GPIO_LED_GPIO,0);
			writeReg(0x01, 0xFB);	//flash mode 电平触发 高触发 硬触发  //0XFB  //0XEB
			#if 1
			temp = readReg(0x02);
			if(temp != 0xCF)
			{
				writeReg(0x02, 0xCF);	
			}
			#else
			writeReg(0x02, 0xCF);	//0XEF  //0XDF
			#endif
			writeReg(0x03, 0x48);	//0X48  //0X0E
			writeReg(SGM3784_REG_ENABLE, 0x03);	//led1,led2 on
			 gpio_set_value(GPIO_LED_STROBE,1);
			PK_DBG("flashDisable_SGM3784_2 ---6---  led1,led2 flash mode start temp=0x%x\n",temp);
		}

	}

	//PK_DBG("---lijin---flashDisable_SGM3784_2 m_duty1=%d,m_duty2=%d,LED1_TORCH=%d,LED1_FLASH=%d,LED2_TORCH=%d,LED2_FLASH=%d\n",m_duty1,m_duty2,readReg(0x08),readReg(0x06),readReg(0x0b),readReg(0x09));
	return 0;
}


int setDuty_SGM3784_2(int duty)
{
/*
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;
*/
	PK_DBG("setDuty_SGM3784_2:m_duty1 = %d, m_duty2 = %d!\n", m_duty1, m_duty2);
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		
	}
	else if(LED1Closeflag == 1)		//led1 close
	{
		#if defined(USE_NEW_DRIVER)
		if(isMovieModeLED2[m_duty2] == 1)
		#else
		if(isMovieMode[0][m_duty2+1] == 1)		//TORCH
		#endif
		{
			#if defined(USE_NEW_DRIVER)
			writeReg(SGM3784_REG_TORCH_LED1,torchLED1Reg[0]);		//write value to led1 touch mode
			writeReg(SGM3784_REG_TORCH_LED2,torchLED2Reg[m_duty2]);		//write value to led2 touch mode
			PK_DBG("setDuty_SGM3784_2:----1----SGM3784_REG_TORCH_LED1 = %d, SGM3784_REG_TORCH_LED2 = %d\n", torchLED1Reg[0], torchLED1Reg[m_duty2]);
			#else
			writeReg(SGM3784_REG_TORCH_LED1,torchLED1Reg[0]);		//write value to led1 touch mode
			writeReg(SGM3784_REG_TORCH_LED2,torchLED2Reg[m_duty2+1]);	//write value to led2 touch mode
			PK_DBG("setDuty_SGM3784_2:----1----SGM3784_REG_TORCH_LED1 = %d, SGM3784_REG_TORCH_LED2 = %d\n", torchLED1Reg[0], torchLED1Reg[m_duty2+1]);
			#endif
		}
		else									//FLASH
		{
			#if defined(USE_NEW_DRIVER)
			writeReg(SGM3784_REG_FLASH_LED1,flashLED1Reg[0]);			//write value to led1 flash mode
			writeReg(SGM3784_REG_FLASH_LED2,flashLED2Reg[m_duty2]);			//write value to led2 flash mode
			PK_DBG("setDuty_SGM3784_2:----2----SGM3784_REG_FLASH_LED1 = %d, SGM3784_REG_FLASH_LED2 = %d\n", flashLED1Reg[0], flashLED1Reg[m_duty2]);
			#else
			writeReg(SGM3784_REG_FLASH_LED1,flashLED1Reg[0]);			//write value to led1 flash mode
			writeReg(SGM3784_REG_FLASH_LED2,flashLED2Reg[m_duty2+1]);	//write value to led2 flash mode
			PK_DBG("setDuty_SGM3784_2:----2----SGM3784_REG_FLASH_LED1 = %d, SGM3784_REG_FLASH_LED2 = %d\n", flashLED1Reg[0], flashLED1Reg[m_duty2+1]);
			#endif			
		}
	}
	else if(LED2Closeflag == 1)		//led2 close
	{
		#if defined(USE_NEW_DRIVER)
		if(isMovieModeLED1[m_duty1] == 1)
		#else
		if(isMovieMode[m_duty1+1][0] == 1)
		#endif
		{
			#if defined(USE_NEW_DRIVER)
			writeReg(SGM3784_REG_TORCH_LED1,torchLED1Reg[m_duty1]);				//write value to led1 touch mode
			writeReg(SGM3784_REG_TORCH_LED2,torchLED2Reg[0]);				//write value to led2 touch mode
			PK_DBG("setDuty_SGM3784_2:----3----SGM3784_REG_TORCH_LED1 = %d, SGM3784_REG_TORCH_LED2 = %d\n", torchLED1Reg[m_duty1], torchLED1Reg[0]);
			#else
			writeReg(SGM3784_REG_TORCH_LED1,torchLED1Reg[m_duty1+1]);			//write value to led1 touch mode
			writeReg(SGM3784_REG_TORCH_LED2,torchLED2Reg[0]);				//write value to led2 touch mode
			PK_DBG("setDuty_SGM3784_2:----3----SGM3784_REG_TORCH_LED1 = %d, SGM3784_REG_TORCH_LED2 = %d\n", torchLED1Reg[m_duty1+1], torchLED1Reg[0]);
			#endif
		}
		else
		{
			#if defined(USE_NEW_DRIVER)	
			writeReg(SGM3784_REG_FLASH_LED1,flashLED1Reg[m_duty1]);				//write value to led1 flash mode
			writeReg(SGM3784_REG_FLASH_LED2,flashLED2Reg[0]);				//write value to led2 flash mode
			PK_DBG("setDuty_SGM3784_2:----4----SGM3784_REG_FLASH_LED1 = %d, SGM3784_REG_FLASH_LED2 = %d\n", flashLED1Reg[m_duty1], flashLED1Reg[0]);
			#else
			writeReg(SGM3784_REG_FLASH_LED1,flashLED1Reg[m_duty1+1]);			//write value to led1 flash mode
			writeReg(SGM3784_REG_FLASH_LED2,flashLED2Reg[0]);				//write value to led2 flash mode			
			PK_DBG("setDuty_SGM3784_2:----4----SGM3784_REG_FLASH_LED1 = %d, SGM3784_REG_FLASH_LED2 = %d\n", flashLED1Reg[m_duty1+1], flashLED1Reg[0]);
			#endif
		}		
	}
	else
	{
		#if defined(USE_NEW_DRIVER)
		if((isMovieModeLED1[m_duty1] == 1)&&(isMovieModeLED2[m_duty2] == 1))
		#else
		if(isMovieMode[m_duty1+1][m_duty2+1] == 1)		//touch mode
		#endif
		{
			#if defined(USE_NEW_DRIVER)
			writeReg(SGM3784_REG_TORCH_LED1,torchLED1Reg[m_duty1]);			//write value to led1 touch mode
			writeReg(SGM3784_REG_TORCH_LED2,torchLED2Reg[m_duty2]);			//write value to led2 touch mode
			PK_DBG("setDuty_SGM3784_2:----5----SGM3784_REG_TORCH_LED1 = %d, SGM3784_REG_TORCH_LED2 = %d\n", torchLED1Reg[m_duty1], torchLED1Reg[m_duty2]);
			#else
			writeReg(SGM3784_REG_TORCH_LED1,torchLED1Reg[m_duty1+1]);			//write value to led1 touch mode
			writeReg(SGM3784_REG_TORCH_LED2,torchLED2Reg[m_duty2+1]);			//write value to led2 touch mode
			PK_DBG("setDuty_SGM3784_2:----5----SGM3784_REG_TORCH_LED1 = %d, SGM3784_REG_TORCH_LED2 = %d\n", torchLED1Reg[m_duty1+1], torchLED1Reg[m_duty2+1]);
			#endif
		}
		else											//flash mode
		{	
			#if defined(USE_NEW_DRIVER)
			writeReg(SGM3784_REG_FLASH_LED1,flashLED1Reg[m_duty1]);			//write value to led1 flash mode
			writeReg(SGM3784_REG_FLASH_LED2,flashLED2Reg[m_duty2]);
			PK_DBG("setDuty_SGM3784_2:----6----SGM3784_REG_FLASH_LED1 = %d, SGM3784_REG_FLASH_LED2 = %d\n", flashLED1Reg[m_duty1], flashLED1Reg[m_duty2]);
			#else
			writeReg(SGM3784_REG_FLASH_LED1,flashLED1Reg[m_duty1+1]);			//write value to led1 flash mode
			writeReg(SGM3784_REG_FLASH_LED2,flashLED2Reg[m_duty2+1]);
			PK_DBG("setDuty_SGM3784_2:----6----SGM3784_REG_FLASH_LED1 = %d, SGM3784_REG_FLASH_LED2 = %d\n", flashLED1Reg[m_duty1+1], flashLED1Reg[m_duty2+1]);
			#endif
		}
	}

	return 0;
}


int FL_Enable(void)
{

	PK_DBG(" FL_Enable line=%d\n",__LINE__);


    return 0;
}



int FL_Disable(void)
{
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_SGM3784_1(duty);

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
	//int ret;
	PK_DBG("LED1_FL_Init!\n");

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	PK_DBG("LED1_FL_Uninit!\n");
	FlashIc_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("LED1TimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("SGM3784_LED1_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%ld\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %ld\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %ld\n",arg);
		m_duty1 = arg;
/*
		if(m_duty1<0)
			m_duty1=0;
		else if(m_duty1>=e_DutyNum)
			m_duty1=e_DutyNum-1;
*/
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %ld\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF11111: %ld\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
				LED1Closeflag = 0;
    			//FL_Enable();
    			FlashIc_Enable();
    			flashEnable_SGM3784_2();
    		}
    		else
    		{
    			LED1Closeflag = 1;
    			FlashIc_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
    	    break;



		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


