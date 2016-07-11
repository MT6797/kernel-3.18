#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/* #include <mach/mt_pm_ldo.h> */
/* #include <mach/mt_gpio.h> */
#endif

#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)			(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))

/* --------------------------------------------------------------------------- */
/* Local Functions */
/* --------------------------------------------------------------------------- */
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) \
	lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
	lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
	lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
	lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int lcm_compare_id(void);


static const unsigned char LCD_MODULE_ID = 0x01;
/* --------------------------------------------------------------------------- */
/* Local Constants */
/* --------------------------------------------------------------------------- */
#define LCM_DSI_CMD_MODE								0	/* 1 */
#define FRAME_WIDTH										(1080)
#define FRAME_HEIGHT									(1920)

#define REGFLAG_DELAY									0xFC
#define REGFLAG_END_OF_TABLE							0xFD

#ifdef BUILD_LK
#define LCD_BIAS_EN_PIN 				GPIO65
#define LCM_RESET_PIN 					GPIO180

#else

#define LCD_BIAS_EN_PIN				65
#define LCM_RESET_PIN 					180

#endif



#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/* --------------------------------------------------------------------------- */
/* Local Variables */
/* --------------------------------------------------------------------------- */

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[128];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28, 0, {} },
	{REGFLAG_DELAY, 30, {}},
	{0x10, 0, {} },
	{REGFLAG_DELAY, 150, {} }
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	//============9885 MTP Flow Header========
	{0xB0,3,{0x98,0x85,0x0A}},

	//============9885 initial Code========
	{0xC4,7,{0x70,0x19,0x23,0x00,0x0F,0x0F,0x00}},
	{0xD0,6,{0x33,0x03,0x34,0x6B,0x67,0xC0}}, //VGH=3AVDD-AVEE=16V(single),0x VGL=2AVEE-AVDD=-13V(single)  P6  VGLO sel@Slp in/out
	{0xD2,4,{0x13,0x13,0xEA,0x22}}, // P1/P2 :Enable AVDDR/AVEER  P3 VGHO sel@Slp in/out  P4  VGLO sel@Slp in/out    VGH/VGHR&VGL/VGLR
	{0xD3,9,{0x33,0x33,0x05,0x03,0x59,0x59,0x22,0x26,0x22}}, //GVDDP=4.3V GVDDN=-4.3V VGHO=15 VGLO=-12 AVDDR=4.7V AVEER=-4.7V
	{0xD5,10,{0x8B,0x00,0x00,0x00,0x01,0x87,0x01,0x87,0x01,0x87}}, //set Vcom,0x register be accessble

	{0xEC,7,{0x76,0x1E,0x32,0x00,0x46,0x00,0x02}}, //black display while video stop
	{0xEF,8,{0x3C,0x05,0x52,0x13,0xE1,0x33,0x5b,0x09}}, 
	{0xD6,12,{0x00,0x00,0x08,0x17,0x23,0x65,0x77,0x44,0x87,0x00,0x00,0x09}},
	//set LVD sequence
	{0xEB,35,{0x9b,0xC7,0x73,0x00,0x58,0x55,0x55,0x55,0x55,0x54,0x00,0x00,0x00,0x00,0x00,0x25,0x4D,0x0F,0xFF,0xFF,0xFF,0xFF,0xFF,0x55,0x55,0x55,0x55,0x32,0x77,0x55,0x43,0x55,0x5E,0xFF,0x55}},    

	//GIP setting 
	{0xE5,73,{0x36,0x36,0xA1,0xF6,0xF6,0x47,0x07,0x55,0x15,0x63,0x23,0x71,0x31,0x6E,0x36,0x85,0x36,0x36,0x36,0x36,0x36,0x36,0xA8,0xF6,0xF6,0x4E,0x0E,0x5C,0x1C,0x6A,0x2A,0x78,0x38,0x76,0x35,0x8C,0x36,0x36,0x36,0x36,0x18,0x70,0x61,0x00,0x4E,0xBB,0x70,0x80,0x00,0x4E,0xBB,0xF7,0x00,0x4E,0xBB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07}},
	{0xEA,66,{0x51,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x00,0x00,0x00,0x96,0x95,0x10,0x11,0x00,0x0A,0x00,0x0F,0x00,0x00,0x00,0x70,0x01,0x10,0x00,0x40,0x80,0xC0,0x00,0x00,0x01,0x23,0x45,0x67,0x89,0xAB,0xCD,0xEF,0xCC,0xCC,0x22,0x33,0x33,0x00,0x11,0x00,0x11,0x00,0x11,0x00,0x11,0xCC,0xDD,0x22,0xCC,0xCC,0xCC,0xCC}},
	{0xED,23,{0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40}}, 
	{0xEE,19,{0x22,0x10,0x02,0x02,0x0F,0x40,0x00,0x07,0x00,0x04,0x00,0x00,0xC0,0xB9,0x77,0x00,0x55,0x05,0x1F}},

	//gamma setting

	//IS compensation Vcom 18C->1A3  -0.2V @L255 1hrs
	{0xC7,122,{0x00,0x77,0x00,0x77,0x00,0xA5,0x00,0xC2,0x00,0xDA,0x00,0xEE,0x00,0xFF,0x01,0x0E,0x01,0x1B,0x01,0x47,0x01,0x6B,0x01,0xA1,0x01,0xCC,0x02,0x0F,0x02,0x46,0x02,0x47,0x02,0x7A,0x02,0xB2,0x02,0xD6,0x03,0x06,0x03,0x25,0x03,0x4C,0x03,0x58,0x03,0x65,0x03,0x74,0x03,0x83,0x03,0x95,0x03,0xAB,0x03,0xC4,0x03,0xD0,0x00,0x18,0x00,0x77,0x00,0xA5,0x00,0xC2,0x00,0xDA,0x00,0xEE,0x00,0xFF,0x01,0x0E,0x01,0x1B,0x01,0x47,0x01,0x6B,0x01,0xA1,0x01,0xCC,0x02,0x0F,0x02,0x46,0x02,0x47,0x02,0x7A,0x02,0xB2,0x02,0xD6,0x03,0x06,0x03,0x25,0x03,0x4C,0x03,0x58,0x03,0x65,0x03,0x74,0x03,0x83,0x03,0x95,0x03,0xAB,0x03,0xC4,0x03,0xD0,0x01,0x01}},


	{0xB0,1,{0x11}},					
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},	
	{REGFLAG_DELAY, 20, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#ifdef BUILD_LK
static struct LCM_setting_table page1_select[] = {
	//CMD_Page 1
	{0xFF, 3,{0x98,0x85,0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			if (table[i].count <= 10)
				MDELAY(table[i].count);
			else
				MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/* --------------------------------------------------------------------------- */
/* LCM Driver Implementations */
/* --------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = SYNC_PULSE_VDO_MODE;
#endif

	params->dsi.LANE_NUM = LCM_FOUR_LANE;

	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.packet_size = 256;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 8;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 24;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 6;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	/* params->dsi.ssc_disable                                                       = 1; */
	params->physical_width  = 68000; 
	params->physical_height = 122000;
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 200;
#else
	params->dsi.PLL_CLOCK = 450;
#endif
}

#ifdef BUILD_LK
#define LCM_GATE_SLAVE_ADDR_WRITE  0x7C  //0x3e
#define I2C_I2C_LCD_BIAS_CHANNEL 0
static struct mt_i2c_t LCM_GATE_i2c;

static int lcm_gate_write_bytes(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    LCM_GATE_i2c.id = I2C_I2C_LCD_BIAS_CHANNEL;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    LCM_GATE_i2c.addr = (LCM_GATE_SLAVE_ADDR_WRITE >> 1);
    LCM_GATE_i2c.mode = ST_MODE;
    LCM_GATE_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&LCM_GATE_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else
extern int lcm_gate_write_bytes(unsigned char addr, unsigned char value);
#endif

static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
#ifdef BUILD_LK
	mt_set_gpio_mode(LCD_BIAS_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BIAS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BIAS_EN_PIN, GPIO_OUT_ONE);	

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(1);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(20);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(100);
	
#else
	gpio_set_value(LCD_BIAS_EN_PIN, 1);	
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(1);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(20);

	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(100);
#endif
/*------LCM Gate Drive  -----*/
	cmd=0x00;
	data=0x0E;
	ret=lcm_gate_write_bytes(cmd,data);
	if(ret)
	{    
#ifdef BUILD_LK		
    	dprintf(0, "[LK]ili9885----lcm_gate----cmd=%0x--i2c write error----\n",cmd);
#else
		printk("[KERNEL]ili9885----lcm_gate---cmd=%0x-- i2c write error-----\n",cmd);
#endif 		    	
	}

	cmd=0x01;
	data=0x0E;
	ret=lcm_gate_write_bytes(cmd,data);
	if(ret)
	{    
#ifdef BUILD_LK		
    	dprintf(0, "[LK]ili9885----lcm_gate----cmd=%0x--i2c write error----\n",cmd);
#else
		printk("[KERNEL]ili9885----lcm_gate---cmd=%0x-- i2c write error-----\n",cmd);
#endif 		    	
	}

	/*----------------	-----*/
	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_suspend_setting,
		   sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK

	mt_set_gpio_mode(LCD_BIAS_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BIAS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BIAS_EN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(10);

#else
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(20);

	//gpio_set_value(LCM_RESET_PIN, 0);
	//MDELAY(10);
	
	gpio_set_value(LCD_BIAS_EN_PIN, 0);
	MDELAY(10);
#endif
}

static void lcm_resume(void)
{
	lcm_init();
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

#define LCM_ID (0x98)

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	
#ifdef BUILD_LK
	unsigned int buffer[5];
	unsigned int array[16];
	
	mt_set_gpio_mode(LCD_BIAS_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCD_BIAS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCD_BIAS_EN_PIN, GPIO_OUT_ONE);

	mt_set_gpio_mode(LCM_RESET_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(LCM_RESET_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(10);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ZERO);
	MDELAY(50);
	
	mt_set_gpio_out(LCM_RESET_PIN, GPIO_OUT_ONE);
	MDELAY(50);

	//push_table(page1_select, sizeof(page1_select) / sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00013700;	 
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xBF, buffer, 1);
	
    id = buffer[0]; 
	dprintf(0, "%s, LK ili9885 debug: ili9885 id = 0x%08x\n", __func__, id);
	
#else
	gpio_set_value(LCD_BIAS_EN_PIN, 1);	
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(10);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(50);

	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(50);
	/*****no need to read id in kernel*****/
	printk("%s, Kernel  read ili9885 id but do not thing\n", __func__);
#endif

	if (id == LCM_ID)
		return 1;
	else
		return 0;

}

LCM_DRIVER lide_ili9885_fhd_dsi_vdo_lcm_drv = {
	.name = "lide_ili9885_fhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
};
