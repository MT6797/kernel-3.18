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
	{REGFLAG_DELAY, 30, {} },
	{0x10, 0, {} },
	{REGFLAG_DELAY, 100, {} },
	{0xB0,1,{0x04}},
	{0xB1,1,{0x01}},
	{REGFLAG_DELAY, 100, {} }
};

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xB0,1,{0x04}},
	{0xD6,1,{0x01}},
	{0xB3,6,{0x14,0x00,0x00,0x00,0x00,0x00}},
	{0xB4,2,{0x0C,0x00}},
	{0xB6,3,{0x4B,0xDB,0x00}},
	{0xC1,35,{0x04,0x60,0x00,0x40,0x10,
					0x00,0x58,0x03,0x00,0x00,
					0x00,0x64,0x84,0x30,0x4A,
					0x00,0x00,0x9D,0xC1,0x01,
					0x00,0xCA,0x00,0x00,0x00,
					0x00,0x00,0x00,0x42,0x00,
					0x02,0x20,0x03,0x98,0x11}},
	{0xC2,8,{0x31,0xF7,0x80,0x08,0x08,0x00,0x00,0x08}},
	{0xC4,11,{0x70,0x00,0x00,0x00,0x00,
					0x00,0x00,0x00,0x00,0x01,0x06}},
	{0xC6,21,{0xC8,0x01,0x6E,0x02,0x67,
					0x00,0x00,0x00,0x00,0x00,
					0x00,0x00,0x00,0x00,0x00,
					0x00,0x00,0x04,0x19,0x0A,0xC8}},
	{0xC7,30,{0x06,0x15,0x1C,0x26,0x34,0x42,
						0x4C,0x5B,0x3F,0x46,0x52,0x60,
						0x68,0x6E,0x7A,0x06,0x15,0x1C,
						0x26,0x34,0x42,0x4C,0x5B,0x3F,
						0x46,0x52,0x60,0x68,0x6E,0x7A}},
	{0xCB,15,{0xE7,0xE0,0xC7,0xE3,0x00,
					0x00,0x00,0x00,0x20,0xE0,
					0x87,0x00,0xE8,0x00,0x00}},
	{0xCC,1,{0x06}},
	{0xD0,10,{0x11,0x00,0x00,0x58,0xCA,0x40,0x19,0x19,0x09,0x00}},
	{0xD1,4,{0x00,0x48,0x16,0x0F}},
	{0xD3,26,{0x1B,0x33,0x99,0xBB,0xB3,
					0x33,0x33,0x33,0x11,0x00,
					0x01,0x00,0x00,0x78,0xA0,
					0x01,0x2F,0x2F,0x33,0x33,
					0x72,0x12,0x8A,0x57,0x3D,0xBC}}, 
	{0xD5,7,{0x06,0x00,0x00,0x01,0x2e,0x01,0x2e}},
	 {REGFLAG_DELAY, 10, {}},      
         
	{0x35,1,{0x00}},
         					
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 30, {}},
	{0x11,1,{0x00}},	
	{REGFLAG_DELAY, 120, {}},
	
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

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 6;
	params->dsi.vertical_frontporch = 8;
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
    	dprintf(0, "[LK]r63350----lcm_gate----cmd=%0x--i2c write error----\n",cmd);
#else
		printk("[KERNEL]r63350----lcm_gate---cmd=%0x-- i2c write error-----\n",cmd);
#endif 		    	
	}

	cmd=0x01;
	data=0x0E;
	ret=lcm_gate_write_bytes(cmd,data);
	if(ret)
	{    
#ifdef BUILD_LK		
    	dprintf(0, "[LK]r63350----lcm_gate----cmd=%0x--i2c write error----\n",cmd);
#else
		printk("[KERNEL]r63350----lcm_gate---cmd=%0x-- i2c write error-----\n",cmd);
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
	MDELAY(10);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(10);
	
	gpio_set_value(LCD_BIAS_EN_PIN, 0);
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

#define LCM_ID (0x5033)

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

	array[0] = 0x00053700;	 
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xBF, buffer, 5);
	
    id = buffer[0]>>16; 
	dprintf(0, "%s, LK r63350 debug: r63350 id = 0x%08x\n", __func__, id);
	
#else
	gpio_set_value(LCD_BIAS_EN_PIN, 1);	
	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(10);

	gpio_set_value(LCM_RESET_PIN, 0);
	MDELAY(50);

	gpio_set_value(LCM_RESET_PIN, 1);
	MDELAY(50);
	/*****no need to read id in kernel*****/
	printk("%s, Kernel  read r63350 id but do not thing\n", __func__);
#endif
	
	if (id == LCM_ID)
		return 1;
	else
		return 0;

}

LCM_DRIVER boyi_r63350_fhd_dsi_vdo_lcm_drv = {
	.name = "boyi_r63350_fhd_dsi_vdo",
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
