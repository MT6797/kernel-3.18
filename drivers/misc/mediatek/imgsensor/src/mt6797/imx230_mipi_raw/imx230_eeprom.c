#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include "kd_camera_typedef.h"

#define PFX "IMX230_pdafotp"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define IMX230_EEPROM_READ_ID  0xA0
#define IMX230_EEPROM_WRITE_ID   0xA1
#define IMX230_I2C_SPEED        100
#define IMX230_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048

BYTE IMX230_DCC_data[96]= {0};
BYTE IMX230_SPC_data[352]= {0};
BYTE IMX230_AWB_data[30]= {0};
BYTE IMX230_LSC_data[800]= {0};

static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;

static void imx230_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > IMX230_MAX_OFFSET)
        return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, IMX230_EEPROM_READ_ID)<0)
		return false;
    return true;
}

static bool _read_imx230_eeprom(kal_uint16 addr, BYTE* data, int size ){
	int i = 0;
	int offset = addr;
	LOG_INF("enter _read_eeprom size = %d\n",size);
	for(i = 0; i < size; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			return false;
		}
		LOG_INF("read_eeprom 0x%0x %d\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}


void read_imx230_LSC(BYTE* data){

	int addr = 0x0023;
	int size = 706;
	int checksum = 0;
	int i;

	LOG_INF("read imx230 SPC, size = %d\n", size);
#if 1
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx230_eeprom(addr, IMX230_LSC_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			//return false;
		}
	}
#endif
	for(i=1;i<705;i++)
	checksum += IMX230_LSC_data[i];
	for(i=0;i<704;i++)
	imx230_write_cmos_sensor(0x7800+i,IMX230_LSC_data[i+1]);
	
	imx230_write_cmos_sensor(0x0220,0x00);
	imx230_write_cmos_sensor(0x0B00,0x01);
	imx230_write_cmos_sensor(0x6303,0x01);
	checksum = checksum%255;
printk("read imx230 lSC,%x %x checksum = %d\n", IMX230_LSC_data[0],IMX230_LSC_data[705],checksum);
	//memcpy(data, IMX230_SPC_data , size);
    //return true;
}
void read_imx230_AWB(BYTE* data){

	int addr = 0x0;
	int size = 25;
	kal_uint16  RG = 0,BG = 0,typical_rg = 0,typical_bg=0;
	kal_uint16 BoverG_dec,RoverG_dec_base,BoverG_dec_base,RoverG_dec;			
	kal_uint16 R_test,B_test,G_test;
	kal_uint16 R_test_H8,R_test_L8,B_test_H8,B_test_L8,G_test_H8,G_test_L8;
	kal_uint32 G_test_R, G_test_B;

	printk("read imx230 SPC, size = %d\n", size);
#if 1
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx230_eeprom(addr, IMX230_AWB_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			//return false;
		}
	}
#endif
	RG = ((IMX230_AWB_data[10] &0xff)<< 8) | (IMX230_AWB_data[11] & 0xff) ;
	BG = ((IMX230_AWB_data[12] &0xff)<< 8) | (IMX230_AWB_data[13] & 0xff) ;
	
	typical_rg = ((IMX230_AWB_data[16] &0xff)<< 8) | (IMX230_AWB_data[17] & 0xff) ;
	typical_bg = ((IMX230_AWB_data[18] &0xff)<< 8) | (IMX230_AWB_data[19] & 0xff) ;
	printk("read_imx230_AWB %x %x %x %x IMX230_AWB_data[10] = %x,IMX230_AWB_data[11] = %x\n",RG,BG,IMX230_AWB_data[12],IMX230_AWB_data[13],IMX230_AWB_data[10],IMX230_AWB_data[11]);
	
	BoverG_dec = BG;
	BoverG_dec_base = 0x254;
	RoverG_dec = RG;
	RoverG_dec_base = 0x1f9;
if(BoverG_dec < BoverG_dec_base)
	{
		if (RoverG_dec < RoverG_dec_base)
		{
			G_test = 0x100;
			B_test = 0x100 * BoverG_dec_base / BoverG_dec;
			R_test = 0x100 * RoverG_dec_base / RoverG_dec;
		}
		else
		{
			R_test = 0x100;
			G_test = 0x100 * RoverG_dec / RoverG_dec_base;
			B_test = G_test * BoverG_dec_base / BoverG_dec;
		}
	}
	else
	{
		if (RoverG_dec < RoverG_dec_base)
		{
			B_test = 0x100;
			G_test = 0x100 * BoverG_dec / BoverG_dec_base;
			R_test = G_test * RoverG_dec_base / RoverG_dec;
		}
		else
		{
			G_test_B = BoverG_dec * 0x100 / BoverG_dec_base;
			G_test_R = RoverG_dec * 0x100 / RoverG_dec_base;
			if(G_test_B > G_test_R )
			{
				B_test = 0x100;
				G_test = G_test_B;
				R_test = G_test_B * RoverG_dec_base / RoverG_dec;
			}
			else
			{
				R_test = 0x100;
				G_test = G_test_R;
				B_test = G_test_R * BoverG_dec_base / BoverG_dec;
			}
		}
	}
	if(R_test < 0x100)
	{
		R_test = 0x100;
	}
	if(G_test < 0x100)
	{
		G_test = 0x100;
	}
	if(B_test < 0x100)
	{
		B_test = 0x100;
	}
	R_test_H8 =( R_test>>8)&0xFF;
	R_test_L8 = R_test &0xFF;
	B_test_H8 = (B_test>>8)&0xFF;
	B_test_L8 = B_test &0xFF;
	G_test_H8 = (G_test>>8)&0xFF;
	G_test_L8 = G_test &0xFF;
	
	//reset the digital gain
	imx230_write_cmos_sensor(0x020e,G_test_H8);
	imx230_write_cmos_sensor(0x020F,G_test_L8);
	imx230_write_cmos_sensor(0x0210,R_test_H8);
	imx230_write_cmos_sensor(0x0211,R_test_L8);
	imx230_write_cmos_sensor(0x0212,B_test_H8);
	imx230_write_cmos_sensor(0x0213,B_test_L8);
	imx230_write_cmos_sensor(0x0214,G_test_H8);
	imx230_write_cmos_sensor(0x0215,G_test_L8);
	
	printk("R_test=0x%x,G_test=0x%x,B_test=0x%x",R_test,G_test,B_test);
	//memcpy(data, IMX230_AWB_data , size);
    //return true;
}
void read_imx230_SPC(BYTE* data){
	//int i;
	int addr = 0x301;
	int size = 0x460-0x301+1;//352
	
	LOG_INF("read imx230 SPC, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx230_eeprom(addr, IMX230_SPC_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			//return false;
		}
	}

	memcpy(data, IMX230_SPC_data , size);
    //return true;
}


void read_imx230_DCC( kal_uint16 addr,BYTE* data, kal_uint32 size){
	//int i;
	addr = 0x463;
	size = 0x4c2-0x463+1;//96
	
	LOG_INF("read imx230 DCC, size = %d\n", size);
	
	if(!get_done || last_size != size || last_offset != addr) {
		if(!_read_imx230_eeprom(addr, IMX230_DCC_data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
		}
	}

	memcpy(data, IMX230_DCC_data , size);
}


