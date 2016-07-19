/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>


#define PFX "S5K3P3_pdafotp"
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
//extern int iBurstWriteReg_multi(u8 *pData, u32 bytes, u16 i2cId, u16 transfer_length);
extern int iMultiReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId, u8 number);


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define S5K3P3_EEPROM_READ_ID  0xA1
#define S5K3P3_EEPROM_WRITE_ID   0xA0
#define S5K3P3_I2C_SPEED        100
#define S5K3P3_MAX_OFFSET		0xFFFF

#define DATA_SIZE 2048
BYTE s5k3P3_eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;


static bool selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	printk("deng, %s() \n",__func__);
    if(addr > S5K3P3_MAX_OFFSET)
        return false;
	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, S5K3P3_EEPROM_WRITE_ID)<0)
		return false;
    return true;
}

static bool _read_3P3_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0, count = 0;
	int offset = addr;
	printk("zhouzhenshu %s offset=0x%x  size=%d  \n",__func__,offset,size);
	// get PDAF calibration step 1 data
	offset = 0x0782; 
	printk("deng, %s() \n",__func__);
	for(i = 0; i < 496; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x    data[%d]0x%x\n",offset, i, data[i]);
		offset++;
		count++;
	}
	
	// get PDAF calibration step 2 data
	offset = 0x0972;
	for(i = 496; i < 1404; i++) {
		if(!selective_read_eeprom(offset, &data[i])){
			LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x    data[%d]0x%x\n",offset, i, data[i]);
		offset++;
		count++;
	}
   for(i=1404;i<2048;i++){
       data[i]=0x00; 
   }
	
	LOG_INF("[mcnex]read_eeprom data count = %d\n",count);
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool read_3P3_eeprom( kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	printk("deng, %s() \n",__func__);
	LOG_INF("read_otp_pdaf_data enter,get_done=%d,last_size=%d,size=%d,last_offset=%d,addr=%d\n",get_done,last_size,size,last_offset,addr);
	if(1){//!get_done || last_size != size || last_offset != addr) {
		//if(!_read_eeprom(addr, eeprom_data, size)){
		if(!_read_3P3_eeprom(addr, data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			LOG_INF("read_otp_pdaf_data fail");
			return false;
		}
	}
	//memcpy(data, eeprom_data, size);
	LOG_INF("read_otp_pdaf_data end");
    return true;
}


