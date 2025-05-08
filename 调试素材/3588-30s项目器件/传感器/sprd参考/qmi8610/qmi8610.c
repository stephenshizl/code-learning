/*****************************************************************************
 *
 * Copyright (c) QST, Inc.  All rights reserved.
 *
 * This source is subject to the QST Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of QST Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("QST Software")
 * have been modified by QST Inc. All revisions are subject to any receiver's
 * applicable license agreements with QST Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 *****************************************************************************/
#include<linux/module.h>
#include<linux/err.h>
#include<linux/errno.h>
#include<linux/delay.h>
#include<linux/fs.h>
#include<linux/i2c.h>

#include<linux/input.h>
#include<linux/input-polldev.h>
#include<linux/miscdevice.h>
#include<linux/uaccess.h>
#include<linux/slab.h>

#include<linux/workqueue.h>
#include<linux/irq.h>
#include<linux/gpio.h>
#include<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif
#include<linux/of_device.h>
#include<linux/of_address.h>
#include<linux/of_gpio.h>

#include<linux/mutex.h>
#include <linux/kthread.h>
#include "qmi8610.h"

//#define ACC_USE_CALI

static struct wakeup_source    qmi8610_acc_ws;
static struct wakeup_source    qmi8610_gyro_ws;

static QmiImu_map g_map;
static QmiImu_t *g_qmiimu=NULL;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void QmiImu_early_suspend (struct early_suspend* es);
static void QmiImu_late_resume(struct early_suspend* es);
#endif

static int offset_Rx = 0;
static int offset_Ry = 0;
static int offset_Rz = 0;
static int offset_Ax = 0;
static int offset_Ay = 0;
static int offset_Az = 0;

static const char offset_g_path[] = "/productinfo/GyroCalibration";
static const char offset_a_path[] = "/productinfo/AccCalibration";
static int QmiImu_write_reg(unsigned char reg, unsigned char value)
{
#if 1
	int ret = 0;
	mutex_lock(&g_qmiimu->op_mutex);
	//ret = i2c_smbus_write_i2c_block_data(g_qmiimu->client, reg, 1, &value);
	ret = i2c_smbus_write_byte_data(g_qmiimu->client, reg, value);
	mutex_unlock(&g_qmiimu->op_mutex);
	if(ret < 0) {
		return ret;
	}
	else {
		return 0;
	}
#else
	int err=0;
	unsigned char retry = 0;
	unsigned char txbuf[8];

	g_qmiimu->client->addr = g_qmiimu->client->addr & I2C_MASK_FLAG;
	txbuf[0] = reg;
	txbuf[1] = value;
	while(retry < 5)
	{
		retry++;
		err = i2c_master_send(g_qmiimu->client, txbuf, 2);
		if(err >= 0)
		{
			break;
		}
		mdelay(1);
	}

	if(err < 0)
		return -EFAULT;
	else
		return 0;
#endif
}

static int QmiImu_read_reg(unsigned char reg, unsigned char *buf, unsigned char len)
{
#if defined(QMI6810_MTK_KK)
	unsigned char retry = 0;
	int ret = 0;
	
	g_qmiimu->client->addr = (g_qmiimu->client->addr&I2C_MASK_FLAG)|I2C_WR_FLAG|I2C_RS_FLAG;
	buf[0] = reg;
	while(retry < 5)
	{
		retry++;
		ret = i2c_master_send(g_qmiimu->client, buf, ((len<<0X08)|0X01));
		
		if(ret >= 0)
			break;

		QMIIMU_ERR("QmiImu_read_reg read retry %d times\n", retry);
		mdelay(1); 
	}
	g_qmiimu->client->addr = (g_qmiimu->client->addr&I2C_MASK_FLAG);

	if(ret < 0)
		return -EFAULT;
	else
		return 0;
#else
	#if 1
	unsigned char reg_addr = reg;
	unsigned char *rxbuf = buf;
	unsigned char left = len;
	unsigned char retry = 0;
	unsigned char offset = 0;

	struct i2c_msg msg[2] = {
		{
		 .addr = g_qmiimu->client->addr,
		 .flags = 0,
		 .buf = &reg_addr,
		 .len = 1,
		},
		{
		 .addr = g_qmiimu->client->addr,
		 .flags = I2C_M_RD,
		},
	};
	if(len > 1)
	{
		reg_addr = reg | 0x80;
	}

	if(rxbuf == NULL)
		return -1;

	while(left > 0) {
		retry = 0;
		reg_addr = reg + offset;
		msg[1].buf = &rxbuf[offset];

		if(left > 8) {
			msg[1].len = 8;
			left -= 8;
			offset += 8;
		} else {
			msg[1].len = left;
			left = 0;
		}

		while(i2c_transfer(g_qmiimu->client->adapter, &msg[0], 2) != 2) {
			retry++;
			if (retry == 20) {
				QMIIMU_ERR("i2c read register=0x%x length=%d failed\n", reg+offset,len);
				return -EIO;
			}
		}
	}
	#else
	unsigned char reg_addr = reg;
	signed int dummy = 0;

	if(len > 1)
	{
		reg_addr = reg | 0x80;
	}
	dummy = i2c_smbus_read_i2c_block_data(g_qmiimu->client, reg_addr, len, buf);
	if (dummy < 0){
		QMIIMU_ERR("i2c read register=0x%x failed \n", reg_addr);
		return dummy;
	}
	#endif
	
	return 0;
#endif
}

static void QmiImu_set_layout(int layout)
{
	if(layout == 0)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 1)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 2)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 3)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}	
	else if(layout == 4)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 5)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 6)
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else if(layout == 7)
	{
		g_map.sign[AXIS_X] = -1;
		g_map.sign[AXIS_Y] = -1;
		g_map.sign[AXIS_Z] = -1;
		g_map.map[AXIS_X] = AXIS_Y;
		g_map.map[AXIS_Y] = AXIS_X;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
	else		
	{
		g_map.sign[AXIS_X] = 1;
		g_map.sign[AXIS_Y] = 1;
		g_map.sign[AXIS_Z] = 1;
		g_map.map[AXIS_X] = AXIS_X;
		g_map.map[AXIS_Y] = AXIS_Y;
		g_map.map[AXIS_Z] = AXIS_Z;
	}
}

static unsigned char QmiImu_get_chipid(void)
{
	int ret;
	unsigned char chipid = 0;

	ret = QmiImu_read_reg(QmiRegister_WhoAmI, &chipid, 1);
	QMIIMU_LOG("chipid = 0x%x", chipid);

	return chipid;
}

static void QmiImu_config_acc(enum QmiImu_AccRange range, enum QmiImu_AccOdr odr, enum QmiImu_LpfConfig lpfEnable, enum QmiImu_HpfConfig hpfEnable)
{
	unsigned char ctl_dada;
	unsigned char range_set;

	range_set = range;
	switch(range)
	{
		case AccRange_2g:
			g_qmiimu->a_lsb = (1<<14);
			break;
		case AccRange_4g:
			g_qmiimu->a_lsb = (1<<13);
			break;
		case AccRange_8g:
			g_qmiimu->a_lsb = (1<<12);
			break;
		default: 
			range_set = 2<<3;
			g_qmiimu->a_lsb = (1<<12);
	}
	ctl_dada = (unsigned char)range_set|(unsigned char)odr;
	QmiImu_write_reg(QmiRegister_Ctrl2, ctl_dada);
// set LPF & HPF
	QmiImu_read_reg(QmiRegister_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0xfc;
	if(lpfEnable == Lpf_Enable)
		ctl_dada |=0x02;
	if(hpfEnable == Hpf_Enable)
		ctl_dada |=0x01;
	QmiImu_write_reg(QmiRegister_Ctrl5,ctl_dada);
// set LPF & HPF
}

static void QmiImu_config_gyro(enum QmiImu_GyrRange range, enum QmiImu_GyrOdr odr, enum QmiImu_LpfConfig lpfEnable, enum QmiImu_HpfConfig hpfEnable)
{
	unsigned char ctl_dada; 
	unsigned char range_set;

	range_set = range;
	switch(range)
	{
		case GyrRange_32dps:
			g_qmiimu->g_lsb = 1024;
			break;
		case GyrRange_64dps:
			g_qmiimu->g_lsb = 512;
			break;
		case GyrRange_128dps:
			g_qmiimu->g_lsb = 256;
			break;
		case GyrRange_256dps:
			g_qmiimu->g_lsb = 128;
			break;
		case GyrRange_512dps:
			g_qmiimu->g_lsb = 64;
			break;
		case GyrRange_1024dps:
			g_qmiimu->g_lsb = 32;
			break;
		case GyrRange_2048dps:
			g_qmiimu->g_lsb = 16;
			break;
		case GyrRange_2560dps:
			g_qmiimu->g_lsb = 8;
			break;
		default:
			range_set = GyrRange_1024dps;
			g_qmiimu->g_lsb = 32;
			break;
	}

	ctl_dada = (unsigned char)range_set | (unsigned char)odr;
	QmiImu_write_reg(QmiRegister_Ctrl3, ctl_dada);
// Conversion from degrees/s to rad/s if necessary
// set LPF & HPF
	QmiImu_read_reg(QmiRegister_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0xf3;
	if(lpfEnable == Lpf_Enable)
		ctl_dada |=0x08;
	if(hpfEnable == Hpf_Enable)
		ctl_dada |=0x04;
	QmiImu_write_reg(QmiRegister_Ctrl5,ctl_dada);
// set LPF & HPF
}

static void QmiImu_config_ae(enum QmiImu_AeOdr odr)
{
	QmiImu_config_acc(AccRange_8g, AccOdr_1024Hz, Lpf_Enable, Hpf_Disable);
	QmiImu_config_gyro(GyrRange_2048dps, GyrOdr_1024Hz, Lpf_Enable, Hpf_Disable);
	QmiImu_write_reg(QmiRegister_Ctrl6, odr);
}

static void QmiImu_config_mag(enum QmiImu_MagDev device, enum QmiImu_MagOdr odr)
{
	
}

static unsigned char QmiImu_readStatus0(void)
{
	unsigned char status;
	QmiImu_read_reg(QmiRegister_Status0, &status, sizeof(status));
	return status;
}

/*
unsigned char QmiImu_readStatus1(void)
{
	unsigned char status;
	QmiImu_read_reg(QmiRegister_Status1, &status, sizeof(status));
	return status;
}

int8_t QmiImu_readTemp(void)
{
	int8_t temp;
	QmiImu_read_reg(QmiRegister_Temperature, (unsigned char*)&temp, sizeof(temp));
	return temp;
}
*/

static int QmiImu_read_acc_xyz(int acc_xyz[3])
{
	unsigned char	buf_reg[6];
	short 			raw_acc_xyz[3];
	int 			temp_xyz[3];
	int				ret = 0;

#if 0	//defined(QST_USE_SPI)
	ret += QmiImu_read_reg(QmiRegister_Ax_L, &buf_reg[0], 6); 	// 0x19, 25
#else
	ret += QmiImu_read_reg(QmiRegister_Ax_L, &buf_reg[0], 1);		// 0x19, 25
	ret += QmiImu_read_reg(QmiRegister_Ax_H, &buf_reg[1], 1);
	ret += QmiImu_read_reg(QmiRegister_Ay_L, &buf_reg[2], 1);
	ret += QmiImu_read_reg(QmiRegister_Ay_H, &buf_reg[3], 1);
	ret += QmiImu_read_reg(QmiRegister_Az_L, &buf_reg[4], 1);
	ret += QmiImu_read_reg(QmiRegister_Az_H, &buf_reg[5], 1);
#endif
	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));
	QMIIMU_LOG("Acc buf_reg: [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]%d\n", 
		buf_reg[0],buf_reg[1],buf_reg[2],buf_reg[3],buf_reg[4],buf_reg[5], g_qmiimu->a_lsb);

	temp_xyz[0] = raw_acc_xyz[0];
	temp_xyz[1] = raw_acc_xyz[1];
	temp_xyz[2] = raw_acc_xyz[2];
	temp_xyz[0] = (int)((temp_xyz[0]*1000)/g_qmiimu->a_lsb);
	temp_xyz[1] = (int)((temp_xyz[1]*1000)/g_qmiimu->a_lsb);
	temp_xyz[2] = (int)((temp_xyz[2]*1000)/g_qmiimu->a_lsb);

	acc_xyz[g_map.map[AXIS_X]] = g_map.sign[AXIS_X]*temp_xyz[0];
	acc_xyz[g_map.map[AXIS_Y]] = g_map.sign[AXIS_Y]*temp_xyz[1];
	acc_xyz[g_map.map[AXIS_Z]] = g_map.sign[AXIS_Z]*temp_xyz[2];

	return ret;
}

static int QmiImu_read_gyro_xyz(int gyro_xyz[3])
{
	unsigned char	buf_reg[6];
	unsigned char	status0 = 0;
	short 			raw_gyro_xyz[3];
	int 			temp_xyz[3];
	int				ret = 0;

	while((status0 & 0x22) == 0)
	{
		status0 = QmiImu_readStatus0();
	}
#if 0//defined(QST_USE_SPI)
	ret += QmiImu_read_reg(QmiRegister_Gx_L, &buf_reg[0], 6); 	// 0x19, 25
#else
	ret += QmiImu_read_reg(QmiRegister_Gx_L, &buf_reg[0], 1);	
	ret += QmiImu_read_reg(QmiRegister_Gx_H, &buf_reg[1], 1);
	ret += QmiImu_read_reg(QmiRegister_Gy_L, &buf_reg[2], 1);
	ret += QmiImu_read_reg(QmiRegister_Gy_H, &buf_reg[3], 1);
	ret += QmiImu_read_reg(QmiRegister_Gz_L, &buf_reg[4], 1);
	ret += QmiImu_read_reg(QmiRegister_Gz_H, &buf_reg[5], 1);
#endif
	raw_gyro_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));	
	QMIIMU_LOG("Gyro buf_reg: [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]%d\n", 
		buf_reg[0],buf_reg[1],buf_reg[2],buf_reg[3],buf_reg[4],buf_reg[5], g_qmiimu->g_lsb);

	temp_xyz[0] = raw_gyro_xyz[0];
	temp_xyz[1] = raw_gyro_xyz[1];
	temp_xyz[2] = raw_gyro_xyz[2];
	temp_xyz[0] = (int)((temp_xyz[0]*1000)/g_qmiimu->g_lsb);
	temp_xyz[1] = (int)((temp_xyz[1]*1000)/g_qmiimu->g_lsb);
	temp_xyz[2] = (int)((temp_xyz[2]*1000)/g_qmiimu->g_lsb);

	gyro_xyz[g_map.map[AXIS_X]] = g_map.sign[AXIS_X]*temp_xyz[0];
	gyro_xyz[g_map.map[AXIS_Y]] = g_map.sign[AXIS_Y]*temp_xyz[1];
	gyro_xyz[g_map.map[AXIS_Z]] = g_map.sign[AXIS_Z]*temp_xyz[2];

	return ret;
}

static void QmiImu_enableSensors(unsigned char enableFlags)
{
	unsigned char ctr7;
	if(enableFlags & QMIIMU_CONFIG_AE_ENABLE)
	{
		enableFlags |= QMIIMU_CTRL7_ACC_ENABLE | QMIIMU_CTRL7_GYR_ENABLE;
	}
	
	QmiImu_read_reg(QmiRegister_Ctrl7, &ctr7, 1);
	QmiImu_write_reg(QmiRegister_Ctrl7, (ctr7|enableFlags));
	mdelay(100);
}

static void QmiImu_disableSensors(unsigned char disableFlags)
{
	unsigned char ctr7;
	
	QmiImu_read_reg(QmiRegister_Ctrl7, &ctr7, 1);
	QmiImu_write_reg(QmiRegister_Ctrl7, (ctr7&(~disableFlags)));
}

#if defined(QMIIMU_WAKEON_MOTION)
static void QmiImu_doCtrl9Command(enum QmiImu_Ctrl9Command cmd)
{
	unsigned char gyroConfig;
	const unsigned char oisModeBits = 0x06;
	unsigned char oisEnabled;
	unsigned char status = 0;
	unsigned int count = 0;

	QmiImu_read_reg(QmiRegister_Ctrl3, &gyroConfig, sizeof(gyroConfig));
	oisEnabled = ((gyroConfig & oisModeBits) == oisModeBits);
	if(oisEnabled)
	{
		QmiImu_write_reg(QmiRegister_Ctrl3, (gyroConfig & ~oisModeBits));
	}

	//g_fisDriverHal->waitForEvent(Qmi_Int1, QmiInt_low);
	//qst_delay(300);

	QmiImu_write_reg(QmiRegister_Ctrl9, cmd);
	//g_fisDriverHal->waitForEvent(Qmi_Int1, QmiInt_high);
	//qst_delay(300);

	// Check that command has been executed
	while(((status & QMIIMU_STATUS1_CMD_DONE)==0)&&(count<10000))
	{
		QmiImu_read_reg(QmiRegister_Status1, &status, sizeof(status));
		count++;
	}
	//assert(status & QMIIMU_STATUS1_CMD_DONE);

	//g_fisDriverHal->waitForEvent(Qmi_Int1, QmiInt_low);
	//qst_delay(300);

	if(oisEnabled)
	{
		// Re-enable OIS mode configuration if necessary		
		QmiImu_write_reg(QmiRegister_Ctrl3, gyroConfig);
	}
}

static void QmiImu_enableWakeOnMotion(void)
{
	unsigned char womCmd[3];
	enum QmiImu_Interrupt interrupt = Qmi_Int1;
	enum QmiImu_InterruptInitialState initialState = InterruptInitialState_low;
	enum QmiImu_WakeOnMotionThreshold threshold = WomThreshold_low;
	unsigned char blankingTime = 0x00;
	const unsigned char blankingTimeMask = 0x3F;

	QmiImu_disableSensors(QMIIMU_CONFIG_ALL_ENABLE);
	//QmiImu_config_acc(AccRange_2g, AccOdr_LowPower_3Hz, Lpf_Disable, Hpf_Disable);
	QmiImu_config_acc(AccRange_2g, AccOdr_LowPower_25Hz, Lpf_Disable, Hpf_Disable);

	womCmd[0] = QmiRegister_Cal1_L;		//WoM Threshold: absolute value in mg (with 1mg/LSB resolution)
	womCmd[1] = threshold;
	womCmd[2] = (unsigned char)interrupt | (unsigned char)initialState | (blankingTime & blankingTimeMask);
	QmiImu_write_reg(QmiRegister_Cal1_L, womCmd[1]);
	QmiImu_write_reg(QmiRegister_Cal1_H, womCmd[2]);

	QmiImu_doCtrl9Command(Ctrl9_ConfigureWakeOnMotion);
	QmiImu_enableSensors(QMIIMU_CTRL7_ACC_ENABLE);
}

static void QmiImu_disableWakeOnMotion(void)
{
	QmiImu_disableSensors(QMIIMU_CTRL7_DISABLE_ALL);
	QmiImu_write_reg(QmiRegister_Cal1_L, 0);
	QmiImu_doCtrl9Command(Ctrl9_ConfigureWakeOnMotion);
}
#endif

static void QmiImu_config_apply(struct QmiImuConfig const* config)
{
	unsigned char fisSensors = config->inputSelection;

	if(fisSensors & QMIIMU_CONFIG_AE_ENABLE)
	{
		QmiImu_config_ae(config->aeOdr);
	}
	else
	{
		if (config->inputSelection & QMIIMU_CONFIG_ACC_ENABLE)
		{
			QmiImu_config_acc(config->accRange, config->accOdr, Lpf_Enable, Hpf_Disable);
		}
		if (config->inputSelection & QMIIMU_CONFIG_GYR_ENABLE)
		{
			QmiImu_config_gyro(config->gyrRange, config->gyrOdr, Lpf_Enable, Hpf_Disable);
		}
	}

	if(config->inputSelection & QMIIMU_CONFIG_MAG_ENABLE)
	{
		QmiImu_config_mag(config->magDev, config->magOdr);
	}
	//QmiImu_enableSensors(fisSensors);	
}

static void QmiImu_set_enable(enum QmiImu_type type, int enable)
{
	if(g_qmiimu->chip_id != 0xfc)	// enable check
	{
		g_qmiimu->chip_id = QmiImu_get_chipid();
		QmiImu_config_apply(&g_qmiimu->config);
	}

	if(enable) {
		if((g_qmiimu->acc_enable == 0)&&(g_qmiimu->gyro_enable == 0)) {
			QmiImu_enableSensors(QMIIMU_CONFIG_ACCGYR_ENABLE);
		}
	}
	else {
		if(((g_qmiimu->acc_enable==0)&&(g_qmiimu->gyro_enable==1))
			||((g_qmiimu->acc_enable==1)&&(g_qmiimu->gyro_enable==0)))
		{
			QmiImu_disableSensors(QMIIMU_CONFIG_ACCGYR_ENABLE);
		}
	}

	if(type == QMI_IMU_ACC)
	{
		if(enable)
		{
			schedule_delayed_work(&g_qmiimu->acc_work, msecs_to_jiffies(600));
		}
		else
		{
			cancel_delayed_work_sync(&g_qmiimu->acc_work);
		}		
		g_qmiimu->acc_enable = enable;
	}
	else if(type == QMI_IMU_GYRO)
	{
		if(enable)
		{		
			schedule_delayed_work(&g_qmiimu->gyro_work, msecs_to_jiffies(600));
		}
		else
		{
			cancel_delayed_work_sync(&g_qmiimu->gyro_work);
		}
		g_qmiimu->gyro_enable = enable;
	}
}

static void QmiImu_set_delay(enum QmiImu_type type, int delay)
{
	if(type == QMI_IMU_ACC)
	{
		cancel_delayed_work_sync(&g_qmiimu->acc_work);
		g_qmiimu->acc_delay = delay;
		if(g_qmiimu->acc_enable)
			schedule_delayed_work(&g_qmiimu->acc_work, msecs_to_jiffies(g_qmiimu->acc_delay));
	}
	else if(type == QMI_IMU_GYRO)
	{
		cancel_delayed_work_sync(&g_qmiimu->gyro_work);
		g_qmiimu->gyro_delay = delay;
		if(g_qmiimu->gyro_enable)
			schedule_delayed_work(&g_qmiimu->gyro_work, msecs_to_jiffies(g_qmiimu->gyro_delay));
	}
}

static ssize_t show_init_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	g_qmiimu->chip_id = QmiImu_get_chipid();
	pr_err("stephen Qmi chip_id:0x%x\n",g_qmiimu->chip_id);
	QmiImu_config_apply(&g_qmiimu->config);
	//QmiImu_enableSensors(g_qmiimu->config.inputSelection);

	return sprintf(buf, "init done!\n");
}

static ssize_t show_chipinfo_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "QMI6810\n");
}

static ssize_t show_sensordata_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int acc[3];
	int gyro[3];
	
	QmiImu_read_acc_xyz(acc);
	QmiImu_read_gyro_xyz(gyro);

	return sprintf(buf, "%d %d %d\n%d %d %d",acc[0],acc[1],acc[2], gyro[0],gyro[1],gyro[2]);
}
		
static ssize_t show_dumpallreg_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res;
	int i =0;
	char strbuf[600];
	char tempstrbuf[24];
	unsigned char databuf[2]={0};
	int length=0;

	QMIIMU_FUN();
	/* Check status register for data availability */
	for(i = 0; i<=50; i++)
	{
		databuf[0] = i;
		res = QmiImu_read_reg(databuf[0], &databuf[1], 1);
		if(res < 0)
			QMIIMU_LOG("qma6981 dump registers 0x%02x failed !\n", i);

		length = scnprintf(tempstrbuf, sizeof(tempstrbuf), "0x%2x=0x%2x\n",i, databuf[1]);
		snprintf(strbuf+length*i, sizeof(strbuf)-length*i, "%s",tempstrbuf);
	}

	return scnprintf(buf, sizeof(strbuf), "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMIIMU_FUN();
	return sprintf(buf, "%d\n", atomic_read(&g_qmiimu->layout));
}

static ssize_t store_layout_value(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int position = 0;

	QMIIMU_FUN();
	if(1 == sscanf(buf, "%d", &position))
	{
		if ((position >= 0) && (position <= 7))
		{
			atomic_set(&g_qmiimu->layout, position);
			QmiImu_set_layout(position);
		}
	}
	else
	{
		QMIIMU_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

static ssize_t QmiImu_enable_acc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMIIMU_FUN();

	return sprintf(buf, "%d\n", g_qmiimu->acc_enable);
}

static ssize_t QmiImu_enable_acc_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int enable=0;

	QMIIMU_FUN();
	
	if(1 == sscanf(buf, "%d", &enable))
	{
		if(enable)
			QmiImu_set_enable(QMI_IMU_ACC, 1);
		else
			QmiImu_set_enable(QMI_IMU_ACC, 0);
	}
	else
	{	
		QMIIMU_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t QmiImu_delay_acc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMIIMU_FUN();
	return sprintf(buf, "%d\n", g_qmiimu->acc_delay);
}

static ssize_t QmiImu_delay_acc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int delay=0;

	QMIIMU_FUN();
	if(1 == sscanf(buf, "%d", &delay))
	{
		if(delay > 2000)
			delay = 2000;
		else if(delay <= 1)
			delay = 1;

		QmiImu_set_delay(QMI_IMU_ACC, delay);
	}
	else
	{
		QMIIMU_ERR("invalid format = '%s'\n", buf);
	}
	
	return count;
}

static ssize_t QmiImu_enable_gyro_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMIIMU_FUN();

	return sprintf(buf, "%d\n", g_qmiimu->gyro_enable);
}

static ssize_t QmiImu_enable_gyro_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned int enable=0;

	QMIIMU_FUN();
	
	if(1 == sscanf(buf, "%d", &enable))
	{
		if(enable)
			QmiImu_set_enable(QMI_IMU_GYRO, 1);
		else
			QmiImu_set_enable(QMI_IMU_GYRO, 0);
	}
	else
	{	
		QMIIMU_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t QmiImu_delay_gyro_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	QMIIMU_FUN();
	return sprintf(buf, "%d\n", g_qmiimu->gyro_delay);
}

static ssize_t QmiImu_delay_gyro_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int delay=0;

	QMIIMU_FUN();
	if(1 == sscanf(buf, "%d", &delay))
	{
		if(delay > 2000)
			delay = 2000;
		else if(delay <= 1)
			delay = 1;

		QmiImu_set_delay(QMI_IMU_GYRO, delay);
	}
	else
	{
		QMIIMU_ERR("invalid format = '%s'\n", buf);
	}
	
	return count;
}

static unsigned char QmiImu_debug_reg_addr=0x00;
static unsigned char QmiImu_debug_read_len=0x01;
static ssize_t QmiImu_getreg(struct device *dev, struct device_attribute *attr, char *buf)
{
	int res = 0;	
	unsigned char data=0xff;	

	QMIIMU_FUN();
	res = QmiImu_read_reg(QmiImu_debug_reg_addr, &data, 1);

	return sprintf(buf, "0x%x = 0x%x\n",QmiImu_debug_reg_addr, data);
}

static ssize_t QmiImu_setreg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int addr, value;
	unsigned char data;	
	int res = 0;	
	
	QMIIMU_LOG("store_setreg buf=%s count=%d\n", buf, (int)count);	
	if(2 == sscanf(buf, "0x%x 0x%x", &addr, &value))	
	{		
		QMIIMU_LOG("get para OK addr=0x%x value=0x%x\n", addr, value);		
		QmiImu_debug_reg_addr = (unsigned char)addr;		
		QmiImu_debug_read_len = 1;		
		data = (unsigned char)value;		
		res = QmiImu_write_reg(QmiImu_debug_reg_addr, data);		
		if(res)		
		{			
			QMIIMU_ERR("write reg 0x%02x fail\n", addr);		
		}	
	}	
	else
	{		
		QMIIMU_ERR("store_reg get para error\n");	
	}
	
	return count;
}

static DEVICE_ATTR(init,			0644, show_init_value, NULL);
static DEVICE_ATTR(chipinfo,		0644, show_chipinfo_value, NULL);
static DEVICE_ATTR(sensordata,	0644, show_sensordata_value,    NULL);
static DEVICE_ATTR(dumpallreg,	0644, show_dumpallreg_value, NULL);
static DEVICE_ATTR(layout,		0644, show_layout_value, store_layout_value);
static DEVICE_ATTR(acc_enable,	0644, QmiImu_enable_acc_show , QmiImu_enable_acc_store);
static DEVICE_ATTR(acc_delay,		0644, QmiImu_delay_acc_show , QmiImu_delay_acc_store);
static DEVICE_ATTR(gyro_enable,	0644, QmiImu_enable_gyro_show , QmiImu_enable_gyro_store);
static DEVICE_ATTR(gyro_delay,	0644, QmiImu_delay_gyro_show , QmiImu_delay_gyro_store);
static DEVICE_ATTR(setreg,		0644, QmiImu_getreg , QmiImu_setreg);

static struct attribute *QmiImu_acc_attributes[] = {
	&dev_attr_init.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_delay.attr,
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_delay.attr,
	&dev_attr_setreg.attr,
	NULL
};

static struct attribute *QmiImu_gyro_attributes[] = {
	&dev_attr_init.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_sensordata.attr,
	&dev_attr_dumpallreg.attr,
	&dev_attr_layout.attr,
	&dev_attr_acc_enable.attr,
	&dev_attr_acc_delay.attr,
	&dev_attr_gyro_enable.attr,
	&dev_attr_gyro_delay.attr,
	&dev_attr_setreg.attr,
	NULL
};


static struct attribute_group QmiImu_acc_attribute_group = {
	//.name = "qmi8610",
	.attrs = QmiImu_acc_attributes
};

static struct attribute_group QmiImu_gyro_attribute_group = {
	//.name = "qmi8610",
	.attrs = QmiImu_gyro_attributes
};
/*
static void get_offset_func(void)
{
 	struct file *fp; 
 	mm_segment_t fs;  
 	loff_t pos; 
	char Gbuffer[100]= {0};
	char Abuffer[100]= {0};
	
	fp = filp_open(offset_g_path, O_RDONLY , 0);
	if (IS_ERR(fp)) {  
		QMIIMU_ERR("open offset_g_path file failed\n");  
		return;
	}

	fs = get_fs();  
	set_fs(KERNEL_DS);

	pos = fp->f_pos; 
	vfs_read(fp, Gbuffer, sizeof(Gbuffer), &pos);  
	fp->f_pos = pos;

	set_fs(fs);

	filp_close(fp, NULL);
	sscanf(Gbuffer, "%d;%d;%d", &offset_Rx,&offset_Ry,&offset_Rz);

	QMIIMU_LOG("Rx: %d, Ry: %d Rz: %d\n",offset_Rx,offset_Ry,offset_Rz);  

	fp = filp_open(offset_a_path, O_RDONLY , 0);
	if (IS_ERR(fp)) {  
		printk("open offset_a_path file failed\n");  
		return;
	}

	fs = get_fs();  
	set_fs(KERNEL_DS);

	pos = fp->f_pos; 
	vfs_read(fp, Abuffer, sizeof(Abuffer), &pos);  
	fp->f_pos = pos;

	set_fs(fs);

	filp_close(fp, NULL);
	sscanf(Abuffer, "%d;%d;%d", &offset_Ax,&offset_Ay,&offset_Az);

	QMIIMU_LOG("Ax: %d, Ay: %d Az: %d\n",offset_Ax,offset_Ay,offset_Az);  

}
*/
static void acc_work_func(struct work_struct *work)
{
	int comres = 1;
	int retry = 0;
	int acc_out[3];

	while(comres)
	{	
		if(retry >= 5)
		{		
			schedule_delayed_work(&g_qmiimu->acc_work, msecs_to_jiffies(g_qmiimu->acc_delay));
			return;
		}		
		comres = QmiImu_read_acc_xyz(acc_out);
		retry++;
	}
#if 0
	if(offset_Ax == 0 && offset_Ay ==0 && offset_Az == 0 
		&& offset_Rx == 0 && offset_Ry == 0 && offset_Rz == 0)
	{
		get_offset_func();
	}
#endif
	QMIIMU_LOG("Acc: %d	%d	%d \n", acc_out[0],acc_out[1],acc_out[2]);
	acc_out[0] = acc_out[0] - offset_Ax;
	acc_out[1] = acc_out[1] - offset_Ay;
	acc_out[2] = acc_out[2] - offset_Az;
	QMIIMU_LOG("offset  Acc: %d	%d	%d \n", acc_out[0],acc_out[1],acc_out[2]);
	input_report_abs(g_qmiimu->acc_input, ABS_X, acc_out[0]);
	input_report_abs(g_qmiimu->acc_input, ABS_Y, acc_out[1]);
	input_report_abs(g_qmiimu->acc_input, ABS_Z, acc_out[2]);
	input_report_abs(g_qmiimu->acc_input, ABS_THROTTLE, 3);
	input_sync(g_qmiimu->acc_input);
	schedule_delayed_work(&g_qmiimu->acc_work, msecs_to_jiffies(g_qmiimu->acc_delay));
}

static void gyro_work_func(struct work_struct *work)
{
	int comres = 1;
	int retry = 0;
	int gyro_out[3];

	while(comres)
	{	
		if(retry >= 5)
		{		
			schedule_delayed_work(&g_qmiimu->gyro_work, msecs_to_jiffies(g_qmiimu->gyro_delay));
			return;
		}		
		comres = QmiImu_read_gyro_xyz(gyro_out);
		retry++;
	}
#if 0
	if(offset_Ax == 0 && offset_Ay ==0 && offset_Az == 0 
		&& offset_Rx == 0 && offset_Ry == 0 && offset_Rz == 0)
	{
		get_offset_func();
	}
#endif
	QMIIMU_LOG("Gyro: %d	%d	%d \n", gyro_out[0],gyro_out[1],gyro_out[2]);
	gyro_out[0] = gyro_out[0] - offset_Rx;
	gyro_out[1] = gyro_out[1] - offset_Ry;
	gyro_out[2] = gyro_out[2] - offset_Rz;
	QMIIMU_LOG("offset  Gyro: %d	%d	%d \n", gyro_out[0],gyro_out[1],gyro_out[2]);
	input_report_abs(g_qmiimu->gyro_input, ABS_RX, gyro_out[0]);
	input_report_abs(g_qmiimu->gyro_input, ABS_RY, gyro_out[1]);
	input_report_abs(g_qmiimu->gyro_input, ABS_RZ, gyro_out[2]);
	input_report_abs(g_qmiimu->gyro_input, ABS_RUDDER, 3);
	input_sync(g_qmiimu->gyro_input);
	schedule_delayed_work(&g_qmiimu->gyro_work, msecs_to_jiffies(g_qmiimu->gyro_delay));
}

static int QmiImu_acc_input_init(void)
{
	struct input_dev *dev = NULL;
	int err = 0;

	QMIIMU_LOG("%s called\n", __func__);
	dev = input_allocate_device();
	if (!dev) {
		QMIIMU_ERR("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}

	dev->name = QMIIMU_ACC_INUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_ABS, ABS_Z);
	input_set_capability(dev, EV_ABS, ABS_THROTTLE);
	
	input_set_abs_params(dev, ABS_X, (-32*1000), (32*1000), 0, 0);
	input_set_abs_params(dev, ABS_Y, (-32*1000), (32*1000), 0, 0);
	input_set_abs_params(dev, ABS_Z, (-32*1000), (32*1000), 0, 0);
	input_set_abs_params(dev, ABS_THROTTLE, 0, 3, 0, 0);
	
	input_set_drvdata(dev, g_qmiimu);

	err = input_register_device(dev);
	
	if (err < 0) {
		QMIIMU_ERR("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	g_qmiimu->acc_input = dev;

	return 0;
}


static void QmiImu_acc_input_deinit(void)
{
	struct input_dev *dev = g_qmiimu->acc_input;

	QMIIMU_LOG("%s called\n", __func__);
	input_unregister_device(dev);
	input_free_device(dev);
}

static int QmiImu_gyro_input_init(void)
{
	struct input_dev *dev = NULL;
	int err = 0;

	QMIIMU_LOG("%s called\n", __func__);
	dev = input_allocate_device();
	if (!dev) {
		QMIIMU_ERR("%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}
	dev->name = QMIIMU_GYRO_INUT_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_RX);
	input_set_capability(dev, EV_ABS, ABS_RY);
	input_set_capability(dev, EV_ABS, ABS_RZ);
	//input_set_capability(dev, EV_ABS, ABS_RUDDER);

	input_set_abs_params(dev, ABS_RX, -32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_RY, -32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_RZ, -32768, 32767, 0, 0);
	//input_set_abs_params(dev, ABS_RUDDER, 0, 3, 0, 0);
	input_set_drvdata(dev, g_qmiimu);
	err = input_register_device(dev);
	if (err < 0) {
		QMIIMU_ERR("%s: can't register device!\n", __func__);
		input_free_device(dev);
		return err;
	}
	g_qmiimu->gyro_input = dev;

	return 0;
}

static void QmiImu_gyro_input_deinit(void)
{
	struct input_dev *dev = g_qmiimu->gyro_input;
	QMIIMU_LOG("%s called\n", __func__);

	input_unregister_device(dev);	
	input_free_device(dev);
}

#if  1

static int qmi8610_acc_stop_thread(void *arg)
{
	__pm_stay_awake(&qmi8610_acc_ws);
	QmiImu_set_enable(QMI_IMU_ACC, 0);
	pr_err("stephen qmi8610_acc_stop_thread \n");
	__pm_relax(&qmi8610_acc_ws);
	
	return 0;
}

static int qmi8610_acc_thread(void *arg)
{
	char ctr7;
	msleep(3000);
	QmiImu_read_reg(QmiRegister_Ctrl7, &ctr7, 1);
	pr_err("stephe Qmi qmi8610_acc_thread ctr7:0x%x\n",ctr7);
	QmiImu_write_reg(QmiRegister_Ctrl7, (ctr7|0x1));
	
	return 0;
}

static int qmi8610_gyro_stop_thread(void *arg)
{
	__pm_stay_awake(&qmi8610_gyro_ws);
	QmiImu_set_enable(QMI_IMU_GYRO, 0);
	pr_err("stephen qmi8610_gyro_stop_thread \n");
	__pm_relax(&qmi8610_gyro_ws);

	return 0;
}

static int qmi8610_gyro_thread(void *arg)
{
	char ctr7;
	msleep(3000);
	QmiImu_read_reg(QmiRegister_Ctrl7, &ctr7, 1);
	pr_err("stephe Qmi qmi8610_gyro_thread ctr7:0x%x\n",ctr7);
	QmiImu_write_reg(QmiRegister_Ctrl7, (ctr7|0x2));

	return 0;
}

#endif

static long QmiImu_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	void __user *argp = (void __user *)arg;
	int temp = 0;

	QMIIMU_LOG("%s: cmd %x\n",__func__, cmd);
	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));

	if(err)
	{
		QMIIMU_ERR("%s: access isn't ok!\n", __func__);
		return -EFAULT;
	}

	if (NULL == g_qmiimu->client)
	{
		QMIIMU_ERR("%s: i2c client isn't exist!\n", __func__);
		return -EFAULT;
	}
	
	switch(cmd)
	{
	case ACC_IOCTL_SET_ENABLE:		
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			QMIIMU_ERR("%s: ACC_IOCTL_SET_ENABLE copy error!\n", __func__);
			return -EFAULT;
		}
		if(temp){
			QmiImu_set_enable(QMI_IMU_ACC, 1);
			kthread_run(qmi8610_acc_thread, NULL, "qmi8610 acc");
		}
		else{
			//QmiImu_set_enable(QMI_IMU_ACC, 0);
			kthread_run(qmi8610_acc_stop_thread, NULL, "qmi8610 stop acc");
		}
			
		break;
	
	case ACC_IOCTL_SET_DELAY:
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			QMIIMU_ERR("%s: ACC_IOCTL_SET_DELAY copy error!\n", __func__);
			return -EFAULT;
		}
		QmiImu_set_delay(QMI_IMU_ACC, temp);
			
		break;

	case GYRO_IOCTL_SET_ENABLE:		
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			QMIIMU_ERR("%s: GYRO_IOCTL_SET_ENABLE copy error!\n", __func__);
			return -EFAULT;
		}
		if(temp){
			QmiImu_set_enable(QMI_IMU_GYRO, 1);
			kthread_run(qmi8610_gyro_thread, NULL, "qmi8610 gyro");
		}
		else{
			//QmiImu_set_enable(QMI_IMU_GYRO, 0);
			kthread_run(qmi8610_gyro_stop_thread, NULL, "qmi8610 stop gyro");
		}
			
		break;
	
	case GYRO_IOCTL_SET_DELAY:
		if(copy_from_user(&temp, argp, sizeof(temp)))
		{
			QMIIMU_ERR("%s: GYRO_IOCTL_SET_DELAY copy error!\n", __func__);
			return -EFAULT;
		}
		QmiImu_set_delay(QMI_IMU_GYRO, temp);	
		break;
	default:
		QMIIMU_ERR("%s: can't recognize the cmd!\n", __func__);
		return 0;
	}
	
    return 0;
}


static int QmiImu_open(struct inode *inode, struct file *file)
{
	int err = 0;

	QMIIMU_FUN();

	err = nonseekable_open(inode, file);
	if (err < 0)
	{
		QMIIMU_ERR("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(g_qmiimu->client);

	return 0;
}

static int QmiImu_release(struct inode *inode, struct file *file)
{
	QMIIMU_FUN();
	file->private_data = NULL;
	return 0;
}

static const struct file_operations QmiImu_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = QmiImu_open,
	.release = QmiImu_release,
	.unlocked_ioctl = QmiImu_unlocked_ioctl,
};

static struct miscdevice QmiImu_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMIIMU_DEV_NAME,
	.fops = &QmiImu_acc_misc_fops,
};

#ifdef CONFIG_OF
static int QmiImu_acc_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;
	unsigned int position = 0;
	
	ret = of_property_read_u32(np, "qst,layout", &position);
	printk("QmiImu_acc_parse_dt qmi8610 position=%d\n",position);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		return 0;
	}
	//add  stephen ++ 
	g_qmiimu->rst_gpio =  of_get_named_gpio(np, "qmi8610,reset-gpio", 0);
	g_qmiimu->int1_gpio = of_get_named_gpio(np, "qmi8610,int1-gpio", 0);
	g_qmiimu->int2_gpio = of_get_named_gpio(np, "qmi8610,int2-gpio", 0);
	pr_err("stephen qmi8610 rst_gpio:%d, int1_gpio:%d,int2_gpio:%d\n",g_qmiimu->rst_gpio,g_qmiimu->int1_gpio,g_qmiimu->int2_gpio);
	//add  stephen --
	atomic_set(&g_qmiimu->layout, (int)position);
	return 0;
}
#endif

static int QmiImu_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	QmiImu_t *qmi8610;
	int err = 0;

	QMIIMU_LOG("%s: start\n",__func__);
	wakeup_source_init(&qmi8610_acc_ws, "qmi8610_acc_ws");
	wakeup_source_init(&qmi8610_gyro_ws, "qmi8610_gyro_ws");
	qmi8610 = kzalloc(sizeof(QmiImu_t), GFP_KERNEL);
	if(!qmi8610) {
		QMIIMU_ERR("%s: can't allocate memory for QmiImu_data!\n", __func__);
		err = -ENOMEM;
		goto exit;
	}
	atomic_set(&qmi8610->layout, 0);

	mutex_init(&qmi8610->op_mutex);
	qmi8610->acc_delay = 600;
	qmi8610->gyro_delay = 600;
	qmi8610->acc_enable = 0;
	qmi8610->gyro_enable = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		QMIIMU_ERR("%s: i2c function not support!\n", __func__);
		kfree(qmi8610);
		err = -ENODEV;
		return err;
	}

	qmi8610->client = client;
	i2c_set_clientdata(client, qmi8610);
	g_qmiimu = qmi8610;

#ifdef CONFIG_OF
	if(client->dev.of_node)
	{
		QmiImu_acc_parse_dt(&client->dev);
	}
#endif
//add  stephen ++
	err = gpio_request(g_qmiimu->rst_gpio, "qmi8610 rst");
	if(err)
		pr_err("stephen qmi8610 err to  request rst:%d\n",g_qmiimu->rst_gpio);
	//高电平复位
	gpio_direction_output(g_qmiimu->rst_gpio, 1);
	msleep(100);
	gpio_direction_output(g_qmiimu->rst_gpio, 0);
	gpio_free(g_qmiimu->rst_gpio);
//add  stephen  --


	printk("atomic_read qmi8610 position=%d\n",atomic_read(&qmi8610->layout));
	QmiImu_set_layout(atomic_read(&qmi8610->layout));

//	qmi8610->config.inputSelection = QMIIMU_CONFIG_ACCGYR_ENABLE;
//	qmi8610->config.accOdr = AccOdr_256Hz;
//	qmi8610->config.accRange = AccRange_8g;
//	qmi8610->config.gyrOdr = GyrOdr_256Hz;
//	qmi8610->config.gyrRange = GyrRange_1024dps;
//	qmi8610->config.magOdr = MagOdr_32Hz;
//	qmi8610->config.magDev = MagDev_AK8963;
//	qmi8610->config.aeOdr = AeOdr_32Hz;

	qmi8610->chip_id = 0x00;
#if 1
	qmi8610->chip_id = QmiImu_get_chipid();
	if(qmi8610->chip_id != 0xfc)
	{
		QMIIMU_ERR("%s: g_qmiimu read id fail!\n", __func__);
		goto exit1;
	}	
	g_qmiimu->config.inputSelection = QMIIMU_CONFIG_ACCGYR_ENABLE;
	g_qmiimu->config.accOdr = AccOdr_256Hz;
	g_qmiimu->config.accRange = AccRange_8g;
	g_qmiimu->config.gyrOdr = GyrOdr_256Hz;
	g_qmiimu->config.gyrRange = GyrRange_1024dps;
	g_qmiimu->config.magOdr = MagOdr_32Hz;
	g_qmiimu->config.magDev = MagDev_AK8963;
	g_qmiimu->config.aeOdr = AeOdr_32Hz;
	QmiImu_config_apply(&g_qmiimu->config);
	QmiImu_disableSensors(QMIIMU_CONFIG_ACCGYR_ENABLE);
#endif

	err = QmiImu_acc_input_init();
	if(err < 0) 
	{
		QMIIMU_ERR("acc input init fail!\n");
		goto exit1;
	}	
	err = QmiImu_gyro_input_init();
	if(err < 0) 
	{
		QMIIMU_ERR("gyro input init fail!\n");
		goto exit2;
	}
	
	err = misc_register(&QmiImu_device);
	if (err) {
		QMIIMU_ERR("%s: create register fail!\n", __func__);
		goto exit3;
	}

	err = sysfs_create_group(&qmi8610->acc_input->dev.kobj, &QmiImu_acc_attribute_group);
	if(err < 0) {
		QMIIMU_ERR("%s: create group fail!\n", __func__);
		goto exit4;
	}	
	err = sysfs_create_group(&qmi8610->gyro_input->dev.kobj, &QmiImu_gyro_attribute_group);
	if(err < 0) {
		sysfs_remove_group(&qmi8610->acc_input->dev.kobj, &QmiImu_acc_attribute_group);
		QMIIMU_ERR("%s: create group fail!\n", __func__);
		goto exit4;
	}
/*
	err = sysfs_create_group(&QmiImu_device->this_device.kobj, &QmiImu_attribute_group);
	if(err < 0) {
		QMIIMU_ERR("%s: create group fail!\n", __func__);
		goto exit2;
	}
*/

	INIT_DELAYED_WORK(&qmi8610->acc_work, acc_work_func);
	INIT_DELAYED_WORK(&qmi8610->gyro_work, gyro_work_func);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	qmi8610->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,	
	qmi8610->early_drv.suspend	= QmiImu_early_suspend,
	qmi8610->early_drv.resume	= QmiImu_late_resume,
	register_early_suspend(&qmi8610->early_drv);
#endif
	printk("%s: qmi8610 probe success\n",__func__);

	return 0;

exit4:
	misc_deregister(&QmiImu_device);
exit3:
	QmiImu_gyro_input_deinit();	
exit2:
	QmiImu_acc_input_deinit();
exit1:
	kfree(qmi8610);
exit:
	return err;	
}



static int QmiImu_i2c_remove(struct i2c_client *client)
{
	QMIIMU_FUN();
	if(g_qmiimu)
	{
		sysfs_remove_group(&g_qmiimu->acc_input->dev.kobj, &QmiImu_acc_attribute_group);
		sysfs_remove_group(&g_qmiimu->gyro_input->dev.kobj, &QmiImu_gyro_attribute_group);
		QmiImu_acc_input_deinit();
		QmiImu_gyro_input_deinit();
		misc_deregister(&QmiImu_device);
		kfree(g_qmiimu);
		g_qmiimu = NULL;
	}
	return 0;
}
/*
static int QmiImu_suspend(struct i2c_client *client, pm_message_t mesg)
{
	QMIIMU_FUN();

	//QmiImu_set_enable(QMI_IMU_ACC, 0);
	//QmiImu_set_enable(QMI_IMU_GYRO, 0);
	//QmiImu_set_mode(QMI_MODE_POWER_DOWN);	
	cancel_delayed_work(&g_qmiimu->acc_work);
	cancel_delayed_work(&g_qmiimu->gyro_work);
	QmiImu_disableSensors(QMIIMU_CONFIG_ACCGYR_ENABLE);

	return 0;
}

static int QmiImu_resume(struct i2c_client *client)
{
	if((g_qmiimu->acc_enable)||(g_qmiimu->gyro_enable))
	{
		QmiImu_config_apply(&g_qmiimu->config);
		QmiImu_enableSensors(QMIIMU_CONFIG_ACCGYR_ENABLE);
		if(g_qmiimu->acc_enable)		
			schedule_delayed_work(&g_qmiimu->acc_work, msecs_to_jiffies(600));
		if(g_qmiimu->gyro_enable)		
			schedule_delayed_work(&g_qmiimu->gyro_work, msecs_to_jiffies(600));
	}

	return 0;
}
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void QmiImu_early_suspend(struct early_suspend* es)
{
	QMIIMU_FUN();
	QmiImu_suspend(g_qmiimu->client,(pm_message_t){.event=0});
	pr_err("stephen QmiImu_early_suspend\n");
}

static void QmiImu_late_resume(struct early_suspend* es)
{
	QMIIMU_FUN();
	QmiImu_resume(g_qmiimu->client);
	pr_err("stephen QmiImu_late_resume\n");
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static const struct i2c_device_id QmiImu_id[] = {
	{QMIIMU_DEV_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, QmiImu_id);

static const struct of_device_id QmiImu_acc_of_match[] = {
       { .compatible = "qst,qmi8610", },
       { }
};

MODULE_DEVICE_TABLE(of, QmiImu_acc_of_match);
static struct i2c_driver QmiImu_driver = {
	.driver = {
		.name = QMIIMU_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = QmiImu_acc_of_match,
	},
	.probe    = QmiImu_i2c_probe,
	.remove   = QmiImu_i2c_remove,
//	.suspend  = QmiImu_suspend,
//	.resume   = QmiImu_resume,
	.id_table = QmiImu_id,
};

static int __init QmiImu_i2c_init(void)
{
	QMIIMU_LOG("QmiImu_i2c_init\n");
	return i2c_add_driver(&QmiImu_driver);
}

static void __exit QmiImu_i2c_exit(void)
{
	QMIIMU_LOG("QmiImu_i2c_exit\n");
	i2c_del_driver(&QmiImu_driver);
}


module_init(QmiImu_i2c_init);
//late_initcall(QmiImu_i2c_init);
module_exit(QmiImu_i2c_exit);

MODULE_DESCRIPTION("qmi8610 driver");
MODULE_AUTHOR("yangzhiqiang@qstcorp.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("v1.2");
