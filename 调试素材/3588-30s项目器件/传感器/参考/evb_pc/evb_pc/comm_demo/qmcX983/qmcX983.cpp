/**
  ******************************************************************************
  * @file    qmcX983.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief    qmcX983驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stdafx.h"
#include "qmcX983.h"

#define QMCX983_LOG		_cprintf
#define QMCX983_ERR		_cprintf

#if 0
struct hwmsen_convert {
	short sign[4];
	short map[4];
};


struct hwmsen_convert qmcX983_map[] = {
	{ { 1, 1, 1}, {0, 1, 2} },
	{ {-1, 1, 1}, {1, 0, 2} },
	{ {-1, -1, 1}, {0, 1, 2} },
	{ { 1, -1, 1}, {1, 0, 2} },

	{ {-1, 1, -1}, {0, 1, 2} },
	{ { 1, 1, -1}, {1, 0, 2} },
	{ { 1, -1, -1}, {0, 1, 2} },
	{ {-1, -1, -1}, {1, 0, 2} }

};
#endif

struct qmcX983_data
{
	unsigned char			slave;
	unsigned char			chip_id;
	unsigned char			layout;
	//kal_int16				lsb_1g;	
	short					xy_sensitivity;
	short					z_sensitivity;
	int						OTP_Kx;
	int						OTP_Ky;
	//struct hwmsen_convert *cvt;
};


static struct qmcX983_data g_qmcX983;



static void qmcX983_delay(unsigned int delay)
{
	Sleep(delay);
}

/**
  * @brief   写数据到qmcX983寄存器
  * @param   
  * @retval  
  */
unsigned char qmcX983_WriteReg(unsigned char reg_add,unsigned char reg_dat)
{
	int ret = 0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = spi_write_reg(reg_add, reg_dat);
		}
		else
		{
			ret = i2c_write_reg(g_qmcX983.slave, reg_add, reg_dat);
		}

	}
	
	return ret;
}

/**
  * @brief   从qmcX983寄存器读取数据
  * @param   
  * @retval  
  */
unsigned char qmcX983_ReadData(unsigned char reg_add,unsigned char* Read, unsigned short num)
{
	int ret = 0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = spi_read_reg(reg_add, Read, num);
		}
		else
		{
			ret = i2c_read_reg(g_qmcX983.slave, reg_add, Read, num);
		}
	}

	return ret;
}


/**
  * @brief   读取qmcX983的ID
  * @param   
  * @retval  正常返回1，异常返回0
  */
static unsigned char qmcX983_device_check(void)
{
	unsigned char databuf[2] = {0};
	unsigned char ret = 0; 

	ret = qmcX983_ReadData(0x0d, databuf, 1);
	if(ret == 0){
		QMCX983_LOG("%s: read 0x0d failed\n",__func__);
		return ret;
	}

	g_qmcX983.chip_id = 0;
	QMCX983_LOG("qmcX983_device_check 0x0d=%02x \n",databuf[0]);
#if 0
	if(0xff == databuf[0])
	{
		g_qmcX983.chip_id = QMC6983_A1_D1;
	}
	else if(0x31 == databuf[0])
	{
		g_qmcX983.chip_id = QMC6983_E1;
	}
	else 
#endif
	if(0x32 == databuf[0])
	{
		//read otp 0x30
		ret = qmcX983_WriteReg(0x2e, 0x01);
		if(ret == 0)
		{
			QMCX983_LOG("write 0x2e=0x01 failed\n");
			return ret; 		
		}
		
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: read 0x2f failed\n",__func__);
			return ret;
		}		
		if(((databuf[0]&0x04 )>> 2))
		{
			g_qmcX983.chip_id = QMC6983_E1_Metal;
		}else
		{
			//read otp 0x3e
			ret = qmcX983_WriteReg(0x2e, 0x0f);
			if(ret == 0)
			{
				QMCX983_LOG("%s: write 0x2e=0x0f failed\n",__func__);
				return ret; 		
			}
			ret = qmcX983_ReadData(0x2f, databuf, 1);
			if(ret == 0)
			{
				QMCX983_LOG("%s: read 0x2f failed\n",__func__);
				return ret;
			}
			if(0x02 == ((databuf[0]&0x3c)>>2))
			{
				g_qmcX983.chip_id = QMC7983_Vertical;
			}
			if(0x03 == ((databuf[0]&0x3c)>>2))
			{
				g_qmcX983.chip_id = QMC7983_Slope;
			}
		}
	}
	
	QMCX983_LOG("%s: get chip_id  %d\n",__func__, g_qmcX983.chip_id);
	return g_qmcX983.chip_id;
}

static void qmcX983_start_measure(void)
{
	qmcX983_WriteReg(CTL_REG_ONE, 0x1d);
}

static void qmcX983_stop_measure(void)
{
	qmcX983_WriteReg(CTL_REG_ONE, 0x1c);
}

static void qmcX983_enable(void)
{
	unsigned char ret = 0;	

	ret = qmcX983_WriteReg(0x21, 0x01);

	ret = qmcX983_WriteReg(0x20, 0x40);

  if(g_qmcX983.chip_id != QMC6983_A1_D1)
	{		
		ret = qmcX983_WriteReg(0x29, 0x80);		
		ret = qmcX983_WriteReg(0x0a, 0x0c);
	}
	
	if((g_qmcX983.chip_id == QMC6983_E1_Metal) || (g_qmcX983.chip_id == QMC7983_Slope))
	{			
		ret = qmcX983_WriteReg(0x1b, 0x80);
	}
	
	ret = qmcX983_WriteReg(0x0b, 0x01);				//the ratio must not be 0, different with qmc5983
	ret = qmcX983_WriteReg(0x0b, 0x01);				//the ratio must not be 0, different with qmc5983
	qmcX983_delay(30);
	qmcX983_start_measure();
}


static int qmcx983_get_OPT(void)
{
	unsigned char databuf[2] = {0};
	unsigned char value[2] = {0};
	unsigned char ret = 0;

	if(g_qmcX983.chip_id == QMC6983_A1_D1)
	{
		g_qmcX983.OTP_Kx = 0;
		g_qmcX983.OTP_Ky = 0;
		return 0;
	}
	else
	{
		//read otp_kx
		ret = qmcX983_WriteReg(0x2e, 0x0a);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        qmcX983_delay(10);
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
    
		if(((value[0]&0x3f) >> 5) == 1)
		{
			g_qmcX983.OTP_Kx = (value[0]&0x1f)-32;
		}
		else
		{
			g_qmcX983.OTP_Kx = value[0]&0x1f;	
		}
		//read otp_ky
		ret = qmcX983_WriteReg(0x2e, 0x0d);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        qmcX983_delay(10);
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[0] = databuf[0];
		qmcX983_delay(10);
		ret = qmcX983_WriteReg(0x2e, 0x0f);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_TxData failed\n",__func__);
			return ret;			
		}
        qmcX983_delay(10);
		ret = qmcX983_ReadData(0x2f, databuf, 1);
		if(ret == 0)
		{
			QMCX983_LOG("%s: I2C_RxData failed\n",__func__);
			return ret;
		}
        value[1] = databuf[0];
		QMCX983_LOG("otp-y value:[0x%x  0x%x] \n",value[0], value[1]);
		if((value[0] >> 7) == 1)
			g_qmcX983.OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6))-32;
		else
			g_qmcX983.OTP_Ky = (((value[0]&0x70) >> 4)*4 + (value[1] >> 6));	
	}
	QMCX983_LOG("kx:%d ky:%d \n",g_qmcX983.OTP_Kx, g_qmcX983.OTP_Ky);

	return ret;
}


unsigned char qmcX983_init(void)
{
	unsigned char ret;
	

	qmcX983_delay(100);
	g_qmcX983.slave = QMCX983_SLAVE_ADDRESS_L;
	if(!qmcX983_device_check())
	{
		g_qmcX983.slave = QMCX983_SLAVE_ADDRESS_H;
		if(!qmcX983_device_check())
		{
			return 0;
		}
	}

	ret = qmcX983_WriteReg(0x09,0x1d);

//	g_qmcX983.layout = 1;
//	g_qmcX983.cvt = &qmcX983_map[g_qmcX983.layout];

	qmcX983_stop_measure();
	qmcX983_enable();
	qmcx983_get_OPT();
	
	return 1;
}


int qmcX983_read_mag_xyz(float *data)
{
	unsigned char ret;
	unsigned char mag_data[6];
	int hw_d[3] = {0};

	float output[3]={0};
	int t1 = 0;
	unsigned char rdy = 0;

#if 1
	while(!(rdy & 0x07) && (t1<3)){
		ret = qmcX983_ReadData(STA_REG_ONE,&rdy,1);
		t1++;
	}
#endif

	ret = qmcX983_ReadData(OUT_X_L,mag_data,6);
	if(ret == 0)
    {
		return -1;
	}

	hw_d[0] = (short)(((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short)(((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short)(((mag_data[5]) << 8) | mag_data[4]);

	output[0] = (float)((float)hw_d[0]/25.0);		// ut,	//mgs  /2.5
	output[1] = (float)((float)hw_d[1]/25.0);		// ut
	output[2] = (float)((float)hw_d[2]/25.0);		// ut
	output[2] = output[2] - output[0]*(float)g_qmcX983.OTP_Kx*0.02f - output[1]*(float)g_qmcX983.OTP_Ky*0.02f;
#if 0
	data[0] = g_qmcX983.cvt->sign[QMCX983_AXIS_X]*output[g_qmcX983.cvt->map[QMCX983_AXIS_X]];
	data[1] = g_qmcX983.cvt->sign[QMCX983_AXIS_Y]*output[g_qmcX983.cvt->map[QMCX983_AXIS_Y]];
	data[2] = g_qmcX983.cvt->sign[QMCX983_AXIS_Z]*output[g_qmcX983.cvt->map[QMCX983_AXIS_Z]];
#else
	data[0] = output[0];
	data[1] = output[1];
	data[2] = output[2];
#endif

	return (int)ret;
}


