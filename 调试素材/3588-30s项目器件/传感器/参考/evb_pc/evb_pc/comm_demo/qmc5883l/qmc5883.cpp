
#include "stdafx.h"
#include "qmc5883.h"

unsigned char qmc5883_slave = QMC5883L_SLAVE_L;

void Qmc5883_delay(int delay)
{
	Sleep(delay);
}

int Qmc5883_ReadReg(unsigned char Reg, unsigned char *value, unsigned short len)
{
	int ret = 0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = spi_read_reg(Reg, value, len);
		}
		else
		{
			ret = i2c_read_reg(qmc5883_slave, Reg, value, len);
		}
	}

	return ret;

}

int Qmc5883_WriteReg(unsigned char Reg, unsigned char Val)
{
	unsigned char ret = 0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = spi_write_reg(Reg, Val);
		}
		else
		{
			ret = i2c_write_reg(qmc5883_slave, Reg, Val);
		}
	}

	return ret;
}

unsigned char Qmc5883_InitConfig(void)
{
	unsigned char Temp;
	unsigned char slaveArray[2] = {QMC5883L_SLAVE_L, QMC5883L_SLAVE_H};
	int  index=0;

	for(index=0; index<2; index++)
	{
		qmc5883_slave = slaveArray[index];
		Qmc5883_WriteReg(0x0B, 0x01);
		Qmc5883_WriteReg(0x20, 0x40);
		Qmc5883_WriteReg(0x21, 0x01);
		Qmc5883_WriteReg(0x09, 0x0D);/****OSR=512,RNG=+/-2G,ODR=200Hz,MODE= continuous*******/
		//QMC5883 initial cost a lot time.
		Qmc5883_delay(5);
		Qmc5883_ReadReg(0x09, &Temp, 1);
		while(Temp == 0x0D)
		{
			QMC5883L_LOG("slave:0x%x Qmc5883_InitConfig OK\n",qmc5883_slave);
			return 1;
		}
		QMC5883L_LOG("slave:0x%x Qmc5883_InitConfig fail\n",qmc5883_slave);
	}
	return 0;
}

unsigned char Qmc5883_GetData(float *Magnet)
{
	unsigned char Buff[6];
	unsigned char Temp = 0;
	short MagnetRawAd[3];

	Qmc5883_ReadReg(0x06, &Temp, 1);
	if(Temp == 0)
	{
		return 0;
	}

	Qmc5883_ReadReg(QMC5883L_REG_DATA_OUTPUT_X, &Buff[0], 6);

	MagnetRawAd[0] = (short)((Buff[1] << 8) | Buff[0]);
	MagnetRawAd[1] = (short)((Buff[3] << 8) | Buff[2]);
	MagnetRawAd[2] = (short)((Buff[5] << 8) | Buff[4]);

//	Magnet[0] = -(float)MagnetRawAd[0] / 12000.f;
//	Magnet[1] = (float)MagnetRawAd[1] / 12000.f;
//	Magnet[2] = -(float)MagnetRawAd[2] / 12000.f;
	
//	Magnet[0] = -(float)MagnetRawAd[0] / 120.f;
//	Magnet[1] = (float)MagnetRawAd[1] / 120.f;
//	Magnet[2] = -(float)MagnetRawAd[2] / 120.f;
	
	Magnet[0] = (float)MagnetRawAd[0] / 120.f;
	Magnet[1] = -(float)MagnetRawAd[1] / 120.f;
	Magnet[2] = -(float)MagnetRawAd[2] / 120.f;

	return 1;
}
