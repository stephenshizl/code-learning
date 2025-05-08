
#include "stdafx.h"
#include "mpu6050.h"


#define mpu6050_printf					_cprintf


static unsigned char mpu6050_slave_addr = 0x68;
static float acc_lsb_div = 0;
static float gyro_lsb_div = 0;

void mpu6050_delay(int ms)
{
	Sleep(ms);
}

unsigned char mpu6050_write_reg(unsigned char reg, unsigned char value)
{
	unsigned char ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			reg &= 0x7f;
			ret = spi_write_reg(reg, value);
		}
		else
		{
			ret = i2c_write_reg(mpu6050_slave_addr, reg, value);
		}

	}
	return ret;
}

unsigned char mpu6050_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	int i, ret;

	for(i=0; i<len; i++)
	{
		ret = mpu6050_write_reg(reg+i, value[i]);
	}

	return ret;
}

unsigned char mpu6050_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	unsigned char ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			reg |= 0x80;
			ret = spi_read_reg(reg, buf, len);
		}
		else
		{
			ret = i2c_read_reg(mpu6050_slave_addr, reg, buf, len);
		}
	}
	return ret;
}

void mpu6050_set_range(unsigned char acc_range, unsigned char gyro_range)
{
	if(acc_range == 0)
	{
		acc_lsb_div = 16384.0f;
		mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG , 0x00);
	}
	else if(acc_range == 1)
	{
		acc_lsb_div = 8192.0f;
		mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG , 0x08);
	}
	else if(acc_range == 2)
	{
		acc_lsb_div = 4096.0f;
		mpu6050_write_reg(MPU6050_RA_ACCEL_CONFIG , 0x10);
	}
	else if(acc_range == 3)
	{
		acc_lsb_div = 2048.0f;
		mpu6050_write_reg(0x1C , 0x18);
	}

	if(gyro_range == 0)
	{
		gyro_lsb_div = 131.0f;
		mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, 0x00);
	}
	else if(gyro_range == 1)
	{
		gyro_lsb_div = 65.5f;
		mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, 0x08);
	}
	else if(gyro_range == 2)
	{
		gyro_lsb_div = 32.8f;
		mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, 0x10);
	}
	else if(gyro_range == 3)
	{
		gyro_lsb_div = 16.4f;
		mpu6050_write_reg(MPU6050_RA_GYRO_CONFIG, 0x18);
	}
}

void mpu6050_config_int(void)
{

}


void mpu6050_read_data(float acc[3], float gyr[3], float *tempearture)
{
	unsigned char reg_data[14];
	unsigned char drdy = 0;
	short raw_acc_xyz[3];
	short raw_gyro_xyz[3];

	reg_data[0] = 0;
	while(!(drdy & 0x01))
	{
		mpu6050_read_reg(0x3a, &drdy, 1);
		//mpu6050_printf("0x3a = 0x%x\n", drdy);
	}
	
	mpu6050_read_reg(0x3b, reg_data, 14);
	raw_acc_xyz[0] = (short)((reg_data[0]<<8) |( reg_data[1]));
	raw_acc_xyz[1] = (short)((reg_data[2]<<8) |( reg_data[3]));
	raw_acc_xyz[2] = (short)((reg_data[4]<<8) |( reg_data[5]));

	acc[0] = (raw_acc_xyz[0]*9.807f)/acc_lsb_div;
	acc[1] = (raw_acc_xyz[1]*9.807f)/acc_lsb_div;
	acc[2] = (raw_acc_xyz[2]*9.807f)/acc_lsb_div;
	
	*tempearture = (float)(((short)((reg_data[6]<<8) |( reg_data[7])))/340.0f) + 36.53f;
	raw_gyro_xyz[0] = (short)((reg_data[8]<<8) |( reg_data[9]));
	raw_gyro_xyz[1] = (short)((reg_data[10]<<8) |( reg_data[11]));
	raw_gyro_xyz[2] = (short)((reg_data[12]<<8) |( reg_data[13]));
	gyr[0] = raw_gyro_xyz[0]/gyro_lsb_div;
	gyr[1] = raw_gyro_xyz[1]/gyro_lsb_div;
	gyr[2] = raw_gyro_xyz[2]/gyro_lsb_div;
}

unsigned char mpu6050_init(void)
{
	unsigned char mpu6050_chip_id = 0x00;
	unsigned char mpu6050_slave[2] = {MPU6050_ADDRESS_AD0_LOW, MPU6050_ADDRESS_AD0_HIGH};
	unsigned char iCount = 0;
	int retry = 0;

	while(iCount<2)
	{
		mpu6050_slave_addr = mpu6050_slave[iCount];
		retry = 0;
		while((mpu6050_chip_id != MPU6050_CHIP_ID)&&(retry++ < 5))
		{
			mpu6050_read_reg(MPU6050_WHO_AM_I, &mpu6050_chip_id, 1);
			mpu6050_printf("mpu6050_chip_id = 0x%x\n", mpu6050_chip_id);
		}
		if(mpu6050_chip_id == MPU6050_CHIP_ID)
		{
			break;
		}
		iCount++;
	}
	
	if(mpu6050_chip_id == MPU6050_CHIP_ID)
	{
		mpu6050_write_reg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //解除休眠状态
		mpu6050_delay(5);
		mpu6050_write_reg(MPU6050_RA_SMPLRT_DIV, 0x05);	    //陀螺仪采样率
		mpu6050_write_reg(MPU6050_RA_CONFIG, 0x03);		// 1K Hz
		mpu6050_set_range(2, 2);		// +- 8, 1000 dps
		mpu6050_write_reg(MPU6050_RA_I2C_MST_CTRL, 0x0d);	    //置 MPU-60X0 内部 8MHz 时钟的分频器。设置 I2C 主机时钟频率, 400K
		return 1;
	}
	else
	{
		mpu6050_chip_id = 0;
		mpu6050_printf("mpu6050_init fail!\n");
		return 0;
	}
}

