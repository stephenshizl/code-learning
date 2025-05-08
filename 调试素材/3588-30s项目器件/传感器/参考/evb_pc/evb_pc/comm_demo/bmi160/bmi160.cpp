
#include "stdafx.h"
#include "bmi160.h"


#define bmi160_printf					_cprintf


static unsigned char bmi160_slave_addr = 0x68;
static unsigned short acc_lsb_div = 0;
static unsigned short gyro_lsb_div = 0;

void bmi160_delay(int ms)
{
	Sleep(ms);
}

unsigned char bmi160_write_reg(unsigned char reg, unsigned char value)
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
			ret = i2c_write_reg(bmi160_slave_addr, reg, value);
		}

	}
	return ret;
}

unsigned char bmi160_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	int i, ret;

	for(i=0; i<len; i++)
	{
		ret = bmi160_write_reg(reg+i, value[i]);
	}

	return ret;
}

unsigned char bmi160_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
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
			ret = i2c_read_reg(bmi160_slave_addr, reg, buf, len);
		}
	}
	return ret;
}

void bmi160_set_range(unsigned char acc_range, unsigned char gyro_range)
{
	if(acc_range == BMI160_ACCEL_RANGE_2G)
		acc_lsb_div = (1<<14);
	else if(acc_range == BMI160_ACCEL_RANGE_4G)
		acc_lsb_div = (1<<13);
	else if(acc_range == BMI160_ACCEL_RANGE_8G)
		acc_lsb_div = (1<<12);
	else if(acc_range == BMI160_ACCEL_RANGE_16G)
		acc_lsb_div = (1<<11);
		
	bmi160_write_reg(BMI160_ACCEL_RANGE_ADDR, acc_range);	// odr 100hz

	if(gyro_range == BMI160_GYRO_RANGE_125_DPS)
		gyro_lsb_div = BMI160_FS_125_LSB;
	else if(gyro_range == BMI160_GYRO_RANGE_250_DPS)
		gyro_lsb_div = BMI160_FS_250_LSB;
	else if(gyro_range == BMI160_GYRO_RANGE_500_DPS)
		gyro_lsb_div = BMI160_FS_500_LSB;
	else if(gyro_range == BMI160_GYRO_RANGE_1000_DPS)
		gyro_lsb_div = BMI160_FS_1000_LSB;
	else if(gyro_range == BMI160_GYRO_RANGE_2000_DPS)
		gyro_lsb_div = BMI160_FS_2000_LSB;
		
	bmi160_write_reg(BMI160_GYRO_RANGE_ADDR, gyro_range);
}

void bmi160_config_int(void)
{
	bmi160_write_reg(BMI160_INT_OUT_CTRL_ADDR, 0xbb);	// int 1,2  edge, high, pp
#if defined(BMI160_INT_LATCH)
	bmi160_write_reg(BMI160_INT_LATCH_ADDR, 0x0b); // temporary, 320ms
#else	
	bmi160_write_reg(BMI160_INT_LATCH_ADDR, 0x00);	// non-latched
#endif
#if defined(BMI160_ANY_MOTION_INT)
	// any motion
	bmi160_write_reg(BMI160_INT_ENABLE_0_ADDR, 0x07);	// any motion enable xyz
	bmi160_write_reg(BMI160_INT_MAP_0_ADDR, 0x04);	// any motion map to int1
	bmi160_write_reg(BMI160_INT_DATA_1_ADDR, 0x00);	//selects filtered (pre-filtered)
	//threshold of any-motion:3.91mg@2g, 7.81mg@4g, 15.63mg@8g, 31.25mg@16g
	bmi160_write_reg(BMI160_INT_MOTION_1_ADDR, 0x02);
#endif
#if defined(BMI160_NO_MOTION_INT)
	// no/slow motion
	bmi160_write_reg(BMI160_INT_ENABLE_2_ADDR, 0x07);	// no/slow motion enable xyz
	bmi160_write_reg(BMI160_INT_MAP_0_ADDR, 0x0c);	// any/no motion map to int1	
	// slow/no motion duration
	bmi160_write_reg(BMI160_INT_MOTION_0_ADDR, 0x0c);		// 0x1c
	//threshold of no/slow-motion:3.91mg@2g, 7.81mg@4g, 15.63mg@8g, 31.25mg@16g
	bmi160_write_reg(BMI160_INT_MOTION_2_ADDR, 0x0b);
	//bit 0--> 1:select no-motion 0:select slow-motion
	bmi160_write_reg(BMI160_INT_MOTION_3_ADDR, 0x01);
#endif
#if defined(BMI160_DRDY_INT)
	bmi160_write_reg(BMI160_INT_ENABLE_1_ADDR, 0x10);	// drdy enable
	bmi160_write_reg(BMI160_INT_MAP_1_ADDR, 0x80);	// any motion map to int1
#endif
}

void bmi160_read_acc_xyz(float acc[3])
{
	unsigned char	buf_reg[6];
	short			raw_acc_xyz[3];
	float			acc_xyz_tmp[3];	

	bmi160_read_reg(BMI160_ACCEL_DATA_ADDR, buf_reg, 6);
		
	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	acc_xyz_tmp[0] = (raw_acc_xyz[0]*9.807f)/acc_lsb_div;
	acc_xyz_tmp[1] = (raw_acc_xyz[1]*9.807f)/acc_lsb_div;
	acc_xyz_tmp[2] = (raw_acc_xyz[2]*9.807f)/acc_lsb_div;

	acc[0] = acc_xyz_tmp[0];
	acc[1] = acc_xyz_tmp[1];
	acc[2] = acc_xyz_tmp[2];
}


void bmi160_read_gyro_xyz(float gyr[3])
{
	float			gyro_xyz_tmp[3];	
	unsigned char	buf_reg[6];
	short			raw_gyro_xyz[3];

	bmi160_read_reg(BMI160_GYRO_DATA_ADDR, buf_reg, 6);

	raw_gyro_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));	

	gyro_xyz_tmp[0] = (raw_gyro_xyz[0]*10.0f)/gyro_lsb_div;
	gyro_xyz_tmp[1] = (raw_gyro_xyz[1]*10.0f)/gyro_lsb_div;
	gyro_xyz_tmp[2] = (raw_gyro_xyz[2]*10.0f)/gyro_lsb_div;
}

void bmi160_read_xyz(float acc[3], float gyr[3])
{
	unsigned char	buf_reg[12];
	short			raw_gyro_xyz[3];
	short			raw_acc_xyz[3];

	bmi160_read_reg(BMI160_GYRO_DATA_ADDR, buf_reg, 12);

	raw_gyro_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));

	raw_acc_xyz[0] = (short)((buf_reg[7]<<8) |( buf_reg[6]));
	raw_acc_xyz[1] = (short)((buf_reg[9]<<8) |( buf_reg[8]));
	raw_acc_xyz[2] = (short)((buf_reg[11]<<8) |( buf_reg[10])); 

	gyr[0] = (raw_gyro_xyz[0]*10.0f)/gyro_lsb_div;
	gyr[1] = (raw_gyro_xyz[1]*10.0f)/gyro_lsb_div;
	gyr[2] = (raw_gyro_xyz[2]*10.0f)/gyro_lsb_div;

	acc[0] = (raw_acc_xyz[0]*9.807f)/acc_lsb_div;
	acc[1] = (raw_acc_xyz[1]*9.807f)/acc_lsb_div;
	acc[2] = (raw_acc_xyz[2]*9.807f)/acc_lsb_div;
}



unsigned char bmi160_init(void)
{
	unsigned char bmi160_chip_id = 0x00;
	unsigned char bmi160_slave[2] = {0x68, 0x69};
	unsigned char iCount = 0;
	int retry = 0;

	while(iCount<2)
	{
		bmi160_slave_addr = bmi160_slave[iCount];
		retry = 0;
		while((bmi160_chip_id != BMI160_CHIP_ID)&&(retry++ < 5))
		{
			bmi160_read_reg(BMI160_CHIP_ID_ADDR, &bmi160_chip_id, 1);
			bmi160_printf("bmi160_chip_id = 0x%x\n", bmi160_chip_id);
		}
		if(bmi160_chip_id == BMI160_CHIP_ID)
		{
			break;
		}
		iCount++;
	}
	
	if(bmi160_chip_id == BMI160_CHIP_ID)
	{
		unsigned char pmu_status=0;

		bmi160_write_reg(BMI160_COMMAND_REG_ADDR, BMI160_SOFT_RESET_CMD);
		bmi160_delay(BMI160_SOFT_RESET_DELAY_MS);
		bmi160_write_reg(BMI160_ACCEL_CONFIG_ADDR, (BMI160_ACCEL_BW_NORMAL_AVG4<<4)|BMI160_ACCEL_ODR_100HZ);	// odr 100hz
		bmi160_set_range(BMI160_ACCEL_RANGE_8G, BMI160_GYRO_RANGE_1000_DPS);
		bmi160_write_reg(BMI160_GYRO_CONFIG_ADDR, (BMI160_GYRO_BW_NORMAL_MODE<<4)|BMI160_GYRO_ODR_400HZ);
		bmi160_write_reg(BMI160_COMMAND_REG_ADDR, BMI160_ACCEL_NORMAL_MODE);
		bmi160_delay(5);
		bmi160_write_reg(BMI160_COMMAND_REG_ADDR, BMI160_GYRO_NORMAL_MODE);
		bmi160_delay(BMI160_GYRO_DELAY_MS);
		// check pmu status
		while(pmu_status != 0x14)
		{
			bmi160_read_reg(BMI160_PMU_STAT_ADDR, &pmu_status, 1);
			bmi160_delay(1);
		}
		// check pmu status
		bmi160_config_int();
		return 1;
	}
	else
	{
		bmi160_chip_id = 0;
		bmi160_printf("bmi160_init fail!\n");
		return 0;
	}
}

