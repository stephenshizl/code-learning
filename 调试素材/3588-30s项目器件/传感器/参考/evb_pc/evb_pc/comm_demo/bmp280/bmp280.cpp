
#include "stdafx.h"
#include "bmp280.h"


#define bmp280_printf					_cprintf

static unsigned char bmp280_slave_addr = 0x68;
struct bmp280_calibration_data bmp280_cali;

void bmp280_delay(int ms)
{
	Sleep(ms);
}

unsigned char bmp280_write_reg(unsigned char reg, unsigned char value)
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
			ret = i2c_write_reg(bmp280_slave_addr, reg, value);
		}

	}
	return ret;
}

unsigned char bmp280_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	int i, ret;

	for(i=0; i<len; i++)
	{
		ret = bmp280_write_reg(reg+i, value[i]);
	}

	return ret;
}

unsigned char bmp280_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
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
			ret = i2c_read_reg(bmp280_slave_addr, reg, buf, len);
		}
	}
	return ret;
}


void bmp280_calc_press(float *pressure, float *tempearture)
{
	unsigned char reg_data[6];
	BMP280_S32_t	P_read, T_read;
	BMP280_S32_t	t1, t2;
	BMP280_S32_t	t_fine, t_raw;
    BMP280_S64_t 	p1, p2, p_raw;

	bmp280_read_reg(BMP280_PRESSURE_MSB_REG, reg_data, 6);
	P_read = (BMP280_S32_t)(((BMP280_U32_t)reg_data[0]<<16)|((BMP280_U32_t)reg_data[1]<<8)|(reg_data[2]));
	P_read = P_read>>4;
	T_read = (BMP280_S32_t)(((BMP280_U32_t)reg_data[3]<<16)|((BMP280_U32_t)reg_data[4]<<8)|(reg_data[5]));
	T_read = T_read>>4;

	//bmp280CompensateT
    t1 = ((((T_read>>3)-((BMP280_S32_t)bmp280_cali.dig_T1<<1)))*((BMP280_S32_t)bmp280_cali.dig_T2))>>11;
    t2  = (((((T_read>>4)-((BMP280_S32_t)bmp280_cali.dig_T1))*((T_read>>4)-((BMP280_S32_t)bmp280_cali.dig_T1)))>>12)*((BMP280_S32_t)bmp280_cali.dig_T3))>> 14;
    t_fine = t1 + t2;
    t_raw = ((t_fine*5+128)>>8);
	*tempearture = t_raw/100.0f;

	
    p1 = ((BMP280_S64_t)t_fine) - 128000;
    p2 = p1 * p1 * (BMP280_S64_t)bmp280_cali.dig_P6;
    p2 = p2 + ((p1*(BMP280_S64_t)bmp280_cali.dig_P5) << 17);
    p2 = p2 + (((BMP280_S64_t)bmp280_cali.dig_P4) << 35);
    p1 = ((p1 * p1 * (BMP280_S64_t)bmp280_cali.dig_P3) >> 8) + ((p1 * (BMP280_S64_t)bmp280_cali.dig_P2) << 12);
    p1 = (((((BMP280_S64_t)1) << 47) + p1)) * ((BMP280_S64_t)bmp280_cali.dig_P1) >> 33;
    if (p1 == 0)
        return;
    p_raw = 1048576 - P_read;
    p_raw = (((p_raw << 31) - p2) * 3125) / p1;
    p1 = (((BMP280_S64_t)bmp280_cali.dig_P9) * (p_raw >> 13) * (p_raw >> 13)) >> 25;
    p2 = (((BMP280_S64_t)bmp280_cali.dig_P8) * p_raw) >> 19;
    p_raw = ((p_raw + p1 + p2) >> 8) + (((BMP280_S64_t)bmp280_cali.dig_P7) << 4);

	*pressure = p_raw/256.0f;		// Pa
	//pressure = p_raw/25600.0f;		//hpa
	//pressure = p_raw/256000.0f;		//kpa
	//elevation = ((pow((101.57/pressure), 1/5.257)-1)*(tempearture+273.15))/0.0065;
}


void bmp280_get_calibration_data(void)
{
	unsigned char buf_wr[BMP280_CALIBRATION_DATA_LENGTH];

	bmp280_read_reg(BMP280_CALIBRATION_DATA_START,buf_wr,BMP280_CALIBRATION_DATA_LENGTH);

	bmp280_cali.dig_T1 = (BMP280_U16_t)((((BMP280_U16_t)(buf_wr[1])) << SHIFT_LEFT_8_POSITION) | buf_wr[0]);
	bmp280_cali.dig_T2 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[3])) << SHIFT_LEFT_8_POSITION) | buf_wr[2]);
	bmp280_cali.dig_T3 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[5])) << SHIFT_LEFT_8_POSITION) | buf_wr[4]);
	bmp280_cali.dig_P1 = (BMP280_U16_t)((((BMP280_U16_t)(buf_wr[7])) << SHIFT_LEFT_8_POSITION) | buf_wr[6]);
	bmp280_cali.dig_P2 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[9])) << SHIFT_LEFT_8_POSITION) | buf_wr[8]);
	bmp280_cali.dig_P3 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[11])) << SHIFT_LEFT_8_POSITION) | buf_wr[10]);
	bmp280_cali.dig_P4 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[13])) << SHIFT_LEFT_8_POSITION) | buf_wr[12]);
	bmp280_cali.dig_P5 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[15])) << SHIFT_LEFT_8_POSITION) | buf_wr[14]);
	bmp280_cali.dig_P6 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[17])) << SHIFT_LEFT_8_POSITION) | buf_wr[16]);
	bmp280_cali.dig_P7 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[19])) << SHIFT_LEFT_8_POSITION) | buf_wr[18]);
	bmp280_cali.dig_P8 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[21])) << SHIFT_LEFT_8_POSITION) | buf_wr[20]);
	bmp280_cali.dig_P9 = (BMP280_S16_t)((((BMP280_S16_t)(buf_wr[23])) << SHIFT_LEFT_8_POSITION) | buf_wr[22]);
	bmp280_printf("Cali data:\n");
	bmp280_printf("%d	%d	%d\n",bmp280_cali.dig_T1,bmp280_cali.dig_T2,bmp280_cali.dig_T3);
	bmp280_printf("%d	%d	%d\n",bmp280_cali.dig_P1,bmp280_cali.dig_P2,bmp280_cali.dig_P3);
	bmp280_printf("%d	%d	%d\n",bmp280_cali.dig_P4,bmp280_cali.dig_P5,bmp280_cali.dig_P6);
	bmp280_printf("%d	%d	%d\n",bmp280_cali.dig_P7,bmp280_cali.dig_P8,bmp280_cali.dig_P9);

}

void bmp280_set_power_mode(unsigned char power_mode)
{
	unsigned char reg_data;

	//bmp280_printf("qmp_set_powermode %d \n", power_mode);
	//if(power_mode == g_qmp6988.power_mode)
	//	return;

	bmp280_read_reg(BMP280_CTRLMEAS_REG, &reg_data, 1);
	reg_data = reg_data&0xfc;
	reg_data |= power_mode;
	bmp280_write_reg(BMP280_CTRLMEAS_REG, reg_data);
	//bmp280_printf("qmp_set_powermode 0xf4=0x%x \n", reg_data);
	//Sleep(5);	//qmp6988_delay(20);
}

void bmp280_set_t_standby(unsigned char standby)
{
	unsigned char reg_data;

	bmp280_read_reg(BMP280_CONFIG_REG, &reg_data, 1);
	reg_data = reg_data&0x1F;
	reg_data |= standby<<5;
	bmp280_write_reg(BMP280_CONFIG_REG,reg_data);
}

void bmp280_set_filter(unsigned char filter)
{	
	unsigned char reg_data;

	if((filter>=BMP280_FILTERCOEFF_OFF) &&(filter<=BMP280_FILTERCOEFF_16))
	{
		bmp280_read_reg(BMP280_CTRLMEAS_REG, &reg_data, 1);
		reg_data = reg_data&0xE3;
		reg_data |= filter<<2;
		bmp280_write_reg(BMP280_CTRLMEAS_REG,reg_data);
	}
	//Sleep(5);
}

void bmp280_set_oversampling_p(unsigned char oversampling_p)
{
	unsigned char reg_data;

	bmp280_read_reg(BMP280_CTRLMEAS_REG, &reg_data, 1);
	if((oversampling_p>=BMP280_OVERSAMPLING_SKIPPED)&&(oversampling_p<=BMP280_OVERSAMPLING_16X))
	{
		reg_data &= 0xe3;
		reg_data |= (oversampling_p<<2);
		bmp280_write_reg(BMP280_CTRLMEAS_REG, reg_data);
	}	
	//Sleep(5);
}

void bmp280_set_oversampling_t(unsigned char oversampling_t)
{
	unsigned char reg_data;

	bmp280_read_reg(BMP280_CTRLMEAS_REG, &reg_data, 1);
	if((oversampling_t>=BMP280_OVERSAMPLING_SKIPPED)&&(oversampling_t<=BMP280_OVERSAMPLING_16X))
	{
		reg_data &= 0x1f;
		reg_data |= (oversampling_t<<5);
		bmp280_write_reg(BMP280_CTRLMEAS_REG, reg_data);
	}	
	//Sleep(5);
}


int bmp280_init(void)
{
	unsigned char bmp280_slave[2] = {0x76, 0x77};
	unsigned char chip_id = 0;
	unsigned char iCount = 0;

	while(iCount<2)
	{
		bmp280_slave_addr = bmp280_slave[iCount];
		bmp280_read_reg(0xd0, &chip_id, 1);
		bmp280_printf("bmp280_init addr=0x%x chip_id=0x%x \n", bmp280_slave_addr, chip_id);		
		if(chip_id == 0x58)
			break;
		iCount++;
	}
	if(chip_id == 0x58)
	{
		bmp280_get_calibration_data();
		bmp280_set_power_mode(BMP280_NORMAL_MODE);
		bmp280_set_filter(BMP280_FILTERCOEFF_OFF);
		bmp280_set_oversampling_p(BMP280_OVERSAMPLING_2X);
		bmp280_set_oversampling_t(BMP280_OVERSAMPLING_1X);
		bmp280_printf("bmp280 find! \n");

		return 1;
	}
	else
	{
		return 0;
	}
}

