
#include "stdafx.h"
#include "lis3dh.h"

static unsigned char lis3dh_slave_addr = 0x18;
static unsigned short acc_lsb_div = 0;
static unsigned short gyro_lsb_div = 0;

void lis3dh_delay(int ms)
{
	Sleep(ms);
}

unsigned char lis3dh_write_reg(unsigned char reg, unsigned char value)
{
	unsigned char ret=0;
	unsigned int retry = 0;

	while((!ret) && (++retry < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			reg &= 0x3f;
			ret = spi_write_reg(reg, value);
		}
		else
		{
			ret = i2c_write_reg(lis3dh_slave_addr, reg, value);
		}

	}
	return ret;
}

unsigned char lis3dh_write_regs(unsigned char reg, unsigned char *value, unsigned char len)
{
	int i, ret;

	for(i=0; i<len; i++)
	{
		ret = lis3dh_write_reg(reg+i, value[i]);
	}

	return ret;
}

unsigned char lis3dh_read_reg(unsigned char reg, unsigned char* buf, unsigned short len)
{
	unsigned char ret=0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			if(len > 1)
				reg |= 0xc0;
			else
				reg |= 0x80;
			ret = spi_read_reg(reg, buf, len);
		}
		else
		{
			ret = i2c_read_reg(lis3dh_slave_addr, reg, buf, len);
		}
	}
	return ret;
}

unsigned char lis3dh_get_slave(void)
{
	return lis3dh_slave_addr;
}


void lis3dh_set_range(unsigned char range)
{
	if(range == LIS3DH_ACCEL_RANGE_2G)
	{
		acc_lsb_div = (1<<14);
		lis3dh_write_reg(0x23, 0x08);
	}
	else if(range == LIS3DH_ACCEL_RANGE_4G)
	{
		acc_lsb_div = (1<<13);
		lis3dh_write_reg(0x23, 0x18);
	}
	else if(range == LIS3DH_ACCEL_RANGE_8G)
	{
		acc_lsb_div = (1<<12);
		lis3dh_write_reg(0x23, 0x28);
	}
	else if(range == LIS3DH_ACCEL_RANGE_16G)
	{
		acc_lsb_div = (1<<11);
		lis3dh_write_reg(0x23, 0x38);
	}

	
}

void lis3dh_config_int(void)
{

}

void lis3dh_read_acc_xyz(float acc[3])
{
	unsigned char	buf_reg[6];
	short			raw_acc_xyz[3];
	float			acc_xyz_tmp[3];	

	lis3dh_read_reg(0x28|0x80, buf_reg, 6);
		
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


void lis3dh_read_tempearture(short *temp)
{
	char temp_reg;

	lis3dh_read_reg(0x0c, (unsigned char*)&temp_reg, 1);
	*temp = temp_reg;
}


unsigned char lis3dh_init(void)
{
	unsigned char lis3dh_chip_id = 0x00;
	unsigned char lis3dh_slave[2] = {0x18, 0x19};
	unsigned char iCount = 0;
	int retry = 0;

	while(iCount<2)
	{
		lis3dh_slave_addr = lis3dh_slave[iCount];
		retry = 0;
		while((lis3dh_chip_id != LIS3DH_CHIP_ID)&&(retry++ <= 1))
		{
			lis3dh_read_reg(0x0f, &lis3dh_chip_id, 1);
			lis3dh_printf("lis3dh_chip_id = 0x%x\n", lis3dh_chip_id);
		}
		if(lis3dh_chip_id == LIS3DH_CHIP_ID)
		{
			break;
		}
		iCount++;
	}
	
	if(lis3dh_chip_id == LIS3DH_CHIP_ID)
	{
		lis3dh_write_reg(0x24, 0x80);
		lis3dh_delay(5);
		lis3dh_write_reg(0x24, 0x00);
		lis3dh_write_reg(0x1f, 0x40);	// temp sensor enable
		lis3dh_write_reg(0x20, 0x57);	// 100Hz, x,y,z enable
		lis3dh_set_range(LIS3DH_ACCEL_RANGE_8G);

		return 1;
	}
	else
	{
		lis3dh_chip_id = 0;
		lis3dh_printf("lis3dh_init fail!\n");
		return 0;
	}
}

