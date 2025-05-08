/**
  ******************************************************************************
  * @file    qma7981.c
  * @author  Yangzhiqiang@qst
  * @version V1.0
  * @date    2017-12-15
  * @brief    qmaX981
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */ 
#include "stdafx.h"
#include "bma25x.h"

typedef struct
{
    short sign[3];
    unsigned short map[3];
}qst_convert;

typedef struct
{
	unsigned char					slave;
	unsigned char					chip_id;
	int				lsb_1g;
	unsigned char					layout;
	qst_convert			cvt;
	unsigned char					fifo_mode;
	int				fifo_len;
} bma25x_data;

static const qst_convert qst_map[] = 
{
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1, -1}, {0, 1, 2} },
    { { 1, 1, -1}, {1, 0, 2} },
    { { 1,-1, -1}, {0, 1, 2} },
    { {-1,-1, -1}, {1, 0, 2} }
};

static bma25x_data g_bma25x;

void bma25x_delay(unsigned int delay)
{
	Sleep(delay);
}

int bma25x_writereg(unsigned char reg_add, unsigned char reg_dat)
{
	int ret = 0;
	unsigned int retry = 0;	

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = 0;
		}
		else
		{
			ret = i2c_write_reg(g_bma25x.slave, reg_add, reg_dat);
		}
	}

	if(ret)
		return BMA25X_SUCCESS;
	else
		return BMA25X_ERROR;
}

int bma25x_readreg(unsigned char reg_add, unsigned char *buf, unsigned short num)
{
	int ret = 0;
	unsigned int retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = 0;
		}
		else
		{
			ret = i2c_read_reg(g_bma25x.slave, reg_add, buf, (unsigned short)num);
		}
	}

	if(ret)
		return BMA25X_SUCCESS;
	else
		return BMA25X_ERROR;
}

unsigned char bma25x_get_slave(void)
{
	return g_bma25x.slave;
}

unsigned char bma25x_chip_id(void)
{
	unsigned char chip_id = 0x00;
	unsigned int retry = 0;

	while(chip_id != BMA25X_DEVICE_ID)
	{
		bma25x_readreg(BMA25X_REG_ID, &chip_id, 1);
		BMA25X_LOG("bma25x_chip_id id=0x%x \n", chip_id);
		if(++retry >= 1)
			break;
	}

	return chip_id;
}

int bma25x_set_range(int range)
{
	int ret = 1;
	unsigned char range_reg = 0;

	if(range == BMA25X_RANGE_2G)
	{
		g_bma25x.lsb_1g = 1024;
		range_reg = 0x03;
	}
	else if(range == BMA25X_RANGE_4G)
	{
		g_bma25x.lsb_1g = 512;
		range_reg = 0x05;
	}
	else if(range == BMA25X_RANGE_8G)
	{
		g_bma25x.lsb_1g = 256;
		range_reg = 0x08;
	}
	else if(range == BMA25X_RANGE_16G)
	{
		g_bma25x.lsb_1g = 128;
		range_reg = 0x0c;
	}
	else
	{
		g_bma25x.lsb_1g = 1024;
		range_reg = 0x03;
	}
	
	ret = bma25x_writereg(0x0F, range_reg);

	return ret;
}

void bma25x_dump_reg(void)
{
#if 0
	unsigned char reg_data = 0;
	int i=0;
	unsigned char reg_map[]=
	{
		0x09,0x0a,0x0b,0x0c,0x0e,
		0x0f,0x10,0x11,0x12,0x13,0x14,0x15,
		0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,
		0x1D,0x1E,0x1F,0x20
	};

	BMA25X_LOG("bma25x_dump_reg\n");
	for(i=0; i< sizeof(reg_map)/sizeof(reg_map[0]); i++)
	{
		bma25x_readreg(reg_map[i],&reg_data,1);
		BMA25X_LOG("0x%x = 0x%x	", reg_map[i], reg_data);
	}
	BMA25X_LOG("\n");
#endif
}

void bma25x_irq_hdlr(void)
{
	

}


int bma25x_read_raw(int *rawData)
{
	unsigned char databuf[6] = {0};	
	int ret;

	ret = bma25x_readreg(0x02, databuf, 6);
	if(ret == BMA25X_ERROR)
	{
		BMA25X_ERR("read xyz error!!!");
		return BMA25X_ERROR;	
	}

	rawData[0] = (short)(((unsigned short)databuf[1]<<8)|(databuf[0]));
	rawData[1] = (short)(((unsigned short)databuf[3]<<8)|(databuf[2]));
	rawData[2] = (short)(((unsigned short)databuf[5]<<8)|(databuf[4]));
	rawData[0] = rawData[0]>>4;
	rawData[1] = rawData[1]>>4;
	rawData[2] = rawData[2]>>4;

	return BMA25X_SUCCESS;
}


int bma25x_read_xyz(float *accData)
{
	int ret;
	int rawData[3];
	int tmpData[3];

	ret = bma25x_read_raw(rawData);
	if(ret == BMA25X_SUCCESS)
	{
		tmpData[g_bma25x.cvt.map[0]] = g_bma25x.cvt.sign[0]*rawData[0];
		tmpData[g_bma25x.cvt.map[1]] = g_bma25x.cvt.sign[1]*rawData[1];
		tmpData[g_bma25x.cvt.map[2]] = g_bma25x.cvt.sign[2]*rawData[2];

		accData[0] = (float)(tmpData[0]*9.807f)/(g_bma25x.lsb_1g);			//GRAVITY_EARTH_1000
		accData[1] = (float)(tmpData[1]*9.807f)/(g_bma25x.lsb_1g);
		accData[2] = (float)(tmpData[2]*9.807f)/(g_bma25x.lsb_1g);
	}

	return ret;

}

void bma25x_soft_reset(void)
{
	BMA25X_LOG("bma25x_soft_reset \n");
}

static int bma25x_initialize(void)
{
	bma25x_soft_reset();

	bma25x_writereg(0x10, 0x0C);
	bma25x_delay(10);
	// +- 8G
	bma25x_set_range(BMA25X_RANGE_8G);
	bma25x_delay(10);

	bma25x_writereg(0x16, 0x00);
	bma25x_delay(10);

	bma25x_writereg(0x17, 0x00);
	bma25x_delay(10);

	bma25x_writereg(0x11, 0x00);
	bma25x_delay(10);

	bma25x_dump_reg();

	return BMA25X_SUCCESS;
}


int bma25x_init(void)
{
	int ret = 0;
	unsigned char slave_addr[2] = {0x18, 0x19};
	unsigned char index=0;

	for(index=0; index<2; index++)
	{
		g_bma25x.slave = slave_addr[0];
		g_bma25x.chip_id = bma25x_chip_id();
		if(g_bma25x.chip_id == BMA25X_DEVICE_ID)
			break;
	}

	if(g_bma25x.chip_id == BMA25X_DEVICE_ID)
	{
		ret = bma25x_initialize();
		if(ret != BMA25X_SUCCESS)
			return BMA25X_ERROR;

		g_bma25x.layout = 3;
		g_bma25x.cvt.map[0] = qst_map[g_bma25x.layout].map[0];
		g_bma25x.cvt.map[1] = qst_map[g_bma25x.layout].map[1];
		g_bma25x.cvt.map[2] = qst_map[g_bma25x.layout].map[2];
		g_bma25x.cvt.sign[0] = qst_map[g_bma25x.layout].sign[0];
		g_bma25x.cvt.sign[1] = qst_map[g_bma25x.layout].sign[1];
		g_bma25x.cvt.sign[2] = qst_map[g_bma25x.layout].sign[2];

		return BMA25X_SUCCESS;
	}
	else
	{
		return BMA25X_ERROR;
	}
}

