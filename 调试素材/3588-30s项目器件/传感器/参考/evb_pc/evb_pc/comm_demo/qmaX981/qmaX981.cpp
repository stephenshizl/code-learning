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
#include "qmaX981.h"

typedef struct
{
    qs16 sign[3];
    qu16 map[3];
}qst_convert;

typedef struct
{
	qu8					slave;
	qu8					chip_id;
	qs32				lsb_1g;
	qu8					layout;
	qst_convert			cvt;
	qu8					fifo_mode;
	qs32				fifo_len;
} qmaX981_data;

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

static qmaX981_data g_qmaX981;
static char qmaX981_use_spi = 0;

void qmaX981_delay(qu32 delay)
{
	Sleep(delay);
}

qs32 qmaX981_writereg(qu8 reg_add, qu8 reg_dat)
{
	qs32 ret = 0;
	qu32 retry = 0;	

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = 0;
		}
		else
		{
			ret = i2c_write_reg(g_qmaX981.slave, reg_add, reg_dat);
		}
	}

	if(ret)
		return QMAX981_SUCCESS;
	else
		return QMAX981_ERROR;
}

qs32 qmaX981_readreg(qu8 reg_add, qu8 *buf, qu16 num)
{
	qs32 ret = 0;
	qu32 retry = 0;

	while((!ret) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = 0;
			//ret = qmaX981_spi_read_reg(reg_add, buf, num);
		}
		else
		{
			ret = i2c_read_reg(g_qmaX981.slave, reg_add, buf, (qu16)num);
		}
	}

	if(ret)
		return QMAX981_SUCCESS;
	else
		return QMAX981_ERROR;
}

qu8 qmaX981_get_slave(void)
{
	return g_qmaX981.slave;
}

qu8 qmaX981_chip_id(void)
{
	qu8 chip_id = 0x00;
	qu32 retry = 0;

	qmaX981_writereg(QMAX981_REG_POWER_CTL, 0x80);
	while(!((chip_id >= QMA7981_DEVICE_ID)&&(chip_id <= QMA7981_DEVICE_ID2)))
	{
		qmaX981_readreg(QMAX981_CHIP_ID, &chip_id, 1);
		QMAX981_LOG("qmaX981_chip_id id=0x%x \n", chip_id);
		if(++retry >= 1)
			break;
	}

	return chip_id;
}

qs32 qmaX981_set_range(qs32 range)
{
	qs32 ret = 0;

	if(range == QMAX981_RANGE_4G)
		g_qmaX981.lsb_1g = 2048;
	else if(range == QMAX981_RANGE_8G)
		g_qmaX981.lsb_1g = 1024;
	else if(range == QMAX981_RANGE_16G)
		g_qmaX981.lsb_1g = 512;
	else if(range == QMAX981_RANGE_32G)
		g_qmaX981.lsb_1g = 256;
	else
		g_qmaX981.lsb_1g = 4096;
	
	ret = qmaX981_writereg(QMAX981_REG_RANGE, range);

	return ret;
}

qs32 qmaX981_set_mode_odr(qs32 mode, qs32 mclk, qs32 div)
{
	qs32 ret = 0;
	qu8 mclk_reg = (qu8)mclk;
	qu8 odr_reg = (qu8)div;

	if(mode >= QMAX981_MODE_ACTIVE)
	{
		ret = qmaX981_writereg(QMAX981_REG_BW_ODR, odr_reg);
		ret = qmaX981_writereg(QMAX981_REG_POWER_CTL, 0xc0|mclk_reg);
		ret = qmaX981_writereg(0x5f, 0x80);
		ret = qmaX981_writereg(0x5f, 0x00);
	}
	else
	{
		ret = qmaX981_writereg(QMAX981_REG_POWER_CTL, 0x00);
	}

	return ret;
}


void qmaX981_dump_reg(void)
{
	qu8 reg_data = 0;
	qs32 i=0;
	qu8 reg_map[]=
	{
		0x09,0x0a,0x0b,0x0c,0x0e,
		0x0f,0x10,0x11,0x12,0x13,0x14,0x15,
		0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,
		0x1D,0x1E,0x1F,0x20
	};
	QMAX981_LOG("qmaX981_dump_reg\n");
	for(i=0; i< sizeof(reg_map)/sizeof(reg_map[0]); i++)
	{
		qmaX981_readreg(reg_map[i],&reg_data,1);
		QMAX981_LOG("0x%x = 0x%x	", reg_map[i], reg_data);
	}
	QMAX981_LOG("\n");
}

#if defined(QMAX981_DATA_READY)
void qmaX981_drdy_config(qs32 int_map, qs32 enable)
{
	qu8 reg_17 = 0;
	qu8 reg_1a = 0;
	qu8 reg_1c = 0;

	QMAX981_LOG("qmaX981_drdy_config %d\n", enable);
	qmaX981_readreg(0x17, &reg_17, 1);
	qmaX981_readreg(0x1a, &reg_1a, 1);
	qmaX981_readreg(0x1c, &reg_1c, 1);

	if(enable)
	{
		reg_17 |= 0x10;
		reg_1a |= 0x10;
		reg_1c |= 0x10;
		qmaX981_writereg(0x17, reg_17);
		if(int_map == QMAX981_MAP_INT1)
			qmaX981_writereg(0x1a, reg_1a);
		else if(int_map == QMAX981_MAP_INT2)
			qmaX981_writereg(0x1c, reg_1c);
	}
	else
	{
		reg_17 &= (~0x10);
		reg_1a &= (~0x10);
		reg_1c &= (~0x10);
		qmaX981_writereg(0x17, reg_17);
		qmaX981_writereg(0x1a, reg_1a);
		qmaX981_writereg(0x1c, reg_1c);
	}

}
#endif

#if defined(QMAX981_FIFO_FUNC)
static qu8 qmaX981_fifo_reg[64*6];

void qmaX981_fifo_config(qs32 mode, qs32 int_map, qs32 enable)
{
	qu8	reg_17, reg_1a, reg_1c=0;

	QMAX981_LOG("qmaX981_fifo_config mode:%d enable:%d\n", mode, enable);
	qmaX981_readreg(0x17, &reg_17, 1);
	qmaX981_readreg(0x1a, &reg_1a, 1);
	qmaX981_readreg(0x1c, &reg_1c, 1);

	if(enable)
	{
		g_qmaX981.fifo_mode = mode;
		if(g_qmaX981.fifo_mode == QMAX981_FIFO_MODE_FIFO)
		{
			qmaX981_writereg(0x31, 0x40);
			qmaX981_writereg(0x3E, 0x47);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qmaX981_writereg(0x17, reg_17|0x20);
			if(int_map == QMAX981_MAP_INT1)
				qmaX981_writereg(0x1a, reg_1a|0x20);
			else if(int_map == QMAX981_MAP_INT2)
				qmaX981_writereg(0x1c, reg_1c|0x20);
		}
		else if(g_qmaX981.fifo_mode == QMAX981_FIFO_MODE_STREAM)
		{	
			qmaX981_writereg(0x31, 0x3f);	// 0x3f
			qmaX981_writereg(0x3E, 0x87);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qmaX981_writereg(0x17, reg_17|0x40);
			if(int_map == QMAX981_MAP_INT1)
				qmaX981_writereg(0x1a, reg_1a|0x40);
			else if(int_map == QMAX981_MAP_INT2)
				qmaX981_writereg(0x1c, reg_1c|0x40);
		}
		else if(g_qmaX981.fifo_mode == QMAX981_FIFO_MODE_BYPASS)
		{
			qmaX981_writereg(0x3E, 0x07);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qmaX981_writereg(0x17, reg_17|0x20);
			if(int_map == QMAX981_MAP_INT1)
				qmaX981_writereg(0x1a, reg_1a|0x20);
			else if(int_map == QMAX981_MAP_INT2)
				qmaX981_writereg(0x1c, reg_1c|0x20);
		}
	}
	else
	{
		g_qmaX981.fifo_mode = QMAX981_FIFO_MODE_NONE;
		reg_17 &= (~0x60);
		reg_1a &= (~0x60);
		reg_1c &= (~0x60);
		qmaX981_writereg(0x17, reg_17);
		qmaX981_writereg(0x1a, reg_1a);
		qmaX981_writereg(0x1c, reg_1c);
	}
}

qs32 qmaX981_read_fifo(qu8 *fifo_buf)
{
	qs32 ret = 0;
	qu8 databuf[2];

#if defined(QMAX981_INT_LATCH)
	ret = qmaX981_readreg(QMAX981_INT_STAT3, databuf, 1);
#endif
	ret = qmaX981_readreg(QMAX981_FIFO_STATE, databuf, 1);
	if(ret != QMAX981_SUCCESS)
	{
		QMAX981_ERR("qmaX981_read_fifo state error\n");
		return ret;
	}
	g_qmaX981.fifo_len = databuf[0]&0x7f;
	if(g_qmaX981.fifo_len > 64)
	{
		QMAX981_ERR("qmaX981_read_fifo depth(%d) error\n",g_qmaX981.fifo_len);
		return QMAX981_ERROR;
	}

#if 0
	qmaX981_readreg(0x3f, fifo_buf, g_qmaX981.fifo_len*6);
#else
	for(int icount=0; icount<g_qmaX981.fifo_len; icount++)
	{
		qmaX981_readreg(0x3f, &fifo_buf[icount*6], 6);
	}
#endif
	if(g_qmaX981.fifo_mode == QMAX981_FIFO_MODE_FIFO)
	{
		ret = qmaX981_writereg(0x3e, 0x47);
	}
	else if(g_qmaX981.fifo_mode == QMAX981_FIFO_MODE_STREAM)
	{
		ret = qmaX981_writereg(0x3e, 0x87);
	}
	else if(g_qmaX981.fifo_mode == QMAX981_FIFO_MODE_BYPASS)
	{
		ret = qmaX981_writereg(0x3e, 0x07);
	}
// log fifo
	return ret;
}

void qmaX981_exe_fifo(qu8 *fifo_buf)
{
	qs32 icount;	
	qs16 raw_data[3];
	//float acc_data[3];

	QMAX981_ERR("fifo_depth=%d\n", g_qmaX981.fifo_len);
// log fifo
	for(icount=0; icount<g_qmaX981.fifo_len; icount++)
	{
		raw_data[0]  = (qs16)(((qs16)(fifo_buf[1+icount*6]<<8)) |(fifo_buf[0+icount*6]));
		raw_data[1]  = (qs16)(((qs16)(fifo_buf[3+icount*6]<<8)) |(fifo_buf[2+icount*6]));
		raw_data[2]  = (qs16)(((qs16)(fifo_buf[5+icount*6]<<8)) |(fifo_buf[4+icount*6]));
		raw_data[0]  = raw_data[0]>>2;
		raw_data[1]  = raw_data[1]>>2;
		raw_data[2]  = raw_data[2]>>2;
		QMAX981_LOG("%d:%d	%d	%d	",icount,raw_data[0],raw_data[1],raw_data[2]);
		if((icount%4==0))
		{
			QMAX981_LOG("\r\n");
		}
		//acc_data[0] = (raw_data[0]*9.807f)/(g_qmaX981.lsb_1g);			//GRAVITY_EARTH_1000
		//acc_data[1] = (raw_data[1]*9.807f)/(g_qmaX981.lsb_1g);
		//acc_data[2] = (raw_data[2]*9.807f)/(g_qmaX981.lsb_1g);
	}	
	QMAX981_LOG("\r\n");
// log fifo
}

qu8* qmaX981_get_fifo(void)
{
	return qmaX981_fifo_reg;
}
#endif

#if defined(QMAX981_STEPCOUNTER)
qu32 qmaX981_read_stepcounter(void)
{
	qu8 data[3];
	qu8 ret;
	qu32 step_num;
	qs32 step_dif;
	static qu32 step_last = 0;

	ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);	
	if(ret != QMAX981_SUCCESS)
	{
		step_num = step_last;
		return QMAX981_SUCCESS;
	}
	ret = qmaX981_readreg(QMAX981_STEP_CNT_M, &data[2], 1);	
	if(ret != QMAX981_SUCCESS)
	{
		step_num = step_last;
		return QMAX981_SUCCESS;
	}
		
	step_num = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);

#if 1//defined(QMAX981_CHECK_ABNORMAL_DATA)
	step_dif = (qs32)(step_num-step_last);
	if(QMAX981_ABS(step_dif) > 100)
	{
		qu32 step_num_temp[3];

		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);	
		ret = qmaX981_readreg(QMAX981_STEP_CNT_M, &data[2], 1);
		step_num_temp[0] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qmaX981_delay(2);
		
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);	
		ret = qmaX981_readreg(QMAX981_STEP_CNT_M, &data[2], 1);
		step_num_temp[1] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qmaX981_delay(2);
		
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);	
		ret = qmaX981_readreg(QMAX981_STEP_CNT_M, &data[2], 1);
		step_num_temp[2] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qmaX981_delay(2);
		if((step_num_temp[0]==step_num_temp[1])&&(step_num_temp[1]==step_num_temp[2]))
		{
			QMAX981_LOG("qmaX981 check data, confirm!\n");
			step_num = step_num_temp[0];
		}
		else
		{	
			QMAX981_LOG("qmaX981 check data, abnormal!\n");
			return step_last;
		}
	}
#endif
	step_last = step_num;

	return step_num;
}

void qmaX981_stepcounter_config(qs32 enable)
{	
	qs32 odr = 141;
	qu8 reg_14 = 0x00;
	qu8 reg_15 = 0x00;

	if(enable)
	{
		qmaX981_writereg(0x12, 0x94);
	}
	else
	{
		qmaX981_writereg(0x12, 0x00);
	}
	qmaX981_writereg(0x13, 0x7f);
	// odr 116.7 Hz, 8.568ms
	reg_14 = (310*odr)/(1000);			// 310 ms
	reg_15 = ((2000/8)*odr)/(1000);		// 2000 ms
	QMAX981_LOG("step time config 0x14=0x%x	0x15=0x%x\n", reg_14, reg_15);

	qmaX981_writereg(0x14, reg_14);
	qmaX981_writereg(0x15, reg_15);

	//qmaX981_writereg(0x1f, 0x09);	// 0 step
	//qmaX981_writereg(0x1f, 0x29);	// 4 step
	//qmaX981_writereg(0x1f, 0x49);	// 8 step
	//qmaX981_writereg(0x1f, 0x69);	// 12 step
	//qmaX981_writereg(0x1f, 0x89);	// 16 step
	qmaX981_writereg(0x1f, 0xa9);	// 24 step
	//qmaX981_writereg(0x1f, 0xc9);	// 32 step
	//qmaX981_writereg(0x1f, 0xe9);	// 40 step
}

#if defined(QMAX981_STEP_INT)
void qmaX981_step_int_config(qs32 int_map, qs32 enable)
{
	qu8	reg_16=0;
	qu8	reg_19=0;
	qu8	reg_1b=0;

	qmaX981_readreg(0x16, &reg_16, 1);
	qmaX981_readreg(0x19, &reg_19, 1);
	qmaX981_readreg(0x1b, &reg_1b, 1);
	if(enable)
	{
		reg_16 |= 0x08;
		reg_19 |= 0x08;
		reg_1b |= 0x08;
		qmaX981_writereg(0x16, reg_16);
		if(int_map == QMAX981_MAP_INT1)
			qmaX981_writereg(0x19, reg_19);
		else if(int_map == QMAX981_MAP_INT2)
			qmaX981_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= (~0x08);
		reg_19 &= (~0x08);
		reg_1b &= (~0x08);

		qmaX981_writereg(0x16, reg_16);
		qmaX981_writereg(0x19, reg_19);
		qmaX981_writereg(0x1b, reg_1b);
	}
}
#endif

#if defined(QMAX981_SIGNIFICANT_STEP_INT)
void qmaX981_sigstep_int_config(qs32 int_map, qs32 enable)
{
	qu8	reg_16=0;
	qu8	reg_19=0;
	qu8	reg_1b=0;

	qmaX981_readreg(0x16, &reg_16, 1);
	qmaX981_readreg(0x19, &reg_19, 1);
	qmaX981_readreg(0x1b, &reg_1b, 1);
	
	qmaX981_writereg(0x1d, 0x1a);
	if(enable)
	{
		reg_16 |= 0x40;
		reg_19 |= 0x40;
		reg_1b |= 0x40;
		qmaX981_writereg(0x16, reg_16);
		if(int_map == QMAX981_MAP_INT1)
			qmaX981_writereg(0x19, reg_19);
		else if(int_map == QMAX981_MAP_INT2)
			qmaX981_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= (~0x40);
		reg_19 &= (~0x40);
		reg_1b &= (~0x40);

		qmaX981_writereg(0x16, reg_16);
		qmaX981_writereg(0x19, reg_19);
		qmaX981_writereg(0x1b, reg_1b);
	}
}
#endif

#endif


#if defined(QMAX981_ANY_MOTION)
void qmaX981_anymotion_config(qs32 int_map, qs32 enable)
{
	qu8 reg_0x18 = 0;
	qu8 reg_0x1a = 0;
	qu8 reg_0x1c = 0;
	qu8 reg_0x2c = 0;
#if defined(QMAX981_SIGNIFICANT_MOTION)
	qu8 reg_0x19 = 0;
	qu8 reg_0x1b = 0;
#endif

	QMAX981_LOG("qmaX981_anymotion_config %d\n", enable);
	qmaX981_readreg(0x18, &reg_0x18, 1);
	qmaX981_readreg(0x1a, &reg_0x1a, 1);
	qmaX981_readreg(0x1c, &reg_0x1c, 1);
	qmaX981_readreg(0x2c, &reg_0x2c, 1);
	reg_0x2c |= 0x00;

	qmaX981_writereg(0x2c, reg_0x2c);
	qmaX981_writereg(0x2e, 0x10);		// 0.488*16*32 = 250mg
	if(enable)
	{
		reg_0x18 |= 0x07;
		reg_0x1a |= 0x01;
		reg_0x1c |= 0x01;

		qmaX981_writereg(0x18, reg_0x18);	// enable any motion
		if(int_map == QMAX981_MAP_INT1)
			qmaX981_writereg(0x1a, reg_0x1a);
		else if(int_map == QMAX981_MAP_INT2)
			qmaX981_writereg(0x1c, reg_0x1c);
	}
	else
	{
		reg_0x18 &= (~0x07);
		reg_0x1a &= (~0x01);
		reg_0x1c &= (~0x01);
		
		qmaX981_writereg(0x18, reg_0x18);
		qmaX981_writereg(0x1a, reg_0x1a);
		qmaX981_writereg(0x1c, reg_0x1c);
	}
	
#if defined(QMAX981_SIGNIFICANT_MOTION)
	qmaX981_readreg(0x19, &reg_0x19, 1);
	qmaX981_readreg(0x1b, &reg_0x1b, 1);
	
	qmaX981_writereg(0x2f, 0x01);		// bit0: select significant motion
	if(enable)
	{
		reg_0x19 |= 0x01;
		reg_0x1b |= 0x01;
		if(int_map == QMAX981_MAP_INT1)
			qmaX981_writereg(0x19, reg_0x19);
		else if(int_map == QMAX981_MAP_INT2)
			qmaX981_writereg(0x1b, reg_0x1b);
	}
	else
	{
		reg_0x19 &= (~0x01);
		reg_0x1b &= (~0x01);
		qmaX981_writereg(0x19, reg_0x19);
		qmaX981_writereg(0x1b, reg_0x1b);
	}
#endif	
}
#endif


#if defined(QMAX981_NO_MOTION)
void qmaX981_nomotion_config(qs32 int_map, qs32 enable)
{
	qu8 reg_0x18 = 0;
	qu8 reg_0x1a = 0;
	qu8 reg_0x1c = 0;
	qu8 reg_0x2c = 0;

	QMAX981_LOG("qmaX981_nomotion_config %d\n", enable);

	qmaX981_readreg(0x18, &reg_0x18, 1);
	qmaX981_readreg(0x1a, &reg_0x1a, 1);
	qmaX981_readreg(0x1c, &reg_0x1c, 1);
	qmaX981_readreg(0x2c, &reg_0x2c, 1);
	reg_0x2c |= 0x24;		// 10s
	//reg_0x2c |= 0xc0; 		// 100s

	qmaX981_writereg(0x2c, reg_0x2c);
	qmaX981_writereg(0x2d, 0x14);
	if(enable)
	{
		reg_0x18 |= 0xe0;
		reg_0x1a |= 0x80;
		reg_0x1c |= 0x80;		
		qmaX981_writereg(0x18, reg_0x18);
		if(int_map == QMAX981_MAP_INT1)
			qmaX981_writereg(0x1a, reg_0x1a);
		else if(int_map == QMAX981_MAP_INT2)
			qmaX981_writereg(0x1c, reg_0x1c);
	}
	else
	{
		reg_0x18 &= (~0xe0);
		reg_0x1a &= (~0x80);
		reg_0x1c &= (~0x80);
		
		qmaX981_writereg(0x18, reg_0x18);
		qmaX981_writereg(0x1a, reg_0x1a);
		qmaX981_writereg(0x1c, reg_0x1c);
	}

}
#endif

#if defined(QMAX981_HAND_RAISE_DOWN)
void qmaX981_hand_raise_down(qs32 int_map, qs32 enable)
{
	qu8 reg_16,reg_19,reg_1b;
	
	qmaX981_readreg(0x16, &reg_16, 1);
	qmaX981_readreg(0x19, &reg_19, 1);
	qmaX981_readreg(0x1b, &reg_1b, 1);

	if(enable)
	{
		reg_16 |= (0x02|0x04);
		reg_19 |= (0x02|0x04);
		reg_1b |= (0x02|0x04);
		qmaX981_writereg(0x16, reg_16);
		if(int_map == QMA6100_MAP_INT1)
			qmaX981_writereg(0x19, reg_19);
		else if(int_map == QMA6100_MAP_INT1)
			qmaX981_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= ~((0x02|0x04));
		reg_19 &= ~((0x02|0x04));
		reg_1b &= ~((0x02|0x04));
		qmaX981_writereg(0x16, reg_16);
		qmaX981_writereg(0x19, reg_19);
		qmaX981_writereg(0x1b, reg_1b);
	}
}
#endif


void qmaX981_irq_hdlr(void)
{
	qs32 ret = 0;
	qu8 databuf[4];
	
#if defined(QMAX981_DATA_READY)
	{
		qs32 raw[3];
		qmaX981_read_raw(raw);
		QMAX981_LOG("drdy	%d	%d	%d\n", raw[0], raw[1], raw[2]);
		return;
	}
#endif
	ret = qmaX981_readreg(0x09, databuf, 4);
	if(ret == QMAX981_SUCCESS)
	{
		QMAX981_LOG(" qma7981_irq_hdlr [0x%x 0x%x 0x%x 0x%x] @ ", databuf[0],databuf[1],databuf[2],databuf[3]);
	}
	else
	{
		return;
	}
#if defined(QMAX981_ANY_MOTION)
	if(databuf[0]&0x07)
	{
		QMAX981_LOG("any motion!\n");
	}
#endif
#if defined(QMAX981_NO_MOTION)
	if(databuf[0]&0x80)
	{
		QMAX981_LOG("no motion!\n");
	}
#endif
#if defined(QMAX981_STEP_INT)
	if(databuf[1]&0x08)
	{
		QMAX981_LOG("step int!\n");
	}
#endif
#if defined(QMAX981_SIGNIFICANT_STEP_INT)
	if(databuf[1]&0x40)
	{
		QMAX981_LOG("significant step int!\n");
	}
#endif	
#if defined(QMAX981_FIFO_FUNC)
	if(databuf[2]&0x20)
	{
		QMAX981_LOG("FIFO FULL\n");
		//qmaX981_read_fifo(qmaX981_fifo_reg);
	}
	if(databuf[2]&0x40)
	{
		QMAX981_LOG("FIFO WMK\n");
		//qmaX981_read_fifo(qmaX981_fifo_reg);
	}
#endif
}



qs32 qmaX981_read_raw(qs32 *rawData)
{
	qu8 databuf[6] = {0};	
	qs32 ret;

	ret = qmaX981_readreg(QMAX981_XOUTL, databuf, 6);
	if(ret == QMAX981_ERROR)
	{
		QMAX981_ERR("read xyz error!!!");
		return QMAX981_ERROR;	
	}

	rawData[0] = (qs16)(((qu16)databuf[1]<<8)|(databuf[0]));
	rawData[1] = (qs16)(((qu16)databuf[3]<<8)|(databuf[2]));
	rawData[2] = (qs16)(((qu16)databuf[5]<<8)|(databuf[4]));
	rawData[0] = rawData[0]>>2;
	rawData[1] = rawData[1]>>2;
	rawData[2] = rawData[2]>>2;

	return QMAX981_SUCCESS;
}


qs8 qmaX981_read_xyz(float *accData)
{
	qs32 ret;
	qs32 rawData[3];
	qs32 tmpData[3];

	ret = qmaX981_read_raw(rawData);
	if(ret == QMAX981_SUCCESS)
	{
		tmpData[g_qmaX981.cvt.map[0]] = g_qmaX981.cvt.sign[0]*rawData[0];
		tmpData[g_qmaX981.cvt.map[1]] = g_qmaX981.cvt.sign[1]*rawData[1];
		tmpData[g_qmaX981.cvt.map[2]] = g_qmaX981.cvt.sign[2]*rawData[2];

		accData[0] = (float)(tmpData[0]*9.807f)/(g_qmaX981.lsb_1g);			//GRAVITY_EARTH_1000
		accData[1] = (float)(tmpData[1]*9.807f)/(g_qmaX981.lsb_1g);
		accData[2] = (float)(tmpData[2]*9.807f)/(g_qmaX981.lsb_1g);
	}

	return ret;

}

void qmaX981_soft_reset(void)
{
	QMAX981_LOG("qmaX981_soft_reset");
	qmaX981_writereg(0x36, 0xb6);
	qmaX981_delay(5);
	qmaX981_writereg(0x36, 0x00);
	qmaX981_delay(2);
}

static qs32 qmaX981_initialize(void)
{
#if defined(QMAX981_INT_LATCH)
	qu8 reg_data[4];
#endif

	qmaX981_soft_reset();
	qmaX981_set_mode_odr(QMAX981_MODE_ACTIVE, QMAX981_MCLK_500K, QMAX981_DIV_3855);
	qmaX981_set_range(QMAX981_RANGE_4G);
	#if 0	// MCLK to int1
	qmaX981_writereg(0x49, 0x01);
	qmaX981_writereg(0x56, 0x10);
	#endif	// MCLK to int1

#if defined(QMAX981_DATA_READY)
	qmaX981_drdy_config(QMAX981_MAP_INT1, QMAX981_ENABLE);
#endif

#if defined(QMAX981_FIFO_FUNC)
	qmaX981_fifo_config(QMAX981_FIFO_MODE_STREAM, QMAX981_MAP_INT_NONE, QMAX981_ENABLE);
#endif

#if defined(QMAX981_STEPCOUNTER)
	qmaX981_stepcounter_config(QMAX981_ENABLE);
	#if defined(QMAX981_STEP_INT)
	qmaX981_step_int_config(QMAX981_MAP_INT1, QMAX981_ENABLE);
	#endif	
	#if defined(QMAX981_SIGNIFICANT_STEP_INT)
	qmaX981_sigstep_int_config(QMAX981_MAP_INT1, QMAX981_ENABLE);
	#endif
#endif

#if defined(QMAX981_ANY_MOTION)
	qmaX981_anymotion_config(QMAX981_MAP_INT1, QMAX981_ENABLE);
#endif
#if defined(QMAX981_NO_MOTION)
	qmaX981_nomotion_config(QMAX981_MAP_INT1, QMAX981_ENABLE);
#endif

#if defined(QMAX981_HAND_RAISE_DOWN)
	qmaX981_hand_raise_down(QMAX981_MAP_INT1, QMAX981_ENABLE);
#endif

#if defined(QMAX981_INT_LATCH)
	qmaX981_writereg(0x21, 0x03);	// default 0x1c, step latch mode
	qmaX981_readreg(0x09, reg_data, 4);
	QMAX981_LOG("read status=[0x%x 0x%x 0x%x 0x%x] \n", reg_data[0],reg_data[1],reg_data[2],reg_data[3]);
#endif

	qmaX981_dump_reg();

	return QMAX981_SUCCESS;
}


qs8 qmaX981_init(void)
{
	qs32 ret = 0;
	qs8 slave_addr[2] = {QMAX981_I2C_SLAVE_ADDR, QMAX981_I2C_SLAVE_ADDR2};
	qs8 index=0;

	for(index=0; index<2; index++)
	{
		g_qmaX981.slave = slave_addr[0];
		g_qmaX981.chip_id = qmaX981_chip_id();
		if((g_qmaX981.chip_id >= QMA7981_DEVICE_ID) && (g_qmaX981.chip_id <= QMA7981_DEVICE_ID2))
			break;
	}
	if((g_qmaX981.chip_id >= QMA7981_DEVICE_ID)&&(g_qmaX981.chip_id <= QMA7981_DEVICE_ID2))
	{
#if defined(QMAX981_FIX_IIC)
		qmaX981_writereg(0x20, 0x45);
		g_qmaX981.slave = QMAX981_I2C_SLAVE_ADDR<<1;
#endif
		ret = qmaX981_initialize();
		if(ret != QMAX981_SUCCESS)
			return QMAX981_ERROR;

		g_qmaX981.layout = 0;
		g_qmaX981.cvt.map[0] = qst_map[g_qmaX981.layout].map[0];
		g_qmaX981.cvt.map[1] = qst_map[g_qmaX981.layout].map[1];
		g_qmaX981.cvt.map[2] = qst_map[g_qmaX981.layout].map[2];
		g_qmaX981.cvt.sign[0] = qst_map[g_qmaX981.layout].sign[0];
		g_qmaX981.cvt.sign[1] = qst_map[g_qmaX981.layout].sign[1];
		g_qmaX981.cvt.sign[2] = qst_map[g_qmaX981.layout].sign[2];

		return QMAX981_SUCCESS;
	}
	else
	{
		return QMAX981_ERROR;
	}
}

