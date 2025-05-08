/**
  ******************************************************************************
  * @file    qma6100.c
  * @author  Yangzhiqiang@qst
  * @version V1.0
  * @date    2020-5-27
  * @brief    qma6100
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */ 
#include "stdafx.h"
#include "qma6100.h"

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
	qma6100_fifo_mode	fifo_mode;
	qs32				fifo_len;	
	qs32				raw[3];
	float				acc[3];
}qma6100_data;

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

static qma6100_data g_qma6100;

void qma6100_delay(qu32 delay)
{
	Sleep(delay);
}

qs32 qma6100_writereg(qu8 reg_add,qu8 reg_dat)
{
	qs32 ret = QMA6100_FAIL;
	qu32 retry = 0;

	while((ret==QMA6100_FAIL) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = spi_write_reg(reg_add&0x7f, reg_dat);
		}
		else
		{
			ret = i2c_write_reg(g_qma6100.slave, reg_add, reg_dat);
		}
	}
	if (ret)
		return QMA6100_SUCCESS;
	else
		return QMA6100_FAIL;
}

qs32 qma6100_readreg(qu8 reg_add,qu8 *buf,qu16 num)
{
	qs32 ret = QMA6100_FAIL;
	qu32 retry = 0;

	while((ret==QMA6100_FAIL) && (retry++ < 5))
	{
		if(get_device_protocol() == USB_SPI)
		{
			ret = spi_read_reg(reg_add|0x80, buf, num);
		}
		else
		{
			ret = i2c_read_reg(g_qma6100.slave, reg_add, buf, (qu16)num);
		}
	}
	if (ret)
		return QMA6100_SUCCESS;
	else
		return QMA6100_FAIL;
}

qu8 qma6100_get_slave(void)
{
	return g_qma6100.slave;
}

qu8 qma6100_chip_id(void)
{
	qu8 chip_id = 0x00;
	qs32 ret = QMA6100_FAIL;
	qs32 retry = 0;

	while(chip_id != QMA6100_DEVICE_ID)
	{
		ret = qma6100_readreg(QMA6100_CHIP_ID, &chip_id, 1);
		QMA6100_LOG("qma6100_chip_id =0x%x ret=%d\n", chip_id, ret);
		if(++retry >= 1)
			break;
	}

	return chip_id;
}

qs32 qma6100_set_range(qu8 range)
{
	qs32 ret = 0;

	if(range == QMA6100_RANGE_4G)
		g_qma6100.lsb_1g = 2048;
	else if(range == QMA6100_RANGE_8G)
		g_qma6100.lsb_1g = 1024;
	else if(range == QMA6100_RANGE_16G)
		g_qma6100.lsb_1g = 512;
	else if(range == QMA6100_RANGE_32G)
		g_qma6100.lsb_1g = 256;
	else
		g_qma6100.lsb_1g = 4096;

	
	ret = qma6100_writereg(QMA6100_REG_RANGE, range);

	return ret;
}

//BW<4:0> ODR ODR (MCLK = 500kHz)
//				BW			ODR
//xx000 MCLK/512		977 Hz		488.5
//xx001 MCLK/256		1953 Hz		976.5
//xx010 MCLK/128		3906 Hz		1953
//xx011 MCLK/64		7813 Hz		3906.5
//xx100 MCLK/32		15625 Hz		7812.5
//xx101 MCLK/1024		488 Hz		244
//xx110 MCLK/2048		244 Hz		122
//xx111 MCLK/4096		122 Hz		61
//Others MCLK/512		977 Hz		488.5S
qs32 qma6100_set_mode_odr(qs32 mode, qs32 mclk, qs32 div, qs32 lpf)
{
	qs32 ret = 0;
	qu8 mclk_reg = (qu8)mclk;
	qu8 odr_reg = (qu8)div;
	qu8 lpf_reg = (qu8)lpf;//|0x80;
#if 0
	qs32 mclk_f, div_f;

	if(mclk == QMA6100_MCLK_500K)
		mclk_f = 500*1000;
	else if(mclk == QMA6100_MCLK_333K)
		mclk_f = 333*1000;
	else if(mclk == QMA6100_MCLK_200K)
		mclk_f = 200*1000;
	else if(mclk == QMA6100_MCLK_100K)
		mclk_f = 100*1000;
	else if(mclk == QMA6100_MCLK_50K)
		mclk_f = 50*1000;
	else if(mclk == QMA6100_MCLK_20K)
		mclk_f = 20*1000;
	else if(mclk == QMA6100_MCLK_10K)
		mclk_f = 10*1000;
	else if(mclk == QMA6100_MCLK_5K)
		mclk_f = 5*1000;
	else 
		mclk_f = 500*1000;

	if(div == QMA6100_DIV_32)
		div_f = 32;
	else if(div == QMA6100_DIV_64)
		div_f = 64;
	else if(div == QMA6100_DIV_128)
		div_f = 128;
	else if(div == QMA6100_DIV_256)
		div_f = 256;
	else if(div == QMA6100_DIV_512)
		div_f = 512;
	else if(div == QMA6100_DIV_1024)
		div_f = 1024;
	else if(div == QMA6100_DIV_2048)
		div_f = 2048;
	else if(div == QMA6100_DIV_4096)
		div_f = 4096;
	else
		div_f = 2048;

	QMA6100_LOG("qma6100_set_odr MCLK:%dKHz DIV:%d BW:%d ODR:%d lpf:%d\n", 
				mclk_f/1000,div_f,mclk_f/div_f,mclk_f/div_f/2, lpf);
#endif
	if(mode >= QMA6100_MODE_ACTIVE)
	{
		ret = qma6100_writereg(QMA6100_REG_POWER_CTL, 0x80|mclk_reg);
		ret = qma6100_writereg(QMA6100_REG_BW_ODR, lpf_reg|odr_reg);
	}
	else
	{
		ret = qma6100_writereg(QMA6100_REG_POWER_CTL, 0x00);
	}

	return ret;
}


void qma6100_dump_reg(void)
{
	qu8 reg_data = 0;
	qs32 i=0;
	qu8 reg_map[]=
	{
		0x09,0x0a,0x0b,0x0c,0x0e,
		0x0f,0x10,0x11,0x12,0x13,0x14,0x15,
		0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,
		0x1D,0x1E,0x1F,0x20,0x30,0x32,
		0x31,0x3e
		//0x20,0x21,0x24,0x25,0x26,
		//0x30,0x31,0x37,0x3e,0x4a,0x50,0x56
	};
	QMA6100_LOG("qma6100_dump_reg\n");
	for(i=0; i< sizeof(reg_map)/sizeof(reg_map[0]); i++)
	{
		qma6100_readreg(reg_map[i],&reg_data,1);
		QMA6100_LOG("0x%x = 0x%x	", reg_map[i], reg_data);
	}
	QMA6100_LOG("\n");
}

#if defined(QMA6100_DATA_READY)
void qma6100_drdy_config(qs32 int_map, qs32 enable)
{
	qu8 reg_17 = 0;
	qu8 reg_1a = 0;
	qu8 reg_1c = 0;

	QMA6100_LOG("qma6100_drdy_config %d\n", enable);
	qma6100_readreg(0x17, &reg_17, 1);
	qma6100_readreg(0x1a, &reg_1a, 1);
	qma6100_readreg(0x1c, &reg_1c, 1);

	if(enable)
	{
		reg_17 |= 0x10;
		reg_1a |= 0x10;
		reg_1c |= 0x10;
		qma6100_writereg(0x17, reg_17);
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x1a, reg_1a);
		else if(int_map == QMA6100_MAP_INT2)
			qma6100_writereg(0x1c, reg_1c);
	}
	else
	{
		reg_17 &= (~0x10);
		reg_1a &= (~0x10);
		reg_1c &= (~0x10);
		qma6100_writereg(0x17, reg_17);
		qma6100_writereg(0x1a, reg_1a);
		qma6100_writereg(0x1c, reg_1c);
	}

}
#endif

#if defined(QMA6100_FIFO_FUNC)
static qu8 qma6100_fifo_reg[64*6];

void qma6100_fifo_config(qs32 mode, qs32 int_map, qs32 enable)
{
	qu8	reg_17, reg_1a, reg_1c=0;

	QMA6100_LOG("qma6100_fifo_config mode:%d enable:%d\n", mode, enable);
	qma6100_readreg(0x17, &reg_17, 1);
	qma6100_readreg(0x1a, &reg_1a, 1);
	qma6100_readreg(0x1c, &reg_1c, 1);

	if(enable)
	{
		g_qma6100.fifo_mode = mode;
		if(g_qma6100.fifo_mode == QMA6100_FIFO_MODE_FIFO)
		{
			qma6100_writereg(0x31, 0x40);
			qma6100_writereg(0x3E, 0x47);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6100_writereg(0x17, reg_17|0x20);
			if(int_map == QMA6100_MAP_INT1)
				qma6100_writereg(0x1a, reg_1a|0x20);
			else if(int_map == QMA6100_MAP_INT2)
				qma6100_writereg(0x1c, reg_1c|0x20);
		}
		else if(g_qma6100.fifo_mode == QMA6100_FIFO_MODE_STREAM)
		{	
			qma6100_writereg(0x31, 0x3f);	// 0x3f
			qma6100_writereg(0x3E, 0x87);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6100_writereg(0x17, reg_17|0x40);
			if(int_map == QMA6100_MAP_INT1)
				qma6100_writereg(0x1a, reg_1a|0x40);
			else if(int_map == QMA6100_MAP_INT2)
				qma6100_writereg(0x1c, reg_1c|0x40);
		}
		else if(g_qma6100.fifo_mode == QMA6100_FIFO_MODE_BYPASS)
		{
			qma6100_writereg(0x3E, 0x07);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
			qma6100_writereg(0x17, reg_17|0x20);
			if(int_map == QMA6100_MAP_INT1)
				qma6100_writereg(0x1a, reg_1a|0x20);
			else if(int_map == QMA6100_MAP_INT2)
				qma6100_writereg(0x1c, reg_1c|0x20);
		}
	}
	else
	{
		g_qma6100.fifo_mode = QMA6100_FIFO_MODE_NONE;
		reg_17 &= (~0x60);
		reg_1a &= (~0x60);
		reg_1c &= (~0x60);
		qma6100_writereg(0x17, reg_17);
		qma6100_writereg(0x1a, reg_1a);
		qma6100_writereg(0x1c, reg_1c);
	}
}

qs32 qma6100_read_fifo(qu8 *fifo_buf)
{
	qs32 ret = 0;
	qu8 databuf[2];

#if defined(QMA6100_INT_LATCH)
	ret = qma6100_readreg(QMA6100_INT_STAT2, databuf, 1);
#endif
	ret = qma6100_readreg(QMA6100_FIFO_STATE, databuf, 1);
	if(ret != QMA6100_SUCCESS)
	{
		QMA6100_ERR("qma6100_read_fifo state error\n");
		return ret;
	}
	g_qma6100.fifo_len = databuf[0]&0x7f;
	if(g_qma6100.fifo_len > 64)
	{
		QMA6100_ERR("qma6100_read_fifo depth(%d) error\n",g_qma6100.fifo_len);
		return QMA6100_FAIL;
	}

	if(fifo_buf)
	{
#if 0
		qma6100_readreg(0x3f, fifo_buf, g_qma6100.fifo_len*6);
#else
		for(int icount=0; icount<g_qma6100.fifo_len; icount++)
		{
			qma6100_readreg(0x3f, &fifo_buf[icount*6], 6);
		}
#endif
	}
	if(g_qma6100.fifo_mode == QMA6100_FIFO_MODE_FIFO)
	{
		ret = qma6100_writereg(0x3e, 0x47);
	}
	else if(g_qma6100.fifo_mode == QMA6100_FIFO_MODE_STREAM)
	{
		ret = qma6100_writereg(0x3e, 0x87);
	}
	else if(g_qma6100.fifo_mode == QMA6100_FIFO_MODE_BYPASS)
	{
		ret = qma6100_writereg(0x3e, 0x07);
	}
// log fifo
	return ret;
}

void qma6100_exe_fifo(qu8 *fifo_buf)
{
	qs32 icount;	
	qs16 raw_data[3];
	//float acc_data[3];

	QMA6100_ERR("fifo_depth=%d\n", g_qma6100.fifo_len);
// log fifo
	for(icount=0; icount<g_qma6100.fifo_len; icount++)
	{
		raw_data[0]  = (qs16)(((qs16)(fifo_buf[1+icount*6]<<8)) |(fifo_buf[0+icount*6]));
		raw_data[1]  = (qs16)(((qs16)(fifo_buf[3+icount*6]<<8)) |(fifo_buf[2+icount*6]));
		raw_data[2]  = (qs16)(((qs16)(fifo_buf[5+icount*6]<<8)) |(fifo_buf[4+icount*6]));
		raw_data[0]  = raw_data[0]>>2;
		raw_data[1]  = raw_data[1]>>2;
		raw_data[2]  = raw_data[2]>>2;
		QMA6100_LOG("%d:%d	%d	%d	",icount,raw_data[0],raw_data[1],raw_data[2]);
		if((icount%4==0))
		{
			QMA6100_LOG("\r\n");
		}
		//acc_data[0] = (raw_data[0]*9.807f)/(g_qma6100.lsb_1g);			//GRAVITY_EARTH_1000
		//acc_data[1] = (raw_data[1]*9.807f)/(g_qma6100.lsb_1g);
		//acc_data[2] = (raw_data[2]*9.807f)/(g_qma6100.lsb_1g);
	}	
	QMA6100_LOG("\r\n");
// log fifo
}

qu8* qma6100_get_fifo(void)
{
	return qma6100_fifo_reg;
}

#endif

#if defined(QMA6100_STEPCOUNTER)
qu32 qma6100_read_stepcounter(void)
{
	qu8 data[3];
	qu8 ret;
	qu32 step_num;
	qs32 step_dif;
	static qu32 step_last = 0;

	ret = qma6100_readreg(QMA6100_STEP_CNT_L, data, 2);	
	if(ret != QMA6100_SUCCESS)
	{
		step_num = step_last;
		return QMA6100_SUCCESS;
	}
	ret = qma6100_readreg(QMA6100_STEP_CNT_M, &data[2], 1);	
	if(ret != QMA6100_SUCCESS)
	{
		step_num = step_last;
		return QMA6100_SUCCESS;
	}
		
	step_num = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);

#if 1//defined(QMA6100_CHECK_ABNORMAL_DATA)
	step_dif = (qs32)(step_num-step_last);
	if(QMA6100_ABS(step_dif) > 100)
	{
		qu32 step_num_temp[3];

		ret = qma6100_readreg(QMA6100_STEP_CNT_L, data, 2);	
		ret = qma6100_readreg(QMA6100_STEP_CNT_M, &data[2], 1);
		step_num_temp[0] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qma6100_delay(2);
		
		ret = qma6100_readreg(QMA6100_STEP_CNT_L, data, 2);	
		ret = qma6100_readreg(QMA6100_STEP_CNT_M, &data[2], 1);
		step_num_temp[1] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qma6100_delay(2);
		
		ret = qma6100_readreg(QMA6100_STEP_CNT_L, data, 2);	
		ret = qma6100_readreg(QMA6100_STEP_CNT_M, &data[2], 1);
		step_num_temp[2] = (qu32)(((qu32)data[2]<<16)|((qu32)data[1]<<8)|data[0]);
		qma6100_delay(2);
		if((step_num_temp[0]==step_num_temp[1])&&(step_num_temp[1]==step_num_temp[2]))
		{
			QMA6100_LOG("qma6100 check data, confirm!\n");
			step_num = step_num_temp[0];
		}
		else
		{	
			QMA6100_LOG("qma6100 check data, abnormal!\n");
			return step_last;
		}
	}
#endif
	step_last = step_num;

	return step_num;
}

void qma6100_stepcounter_config(qs32 enable)
{	
	qs32 odr = 75;
	qu8 reg_14 = 0x00;
	qu8 reg_15 = 0x00;

	if(enable)
	{
		qma6100_writereg(0x12, 0x94);
	}
	else
	{
		qma6100_writereg(0x12, 0x00);
	}
	qma6100_writereg(0x13, 0x7f);
	// odr 116.7 Hz, 8.568ms
	reg_14 = (310*odr)/(1000);			// 310 ms
	reg_15 = ((2000/8)*odr)/(1000);		// 2000 ms
	QMA6100_LOG("step time config 0x14=0x%x	0x15=0x%x\n", reg_14, reg_15);

	qma6100_writereg(0x14, reg_14);
	qma6100_writereg(0x15, reg_15);

	//qma6100_writereg(0x1f, 0x09);	// 0 step
	//qma6100_writereg(0x1f, 0x29);	// 4 step
	//qma6100_writereg(0x1f, 0x49);	// 8 step
	qma6100_writereg(0x1f, 0x69);	// 12 step
	//qma6100_writereg(0x1f, 0x89);	// 16 step
	//qma6100_writereg(0x1f, 0xa9);	// 24 step
	//qma6100_writereg(0x1f, 0xc9);	// 32 step
	//qma6100_writereg(0x1f, 0xe9);	// 40 step
}

#if defined(QMA6100_STEP_INT)
void qma6100_step_int_config(qs32 int_map, qs32 enable)
{
	qu8	reg_16=0;
	qu8	reg_19=0;
	qu8	reg_1b=0;

	qma6100_readreg(0x16, &reg_16, 1);
	qma6100_readreg(0x19, &reg_19, 1);
	qma6100_readreg(0x1b, &reg_1b, 1);
	if(enable)
	{
		reg_16 |= 0x08;
		reg_19 |= 0x08;
		reg_1b |= 0x08;
		qma6100_writereg(0x16, reg_16);
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x19, reg_19);
		else if(int_map == QMA6100_MAP_INT2)
			qma6100_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= (~0x08);
		reg_19 &= (~0x08);
		reg_1b &= (~0x08);

		qma6100_writereg(0x16, reg_16);
		qma6100_writereg(0x19, reg_19);
		qma6100_writereg(0x1b, reg_1b);
	}
}
#endif

#if defined(QMA6100_SIGNIFICANT_STEP_INT)
void qma6100_sigstep_int_config(qs32 int_map, qs32 enable)
{
	qu8	reg_16=0;
	qu8	reg_19=0;
	qu8	reg_1b=0;

	qma6100_readreg(0x16, &reg_16, 1);
	qma6100_readreg(0x19, &reg_19, 1);
	qma6100_readreg(0x1b, &reg_1b, 1);
	
	qma6100_writereg(0x1d, 0x1a);
	if(enable)
	{
		reg_16 |= 0x40;
		reg_19 |= 0x40;
		reg_1b |= 0x40;
		qma6100_writereg(0x16, reg_16);
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x19, reg_19);
		else if(int_map == QMA6100_MAP_INT2)
			qma6100_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= (~0x40);
		reg_19 &= (~0x40);
		reg_1b &= (~0x40);

		qma6100_writereg(0x16, reg_16);
		qma6100_writereg(0x19, reg_19);
		qma6100_writereg(0x1b, reg_1b);
	}
}
#endif


#endif

#if defined(QMA6100_ANY_MOTION)
void qma6100_anymotion_config(qs32 int_map, qs32 enable)
{
	qu8 reg_0x18 = 0;
	qu8 reg_0x1a = 0;
	qu8 reg_0x1c = 0;
	qu8 reg_0x2c = 0;
#if defined(QMA6100_SIGNIFICANT_MOTION)
	qu8 reg_0x19 = 0;
	qu8 reg_0x1b = 0;
#endif

	QMA6100_LOG("qma6100_anymotion_config %d\n", enable);
	qma6100_readreg(0x18, &reg_0x18, 1);
	qma6100_readreg(0x1a, &reg_0x1a, 1);
	qma6100_readreg(0x1c, &reg_0x1c, 1);
	qma6100_readreg(0x2c, &reg_0x2c, 1);
	reg_0x2c |= 0x00;

	qma6100_writereg(0x2c, reg_0x2c);
	qma6100_writereg(0x2e, 0x10);		// 0.488*16*32 = 250mg
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF
	qma6100_writereg(0x30, 0x80|0x40|0x3f);	// default 0x3f
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF
	if(enable)
	{
		reg_0x18 |= 0x07;
		reg_0x1a |= 0x01;
		reg_0x1c |= 0x01;

		qma6100_writereg(0x18, reg_0x18);	// enable any motion
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x1a, reg_0x1a);
		else if(int_map == QMA6100_MAP_INT2)
			qma6100_writereg(0x1c, reg_0x1c);
	}
	else
	{
		reg_0x18 &= (~0x07);
		reg_0x1a &= (~0x01);
		reg_0x1c &= (~0x01);
		
		qma6100_writereg(0x18, reg_0x18);
		qma6100_writereg(0x1a, reg_0x1a);
		qma6100_writereg(0x1c, reg_0x1c);
	}
	
#if defined(QMA6100_SIGNIFICANT_MOTION)
	qma6100_readreg(0x19, &reg_0x19, 1);
	qma6100_readreg(0x1b, &reg_0x1b, 1);
	
	qma6100_writereg(0x2f, 0x01);		// bit0: selecat significant motion
	if(enable)
	{
		reg_0x19 |= 0x01;
		reg_0x1b |= 0x01;
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x19, reg_0x19);
		else if(int_map == QMA6100_MAP_INT2)
			qma6100_writereg(0x1b, reg_0x1b);
	}
	else
	{
		reg_0x19 &= (~0x01);
		reg_0x1b &= (~0x01);
		qma6100_writereg(0x19, reg_0x19);
		qma6100_writereg(0x1b, reg_0x1b);
	}
#endif	
}
#endif

#if defined(QMA6100_NO_MOTION)
void qma6100_nomotion_config(qs32 int_map, qs32 enable)
{
	qu8 reg_0x18 = 0;
	qu8 reg_0x1a = 0;
	qu8 reg_0x1c = 0;
	qu8 reg_0x2c = 0;

	QMA6100_LOG("qma6100_nomotion_config %d\n", enable);

	qma6100_readreg(0x18, &reg_0x18, 1);
	qma6100_readreg(0x1a, &reg_0x1a, 1);
	qma6100_readreg(0x1c, &reg_0x1c, 1);
	qma6100_readreg(0x2c, &reg_0x2c, 1);
	reg_0x2c |= 0x24;		// 10s
	//reg_0x2c |= 0xc0; 		// 100s

	qma6100_writereg(0x2c, reg_0x2c);
	qma6100_writereg(0x2d, 0x14);
	if(enable)
	{
		reg_0x18 |= 0xe0;
		reg_0x1a |= 0x80;
		reg_0x1c |= 0x80;		
		qma6100_writereg(0x18, reg_0x18);
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x1a, reg_0x1a);
		else if(int_map == QMA6100_MAP_INT2)
			qma6100_writereg(0x1c, reg_0x1c);
	}
	else
	{
		reg_0x18 &= (~0xe0);
		reg_0x1a &= (~0x80);
		reg_0x1c &= (~0x80);
		
		qma6100_writereg(0x18, reg_0x18);
		qma6100_writereg(0x1a, reg_0x1a);
		qma6100_writereg(0x1c, reg_0x1c);
	}

}
#endif

#if defined(QMA6100_TAP_FUNC)
void qma6100_tap_config(qs32 tap_type, qs32 int_map, qs32 enable)
{
	qu8 reg_16,reg_19,reg_1a,reg_1b,reg_1c;
	qu8 tap_reg_en = (qu8)(tap_type);
	qu8 tap_reg_int = (qu8)(tap_type&0xfe);

	qma6100_readreg(0x16, &reg_16, 1);
	qma6100_readreg(0x19, &reg_19, 1);
	qma6100_readreg(0x1a, &reg_1a, 1);
	qma6100_readreg(0x1b, &reg_1b, 1);
	qma6100_readreg(0x1c, &reg_1c, 1);

	qma6100_writereg(0x1e, 0x05);		// TAP_QUIET_TH 31.25*8 = 250mg 
	qma6100_writereg(0x2a, 0x85);		// tap config1
	qma6100_writereg(0x2b, (0xc0+6));		// tap config2
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF
	qma6100_writereg(0x30, 0x80|0x40|0x3f);	// default 0x3f
	// add by yang, tep counter, raise wake, and tap detector,any motion by pass LPF

	if(enable)
	{
		reg_16 |= tap_reg_en;
		qma6100_writereg(0x16, reg_16);

		if(int_map == QMA6100_MAP_INT1)
		{
			if(tap_type & QMA6100_TAP_QUARTER)
			{
				reg_1a |= 0x02;
				qma6100_writereg(0x1a, reg_1a);
			}
			if(tap_type & (QMA6100_TAP_SINGLE|QMA6100_TAP_DOUBLE|QMA6100_TAP_TRIPLE))
			{
				reg_19 |= (tap_reg_int);
				qma6100_writereg(0x19, reg_19);
			}
		}
		else if(int_map == QMA6100_MAP_INT2)
		{
			if(tap_type & QMA6100_TAP_QUARTER)
			{
				reg_1c |= 0x02;
				qma6100_writereg(0x1c, reg_1c);
			}
			if(tap_type & (QMA6100_TAP_SINGLE|QMA6100_TAP_DOUBLE|QMA6100_TAP_TRIPLE))
			{
				reg_1b |= (tap_reg_int);
				qma6100_writereg(0x1b, reg_1b);
			}
		}
	}
	else
	{	
		reg_16 &= (~tap_reg_en);
		qma6100_writereg(0x16, reg_16);

		if(int_map == QMA6100_MAP_INT1)
		{
			if(tap_type & QMA6100_TAP_QUARTER)
			{
				reg_1a &= (~0x02);
				qma6100_writereg(0x1a, reg_1a);
			}
			if(tap_type & (QMA6100_TAP_SINGLE|QMA6100_TAP_DOUBLE|QMA6100_TAP_TRIPLE))
			{
				reg_19 &= (~tap_reg_int);
				qma6100_writereg(0x19, reg_19);
			}
		}
		else if(int_map == QMA6100_MAP_INT2)
		{
			if(tap_type & QMA6100_TAP_QUARTER)
			{
				reg_1c &= (~0x02);
				qma6100_writereg(0x1c, reg_1c);
			}
			if(tap_type & (QMA6100_TAP_SINGLE|QMA6100_TAP_DOUBLE|QMA6100_TAP_TRIPLE))
			{
				reg_1b &= (~tap_reg_int);
				qma6100_writereg(0x1b, reg_1b);
			}
		}
	}
}
#endif

#if defined(QMA6100_HAND_RAISE_DOWN)
void qma6100_hand_raise_down(qs32 int_map, qs32 enable)
{
	qu8 reg_16,reg_19,reg_1b;
	
	qma6100_readreg(0x16, &reg_16, 1);
	qma6100_readreg(0x19, &reg_19, 1);
	qma6100_readreg(0x1b, &reg_1b, 1);

	// 0x24: RAISE_WAKE_TIMEOUT_TH<7:0>: Raise_wake_timeout_th[11:0] * ODR period = timeout count
	// 0x25: RAISE_WAKE_PERIOD<7:0>: Raise_wake_period[10:0] * ODR period = wake count
	// 0x26:
	// RAISE_MODE: 0:raise wake function, 1:ear-in function
	// RAISE_WAKE_PERIOD<10:8>: Raise_wake_period[10:0] * ODR period = wake count
	// RAISE_WAKE_TIMEOUT_TH<11:8>: Raise_wake_timeout_th[11:0] * ODR period = timeout count

	if(enable)
	{
		reg_16 |= (0x02|0x04);
		reg_19 |= (0x02|0x04);
		reg_1b |= (0x02|0x04);
		qma6100_writereg(0x16, reg_16);
		if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x19, reg_19);
		else if(int_map == QMA6100_MAP_INT1)
			qma6100_writereg(0x1b, reg_1b);
	}
	else
	{
		reg_16 &= ~((0x02|0x04));
		reg_19 &= ~((0x02|0x04));
		reg_1b &= ~((0x02|0x04));
		qma6100_writereg(0x16, reg_16);
		qma6100_writereg(0x19, reg_19);
		qma6100_writereg(0x1b, reg_1b);
	}
}
#endif

void qma6100_irq_hdlr(void)
{
	qu8 ret = QMA6100_FAIL;
	qu8 databuf[4];
	qs32 retry = 0;

	while((ret==QMA6100_FAIL)&&(retry++<10))
	{
		ret = qma6100_readreg(QMA6100_INT_STAT0, databuf, 4);
		if(ret == QMA6100_SUCCESS)
		{
			break;
		}
	}
	if(ret == QMA6100_FAIL)
	{
		QMA6100_LOG("qma6100_irq_hdlr read status fail!\n");
		return;
	}
	else
	{
		QMA6100_LOG("irq [0x%x 0x%x 0x%x 0x%x] @ ", databuf[0],databuf[1],databuf[2],databuf[3]);
	}

#if defined(QMA6100_DATA_READY)
	if(databuf[2]&0x10)
	{
		qma6100_read_raw_xyz(g_qma6100.raw);
		QMA6100_LOG("drdy	%d	%d	%d\n",g_qma6100.raw[0],g_qma6100.raw[1],g_qma6100.raw[2]);
		//qma6100_read_acc_xyz(g_qma6100.acc);
		//QMA6100_LOG("drdy	%f	%f	%f\n",g_qma6100.acc[0],g_qma6100.acc[1],g_qma6100.acc[2]);
	}
#endif
#if defined(QMA6100_FIFO_FUNC)
	if((databuf[2]&0x40)&&(g_qma6100.fifo_mode==QMA6100_FIFO_MODE_STREAM))
	{
		QMA6100_LOG("FIFO WMK\n");
		qma6100_read_fifo(qma6100_fifo_reg);
		qma6100_exe_fifo(qma6100_fifo_reg);
	}
	else if((databuf[2]&0x20)) 
	{
		QMA6100_LOG("FIFO FULL\n");
		qma6100_read_fifo(qma6100_fifo_reg);
		qma6100_exe_fifo(qma6100_fifo_reg);
	}
#endif
#if defined(QMA6100_ANY_MOTION)
	if(databuf[0]&0x07)
	{
		QMA6100_LOG("any motion!\n");
	}
#if defined(QMA6100_SIGNIFICANT_MOTION)
	if(databuf[1]&0x01)
	{
		QMA6100_LOG("significant motion!\n");
	}
#endif
#endif
#if defined(QMA6100_NO_MOTION)
	if(databuf[0]&0x80)
	{
		QMA6100_LOG("no motion!\n");
	}
#endif
#if defined(QMA6100_STEP_INT)
	if(databuf[1]&0x08)
	{
		QMA6100_LOG("step int!\n");
	}
#endif
#if defined(QMA6100_SIGNIFICANT_STEP_INT)
	if(databuf[1]&0x40)
	{
		QMA6100_LOG("significant step int!\n");
	}
#endif
#if defined(QMA6100_TAP_FUNC)
	if(databuf[1]&0x80)
	{
		QMA6100_LOG("SINGLE tap int!\n");
	}
	if(databuf[1]&0x20)
	{
		QMA6100_LOG("DOUBLE tap int!\n");
	}
	if(databuf[1]&0x10)
	{
		QMA6100_LOG("TRIPLE tap int!\n");
	}	
	if(databuf[2]&0x01)
	{
		QMA6100_LOG("QUARTER tap int!\n");
	}
#endif
#if defined(QMA6100_HAND_RAISE_DOWN)
	if(databuf[1]&0x02)
	{
		QMA6100_LOG("hand raise!\n");
	}
	if(databuf[1]&0x04)
	{
		QMA6100_LOG("hand down!\n");
	}
#endif
}


qu8 qma6100_drdy_read_raw_xyz(qs32 *data)
{
	qu8 databuf[6] = {0};	
	qs16 raw_data[3];	
	qu8 drdy = 0;
	qs32 ret;

	while(!drdy)
	{
		ret = qma6100_readreg(QMA6100_XOUTL, databuf, 6);
		if(ret == QMA6100_FAIL){
			QMA6100_ERR("read xyz error!!!");
			return 0;	
		}
		drdy = (databuf[0]&0x01) + (databuf[2]&0x01) + (databuf[4]&0x01);
		//QMA6100_LOG("drdy 0x%d \n",drdy);
	}

	raw_data[0] = (qs16)(((qu16)(databuf[1]<<8))|(databuf[0]));
	raw_data[1] = (qs16)(((qu16)(databuf[3]<<8))|(databuf[2]));
	raw_data[2] = (qs16)(((qu16)(databuf[5]<<8))|(databuf[4]));
	data[0] = raw_data[0]>>2;
	data[1] = raw_data[1]>>2;
	data[2] = raw_data[2]>>2;

	return QMA6100_SUCCESS;
}

qs32 qma6100_read_raw_xyz(qs32 *data)
{
	qu8 databuf[6] = {0}; 	
	qs16 raw_data[3];	
	qs32 ret;

	ret = qma6100_readreg(QMA6100_XOUTL, databuf, 6);
	if(ret == QMA6100_FAIL){
		QMA6100_ERR("read xyz error!!!");
		return QMA6100_FAIL;	
	}

	raw_data[0] = (qs16)(((qu16)(databuf[1]<<8))|(databuf[0]));
	raw_data[1] = (qs16)(((qu16)(databuf[3]<<8))|(databuf[2]));
	raw_data[2] = (qs16)(((qu16)(databuf[5]<<8))|(databuf[4]));
	data[0] = raw_data[0]>>2;
	data[1] = raw_data[1]>>2;
	data[2] = raw_data[2]>>2;

	return QMA6100_SUCCESS;
}

qs32 qma6100_read_acc_xyz(float *accData)
{
	qs32 ret;
	qs32 rawData[3];
	qs32 tempData[3];

	ret = qma6100_read_raw_xyz(rawData);
	if(ret == QMA6100_SUCCESS)
	{
		tempData[g_qma6100.cvt.map[0]] = g_qma6100.cvt.sign[0]*rawData[0];
		tempData[g_qma6100.cvt.map[1]] = g_qma6100.cvt.sign[1]*rawData[1];
		tempData[g_qma6100.cvt.map[2]] = g_qma6100.cvt.sign[2]*rawData[2];

		accData[0] = (float)(tempData[0]*9.807f)/(g_qma6100.lsb_1g);			//GRAVITY_EARTH_1000
		accData[1] = (float)(tempData[1]*9.807f)/(g_qma6100.lsb_1g);
		accData[2] = (float)(tempData[2]*9.807f)/(g_qma6100.lsb_1g);
	}

	return ret;

}


qs32 qma6100_soft_reset(void)
{
	qu8 reg_0x11 = 0;
	qu8 reg_0x33 = 0;
	qu32 retry = 0;

	QMA6100_LOG("qma6100_soft_reset");
	qma6100_writereg(0x36, 0xb6);
	qma6100_delay(5);
	qma6100_writereg(0x36, 0x00);
	qma6100_delay(2);

	retry = 0;
	qma6100_writereg(0x11, 0x80);
	while(reg_0x11 != 0x80)
	{
		qma6100_delay(5);
		qma6100_readreg(0x11,&reg_0x11,1);
		QMA6100_LOG("confirm read 0x11 = 0x%x\n",reg_0x11);
		if(retry++ > 100)
			break;
	}
	// load otp
	qma6100_writereg(0x33, 0x08);
	qma6100_delay(5);
	retry = 0;
	while(reg_0x33 != 0x05)
	{
		qma6100_readreg(0x33,&reg_0x33,1);
		QMA6100_LOG("confirm read 0x33 = 0x%x\n",reg_0x33);
		qma6100_delay(5);
		if(retry++ > 100)
			break;
	}

	return QMA6100_SUCCESS;
}


static qs32 qma6100_initialize(void)
{
#if defined(QMA6100_INT_LATCH)
	qu8 reg_data[4];
#endif

	qma6100_soft_reset();
#if 0
	qma6100_set_mode_odr(QMA6100_MODE_ACTIVE, QMA6100_MCLK_500K, QMA6100_DIV_2048, QMA6100_LPF_16);
	qma6100_set_range(QMA6100_RANGE_8G);
	qma6100_writereg(0x56, 0x01);
#else
	// config by peili
	qma6100_set_mode_odr(QMA6100_MODE_ACTIVE, QMA6100_MCLK_50K, QMA6100_DIV_512, QMA6100_LPF_0);
	qma6100_set_range(QMA6100_RANGE_8G);
	qma6100_writereg(0x4a, 0x20);
	qma6100_writereg(0x50, 0x49);
	qma6100_writereg(0x56, 0x01);

	#if 0	// MCLK to int1
	qma6100_writereg(0x49, 0x01);
	qma6100_writereg(0x56, 0x10);
	#endif	// MCLK to int1
	// config by peili
#endif

#if defined(QMA6100_DATA_READY)
	qma6100_drdy_config(QMA6100_MAP_INT1, QMA6100_ENABLE);
#endif

#if defined(QMA6100_FIFO_FUNC)
	qma6100_fifo_config(QMA6100_FIFO_MODE_STREAM, QMA6100_MAP_INT_NONE, QMA6100_ENABLE);
#endif

#if defined(QMA6100_STEPCOUNTER)
	qma6100_stepcounter_config(QMA6100_ENABLE);
	#if defined(QMA6100_STEP_INT)
	qma6100_step_int_config(QMA6100_MAP_INT1, QMA6100_ENABLE);
	#endif	
	#if defined(QMA6100_SIGNIFICANT_STEP_INT)
	qma6100_sigstep_int_config(QMA6100_MAP_INT1, QMA6100_ENABLE);
	#endif
#endif

#if defined(QMA6100_ANY_MOTION)
	qma6100_anymotion_config(QMA6100_MAP_INT1, QMA6100_ENABLE);
#endif
#if defined(QMA6100_NO_MOTION)
	qma6100_nomotion_config(QMA6100_MAP_INT1, QMA6100_ENABLE);
#endif

#if defined(QMA6100_TAP_FUNC)
	qma6100_tap_config(QMA6100_TAP_SINGLE, QMA6100_MAP_INT1, QMA6100_ENABLE);
#endif

#if defined(QMA6100_HAND_RAISE_DOWN)
	qma6100_hand_raise_down(QMA6100_MAP_INT1, QMA6100_ENABLE);
#endif

#if defined(QMA6100_INT_LATCH)
	qma6100_writereg(0x21, 0x03);	// default 0x1c, step latch mode
	qma6100_readreg(0x09, reg_data, 4);
	QMA6100_LOG("read status=[0x%x 0x%x 0x%x 0x%x] \n", reg_data[0],reg_data[1],reg_data[2],reg_data[3]);
#endif

	qma6100_dump_reg();

	return QMA6100_SUCCESS;
}


qs32 qma6100_init(void)
{
	qs32 ret = 0;
	qu8 slave_addr[2] = {QMA6100_I2C_SLAVE_ADDR, QMA6100_I2C_SLAVE_ADDR2};
	qu8 index=0;

	for(index=0; index<2; index++)
	{
		g_qma6100.chip_id = 0;
		g_qma6100.slave = slave_addr[index];
		g_qma6100.chip_id = qma6100_chip_id();
		if(g_qma6100.chip_id == QMA6100_DEVICE_ID)
		{
			QMA6100_LOG("qma6100 find \n");
			break;
		}
	}
#if defined(QMA6100_FIX_IIC)
	qma6100_writereg(0x20, 0x45);
	g_qma6100.slave = QMA6100_I2C_SLAVE_ADDR;
#endif

	if(g_qma6100.chip_id == QMA6100_DEVICE_ID)
		ret = qma6100_initialize();
	else
		ret = QMA6100_FAIL;
	
	//g_qma6100.layout = 3;
	g_qma6100.layout = 0;
	g_qma6100.cvt.map[0] = qst_map[g_qma6100.layout].map[0];
	g_qma6100.cvt.map[1] = qst_map[g_qma6100.layout].map[1];
	g_qma6100.cvt.map[2] = qst_map[g_qma6100.layout].map[2];
	g_qma6100.cvt.sign[0] = qst_map[g_qma6100.layout].sign[0];
	g_qma6100.cvt.sign[1] = qst_map[g_qma6100.layout].sign[1];
	g_qma6100.cvt.sign[2] = qst_map[g_qma6100.layout].sign[2];

	return ret;
}

