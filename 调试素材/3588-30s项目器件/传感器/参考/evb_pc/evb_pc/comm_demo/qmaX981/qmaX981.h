#ifndef __QMAX981_H
#define __QMAX981_H


#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

typedef		char					qs8;
typedef		unsigned char			qu8;
typedef		short					qs16;
typedef		unsigned short			qu16;
typedef		int						qs32;
typedef		unsigned int			qu32;
typedef		long long				qs64;
typedef		unsigned long long		qu64;
typedef		float					qf32;
typedef		double					qd64;

#define QMAX981_LOG					_cprintf
#define QMAX981_ERR					_cprintf

//#define QMAX981_QST_USE_SPI

#define QMAX981_STEPCOUNTER
//#define QMAX981_STEP_INT
//#define QMAX981_SIGNIFICANT_STEP_INT
//#define QMAX981_FIFO_FUNC
//#define QMAX981_DATA_READY
#define QMAX981_ANY_MOTION
//#define QMAX981_NO_MOTION
//#define QMAX981_SIGNIFICANT_MOTION
//#define QMAX981_INT_LATCH

#define QMA7981_DEVICE_ID		    0xe7
#define QMA7981_DEVICE_ID2		    0xe9
#define QMAX981_I2C_SLAVE_ADDR		0x12	// AD0 GND 0x12, AD0 VDD 0x13
#define QMAX981_I2C_SLAVE_ADDR2		0x13	// AD0 GND 0x12, AD0 VDD 0x13
#define QMAX981_ERROR				0
#define QMAX981_SUCCESS				1

#define GRAVITY_EARTH_1000          9807	// about (9.80665f)*1000   mm/s2
#define QMAX981_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))
#define QMA6981_OFFSET 				0x00

#define QMAX981_CHIP_ID		    	0x00
#define QMAX981_XOUTL				0x01
#define QMAX981_XOUTH				0x02
#define QMAX981_YOUTL				0x03
#define QMAX981_YOUTH				0x04
#define QMAX981_ZOUTL				0x05
#define QMAX981_ZOUTH				0x06
#define QMAX981_STEP_CNT_L			0x07
#define QMAX981_INT_STAT0			0x09
#define QMAX981_INT_STAT1			0x0a
#define QMAX981_INT_STAT2			0x0b
#define QMAX981_INT_STAT3			0x0c
#define QMAX981_FIFO_STATE			0x0e
#define QMAX981_STEP_CNT_M			0x0e
#define QMAX981_REG_RANGE			0x0f
#define QMAX981_REG_BW_ODR			0x10
#define QMAX981_REG_POWER_CTL		0x11
#define QMAX981_STEP_SAMPLE_CNT		0x12
#define QMAX981_STEP_PRECISION		0x13
#define QMAX981_STEP_TIME_LOW		0x14
#define QMAX981_STEP_TIME_UP		0x15
#define QMAX981_INTPIN_CFG			0x20
#define QMAX981_INT_CFG				0x21
#define QMAX981_OS_CUST_X		    0x27
#define QMAX981_OS_CUST_Y			0x28
#define QMAX981_OS_CUST_Z			0x29
#define QMAX981_STEP_TIME_UP		0x15
/* Accelerometer Sensor Full Scale */
#define QMAX981_RANGE_2G			0x01
#define QMAX981_RANGE_4G			0x02
#define QMAX981_RANGE_8G			0x04
#define QMAX981_RANGE_16G			0x08
#define QMAX981_RANGE_32G			0x0f

typedef enum
{
	QMAX981_DISABLE = 0,
	QMAX981_ENABLE = 1
}qmaX981_enable;

typedef enum
{
	QMAX981_MODE_SLEEP,
	QMAX981_MODE_ACTIVE,
	QMAX981_MODE_MAX
}qmaX981_power_mode;

typedef enum
{
	QMAX981_FIFO_MODE_NONE,
	QMAX981_FIFO_MODE_FIFO,
	QMAX981_FIFO_MODE_STREAM,
	QMAX981_FIFO_MODE_BYPASS,
	QMAX981_FIFO_MODE_MAX
}qmaX981_fifo_mode;

typedef enum
{
	QMAX981_TAP_SINGLE = 0x80,
	QMAX981_TAP_DOUBLE = 0x20,

	QMAX981_TAP_MAX = 0xff
}qmaX981_tap;

typedef enum
{
	QMAX981_MAP_INT1,
	QMAX981_MAP_INT2,
	QMAX981_MAP_INT_NONE
}qmaX981_int_map;

typedef enum
{
	QMAX981_MCLK_500K = 0x00,
	QMAX981_MCLK_333K = 0x01,
	QMAX981_MCLK_200K = 0x02,
	QMAX981_MCLK_100K = 0x03,
	QMAX981_MCLK_50K = 0x04,
	QMAX981_MCLK_20K = 0x05,
	QMAX981_MCLK_10K = 0x06,
	QMAX981_MCLK_5K = 0x07,
	QMAX981_MCLK_RESERVED = 0xff
}qmaX981_mclk;

typedef enum
{
	QMAX981_DIV_7695 = 0x00,
	QMAX981_DIV_3855 = 0x01,
	QMAX981_DIV_1935 = 0x02,
	QMAX981_DIV_975 = 0x03,
	QMAX981_DIV_UNKNOW = 0x04,
	QMAX981_DIV_15375 = 0x05,
	QMAX981_DIV_30735 = 0x06,
	QMAX981_DIV_61455 = 0x07,
	QMAX981_DIV_RESERVED = 0xff
}qmaX981_div;


extern qs32 qmaX981_writereg(qu8 reg_add, qu8 reg_dat);
extern qs32 qmaX981_readreg(qu8 reg_add, qu8 *buf, qu16 num);
extern qu8 qmaX981_get_slave(void);
extern qs8 qmaX981_init(void);
extern qs8 qmaX981_read_xyz(float *accData);
extern qs32 qmaX981_read_raw(qs32 *rawData);
#if defined(QMAX981_STEPCOUNTER)
extern qu32 qmaX981_read_stepcounter(void);
#endif
#if defined(QMAX981_FIFO_FUNC)
extern qs32 qmaX981_read_fifo(qu8 *fifo_buf);
extern qu8* qmaX981_get_fifo(void);
#endif
extern void qmaX981_irq_hdlr(void);

#endif  /*QMAX981*/
