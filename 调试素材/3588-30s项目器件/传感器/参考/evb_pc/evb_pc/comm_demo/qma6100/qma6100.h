#ifndef __QMA6100_H
#define __QMA6100_H


#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"


//typedef		void					void; 	
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


#define QMA6100_LOG		_cprintf
#define QMA6100_ERR		_cprintf

//#define QMA6100_QST_USE_SPI

#define QMA6100_DATA_READY
//#define QMA6100_FIFO_FUNC

//#define QMA6100_ANY_MOTION
//#define QMA6100_NO_MOTION
//#define QMA6100_SIGNIFICANT_MOTION

#define QMA6100_STEPCOUNTER
//#define QMA6100_STEP_INT
//#define QMA6100_SIGNIFICANT_STEP_INT

//#define QMA6100_TAP_FUNC

//#define QMA6100_HAND_RAISE_DOWN

#define QMA6100_INT_LATCH

#define QMA6100_DEVICE_ID		    0xfa
#define QMA6100_I2C_SLAVE_ADDR		0x12	// AD0 GND 0x12, AD0 VDD 0x13
#define QMA6100_I2C_SLAVE_ADDR2		0x13	// AD0 GND 0x12, AD0 VDD 0x13
#define QMA6100_FAIL				0
#define QMA6100_SUCCESS				1

#define GRAVITY_EARTH_1000          9807	// about (9.80665f)*1000   mm/s2
#define QMA6100_ABS(X) 				((X) < 0 ? (-1 * (X)) : (X))

#define QMA6100_DELAY				0xff
/*Register Map*/
#define QMA6100_CHIP_ID		    	0x00
#define QMA6100_XOUTL				0x01
#define QMA6100_XOUTH				0x02
#define QMA6100_YOUTL				0x03
#define QMA6100_YOUTH				0x04
#define QMA6100_ZOUTL				0x05
#define QMA6100_ZOUTH				0x06
#define QMA6100_STEP_CNT_L			0x07
#define QMA6100_INT_STAT0			0x09
#define QMA6100_INT_STAT1			0x0a
#define QMA6100_INT_STAT2			0x0b
#define QMA6100_INT_STAT3			0x0c
#define QMA6100_STEP_CNT_M			0x0d
#define QMA6100_FIFO_STATE			0x0e
#define QMA6100_REG_RANGE			0x0f
#define QMA6100_REG_BW_ODR			0x10
#define QMA6100_REG_POWER_CTL		0x11
#define QMA6100_STEP_SAMPLE_CNT		0x12
#define QMA6100_STEP_PRECISION		0x13
#define QMA6100_STEP_TIME_LOW		0x14
#define QMA6100_STEP_TIME_UP		0x15
#define QMA6100_INTPIN_CFG			0x20
#define QMA6100_INT_CFG				0x21
#define QMA6100_OS_CUST_X		    0x27
#define QMA6100_OS_CUST_Y			0x28
#define QMA6100_OS_CUST_Z			0x29
/*ODR SET @lower ODR*/
#define QMA6981_ODR_7808HZ			0x04
#define QMA6981_ODR_3904HZ			0x03
#define QMA6981_ODR_1952HZ			0x02
#define QMA6981_ODR_976HZ			0x01
#define QMA6981_ODR_488HZ			0x00
#define QMA6981_ODR_244HZ			0x05
#define QMA6981_ODR_122HZ			0x06
#define QMA6981_ODR_61HZ			0x07

/* Accelerometer Sensor Full Scale */
#define QMA6100_RANGE_2G			0x01
#define QMA6100_RANGE_4G			0x02
#define QMA6100_RANGE_8G			0x04
#define QMA6100_RANGE_16G			0x08
#define QMA6100_RANGE_32G			0x0f

typedef enum
{
	QMA6100_DISABLE = 0,
	QMA6100_ENABLE = 1
}qma6100_enable;

typedef enum
{
	QMA6100_MODE_SLEEP,
	QMA6100_MODE_ACTIVE,
	QMA6100_MODE_MAX
}qma6100_power_mode;

typedef enum
{
	QMA6100_FIFO_MODE_NONE,
	QMA6100_FIFO_MODE_FIFO,
	QMA6100_FIFO_MODE_STREAM,
	QMA6100_FIFO_MODE_BYPASS,
	QMA6100_FIFO_MODE_MAX
}qma6100_fifo_mode;

typedef enum
{
	QMA6100_TAP_SINGLE = 0x80,
	QMA6100_TAP_DOUBLE = 0x20,
	QMA6100_TAP_TRIPLE = 0x10,
	QMA6100_TAP_QUARTER = 0x01,
	QMA6100_TAP_MAX = 0xff
}qma6100_tap;

typedef enum
{
	QMA6100_MAP_INT1,
	QMA6100_MAP_INT2,
	QMA6100_MAP_INT_NONE
}qma6100_int_map;

typedef enum
{
	QMA6100_MCLK_500K = 0x00,
	QMA6100_MCLK_333K = 0x01,
	QMA6100_MCLK_200K = 0x02,
	QMA6100_MCLK_100K = 0x03,
	QMA6100_MCLK_50K = 0x04,
	QMA6100_MCLK_20K = 0x05,
	QMA6100_MCLK_10K = 0x06,
	QMA6100_MCLK_5K = 0x07,
	QMA6100_MCLK_RESERVED = 0xff
}qma6100_mclk;

typedef enum
{
	QMA6100_DIV_512 = 0x00,
	QMA6100_DIV_256 = 0x01,
	QMA6100_DIV_128 = 0x02,
	QMA6100_DIV_64 = 0x03,
	QMA6100_DIV_32 = 0x04,
	QMA6100_DIV_1024 = 0x05,
	QMA6100_DIV_2048 = 0x06,
	QMA6100_DIV_4096 = 0x07,
	QMA6100_DIV_RESERVED = 0xff
}qma6100_div;

typedef enum
{
	QMA6100_LPF_0 = (0x00<<5),
	QMA6100_LPF_2 = (0x01<<5),
	QMA6100_LPF_4 = (0x02<<5),
	QMA6100_LPF_16 = (0x03<<5),
	QMA6100_LPF_RESERVED = 0xff
}qma6100_lpf;


extern qs32 qma6100_writereg(qu8 reg_add,qu8 reg_dat);
extern qs32 qma6100_readreg(qu8 reg_add,qu8 *buf,qu16 num);
extern qu8 qma6100_get_slave(void);
extern qu8 qma6100_chip_id(void);
extern qs32 qma6100_init(void);
extern qs32 qma6100_set_range(unsigned char range);
extern qs32 qma6100_set_mode_odr(qs32 mode, qs32 mclk, qs32 div, qs32 lpf);
extern qs32 qma6100_read_raw_xyz(qs32 *data);
extern qs32 qma6100_read_acc_xyz(float *accData);
#if defined(QMA6100_DATA_READY)
extern void qma6100_drdy_config(qs32 int_map, qs32 enable);
#endif
#if defined(QMA6100_FIFO_FUNC)
extern void qma6100_fifo_config(qs32 mode, qs32 int_map, qs32 enable);
extern qs32 qma6100_read_fifo(qu8 *fifo_buf);
extern void qma6100_exe_fifo(qu8 *fifo_buf);
extern qu8* qma6100_get_fifo(void);
#endif

#if defined(QMA6100_STEPCOUNTER)
extern qu32 qma6100_read_stepcounter(void);
extern void qma6100_stepcounter_config(qs32 enable);
#if defined(QMA6100_STEP_INT)
extern void qma6100_step_int_config(qs32 int_map, qs32 enable);
#endif
#if defined(QMA6100_SIGNIFICANT_STEP_INT)
extern void qma6100_sigstep_int_config(qs32 int_map, qs32 enable);
#endif
#endif

#if defined(QMA6100_ANY_MOTION)
extern void qma6100_anymotion_config(qs32 int_map, qs32 enable);
#endif
#if defined(QMA6100_NO_MOTION)
extern void qma6100_nomotion_config(qs32 int_map, qs32 enable);
#endif

extern void qma6100_irq_hdlr(void);


#endif  /*QMX6981*/
