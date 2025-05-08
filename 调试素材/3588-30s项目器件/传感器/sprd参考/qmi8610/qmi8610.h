/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef QMI8610_H
#define QMI8610_H

#include<linux/ioctl.h> 
#include<linux/module.h>
#include<linux/err.h>
#include<linux/errno.h>
#include<linux/delay.h>
#include<linux/fs.h>
#include<linux/i2c.h>

#include<linux/input.h>
#include<linux/input-polldev.h>
#include<linux/miscdevice.h>
#include<linux/uaccess.h>
#include<linux/slab.h>

#include<linux/workqueue.h>
#include<linux/irq.h>
#include<linux/gpio.h>
#include<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include<linux/earlysuspend.h>
#endif
#include<linux/of_device.h>
#include<linux/of_address.h>
#include<linux/of_gpio.h>

#include<linux/mutex.h>

//#define QMI8610_MTK_KK

#define QMIIMU_DEV_NAME					"qmi8610"
#define QMIIMU_ACC_INUT_NAME			"accelerometer"
#define QMIIMU_GYRO_INUT_NAME			"gyroscope"
#define QMIIMU_TAG                  	"[Qmiimu] "
#if 0
#define QMIIMU_FUN(f)               	printk(QMIIMU_TAG"%s\n", __FUNCTION__)
#define QMIIMU_ERR(fmt, args...)    	printk(QMIIMU_TAG"%s %d\n" fmt, __FUNCTION__, __LINE__, ##args)
#define QMIIMU_LOG(fmt, args...)    	printk(QMIIMU_TAG"%s %d\n" fmt, __FUNCTION__, __LINE__, ##args)
#else
#define QMIIMU_FUN(f)               	do {} while (0)
#define QMIIMU_ERR(fmt, args...)    	do {} while (0)
#define QMIIMU_LOG(fmt, args...)    	do {} while (0)
#endif


#ifndef I2C_MASK_FLAG
#define I2C_MASK_FLAG   (0x00ff)
#define I2C_DMA_FLAG    (0x2000)
#define I2C_ENEXT_FLAG  (0x0200)
#endif

#define QMIIMU_I2C_SLAVE_ADDR			0x6a
#define QMIIMU_I2C_SLAVE_ADDR2			0x6b

#define QST_PI							(3.14159265358979323846f)
#define EARTH_G							(9.807f)

#define QMIIMU_CTRL7_DISABLE_ALL		(0x0)
#define QMIIMU_CTRL7_ACC_ENABLE			(0x1)
#define QMIIMU_CTRL7_GYR_ENABLE			(0x2)
#define QMIIMU_CTRL7_MAG_ENABLE			(0x4)
#define QMIIMU_CTRL7_AE_ENABLE			(0x8)
#define QMIIMU_CTRL7_ENABLE_MASK		(0xF)

#define QMIIMU_CONFIG_ACC_ENABLE		QMIIMU_CTRL7_ACC_ENABLE
#define QMIIMU_CONFIG_GYR_ENABLE		QMIIMU_CTRL7_GYR_ENABLE
#define QMIIMU_CONFIG_MAG_ENABLE		QMIIMU_CTRL7_MAG_ENABLE
#define QMIIMU_CONFIG_AE_ENABLE			QMIIMU_CTRL7_AE_ENABLE
#define QMIIMU_CONFIG_ACCGYR_ENABLE		(QMIIMU_CONFIG_ACC_ENABLE|QMIIMU_CONFIG_GYR_ENABLE)
#define QMIIMU_CONFIG_ACCGYRMAG_ENABLE	(QMIIMU_CONFIG_ACC_ENABLE|QMIIMU_CONFIG_GYR_ENABLE|QMIIMU_CONFIG_MAG_ENABLE)
#define QMIIMU_CONFIG_AEMAG_ENABLE		(QMIIMU_CONFIG_AE_ENABLE|QMIIMU_CONFIG_MAG_ENABLE)
#define QMIIMU_CONFIG_ALL_ENABLE		(QMIIMU_CONFIG_ACC_ENABLE|QMIIMU_CONFIG_GYR_ENABLE|QMIIMU_CONFIG_MAG_ENABLE|QMIIMU_CTRL7_AE_ENABLE)

#define QMIIMU_STATUS1_CMD_DONE			(0x01)
#define QMIIMU_STATUS1_WAKEUP_EVENT		(0x04)

enum QMI8610Register
{
	/*! \brief QMI device identifier register. */
	QmiRegister_WhoAmI=0, // 0
	/*! \brief QMI hardware revision register. */
	QmiRegister_Revision, // 1
	/*! \brief General and power management modes. */
	QmiRegister_Ctrl1, // 2
	/*! \brief Accelerometer control. */
	QmiRegister_Ctrl2, // 3
	/*! \brief Gyroscope control. */
	QmiRegister_Ctrl3, // 4
	/*! \brief Magnetometer control. */
	QmiRegister_Ctrl4, // 5
	/*! \brief Data processing settings. */
	QmiRegister_Ctrl5, // 6
	/*! \brief AttitudeEngine control. */
	QmiRegister_Ctrl6, // 7
	/*! \brief Sensor enabled status. */
	QmiRegister_Ctrl7, // 8
	/*! \brief Reserved - do not write. */
	QmiRegister_Ctrl8, // 9
	/*! \brief Host command register. */
	QmiRegister_Ctrl9,
	/*! \brief Calibration register 1 least significant byte. */
	QmiRegister_Cal1_L,
	/*! \brief Calibration register 1 most significant byte. */
	QmiRegister_Cal1_H,
	/*! \brief Calibration register 2 least significant byte. */
	QmiRegister_Cal2_L,
	/*! \brief Calibration register 2 most significant byte. */
	QmiRegister_Cal2_H,
	/*! \brief Calibration register 3 least significant byte. */
	QmiRegister_Cal3_L,
	/*! \brief Calibration register 3 most significant byte. */
	QmiRegister_Cal3_H,
	/*! \brief Calibration register 4 least significant byte. */
	QmiRegister_Cal4_L,
	/*! \brief Calibration register 4 most significant byte. */
	QmiRegister_Cal4_H,
	/*! \brief FIFO control register. */
	QmiRegister_FifoCtrl,
	/*! \brief FIFO data register. */
	QmiRegister_FifoData,
	/*! \brief FIFO status register. */
	QmiRegister_FifoStatus,
	/*! \brief Output data overrun and availability. */
	QmiRegister_Status0,
	/*! \brief Miscellaneous status register. */
	QmiRegister_Status1,
	/*! \brief Sample counter. */
	QmiRegister_CountOut,
	/*! \brief Accelerometer X axis least significant byte. */
	QmiRegister_Ax_L,
	/*! \brief Accelerometer X axis most significant byte. */
	QmiRegister_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte. */
	QmiRegister_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte. */
	QmiRegister_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte. */
	QmiRegister_Az_L,
	/*! \brief Accelerometer Z axis most significant byte. */
	QmiRegister_Az_H,
	/*! \brief Gyroscope X axis least significant byte. */
	QmiRegister_Gx_L,
	/*! \brief Gyroscope X axis most significant byte. */
	QmiRegister_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte. */
	QmiRegister_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte. */
	QmiRegister_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte. */
	QmiRegister_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte. */
	QmiRegister_Gz_H,
	/*! \brief Magnetometer X axis least significant byte. */
	QmiRegister_Mx_L,
	/*! \brief Magnetometer X axis most significant byte. */
	QmiRegister_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte. */
	QmiRegister_My_L,
	/*! \brief Magnetometer Y axis most significant byte. */
	QmiRegister_My_H,
	/*! \brief Magnetometer Z axis least significant byte. */
	QmiRegister_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte. */
	QmiRegister_Mz_H,
	/*! \brief Quaternion increment W least significant byte. */
	QmiRegister_Q1_L = 45,
	/*! \brief Quaternion increment W most significant byte. */
	QmiRegister_Q1_H,
	/*! \brief Quaternion increment X least significant byte. */
	QmiRegister_Q2_L,
	/*! \brief Quaternion increment X most significant byte. */
	QmiRegister_Q2_H,
	/*! \brief Quaternion increment Y least significant byte. */
	QmiRegister_Q3_L,
	/*! \brief Quaternion increment Y most significant byte. */
	QmiRegister_Q3_H,
	/*! \brief Quaternion increment Z least significant byte. */
	QmiRegister_Q4_L,
	/*! \brief Quaternion increment Z most significant byte. */
	QmiRegister_Q4_H,
	/*! \brief Velocity increment X least significant byte. */
	QmiRegister_Dvx_L,
	/*! \brief Velocity increment X most significant byte. */
	QmiRegister_Dvx_H,
	/*! \brief Velocity increment Y least significant byte. */
	QmiRegister_Dvy_L,
	/*! \brief Velocity increment Y most significant byte. */
	QmiRegister_Dvy_H,
	/*! \brief Velocity increment Z least significant byte. */
	QmiRegister_Dvz_L,
	/*! \brief Velocity increment Z most significant byte. */
	QmiRegister_Dvz_H,
	/*! \brief Temperature output. */
	QmiRegister_Temperature,
	/*! \brief AttitudeEngine clipping flags. */
	QmiRegister_AeClipping,
	/*! \brief AttitudeEngine overflow flags. */
	QmiRegister_AeOverflow,
};

enum QmiImu_Ctrl9Command
{
	/*! \brief No operation. */
	Ctrl9_Nop = 0,
	/*! \brief Reset FIFO. */
	Ctrl9_ResetFifo = 0x2,
	/*! \brief Set magnetometer X calibration values. */
	Ctrl9_SetMagXCalibration = 0x6,
	/*! \brief Set magnetometer Y calibration values. */
	Ctrl9_SetMagYCalibration = 0x7,
	/*! \brief Set magnetometer Z calibration values. */
	Ctrl9_SetMagZCalibration = 0x8,
	/*! \brief Set accelerometer offset correction value. */
	Ctrl9_SetAccelOffset = 0x12,
	/*! \brief Set gyroscope offset correction value. */
	Ctrl9_SetGyroOffset = 0x13,
	/*! \brief Set accelerometer sensitivity. */
	Ctrl9_SetAccelSensitivity = 0x14,
	/*! \brief Set gyroscope sensitivity. */
	Ctrl9_SetGyroSensitivity = 0x15,
	/*! \brief Update magnemoter bias compensation. */
	Ctrl9_UpdateMagBias = 0xB,
	/*! \brief Trigger motion on demand sample. */
	Ctrl9_TriggerMotionOnDemand = 0x0c,
	/*! \brief Update gyroscope bias compensation. */
	Ctrl9_UpdateAttitudeEngineGyroBias = 0xE,
	/*! \brief Read frequency correction value. */
	Ctrl9_ReadTrimmedFrequencyValue = 0x18,
	/*! \brief Prepare for FIFO read sequence. */
	Ctrl9_ReadFifo = 0x0D,
	/*! \brief Set wake on motion parameters. */
	Ctrl9_ConfigureWakeOnMotion = 0x19,
};

enum QmiImu_LpfConfig
{
	Lpf_Disable, /*!< \brief Disable low pass filter. */
	Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum QmiImu_HpfConfig
{
	Hpf_Disable, /*!< \brief Disable high pass filter. */
	Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum QmiImu_AccRange
{
	AccRange_2g = 0 << 3, /*!< \brief +/- 2g range */
	AccRange_4g = 1 << 3, /*!< \brief +/- 4g range */
	AccRange_8g = 2 << 3, /*!< \brief +/- 8g range */
	AccRange_16g = 3 << 3 /*!< \brief +/- 16g range */
};


enum QmiImu_AccOdr
{
	AccOdr_1024Hz = 0,  /*!< \brief High resolution 1024Hz output rate. */
	AccOdr_256Hz = 1, /*!< \brief High resolution 256Hz output rate. */
	AccOdr_128Hz = 2, /*!< \brief High resolution 128Hz output rate. */
	AccOdr_32Hz = 3,  /*!< \brief High resolution 32Hz output rate. */
	AccOdr_LowPower_128Hz = 4, /*!< \brief Low power 128Hz output rate. */
	AccOdr_LowPower_64Hz = 5,  /*!< \brief Low power 64Hz output rate. */
	AccOdr_LowPower_25Hz = 6,  /*!< \brief Low power 25Hz output rate. */
	AccOdr_LowPower_3Hz = 7    /*!< \brief Low power 3Hz output rate. */
};

enum QmiImu_GyrRange
{
	GyrRange_32dps = 0 << 3,   /*!< \brief +-32 degrees per second. */
	GyrRange_64dps = 1 << 3,   /*!< \brief +-64 degrees per second. */
	GyrRange_128dps = 2 << 3,  /*!< \brief +-128 degrees per second. */
	GyrRange_256dps = 3 << 3,  /*!< \brief +-256 degrees per second. */
	GyrRange_512dps = 4 << 3,  /*!< \brief +-512 degrees per second. */
	GyrRange_1024dps = 5 << 3, /*!< \brief +-1024 degrees per second. */
	GyrRange_2048dps = 6 << 3, /*!< \brief +-2048 degrees per second. */
	GyrRange_2560dps = 7 << 3  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum QmiImu_GyrOdr
{
	GyrOdr_1024Hz			= 0,	/*!< \brief High resolution 1024Hz output rate. */
	GyrOdr_256Hz			= 1,	/*!< \brief High resolution 256Hz output rate. */
	GyrOdr_128Hz			= 2,	/*!< \brief High resolution 128Hz output rate. */
	GyrOdr_32Hz				= 3,	/*!< \brief High resolution 32Hz output rate. */
	GyrOdr_OIS_8192Hz		= 6,	/*!< \brief OIS Mode 8192Hz output rate. */
	GyrOdr_OIS_LL_8192Hz	= 7		/*!< \brief OIS LL Mode 8192Hz output rate. */
};

enum QmiImu_AeOdr
{
	AeOdr_1Hz = 0,  /*!< \brief 1Hz output rate. */
	AeOdr_2Hz = 1,  /*!< \brief 2Hz output rate. */
	AeOdr_4Hz = 2,  /*!< \brief 4Hz output rate. */
	AeOdr_8Hz = 3,  /*!< \brief 8Hz output rate. */
	AeOdr_16Hz = 4, /*!< \brief 16Hz output rate. */
	AeOdr_32Hz = 5, /*!< \brief 32Hz output rate. */
	AeOdr_64Hz = 6,  /*!< \brief 64Hz output rate. */
	/*!
	 * \brief Motion on demand mode.
	 *
	 * In motion on demand mode the application can trigger AttitudeEngine
	 * output samples as necessary. This allows the AttitudeEngine to be
	 * synchronized with external data sources.
	 *
	 * When in Motion on Demand mode the application should request new data
	 * by calling the QmiImu_requestAttitudeEngineData() function. The
	 * AttitudeEngine will respond with a data ready event (INT2) when the
	 * data is available to be read.
	 */
	AeOdr_motionOnDemand = 128
};

enum QmiImu_MagOdr
{
	MagOdr_32Hz = 2   /*!< \brief 32Hz output rate. */
};
	
enum QmiImu_MagDev
{
	MagDev_AK8975 = (0 << 4), /*!< \brief AKM AK8975. */
	MagDev_AK8963 = (1 << 4) /*!< \brief AKM AK8963. */
};

enum QmiImu_AccUnit
{
	AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum QmiImu_GyrUnit
{
	GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

struct QmiImuConfig
{
	/*! \brief Sensor fusion input selection. */
	uint8_t inputSelection;
	/*! \brief Accelerometer dynamic range configuration. */
	enum QmiImu_AccRange accRange;
	/*! \brief Accelerometer output rate. */
	enum QmiImu_AccOdr accOdr;
	/*! \brief Gyroscope dynamic range configuration. */
	enum QmiImu_GyrRange gyrRange;
	/*! \brief Gyroscope output rate. */
	enum QmiImu_GyrOdr gyrOdr;
	/*! \brief AttitudeEngine output rate. */
	enum QmiImu_AeOdr aeOdr;
	/*!
	 * \brief Magnetometer output data rate.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 * In this case the external magnetometer is sampled at the QMI output
	 * data rate, or at an integer divisor thereof such that the maximum
	 * sample rate is not exceeded.
	 */
	enum QmiImu_MagOdr magOdr;

	/*!
	 * \brief Magnetometer device to use.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 */
	enum QmiImu_MagDev magDev;
};


#define QMIIMU_SAMPLE_SIZE (3 * sizeof(int16_t))
#define QMIIMU_AE_SAMPLE_SIZE ((4+3+1) * sizeof(int16_t) + sizeof(uint8_t))
struct QmiImuRawSample
{
	/*! \brief The sample counter of the sample. */
	uint8_t sampleCounter;
	/*!
	 * \brief Pointer to accelerometer data in the sample buffer.
	 *
	 * \c NULL if no accelerometer data is available in the buffer.
	 */
	uint8_t const* accelerometerData;
	/*!
	 * \brief Pointer to gyroscope data in the sample buffer.
	 *
	 * \c NULL if no gyroscope data is available in the buffer.
	 */
	uint8_t const* gyroscopeData;
	/*!
	 * \brief Pointer to magnetometer data in the sample buffer.
	 *
	 * \c NULL if no magnetometer data is available in the buffer.
	 */
	uint8_t const* magnetometerData;
	/*!
	 * \brief Pointer to AttitudeEngine data in the sample buffer.
	 *
	 * \c NULL if no AttitudeEngine data is available in the buffer.
	 */
	uint8_t const* attitudeEngineData;
	/*! \brief Raw sample buffer. */
	uint8_t sampleBuffer[QMIIMU_SAMPLE_SIZE + QMIIMU_AE_SAMPLE_SIZE];
	/*! \brief Contents of the QMI status 1 register. */
	uint8_t status1;
	//uint8_t status0;
	//uint32_t durT;
};

struct QmiImu_offsetCalibration
{
	enum QmiImu_AccUnit accUnit;
	float accOffset[3];
	enum QmiImu_GyrUnit gyrUnit;
	float gyrOffset[3];
};

struct QmiImu_sensitivityCalibration
{
	float accSensitivity[3];
	float gyrSensitivity[3];
};

enum QmiImu_Interrupt
{
	/*! \brief QMI INT1 line. */
	Qmi_Int1 = (0 << 6),
	/*! \brief QMI INT2 line. */
	Qmi_Int2 = (1 << 6)
};

enum QmiImu_InterruptInitialState
{
	InterruptInitialState_high = (1 << 7), /*!< Interrupt high. */
	InterruptInitialState_low  = (0 << 7)  /*!< Interrupt low. */
};

enum QmiImu_WakeOnMotionThreshold
{
	WomThreshold_high = 128, /*!< High threshold - large motion needed to wake. */
	WomThreshold_low  = 32   /*!< Low threshold - small motion needed to wake. */
};

enum QmiImu_type {
	QMI_IMU_ACC = 0,
	QMI_IMU_GYRO = 1,

	QMI_IMU_MAX
};

enum QmiImu_axis {
	AXIS_X = 0,
	AXIS_Y,
	AXIS_Z,
	AXIS_NUM
};

typedef struct{
	signed char sign[3];
	unsigned char map[3];
} QmiImu_map;

typedef struct
{
	unsigned char				chip_id;
	struct i2c_client			*client;
	struct QmiImuConfig 		config;
	enum QmiImu_AccUnit			a_uint;
	int							a_lsb;
	enum QmiImu_GyrUnit			g_uint;
	int							g_lsb;

	struct mutex				op_mutex;	
	atomic_t					trace;	
	atomic_t					layout;
	int							acc_delay;	
	int							gyro_delay;
	int							acc_enable;	
	int							gyro_enable;	
	int							rst_gpio;	
	int							int1_gpio;	
	int							int2_gpio;	
	struct						delayed_work acc_work;
	struct						delayed_work gyro_work;
	struct input_dev 			*acc_input;	
	struct input_dev 			*gyro_input;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		early_drv; 
#endif
} QmiImu_t;


#define ACC_IOCTL_BASE                   77
#define GYRO_IOCTL_BASE                  78
#define ACC_IOCTL_SET_ENABLE             _IOW(ACC_IOCTL_BASE, 0, int)
#define ACC_IOCTL_SET_DELAY              _IOW(ACC_IOCTL_BASE, 1, int)
#define GYRO_IOCTL_SET_ENABLE            _IOW(GYRO_IOCTL_BASE, 0, int)
#define GYRO_IOCTL_SET_DELAY             _IOW(GYRO_IOCTL_BASE, 1, int)
#if 0
#define QMIIMU_ATTR_WR		S_IRUGO | S_IWUSR
#define QMIIMU_ATTR_R		S_IRUGO
#define QMIIMU_ATTR_W		S_IWUSR

#define QMIIMU_ATTR_WR		444 | 200
#define QMIIMU_ATTR_R		444
#define QMIIMU_ATTR_W		200
#endif
#define QMIIMU_ATTR_WR		(444 | 222)
#define QMIIMU_ATTR_R		(444)
#define QMIIMU_ATTR_W		(220)

#endif

