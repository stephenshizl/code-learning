#ifndef QMI8610_H
#define QMI8610_H

#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"


#ifndef M_PI
#define M_PI	(3.14159265358979323846f)
#endif
#ifndef ONE_G
#define ONE_G	(9.807f)
#endif

//#define QMI8610_USE_FIFO

#define QMI8610_CTRL7_DISABLE_ALL			(0x0)
#define QMI8610_CTRL7_ACC_ENABLE			(0x1)
#define QMI8610_CTRL7_GYR_ENABLE			(0x2)
#define QMI8610_CTRL7_MAG_ENABLE			(0x4)
#define QMI8610_CTRL7_AE_ENABLE				(0x8)
#define QMI8610_CTRL7_ENABLE_MASK			(0xF)

#define QMI8610_CONFIG_ACC_ENABLE			QMI8610_CTRL7_ACC_ENABLE
#define QMI8610_CONFIG_GYR_ENABLE			QMI8610_CTRL7_GYR_ENABLE
#define QMI8610_CONFIG_MAG_ENABLE			QMI8610_CTRL7_MAG_ENABLE
#define QMI8610_CONFIG_AE_ENABLE			QMI8610_CTRL7_AE_ENABLE
#define QMI8610_CONFIG_ACCGYR_ENABLE		(QMI8610_CONFIG_ACC_ENABLE | QMI8610_CONFIG_GYR_ENABLE)
#define QMI8610_CONFIG_ACCGYRMAG_ENABLE		(QMI8610_CONFIG_ACC_ENABLE | QMI8610_CONFIG_GYR_ENABLE | QMI8610_CONFIG_MAG_ENABLE)
#define QMI8610_CONFIG_AEMAG_ENABLE			(QMI8610_CONFIG_AE_ENABLE | QMI8610_CONFIG_MAG_ENABLE)

#define QMI8610_STATUS1_CMD_DONE			(0x01)
#define QMI8610_STATUS1_WAKEUP_EVENT		(0x04)

enum Qmi8610Register
{
	/*! \brief FIS device identifier register. */
	Qmi8610Register_WhoAmI=0, // 0
	/*! \brief FIS hardware revision register. */
	Qmi8610Register_Revision, // 1
	/*! \brief General and power management modes. */
	Qmi8610Register_Ctrl1, // 2
	/*! \brief Accelerometer control. */
	Qmi8610Register_Ctrl2, // 3
	/*! \brief Gyroscope control. */
	Qmi8610Register_Ctrl3, // 4
	/*! \brief Magnetometer control. */
	Qmi8610Register_Ctrl4, // 5
	/*! \brief Data processing settings. */
	Qmi8610Register_Ctrl5, // 6
	/*! \brief AttitudeEngine control. */
	Qmi8610Register_Ctrl6, // 7
	/*! \brief Sensor enabled status. */
	Qmi8610Register_Ctrl7, // 8
	/*! \brief Reserved - do not write. */
	Qmi8610Register_Ctrl8, // 9
	/*! \brief Host command register. */
	Qmi8610Register_Ctrl9,
	/*! \brief Calibration register 1 least significant byte. */
	Qmi8610Register_Cal1_L,
	/*! \brief Calibration register 1 most significant byte. */
	Qmi8610Register_Cal1_H,
	/*! \brief Calibration register 2 least significant byte. */
	Qmi8610Register_Cal2_L,
	/*! \brief Calibration register 2 most significant byte. */
	Qmi8610Register_Cal2_H,
	/*! \brief Calibration register 3 least significant byte. */
	Qmi8610Register_Cal3_L,
	/*! \brief Calibration register 3 most significant byte. */
	Qmi8610Register_Cal3_H,
	/*! \brief Calibration register 4 least significant byte. */
	Qmi8610Register_Cal4_L,
	/*! \brief Calibration register 4 most significant byte. */
	Qmi8610Register_Cal4_H,
	/*! \brief FIFO control register. */
	Qmi8610Register_FifoCtrl,
	/*! \brief FIFO data register. */
	Qmi8610Register_FifoData,
	/*! \brief FIFO status register. */
	Qmi8610Register_FifoStatus,
	/*! \brief Output data overrun and availability. */
	Qmi8610Register_Status0,
	/*! \brief Miscellaneous status register. */
	Qmi8610Register_Status1,
	/*! \brief Sample counter. */
	Qmi8610Register_CountOut,
	/*! \brief Accelerometer X axis least significant byte. */
	Qmi8610Register_Ax_L,
	/*! \brief Accelerometer X axis most significant byte. */
	Qmi8610Register_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte. */
	Qmi8610Register_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte. */
	Qmi8610Register_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte. */
	Qmi8610Register_Az_L,
	/*! \brief Accelerometer Z axis most significant byte. */
	Qmi8610Register_Az_H,
	/*! \brief Gyroscope X axis least significant byte. */
	Qmi8610Register_Gx_L,
	/*! \brief Gyroscope X axis most significant byte. */
	Qmi8610Register_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte. */
	Qmi8610Register_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte. */
	Qmi8610Register_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte. */
	Qmi8610Register_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte. */
	Qmi8610Register_Gz_H,
	/*! \brief Magnetometer X axis least significant byte. */
	Qmi8610Register_Mx_L,
	/*! \brief Magnetometer X axis most significant byte. */
	Qmi8610Register_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte. */
	Qmi8610Register_My_L,
	/*! \brief Magnetometer Y axis most significant byte. */
	Qmi8610Register_My_H,
	/*! \brief Magnetometer Z axis least significant byte. */
	Qmi8610Register_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte. */
	Qmi8610Register_Mz_H,
	/*! \brief Quaternion increment W least significant byte. */
	Qmi8610Register_Q1_L = 45,
	/*! \brief Quaternion increment W most significant byte. */
	Qmi8610Register_Q1_H,
	/*! \brief Quaternion increment X least significant byte. */
	Qmi8610Register_Q2_L,
	/*! \brief Quaternion increment X most significant byte. */
	Qmi8610Register_Q2_H,
	/*! \brief Quaternion increment Y least significant byte. */
	Qmi8610Register_Q3_L,
	/*! \brief Quaternion increment Y most significant byte. */
	Qmi8610Register_Q3_H,
	/*! \brief Quaternion increment Z least significant byte. */
	Qmi8610Register_Q4_L,
	/*! \brief Quaternion increment Z most significant byte. */
	Qmi8610Register_Q4_H,
	/*! \brief Velocity increment X least significant byte. */
	Qmi8610Register_Dvx_L,
	/*! \brief Velocity increment X most significant byte. */
	Qmi8610Register_Dvx_H,
	/*! \brief Velocity increment Y least significant byte. */
	Qmi8610Register_Dvy_L,
	/*! \brief Velocity increment Y most significant byte. */
	Qmi8610Register_Dvy_H,
	/*! \brief Velocity increment Z least significant byte. */
	Qmi8610Register_Dvz_L,
	/*! \brief Velocity increment Z most significant byte. */
	Qmi8610Register_Dvz_H,
	/*! \brief Temperature output. */
	Qmi8610Register_Temperature,
	/*! \brief AttitudeEngine clipping flags. */
	Qmi8610Register_AeClipping,
	/*! \brief AttitudeEngine overflow flags. */
	Qmi8610Register_AeOverflow,
};

enum qmi8610_Ctrl9Command
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

enum qmi8610_FifoWatermarkLevel
{
	Fifo_WatermarkEmpty = (0 << 4),
	Fifo_WatermarkOneQuarter = (1 << 4),
	Fifo_WatermarkHalf = (2 << 4),
	Fifo_WatermarkThreeQuarters = (3 << 4)
};

enum qmi8610_FifoSize
{
	FifoSize_16 = (0 << 2), /*!< \brief 16 sample FIFO. */
	FifoSize_32 = (1 << 2), /*!< \brief 32 sample FIFO. */
	FifoSize_64 = (2 << 2), /*!< \brief 64 sample FIFO. */
	FifoSize_128 = (3 << 2) /*!< \brief 128 sample FIFO. */
};

enum qmi8610_FifoMode
{
	FifoMode_Disable = 0,
	FifoMode_Fifo = 1,
	FifoMode_Stream = 2
};



enum qmi8610_LpfConfig
{
	Qmi8610Lpf_Disable, /*!< \brief Disable low pass filter. */
	Qmi8610Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum qmi8610_HpfConfig
{
	Qmi8610Hpf_Disable, /*!< \brief Disable high pass filter. */
	Qmi8610Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum qmi8610_AccRange
{
	Qmi8610AccRange_2g = 0 << 3, /*!< \brief +/- 2g range */
	Qmi8610AccRange_4g = 1 << 3, /*!< \brief +/- 4g range */
	Qmi8610AccRange_8g = 2 << 3, /*!< \brief +/- 8g range */
	Qmi8610AccRange_16g = 3 << 3 /*!< \brief +/- 16g range */
};


enum qmi8610_AccOdr
{
	Qmi8610AccOdr_1024Hz = 0,  /*!< \brief High resolution 1024Hz output rate. */
	Qmi8610AccOdr_256Hz = 1, /*!< \brief High resolution 256Hz output rate. */
	Qmi8610AccOdr_128Hz = 2, /*!< \brief High resolution 128Hz output rate. */
	Qmi8610AccOdr_32Hz = 3,  /*!< \brief High resolution 32Hz output rate. */
	Qmi8610AccOdr_LowPower_128Hz = 4, /*!< \brief Low power 128Hz output rate. */
	Qmi8610AccOdr_LowPower_64Hz = 5,  /*!< \brief Low power 64Hz output rate. */
	Qmi8610AccOdr_LowPower_25Hz = 6,  /*!< \brief Low power 25Hz output rate. */
	Qmi8610AccOdr_LowPower_3Hz = 7    /*!< \brief Low power 3Hz output rate. */
};

enum qmi8610_GyrRange
{
	Qmi8610GyrRange_32dps = 0 << 3,   /*!< \brief +-32 degrees per second. */
	Qmi8610GyrRange_64dps = 1 << 3,   /*!< \brief +-64 degrees per second. */
	Qmi8610GyrRange_128dps = 2 << 3,  /*!< \brief +-128 degrees per second. */
	Qmi8610GyrRange_256dps = 3 << 3,  /*!< \brief +-256 degrees per second. */
	Qmi8610GyrRange_512dps = 4 << 3,  /*!< \brief +-512 degrees per second. */
	Qmi8610GyrRange_1024dps = 5 << 3, /*!< \brief +-1024 degrees per second. */
	Qmi8610GyrRange_2048dps = 6 << 3, /*!< \brief +-2048 degrees per second. */
	Qmi8610GyrRange_2560dps = 7 << 3  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum qmi8610_GyrOdr
{
	Qmi8610GyrOdr_1024Hz			= 0,	/*!< \brief High resolution 1024Hz output rate. */
	Qmi8610GyrOdr_256Hz			= 1,	/*!< \brief High resolution 256Hz output rate. */
	Qmi8610GyrOdr_128Hz			= 2,	/*!< \brief High resolution 128Hz output rate. */
	Qmi8610GyrOdr_32Hz				= 3,	/*!< \brief High resolution 32Hz output rate. */
	Qmi8610GyrOdr_OIS_8192Hz		= 6,	/*!< \brief OIS Mode 8192Hz output rate. */
	Qmi8610GyrOdr_OIS_LL_8192Hz	= 7		/*!< \brief OIS LL Mode 8192Hz output rate. */
};

enum qmi8610_AeOdr
{
	Qmi8610AeOdr_1Hz = 0,  /*!< \brief 1Hz output rate. */
	Qmi8610AeOdr_2Hz = 1,  /*!< \brief 2Hz output rate. */
	Qmi8610AeOdr_4Hz = 2,  /*!< \brief 4Hz output rate. */
	Qmi8610AeOdr_8Hz = 3,  /*!< \brief 8Hz output rate. */
	Qmi8610AeOdr_16Hz = 4, /*!< \brief 16Hz output rate. */
	Qmi8610AeOdr_32Hz = 5, /*!< \brief 32Hz output rate. */
	Qmi8610AeOdr_64Hz = 6,  /*!< \brief 64Hz output rate. */
	/*!
	 * \brief Motion on demand mode.
	 *
	 * In motion on demand mode the application can trigger AttitudeEngine
	 * output samples as necessary. This allows the AttitudeEngine to be
	 * synchronized with external data sources.
	 *
	 * When in Motion on Demand mode the application should request new data
	 * by calling the qmi8610_requestAttitudeEngineData() function. The
	 * AttitudeEngine will respond with a data ready event (INT2) when the
	 * data is available to be read.
	 */
	AeOdr_motionOnDemand = 128
};

enum qmi8610_MagOdr
{
	Qmi8610MagOdr_32Hz = 2   /*!< \brief 32Hz output rate. */
};
	
enum qmi8610_MagDev
{
	MagDev_AK8975 = (0 << 4), /*!< \brief AKM AK8975. */
	MagDev_AK8963 = (1 << 4) /*!< \brief AKM AK8963. */
};

enum qmi8610_AccUnit
{
	Qmi8610AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	Qmi8610AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum qmi8610_GyrUnit
{
	Qmi8610GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	Qmi8610GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

struct Qmi8610Config
{
	/*! \brief Sensor fusion input selection. */
	unsigned char inputSelection;
	/*! \brief Accelerometer dynamic range configuration. */
	enum qmi8610_AccRange accRange;
	/*! \brief Accelerometer output rate. */
	enum qmi8610_AccOdr accOdr;
	/*! \brief Gyroscope dynamic range configuration. */
	enum qmi8610_GyrRange gyrRange;
	/*! \brief Gyroscope output rate. */
	enum qmi8610_GyrOdr gyrOdr;
	/*! \brief AttitudeEngine output rate. */
	enum qmi8610_AeOdr aeOdr;
	/*!
	 * \brief Magnetometer output data rate.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 * In this case the external magnetometer is sampled at the FIS output
	 * data rate, or at an integer divisor thereof such that the maximum
	 * sample rate is not exceeded.
	 */
	enum qmi8610_MagOdr magOdr;

	/*!
	 * \brief Magnetometer device to use.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 */
	enum qmi8610_MagDev magDev;
};


#define QMI8610_SAMPLE_SIZE (3 * sizeof(short))
#define QMI8610_AE_SAMPLE_SIZE ((4+3+1) * sizeof(short) + sizeof(unsigned char))
struct Qmi8610RawSample
{
	/*! \brief The sample counter of the sample. */
	unsigned char sampleCounter;
	/*!
	 * \brief Pointer to accelerometer data in the sample buffer.
	 *
	 * \c NULL if no accelerometer data is available in the buffer.
	 */
	unsigned char const* accelerometerData;
	/*!
	 * \brief Pointer to gyroscope data in the sample buffer.
	 *
	 * \c NULL if no gyroscope data is available in the buffer.
	 */
	unsigned char const* gyroscopeData;
	/*!
	 * \brief Pointer to magnetometer data in the sample buffer.
	 *
	 * \c NULL if no magnetometer data is available in the buffer.
	 */
	unsigned char const* magnetometerData;
	/*!
	 * \brief Pointer to AttitudeEngine data in the sample buffer.
	 *
	 * \c NULL if no AttitudeEngine data is available in the buffer.
	 */
	unsigned char const* attitudeEngineData;
	/*! \brief Raw sample buffer. */
	unsigned char sampleBuffer[QMI8610_SAMPLE_SIZE + QMI8610_AE_SAMPLE_SIZE];
	/*! \brief Contents of the FIS status 1 register. */
	unsigned char status1;
	//unsigned char status0;
	//uint32_t durT;
};

struct qmi8610_offsetCalibration
{
	enum qmi8610_AccUnit accUnit;
	float accOffset[3];
	enum qmi8610_GyrUnit gyrUnit;
	float gyrOffset[3];
};

struct qmi8610_sensitivityCalibration
{
	float accSensitivity[3];
	float gyrSensitivity[3];
};

enum qmi8610_Interrupt
{
	/*! \brief FIS INT1 line. */
	Qmi8610_Int1 = (0 << 6),
	/*! \brief FIS INT2 line. */
	Qmi8610_Int2 = (1 << 6)
};

enum qmi8610_InterruptInitialState
{
	InterruptInitialState_high = (1 << 7), /*!< Interrupt high. */
	InterruptInitialState_low  = (0 << 7)  /*!< Interrupt low. */
};

enum qmi8610_WakeOnMotionThreshold
{
	WomThreshold_high = 128, /*!< High threshold - large motion needed to wake. */
	WomThreshold_low  = 32   /*!< Low threshold - small motion needed to wake. */
};


extern unsigned char qmi8610_write_reg(unsigned char reg, unsigned char value);
extern unsigned char qmi8610_read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
extern unsigned char qmi8610_init(void);
extern void qmi8610_Config_apply(struct Qmi8610Config const* config);
extern void qmi8610_enableSensors(unsigned char enableFlags);
extern void qmi8610_read_acc_xyz(float acc_xyz[3]);
extern void qmi8610_read_gyro_xyz(float gyro_xyz[3]);
extern void qmi8610_read_xyz(float acc[3], float gyro[3], unsigned char *tim_count);
extern void qmi8610_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned char *tim_count);
extern unsigned char qmi8610_readStatus0(void);
extern unsigned char qmi8610_readStatus1(void);
extern char qmi8610_readTemp(void);
extern void qmi8610_enableWakeOnMotion(void);
extern void qmi8610_disableWakeOnMotion(void);

extern unsigned short qmi8610_configureFifo(enum qmi8610_FifoWatermarkLevel watermark, enum qmi8610_FifoSize size, enum qmi8610_FifoMode mode);
#if defined(QMI8610_USE_FIFO)
extern void qmi8610_readFifo(unsigned char* data, unsigned short dataLength);
extern void qmi8610_read_fifo(void);
#endif
// for XKF3
extern void qmi8610_processAccelerometerData(unsigned char const* rawData, float* calibratedData);
extern void qmi8610_processGyroscopeData(unsigned char const* rawData, float* calibratedData);
extern void qmi8610_read_rawsample(struct Qmi8610RawSample *sample);
extern void qmi8610_applyOffsetCalibration(struct qmi8610_offsetCalibration const* cal);
// for XKF3
#endif

