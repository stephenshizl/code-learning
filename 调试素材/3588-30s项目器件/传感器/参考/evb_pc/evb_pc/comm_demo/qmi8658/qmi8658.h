#ifndef QMI8658_H
#define QMI8658_H

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

#define QMI8658_CTRL7_DISABLE_ALL			(0x0)
#define QMI8658_CTRL7_ACC_ENABLE			(0x1)
#define QMI8658_CTRL7_GYR_ENABLE			(0x2)
#define QMI8658_CTRL7_MAG_ENABLE			(0x4)
#define QMI8658_CTRL7_AE_ENABLE				(0x8)
#define QMI8658_CTRL7_GYR_SNOOZE_ENABLE		(0x10)
#define QMI8658_CTRL7_ENABLE_MASK			(0xF)

#define QMI8658_CONFIG_ACC_ENABLE			QMI8658_CTRL7_ACC_ENABLE
#define QMI8658_CONFIG_GYR_ENABLE			QMI8658_CTRL7_GYR_ENABLE
#define QMI8658_CONFIG_MAG_ENABLE			QMI8658_CTRL7_MAG_ENABLE
#define QMI8658_CONFIG_AE_ENABLE			QMI8658_CTRL7_AE_ENABLE
#define QMI8658_CONFIG_ACCGYR_ENABLE		(QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE)
#define QMI8658_CONFIG_ACCGYRMAG_ENABLE		(QMI8658_CONFIG_ACC_ENABLE | QMI8658_CONFIG_GYR_ENABLE | QMI8658_CONFIG_MAG_ENABLE)
#define QMI8658_CONFIG_AEMAG_ENABLE			(QMI8658_CONFIG_AE_ENABLE | QMI8658_CONFIG_MAG_ENABLE)

#define QMI8658_STATUS1_CMD_DONE			(0x01)
#define QMI8658_STATUS1_WAKEUP_EVENT		(0x04)

enum Qmi8658Register
{
	/*! \brief FIS device identifier register. */
	Qmi8658Register_WhoAmI = 0, // 0
	/*! \brief FIS hardware revision register. */
	Qmi8658Register_Revision, // 1
	/*! \brief General and power management modes. */
	Qmi8658Register_Ctrl1, // 2
	/*! \brief Accelerometer control. */
	Qmi8658Register_Ctrl2, // 3
	/*! \brief Gyroscope control. */
	Qmi8658Register_Ctrl3, // 4
	/*! \brief Magnetometer control. */
	Qmi8658Register_Ctrl4, // 5
	/*! \brief Data processing settings. */
	Qmi8658Register_Ctrl5, // 6
	/*! \brief AttitudeEngine control. */
	Qmi8658Register_Ctrl6, // 7
	/*! \brief Sensor enabled status. */
	Qmi8658Register_Ctrl7, // 8
	/*! \brief Reserved - do not write. */
	Qmi8658Register_Ctrl8, // 9
	/*! \brief Host command register. */
	Qmi8658Register_Ctrl9,	// 10
	/*! \brief Calibration register 1 most significant byte. */
	Qmi8658Register_Cal1_L = 11,
	/*! \brief Calibration register 1 least significant byte. */
	Qmi8658Register_Cal1_H,
	/*! \brief Calibration register 2 most significant byte. */
	Qmi8658Register_Cal2_L,
	/*! \brief Calibration register 2 least significant byte. */
	Qmi8658Register_Cal2_H,
	/*! \brief Calibration register 3 most significant byte. */
	Qmi8658Register_Cal3_L,
	/*! \brief Calibration register 3 least significant byte. */
	Qmi8658Register_Cal3_H,
	/*! \brief Calibration register 4 most significant byte. */
	Qmi8658Register_Cal4_L,
	/*! \brief Calibration register 4 least significant byte. */
	Qmi8658Register_Cal4_H,
	/*! \brief FIFO control register. */
	Qmi8658Register_FifoCtrl = 19,
	/*! \brief FIFO data register. */
	Qmi8658Register_FifoData,	// 20
	/*! \brief FIFO status register. */
	Qmi8658Register_FifoStatus,	// 21	
	/*! \brief Output data overrun and availability. */
	Qmi8658Register_StatusInt = 45,
	/*! \brief Output data overrun and availability. */
	Qmi8658Register_Status0,
	/*! \brief Miscellaneous status register. */
	Qmi8658Register_Status1,
	/*! \brief timestamp low. */
	Qmi8658Register_Timestamp_L = 48,
	/*! \brief timestamp low. */
	Qmi8658Register_Timestamp_M,
	/*! \brief timestamp low. */
	Qmi8658Register_Timestamp_H,
	/*! \brief tempearture low. */
	Qmi8658Register_Tempearture_L = 51,
	/*! \brief tempearture low. */
	Qmi8658Register_Tempearture_H,
	/*! \brief Accelerometer X axis least significant byte. */
	Qmi8658Register_Ax_L = 53,
	/*! \brief Accelerometer X axis most significant byte. */
	Qmi8658Register_Ax_H,
	/*! \brief Accelerometer Y axis least significant byte. */
	Qmi8658Register_Ay_L,
	/*! \brief Accelerometer Y axis most significant byte. */
	Qmi8658Register_Ay_H,
	/*! \brief Accelerometer Z axis least significant byte. */
	Qmi8658Register_Az_L,
	/*! \brief Accelerometer Z axis most significant byte. */
	Qmi8658Register_Az_H,
	/*! \brief Gyroscope X axis least significant byte. */
	Qmi8658Register_Gx_L = 59,
	/*! \brief Gyroscope X axis most significant byte. */
	Qmi8658Register_Gx_H,
	/*! \brief Gyroscope Y axis least significant byte. */
	Qmi8658Register_Gy_L,
	/*! \brief Gyroscope Y axis most significant byte. */
	Qmi8658Register_Gy_H,
	/*! \brief Gyroscope Z axis least significant byte. */
	Qmi8658Register_Gz_L,
	/*! \brief Gyroscope Z axis most significant byte. */
	Qmi8658Register_Gz_H,
	/*! \brief Magnetometer X axis least significant byte. */
	Qmi8658Register_Mx_L = 65,
	/*! \brief Magnetometer X axis most significant byte. */
	Qmi8658Register_Mx_H,
	/*! \brief Magnetometer Y axis least significant byte. */
	Qmi8658Register_My_L,
	/*! \brief Magnetometer Y axis most significant byte. */
	Qmi8658Register_My_H,
	/*! \brief Magnetometer Z axis least significant byte. */
	Qmi8658Register_Mz_L,
	/*! \brief Magnetometer Z axis most significant byte. */
	Qmi8658Register_Mz_H,
	/*! \brief Quaternion increment W least significant byte. */
	Qmi8658Register_Q1_L = 73,
	/*! \brief Quaternion increment W most significant byte. */
	Qmi8658Register_Q1_H,
	/*! \brief Quaternion increment X least significant byte. */
	Qmi8658Register_Q2_L,
	/*! \brief Quaternion increment X most significant byte. */
	Qmi8658Register_Q2_H,
	/*! \brief Quaternion increment Y least significant byte. */
	Qmi8658Register_Q3_L,
	/*! \brief Quaternion increment Y most significant byte. */
	Qmi8658Register_Q3_H,
	/*! \brief Quaternion increment Z least significant byte. */
	Qmi8658Register_Q4_L,
	/*! \brief Quaternion increment Z most significant byte. */
	Qmi8658Register_Q4_H,
	/*! \brief Velocity increment X least significant byte. */
	Qmi8658Register_Dvx_L = 81,
	/*! \brief Velocity increment X most significant byte. */
	Qmi8658Register_Dvx_H,
	/*! \brief Velocity increment Y least significant byte. */
	Qmi8658Register_Dvy_L,
	/*! \brief Velocity increment Y most significant byte. */
	Qmi8658Register_Dvy_H,
	/*! \brief Velocity increment Z least significant byte. */
	Qmi8658Register_Dvz_L,
	/*! \brief Velocity increment Z most significant byte. */
	Qmi8658Register_Dvz_H,
	/*! \brief AttitudeEngine reg1. */
	Qmi8658Register_AeReg1 = 87,
	/*! \brief AttitudeEngine overflow flags. */
	Qmi8658Register_AeOverflow,

	Qmi8658Register_I2CM_STATUS = 110
};

enum Qmi8658_Ois_Register
{
	/*-----------------------------*/
	/* Setup and Control Registers */
	/*-----------------------------*/
	/*! \brief SPI Endian Selection, and SPI 3/4 Wire */
	Qmi8658_OIS_Reg_Ctrl1 = 0x02,		// 2  [0x02] -- Dflt: 0x20
	/*! \brief Accelerometer control: ODR, Full Scale, Self Test  */
	Qmi8658_OIS_Reg_Ctrl2, 		   		//  3  [0x03]
	/*! \brief Gyroscope control: ODR, Full Scale, Self Test */
	Qmi8658_OIS_Reg_Ctrl3,				//  4  [0x04]
	/*! \brief Sensor Data Processing Settings */
	Qmi8658_OIS_Reg_Ctrl5	= 0x06,				//  6  [0x06]
	/*! \brief Sensor enabled status: Enable Sensors  */
	Qmi8658_OIS_Reg_Ctrl7   = 0x08,				//  8  [0x08]
	/*-------------------*/
	/* Status Registers  */
	/*-------------------*/
	/*! \brief Sensor Data Availability and Lock Register */
	Qmi8658_OIS_Reg_StatusInt = 0x2D,			// 45  [0x2D]
	/*! \brief Output data overrun and availability */
	Qmi8658_OIS_Reg_Status0   = 0x2E,			// 46  [0x2E]

	/*-----------------------------------------------------*/
	/* OIS Sensor Data Output Registers. 16-bit 2's complement */
	/*-----------------------------------------------------*/
	/*! \brief Accelerometer X axis least significant byte */
	Qmi8658_OIS_Reg_Ax_L  = 0x33,			// 53  [0x35]
	/*! \brief Accelerometer X axis most significant byte */
	Qmi8658_OIS_Reg_Ax_H,					// 54  [0x36]
	/*! \brief Accelerometer Y axis least significant byte */
	Qmi8658_OIS_Reg_Ay_L,					// 55  [0x37]
	/*! \brief Accelerometer Y axis most significant byte */
	Qmi8658_OIS_Reg_Ay_H,					// 56  [0x38]
	/*! \brief Accelerometer Z axis least significant byte */
	Qmi8658_OIS_Reg_Az_L,					// 57  [0x39]
	/*! \brief Accelerometer Z axis most significant byte */
	Qmi8658_OIS_Reg_Az_H,					// 58  [0x3A]

	/*! \brief Gyroscope X axis least significant byte */
	Qmi8658_OIS_Reg_Gx_L  = 0x3B,			// 59  [0x3B]
	/*! \brief Gyroscope X axis most significant byte */
	Qmi8658_OIS_Reg_Gx_H,					// 60  [0x3C]
	/*! \brief Gyroscope Y axis least significant byte */
	Qmi8658_OIS_Reg_Gy_L,					// 61  [0x3D]
	/*! \brief Gyroscope Y axis most significant byte */
	Qmi8658_OIS_Reg_Gy_H,					// 62  [0x3E]
	/*! \brief Gyroscope Z axis least significant byte */
	Qmi8658_OIS_Reg_Gz_L,					// 63  [0x3F]
	/*! \brief Gyroscope Z axis most significant byte */
	Qmi8658_OIS_Reg_Gz_H,					// 64  [0x40]
};

enum Qmi8658_Ctrl9Command
{
	Qmi8658_Ctrl9_Cmd_NOP					= 0X00,
	Qmi8658_Ctrl9_Cmd_GyroBias				= 0X01,
	Qmi8658_Ctrl9_Cmd_Rqst_Sdi_Mod			= 0X03,
	Qmi8658_Ctrl9_Cmd_WoM_Setting			= 0x08,
	Qmi8658_Ctrl9_Cmd_AccelHostDeltaOffset	= 0x09,
	Qmi8658_Ctrl9_Cmd_GyroHostDeltaOffset	= 0x0A,
	Qmi8658_Ctrl9_Cmd_Dbg_WoM_Data_Enable	= 0xF8,

};


enum Qmi8658_LpfConfig
{
	Qmi8658Lpf_Disable, /*!< \brief Disable low pass filter. */
	Qmi8658Lpf_Enable   /*!< \brief Enable low pass filter. */
};

enum Qmi8658_HpfConfig
{
	Qmi8658Hpf_Disable, /*!< \brief Disable high pass filter. */
	Qmi8658Hpf_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_StConfig
{
	Qmi8658St_Disable, /*!< \brief Disable high pass filter. */
	Qmi8658St_Enable   /*!< \brief Enable high pass filter. */
};

enum Qmi8658_LpfMode
{
	A_LSP_MODE_0 = 0x00<<1,
	A_LSP_MODE_1 = 0x01<<1,
	A_LSP_MODE_2 = 0x02<<1,
	A_LSP_MODE_3 = 0x03<<1,
	
	G_LSP_MODE_0 = 0x00<<5,
	G_LSP_MODE_1 = 0x01<<5,
	G_LSP_MODE_2 = 0x02<<5,
	G_LSP_MODE_3 = 0x03<<5
};

enum Qmi8658_AccRange
{
	Qmi8658AccRange_2g = 0x00 << 4, /*!< \brief +/- 2g range */
	Qmi8658AccRange_4g = 0x01 << 4, /*!< \brief +/- 4g range */
	Qmi8658AccRange_8g = 0x02 << 4, /*!< \brief +/- 8g range */
	Qmi8658AccRange_16g = 0x03 << 4 /*!< \brief +/- 16g range */
};


enum Qmi8658_AccOdr
{
	Qmi8658AccOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	Qmi8658AccOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	Qmi8658AccOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	Qmi8658AccOdr_1000Hz = 0x03,  /*!< \brief High resolution 1000Hz output rate. */
	Qmi8658AccOdr_500Hz = 0x04,  /*!< \brief High resolution 500Hz output rate. */
	Qmi8658AccOdr_250Hz = 0x05, /*!< \brief High resolution 250Hz output rate. */
	Qmi8658AccOdr_125Hz = 0x06, /*!< \brief High resolution 125Hz output rate. */
	Qmi8658AccOdr_62_5Hz = 0x07, /*!< \brief High resolution 62.5Hz output rate. */
	Qmi8658AccOdr_31_25Hz = 0x08,  /*!< \brief High resolution 31.25Hz output rate. */
	Qmi8658AccOdr_LowPower_128Hz = 0x0c, /*!< \brief Low power 128Hz output rate. */
	Qmi8658AccOdr_LowPower_21Hz = 0x0d,  /*!< \brief Low power 21Hz output rate. */
	Qmi8658AccOdr_LowPower_11Hz = 0x0e,  /*!< \brief Low power 11Hz output rate. */
	Qmi8658AccOdr_LowPower_3Hz = 0x0f    /*!< \brief Low power 3Hz output rate. */
};

enum Qmi8658_GyrRange
{
	Qmi8658GyrRange_32dps = 0 << 4,   /*!< \brief +-32 degrees per second. */
	Qmi8658GyrRange_64dps = 1 << 4,   /*!< \brief +-64 degrees per second. */
	Qmi8658GyrRange_128dps = 2 << 4,  /*!< \brief +-128 degrees per second. */
	Qmi8658GyrRange_256dps = 3 << 4,  /*!< \brief +-256 degrees per second. */
	Qmi8658GyrRange_512dps = 4 << 4,  /*!< \brief +-512 degrees per second. */
	Qmi8658GyrRange_1024dps = 5 << 4, /*!< \brief +-1024 degrees per second. */
	Qmi8658GyrRange_2048dps = 6 << 4, /*!< \brief +-2048 degrees per second. */
	Qmi8658GyrRange_4096dps = 7 << 4  /*!< \brief +-2560 degrees per second. */
};

/*!
 * \brief Gyroscope output rate configuration.
 */
enum Qmi8658_GyrOdr
{
	Qmi8658GyrOdr_8000Hz = 0x00,  /*!< \brief High resolution 8000Hz output rate. */
	Qmi8658GyrOdr_4000Hz = 0x01,  /*!< \brief High resolution 4000Hz output rate. */
	Qmi8658GyrOdr_2000Hz = 0x02,  /*!< \brief High resolution 2000Hz output rate. */
	Qmi8658GyrOdr_1000Hz = 0x03,	/*!< \brief High resolution 1000Hz output rate. */
	Qmi8658GyrOdr_500Hz	= 0x04,	/*!< \brief High resolution 500Hz output rate. */
	Qmi8658GyrOdr_250Hz	= 0x05,	/*!< \brief High resolution 250Hz output rate. */
	Qmi8658GyrOdr_125Hz	= 0x06,	/*!< \brief High resolution 125Hz output rate. */
	Qmi8658GyrOdr_62_5Hz 	= 0x07,	/*!< \brief High resolution 62.5Hz output rate. */
	Qmi8658GyrOdr_31_25Hz	= 0x08	/*!< \brief High resolution 31.25Hz output rate. */
};

enum Qmi8658_AeOdr
{
	Qmi8658AeOdr_1Hz = 0x00,  /*!< \brief 1Hz output rate. */
	Qmi8658AeOdr_2Hz = 0x01,  /*!< \brief 2Hz output rate. */
	Qmi8658AeOdr_4Hz = 0x02,  /*!< \brief 4Hz output rate. */
	Qmi8658AeOdr_8Hz = 0x03,  /*!< \brief 8Hz output rate. */
	Qmi8658AeOdr_16Hz = 0x04, /*!< \brief 16Hz output rate. */
	Qmi8658AeOdr_32Hz = 0x05, /*!< \brief 32Hz output rate. */
	Qmi8658AeOdr_64Hz = 0x06,  /*!< \brief 64Hz output rate. */
	Qmi8658AeOdr_128Hz = 0x07,  /*!< \brief 128Hz output rate. */
	/*!
	 * \brief Motion on demand mode.
	 *
	 * In motion on demand mode the application can trigger AttitudeEngine
	 * output samples as necessary. This allows the AttitudeEngine to be
	 * synchronized with external data sources.
	 *
	 * When in Motion on Demand mode the application should request new data
	 * by calling the Qmi8658_requestAttitudeEngineData() function. The
	 * AttitudeEngine will respond with a data ready event (INT2) when the
	 * data is available to be read.
	 */
	Qmi8658AeOdr_motionOnDemand = 128
};

enum Qmi8658_MagOdr
{
	Qmi8658MagOdr_1000Hz = 0x00,   /*!< \brief 1000Hz output rate. */
	Qmi8658MagOdr_500Hz = 0x01,	/*!< \brief 500Hz output rate. */
	Qmi8658MagOdr_250Hz = 0x02,	/*!< \brief 250Hz output rate. */
	Qmi8658MagOdr_125Hz = 0x03,	/*!< \brief 125Hz output rate. */
	Qmi8658MagOdr_62_5Hz = 0x04,	/*!< \brief 62.5Hz output rate. */
	Qmi8658MagOdr_31_25Hz = 0x05	/*!< \brief 31.25Hz output rate. */
};
	
enum Qmi8658_MagDev
{
	MagDev_AKM09918 = (0 << 3), /*!< \brief AKM09918. */
};

enum Qmi8658_AccUnit
{
	Qmi8658AccUnit_g,  /*!< \brief Accelerometer output in terms of g (9.81m/s^2). */
	Qmi8658AccUnit_ms2 /*!< \brief Accelerometer output in terms of m/s^2. */
};

enum Qmi8658_GyrUnit
{
	Qmi8658GyrUnit_dps, /*!< \brief Gyroscope output in degrees/s. */
	Qmi8658GyrUnit_rads /*!< \brief Gyroscope output in rad/s. */
};

struct Qmi8658Config
{
	/*! \brief Sensor fusion input selection. */
	unsigned char inputSelection;
	/*! \brief Accelerometer dynamic range configuration. */
	enum Qmi8658_AccRange accRange;
	/*! \brief Accelerometer output rate. */
	enum Qmi8658_AccOdr accOdr;
	/*! \brief Gyroscope dynamic range configuration. */
	enum Qmi8658_GyrRange gyrRange;
	/*! \brief Gyroscope output rate. */
	enum Qmi8658_GyrOdr gyrOdr;
	/*! \brief AttitudeEngine output rate. */
	enum Qmi8658_AeOdr aeOdr;
	/*!
	 * \brief Magnetometer output data rate.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 * In this case the external magnetometer is sampled at the FIS output
	 * data rate, or at an integer divisor thereof such that the maximum
	 * sample rate is not exceeded.
	 */
	enum Qmi8658_MagOdr magOdr;

	/*!
	 * \brief Magnetometer device to use.
	 *
	 * \remark This parameter is not used when using an external magnetometer.
	 */
	enum Qmi8658_MagDev magDev;
};


#define QMI8658_SAMPLE_SIZE (3 * sizeof(short))
#define QMI8658_AE_SAMPLE_SIZE ((4+3+1) * sizeof(short) + sizeof(unsigned char))
struct FisImuRawSample
{
	/*! \brief The sample counter of the sample. */
	unsigned char timestamp[3];
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
	unsigned char sampleBuffer[QMI8658_SAMPLE_SIZE + QMI8658_AE_SAMPLE_SIZE];
	/*! \brief Contents of the FIS status 1 register. */
	unsigned char status1;
	//unsigned char status0;
	//unsigned int durT;
};

struct Qmi8658_offsetCalibration
{
	enum Qmi8658_AccUnit accUnit;
	float accOffset[3];
	enum Qmi8658_GyrUnit gyrUnit;
	float gyrOffset[3];
};

struct Qmi8658_sensitivityCalibration
{
	float accSensitivity[3];
	float gyrSensitivity[3];
};

enum Qmi8658_Interrupt
{
	/*! \brief FIS INT1 line. */
	Qmi8658_Int1 = (0 << 6),
	/*! \brief FIS INT2 line. */
	Qmi8658_Int2 = (1 << 6)
};

enum Qmi8658_InterruptState
{
	Qmi8658State_high = (1 << 7), /*!< Interrupt high. */
	Qmi8658State_low  = (0 << 7)  /*!< Interrupt low. */
};

enum Qmi8658_WakeOnMotionThreshold
{
	Qmi8658WomThreshold_high = 128, /*!< High threshold - large motion needed to wake. */
	Qmi8658WomThreshold_low  = 32   /*!< Low threshold - small motion needed to wake. */
};


extern unsigned char Qmi8658_write_reg(unsigned char reg, unsigned char value);
extern unsigned char Qmi8658_read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
extern unsigned char Qmi8658_init(void);
extern void Qmi8658_Config_apply(struct Qmi8658Config const* config);
extern void Qmi8658_enableSensors(unsigned char enableFlags);
extern void Qmi8658_read_acc_xyz(float acc_xyz[3]);
extern void Qmi8658_read_gyro_xyz(float gyro_xyz[3]);
extern void Qmi8658_read_xyz(float acc[3], float gyro[3], unsigned int *tim_count);
extern void Qmi8658_read_xyz_raw(short raw_acc_xyz[3], short raw_gyro_xyz[3], unsigned int *tim_count);
extern void Qmi8658_read_ae(float quat[4], float velocity[3]);
extern unsigned char Qmi8658_readStatus0(void);
extern unsigned char Qmi8658_readStatus1(void);
extern float Qmi8658_readTemp(void);
extern void Qmi8658_enableWakeOnMotion(void);
extern void Qmi8658_disableWakeOnMotion(void);


#endif
