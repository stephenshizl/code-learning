#ifndef BMI160_H
#define BMI160_H

#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

/** Mask definitions */
#define BMI160_ACCEL_BW_MASK                    0x70
#define BMI160_ACCEL_ODR_MASK                   0x0F
#define BMI160_ACCEL_UNDERSAMPLING_MASK         0x80
#define BMI160_ACCEL_RANGE_MASK                 0x0F
#define BMI160_GYRO_BW_MASK                     0x30
#define BMI160_GYRO_ODR_MASK                    0x0F
#define BMI160_GYRO_RANGE_MSK                   0x07

/** Mask definitions for INT_EN registers */
#define BMI160_ANY_MOTION_X_INT_EN_MASK         0x01
#define BMI160_HIGH_G_X_INT_EN_MASK             0x01
#define BMI160_NO_MOTION_X_INT_EN_MASK          0x01
#define BMI160_ANY_MOTION_Y_INT_EN_MASK         0x02
#define BMI160_HIGH_G_Y_INT_EN_MASK             0x02
#define BMI160_NO_MOTION_Y_INT_EN_MASK          0x02
#define BMI160_ANY_MOTION_Z_INT_EN_MASK         0x04
#define BMI160_HIGH_G_Z_INT_EN_MASK             0x04
#define BMI160_NO_MOTION_Z_INT_EN_MASK          0x04
#define BMI160_SIG_MOTION_INT_EN_MASK           0x07
#define BMI160_ANY_MOTION_ALL_INT_EN_MASK       0x07
#define BMI160_STEP_DETECT_INT_EN_MASK          0x08
#define BMI160_DOUBLE_TAP_INT_EN_MASK           0x10
#define BMI160_SINGLE_TAP_INT_EN_MASK           0x20
#define BMI160_FIFO_FULL_INT_EN_MASK            0x20
#define BMI160_ORIENT_INT_EN_MASK               0x40
#define BMI160_FIFO_WATERMARK_INT_EN_MASK       0x40
#define BMI160_LOW_G_INT_EN_MASK                0x08
#define BMI160_STEP_DETECT_EN_MASK              0x08
#define BMI160_FLAT_INT_EN_MASK                 0x80
#define BMI160_DATA_RDY_INT_EN_MASK             0x10

/** Mask definitions for INT_OUT_CTRL register */
#define BMI160_INT1_EDGE_CTRL_MASK              0x01
#define BMI160_INT1_OUTPUT_MODE_MASK            0x04
#define BMI160_INT1_OUTPUT_TYPE_MASK            0x02
#define BMI160_INT1_OUTPUT_EN_MASK              0x08
#define BMI160_INT2_EDGE_CTRL_MASK              0x10
#define BMI160_INT2_OUTPUT_MODE_MASK            0x40
#define BMI160_INT2_OUTPUT_TYPE_MASK            0x20
#define BMI160_INT2_OUTPUT_EN_MASK              0x80

/** Mask definitions for INT_LATCH register */
#define BMI160_INT1_INPUT_EN_MASK               0x10
#define BMI160_INT2_INPUT_EN_MASK               0x20
#define BMI160_INT_LATCH_MASK                   0x0F

/** Mask definitions for INT_MAP register */
#define BMI160_INT1_LOW_G_MASK                  0x01
#define BMI160_INT1_HIGH_G_MASK                 0x02
#define BMI160_INT1_SLOPE_MASK                  0x04
#define BMI160_INT1_NO_MOTION_MASK              0x08
#define BMI160_INT1_DOUBLE_TAP_MASK             0x10
#define BMI160_INT1_SINGLE_TAP_MASK             0x20
#define BMI160_INT1_FIFO_FULL_MASK              0x20
#define BMI160_INT1_FIFO_WM_MASK                0x40
#define BMI160_INT1_ORIENT_MASK                 0x40
#define BMI160_INT1_FLAT_MASK                   0x80
#define BMI160_INT1_DATA_READY_MASK             0x80
#define BMI160_INT2_LOW_G_MASK                  0x01
#define BMI160_INT1_LOW_STEP_DETECT_MASK        0x01
#define BMI160_INT2_LOW_STEP_DETECT_MASK        0x01
#define BMI160_INT2_HIGH_G_MASK                 0x02
#define BMI160_INT2_FIFO_FULL_MASK              0x02
#define BMI160_INT2_FIFO_WM_MASK    0x04
#define BMI160_INT2_SLOPE_MASK                  0x04
#define BMI160_INT2_DATA_READY_MASK             0x08
#define BMI160_INT2_NO_MOTION_MASK              0x08
#define BMI160_INT2_DOUBLE_TAP_MASK             0x10
#define BMI160_INT2_SINGLE_TAP_MASK             0x20
#define BMI160_INT2_ORIENT_MASK                 0x40
#define BMI160_INT2_FLAT_MASK                   0x80

/** Mask definitions for INT_DATA register */
#define BMI160_TAP_SRC_INT_MASK                 0x08
#define BMI160_LOW_HIGH_SRC_INT_MASK            0x80
#define BMI160_MOTION_SRC_INT_MASK              0x80

/** Mask definitions for INT_MOTION register */
#define BMI160_SLOPE_INT_DUR_MASK               0x03
#define BMI160_NO_MOTION_INT_DUR_MASK           0xFC
#define BMI160_NO_MOTION_SEL_BIT_MASK           0x01

/** Mask definitions for INT_TAP register */
#define BMI160_TAP_DUR_MASK                     0x07
#define BMI160_TAP_SHOCK_DUR_MASK               0x40
#define BMI160_TAP_QUIET_DUR_MASK               0x80
#define BMI160_TAP_THRES_MASK                   0x1F

/** Mask definitions for INT_FLAT register */
#define BMI160_FLAT_THRES_MASK                  0x3F
#define BMI160_FLAT_HOLD_TIME_MASK              0x30
#define BMI160_FLAT_HYST_MASK                   0x07

/** Mask definitions for INT_LOWHIGH register */
#define BMI160_LOW_G_HYST_MASK                  0x03
#define BMI160_LOW_G_LOW_MODE_MASK              0x04
#define BMI160_HIGH_G_HYST_MASK                 0xC0

/** Mask definitions for INT_SIG_MOTION register */
#define BMI160_SIG_MOTION_SEL_MASK              0x02
#define BMI160_SIG_MOTION_SKIP_MASK             0x0C
#define BMI160_SIG_MOTION_PROOF_MASK            0x30

/** Mask definitions for INT_ORIENT register */
#define BMI160_ORIENT_MODE_MASK                 0x03
#define BMI160_ORIENT_BLOCK_MASK                0x0C
#define BMI160_ORIENT_HYST_MASK                 0xF0
#define BMI160_ORIENT_THETA_MASK                0x3F
#define BMI160_ORIENT_UD_ENABLE                 0x40
#define BMI160_AXES_EN_MASK                     0x80

/** Mask definitions for FIFO_CONFIG register */
#define BMI160_FIFO_GYRO      0x80
#define BMI160_FIFO_ACCEL     0x40
#define BMI160_FIFO_AUX       0x20
#define BMI160_FIFO_TAG_INT1      0x08
#define BMI160_FIFO_TAG_INT2      0x04
#define BMI160_FIFO_TIME      0x02
#define BMI160_FIFO_HEADER      0x10
#define BMI160_FIFO_CONFIG_1_MASK               0xFE


/** Mask definitions for STEP_CONF register */
#define BMI160_STEP_COUNT_EN_BIT_MASK           0x08
#define BMI160_STEP_DETECT_MIN_THRES_MASK       0x18
#define BMI160_STEP_DETECT_STEPTIME_MIN_MASK    0x07
#define BMI160_STEP_MIN_BUF_MASK                0x07

/** Mask definition for FIFO Header Data Tag */
#define BMI160_FIFO_TAG_INTR_MASK               0xFC

/** Fifo byte counter mask definitions */
#define BMI160_FIFO_BYTE_COUNTER_MASK           0x07

/** Enable/disable bit value */
#define BMI160_ENABLE                           0x01
#define BMI160_DISABLE                          0x00

/** Latch Duration */
#define BMI160_LATCH_DUR_NONE                   0x00
#define BMI160_LATCH_DUR_312_5_MICRO_SEC        0x01
#define BMI160_LATCH_DUR_625_MICRO_SEC          0x02
#define BMI160_LATCH_DUR_1_25_MILLI_SEC         0x03
#define BMI160_LATCH_DUR_2_5_MILLI_SEC          0x04
#define BMI160_LATCH_DUR_5_MILLI_SEC            0x05
#define BMI160_LATCH_DUR_10_MILLI_SEC           0x06
#define BMI160_LATCH_DUR_20_MILLI_SEC           0x07
#define BMI160_LATCH_DUR_40_MILLI_SEC           0x08
#define BMI160_LATCH_DUR_80_MILLI_SEC           0x09
#define BMI160_LATCH_DUR_160_MILLI_SEC          0x0A
#define BMI160_LATCH_DUR_320_MILLI_SEC          0x0B
#define BMI160_LATCH_DUR_640_MILLI_SEC          0x0C
#define BMI160_LATCH_DUR_1_28_SEC               0x0D
#define BMI160_LATCH_DUR_2_56_SEC               0x0E
#define BMI160_LATCHED                          0x0F

/** BMI160 Register map */
#define BMI160_CHIP_ID_ADDR   0x00
#define BMI160_ERROR_REG_ADDR   0x02
#define BMI160_PMU_STAT_ADDR	0X03
#define BMI160_AUX_DATA_ADDR    0x04
#define BMI160_GYRO_DATA_ADDR   0x0C
#define BMI160_ACCEL_DATA_ADDR    0x12
#define BMI160_STATUS_ADDR    0x1B
#define BMI160_INT_STATUS_ADDR    0x1C
#define BMI160_FIFO_LENGTH_ADDR   0x22
#define BMI160_FIFO_DATA_ADDR   0x24
#define BMI160_ACCEL_CONFIG_ADDR  0x40
#define BMI160_ACCEL_RANGE_ADDR   0x41
#define BMI160_GYRO_CONFIG_ADDR   0x42
#define BMI160_GYRO_RANGE_ADDR    0x43
#define BMI160_AUX_ODR_ADDR   0x44
#define BMI160_FIFO_DOWN_ADDR           0x45
#define BMI160_FIFO_CONFIG_0_ADDR       0x46
#define BMI160_FIFO_CONFIG_1_ADDR       0x47
#define BMI160_AUX_IF_0_ADDR    0x4B
#define BMI160_AUX_IF_1_ADDR    0x4C
#define BMI160_AUX_IF_2_ADDR    0x4D
#define BMI160_AUX_IF_3_ADDR    0x4E
#define BMI160_AUX_IF_4_ADDR    0x4F
#define BMI160_INT_ENABLE_0_ADDR         0x50
#define BMI160_INT_ENABLE_1_ADDR         0x51
#define BMI160_INT_ENABLE_2_ADDR         0x52
#define BMI160_INT_OUT_CTRL_ADDR         0x53
#define BMI160_INT_LATCH_ADDR            0x54
#define BMI160_INT_MAP_0_ADDR            0x55
#define BMI160_INT_MAP_1_ADDR            0x56
#define BMI160_INT_MAP_2_ADDR            0x57
#define BMI160_INT_DATA_0_ADDR           0x58
#define BMI160_INT_DATA_1_ADDR           0x59
#define BMI160_INT_LOWHIGH_0_ADDR        0x5A
#define BMI160_INT_LOWHIGH_1_ADDR        0x5B
#define BMI160_INT_LOWHIGH_2_ADDR        0x5C
#define BMI160_INT_LOWHIGH_3_ADDR        0x5D
#define BMI160_INT_LOWHIGH_4_ADDR        0x5E
#define BMI160_INT_MOTION_0_ADDR         0x5F
#define BMI160_INT_MOTION_1_ADDR         0x60
#define BMI160_INT_MOTION_2_ADDR         0x61
#define BMI160_INT_MOTION_3_ADDR         0x62
#define BMI160_INT_TAP_0_ADDR            0x63
#define BMI160_INT_TAP_1_ADDR            0x64
#define BMI160_INT_ORIENT_0_ADDR         0x65
#define BMI160_INT_ORIENT_1_ADDR         0x66
#define BMI160_INT_FLAT_0_ADDR           0x67
#define BMI160_INT_FLAT_1_ADDR           0x68
#define BMI160_FOC_CONF_ADDR             0x69
#define BMI160_CONF_ADDR                 0x6A

#define BMI160_IF_CONF_ADDR				0x6B
#define BMI160_SELF_TEST_ADDR			0x6D
#define BMI160_OFFSET_ADDR				0x71
#define BMI160_OFFSET_CONF_ADDR			0x77
#define BMI160_INT_STEP_CNT_0_ADDR		0x78
#define BMI160_INT_STEP_CONFIG_0_ADDR	0x7A
#define BMI160_INT_STEP_CONFIG_1_ADDR	0x7B
#define BMI160_COMMAND_REG_ADDR			0x7E
#define BMI160_SPI_COMM_TEST_ADDR		0x7F
#define BMI160_INTL_PULLUP_CONF_ADDR	0x85

/** Error code definitions */
#define BMI160_OK                         0
#define BMI160_E_NULL_PTR                 -1
#define BMI160_E_COM_FAIL                 -2
#define BMI160_E_DEV_NOT_FOUND            -3
#define BMI160_E_OUT_OF_RANGE             -4
#define BMI160_E_INVALID_INPUT            -5
#define BMI160_E_ACCEL_ODR_BW_INVALID   -6
#define BMI160_E_GYRO_ODR_BW_INVALID    -7
#define BMI160_E_LWP_PRE_FLTR_INT_INVALID -8
#define BMI160_E_LWP_PRE_FLTR_INVALID   -9
#define BMI160_E_AUX_NOT_FOUND      -10
#define BMI160_FOC_FAILURE      -11
#define BMI160_ERR_CHOOSE      -12

/**\name API warning codes */
#define BMI160_W_GYRO_SELF_TEST_FAIL  1
#define BMI160_W_ACCEl_SELF_TEST_FAIL 2

/** BMI160 unique chip identifier */
#define BMI160_CHIP_ID                   0xD1

/** Soft reset command */
#define BMI160_SOFT_RESET_CMD            0xb6
#define BMI160_SOFT_RESET_DELAY_MS       15
/** Start FOC command */
#define BMI160_START_FOC_CMD            0x03
/** NVM backup enabling command */
#define BMI160_NVM_BACKUP_EN    0xA0

/* Delay in ms settings */
#define BMI160_ACCEL_DELAY_MS            5
#define BMI160_GYRO_DELAY_MS             81
#define BMI160_ONE_MS_DELAY              1
#define BMI160_AUX_COM_DELAY     10
#define BMI160_GYRO_SELF_TEST_DELAY  20
#define BMI160_ACCEL_SELF_TEST_DELAY   50

/** Self test configurations */
#define BMI160_ACCEL_SELF_TEST_CONFIG   0x2C
#define BMI160_ACCEL_SELF_TEST_POSITIVE_EN  0x0D
#define BMI160_ACCEL_SELF_TEST_NEGATIVE_EN  0x09
#define BMI160_ACCEL_SELF_TEST_LIMIT    8192

/** Power mode settings */
/* Accel power mode */
#define BMI160_ACCEL_NORMAL_MODE         0x11
#define BMI160_ACCEL_LOWPOWER_MODE       0x12
#define BMI160_ACCEL_SUSPEND_MODE        0x10

/* Gyro power mode */
#define BMI160_GYRO_SUSPEND_MODE         0x14
#define BMI160_GYRO_NORMAL_MODE          0x15
#define BMI160_GYRO_FASTSTARTUP_MODE     0x17

/* Aux power mode */
#define BMI160_AUX_SUSPEND_MODE   0x18
#define BMI160_AUX_NORMAL_MODE    0x19
#define BMI160_AUX_LOWPOWER_MODE  0x1A

/** Range settings */
/* Accel Range */
#define BMI160_ACCEL_RANGE_2G            0x03
#define BMI160_ACCEL_RANGE_4G            0x05
#define BMI160_ACCEL_RANGE_8G            0x08
#define BMI160_ACCEL_RANGE_16G           0x0C

/* Gyro Range */
#define BMI160_FS_125_LSB               2624
#define BMI160_FS_250_LSB               1312
#define BMI160_FS_500_LSB               656
#define BMI160_FS_1000_LSB              328
#define BMI160_FS_2000_LSB              164

#define BMI160_GYRO_RANGE_2000_DPS       0x00
#define BMI160_GYRO_RANGE_1000_DPS       0x01
#define BMI160_GYRO_RANGE_500_DPS        0x02
#define BMI160_GYRO_RANGE_250_DPS        0x03
#define BMI160_GYRO_RANGE_125_DPS        0x04

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMI160_ACCEL_BW_OSR4_AVG1        0x00
#define BMI160_ACCEL_BW_OSR2_AVG2        0x01
#define BMI160_ACCEL_BW_NORMAL_AVG4      0x02
#define BMI160_ACCEL_BW_RES_AVG8         0x03
#define BMI160_ACCEL_BW_RES_AVG16        0x04
#define BMI160_ACCEL_BW_RES_AVG32        0x05
#define BMI160_ACCEL_BW_RES_AVG64        0x06
#define BMI160_ACCEL_BW_RES_AVG128       0x07

#define BMI160_GYRO_BW_OSR4_MODE         0x00
#define BMI160_GYRO_BW_OSR2_MODE         0x01
#define BMI160_GYRO_BW_NORMAL_MODE       0x02

/* Output Data Rate settings */
/* Accel Output data rate */
#define BMI160_ACCEL_ODR_RESERVED        0x00
#define BMI160_ACCEL_ODR_0_78HZ          0x01
#define BMI160_ACCEL_ODR_1_56HZ          0x02
#define BMI160_ACCEL_ODR_3_12HZ          0x03
#define BMI160_ACCEL_ODR_6_25HZ          0x04
#define BMI160_ACCEL_ODR_12_5HZ          0x05
#define BMI160_ACCEL_ODR_25HZ            0x06
#define BMI160_ACCEL_ODR_50HZ            0x07
#define BMI160_ACCEL_ODR_100HZ           0x08
#define BMI160_ACCEL_ODR_200HZ           0x09
#define BMI160_ACCEL_ODR_400HZ           0x0A
#define BMI160_ACCEL_ODR_800HZ           0x0B
#define BMI160_ACCEL_ODR_1600HZ          0x0C
#define BMI160_ACCEL_ODR_RESERVED0       0x0D
#define BMI160_ACCEL_ODR_RESERVED1       0x0E
#define BMI160_ACCEL_ODR_RESERVED2       0x0F

/* Gyro Output data rate */
#define BMI160_GYRO_ODR_RESERVED         0x00
#define BMI160_GYRO_ODR_25HZ             0x06
#define BMI160_GYRO_ODR_50HZ             0x07
#define BMI160_GYRO_ODR_100HZ            0x08
#define BMI160_GYRO_ODR_200HZ            0x09
#define BMI160_GYRO_ODR_400HZ            0x0A
#define BMI160_GYRO_ODR_800HZ            0x0B
#define BMI160_GYRO_ODR_1600HZ           0x0C
#define BMI160_GYRO_ODR_3200HZ           0x0D

/* Auxiliary sensor Output data rate */
#define BMI160_AUX_ODR_RESERVED        0x00
#define BMI160_AUX_ODR_0_78HZ          0x01
#define BMI160_AUX_ODR_1_56HZ          0x02
#define BMI160_AUX_ODR_3_12HZ          0x03
#define BMI160_AUX_ODR_6_25HZ          0x04
#define BMI160_AUX_ODR_12_5HZ          0x05
#define BMI160_AUX_ODR_25HZ            0x06
#define BMI160_AUX_ODR_50HZ            0x07
#define BMI160_AUX_ODR_100HZ           0x08
#define BMI160_AUX_ODR_200HZ           0x09
#define BMI160_AUX_ODR_400HZ           0x0A
#define BMI160_AUX_ODR_800HZ           0x0B

/* Maximum limits definition */
#define BMI160_ACCEL_ODR_MAX             15
#define BMI160_ACCEL_BW_MAX              2
#define BMI160_ACCEL_RANGE_MAX           12
#define BMI160_GYRO_ODR_MAX              13
#define BMI160_GYRO_BW_MAX               2
#define BMI160_GYRO_RANGE_MAX            4

/** FIFO_CONFIG Definitions */
#define BMI160_FIFO_TIME_ENABLE          0x02
#define BMI160_FIFO_TAG_INT2_ENABLE      0x04
#define BMI160_FIFO_TAG_INT1_ENABLE      0x08
#define BMI160_FIFO_HEAD_ENABLE          0x10
#define BMI160_FIFO_M_ENABLE             0x20
#define BMI160_FIFO_A_ENABLE             0x40
#define BMI160_FIFO_M_A_ENABLE           0x60
#define BMI160_FIFO_G_ENABLE             0x80
#define BMI160_FIFO_M_G_ENABLE           0xA0
#define BMI160_FIFO_G_A_ENABLE           0xC0
#define BMI160_FIFO_M_G_A_ENABLE         0xE0


/* Accel, gyro and aux. sensor length and also their combined
 * length definitions in FIFO */
#define BMI160_FIFO_G_LENGTH             6
#define BMI160_FIFO_A_LENGTH             6
#define BMI160_FIFO_M_LENGTH             8
#define BMI160_FIFO_GA_LENGTH            12
#define BMI160_FIFO_MA_LENGTH            14
#define BMI160_FIFO_MG_LENGTH            14
#define BMI160_FIFO_MGA_LENGTH           20


/** FIFO Header Data definitions */
#define BMI160_FIFO_HEAD_SKIP_FRAME      0x40
#define BMI160_FIFO_HEAD_SENSOR_TIME     0x44
#define BMI160_FIFO_HEAD_INPUT_CONFIG    0x48
#define BMI160_FIFO_HEAD_OVER_READ       0x80
#define BMI160_FIFO_HEAD_A               0x84
#define BMI160_FIFO_HEAD_G               0x88
#define BMI160_FIFO_HEAD_G_A             0x8C
#define BMI160_FIFO_HEAD_M               0x90
#define BMI160_FIFO_HEAD_M_A             0x94
#define BMI160_FIFO_HEAD_M_G             0x98
#define BMI160_FIFO_HEAD_M_G_A           0x9C


/** FIFO sensor time length definitions */
#define BMI160_SENSOR_TIME_LENGTH        3


/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
#define  BMI160_ACCEL_FIFO_DOWN_ZERO     0x00
#define  BMI160_ACCEL_FIFO_DOWN_ONE      0x10
#define  BMI160_ACCEL_FIFO_DOWN_TWO      0x20
#define  BMI160_ACCEL_FIFO_DOWN_THREE    0x30
#define  BMI160_ACCEL_FIFO_DOWN_FOUR     0x40
#define  BMI160_ACCEL_FIFO_DOWN_FIVE     0x50
#define  BMI160_ACCEL_FIFO_DOWN_SIX      0x60
#define  BMI160_ACCEL_FIFO_DOWN_SEVEN    0x70

/* Gyro fifo down-smapling values*/
#define  BMI160_GYRO_FIFO_DOWN_ZERO      0x00
#define  BMI160_GYRO_FIFO_DOWN_ONE       0x01
#define  BMI160_GYRO_FIFO_DOWN_TWO       0x02
#define  BMI160_GYRO_FIFO_DOWN_THREE     0x03
#define  BMI160_GYRO_FIFO_DOWN_FOUR      0x04
#define  BMI160_GYRO_FIFO_DOWN_FIVE      0x05
#define  BMI160_GYRO_FIFO_DOWN_SIX       0x06
#define  BMI160_GYRO_FIFO_DOWN_SEVEN     0x07

/* Accel Fifo filter enable*/
#define  BMI160_ACCEL_FIFO_FILT_EN       0x80

/* Gyro Fifo filter enable*/
#define  BMI160_GYRO_FIFO_FILT_EN        0x08

/** Definitions to check validity of FIFO frames */
#define FIFO_CONFIG_MSB_CHECK            0x80
#define FIFO_CONFIG_LSB_CHECK            0x00

/*! BMI160 accel FOC configurations */
#define BMI160_FOC_ACCEL_DISABLED        0x00
#define BMI160_FOC_ACCEL_POSITIVE_G      0x01
#define BMI160_FOC_ACCEL_NEGATIVE_G      0x02
#define BMI160_FOC_ACCEL_0G              0x03

/** Array Parameter DefinItions */
#define BMI160_SENSOR_TIME_LSB_BYTE      0
#define BMI160_SENSOR_TIME_XLSB_BYTE     1
#define BMI160_SENSOR_TIME_MSB_BYTE      2


/** Interface settings */
#define BMI160_SPI_INTF                  1
#define BMI160_I2C_INTF                  0
#define BMI160_SPI_RD_MASK               0x80
#define BMI160_SPI_WR_MASK               0x7F

/* Sensor & time select definition*/
#define BMI160_ACCEL_SEL    0x01
#define BMI160_GYRO_SEL     0x02
#define BMI160_TIME_SEL     0x04

/* Sensor select mask*/
#define BMI160_SEN_SEL_MASK   0x07

/* Error code mask */
#define BMI160_ERR_REG_MASK   0x0F

/* BMI160 I2C address */
#define BMI160_I2C_ADDR                 0x68
#define BMI160_I2C_ADDR_2                 0x69

/* BMI160 secondary IF address */
#define BMI160_AUX_BMM150_I2C_ADDR    0x10

/** BMI160 Length definitions */
#define BMI160_ONE                       1
#define BMI160_TWO                       2
#define BMI160_THREE                     3
#define BMI160_FOUR                      4
#define BMI160_FIVE                      5

/** BMI160 fifo level Margin */
#define BMI160_FIFO_LEVEL_MARGIN         16

/** BMI160 fifo flush Command */
#define BMI160_FIFO_FLUSH_VALUE          0xB0

/** BMI160 offset values for xyz axes of accel */
#define BMI160_ACCEL_MIN_OFFSET         -128
#define BMI160_ACCEL_MAX_OFFSET         127

/** BMI160 offset values for xyz axes of gyro */
#define BMI160_GYRO_MIN_OFFSET         -512
#define BMI160_GYRO_MAX_OFFSET         511

/** BMI160 fifo full interrupt position and mask */
#define BMI160_FIFO_FULL_INT_POS  5
#define BMI160_FIFO_FULL_INT_MSK  0x20
#define BMI160_FIFO_WTM_INT_POS   6
#define BMI160_FIFO_WTM_INT_MSK   0x40

#define BMI160_FIFO_FULL_INT_PIN1_POS 5
#define BMI160_FIFO_FULL_INT_PIN1_MSK 0x20
#define BMI160_FIFO_FULL_INT_PIN2_POS 1
#define BMI160_FIFO_FULL_INT_PIN2_MSK 0x02

#define BMI160_FIFO_WTM_INT_PIN1_POS  6
#define BMI160_FIFO_WTM_INT_PIN1_MSK  0x40
#define BMI160_FIFO_WTM_INT_PIN2_POS  2
#define BMI160_FIFO_WTM_INT_PIN2_MSK  0x04

#define BMI160_MANUAL_MODE_EN_POS 7
#define BMI160_MANUAL_MODE_EN_MSK 0x80
#define BMI160_AUX_READ_BURST_POS 0
#define BMI160_AUX_READ_BURST_MSK 0x03

#define BMI160_GYRO_SELF_TEST_POS 4
#define BMI160_GYRO_SELF_TEST_MSK 0x10
#define BMI160_GYRO_SELF_TEST_STATUS_POS  1
#define BMI160_GYRO_SELF_TEST_STATUS_MSK  0x02

#define BMI160_GYRO_FOC_EN_POS  6
#define BMI160_GYRO_FOC_EN_MSK  0x40

#define BMI160_ACCEL_FOC_X_CONF_POS 4
#define BMI160_ACCEL_FOC_X_CONF_MSK 0x30

#define BMI160_ACCEL_FOC_Y_CONF_POS 2
#define BMI160_ACCEL_FOC_Y_CONF_MSK 0x0C

#define BMI160_ACCEL_FOC_Z_CONF_MSK 0x03

#define BMI160_FOC_STATUS_POS 3
#define BMI160_FOC_STATUS_MSK 0x08

#define BMI160_GYRO_OFFSET_X_MSK  0x03

#define BMI160_GYRO_OFFSET_Y_POS  2
#define BMI160_GYRO_OFFSET_Y_MSK  0x0C

#define BMI160_GYRO_OFFSET_Z_POS  4
#define BMI160_GYRO_OFFSET_Z_MSK  0x30

#define BMI160_GYRO_OFFSET_EN_POS 7
#define BMI160_GYRO_OFFSET_EN_MSK 0x80

#define BMI160_ACCEL_OFFSET_EN_POS  6
#define BMI160_ACCEL_OFFSET_EN_MSK  0x40


#define BMI160_GYRO_OFFSET_POS          8
#define BMI160_GYRO_OFFSET_MSK          0x0300

#define BMI160_NVM_UPDATE_POS         1
#define BMI160_NVM_UPDATE_MSK         0x02

#define BMI160_NVM_STATUS_POS         4
#define BMI160_NVM_STATUS_MSK         0x10


/* BIT SLICE GET AND SET FUNCTIONS */
#define  BMI160_GET_BITS(regvar, bitname)\
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMI160_SET_BITS(regvar, bitname, val)\
    ((regvar & ~bitname##_MSK) | \
    ((val<<bitname##_POS)&bitname##_MSK))

#define BMI160_SET_BITS_POS_0(reg_data, bitname, data) \
        ((reg_data & ~(bitname##_MSK)) | \
        (data & bitname##_MSK))

#define BMI160_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name UTILITY MACROS */
#define BMI160_SET_LOW_BYTE     UINT16_C(0x00FF)
#define BMI160_SET_HIGH_BYTE    UINT16_C(0xFF00)

#define BMI160_GET_LSB(var) (uint8_t)(var & BMI160_SET_LOW_BYTE)
#define BMI160_GET_MSB(var) (uint8_t)((var & BMI160_SET_HIGH_BYTE) >> 8)


unsigned char bmi160_write_reg(unsigned char reg, unsigned char value);
unsigned char bmi160_read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
unsigned char bmi160_init(void);
void bmi160_read_xyz(float acc[3], float gyr[3]);

#endif
