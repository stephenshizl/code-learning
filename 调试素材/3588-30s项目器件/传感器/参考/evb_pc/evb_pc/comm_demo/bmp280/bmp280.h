#ifndef BMP280_H
#define BMP280_H

#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

// bmp280
#define BMP280_U16_t unsigned short
#define BMP280_S16_t short
#define BMP280_U32_t unsigned int
#define BMP280_S32_t int
#define BMP280_U64_t unsigned long long
#define BMP280_S64_t long long

#define SHIFT_RIGHT_4_POSITION				 4
#define SHIFT_LEFT_2_POSITION                2
#define SHIFT_LEFT_4_POSITION                4
#define SHIFT_LEFT_5_POSITION                5
#define SHIFT_LEFT_8_POSITION                8
#define SHIFT_LEFT_12_POSITION               12
#define SHIFT_LEFT_16_POSITION               16

#define BMP280_CALIBRATION_DATA_START		0x88 /* BMP280_DIG_T1_LSB_REG */
#define BMP280_CALIBRATION_DATA_LENGTH		24
/* power mode */
#define BMP280_SLEEP_MODE                    0x00
#define BMP280_FORCED_MODE                   0x01
#define BMP280_NORMAL_MODE                   0x03
#define BMP280_CTRLMEAS_REG                  0xF4  /* Ctrl Measure Register */
/* filter */
#define BMP280_FILTERCOEFF_OFF               0x00
#define BMP280_FILTERCOEFF_2                 0x01
#define BMP280_FILTERCOEFF_4                 0x02
#define BMP280_FILTERCOEFF_8                 0x03
#define BMP280_FILTERCOEFF_16                0x04
#define BMP280_CONFIG_REG                    0xF5  /* Configuration Register */
/* oversampling */
#define BMP280_OVERSAMPLING_SKIPPED          0x00
#define BMP280_OVERSAMPLING_1X               0x01
#define BMP280_OVERSAMPLING_2X               0x02
#define BMP280_OVERSAMPLING_4X               0x03
#define BMP280_OVERSAMPLING_8X               0x04
#define BMP280_OVERSAMPLING_16X              0x05
/* data */
#define BMP280_PRESSURE_MSB_REG              0xF7  /* Pressure MSB Register */
#define BMP280_TEMPERATURE_MSB_REG           0xFA  /* Temperature MSB Reg */


/* bmp280 calibration */
struct bmp280_calibration_data {
	BMP280_U16_t dig_T1;
	BMP280_S16_t dig_T2;
	BMP280_S16_t dig_T3;
	BMP280_U16_t dig_P1;
	BMP280_S16_t dig_P2;
	BMP280_S16_t dig_P3;
	BMP280_S16_t dig_P4;
	BMP280_S16_t dig_P5;
	BMP280_S16_t dig_P6;
	BMP280_S16_t dig_P7;
	BMP280_S16_t dig_P8;
	BMP280_S16_t dig_P9;
};

// bmp280
int bmp280_init(void);
void bmp280_calc_press(float *pressure, float *tempearture);

#endif
