#ifndef __QMC5883__H__
#define __QMC5883__H__

#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

#define QMC5883L_LOG					_cprintf

#define QMC5883L_SLAVE_L 0x0c
#define QMC5883L_SLAVE_H 0x0d

// Registers
#define QMC5883L_REG_CONF1 0x09
#define QMC5883L_REG_CONF2 0x0A

// data output rates for 5883L
#define QMC5883L_ODR_10HZ (0x00 << 2)
#define QMC5883L_ODR_50HZ  (0x01 << 2)
#define QMC5883L_ODR_100HZ (0x02 << 2)
#define QMC5883L_ODR_200HZ (0x03 << 2)

// Sensor operation modes
#define QMC5883L_MODE_STANDBY 0x00
#define QMC5883L_MODE_CONTINUOUS 0x01

#define QMC5883L_RNG_2G (0x00 << 4)
#define QMC5883L_RNG_8G (0x01 << 4)

#define QMC5883L_OSR_512 (0x00 << 6)
#define QMC5883L_OSR_256 (0x01 << 6)
#define QMC5883L_OSR_128	(0x10 << 6)
#define QMC5883L_OSR_64	(0x11	<< 6)

#define QMC5883L_RST 0x80

#define QMC5883L_REG_DATA_OUTPUT_X 0x00
#define QMC5883L_REG_STATUS 0x06

#define QMC5883L_REG_ID 0x0D
#define QMC5883_ID_VAL 0xFF


int Qmc5883_WriteReg(unsigned char Reg, unsigned char Val);
int Qmc5883_ReadReg(unsigned char Reg, unsigned char *value, unsigned short len);
unsigned char Qmc5883_InitConfig(void);	
unsigned char Qmc5883_GetData(float *Magnet);

#endif  /* __QMA6981_H__ */
