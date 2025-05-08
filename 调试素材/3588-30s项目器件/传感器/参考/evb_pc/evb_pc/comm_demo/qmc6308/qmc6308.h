#ifndef __QMC6308_H
#define __QMC6308_H

#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

/* vendor chip id*/
#define QMC6308_IIC_ADDR				0x2c		// 6308: (0x2c<<1) 6310: (0x1c<<1)
#define QMC6310_IIC_ADDR_U				0x1c		// 6310U
#define QMC6310_IIC_ADDR_N				0x3c		// 6310N

#define QMC6308_CHIP_ID_REG				0x00

/*data output register*/
#define QMC6308_DATA_OUT_X_LSB_REG		0x01
#define QMC6308_DATA_OUT_X_MSB_REG		0x02
#define QMC6308_DATA_OUT_Y_LSB_REG		0x03
#define QMC6308_DATA_OUT_Y_MSB_REG		0x04
#define QMC6308_DATA_OUT_Z_LSB_REG		0x05
#define QMC6308_DATA_OUT_Z_MSB_REG		0x06
/*Status registers */
#define QMC6308_STATUS_REG				0x09
/* configuration registers */
#define QMC6308_CTL_REG_ONE				0x0A  /* Contrl register one */
#define QMC6308_CTL_REG_TWO				0x0B  /* Contrl register two */

/* Magnetic Sensor Operating Mode MODE[1:0]*/
#define QMC6308_SUSPEND_MODE			0x00
#define QMC6308_NORMAL_MODE				0x01
#define QMC6308_SINGLE_MODE				0x02
#define QMC6308_H_PFM_MODE				0x03

/*data output rate OSR2[2:0]*/
#define OUTPUT_DATA_RATE_800HZ 			0x00
#define OUTPUT_DATA_RATE_400HZ 			0x01
#define OUTPUT_DATA_RATE_200HZ 			0x02
#define OUTPUT_DATA_RATE_100HZ 			0x03

/*oversample Ratio  OSR[1]*/
#define OVERSAMPLE_RATE_256				0x01
#define OVERSAMPLE_RATE_128 			0x00

#define SET_RESET_ON 					0x00
#define SET_ONLY_ON 					0x01
#define SET_RESET_OFF 					0x02

#define QMC6308_DEFAULT_DELAY			200

enum{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,

	AXIS_TOTAL
};

typedef struct{
	signed char sign[3];
	unsigned char map[3];
} qmc6308_map;

int qmc6308_write_reg(unsigned char addr, unsigned char data);
int qmc6308_read_block(unsigned char addr, unsigned char *data, unsigned char len);
int qmc6308_init(void);
int qmc6308_read_mag_xyz(float *data);

#endif

