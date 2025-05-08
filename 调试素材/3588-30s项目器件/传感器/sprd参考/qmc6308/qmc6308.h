/****************************************************************************
*   This software is licensed under the terms of the GNU General Public License version 2,
*   as published by the Free Software Foundation, and may be copied, distributed, and
*   modified under those terms.
*   This program is distributed in the hope that it will be useful, but WITHOUT ANY 
*   WARRANTY; without even the implied warranty of MERCHANTABILITY or
*   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
*   for more details.
*
*   Copyright (C) 2012 by QST(Shanghai XiRui Keji) Corporation 
****************************************************************************/

#ifndef __QMC6308_H__
#define __QMC6308_H__

#include <linux/ioctl.h>  /* For IOCTL macros */


#ifndef QMC_IOCTL_WRITE
#define MSENSOR						   0x83
/*IOCTL for HAL*/
#define QMC6308_IOCTL_MSENSOR_ENABLE	_IOW(MSENSOR,0x55,int)
#define QMC6308_IOCTL_OSENSOR_ENABLE	_IOW(MSENSOR,0x56,int)
#define QMC6308_IOCTL_SET_DELAY			_IOW(MSENSOR,0x29,int)
#define QMC6308_IOCTL_GET_DELAY			_IOR(MSENSOR,0x1d,int)
#define QMC6308_IOCTL_GET_DIRECTION		_IOR(MSENSOR,0x44,char)
#define QMC6308_IOCTL_GET_OTPK 			_IOR(MSENSOR,0x45,int[2])

#endif


#define QMC6308_IOCTL_BASE 'm'
/* The following define the IOCTL command values via the ioctl macros */
#define QMC6308_SET_MODE		  			_IOW(QMC6308_IOCTL_BASE, 1, int)
#define QMC6308_SET_RANGE		  			_IOW(QMC6308_IOCTL_BASE, 2, int)
#define QMC6308_READ_MAGN_XYZ	  			_IOR(QMC6308_IOCTL_BASE, 3, char *)
#define QMC6308_SET_ODR         			_IOR(QMC6308_IOCTL_BASE, 4, char)
#define QMC6308_SELF_TEST	   		  		_IOWR(QMC6308_IOCTL_BASE, 5, char *)
#define QMC6308_SET_OVERSAMPLE_RATIO  		_IOWR(QMC6308_IOCTL_BASE, 6, char *)

 
/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/
/* Magnetometer registers */
#define CTL_REG_ONE	0x0a  /* Contrl register one */
#define CTL_REG_TWO	0x0b  /* Contrl register two */

/* Output register start address*/
#define OUT_X_REG		0x01

/*Status registers */
#define STA_REG_ONE    0x09
//#define STA_REG_TWO    0x0c

/*OTP REG*/
//#define OTP_REGCONF1    0X2e
//#define OTP_REGCONF2    0X2f

/* Temperature registers */
#define TEMP_H_REG 		0x08
#define TEMP_L_REG 		0x07

/*different from qmcX983,the ratio register*/
//#define RATIO_REG		0x0b
#define QMC6308_CHIP_ID 0x00


#define QMC_I2C_NAME			"qmcX983"
#define QMC_MISCDEV_NAME		"qmcX983"//"msensor"
#define QMC_SYSCLS_NAME			"compass"
#define QMC_SYSDEV_NAME			"qmcX983"

/* Magnetic Sensor Operating Mode */
#define QMC6308_STANDBY_MODE	0x00
#define QMC6308_CC_MODE			0x01
#define QMC6308_SELFTEST_MODE	0x02
#define QMC6308_RESERVE_MODE	0x03


/* Magnetometer output data rate  */
#define QMC6308_ODR_10		0x00	/* 0.75Hz output data rate */
#define QMC6308_ODR_50		0x01	/* 1.5Hz output data rate */
#define QMC6308_ODR_100		0x02	/* 3Hz output data rate */
#define QMC6308_ODR7_200	0x03	/* 7.5Hz output data rate */


/* Magnetometer full scale  */
#define QMC6308_RNG_2G		0x00
#define QMC6308_RNG_8G		0x01
#define QMC6308_RNG_12G		0x02
#define QMC6308_RNG_20G		0x03

#define RNG_2G		2
#define RNG_8G		8
#define RNG_12G		12
#define RNG_20G		20

/*data output rate HZ*/
#define DATA_OUTPUT_RATE_10HZ 	0x00
#define DATA_OUTPUT_RATE_50HZ 	0x01
#define DATA_OUTPUT_RATE_100HZ 	0x02
#define DATA_OUTPUT_RATE_200HZ 	0x03

/*oversample Ratio */
#define OVERSAMPLE_RATIO_512 	0x00
#define OVERSAMPLE_RATIO_256 	0x01
#define OVERSAMPLE_RATIO_128 	0x02
#define OVERSAMPLE_RATIO_64 	0x03

#ifdef __KERNEL__

struct qmc6308_platform_data{

	char layout;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */


#endif  /* __QMC6308_H__ */
