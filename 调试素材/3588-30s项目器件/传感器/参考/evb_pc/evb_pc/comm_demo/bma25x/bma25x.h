#ifndef __BMA25X_H
#define __BMA25X_H


#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

#define BMA25X_LOG					_cprintf
#define BMA25X_ERR					_cprintf

#define BMA25X_DEVICE_ID		    0xfa
#define BMA25X_SUCCESS				1
#define BMA25X_ERROR				0

#define BMA25X_REG_ID		    0x00
enum
{
	BMA25X_RANGE_2G,
	BMA25X_RANGE_4G,
	BMA25X_RANGE_8G,
	BMA25X_RANGE_16G,
	BMA25X_RANGE_MAX
};


extern int bma25x_writereg(unsigned char reg_add, unsigned char reg_dat);
extern int bma25x_readreg(unsigned char reg_add, unsigned char *buf, unsigned short num);
extern unsigned char bma25x_get_slave(void);
extern int bma25x_init(void);
extern int bma25x_read_xyz(float *accData);
extern int bma25x_read_raw(int *rawData);
extern void bma25x_irq_hdlr(void);

#endif  /*QMAX981*/
