#ifndef LIS3DH_H
#define LIS3DH_H

#include "usb_device.h"
#include <string.h>
#include <stdio.h>
#include "conio.h"

#define lis3dh_printf					_cprintf

#define LIS3DH_CHIP_ID                   0x33

#define LIS3DH_ACCEL_RANGE_2G            0x00
#define LIS3DH_ACCEL_RANGE_4G            0x01
#define LIS3DH_ACCEL_RANGE_8G            0x02
#define LIS3DH_ACCEL_RANGE_16G           0x03

unsigned char lis3dh_write_reg(unsigned char reg, unsigned char value);
unsigned char lis3dh_read_reg(unsigned char reg, unsigned char* buf, unsigned short len);
unsigned char lis3dh_get_slave(void);
unsigned char lis3dh_init(void);
void lis3dh_read_acc_xyz(float acc[3]);
void lis3dh_read_tempearture(short *temp);

#endif
