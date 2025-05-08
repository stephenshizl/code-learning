#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include "conio.h"
#include "USBIOX.H"
#include "USB2UARTSPIIICDLL.h"

enum
{
	USB_NONE,
	USB_I2C,
	USB_SPI,
	USB_SERIAL,
	USB_MAX
};

enum
{
	DEVICE_NONE,
	DEVICE_CH341A,
	DEVICE_STM32F103,
	DEVICE_CP2102,
	DEVICE_MAX,
};

typedef void (*irq_callback)(void);

typedef struct
{
	int				protocol;
	int				device;
	//unsigned char slave;
	BOOL			usb_open;
	UINT			mCh341Index;
	HANDLE			mCh341Handle;	
	UINT			mStm32Index;
	// cp2102 com port
	BOOL			com_open;
	HANDLE			hCom;
	// cp2102 com port 
	irq_callback	irq1_func;

	unsigned char	buf[64];
}usb_device_t;


int get_device_protocol(void);
int usb_open_device(void);
BOOL usb_close_device(void);
int com_open_device(int com_num);
int com_close_device(void);
BOOL i2c_init(int device, int rate);
BOOL spi_init(int device, unsigned int mode);
void spi_config(unsigned int rate, unsigned int mode);

BOOL i2c_write_reg(unsigned char slave, unsigned char addr, unsigned char value);
BOOL i2c_read_reg(unsigned char slave, unsigned char addr, unsigned char *buf, unsigned short len);
BOOL spi_write_reg(unsigned char addr, unsigned char value);
BOOL spi_read_reg(unsigned char addr, unsigned char *buf, unsigned short len);
BOOL device_write_io(unsigned char value);
void device_register_irq(irq_callback irq_fun);

#endif
