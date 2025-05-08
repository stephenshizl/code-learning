// comm_demo.cpp : 定义应用程序的类行为。
//

#include "stdafx.h"
#include "usb_device.h"

#define USB_DEVICE_LOG		_cprintf

static usb_device_t device_info = 
{
	USB_I2C,
	DEVICE_CH341A,

	FALSE,
	0,
	INVALID_HANDLE_VALUE,
	0,

	FALSE,
	INVALID_HANDLE_VALUE,

	NULL,

	{0,}
};

int get_device_protocol(void)
{
	return device_info.protocol;
}

int usb_open_device(void)
{	
	if(device_info.usb_open)
	{	
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
			return DEVICE_CH341A;
		else
			return DEVICE_STM32F103;
	}
	else
	{
		if(device_info.mCh341Handle == INVALID_HANDLE_VALUE)
		{
			device_info.mCh341Index = 0;
			device_info.mCh341Handle = USBIO_OpenDevice(device_info.mCh341Index);
		}
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
		{
			USB_DEVICE_LOG("ch341 open OK!\n");
			device_info.usb_open = TRUE;
			return DEVICE_CH341A;			
		}
		else
		{
			device_info.mStm32Index = 0;
			device_info.usb_open = OpenUsb(device_info.mStm32Index);
			if(device_info.usb_open)
			{		
				USB_DEVICE_LOG("stm32 open OK!\n");
				return DEVICE_STM32F103;
			}
		}
	}

	return DEVICE_NONE;
}

BOOL usb_close_device(void)
{
	if(device_info.usb_open)
	{
		if(device_info.mCh341Handle!=INVALID_HANDLE_VALUE)
		{
			USBIO_CloseDevice(device_info.mCh341Index);
			device_info.mCh341Handle=INVALID_HANDLE_VALUE;
			device_info.usb_open = FALSE;			
			device_info.device = DEVICE_NONE;
			USB_DEVICE_LOG("ch341 close OK!\n");
		}
		else
		{
			CloseUSB(device_info.mStm32Index);
			device_info.usb_open = FALSE;
			device_info.device = DEVICE_NONE;
			USB_DEVICE_LOG("stm32 close OK!\n");
		}
	}

	return TRUE;
}


int com_open_device(int com_num)
{
	if(device_info.com_open)
	{	
		if(device_info.hCom != INVALID_HANDLE_VALUE)
			return DEVICE_CP2102;
		else
			return DEVICE_NONE;
	}
	else
	{
		DWORD			len=0;
		unsigned char	buf_wr[32];
		CString			port_name;
		COMMTIMEOUTS	timeouts;
		DCB				dcb;
		BOOL			com_status;
		int				retry_count = 0;

		if(device_info.hCom == INVALID_HANDLE_VALUE)
		{		
			port_name.Empty();
			port_name.Format(L"\\\\.\\COM%d", com_num);
			device_info.hCom = CreateFile(port_name.GetString(), GENERIC_READ|GENERIC_WRITE,
											0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		}
		if(device_info.hCom != INVALID_HANDLE_VALUE)
		{	
			USB_DEVICE_LOG("Com%d open Success\n", com_num);
			SetupComm(device_info.hCom, 1024, 1024);
			GetCommState(device_info.hCom, &dcb);
			dcb.BaudRate = 9600;	//9600;
			dcb.ByteSize = 8;
			dcb.Parity = NOPARITY;
			dcb.StopBits = ONESTOPBIT;
			if(!SetCommState(device_info.hCom, &dcb))
			{
				USB_DEVICE_LOG("SetCommState fail\n");
				AfxMessageBox(_T("SetCommState fail"), MB_OK, MB_ICONINFORMATION);
				return DEVICE_NONE;
			}
			timeouts.ReadIntervalTimeout = 300;  
			timeouts.ReadTotalTimeoutMultiplier = 5;  
			timeouts.ReadTotalTimeoutConstant = 3;  
			timeouts.WriteTotalTimeoutMultiplier = 300;  
			timeouts.WriteTotalTimeoutConstant = 3;  
			if(!SetCommTimeouts(device_info.hCom, &timeouts))
			{
				USB_DEVICE_LOG("set com time-out error! \n");
				AfxMessageBox(_T("set com time-out fail"), MB_OK, MB_ICONINFORMATION);
				return DEVICE_NONE;
			}
			//EscapeCommFunction(hCom, CLRDTR);
			Sleep(20);
			PurgeComm(device_info.hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

			memset(buf_wr, 0, sizeof(buf_wr));
			buf_wr[0] = 0x49;
			buf_wr[1] = 0x50;
			com_status = WriteFile(device_info.hCom, buf_wr, 2, &len, NULL);			
			memset(buf_wr, 0, sizeof(buf_wr));
			if(com_status && (len==2))
			{
				retry_count = 0;
				while(retry_count<0xff)
				{
					com_status = ReadFile(device_info.hCom, buf_wr, 1, &len, NULL);
					if(com_status && (len==1))
					{
						USB_DEVICE_LOG("read com_status=%d, data=0x%x \n",com_status, buf_wr[0]);
						break;
					}
					Sleep(2);
					retry_count++;
				}
				if(buf_wr[0] == 0x40)
				{
					USB_DEVICE_LOG("CP2102 com to i2c controler communication OK!\n");
				}
				else
				{
					USB_DEVICE_LOG("com to i2c controler communication Fail!\n");
					//return DEVICE_NONE;
				}
			}
			else
			{
				USB_DEVICE_LOG("WriteFile error \n");
				return DEVICE_NONE;
			}

			memset(buf_wr, 0, sizeof(buf_wr));
			buf_wr[0] = 0x57;
			buf_wr[1] = 0x0;
			buf_wr[2] = 0x30;
			buf_wr[3] = 0x1;
			buf_wr[4] = 0x00;
			buf_wr[5] = 0x50;
			com_status = WriteFile(device_info.hCom, buf_wr, 6, &len, NULL);  ///strlen(buf_wr) 这个会有问题
			Sleep(100);

			//here we change the baut rate to 115200
			SetupComm(device_info.hCom, 1024, 1024);
			GetCommState(device_info.hCom, &dcb);

			dcb.BaudRate = 115200;
			dcb.ByteSize = 8;
			dcb.Parity = NOPARITY;
			dcb.StopBits = ONESTOPBIT;
			if(!SetCommState(device_info.hCom, &dcb))
				USB_DEVICE_LOG("串口参数设置失败\n");
			
			timeouts.ReadIntervalTimeout = 1; 
			timeouts.ReadTotalTimeoutMultiplier = 2; 
			timeouts.ReadTotalTimeoutConstant = 100; 
			timeouts.WriteTotalTimeoutConstant = 100; 
			timeouts.WriteTotalTimeoutMultiplier = 2; 

			if(!SetCommTimeouts(device_info.hCom, &timeouts))
			{
				USB_DEVICE_LOG("Set Comm Time out error \n");
				return DEVICE_NONE;
			}

			Sleep(100);
			PurgeComm(device_info.hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
			memset(buf_wr, 0, sizeof(buf_wr));
			buf_wr[0] = 0x49;
			buf_wr[1] = 0x50;
			WriteFile(device_info.hCom, buf_wr, 2, &len, NULL);

			Sleep(20);
		// we can get GPIO status 0x40
			retry_count = 0x00ff;
			while(retry_count--)
			{
				com_status = ReadFile(device_info.hCom, buf_wr, 1, &len, NULL);
				if(com_status) //收到数据   
				{
					if(buf_wr[0] == 0x40)
					{
						USB_DEVICE_LOG("CP2102 com to i2c controler communication OK!\n");
						break;
					}
					else
					{
						USB_DEVICE_LOG("com to i2c controler communication Fail!\n");
						return DEVICE_NONE;
					}		
				}
			}
			PurgeComm(device_info.hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

			device_info.device = DEVICE_CP2102;
			device_info.com_open = TRUE;
			return DEVICE_CP2102;			
		}
		else
		{
			AfxMessageBox(_T("open com fail"), MB_OK, MB_ICONINFORMATION);
			device_info.device = DEVICE_NONE;
			return DEVICE_NONE;
		}
	}

	return DEVICE_NONE;
}


int com_close_device(void)
{
	BOOL com_status = FALSE;
	int retry = 0;

	if(device_info.com_open)
	{
		if(device_info.hCom)
		{
			while((com_status == FALSE) && (retry < 100))
			{
				com_status = CloseHandle(device_info.hCom);
				if(com_status)
				{
					device_info.hCom = INVALID_HANDLE_VALUE;
					device_info.com_open = FALSE;
					device_info.device = DEVICE_NONE;
					USB_DEVICE_LOG("com close OK!\n");
				}
				Sleep(2);
			}
		}
	}
	return 1;
}


BOOL i2c_init(int device, int rate)
{
	BOOL status = FALSE;

	if(device == DEVICE_CH341A)
	{	
		if(device_info.mCh341Handle)
		{
			USBIO_CloseDevice(device_info.mCh341Index);
			device_info.mCh341Handle = INVALID_HANDLE_VALUE;
		}
		if(device_info.mCh341Handle==INVALID_HANDLE_VALUE)
		{
			device_info.mCh341Handle = USBIO_OpenDevice(device_info.mCh341Index);
		}
		USB_DEVICE_LOG("mCh341Handle = %d \n", (int)device_info.mCh341Handle);
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
		{
			device_info.usb_open = TRUE;
			//0x81-100k 0x82-400k 0x83-750k
			if(rate == 20*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x80);
			else if(rate == 100*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x81);
			else if(rate == 400*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x82);
			else if(rate == 750*1000)
				USBIO_SetStream(device_info.mCh341Index, 0x83);
			else
				USBIO_SetStream(device_info.mCh341Index, 0x81);
			// 设置中断，INT脚上升沿，执行回调函数。
			//USBIO_SetIntRoutine(device_info.mCh341Index,ch341_int_callback);
		}
		else
		{
			device_info.usb_open = FALSE;
			USB_DEVICE_LOG("CH341A open fail!\n");
		}
	}
	else if(device == DEVICE_STM32F103)
	{
		if(device_info.usb_open == FALSE)
		{
			status = OpenUsb(device_info.mStm32Index);
			device_info.usb_open = status;
		}
		USB_DEVICE_LOG("stm32f103 open status = %d \n", status);
		if(device_info.usb_open == TRUE)
		{
			device_info.usb_open = TRUE;
			if(rate == 1*1000)
				ConfigIICParam(IIC_Rate_1K, device_info.mStm32Index);
			else if(rate == 10*1000)
				ConfigIICParam(IIC_Rate_10K, device_info.mStm32Index);
			else if(rate == 100*1000)
				ConfigIICParam(IIC_Rate_100K, device_info.mStm32Index);
			else if(rate == 400*1000)
				ConfigIICParam(IIC_Rate_400K, device_info.mStm32Index);
		}
		else
		{
			device_info.usb_open = FALSE;
			USB_DEVICE_LOG("STM32F103 open fail!\n");
		}
	}
	device_info.protocol = USB_I2C;
	device_info.device = device;
	memset(device_info.buf, 0, sizeof(device_info.buf));
	
	return device_info.usb_open;
}

BOOL spi_init(int device, unsigned int mode)
{
	BOOL status = FALSE;

	if(device == DEVICE_CH341A)
	{
		if(device_info.mCh341Handle)
		{
			USBIO_CloseDevice(device_info.mCh341Index);
			device_info.mCh341Handle = INVALID_HANDLE_VALUE;
		}
		if(device_info.mCh341Handle==INVALID_HANDLE_VALUE)
		{
			device_info.mCh341Handle = USBIO_OpenDevice(device_info.mCh341Index);
		}
		USB_DEVICE_LOG("mCh341Handle = %d \n", (int)device_info.mCh341Handle);
		if(device_info.mCh341Handle != INVALID_HANDLE_VALUE)
		{
			device_info.usb_open = TRUE;
			USBIO_SetStream(device_info.mCh341Index,0x82);
			USBIO_Set_D5_D0(device_info.mCh341Index,0xff,0x00); 	// add by yangzhiqiang
		}
		else
		{
			device_info.usb_open = FALSE;
			USB_DEVICE_LOG("CH341A open fail!\n");
		}
	}
	else if(device == DEVICE_STM32F103)
	{
		if(device_info.usb_open == FALSE)
		{
			status = OpenUsb(device_info.mStm32Index);
			device_info.usb_open = status;
		}
		USB_DEVICE_LOG("stm32f103 open status = %d \n", status);
		if(device_info.usb_open == TRUE)
		{
			device_info.usb_open = TRUE;
			ConfigSPIParam(SPI_Rate_4_5M, SPI_MSB, mode, device_info.mStm32Index);
		}
		else
		{
			device_info.usb_open = FALSE;
			USB_DEVICE_LOG("STM32F103 open fail!\n");
		}
	}
	device_info.protocol = USB_SPI;
	device_info.device = device;
	memset(device_info.buf, 0, sizeof(device_info.buf));
	
	return device_info.usb_open;
}

void spi_config(unsigned int rate, unsigned int mode)
{
	if(device_info.device == DEVICE_STM32F103)
	{
		if(device_info.usb_open == TRUE)
		{
			if(rate < 3*1000*1000)
				ConfigSPIParam(SPI_Rate_2_25M, SPI_MSB, mode, device_info.mStm32Index);
			else if(rate < 5*1000*1000)
				ConfigSPIParam(SPI_Rate_4_5M, SPI_MSB, mode, device_info.mStm32Index);
			else if(rate < 10*1000*1000)
				ConfigSPIParam(SPI_Rate_9M, SPI_MSB, mode, device_info.mStm32Index);
			else
				ConfigSPIParam(SPI_Rate_18M, SPI_MSB, mode, device_info.mStm32Index);
		}
		else
		{
			device_info.usb_open = FALSE;
			USB_DEVICE_LOG("STM32F103 not open!\n");
		}
	}
}

BOOL i2c_write_reg(unsigned char slave, unsigned char addr, unsigned char value)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		status = USBIO_WriteI2C(device_info.mCh341Index, slave, addr, value);
		//Sleep(2);	// add by yangzhiqiang 0819
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = slave<<1;
		device_info.buf[1] = addr;
		device_info.buf[2] = value;
		status = IICSendData(TRUE, device_info.buf, 3, 1, device_info.mStm32Index);
	}
	else if(device_info.device == DEVICE_CP2102)
	{
		DWORD nWrite;
		int retry_count;

		//PurgeComm(device_info.hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
		//Sleep(1);

		device_info.buf[0] = 83;	  // 'S'
		device_info.buf[1] = slave<<1;
		device_info.buf[2] = 2;  	// 2 byte 
		device_info.buf[3] = addr;	 //  addr
		device_info.buf[4] = value; //  value   
		device_info.buf[5] = 80;	  // 'P'

		retry_count = 0;		//0x00ff;
		status = FALSE;
		while(status == FALSE)
		{
			status = WriteFile(device_info.hCom, device_info.buf, 6, &nWrite, NULL);
			if(status || (retry_count++>100))
			{
				break;
			}
			Sleep(1);			
		}
		if(!status)
			USB_DEVICE_LOG("i2c_write_reg fail!\n");
		Sleep(2);	// after write delay 20ms
	}

	return status;
}

BOOL i2c_read_reg(unsigned char slave, unsigned char addr, unsigned char *buf, unsigned short len)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		device_info.buf[0] = slave<<1;
		device_info.buf[1] = addr;
		status = USBIO_StreamI2C(device_info.mCh341Index, 2, device_info.buf, len, buf);
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = slave<<1;
		device_info.buf[1] = addr;
		status = IICSendData(TRUE, device_info.buf, 2, 0, device_info.mStm32Index);
		device_info.buf[0] = (slave<<1)|0x01;
		status = IICSendData(TRUE, device_info.buf, 1, 0, device_info.mStm32Index);
		status = IICRcvData(buf, len, 1, device_info.mStm32Index);
	}
	else if(device_info.device == DEVICE_CP2102)
	{
		DWORD nWrite;
		int retry_count = 0;

		//PurgeComm(device_info.hCom, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
		//Sleep(1);
		//memset(buf_wr, 0, sizeof(buf_wr));

		// Point to the Register
		device_info.buf[0] = 83;   // S
		device_info.buf[1] = slave<<1;// Write command
		device_info.buf[2] = 1;    // # of bytes
		device_info.buf[3] = addr; // reg
		device_info.buf[4] = 80;   // P
		status = WriteFile(device_info.hCom, device_info.buf, 5, &nWrite, NULL);

		//memset(buf_wr, 0, sizeof(buf_wr));
		// Request 1 byte of data
		device_info.buf[0] = 83;   // S
		device_info.buf[1] = (slave<<1)|0x01; // Read command
		device_info.buf[2] = (unsigned char)len;    // # of bytes
		device_info.buf[3] = 80;   // P
		status = WriteFile(device_info.hCom, device_info.buf, 4, &nWrite, NULL);
		retry_count = 0;
		status = FALSE;
		while(status == FALSE)
		{
			status = ReadFile(device_info.hCom, buf, len, &nWrite, NULL);
			if(status || (retry_count++>100))
			{
				break;
			}
			Sleep(1);
		}
		
		if(!status)
			USB_DEVICE_LOG("i2c_read_reg fail!\n");
		//Sleep(2);
	}

	return status;
}

BOOL spi_write_reg(unsigned char addr, unsigned char value)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		device_info.buf[0] = addr;
		device_info.buf[1] = value;
		status = USBIO_StreamSPI4(device_info.mCh341Index, 0x80, 2, device_info.buf);		// use CS0
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = addr;
		device_info.buf[1] = value;
		status = SPISendData(0, 1, device_info.buf, 2, device_info.mStm32Index);
	}
	return status;
}

BOOL spi_read_reg(unsigned char addr, unsigned char *buf, unsigned short len)
{
	BOOL status = FALSE;

	if(device_info.device == DEVICE_CH341A)
	{
		device_info.buf[0] = addr;
		status = USBIO_StreamSPI4(device_info.mCh341Index, 0x80, len+1, device_info.buf);		// use CS0
		memcpy(buf, &device_info.buf[1], len);
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		device_info.buf[0] = addr;
		status = SPISendData(0, 0, device_info.buf, 1, device_info.mStm32Index);
		status = SPIRcvData(0, 1, buf, len, device_info.mStm32Index);
	}
	return status;
}



BOOL device_write_io(unsigned char value)
{
	int index;
	BOOL status;

	if(device_info.device == DEVICE_CH341A)
	{
		status = USBIO_Set_D5_D0(device_info.mCh341Index, 0x3f, 0x3f&value);
	}
	else if(device_info.device == DEVICE_STM32F103)
	{
		for(index=0; index<8; index++)
		{
			if((value)&(1<<index))
				status = IOSetAndRead(index, 1, 1, device_info.mStm32Index);
			else
				status = IOSetAndRead(index, 1, 0, device_info.mStm32Index);
		}
	}

	return status;
}


VOID CALLBACK ch341_irq_callback(ULONG iStatus)
{
	if(device_info.irq1_func)
	{	
		SYSTEMTIME st;	 
		GetLocalTime(&st);	 
		USB_DEVICE_LOG("%d-%d-%d-%02d-%02d-%02d.%03d ==>", st.wYear,st.wMonth,st.wDay,
														st.wHour,st.wMinute,st.wSecond,st.wMilliseconds);
		device_info.irq1_func();
	}
}

void device_register_irq(irq_callback irq_fun)
{
	if(device_info.device == DEVICE_CH341A)
	{
		USBIO_SetIntRoutine(device_info.mCh341Index, ch341_irq_callback);
		device_info.irq1_func = irq_fun;
	}
}


