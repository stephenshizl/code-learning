/*********************************************************************************
使用本转接板分四步调用函数：
第一步：打开usb调用函数 OpenUsb
第二步：配置UART或者SPI或者IIC参数
        调用函数：ConfigUARTParam 或 ConfigSPIParam 或 ConfigIICParam
第三步：发送或接受数据
        发送数据调用：UARTSendData 或者 SPISendData 或者 IICSendData
        接收数据调用：UARTRcvData 或者 SPIRcvData 或者 IICRcvData
        设置IO口调用：IOSetAndRead
第四步：关闭USB调用函数 CloseUSB
*********************************************************************************/
/*********************************************************************************
UsbIndex:
关于每个函数中UsbIndex参数的说明：
1. 如果您的电脑上只连接了一个转接板，这个参数使用默认值0
2. 如果您的电脑上连接了两个或者两个以上设备，这个参数指连接到电脑的第几个转接板，从0开始，最大9
*********************************************************************************/
#ifndef __USB2UARTSPIIICDLL_H__
#define	__USB2UARTSPIIICDLL_H__

#ifdef USB2UARTSPIIICDLL_EXPORTS
#define USB2UARTSPIIICDLL_API extern "C" __declspec(dllexport)
#else
#define USB2UARTSPIIICDLL_API extern "C" __declspec(dllimport)
#endif


//UART波特率
#define UART_BaudRate_1200                  1200
#define UART_BaudRate_2400                  2400
#define UART_BaudRate_4800                  4800
#define UART_BaudRate_9600                  9600
#define UART_BaudRate_14400                 14400
#define UART_BaudRate_19200                 19200
#define UART_BaudRate_38400                 38400
#define UART_BaudRate_57600                 57600
#define UART_BaudRate_115200                115200
//UART奇偶校验位
#define UART_Parity_No                      0
#define UART_Parity_Even                    1
#define UART_Parity_Odd                     2
//UART停止位
#define UART_StopBits_1                     0
#define UART_StopBits_1_5                   1
#define UART_StopBits_2                     2
//SPI速率
#define SPI_Rate_281K           0
#define SPI_Rate_562K           1
#define SPI_Rate_1_125M           2
#define SPI_Rate_2_25M            3
#define SPI_Rate_4_5M           4
#define SPI_Rate_9M             5
#define SPI_Rate_18M            6
#define SPI_Rate_1K            7
//SPI帧格式
#define SPI_MSB               0
#define SPI_LSB               1
//SPI子模式
#define SPI_SubMode_0           0
#define SPI_SubMode_1           1
#define SPI_SubMode_2           2
#define SPI_SubMode_3           3
//IIC速率
#define IIC_Rate_100K           0
#define IIC_Rate_200K           1
#define IIC_Rate_400K           2
#define IIC_Rate_10K           3
#define IIC_Rate_1K           4
//SPI从模式下是否检测CS管脚
#define SPI_SLAVE_CSDS               0
#define SPI_SLAVE_CSEN               1

//IO口
/****************************************************************************
名称：OpenUsb                           
参数：UsbIndex USB索引(0-9)                               
返回值：TRUE    打开转接板USB成功 
        FALSH   打开转接板USB失败
功能：打开转接板USB
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL OpenUsb(int UsbIndex);


/****************************************************************************
名称：CloseUSB                            
参数：UsbIndex USB索引(0-9)                                
返回值：TRUE    关闭转接板USB成功 
        FALSH   关闭转接板USB失败
功能：关闭转接板USB
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL CloseUSB(int UsbIndex);


/****************************************************************************
名称：ConfigUARTParam                           
参数：baudRate  UART波特率     只能是如下参数:
				UART_BaudRate_1200  
                UART_BaudRate_2400  
                UART_BaudRate_4800  
                UART_BaudRate_9600  
                UART_BaudRate_14400 
                UART_BaudRate_19200 
                UART_BaudRate_38400 
                UART_BaudRate_57600 
                UART_BaudRate_115200
      parity    UART奇偶校验位  只能是如下参数:
                UART_Parity_No                    无校验
                UART_Parity_Even                  偶校验
                UART_Parity_Odd                   奇校验
      stopBits  UART停止位位    只能是如下参数:
                UART_StopBits_1                   1位停止位
                UART_StopBits_1_5                 1.5位停止位
                UART_StopBits_2                   2位停止位

         
返回值：TRUE    参数配置成功  
        FALSH   参数配置失败
功能：配置UART参数
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigUARTParam (unsigned int baudRate,unsigned int parity,unsigned int stopBits,int UsbIndex);

/****************************************************************************
名称：ConfigSPIParam                           
参数：rate      SPI速率     只能是如下参数:
                SPI_Rate_281K                   281K      
                SPI_Rate_562K                   562K  
                SPI_Rate_1_125M                 1.125M  
                SPI_Rate_2_25M                  2.25M
                SPI_Rate_4_5M                   4.5M
                SPI_Rate_9M                     9M
                SPI_Rate_18M                    18M
				SPI_Rate_1K						1K

      fistBit   SPI帧格式   只能是如下参数:
                SPI_MSB                         先发高位
                SPI_LSB                         先发低位
      subMode   SPI子模式   只能是如下参数:
                SPI_SubMode_0                   子模式0（CPOL=0, CPHA=0）
                SPI_SubMode_1                   子模式1（CPOL=0, CPHA=1）
                SPI_SubMode_2                   子模式2（CPOL=1, CPHA=0）
                SPI_SubMode_3                   子模式3（CPOL=1, CPHA=1）
         
返回值：TRUE    参数配置成功  
        FALSH   参数配置失败
功能：设置SPI主模式参数  
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigSPIParam(unsigned int rate,unsigned int fistBit,unsigned int subMode,int UsbIndex);

/****************************************************************************
名称：ConfigSPIParamSlave                               
参数：CSEn      SPI CS管脚 是否有效     只能是如下参数:
                SPI_SLAVE_CSDS               0             不检测CS管脚   实时接收和发送数据
				SPI_SLAVE_CSEN               1             只有在CS管脚为低电平的时候  才能发送和接收数据

      fistBit   SPI帧格式   只能是如下参数:
                SPI_MSB                         先发高位
                SPI_LSB                         先发低位
      subMode   SPI子模式   只能是如下参数:
                SPI_SubMode_0                   子模式0（CPOL=0, CPHA=0）
                SPI_SubMode_1                   子模式1（CPOL=0, CPHA=1）
                SPI_SubMode_2                   子模式2（CPOL=1, CPHA=0）
                SPI_SubMode_3                   子模式3（CPOL=1, CPHA=1）
         
返回值：TRUE    参数配置成功  
        FALSH   参数配置失败
功能：设置SPI从模式参数 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigSPIParamSlave(unsigned int CSEn,unsigned int fistBit,unsigned int subMode,int UsbIndex);

/****************************************************************************
名称：ConfigIICParam                           
参数：rate      IIC速率     只能是如下参数:
                IIC_Rate_100K
                IIC_Rate_200K
                IIC_Rate_400K
				IIC_Rate_10K
				IIC_Rate_1K
返回值：TRUE    参数配置成功  
        FALSH   参数配置失败
功能：配置IIC主模式参数
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigIICParam(unsigned int rate,int UsbIndex);

/****************************************************************************
名称：ConfigIICParamSlave                           
参数：addr      IIC从模式下的设备地址  只能在0x00到0x7F之间
返回值：TRUE    参数配置成功  
        FALSH   参数配置失败
功能：配置IIC从模式参数
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL ConfigIICParamSlave(unsigned char addr,int UsbIndex);

/****************************************************************************
名称：UARTSendData                           
参数：sendBuf   要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：UART发送数据
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL UARTSendData(unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
名称：UARTRcvData                           
参数：rcvBuf    要接收数据的缓存
      len       要接收数据的最大长度
返回值：正数    表示实际接收的字节数（最大4096）  
        负数    接收失败
功能：UART接收数据
****************************************************************************/
USB2UARTSPIIICDLL_API int UARTRcvData(unsigned char *rcvBuf, unsigned int len,int UsbIndex);

/****************************************************************************
名称：SPISendData                          
参数：startCS   发送数据前CS管脚的电平
                0：低电平
                1：高电平
      endCS     发送数据后CS管脚的电平
                0：低电平
                1：高电平
      sndBuf    要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：SPI 主模式下发送数据 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendData(unsigned int startCS, unsigned int endCS,unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
名称：SPISendAndRecData                     
参数：startCS   发送数据前CS管脚的电平
                0：低电平
                1：高电平
      endCS     发送数据后CS管脚的电平
                0：低电平
                1：高电平
      sndBuf    要发送数据的缓存
	  rcvBuf    要接收数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：SPI 主模式下发送和接收数据 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendAndRecData(unsigned int startCS, unsigned int endCS,unsigned char *sendBuf,unsigned char *rcvBuf,unsigned int len,int UsbIndex);

/****************************************************************************
名称：SPIRcvData                         
参数：startCS   接收数据前CS管脚的电平
                0：低电平
                1：高电平
      endCS     接收数据后CS管脚的电平
                0：低电平
                1：高电平
      rcvBuf    要接收数据的缓存
      len       要接收数据的长度（最大4096）
返回值：TRUE    接收成功  
        FALSH   接收失败
功能：SPI 主模式下接收数据 
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPIRcvData(unsigned int startCS, unsigned int endCS,unsigned char *rcvBuf, unsigned int len,int UsbIndex);


/****************************************************************************
名称：SPISendDataSlave                          
参数：sndBuf    要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：spi从模式下 将要发送的数据预装到单片机   
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendDataSlave(unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
名称：SPIRcvDataSlave                           
参数：rcvBuf    要接收数据的缓存
      len       要接收数据的最大长度
返回值：正数    表示实际接收的字节数（最大4096）  
        负数    接收失败
功能：SPI从模式接收数据接收数据
****************************************************************************/
USB2UARTSPIIICDLL_API int SPIRcvDataSlave(unsigned char *rcvBuf, unsigned int len,int UsbIndex);


/****************************************************************************
名称：IICSendData                         
参数：strartBit 发送数据前是否发送IIC Start信号
                TRUE：发送
                FALSH：不发送
      stopBit   发送数据后是否发送IIC Stop信号
                TRUE：发送
                FALSH：不发送
      sendBuf   要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：IIC发送数据
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IICSendData(BOOL strartBit,unsigned char *sendBuf, unsigned int len, BOOL stopBit,int UsbIndex);

/****************************************************************************
名称：IICRcvData                         
参数：stopBit   接收数据后是否发送IIC Stop信号
                TRUE：发送
                FALSH：不发送
      rcvBuf    要接收数据的缓存
      len       要接收数据的长度（最大4096）
返回值：TRUE    接收成功  
        FALSH   接收失败
功能：IIC接收数据
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IICRcvData(unsigned char *rcvBuf, unsigned int len, BOOL stopBit,int UsbIndex);


/****************************************************************************
名称：IICSendDataSlave                          
参数：sndBuf    要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：IIC从模式下 将要发送的数据预装到单片机   
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IICSendDataSlave(unsigned char *sendBuf, unsigned int len,int UsbIndex);

/****************************************************************************
名称：IICRcvDataSlave                           
参数：rcvBuf    要接收数据的缓存
      len       要接收数据的最大长度
返回值：正数    表示实际接收的字节数（最大4096）  
        负数    接收失败
功能：IIC从模式接收数据接收数据
****************************************************************************/
USB2UARTSPIIICDLL_API int IICRcvDataSlave(unsigned char *rcvBuf, unsigned int len,int UsbIndex);

/****************************************************************************
名称：IOSetAndRead                        
参数：IONum     要设置或读取的IO口号（0-7）
      IODir     该IO的方向  
                1：输出
                0：输入
      IOBit     该IO口的电平（只对输出有效）
                1：高电平
                0：低电平
返回值：如果IO口被设置成输入
        返回TRUE：表示该IO口读取到高电平 
        返回FALSH：表示该IO口读取到低电平
功能：设置和读取IO口的电平
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL IOSetAndRead(unsigned int IONum, unsigned int IODir, unsigned int IOBit,int UsbIndex);

/****************************************************************************
名称：SPISendDataFast                         
参数：startCS   发送数据前CS管脚的电平
                0：低电平
                1：高电平
      endCS     发送数据后CS管脚的电平
                0：低电平
                1：高电平
      sndBuf    要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：SPI 主模式下发送数据 快速模式 此模式下不支持1K速率
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendDataFast(unsigned int startCS, unsigned int endCS,unsigned char *sendBuf, unsigned int len,int UsbIndex);
/****************************************************************************
名称：SPIRcvDataFast                        
参数：startCS   接收数据前CS管脚的电平
                0：低电平
                1：高电平
      endCS     接收数据后CS管脚的电平
                0：低电平
                1：高电平
      rcvBuf    要接收数据的缓存
      len       要接收数据的长度（最大4096）
返回值：TRUE    接收成功  
        FALSH   接收失败
功能：SPI 主模式下接收数据  快速模式  此模式下不支持1K速率
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPIRcvDataFast(unsigned int startCS, unsigned int endCS,unsigned char *rcvBuf, unsigned int len,int UsbIndex);
/****************************************************************************
名称：SPISendDataFast                         
参数：sndBuf    要发送数据的缓存
      len       要发送数据的长度（最大4096）
返回值：TRUE    发送成功  
        FALSH   发送失败
功能：SPI 主模式下发送数据 此模式下每发送一个字节CS管脚电平跳变一次 此模式下不支持1K速率
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPISendDataSpecial(unsigned char *sendBuf, unsigned int len,int UsbIndex);
/****************************************************************************
名称：SPIRcvDataFast                        
参数：rcvBuf    要接收数据的缓存
      len       要接收数据的长度（最大4096）
返回值：TRUE    接收成功  
        FALSH   接收失败
功能：SPI 主模式下接收数据  此模式下每接收一个字节CS管脚电平跳变一次  此模式下不支持1K速率
****************************************************************************/
USB2UARTSPIIICDLL_API BOOL SPIRcvDataSpecial(unsigned char *rcvBuf, unsigned int len,int UsbIndex);

#endif






