# QT-STM32-GY51
  本设计利用stm32c8t6单片机通过IIC协议读取
  GY-511传感器中加速度值以及磁感应强度值
  经过算法计算得到磁偏角
  并设计图形界面显示指南针状态，
  实现一个简易的指南针系统
  

GY-511——STM32 F103 c8t6

3.3V————3.3V

GND————GND

SCL————B6

SDA————B7

（2）CP2102——STM32 F103 c8t6

TXD————A10

RXD————A9

（3）ST-Link——STM32 F103 c8t6

GND————GND

3.3V————3.3V

SWDIO————SWDIO

SWCLK————SWCLK


5.运行环境

STM32程序：Keil uVision5

Qt程序：Qt Creato 4.8.2 Qt5.12.1
