#include <REGX52.H>
//#include <AT89X52.H>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "qst_sw_i2c.h"
#include "qst_common.h"

#define FSOC 24000000L    //����
#define BAUD  9600        //������

volatile uint_8 uart_sending;

void uart_init(void)                                //?????
{
	EA=0; //�ر����ж�
	TMOD&=0x0F;				//����T1�Ŀ���λ
	TMOD|=0x20;    //��ʱ��1��ģʽ2����ģʽ
	SCON=0x50;     //����Ϊ������ʽ1
	TH1=0xf3;//256-(FSOC/(BAUD*12*16));//0xf3;//4800 bps
	TL1=0xf3;//256-(FSOC/(BAUD*12*16));//0xf3;//256-jingzhen/(botelv*12*16);
	PCON|=0x80;    //�������ʼӱ�
	ES=1;         //�򿪽����ж�
	TR1=1;        //�򿪼�����
	REN=1;        //???? 
	EA=1;         //�����ж�
}

#if 0
void uart_send_byte(unsigned char d)
{
	SBUF=d;
	uart_sending=1;
	while(uart_sending);
}
#endif

void uart_send_string(unsigned char * pd)
{
	while((*pd)!='\0')
	{
		//uart_send_byte(*pd);
		SBUF=* pd;
		uart_sending=1;
		while(uart_sending);
		pd++;
	}
}

void qst_log(const char *format, ...)
{
	va_list arg_list;
	int_8 xdata log_buf[60];								// qst log buffer

	memset(log_buf, 0, sizeof(log_buf));
	va_start(arg_list, format);
	//vsnprintf(qst_log_str, sizeof(qst_log_str), format, arg_list);
	vsprintf(log_buf,format, arg_list);	
	//sprintf(log_buf,format, arg_list);
	va_end(arg_list);
	uart_send_string(log_buf);
}


/*******************************************************************************
* ������         : Usart() interrupt 4
* ��������		  : ����ͨ���жϺ���
* ����           : ��
* ���         	 : ��
*******************************************************************************/
void uart(void) interrupt 4
{
 if(RI)    //�����ն˱�־
 {
  RI=0;   //��������жϱ�־λ
 }
 else			// �����жϱ�־
 {
  TI=0;			//���������ɱ�־λ
  uart_sending=0;  //���������ɱ�־λ
 }
}
