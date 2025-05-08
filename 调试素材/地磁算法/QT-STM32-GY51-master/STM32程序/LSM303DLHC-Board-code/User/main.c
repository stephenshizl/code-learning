/*********************************************************************************************************
*
* File                : main.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include  <math.h>
#include "stm32f10x.h"
#include "I2C.h"
#include "LSM303.h"
#include "systick.h"
#include "printf.h"
float buf_A[3],buf_M[3];
float north;

/*******************************************************************************
* Function Name  : Delay
* Description    : Delay Time
* Input          : - nCount: Delay Time
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void Delay(uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
  //USART_Configuration();
	printf_init();
	I2C_Configuration();
	printf("\r\n**********************\r\n");
 
  LSM303A_Init();
  LSM303M_Init();
	
 	printf("\r\n********         LSM303DLHC       *********************\r\n");

  while (1)
  {
		LAM303M_Raed(buf_M);
		LAM303A_Raed(buf_A);
		north=Data_conversion(buf_A,buf_M);
		
//		printf("X_A=%f m/s^2\r\n",buf_A[0]);
//		printf("Y_A=%f m/s^2\r\n",buf_A[1]);
//		printf("Z_A=%f m/s^2\r\n",buf_A[2]);

//		printf("X_M=%f T\r\n",buf_M[0] );
//		printf("Y_M=%f T\r\n",buf_M[1]);
//		printf("Z_M=%f T\r\n",buf_M[2]);
//		printf("north=%f degree \r\n",north);
//		printf("\r\n");

		printf("@%d#", (int)north);

		delay_ms(10);
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

