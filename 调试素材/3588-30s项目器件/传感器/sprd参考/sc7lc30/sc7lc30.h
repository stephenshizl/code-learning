#ifndef __SC7LC30_PLS_H__
#define __SC7LC30_PLS_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#define SC7LC30_PS_INTERRUPT_MODE                 
//#define SL_IRQ_GPIO_NUMBER 		     93                
                  


#define SL_IOCTL_MAGIC               0x1C                                         
#define SL_IOCTL_GET_PFLAG           _IOR(SL_IOCTL_MAGIC, 1, int)                  
#define SL_IOCTL_GET_LFLAG           _IOR(SL_IOCTL_MAGIC, 2, int)                  
#define SL_IOCTL_SET_PFLAG           _IOW(SL_IOCTL_MAGIC, 3, int)                 
#define SL_IOCTL_SET_LFLAG           _IOW(SL_IOCTL_MAGIC, 4, int)                 
#define SL_IOCTL_GET_DATA            _IOW(SL_IOCTL_MAGIC, 5, unsigned char)  
#define SL_IOCTL_GET_CHIPINFO            _IOR(SL_IOCTL_MAGIC, 6, char)



/*****************register map ****************************/
#define SC7LC30_INTC_CONFIG	         0x00   
#define SC7LC30_PX_IT_TIME	    	 0x01   
#define SC7LC30_ALS_GAIN	         0x02   
#define SC7LC30_PS_GAIN	    		 0x03 
#define SC7LC30_PERS        		 0x04   
#define SC7LC30_LED_CTL		         0x05  
#define SC7LC30_WAIT_TIME	         0x06   
#define SC7LC30_ALS_LTHH		     0x07    
#define SC7LC30_ALS_HTHH		     0x09   
#define SC7LC30_ALS_HTHL		     0x0a      
#define SC7LC30_PX_LTHH			     0x0b    
#define SC7LC30_PX_LTHL			     0x0c    
#define SC7LC30_PX_HTHH			     0x0d   
#define SC7LC30_PX_HTHL			     0x0e 
#define SC7LC30_PSDATA_READY		0x1F
#define	SC7LC30_PX_MSB		         0x20  
#define	SC7LC30_PX_LSB		         0x21  
#define	SC7LC30_ADC_MSB		         0x22 
#define	SC7LC30_ADC_LSB		         0x23  
#define  SC7LC30_POFFSETH_REG        0x2c   
#define  SC7LC30_POFFSETL_REG        0x2d   
#define SC7LC30_INT_STATUS_REG	         0x30   
#define SC7LC30_CONFIG_REG	         0x31   
#define  SC7LC30_COMMAND_REG         0x32   
#define  SC7LC30_COMMAND_2_REG       0x33
#define  SC7LC30_NF_FLAG_REG         0x34



struct sc7lc30_platform_data {
	uint32_t int_flags;
	int int_pin;
	uint8_t mode_reg;
	uint8_t it_val;
	uint8_t led_ctrl;
	uint8_t ps_gain;
	uint8_t als_gain;
	uint8_t wait_time;
	uint8_t config_val;
	uint8_t command_val;
	uint8_t pers_val;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
};
#endif
