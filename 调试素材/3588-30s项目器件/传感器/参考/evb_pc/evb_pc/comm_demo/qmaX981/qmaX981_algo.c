/* qmaX981 motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#include "qmaX981.h"
#include <stdbool.h>
#include <string.h>
#include <Math.h>

#if defined(QMAX981_STEP_COUNTER)

typedef enum
{
	qst_false,
	qst_true
} qst_bool;

typedef struct
{
	int	x;
	int y;
	int z;
} qst_acc_data;


#if defined(QMAX981_CHECK_ABNORMAL_DATA)
typedef struct
{
	int last_data;
	int curr_data;
	int more_data[3];
}qmaX981_data_check;

#define QMA6981_DIFF(x, y)		((x)>=(y)?((x)-(y)):((x)+65535-(y)))
#define QMAX981_ABNORMAL_DIFF		30
static qmaX981_data_check g_qmaX981_data_c;
#endif

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
#define STEP_INT_START_VLUE		4
#define STEP_DUMMY_VLUE			8

#define STEP_END	qst_false
#define STEP_START	qst_true

struct qmaX981_stepcount{
	//int stepcounter_pre_start;
	int stepcounter_pre_end;  
	int stepcounter_next_start;
	int stepcounter_next_end;  
	int stepcounter_pre;
	int stepcounter_pre_fix;
	qst_bool stepcounter_statu;
	qst_bool stepcounter_start_int_update_diff;
	int back;
	int step_diff;
	qst_bool int_statu_flag;
};

static struct qmaX981_stepcount step_count_index;
void qmaX981_step_debounce_reset(void)
{
	memset(&step_count_index, 0, sizeof(step_count_index));
}

int qmaX981_step_debounce_int_work(int data, unsigned char irq_level)
{
	int temp = 0;

	if(irq_level == 0)
	{	
		step_count_index.int_statu_flag = qst_false;
		step_count_index.stepcounter_next_end = data;
		step_count_index.stepcounter_statu = STEP_END;
		QMAX981_LOG("step_int stepcounter_next_end = %d stepcounter_next_start = %d\n", step_count_index.stepcounter_next_end,step_count_index.stepcounter_next_start);
		if(step_count_index.stepcounter_next_end < step_count_index.stepcounter_next_start)
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_end - step_count_index.stepcounter_next_start;
		}
		QMAX981_LOG("step_int_end  temp =%d\n" ,temp);
		if (temp < STEP_DUMMY_VLUE)
		{
			step_count_index.step_diff += (temp+STEP_INT_START_VLUE);
			// add by yangzhiqiang for step_diff
			if(step_count_index.step_diff > data)
			{
				step_count_index.step_diff = data;
			}
			// yangzhiqiang for step_diff
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
			
		}
		else
		{
			step_count_index.stepcounter_pre_end = step_count_index.stepcounter_next_end;
		}		
		//irq_set_irq_type(g_QMA6981_ptr->irq,IRQF_TRIGGER_RISING);
		QMAX981_LOG("step_int_end\n" );
	}
	else
	{
		step_count_index.int_statu_flag = qst_true;
		step_count_index.stepcounter_next_start= data;
		step_count_index.stepcounter_statu = STEP_START;
		QMAX981_LOG("step_int stepcounter_next_start = %d stepcounter_pre_end = %d\n", step_count_index.stepcounter_next_start,step_count_index.stepcounter_pre_end);
		if (step_count_index.stepcounter_next_start < step_count_index.stepcounter_pre_end)
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end+65536;
		}
		else
		{
			temp = step_count_index.stepcounter_next_start - step_count_index.stepcounter_pre_end;
		}
		QMAX981_LOG("step_int_start temp =%d\n" ,temp);
		if (temp >STEP_INT_START_VLUE)
		{
			step_count_index.step_diff += (temp - STEP_INT_START_VLUE);
		}
		//res = request_irq(g_QMA6981_ptr->irq, QMA6981_eint_handler, IRQF_TRIGGER_FALLING, "gsensor-eint", NULL);
		//irq_set_irq_type(g_QMA6981_ptr->irq,IRQF_TRIGGER_FALLING);
		QMAX981_LOG("step_int_start\n" );
	}

	return step_count_index.int_statu_flag;
}


int qmaX981_step_debounce_read_data(int result)
{
	int tempp = 0;
	int data = 0;

	if (result < step_count_index.stepcounter_pre)
	{
		step_count_index.back++;
		data = result- step_count_index.stepcounter_pre + 65536;
		step_count_index.stepcounter_pre = result;
	}
	else
	{
		//nothing
		step_count_index.stepcounter_pre = result;
		data = result;
	}

	if (step_count_index.stepcounter_statu == STEP_START)
	{
	/*
		if (data >= step_count_index.stepcounter_pre_end)
		{
			tempp = data - step_count_index.stepcounter_pre_end;
		}
		else
		{
			tempp = data - step_count_index.stepcounter_pre_end +65536;
		}
	*/
		if (data >= step_count_index.stepcounter_next_start)
		{
			tempp = data - step_count_index.stepcounter_next_start + 4;
		}
		else
		{
			tempp = data - step_count_index.stepcounter_next_start +65540;
		}

		QMAX981_LOG("ReadStepCounter_running data= %d,stepcounter_next_start = %d,tempp = %d,stepcounter_pre_end =%d \n",data,step_count_index.stepcounter_next_start,tempp,step_count_index.stepcounter_pre_end);
		
		if (tempp < (STEP_INT_START_VLUE+STEP_DUMMY_VLUE))
		{
			data = step_count_index.stepcounter_pre_fix;
			QMAX981_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
		}
		else
		{
			if (step_count_index.step_diff >data)
			{
				step_count_index.step_diff = 0;
			}
			else
			{
				 data = data -  step_count_index.step_diff;
				 step_count_index.stepcounter_pre_fix = data;
				 QMAX981_LOG("ReadStepCounter_running stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
			}
		}
	
	}
	else 
	{
		// add by yangzhiqiang for step_diff
		if(step_count_index.step_diff > data)
		{
			step_count_index.step_diff = data;
		}
		// yangzhiqiang for step_diff		
#if 1//defined(QMA6981_STEP_TO_ZERO)
		step_count_index.stepcounter_pre_end = data;
#endif
		data  = data -  step_count_index.step_diff;
		 step_count_index.stepcounter_pre_fix = data;
		 QMAX981_LOG("ReadStepCounter_end stepcounter_pre_fix = %d\n",step_count_index.stepcounter_pre_fix);
	}
	//mutex_unlock(&qma6981_mutex); // removed by yangzhiqiang
	QMAX981_LOG("ReadStepCounter=%d, step_diff= %d\n",data,step_count_index.step_diff );

	return data;
}
#endif


#if defined(QMAX981_CHECK_ABNORMAL_DATA)
int qmaX981_check_abnormal_data(int data_in, int *data_out)
{
	int ret = 0;
	int diff;
	unsigned char data[2];

	g_qmaX981_data_c.curr_data = data_in;
	diff = QMA6981_DIFF(g_qmaX981_data_c.curr_data, g_qmaX981_data_c.last_data);
	if(diff > QMAX981_ABNORMAL_DIFF)
	{
		// read data 1
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		if(ret == 0)
			g_qmaX981_data_c.more_data[0] = -1;
		else
			g_qmaX981_data_c.more_data[0] = ((data[1]<<8) |( data[0]));
		
		// read data 2
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		if(ret == 0)
			g_qmaX981_data_c.more_data[1] = -1;
		else
			g_qmaX981_data_c.more_data[1] = ((data[1]<<8) |( data[0]));


		// read data 3
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		if(ret == 0)
			g_qmaX981_data_c.more_data[2] = -1;
		else
			g_qmaX981_data_c.more_data[2] = ((data[1]<<8) |( data[0]));


		if((g_qmaX981_data_c.more_data[0]<0)||(g_qmaX981_data_c.more_data[1]<0)||(g_qmaX981_data_c.more_data[2]<0))
		{
			return -1;
		}
		
		//if((QMA6981_ABS(g_qmaX981_data_c.more_data[0]-g_qmaX981_data_c.curr_data) > 1)
		//	||(QMA6981_ABS(g_qmaX981_data_c.more_data[1]-g_qmaX981_data_c.curr_data) > 1)
		//	||(QMA6981_ABS(g_qmaX981_data_c.more_data[2]-g_qmaX981_data_c.curr_data) > 1)
		//	)
		if((g_qmaX981_data_c.more_data[0]==g_qmaX981_data_c.more_data[1])
			||(g_qmaX981_data_c.more_data[1]==g_qmaX981_data_c.more_data[2]))
		{		
			*data_out = g_qmaX981_data_c.more_data[0];
			g_qmaX981_data_c.last_data = g_qmaX981_data_c.more_data[0];
		}
		else
		{		
			return -1;
		}
	}
	else
	{
		g_qmaX981_data_c.last_data = g_qmaX981_data_c.curr_data;
	}

	return 0;
}
#endif

#endif

#if defined(QMAX981_AUTO_CALI)
#define QMAX981_STATIC_THRES	400		// 40mg
typedef struct
{
	int 			flat_count;
	int 			count_max;
	qst_acc_data	flat;
	//int 			acc_count;
	qst_acc_data	acc;
	qst_acc_data	cali;
}qmaX981_auto_cali;

static qmaX981_auto_cali g_qmaX981_cali;

int qmaX981_check_flat_auto_cali(int acc_data[3], int delay)
{
	int x_diff, y_diff;
	int result = 0;
	
	x_diff = QMAX981_ABS(acc_data[0]-g_qmaX981_cali.flat.x);
	y_diff = QMAX981_ABS(acc_data[1]-g_qmaX981_cali.flat.y);
	g_qmaX981_cali.flat.x = acc_data[0];
	g_qmaX981_cali.flat.y = acc_data[1];
	g_qmaX981_cali.flat.z = acc_data[2];
	if(delay > 0)
	{
		g_qmaX981_cali.count_max = (3000/delay)+1;
		if(g_qmaX981_cali.count_max < 10)
			g_qmaX981_cali.count_max = 10;
	}
	else
	{
		g_qmaX981_cali.count_max = 10;
	}

	if((x_diff<QMAX981_STATIC_THRES)&&(y_diff<QMAX981_STATIC_THRES)
		&&(QMAX981_ABS(acc_data[0])<1500)&&(QMAX981_ABS(acc_data[1])<1500))
	{
		if(g_qmaX981_cali.flat_count == 0)
		{
			g_qmaX981_cali.acc.x = 0;
			g_qmaX981_cali.acc.y = 0;
			g_qmaX981_cali.acc.z = 0;
		}
		g_qmaX981_cali.acc.x += acc_data[0];
		g_qmaX981_cali.acc.y += acc_data[1];
		g_qmaX981_cali.acc.z += acc_data[2];
		
		g_qmaX981_cali.flat_count++;
	}
	else
	{
		g_qmaX981_cali.flat_count = 0;
	}

	if(g_qmaX981_cali.flat_count >= g_qmaX981_cali.count_max)
	{
		g_qmaX981_cali.cali.x += 0-(g_qmaX981_cali.acc.x/g_qmaX981_cali.count_max);
		g_qmaX981_cali.cali.y += 0-(g_qmaX981_cali.acc.y/g_qmaX981_cali.count_max);
		g_qmaX981_cali.cali.z += 9807-(g_qmaX981_cali.acc.z/g_qmaX981_cali.count_max);

		result = 1;
	}

	return result;
}


void qmaX981_auto_cali_update(int *cali_data)
{
	cali_data[0] = g_qmaX981_cali.cali.x;
	cali_data[1] = g_qmaX981_cali.cali.y;
	cali_data[2] = g_qmaX981_cali.cali.z;
}


void qmaX981_auto_cali_reset(void)
{
	memset(&g_qmaX981_cali, 0, sizeof(g_qmaX981_cali));
}
#endif


#if defined(QMA7981_MULT_CLICK)
typedef struct
{
	unsigned char check_click;
	unsigned short click_num;
	unsigned short read_data_num;
	unsigned short static_num;
	unsigned char  moving_flag;
	unsigned short t_msec_1;			// check_click timer
	unsigned short t_msec_2;			// check static timer
	unsigned short t_msec_out;			// timeout
}qst_click_check;

static unsigned int acc_data_curr[3];
static unsigned int acc_data[3];
static qst_click_check g_click;

extern void qst_evb_set_click_num(unsigned short click_num);

void click_timer_cbk_out(int timerId)
{
	//QMAX981_LOG("qmaX981_timer_cbk_out \n");	
	bsp_stop_timer(timerId);
	bsp_stop_timer(1);
	if(g_click.click_num >= 2)
	{
#if 0
		if(click_timer_check_moving())
		{
			QMAX981_LOG(" click detect ingor! moving!!! \n");
		}
		else
		{
			QMAX981_LOG(" click detect num=%d !!! \n", g_click.click_num);
		}
#else
		if(g_click.moving_flag == 0)
		{
			//QMAX981_LOG(" click detect num=%d !!! \n", g_click.click_num);
			qst_evb_set_click_num(g_click.click_num);
		}
#endif
	}
	g_click.check_click = 1;
	g_click.static_num = 0;
	g_click.read_data_num = 0;
	g_click.click_num = 0;
	g_click.moving_flag = 0;
			
}

void click_timer_read_acc(int timerId)
{
	int data1, data2, ret;

	ret = qma7981_read_raw_xyz(acc_data_curr);
	if(ret)
	{
		data1 = QMAX981_ABS(acc_data_curr[0])+QMAX981_ABS(acc_data_curr[1])+QMAX981_ABS(acc_data_curr[2]);
		data2 = QMAX981_ABS(acc_data[0])+QMAX981_ABS(acc_data[1])+QMAX981_ABS(acc_data[2]);
		//QMAX981_LOG("acc_diff = %d \n", QMAX981_ABS(data1-data2));
		if(QMAX981_ABS(data1-data2) < 130)
		{
			g_click.static_num++;
		}
		acc_data[0] = acc_data_curr[0];
		acc_data[1] = acc_data_curr[1];
		acc_data[2] = acc_data_curr[2];
		g_click.read_data_num++;
	}
}

int click_timer_check_moving(void)
{
	if(g_click.t_msec_2 == 0)
	{
		return 0;
	}
	else
	{
		//QMAX981_LOG("read_data_num = %d static_num=%d \n", g_click.read_data_num, g_click.static_num);
		if(g_click.static_num > (g_click.read_data_num*45/100))
			return 0;
		else
			return 1;
	}
}


void click_timer_cbk_1(int timerId)
{
	bsp_stop_timer(timerId);
	g_click.check_click = 1;

	bsp_start_timer(0, g_click.t_msec_out, click_timer_cbk_out);	
	if(g_click.click_num >= 2)
	{
		if(click_timer_check_moving())
		{
			QMAX981_LOG(" device is moving!!! \n");
			g_click.moving_flag = 1;
		}
		else
		{
			//g_click.moving_flag = 0;
		}
	}
}

void click_any_motion_hdlr(void)
{
	if(g_click.check_click)
	{
		bsp_stop_timer(0);
		g_click.check_click = 0;
		g_click.click_num++;
		bsp_start_timer(0, g_click.t_msec_1, click_timer_cbk_1);
		// add by yangzhiqiang , read gsensor data, check moving
		if((g_click.t_msec_2 > 0)&&(g_click.click_num == 1))
		{
			g_click.static_num = 0;
			g_click.read_data_num = 0;
			bsp_start_timer(1, g_click.t_msec_2, click_timer_read_acc);
		}
		// add by yangzhiqiang
		//QMAX981_LOG(" any motion! %d\n", g_click.click_num);
	}
}

void click_para_init(void)
{
	g_click.check_click = 1;
	g_click.click_num = 0;
	g_click.read_data_num = 0;
	g_click.static_num = 0;
	g_click.moving_flag = 0;
	g_click.t_msec_1 = 150; //75;	//150;
#if 1
	g_click.t_msec_2 = 8;	//8;
	g_click.t_msec_out = 220;	//200;	//150;	//200;	//350;
#else
	g_click.t_msec_2 = 8;
	g_click.t_msec_out = 500;
#endif
}

#endif


#define FILTER_NUM 64
int16_t acc_filter[FILTER_NUM] = {0};
const float IIR_B[3][3] = {
//  {
//   0.001875439870576,                 0,                 0 
//  },
//  {
//                   1,    1.799441161951,                 1 
//  },
//  {
//                   1,                 0,                 0 
//  }
  {
    0.09281070473229,                 0,                 0 
  },
  {
                   1,    1.992099126127,                 1 
  },
  {
                   1,                 0,                 0 
  }
};
const float IIR_A[3][3] = {
//  {
//                   1,                 0,                 0 
//  },
//  {
//                   1,   -1.877117394646,   0.8842430180867 
//  },
//  {
//                   1,                 0,                 0 
//  }
  {
                   1,                 0,                 0 
  },
  {
                   1,  -0.9741716831754,   0.3446812164324 
  },
  {
                   1,                 0,                 0 
  }
};

float modulo_last[2];
float modulo_mlast[2];

static float IIR_DR2(float x, float *plast, const float (*A)[3], const float (*B)[3])
{
  float tmp = x * B[0][0];
 
  float last = tmp - (A[1][1] * plast[0] + A[1][2] * plast[1]);
  tmp = last + (B[1][1] * plast[0] + B[1][2] * plast[1]);
 
  plast[1] = plast[0];
  plast[0] = last;
 
  return tmp;
}
int16_t IIR_Filter(float modulo)
{
  float mid;
  int16_t acc_filter;
 
  mid = IIR_DR2(modulo, modulo_last, IIR_A, IIR_B);
  acc_filter = (int16_t)mid;//IIR_DR2(mid[0], x_mlast, &IIR_A[2], &IIR_B[2]);
  return acc_filter;
}

#define MAX_LEN		31
#define STEP_START_NUNM		16
static int16_t step_algo_buf[MAX_LEN];
static int16_t step_algo_index = 0;
static uint32_t time_stamp0 = 0;
static uint32_t time_stamp1 = 0;
static uint32_t algo_step_num = 0;
static uint32_t algo_step_temp = 0;
	
float qst_step_stdev(void)
{
	int32_t sum = 0;
	int32_t i;
	int32_t avg;
	float stdev;
	
	sum = 0;
	for(i=0; i<MAX_LEN; i++)
	{
		sum += step_algo_buf[i];
	}
	avg = sum/MAX_LEN;
	
	sum = 0;
	for(i=0; i<MAX_LEN; i++)
	{
		sum += (step_algo_buf[i]-avg)*(step_algo_buf[i]-avg);
	}
	stdev = sqrtf(sum/MAX_LEN);
	//printf("stdev;%f \n", stdev);
	return stdev;
}
	
uint32_t qst_step_algo(float norm, uint16_t ms)
{
	int16_t norm_f = 0;
	int16_t i, max_index, max_data;
	
	norm_f = IIR_Filter(norm);
	time_stamp1 += ms;
	if((time_stamp1 - time_stamp0) > 2000)
	{
		algo_step_temp = 0;
	}
	if(step_algo_index < MAX_LEN)
	{
		step_algo_buf[step_algo_index++] = norm_f;
	}
	else
	{
		for(i=MAX_LEN-1; i>0; i--)
		{
			step_algo_buf[i] = step_algo_buf[i-1];
		}
		step_algo_buf[0] = norm_f;
		
		max_index = 0;
		max_data = step_algo_buf[0];
		for(i=0; i<MAX_LEN; i++)
		{
			if(step_algo_buf[i] > max_data)
			{
				max_data = step_algo_buf[i];
				max_index = i;
			}
		}

		if(max_index == MAX_LEN/2)	// max data is middle data
		{
//			printf("max_index == MAX_LEN/2\n");
			if(step_algo_buf[max_index] > 10.5)
			{
//				printf("condition 2\n");
				if(((time_stamp1-time_stamp0) > 250)/* && ((time_stamp1-time_stamp0) < 2000)*/)		// time window valid
				{
					if(qst_step_stdev() > 0.15)
					{
//						printf("step valid \n");
						if(algo_step_temp < STEP_START_NUNM)
						{
							algo_step_temp++;
							printf("step temp:%d \n", algo_step_temp);
						}
						else if(algo_step_temp == STEP_START_NUNM)
						{
							algo_step_temp++;
							algo_step_num += algo_step_temp;
						}
						else
						{
							algo_step_temp++;
							algo_step_num++;
							printf("step num:%d \n", algo_step_num);							
						}
					}
				}
				time_stamp0 = time_stamp1;
			}
		}
	}
	
	return algo_step_num;
}
