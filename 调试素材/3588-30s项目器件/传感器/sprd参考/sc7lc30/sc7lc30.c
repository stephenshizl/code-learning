

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/irq.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
//#include <linux/wakelock.h>

#include <linux/io.h>
#include <linux/of_gpio.h>

#include <linux/poll.h>

#include "sc7lc30.h"

#define SC7LC30_ALS_SC_PARAMETER_L               5
#define SC7LC30_ALS_SC_PARAMETER_H               33
#define SC7LC30_ALS_SC_THRES                     1500  

#define SL_POLL_ALS
//#define SL_POLL_PS
#define SL_AUTO_CALI
#define CALI_PS_EVERY_TIME

#define LOW_PASS
#define PLS_DEBUG 0
#define AUTO_CALI
#define DRIVER_VERSION "1.0.1"
#if PLS_DEBUG
#define PLS_DBG(format, ...)	\
	printk(KERN_INFO "SC7LC30 " format "\n", ## __VA_ARGS__)
#else
	#define PLS_DBG(format, ...)
#endif



#define SL_ALS_FIR

#ifdef SL_ALS_FIR
	#define SL_FIR_LEN	4
	#define MAX_FIR_LEN 32
	
struct als_filter {
    u16 raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
};
#endif

#define SILAN_DEVICE_NAME		"Silan_alsps"
 
#define ALSPS_INPUT_NAME		"alps_pxy"

#ifdef SL_AUTO_CALI
	#define SL_MAX_MIN_DIFF	40   //djq
	#define SL_LT_N_CT	80
	#define SL_HT_N_CT	120
     
#endif

#define SL_PS_TIME_READ   20
#define PS_CALI_AFTER_NUM 2
struct input_dev *sl_input_dev = NULL;

struct sc7lc30_data{
	
	struct i2c_client *client;
	struct input_dev *input;

	ktime_t als_poll_delay;
	atomic_t	recv_reg;
    int32_t irq;
    struct work_struct sl_work;
	struct workqueue_struct *sl_wq;	
#ifdef SL_POLL_ALS		
	struct work_struct sl_als_work;
	struct hrtimer als_timer;	
	struct workqueue_struct *sl_als_wq;
#endif
#ifdef SL_AUTO_CALI
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *sl_ps_tune0_wq;
	struct work_struct sl_ps_tune0_work;
	ktime_t ps_tune0_delay;
	int sl_max_min_diff;
	int sl_lt_n_ct;
	int sl_ht_n_ct;
#endif

	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	struct mutex io_lock;
	//struct wake_lock ps_wakelock;
	int als_lux_last;
	int als_code_last;
	int ps_code_last;
	int ps_distance_last;
	uint8_t mode_reg;
	uint8_t	it_val;
	uint8_t	led_ctrl;
	uint8_t 	ps_gain;
	uint8_t	als_gain;
	uint8_t	wait_time;
	uint8_t wait_reg;
	uint8_t state_reg;
	uint8_t	config_val;
	uint8_t	command_val;
	uint8_t	pers_val;
	bool als_enabled;
	bool ps_enabled;
	bool re_enable_als;

#ifdef SL_ALS_FIR
	struct als_filter      fir;
	atomic_t                firlength;	
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend sl_early_suspend;
#endif

	struct sc7lc30_platform_data *pdata;

	int int_pin;
};

static struct i2c_client *sc7lc30_client = NULL;
static int     PS_CALI_AFTER_DOT_COUNT = 0;
static int     PS_CALI_DONE = 1;
static bool FIRST_ENABLE;
static struct sc7lc30_data *pls_data = NULL;
static uint32_t sc7lc30_alscode2lux(struct sc7lc30_data *ps_data, uint32_t alscode);
static int32_t sc7lc30_enable_als(struct sc7lc30_data *ps_data, uint8_t enable);
static int32_t sc7lc30_check_pid(struct sc7lc30_data *ps_data);
static int32_t sc7lc30_enable_ps(struct sc7lc30_data *ps_data, uint8_t enable);
static int32_t sc7lc30_get_nf_state(struct sc7lc30_data *ps_data);

static int sc7lc30_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	uint8_t retry;	
	int err;
	struct i2c_msg msgs[] = 
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = values,
		},
	};
	
	for (retry = 0; retry < 5; retry++) 
	{
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err == 2)
			break;
		else
			mdelay(5);
	}
	
	if (retry >= 5) 
	{
		PLS_DBG(KERN_ERR "%s: i2c read fail, err=%d\n", __func__, err);
		return -EIO;
	} 
	return 0;		
}

static int sc7lc30_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	int retry;
	int err;	
	unsigned char data[11];
	struct i2c_msg msg;
	int index;

    if (!client)
		return -EINVAL;
    else if (length >= 10) 
	{        
        PLS_DBG(KERN_ERR "%s:length %d exceeds 10\n", __func__, length);
        return -EINVAL;
    }   	
	
	data[0] = command;
	for (index = 1; index <= length; index++)
		data[index] = values[index-1];	
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = length+1;
	msg.buf = data;
	
	for (retry = 0; retry < 5; retry++) 
	{
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			mdelay(5);
	}
	
	if (retry >= 5) 
	{
		PLS_DBG(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);		
		return -EIO;
	}
	return 0;
}

static int sc7lc30_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;
	err = sc7lc30_i2c_read_data(client, command, 1, &value);
	if(err < 0)
		return err;
	return value;
}

static int sc7lc30_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
	int err;
	err = sc7lc30_i2c_write_data(client, command, 1, &value);
	return err;
}

static int sc7lc30_parse_dt(struct device *dev,struct sc7lc30_platform_data *pdata)
{	
	struct device_node *np = dev->of_node;
	PLS_DBG(KERN_INFO "%s\n",__func__);
	
	pdata->mode_reg = 0x01;
	pdata->it_val = 0x58;//0x7A; //wxs 0x78
	pdata->led_ctrl = 0xFF;
	pdata->ps_gain = 0x33;
	pdata->als_gain = 0x3B;
	pdata->config_val = 0x00;
	pdata->wait_time = 0x21; //djq,2019-01-09(0x21->0x0a) , not changed
	pdata->command_val	= 0x43;
	pdata->pers_val = 0x10;
	pdata->ps_thd_h = 0xFFFF;//150;   //djq,2019-03-07
	pdata->ps_thd_l = 0x0000;//100;   //djq,2019-03-07
	pdata->int_pin = of_get_gpio(np, 0);
	if (pdata->int_pin < 0) {
		PLS_DBG(KERN_ERR "%s: fail to get irq_gpio_number \n",__func__);
	}
	return 0;
}

static void sl_als_report(struct sc7lc30_data *ps_data, int als)
{
	ps_data->als_lux_last = als;
	input_report_abs(sl_input_dev, ABS_MISC, als);
	input_sync(sl_input_dev);

#ifdef PLS_DEBUG
	PLS_DBG(KERN_INFO "%s: als input event %d lux\n",__func__, als);
#endif
}

static int32_t sc7lc30_set_ps_thd_l(struct sc7lc30_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = sc7lc30_i2c_write_data(ps_data->client, SC7LC30_PX_LTHH, 2, val);		
	if(ret < 0)
		PLS_DBG(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;	
}
static int32_t sc7lc30_set_ps_thd_h(struct sc7lc30_data *ps_data, uint16_t thd_h)
{	
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = sc7lc30_i2c_write_data(ps_data->client, SC7LC30_PX_HTHH, 2, val);		
	if(ret < 0)
		PLS_DBG(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;
}

static int sc7lc30_get_flag(struct sc7lc30_data *ps_data)
{
	int ret;
    ret = sc7lc30_i2c_smbus_read_byte_data(ps_data->client,SC7LC30_PSDATA_READY);	
	if(ret < 0)
		PLS_DBG(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int sc7lc30_get_ps_reading(struct sc7lc30_data *ps_data)
{
	uint8_t org_flag_reg;
	unsigned char value[2];
	int ret;
	int i = 0;  

	for(i = 0; i < SL_PS_TIME_READ; i++)
	{
		org_flag_reg = sc7lc30_get_flag(ps_data);
		if((org_flag_reg & 0x10))
		{
			break;
		}else
		{
            mdelay(20);
			i++;
		}
	}	
		
	if(i == SL_PS_TIME_READ)
		return -1;

	ret = sc7lc30_i2c_read_data(ps_data->client, SC7LC30_PX_MSB, 2, &value[0]);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}

	ps_data->ps_code_last = (value[0]<<8) | value[1];
	PLS_DBG("wqj ---->>>>>  ps_data->ps_code_last =%d \n",ps_data->ps_code_last);
	return ps_data->ps_code_last;
}


static int sc7lc30_get_als_reading(struct sc7lc30_data *ps_data)
{
	
	uint8_t flag_reg;
	int32_t als_data;
#ifdef SL_ALS_FIR
	int index;	 
	int firlen = atomic_read(&ps_data->firlength);	 
#endif	
	unsigned char value[2];
	int ret;

	flag_reg = sc7lc30_get_flag(ps_data);
	if(flag_reg < 0)
		return flag_reg;		
	if(!(flag_reg&0x20))
	{
		PLS_DBG(KERN_INFO "%s: als is not ready\n", __func__);
		return -1;
	}
		ret = sc7lc30_i2c_read_data(ps_data->client, SC7LC30_ADC_MSB, 2, &value[0]);
		if(ret < 0)
		{
			PLS_DBG(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
			return ret;
		}
		als_data = (value[0]<<8) | value[1];
//		als_data = 2 * als_data; //djq,2019-01-09(0x01 reg,value 0x7B->0x7A)
		PLS_DBG("%s: raw als_data=%d\n", __func__, als_data);
		
		ps_data->als_code_last = als_data;
	
#ifdef SL_ALS_FIR
		if(ps_data->fir.number < firlen)
		{				 
			ps_data->fir.raw[ps_data->fir.number] = als_data;
			ps_data->fir.sum += als_data;
			ps_data->fir.number++;
			ps_data->fir.idx++;
		}
		else
		{
			index = ps_data->fir.idx % firlen;
			ps_data->fir.sum -= ps_data->fir.raw[index];
			ps_data->fir.raw[index] = als_data;
			ps_data->fir.sum += als_data;
			ps_data->fir.idx = index + 1;
			als_data = ps_data->fir.sum/firlen;
		}	
#endif	
		
		return als_data;	
}

#ifdef SL_POLL_ALS
static enum hrtimer_restart sl_als_timer_func(struct hrtimer *timer)
{
	struct sc7lc30_data *ps_data = pls_data;
	queue_work(ps_data->sl_als_wq, &ps_data->sl_als_work);	
	hrtimer_forward_now(&ps_data->als_timer, ps_data->als_poll_delay);
	return HRTIMER_RESTART;	
}

static void sl_als_poll_work_func(struct work_struct *work)
{
	struct sc7lc30_data *ps_data = pls_data;
	//unsigned long delay = msecs_to_jiffies(200);  //djq	
	int32_t reading = 0, reading_lux;

		

	reading = sc7lc30_get_als_reading(ps_data);
	if(reading < 0)
		return;
	 PLS_DBG("%s: als_data=%d\n", __func__, reading);

	reading_lux = sc7lc30_alscode2lux(ps_data, reading);

	{
		sl_als_report(ps_data, reading_lux);
		PLS_DBG("test 540 %s: als_data=%d\n", __func__, reading);
	}
	//schedule_delayed_work(&ps_data->work, delay);  //djq
}
#endif
static void sl_ps_report(struct sc7lc30_data *ps_data, int nf)
{
	ps_data->ps_distance_last = nf;
	input_report_abs(sl_input_dev, ABS_DISTANCE, nf);
	input_sync(sl_input_dev);
	PLS_DBG("%s:ps_distance_last = %d\n ",__func__,nf);
}


#ifdef SL_AUTO_CALI



static int sl_ps_tune_zero_func_fae(struct sc7lc30_data *ps_data)
{
	int32_t word_data;
	int diff;
	unsigned char value[2];
	int print_value = 0;
	
	PLS_DBG("%s: ============\n",__func__);
			
#ifdef CALI_PS_EVERY_TIME
	if(!(ps_data->ps_enabled))
#else
	if(ps_data->psi_set || !(ps_data->ps_enabled))
#endif
	{
		return 0;
	}		
	word_data = sc7lc30_get_ps_reading( ps_data);
	if(word_data <= 0)
	{
		//PLS_DBG(KERN_ERR "%s: incorrect word data (0)\n", __func__);
		return 0xFFFF;
	}
	
    if(PS_CALI_AFTER_DOT_COUNT < PS_CALI_AFTER_NUM)
	{
		PS_CALI_AFTER_DOT_COUNT += 1;
		return 0xFFFF;
	}	
	else
    
	{	
		if(word_data > ps_data->psa)
		{
			ps_data->psa = word_data;
			PLS_DBG(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);
		}
		if(word_data < ps_data->psi)
		{
			ps_data->psi = word_data;	
			PLS_DBG(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);	
		}	
		
		diff = ps_data->psa - ps_data->psi;
		if(diff > ps_data->sl_max_min_diff  &&  PS_CALI_DONE == 1)
		{
			ps_data->psi_set = ps_data->psi;	
			PLS_DBG("%s: FAE tune0 psa-psi(%d) > STK_DIFF found\n", __func__, diff);
#ifdef CALI_PS_EVERY_TIME
			ps_data->ps_thd_h = ps_data->psi + ps_data->sl_ht_n_ct;
			ps_data->ps_thd_l = ps_data->psi + ps_data->sl_lt_n_ct;
			PLS_DBG(KERN_INFO "%s: update thd, HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l); 	
#else
			ps_data->ps_thd_h = ps_data->psi + ps_data->sl_ht_n_ct;
			ps_data->ps_thd_l = ps_data->psi + ps_data->sl_lt_n_ct;
			PLS_DBG(KERN_INFO "%s: update thd, HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l); 		
#endif			
			sc7lc30_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
			sc7lc30_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
			PLS_DBG(KERN_INFO "%s: ps_data->ps_thd_h=%d, ps_data->ps_thd_l=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l); 
		
			sc7lc30_i2c_read_data(ps_data->client, 0x0b, 1, &value[0]);
			print_value = (int)(value[0] & 0x00ff);
			PLS_DBG(KERN_INFO "reg=0x0b, value=0x%2x\n",print_value);
		
			sc7lc30_i2c_read_data(ps_data->client, 0x0c, 1, &value[0]);
			print_value = (int)(value[0] & 0x00ff);
			PLS_DBG(KERN_INFO "reg=0x0c, value=0x%2x\n",print_value);
		
			sc7lc30_i2c_read_data(ps_data->client, 0x0d, 1, &value[0]);
			print_value = (int)(value[0] & 0x00ff);
			PLS_DBG(KERN_INFO "reg=0x0d, value=0x%2x\n",print_value);
		
			sc7lc30_i2c_read_data(ps_data->client, 0x0e, 1, &value[0]);
			print_value = (int)(value[0] & 0x00ff);
			PLS_DBG(KERN_INFO "reg=0x0e, value=0x%2x\n",print_value);
		    PS_CALI_DONE = 0;
			hrtimer_cancel(&ps_data->ps_tune0_timer);		
		}
	}	
	return 0;
}

static void sl_ps_tune0_work_func(struct work_struct *work)
{
	struct sc7lc30_data *ps_data = pls_data;
    PLS_DBG("%s: ============\n",__func__);
	sl_ps_tune_zero_func_fae(ps_data);
	return;
}

static int sc7lc30_get_int_flag(struct sc7lc30_data *ps_data)
{
	int ret;
    ret = sc7lc30_i2c_smbus_read_byte_data(ps_data->client,SC7LC30_INT_STATUS_REG);	
	if(ret < 0)
		PLS_DBG(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}


static int sl_als_int_handle(struct sc7lc30_data *ps_data,int als_data)
{
	int lux;

	ps_data->als_code_last = als_data;

	lux = sc7lc30_alscode2lux(ps_data, als_data);
	sl_als_report(ps_data, lux);	
	return 0;
}


static void sl_ps_int_handle(struct sc7lc30_data *ps_data,int data,int nf)
{
	sl_ps_report(ps_data, nf);
	PLS_DBG(KERN_INFO "%s: ps input event=%d, ps code=%d\n",__func__, nf, ps_data->ps_code_last);	
}


static void sl_work_func(struct work_struct *work)
{
	int32_t ret;
	int32_t org_flag_reg;
	struct sc7lc30_data *ps_data = pls_data;
	uint32_t reading;	
	int near_far_state;

	/* get reg00 for int state */	
	org_flag_reg = sc7lc30_get_int_flag(ps_data);
	if(org_flag_reg < 0)
		goto err_i2c_rw;	

	PLS_DBG(KERN_INFO "%s: flag=0x%x\n", __func__, org_flag_reg);

    if(org_flag_reg & 0x04)
    {
		reading = sc7lc30_get_als_reading(ps_data);
		if(reading < 0)
			goto err_i2c_rw;
		ret = sl_als_int_handle(ps_data, reading);
		if(ret < 0)
			goto err_i2c_rw;
    }
    if (org_flag_reg & 0x40)
    {
		reading = sc7lc30_get_ps_reading(ps_data);		
		if(reading < 0)
			goto err_i2c_rw;
		ps_data->ps_code_last = reading;


		near_far_state = sc7lc30_get_nf_state(ps_data);	
		sl_ps_int_handle(ps_data, reading, near_far_state);

	}


	usleep_range(1000, 2000);
    enable_irq(ps_data->irq);
	return;

err_i2c_rw:
	msleep(30);
	disable_irq(ps_data->irq);
	return;	
}
static irqreturn_t sl_oss_irq_handler(int irq, void *data)
{
	struct sc7lc30_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->sl_wq,&pData->sl_work);
	return IRQ_HANDLED;
}

static enum hrtimer_restart sl_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct sc7lc30_data *ps_data = pls_data;
	queue_work(ps_data->sl_ps_tune0_wq, &ps_data->sl_ps_tune0_work);	
	hrtimer_forward_now(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
#endif

static int sc7lc30_set_wq(struct sc7lc30_data *ps_data)
{
#ifdef SL_POLL_ALS	
    ps_data->sl_als_wq = create_singlethread_workqueue("sl_als_wq");
	INIT_WORK(&ps_data->sl_als_work, sl_als_poll_work_func);
	hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);		
	ps_data->als_timer.function = sl_als_timer_func;
#endif	


#ifdef SL_AUTO_CALI
	ps_data->sl_ps_tune0_wq = create_singlethread_workqueue("sl_ps_tune0_wq");
	INIT_WORK(&ps_data->sl_ps_tune0_work, sl_ps_tune0_work_func);
	hrtimer_init(&ps_data->ps_tune0_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	ps_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_tune0_timer.function = sl_ps_tune0_timer_func;
#endif

	ps_data->sl_wq = create_singlethread_workqueue("sl_wq");
	INIT_WORK(&ps_data->sl_work, sl_work_func);
	return 0;
}


static void sc7lc30_proc_plat_data(struct sc7lc30_data *ps_data, struct sc7lc30_platform_data *plat_data)
{
	uint8_t w_reg;
	
	ps_data->it_val = plat_data->it_val;
		
	ps_data->it_val = plat_data->it_val;
	ps_data->led_ctrl = plat_data->led_ctrl;
	ps_data->ps_gain = plat_data->ps_gain;
	ps_data->als_gain = plat_data->als_gain;
	ps_data->config_val = plat_data->config_val;
	ps_data->command_val = plat_data->command_val;
	ps_data->pers_val = plat_data->pers_val;
	ps_data->wait_time = plat_data->wait_time;	
	if(ps_data->wait_reg < 2)
	{
		PLS_DBG(KERN_WARNING "%s: wait_reg should be larger than 2, force to write 2\n", __func__);
		ps_data->wait_reg = 2;
	}
	else if (ps_data->wait_reg > 0xFF)
	{
		PLS_DBG(KERN_WARNING "%s: wait_reg should be less than 0xFF, force to write 0xFF\n", __func__);
		ps_data->wait_reg = 0xFF;		
	}
	
	if(ps_data->ps_thd_h == 0 && ps_data->ps_thd_l == 0)
	{
		ps_data->ps_thd_h = plat_data->ps_thd_h;
		ps_data->ps_thd_l = plat_data->ps_thd_l;		
	}
	
	
	w_reg = 0;
	
	w_reg |= 0x51;

#ifndef SL_POLL_ALS
	w_reg |= 0x31;
#endif
	ps_data->mode_reg = w_reg;
	return;
}
static int32_t sc7lc30_init_all_reg(struct sc7lc30_data *ps_data)
{
	int32_t ret;
	PLS_DBG(KERN_INFO "%s: sc7lc30_init_all_reg start\n", __func__);
    ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_INTC_CONFIG, ps_data->mode_reg);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_PX_IT_TIME, ps_data->it_val);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_ALS_GAIN, ps_data->als_gain);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_PS_GAIN, ps_data->ps_gain);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_WAIT_TIME, ps_data->wait_time);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_PERS, ps_data->pers_val);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	 ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_LED_CTL, ps_data->led_ctrl);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	 ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_CONFIG_REG, ps_data->config_val);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	 ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_COMMAND_REG, ps_data->command_val);
    if (ret < 0)
    {
        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }
	PLS_DBG(KERN_INFO "%s: sc7lc30_init_all_reg successfully\n", __func__);

//#ifdef SL_AUTO_CALI	
//	ps_data->psa = 0x0;
//	ps_data->psi = 0xFFFF;	
//#endif	
//	sc7lc30_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
//	sc7lc30_set_ps_thd_l(ps_data, ps_data->ps_thd_l);	
		
	return 0;	
}

#ifdef SL_POLL_ALS
static void sc7lc30_als_set_poll_delay(struct sc7lc30_data *ps_data)
{
	uint8_t als_it = ps_data->it_val & 0x0F;
		
	if(als_it == 0x8)
	{
		ps_data->als_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);	
	}
	else if(als_it == 0x9)
	{
		ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	}
	else if(als_it == 0xA)
	{
		ps_data->als_poll_delay = ns_to_ktime(220 * NSEC_PER_MSEC);		
	}
	else if(als_it == 0xB)
	{
		ps_data->als_poll_delay = ns_to_ktime(440 * NSEC_PER_MSEC);		
	}
	else if(als_it == 0xC)
	{
		ps_data->als_poll_delay = ns_to_ktime(880 * NSEC_PER_MSEC);		
	}
	else
	{
		ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);		
		PLS_DBG(KERN_INFO "%s: unknown ALS_IT=%d, set als_poll_delay=110ms\n", __func__, als_it);
	}	
}
#endif

static int32_t sc7lc30_init_all_setting(struct i2c_client *client, struct sc7lc30_platform_data *plat_data)
{
	int32_t ret;
	struct sc7lc30_data *ps_data = pls_data;

	ret = sc7lc30_check_pid(ps_data);
	if(ret < 0)
		return ret;
	sc7lc30_proc_plat_data(ps_data, plat_data);
	ret = sc7lc30_init_all_reg(ps_data);
	if(ret < 0)
		return ret;
#ifdef SL_POLL_ALS
	sc7lc30_als_set_poll_delay(ps_data);
#endif	
	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;		
	ps_data->re_enable_als = false;

				
#ifdef SL_AUTO_CALI		

	ps_data->psi_set = 0;		
	
#endif	
#ifdef SL_ALS_FIR
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
	atomic_set(&ps_data->firlength, SL_FIR_LEN);	
#endif
	atomic_set(&ps_data->recv_reg, 0);  
	ps_data->ps_distance_last = -1;
	ps_data->als_code_last = 100;
    return 0;
}

static int sc7lc30_device_ctl(struct sc7lc30_data *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable) {
		if(FIRST_ENABLE)
		{
			ret = sc7lc30_init_all_setting(ps_data->client, ps_data->pdata);
			if (ret < 0) {
			}
			FIRST_ENABLE = false;
		}
	} 
	else if (!enable) {
	
	    if (!ps_data->als_enabled && !ps_data->ps_enabled) {

		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				ps_data->als_enabled, ps_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d\n",
			enable);
	}
	return 0;
}

uint32_t sc7lc30_alscode2lux(struct sc7lc30_data *ps_data, uint32_t alscode)
{
/*	uint32_t als_value;
	if(alscode < SC7LC30_ALS_SC_THRES)
	{
	 	als_value = alscode * SC7LC30_ALS_SC_PARAMETER_L / 100;
	}
	else
	{
		als_value = SC7LC30_ALS_SC_THRES * SC7LC30_ALS_SC_PARAMETER_L / 100 + (alscode - SC7LC30_ALS_SC_THRES) * SC7LC30_ALS_SC_PARAMETER_H / 1000;
	}*/
	return alscode;
}


static ssize_t sl_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    int32_t ret;	
    ret = ps_data->ps_distance_last;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static int sc7lc30_wxs_reading(struct sc7lc30_data *ps_data)
{
	uint8_t org_flag_reg;
	unsigned char value[2];
	int ret;
    int i = 0;
	
	for(i = 0; i < SL_PS_TIME_READ; i++)
	{
		org_flag_reg = sc7lc30_get_flag(ps_data);
		if((org_flag_reg & 0x10))
		{
			break;
		}else
		{
            mdelay(20);
			i++;
		}
	}	
		
	if(i == SL_PS_TIME_READ)
		return -1;
		
	sc7lc30_i2c_read_data(ps_data->client, SC7LC30_PX_LSB, 2, &value[0]);
	sc7lc30_i2c_read_data(ps_data->client, SC7LC30_PX_MSB, 2, &value[1]);

	ret = (((value[1] & 0xff) << 8) + value[0]);
	return ret;
}

static ssize_t sl_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
	int32_t ret;	
	ret = sc7lc30_wxs_reading(ps_data);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}
static ssize_t sl_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    int32_t ret;	
    ret = ps_data->ps_thd_l;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t sl_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    int32_t ret;	
    ret = ps_data->ps_thd_h;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}
static ssize_t sl_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    uint8_t ret;
	uint8_t buffer;
	sc7lc30_i2c_read_data(ps_data->client,SC7LC30_INTC_CONFIG,1,&buffer);
    ret = buffer;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

#ifdef SL_AUTO_CALI
static ssize_t sl_ps_maxdiff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    int32_t ret;	
    ret = ps_data->sl_max_min_diff;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}
static ssize_t sl_ps_ltnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    int32_t ret;	
    ret = ps_data->sl_lt_n_ct;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}
static ssize_t sl_ps_htnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;
    int32_t ret;	
    ret = ps_data->sl_ht_n_ct;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}
#endif
static ssize_t sl_all_reg_show(struct device* dev, struct device_attribute *attr, char *buf)
{
    struct sc7lc30_data *ps_data =  pls_data;    
    ssize_t ret = 0;
	char i = 0;
	uint8_t val = 0;

	for(i = 0;i <= 0x34;i++){
		sc7lc30_i2c_read_data(ps_data->client,i,1,&val);
		ret += sprintf(&buf[ret], "0x%x: 0x%x\n", i, val);
	}	

	return ret;
}
static ssize_t sl_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ps_data->recv_reg));     		
}


static ssize_t sl_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;	
	struct sc7lc30_data *ps_data = pls_data;	
	
	if((ret = kstrtoul(buf, 16, &value)) < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	recv_data = sc7lc30_i2c_smbus_read_byte_data(ps_data->client,value);
	atomic_set(&ps_data->recv_reg, recv_data);
	return size;
}

static ssize_t sl_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	int32_t ret, i;
	char *token[10];
	struct sc7lc30_data *ps_data =  pls_data;	
	
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = kstrtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if((ret = kstrtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	PLS_DBG(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);		

	ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, (unsigned char)addr, (unsigned char)cmd);
	if (0 != ret)
	{	
		PLS_DBG(KERN_ERR "%s: sc7lc30_i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}
	
	return size;
}


static ssize_t sl_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	ps_data->ps_distance_last = value;
	sl_ps_report(ps_data, value);
	PLS_DBG(KERN_INFO "%s: ps input event=%d\n",__func__, (int)value);		
    return size;
}
static ssize_t sl_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	ps_data->ps_thd_l = value;	
	sc7lc30_set_ps_thd_l( ps_data,ps_data->ps_thd_l);
    return size;
}
static ssize_t sl_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	ps_data->ps_thd_h = value;	
	sc7lc30_set_ps_thd_h( ps_data,ps_data->ps_thd_h);
    return size;
}

#ifdef SL_AUTO_CALI
static ssize_t sl_ps_maxdiff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	ps_data->sl_max_min_diff = value;
    return size;
}
static ssize_t sl_ps_ltnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	ps_data->sl_lt_n_ct= value;
    return size;
}
static ssize_t sl_ps_htnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data =  pls_data;	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	ps_data->sl_ht_n_ct= value;
    return size;
}
#endif

static ssize_t sl_sensor_show_als_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ret;
	struct sc7lc30_data *ps_data = pls_data;
    ret = ps_data->als_enabled?1:0;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t sl_sensor_store_als_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint16_t mode=0;
	struct sc7lc30_data *ps_data = pls_data;
	PLS_DBG("buf=%s\n", buf);
	sscanf(buf, "%hu",&mode);
	sc7lc30_enable_als(ps_data, mode);

	return count;
}

static ssize_t sl_sensor_show_ps_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ret;
	struct sc7lc30_data *ps_data = pls_data;
    ret = ps_data->ps_enabled?1:0;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t sl_sensor_store_ps_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct sc7lc30_data *ps_data = pls_data;
	uint16_t ps_enable = 0;
	uint8_t en;
	
	sscanf(buf, "%hu",&ps_enable);
	if(ps_enable)
		en = 1;
	else
		en = 0;

	PLS_DBG(KERN_INFO "%s: Enable PS : %d\n", __func__, en);
	mutex_lock(&ps_data->io_lock);
	sc7lc30_enable_ps(ps_data, en);
	mutex_unlock(&ps_data->io_lock);
	ps_data->ps_enabled = en?true:false;
	return size;
}

static DEVICE_ATTR(als_enable, 0664, sl_sensor_show_als_enable, sl_sensor_store_als_enable);
static DEVICE_ATTR(ps_enable,0664,sl_sensor_show_ps_enable,sl_sensor_store_ps_enable);
static DEVICE_ATTR(ps_distance,0664,sl_ps_distance_show, sl_ps_distance_store);
static DEVICE_ATTR(ps_code, 0444, sl_ps_code_show, NULL);
static DEVICE_ATTR(ps_code_thd_l,0664,sl_ps_code_thd_l_show,sl_ps_code_thd_l_store);
static DEVICE_ATTR(ps_code_thd_h,0664,sl_ps_code_thd_h_show,sl_ps_code_thd_h_store);
static DEVICE_ATTR(ps_recv,0664,sl_recv_show,sl_recv_store);
static DEVICE_ATTR(ps_send,0664,NULL, sl_send_store);
static DEVICE_ATTR(all_reg, 0444, sl_all_reg_show, NULL);
static DEVICE_ATTR(status, 0444, sl_status_show, NULL);
#ifdef SL_AUTO_CALI
static DEVICE_ATTR(ps_maxdiff,0664,sl_ps_maxdiff_show, sl_ps_maxdiff_store);
static DEVICE_ATTR(ps_ltnct,0664,sl_ps_ltnct_show, sl_ps_ltnct_store);
static DEVICE_ATTR(ps_htnct,0664,sl_ps_htnct_show, sl_ps_htnct_store);
#endif
static DEVICE_ATTR(als, 0660, sl_sensor_show_als_enable, sl_sensor_store_als_enable);
static DEVICE_ATTR(proximity, 0660, sl_sensor_show_ps_enable, sl_sensor_store_ps_enable);

static struct attribute *sl_sensor_attr_list[] =
{
	&dev_attr_als_enable.attr,
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_distance.attr,
	&dev_attr_ps_code.attr,
	&dev_attr_ps_code_thd_l.attr,
	&dev_attr_ps_code_thd_h.attr,	
	&dev_attr_ps_recv.attr,
	&dev_attr_ps_send.attr,	
	&dev_attr_all_reg.attr,
	&dev_attr_status.attr,
#ifdef SL_AUTO_CALI  
	&dev_attr_ps_maxdiff.attr,
	&dev_attr_ps_ltnct.attr,
	&dev_attr_ps_htnct.attr,
#endif	
	NULL
};

static struct attribute_group sl_ps_attribute_group = {
	.attrs = sl_sensor_attr_list,
};

static int sc7lc30_set_input_devices(struct sc7lc30_data *ps_data)
{
	int err;
  struct input_dev *input_dev = NULL;
	input_dev = input_allocate_device();
	if (!input_dev)
	{
		PLS_DBG(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		return err;
	}
	input_dev->name = ALSPS_INPUT_NAME;
	input_dev->phys = ALSPS_INPUT_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &sc7lc30_client->dev;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0010;
	ps_data->input = input_dev;
	sl_input_dev = input_dev;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0,1, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, 0, 65535, 0, 0);
	err = input_register_device(input_dev);
	if (err<0)
	{
		PLS_DBG(KERN_ERR "%s: can not register als input device\n", __func__);		
		goto err_als_input_register;
	}
	err = sysfs_create_group(&ps_data->input->dev.kobj, &sl_ps_attribute_group);
	if (err < 0) 
	{
		PLS_DBG(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		goto err_ps_create_group;
	}
	input_set_drvdata(ps_data->input, ps_data);
	return 0;
err_ps_create_group:	
	sysfs_remove_group(&ps_data->input->dev.kobj, &sl_ps_attribute_group);
err_als_input_register:
	input_free_device(ps_data->input);

	return err;
}

static int32_t sc7lc30_check_pid(struct sc7lc30_data *ps_data)
{

	PLS_DBG("%s: sc7lc30 is working\n",__func__);
	return 0;
}

static int sc7lc30_setup_irq(struct i2c_client *client)
{		
	int irq, err = -EIO;
	struct sc7lc30_data *ps_data = pls_data;
	irq = gpio_to_irq(ps_data->int_pin);
	PLS_DBG(KERN_INFO "%s: int pin #=%d, irq=%d\n",__func__, ps_data->int_pin, irq);	
	if (irq <= 0)
	{
		PLS_DBG(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;	
	err = gpio_request(ps_data->int_pin,"sl-int");        
	if(err < 0)
	{
		PLS_DBG(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
		return err;
	}	
	
	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		PLS_DBG(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
		return err;
	}		

	err = request_irq(irq, sl_oss_irq_handler, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND, SILAN_DEVICE_NAME, ps_data);
	if (err < 0) 
	{
		PLS_DBG(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);		
		goto err_request_any_context_irq;
	}
	disable_irq(irq);
	
	return 0;
err_request_any_context_irq:	
	
	gpio_free(ps_data->int_pin);		

	return err;
}

static int32_t sc7lc30_set_state(struct sc7lc30_data *ps_data, uint8_t state)
{
	int ret;		
    ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client,SC7LC30_INTC_CONFIG, state);	
	if(ret < 0)
		PLS_DBG(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t sc7lc30_get_state(struct sc7lc30_data *ps_data)
{
	int ret;
    ret = sc7lc30_i2c_smbus_read_byte_data(ps_data->client,SC7LC30_INTC_CONFIG);	
	if(ret < 0)
		PLS_DBG(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t sc7lc30_enable_als(struct sc7lc30_data *ps_data, uint8_t enable)
{
	int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable;
	curr_als_enable = enable;

	if(curr_als_enable == ps_data->als_enabled)
	{
		PLS_DBG("sc7lc30_enable_als: als has already enable\n");
		return 0;
	}
		
	if (enable)
	{
		ret = sc7lc30_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}	

	ret = sc7lc30_get_state(ps_data);
	if(ret < 0)
		return ret;
	
	w_state_reg = ret;
	if(enable)
	{
		w_state_reg |= 0x03;	
		if (ps_data->ps_enabled)		
			w_state_reg |= 0x07;	
	
	ret = sc7lc30_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;	
	ps_data->state_reg = w_state_reg;
	}
    if (enable)
    {	
		ps_data->als_enabled = true;
#ifdef SL_POLL_ALS			
		hrtimer_start(&ps_data->als_timer, ps_data->als_poll_delay, HRTIMER_MODE_REL);	

#endif		
    }
	else
	{
		ps_data->als_enabled = false;
#ifdef SL_POLL_ALS			
		hrtimer_cancel(&ps_data->als_timer);
		cancel_work_sync(&ps_data->sl_als_work);
		
#endif
		w_state_reg &= (~(0x02));
				
		ret = sc7lc30_set_state(ps_data, w_state_reg);
		if(ret < 0)
			return ret;	
		ps_data->state_reg = w_state_reg;		
				
	}
    return ret;
}

static int32_t sc7lc30_get_nf_state(struct sc7lc30_data *ps_data)
{
	uint8_t nf_state = -1;	
	int ret;
	ret = sc7lc30_i2c_read_data(ps_data->client, SC7LC30_NF_FLAG_REG, 1, &nf_state);
	if(ret < 0)
	{
		PLS_DBG(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}	
	nf_state = nf_state & 0x01;	
	PLS_DBG(KERN_ERR "%s ok, nf_state=0x%x", __func__, nf_state);
	return nf_state;
}

static int32_t sc7lc30_enable_ps(struct sc7lc30_data *ps_data, uint8_t enable)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;	
	uint32_t reading;
	int32_t near_far_state;		
	PLS_DBG("%s:sc7lc30_enable_ps dongjianqing judge enable ps\n",__func__);
	
	if (enable) {
		ret = sc7lc30_device_ctl(ps_data, enable);  //power on platform ld
		if (ret)
			return ret;
	}	
	curr_ps_enable = enable;	
	if(curr_ps_enable == ps_data->ps_enabled)
		PLS_DBG("%s:robert  ps is already enable\n",__func__);
	
#ifdef SL_AUTO_CALI
	if (!(ps_data->psi_set) && !enable)
	{
		hrtimer_cancel(&ps_data->ps_tune0_timer);					
		cancel_work_sync(&ps_data->sl_ps_tune0_work);
	}
#endif		

    if(enable)
	{
		sc7lc30_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		sc7lc30_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
	}

	ret = sc7lc30_get_state(ps_data);
	if(ret < 0)
		return ret;
	w_state_reg = ret;
	
	if(enable)	
	{
		ps_data->ps_enabled = true;		
		w_state_reg |= 0x55;
		ret = sc7lc30_i2c_smbus_write_byte_data(ps_data->client, SC7LC30_COMMAND_2_REG, 0x10);
	    if (ret < 0)
	    {
	        PLS_DBG(KERN_ERR "%s: write i2c error\n", __func__);
	        return ret;
	    }		
		if((ps_data->als_enabled))
			w_state_reg |= 0x03;			
		
	ret = sc7lc30_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;	
	ps_data->state_reg = w_state_reg;
		
#ifdef SL_AUTO_CALI
	#ifdef CALI_PS_EVERY_TIME
		ps_data->psi_set = 0;
		ps_data->psa = 0;
		ps_data->psi = 0xFFFF;
		PLS_DBG("%s: ==========%d==\n",__func__,__LINE__);
		hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);					
	#else
		if (!(ps_data->psi_set))
			hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);			
	#endif	/* #ifdef CALI_PS_EVERY_TIME */
#endif
		{
			//usleep_range(4000, 5000);
			mdelay(10);
			reading = sc7lc30_get_ps_reading(ps_data);			
			if (reading < 0)
				return reading;	
		
			near_far_state = sc7lc30_get_nf_state(ps_data);
			if (near_far_state < 0)
				return near_far_state;				
			
			sl_ps_report(ps_data, near_far_state);
			PLS_DBG(KERN_INFO "%s: ps input event=%d, ps=%d\n",__func__, near_far_state, reading);
		}
#ifndef SL_POLL_ALS		
			if(!(ps_data->als_enabled))
#endif	
			//disable_irq_nosync(ps_data->irq);
			enable_irq(ps_data->irq);							
		
				

		PLS_DBG(KERN_INFO "%s: HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
	}
	else
	{
		ps_data->ps_enabled = false;
#ifndef SL_POLL_ALS
		if(!(ps_data->als_enabled))	
#endif	
			disable_irq_nosync(ps_data->irq);
			
			
		ret = sc7lc30_device_ctl(ps_data, enable);
		if (ret)
			return ret;		
		w_state_reg &= (~(0x04));
				
		ret = sc7lc30_set_state(ps_data, w_state_reg);
		if(ret < 0)
			return ret;	
		ps_data->state_reg = w_state_reg;		
	}
	return ret;
}

static int sc7lc30_pls_open(struct inode *inode, struct file *file)
{
	int err;
	PLS_DBG("%s\n", __func__);
	err = nonseekable_open(inode, file);
	if (err < 0)
	{
		PLS_DBG("%s: open fail!\n", __func__);
		return err;
	}

	file->private_data = i2c_get_clientdata(sc7lc30_client);
	return 0;
}

static int sc7lc30_pls_release(struct inode *inode, struct file *file)
{
	PLS_DBG("%s", __func__);
	return 0;//sc7lc30_pls_disable(SC7LC30_PLS_BOTH);
}

static long sc7lc30_pls_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;
	unsigned char data;
	int ret;
	struct sc7lc30_data *ps_data =  pls_data;
	PLS_DBG("%s: cmd %d", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case SL_IOCTL_GET_PFLAG:
		flag = ps_data->ps_enabled? 1:0;
		if (copy_to_user(argp, &flag, sizeof(flag))) {
			return -EFAULT;
		}
		PLS_DBG("%s: get flag=%d", __func__, flag);
		break;
	case SL_IOCTL_GET_LFLAG:
		flag = ps_data->als_enabled? 1:0;
		if (copy_to_user(argp, &flag, sizeof(flag))) {
			return -EFAULT;
		}
		PLS_DBG("%s: get flag=%d", __func__, flag);
		break;
	case SL_IOCTL_GET_DATA:
		ret = sc7lc30_i2c_read_data(ps_data->client, SC7LC30_ADC_MSB, 2, &data);
		if(ret < 0)
		{
			PLS_DBG(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
			return ret;
		}
		if (copy_to_user(argp, &data, sizeof(data))) {
			return -EFAULT;
		}
		PLS_DBG("%s: get data=%d", __func__, flag);
		break;
	case SL_IOCTL_SET_PFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		PLS_DBG("%s: set flag=%d", __func__, flag);
		if (flag < 0 || flag > 1) {
			return -EINVAL;
		}
		//ps_data->ps_enabled = flag?1:0;
		mutex_lock(&ps_data->io_lock);
		ret = sc7lc30_enable_ps(ps_data, flag);			
		mutex_unlock(&ps_data->io_lock);
		break;
	case SL_IOCTL_SET_LFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		PLS_DBG("%s: set flag=%d", __func__, flag);
		if (flag < 0 || flag > 1) {
			return -EINVAL;
		}
		//ps_data->ps_enabled = flag?1:0;
		mutex_lock(&ps_data->io_lock);
		ret = sc7lc30_enable_als(ps_data, flag);
		mutex_unlock(&ps_data->io_lock);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}
	return 0;
}

static struct file_operations sc7lc30_pls_fops = {
	.owner				= THIS_MODULE,
	.open				= sc7lc30_pls_open,
	.release			= sc7lc30_pls_release,
	.unlocked_ioctl		= sc7lc30_pls_ioctl,
};

static struct miscdevice sc7lc30_pls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SILAN_DEVICE_NAME,
	.fops = &sc7lc30_pls_fops,
};

static int sc7lc30_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	int err = -ENODEV;
	struct sc7lc30_data *ps_data;
	struct sc7lc30_platform_data *plat_data;
	struct class *pls_class;
	struct device *pls_cmd_dev;
	PLS_DBG(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);

  if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
  {
      PLS_DBG(KERN_ERR "%s: No Support for I2C_FUNC_I2C\n", __func__);
      return -ENODEV;
  }

	ps_data = kzalloc(sizeof(struct sc7lc30_data),GFP_KERNEL);
	if(!ps_data)
	{
		PLS_DBG(KERN_ERR "%s: failed to allocate sc7lc30_data\n", __func__);
		return -ENOMEM;
	}
	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	sc7lc30_client = client;  //djq
	mutex_init(&ps_data->io_lock);
	//wake_lock_init(&ps_data->ps_wakelock,WAKE_LOCK_SUSPEND, "sl_input_wakelock");

	pls_data = ps_data;
	if (client->dev.of_node) {
		PLS_DBG(KERN_INFO "%s: probe with device tree\n", __func__);
		plat_data = devm_kzalloc(&client->dev,
			sizeof(struct sc7lc30_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = sc7lc30_parse_dt(&client->dev, plat_data);
		if (err)
		{
			dev_err(&client->dev,
				"%s: sc7lc30_parse_dt ret=%d\n", __func__, err);
			return err;
		}

	} else {
		PLS_DBG(KERN_INFO "%s: probe with platform data\n", __func__);	

		plat_data = client->dev.platform_data;	
	
	}
	if (!plat_data) {
		dev_err(&client->dev,
			"%s: no sc7lc30 platform data!\n", __func__);
		goto err_als_input_allocate;
	}
	ps_data->int_pin = plat_data->int_pin;
	ps_data->pdata = plat_data;	
	sc7lc30_init_all_reg(ps_data);

	err = sc7lc30_set_input_devices(ps_data);
	if(err < 0)
		goto err_init_all_setting;	

	sc7lc30_set_wq(ps_data);
	ps_data->ps_thd_h = 0xFFFF; //djq
	ps_data->ps_thd_l = 0x0000;	
#ifdef SL_AUTO_CALI
	ps_data->sl_max_min_diff = SL_MAX_MIN_DIFF;
	ps_data->sl_lt_n_ct = SL_LT_N_CT;
	ps_data->sl_ht_n_ct = SL_HT_N_CT;
#endif	

	err = sc7lc30_check_pid(ps_data);
	if(err < 0)
		goto err_init_all_setting;
		
	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;

	FIRST_ENABLE = true;
	err = misc_register(&sc7lc30_pls_device);
	if (err) {
		PLS_DBG("%s: sc7lc30_pls_device register failed\n", __func__);
		goto err_setup_input_device;
	}
	
	err = sc7lc30_setup_irq(client);
	if(err < 0)
		goto err_sc7lc30_setup_irq;

	pls_class = class_create(THIS_MODULE, "xr-pls");
	if (IS_ERR(pls_class))
		PLS_DBG("Failed to create class(xr-pls)!\n");
	pls_cmd_dev = device_create(pls_class, NULL, 0, NULL, "device");
	if (IS_ERR(pls_cmd_dev))
		PLS_DBG("Failed to create device(pls_cmd_dev)!\n");
	if(device_create_file(pls_cmd_dev, &dev_attr_proximity) < 0) // /sys/class/xr-pls/device/proximity
	{
		PLS_DBG("Failed to create device file(%s)!\n", dev_attr_proximity.attr.name);
	}
	if(device_create_file(pls_cmd_dev, &dev_attr_als) < 0) // /sys/class/xr-pls/device/als
	{
		PLS_DBG("Failed to create device file(%s)!\n", dev_attr_als.attr.name);
	}

	
	//sc7lc30_enable_ps(ps_data, 1);
	
	//sc7lc30_enable_als(ps_data, 1);
	
	PLS_DBG(KERN_INFO "%s: probe successfully\n", __func__);
	return 0;
	
err_sc7lc30_setup_irq:
	free_irq(ps_data->irq, ps_data);

	//free_gpio_irq(ps_data->int_pin);		
err_setup_input_device:
	misc_deregister(&sc7lc30_pls_device);
err_init_all_setting:
#ifdef SL_POLL_ALS		
	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->sl_als_wq);	
#endif
#ifdef SL_AUTO_CALI
	destroy_workqueue(ps_data->sl_ps_tune0_wq);	
#endif			
	destroy_workqueue(ps_data->sl_wq);			
err_als_input_allocate:
    //wake_lock_destroy(&ps_data->ps_wakelock);	
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
    return err;
}

static int sc7lc30_remove(struct i2c_client *client)
{
	struct sc7lc30_data *ps_data = pls_data;
	
	device_init_wakeup(&client->dev, false);

	misc_deregister(&sc7lc30_pls_device);
	free_irq(ps_data->irq, ps_data);
	#ifdef SPREADTRUM_PLATFORM	
		sprd_free_gpio_irq(ps_data->int_pin);		
	#else	
		gpio_free(ps_data->int_pin);	
	#endif		

#ifdef SL_POLL_ALS		
	hrtimer_try_to_cancel(&ps_data->als_timer);	
	destroy_workqueue(ps_data->sl_als_wq);		
#endif	
#ifdef SL_AUTO_CALI
	destroy_workqueue(ps_data->sl_ps_tune0_wq);	
#endif		
	//wake_lock_destroy(&ps_data->ps_wakelock);	
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
	
    return 0;
}
static int sc7lc30_suspend(struct device *dev)
{
	struct sc7lc30_data *ps_data = pls_data;
#if ( !defined(SL_POLL_PS))
	int err;
#endif
		
    struct i2c_client *client = to_i2c_client(dev);	

	PLS_DBG(KERN_INFO "%s\n", __func__);	
	mutex_lock(&ps_data->io_lock);  	

	if(ps_data->als_enabled)
	{
		PLS_DBG(KERN_INFO "%s: Enable ALS : 0\n", __func__);
		sc7lc30_enable_als(ps_data, 0);		
		ps_data->re_enable_als = true;
	}

	if(ps_data->ps_enabled)
	{		
		if(device_may_wakeup(&client->dev))
		{
			err = enable_irq_wake(ps_data->irq);	
			if (err)
				PLS_DBG(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);				
		}
		else
		{
			PLS_DBG(KERN_ERR "%s: not support wakeup source\n", __func__);
		}
	}
	mutex_unlock(&ps_data->io_lock);	
	pr_err("stephen sc7lc30_suspend\n");	
	return 0;	
}

static int sc7lc30_resume(struct device *dev)
{
	struct sc7lc30_data *ps_data = pls_data;	
#if (!defined(SL_POLL_PS))	
	int err;
#endif
    struct i2c_client *client = to_i2c_client(dev);	
	
	PLS_DBG(KERN_INFO "%s\n", __func__);	
	mutex_lock(&ps_data->io_lock); 		
 	
 		
	if(ps_data->re_enable_als)
	{
		PLS_DBG(KERN_INFO "%s: Enable ALS : 1\n", __func__);		
		sc7lc30_enable_als(ps_data, 1);		
		ps_data->re_enable_als = false;		
	}
 

	if(ps_data->ps_enabled)
	{
		if(device_may_wakeup(&client->dev))
		{	
			err = disable_irq_wake(ps_data->irq);	
			if (err)		
				PLS_DBG(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);		
		}		
	}
	mutex_unlock(&ps_data->io_lock);
 	pr_err("stephen sc7lc30_resume \n");	
	return 0;	
}

static const struct dev_pm_ops sc7lc30_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(sc7lc30_suspend, sc7lc30_resume)
};

static const struct i2c_device_id silan_ps_id[] =
{
    { "Silan_alsps", 0},  //{ SILAN_DEV_NAME, 0}, 
    {}
};
MODULE_DEVICE_TABLE(i2c, silan_ps_id);

static struct of_device_id silan_match_table[] = {
	{ .compatible = "SILAN,Silan_alsps", },  //sc7lc30_pls
	{ },
};

static struct i2c_driver sc7lc30_ps_driver =
{
    .probe = sc7lc30_probe,
    .remove = sc7lc30_remove,
    .id_table = silan_ps_id,
    .driver = {
        .name = SILAN_DEVICE_NAME,
		.owner = THIS_MODULE,	
#ifdef CONFIG_OF		
		.of_match_table = silan_match_table,		
#endif
    .pm = &sc7lc30_pm_ops,				
    },

};

static int __init sc7lc30_init(void)
{
	int ret;
    ret = i2c_add_driver(&sc7lc30_ps_driver);
    if (ret)
	{
		i2c_del_driver(&sc7lc30_ps_driver);
        return ret;
	}
    return 0;
}

static void __exit sc7lc30_exit(void)
{
    i2c_del_driver(&sc7lc30_ps_driver);	
}

module_init(sc7lc30_init);
module_exit(sc7lc30_exit);

MODULE_AUTHOR("Jianqing Dong<dongjainqing@silan.com.cn>");
MODULE_DESCRIPTION("Silan sc7lc30 Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);


