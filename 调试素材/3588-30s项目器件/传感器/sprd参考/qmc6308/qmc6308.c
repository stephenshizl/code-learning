/* drivers/misc/qmc6308.c - qmc compass driver
 *
 * Copyright (C) qst Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
//#include <linux/i2c/qmc6308.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/poll.h>

#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include "qmc6308.h"
#if 0
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#endif

//#define QMC7983  		0x32
//#define QMC6308 		0x31
//#define QMC6308_A1_D1   0xff

#define DEBUG 0
#define qmc_debug
#define	QMC6308_BUFSIZE		0x20

#define MSE_TAG					"[Msensor] "
#define MSE_FUN(f)				printk(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_ERR(fmt, args...)		printk(KERN_ERR MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)		printk(MSE_TAG fmt, ##args)
/*
 * QMC6308 magnetometer data
 * brief Structure containing magnetic field values for x,y and z-axis in
 * signed short
*/
//static int KxInt;
//static int KyInt;
static unsigned char v_open_flag = 0x00;
static unsigned char maskid = 0x00;
//static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

struct QMC6308_t {
	short	x, /**< x-axis magnetic field data. Range -8000 to 8000. */
			y, /**< y-axis magnetic field data. Range -8000 to 8000. */
			z; /**< z-axis magnetic filed data. Range -8000 to 8000. */
};

/* Save last device state for power down */
struct qmc_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct qmc6308_data{
	struct i2c_client 				*client;
	struct qmc6308_platform_data 	*pdata;

	struct device					*class_dev;
	struct class					*compass;
	struct input_dev 				*input;
	struct delayed_work 			work;
	
	struct mutex 					lock;
	
	//short 							xy_sensitivity;
	//short 							z_sensitivity;
	//int								OTP_Kx;
	//int 							OTP_Ky;
	
	int 							msec_delay;//msec
	char 							layout;
	int 							menabled;
	int 							oenabled;
	
	struct completion 				data_updated;
	wait_queue_head_t 				state_wq;
	
	struct regulator 				*vdd;
	struct regulator 				*vio;
	struct qmc_sensor_state			state;
};

static struct qmc6308_data *mag;


static char qmc6308_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char qmcX6983_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

int qmc6308_read_mag_xyz(struct QMC6308_t *data);


/*  i2c write routine for qmc6308 magnetometer */
static char qmc6308_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int dummy;
	int i;

	if (mag->client == NULL)  /*  No global client pointer? */
		return -1;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(mag->client,
						  reg_addr++, data[i]);
		if (dummy) {
			printk( "i2c write error\n");
			return dummy;
		}
	}
	return 0;
}

/*  i2c read routine for QMC6308 magnetometer */
static char qmcX6983_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	char dummy = 0;
	int i = 0;

	if (mag->client == NULL)  /*  No global client pointer? */
		return -1;

	while (i < len) 
   {
		dummy = i2c_smbus_read_byte_data(mag->client,
						 reg_addr++);
//		printk( "read - %d\n", dummy);
		if (dummy >= 0) {
			data[i] = dummy;
			i++;
		} else {
			printk("i2c read error\n ");
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}


int qmc6308_self_test(char mode, short* buf)
{
	return 0;
}


int qmc6308_read_mag_xyz(struct QMC6308_t *data)
{
	int res;
	unsigned char mag_data[6];
	int hw_d[3] = { 0 };

	unsigned char data_A;
	unsigned char data_B;
	unsigned char data_D;

	qmcX6983_i2c_read(0x0a, &data_A, 1);
	qmcX6983_i2c_read(0x0b, &data_B, 1);
	qmcX6983_i2c_read(0x0d, &data_D, 1);
	//MSE_LOG( "Read Reg 0a=0x%x, 0b=0x%x, 0d=0x%x\n",data_A,data_B,data_D);

	res = qmcX6983_i2c_read(OUT_X_REG, mag_data, 6);

//	MSE_LOG( "mag_data[%d, %d, %d, %d, %d, %d]\n",
//	mag_data[0], mag_data[1], mag_data[2],
//	mag_data[3], mag_data[4], mag_data[5]);

	hw_d[0] = (short) (((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short) (((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short) (((mag_data[5]) << 8) | mag_data[4]);


	hw_d[0] = hw_d[0];
	hw_d[1] = hw_d[1];
	hw_d[2] = hw_d[2];

	//MSE_LOG( "Hx=%d, Hy=%d, Hz=%d\n",hw_d[0],hw_d[1],hw_d[2]);

	data->x = hw_d[0];
	data->y = hw_d[1];
	data->z = hw_d[2];

	return res;
}

/* Set the Gain range */
/*
int qmc6308_set_range(short range)
{
	int err = 0;
	int ran ;
	switch (range) {
	case QMC6308_RNG_2G:
		ran = RNG_2G;
		break;
	case QMC6308_RNG_8G:
		ran = RNG_8G;
		break;
	case QMC6308_RNG_12G:
		ran = RNG_12G;
		break;
	case QMC6308_RNG_20G:
		ran = RNG_20G;
		break;
	default:
		return -EINVAL;
	}
	mag->xy_sensitivity = 20000/ran;
	mag->z_sensitivity = 20000/ran;

	return err;
}
*/

/* Set the sensor mode */
int qmc6308_set_mode(char mode)
{
	int err = 0;
	unsigned char data;

	qmcX6983_i2c_read(CTL_REG_ONE, &data, 1);
	data &= 0xfc;
	data |= mode;
	err = qmc6308_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}
/*
int qmc6308_set_ratio(char ratio)
{
	int err = 0;
	unsigned char data;

	data = ratio;
	err = qmc6308_i2c_write(RATIO_REG, &data, 1);
	return err;
}

int qmc6308_set_output_data_rate(char rate)
{
	int err = 0;
	unsigned char data;

	data = rate;
	data &= 0xf3;
	data |= (rate << 2);
	err = qmc6308_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}

int qmc6308_set_oversample_ratio(char ratio)
{
	int err = 0;
	unsigned char data;

	data = ratio;
	data &= 0x3f;
	data |= (ratio << 6);
	err = qmc6308_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}
*/

static void qmc6308_start_measure(struct qmc6308_data *qmc6308)
{
	int err = 0;
	unsigned char data;
	//MSE_LOG( "###qmc6308_start_measure###\n");
	// soft reset
	//data = 0x80;
	//err = qmc6308_i2c_write(CTL_REG_TWO, &data, 1);
	//mdelay(5);
	// soft reset
	if(1)//(maskid==0x80)
	{
		data = 0x40;
		err = qmc6308_i2c_write(0x0d, &data, 1);
		MSE_LOG( "qmc6308_i2c_write 0x0d return %d\n",err);
		mdelay(5);
		data = 0x00;
		err = qmc6308_i2c_write(CTL_REG_TWO, &data, 1);
		mdelay(5);
		data = 0xc3;
	}
	else{
		data = 0x40;
		err = qmc6308_i2c_write(0x0d, &data, 1);
		data = 0x08;
		err = qmc6308_i2c_write(CTL_REG_TWO, &data, 1);
		data = 0x63;
	}
		
	MSE_LOG( "qst qmc6308_start_measure data-->%0x\n",data);
	err = qmc6308_i2c_write(CTL_REG_ONE, &data, 1);
	mdelay(5);
}

static void qmc6308_stop_measure(struct qmc6308_data *qmc6308)
{
	unsigned char data;
	data = 0x00;
	MSE_LOG( "qst qmc6308_stop_measure data-->%0x\n",data);
	qmc6308_i2c_write(CTL_REG_ONE, &data, 1);
}

static int qmc6308_enable(struct qmc6308_data *qmc6308)
{
	MSE_LOG("qmc6308_enable!\n");
	//qmc6308_set_range(QMC6308_RNG_8G);  
	//qmc6308_set_ratio(1);
	qmc6308_start_measure(qmc6308);
	schedule_delayed_work(&qmc6308->work,
			msecs_to_jiffies(qmc6308->msec_delay));
    
	return 0;
}

static int qmc6308_disable(struct qmc6308_data *qmc6308)
{
	MSE_LOG("qmc6308_disable!\n");
	qmc6308_stop_measure(qmc6308);
	cancel_delayed_work(&qmc6308->work);
	return 0;
}

static void qmc6308_work(struct work_struct *work)
{
	int ret;
	unsigned char data[6];
	unsigned char drdy = 0;
	int retry = 0;
	//int needRetry = 0;
	struct qmc6308_data *qmc6308 = container_of((struct delayed_work *)work,
						struct qmc6308_data, work);

	mutex_lock(&qmc6308->lock);

	if ((!qmc6308->menabled)&&(!qmc6308->oenabled))
		goto out;

	while(!(drdy & 0x01) && retry<5){
		qmcX6983_i2c_read(STA_REG_ONE, &drdy, 1);
		//drdy = data[0];
		//MSE_LOG("qmc6308 Status register is (%02X)\n", drdy);
		retry ++;
	}
	
	ret = qmc6308_read_mag_xyz((struct QMC6308_t *)data);
	if (ret < 0) {
		MSE_ERR("qmc6308_read_mag_xyz fail\n");
		
	}else{
		//MSE_LOG( "x=%d,y=%d,z=%d\n",((struct QMC6308_t *)data)->x,((struct QMC6308_t *)data)->y,((struct QMC6308_t *)data)->z);

		input_report_abs(qmc6308->input, ABS_X, ((struct QMC6308_t *)data)->x);
		input_report_abs(qmc6308->input, ABS_Y, ((struct QMC6308_t *)data)->y);
		input_report_abs(qmc6308->input, ABS_Z, ((struct QMC6308_t *)data)->z);
		input_sync(qmc6308->input);
	}

	schedule_delayed_work(&qmc6308->work,
						      msecs_to_jiffies(qmc6308->msec_delay));

out:
	mutex_unlock(&qmc6308->lock);
}

static int qmc6308_input_init(struct qmc6308_data *qmc6308)
{
	struct input_dev *dev;
	int ret;
	MSE_LOG("qmc6308_input_init\n");
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "compass";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(dev, ABS_X,-32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_Y,-32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_Z,-32768, 32767, 0, 0);			

	input_set_drvdata(dev, qmc6308);
	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}
	qmc6308->input= dev;
	return 0;
}


#if 0
static void remove_device_attributes(
	struct device *dev,
	struct device_attribute *attrs)
{
	int i;

	for (i = 0 ; NULL != attrs[i].attr.name ; ++i)
		device_remove_file(dev, &attrs[i]);
}
#endif

#ifdef qmc_debug
static int id = -1;
static ssize_t attr_get_idchk(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&qmc6308->lock);
	val = id;
	mutex_unlock(&qmc6308->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_idchk(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_dbg(&qmc6308->client->dev, "val=%lu\n", val);

	mutex_lock(&qmc6308->lock);
	id = i2c_smbus_read_word_data(qmc6308->client, (int)val);
	if ((id & 0x00FF) == 0x0048) {
		printk( "I2C driver registered!\n");
	}
	id = id & 0x00FF;
	mutex_unlock(&qmc6308->lock);

	return size;
}


static unsigned char regbuf[2] = {0};
static ssize_t show_WRregisters_value(struct device *dev,struct device_attribute *attr, char *buf)
{
		char strbuf[QMC6308_BUFSIZE];
		unsigned char data;
		unsigned char rdy = 0;
		struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);

		/* Check status register for data availability */
		int t1 = 0;
		while(!(rdy & 0x07) && t1<3){
			msleep(2);
			qmcX6983_i2c_read(STA_REG_ONE, &data, 1);
			rdy = data;
			MSE_LOG("QMC6308 Status register is (0x%02x)\n", rdy);
			t1 ++;
		}	

		mutex_lock(&qmc6308->lock);
		qmcX6983_i2c_read(regbuf[0], &data, 1);
		mutex_unlock(&qmc6308->lock);
				
		MSE_LOG("QMC6308 hw_registers = 0x%02x\n",data);  
	   
	    sprintf(strbuf, "hw_registers = 0x%02x\n", data);
	   
	   return sprintf(buf, "%s\n", strbuf);
}

static ssize_t store_WRregisters_value(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	    struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	    int err = 0;
		unsigned char data;
		if(NULL == qmc6308)
		{
			MSE_LOG(KERN_ERR "QMC6308_i2c_data is null!!\n");
			return 0;
		}
		mutex_lock(&qmc6308->lock);
		data = *buf;
		err = qmc6308_i2c_write(regbuf[0], &data, 1);
		mutex_unlock(&qmc6308->lock);

		return count;
}

static ssize_t show_registers_value(struct device *dev, struct device_attribute *attr, char *buf)
{
       
	char strbuf[QMC6308_BUFSIZE];
				
	MSE_LOG("QMC6308 hw_registers = 0x%02x\n",regbuf[0]);  
	   
	sprintf(strbuf, "hw_registers = 0x%02x\n", regbuf[0]);
	   
    return sprintf(buf, "%s\n", strbuf);
}

static ssize_t store_registers_value(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
		struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
		if(NULL == qmc6308)
		{
			MSE_ERR("QMC6308_i2c_data is null!!\n");
			return 0;
		}
		regbuf[0] = *buf;
		return count;
}

static ssize_t show_dumpallreg_value(struct device *dev, struct device_attribute *attr, char *buf)
{
		char strbuf[300];
		char tempstrbuf[24];
		unsigned char data;
		int length=0;
		unsigned char rdy = 0;
		int i;
	  
		/* Check status register for data availability */
		int t1 = 0;
		while(!(rdy & 0x07) && t1<3){
			msleep(2);
			qmcX6983_i2c_read(STA_REG_ONE, &data, 1);
			rdy = data;
			MSE_LOG("QMC6308 Status register is (0x%02x)\n", rdy);
			t1 ++;
		}	
		for(i =0;i<16;i++)
		{
			qmcX6983_i2c_read(i, &data, 1);
			length = sprintf(tempstrbuf, "reg[0x%2x] =  0x%2x \n",i, data);
			sprintf(strbuf+length*i, "  %s \n",tempstrbuf);
		}
	   
	   return sprintf(buf, "%s\n", strbuf);
}
#endif

static ssize_t attr_get_enable(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&qmc6308->lock);
	val = qmc6308->menabled;
	mutex_unlock(&qmc6308->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,struct device_attribute *attr,const char *buf, size_t size)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&qmc6308->lock);
	if (val) {
		qmc6308_enable(qmc6308);
	} else {
		qmc6308_disable(qmc6308);
	}
	qmc6308->menabled = val;
	mutex_unlock(&qmc6308->lock);

	return size;
}

static ssize_t qmc_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&qmc6308->lock);
	val = qmc6308->msec_delay;
	mutex_unlock(&qmc6308->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t qmc_delay_store(struct device *dev, struct device_attribute *attr, const char *buf,size_t size)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	unsigned long  val;

	if(NULL == buf)
		return -EINVAL;

	if(kstrtol(buf, 10, &val))
		return -EINVAL;
	
	mutex_lock(&qmc6308->lock);
	qmc6308->msec_delay = (int)val;
	mutex_unlock(&qmc6308->lock);

	return size;
}

static ssize_t show_layout_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qmc6308_data *mag = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n",mag->layout);
}

static ssize_t store_layout_value(struct device *dev, struct device_attribute *attr, const char *buf,size_t count)
{
	int layout = 0;
	struct qmc6308_data *mag = dev_get_drvdata(dev);
	
	if(buf == NULL)
		return -EINVAL;
	
	if (0 == count)
	return 0;
	
	if(1 == sscanf(buf,"%d",&layout))
	{
		mutex_lock(&mag->lock);
		mag->layout = layout;
		mutex_unlock(&mag->lock);
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
/*
static ssize_t qmc_compass_sysfs_OTP_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qmc6308_data *mag = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d,%d\n", mag->OTP_Kx,mag->OTP_Ky);
}


static ssize_t qmc_compass_sysfs_OTP_store(
	struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int Kx = 0;
	int Ky = 0;
	struct qmc6308_data *mag = dev_get_drvdata(dev);
	
	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if(sscanf(buf,"%d,%d", &Kx,&Ky))
	{
		mutex_lock(&mag->lock);
		mag->OTP_Kx = Kx;
		mag->OTP_Ky = Ky;
		mutex_unlock(&mag->lock);
	}
	else
	{
		MSE_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}
*/
#ifdef qmc_debug
static  DEVICE_ATTR(dumpallreg,  0660 , show_dumpallreg_value, NULL);
static  DEVICE_ATTR(WRregisters, 0660, show_WRregisters_value, store_WRregisters_value);
static  DEVICE_ATTR(registers,   0660, show_registers_value, store_registers_value);
static  DEVICE_ATTR(idchk, 0660, attr_get_idchk, attr_set_idchk);
#endif	
static DEVICE_ATTR(delay_mag, 0660, qmc_delay_show, qmc_delay_store);
static DEVICE_ATTR(enable_mag, 0660, attr_get_enable, attr_set_enable);
static DEVICE_ATTR(direction, 0660, show_layout_value, store_layout_value);

static dev_t const nodev = MKDEV(MISC_MAJOR, 240);

#if 0 //jiangdayao remove
static int create_sysfs_interfaces(struct qmc6308_data *mag)
{
	int err;

	if (NULL == mag)
		return -EINVAL;

	err = 0;
	/*creat class @ sys/bus*/
	mag->compass = class_create(THIS_MODULE, QMC_SYSCLS_NAME);
	if (IS_ERR(mag->compass)) {
		err = PTR_ERR(mag->compass);
		goto exit_class_create_failed;
	}

	/*creat device file @ /dev/ and sys/bus/QMC_SYSCLS_NAME */
	mag->class_dev = device_create(
						mag->compass,
						NULL,
						nodev,
						mag,
						QMC_SYSDEV_NAME);
	if (IS_ERR(mag->class_dev)) {
		err = PTR_ERR(mag->class_dev);
		goto exit_class_device_create_failed;
	}

	/*creat device attributes @sys/bus/QMC_SYSCLS_NAME/QMC_SYSDEV_NAME */
#if 0
	err = create_device_attributes(
			mag->class_dev,
			qmc_compass_attributes);
	if (0 > err)
		goto exit_sysfs_create_attributes_failed;
#else
	if(device_create_file(mag->class_dev, &dev_attr_enable_mag) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
	if(device_create_file(mag->class_dev, &dev_attr_delay_mag) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
	if(device_create_file(mag->class_dev, &dev_attr_direction) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
//	if(device_create_file(mag->class_dev, &dev_attr_otp) < 0)
//	{
//		goto exit_sysfs_create_attributes_failed;
//	}
	
#ifdef qmc_debug
	if(device_create_file(mag->class_dev, &dev_attr_dumpallreg) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
	if(device_create_file(mag->class_dev, &dev_attr_WRregisters) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
	if(device_create_file(mag->class_dev, &dev_attr_registers) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
	if(device_create_file(mag->class_dev, &dev_attr_idchk) < 0)
	{
		goto exit_sysfs_create_attributes_failed;
	}
#endif

#endif

	return err;

exit_sysfs_create_attributes_failed:
	device_destroy(mag->compass, nodev);
exit_class_device_create_failed:
	mag->class_dev = NULL;
	class_destroy(mag->compass);
exit_class_create_failed:
	mag->compass = NULL;
	return err;
}
#endif

#if 0
static void remove_sysfs_interfaces(struct qmc6308_data *mag)
{
	if (NULL == mag)
		return;

	if (NULL != mag->class_dev) {

		remove_device_attributes(
			mag->class_dev,
			qmc_compass_attributes);

	}
	if (NULL != mag->compass) {
		device_destroy(
			mag->compass,
			nodev);
		class_destroy(mag->compass);
		mag->compass = NULL;
	}
}
#endif
static void qmc6308_set_delay(struct device *dev, int delay)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct qmc6308_data *qmc6308 = i2c_get_clientdata(client);
	
	mutex_lock(&qmc6308->lock);
	if (v_open_flag) {
		cancel_delayed_work_sync(&qmc6308->work);
		schedule_delayed_work(&qmc6308->work, msecs_to_jiffies(delay));
	}
	mutex_unlock(&qmc6308->lock);
	
}

/*  ioctl command for QMC6308 device file */
static long qmc6308_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	//int buff[2] = {0};
	int enable;
	int delay;
	void __user *argp = (void __user *)arg;
	
	struct qmc6308_data *mag = file->private_data;

	if (mag->client == NULL) {
		return -EFAULT;
	}

	switch (cmd){
	case QMC6308_IOCTL_MSENSOR_ENABLE:
		if(argp == NULL)
		{
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		if(copy_from_user(&enable, argp, sizeof(enable)))
		{
			MSE_ERR("copy_from_user failed.");
			return -EFAULT;
		}
		else
		{
		    MSE_LOG( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",enable);
			if(enable == 1)
			{
				atomic_set(&m_flag, 1);
				v_open_flag |= 0x01;
			}
			else
			{
				atomic_set(&m_flag, 0);
				v_open_flag &= 0xfe;
			}
					
			MSE_LOG("qmc6308 v_open_flag = 0x%x\n",v_open_flag);
			
			mutex_lock(&mag->lock);
			if(v_open_flag)
			{
				qmc6308_enable(mag);
			}
			else
			{
				qmc6308_disable(mag);
			}
			mag->menabled = enable;
			mutex_unlock(&mag->lock);
		}
		break;


	case QMC6308_IOCTL_OSENSOR_ENABLE:
		if(argp == NULL)
		{
			MSE_ERR("IO parameter pointer is NULL!\r\n");
			break;
		}
		if(copy_from_user(&enable, argp, sizeof(enable)))
		{
			MSE_ERR("copy_from_user failed.");
			return -EFAULT;
		}
		else
		{
		    MSE_LOG("QMC6308_IOCTL_OSENSOR_ENABLE enable=%d!\r\n",enable);
			if(enable == 1)
			{
				atomic_set(&o_flag, 1);
				v_open_flag |= 0x02;
			}
			else 
			{	   
				atomic_set(&o_flag, 0);
				v_open_flag &= 0xfd;  
			}	
			
			MSE_LOG("qmc6308 v_open_flag = 0x%x\n",v_open_flag);

			mutex_lock(&mag->lock);

			if(v_open_flag)
			{
				qmc6308_enable(mag);
			}
			else
			{
				qmc6308_disable(mag);
			}
			mag->oenabled = enable;
			mutex_unlock(&mag->lock);
		}
		break;
		
	case QMC6308_IOCTL_SET_DELAY:
		if(copy_from_user(&delay,argp,sizeof(delay)))
		{
			MSE_ERR("copy_from_user fail\n");
			return -EFAULT;
		}
		if (delay < 0 || delay > 1000)
		{
			MSE_ERR("%s: set delay over limit!\n", __func__);
			return -EINVAL;
		}
		
		mag->msec_delay = delay;
		
		qmc6308_set_delay(&(mag->client->dev),mag->msec_delay);
		break;
		
	case QMC6308_IOCTL_GET_DELAY:
		if(copy_to_user(argp,&mag->msec_delay,sizeof(int)))
		{
			MSE_ERR("copy_to_user fail\n");
			return -EFAULT;
		}
		break;
/*		
	case QMC6308_SET_RANGE:
		if(copy_from_user(data,argp,sizeof(data[0])) != 0) {
			MSE_ERR("copy_from_user error\n");
			return -EFAULT;
		}
		err = qmc6308_set_range(*data);
		return err;
*/
	case QMC6308_SET_MODE:
		if(copy_from_user(data,argp,sizeof(data[0])) != 0) {
			MSE_ERR("copy_from_user error\n");
			return -EFAULT;
		}
		err = qmc6308_set_mode(data[0]);
		return err;
 
	case QMC6308_READ_MAGN_XYZ:
		err = qmc6308_read_mag_xyz((struct QMC6308_t *)data);
		MSE_LOG("mag_data[%d, %d, %d, %d, %d, %d]\n",
				data[0], data[1], data[2],
				data[3], data[4], data[5]);
		if (copy_to_user(argp, (struct QMC6308_t *)data, sizeof(data)) != 0) {
			return -EFAULT;
		}
		return err;
/*
	case QMC6308_SET_ODR:
		if (copy_from_user(data,argp,sizeof(data[0])) != 0) {
			MSE_ERR("copy_from_user error\n");
			return -EFAULT;
		}
		err = qmc6308_set_output_data_rate(data[0]);
		return err;

	case QMC6308_SET_OVERSAMPLE_RATIO:
		if (copy_from_user(data,argp,sizeof(data[0])) != 0) {
			MSE_ERR("copy_from_user error\n");
			return -EFAULT;
		}
		err = qmc6308_set_oversample_ratio(data[0]);
		return err;
*/
	case QMC6308_IOCTL_GET_DIRECTION:
		if (copy_to_user(argp,&(mag->layout), sizeof(char)) != 0) {
			return -EFAULT;
		}	
		break;
		
	case QMC6308_IOCTL_GET_OTPK:
		//buff[0] = mag->OTP_Kx;
		//buff[1] = mag->OTP_Ky;
		//if (copy_to_user(argp,buff,sizeof(buff)) != 0) {
		//	return -EFAULT;
		//}		
		break;	

	default:
		printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
		break;
	}

       return 0;
}

static int qmc6308_open(struct inode *inode,struct file *file)
{
	file->private_data = mag;
	return nonseekable_open(inode,file);
}

static int qmc6308_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations qmc6308_fops = {
	.owner = THIS_MODULE,
	.open = qmc6308_open,
	.release = qmc6308_release,
	.unlocked_ioctl = qmc6308_unlocked_ioctl,
};

static struct miscdevice qmc_compass_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = QMC_MISCDEV_NAME,
	.fops = &qmc6308_fops,
};

//jiangdayao add start
static struct attribute *Qmc6308_attributes[] = {
#ifdef qmc_debug
	&dev_attr_dumpallreg.attr,
	&dev_attr_WRregisters.attr,
	&dev_attr_registers.attr,
	&dev_attr_idchk.attr,
#endif	
	&dev_attr_delay_mag.attr,
	&dev_attr_enable_mag.attr,
	&dev_attr_direction.attr,
	NULL
};

static struct attribute_group Qmc6308_attribute_group = {
	//.name = "qmc6308",
	.attrs = Qmc6308_attributes
};

#ifdef CONFIG_OF
static int Qmc6308_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;
	unsigned int position = 0;
	
	ret = of_property_read_u32(np, "qst,layout", &position);
	printk("Qmc6308_parse_dt Qmc6308 position=%d\n",position);
	if(ret){
		dev_err(dev, "fail to get qmc_range\n");
		return 0;
	}
	mag->layout = position;
	return 0;
}
#endif


//jiangdayao add end


static int qmc6308_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{

	int err = 0;
	int ret = 0;
	struct qmc6308_platform_data *pdata;
	printk( "qmc6308_probe --- \n");

	if(client == NULL){
		printk( "QMC6308 CLIENT IS NULL");
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit;
	}
	
  	ret = i2c_smbus_read_byte_data(client, QMC6308_CHIP_ID);
	if (ret < 0) {
		err = -ENXIO;
		goto exit;
	}
	printk( "QMC6308 chip id = %d \n",ret);
	maskid = ret;
	mag = kzalloc(sizeof(struct qmc6308_data), GFP_KERNEL);
	if (!mag) {
		printk(
			"%s: memory allocation failed.", __func__);
		err = -ENOMEM;
	goto exit;
}

	if (mag == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	
	pdata = client->dev.platform_data;
	if (pdata) {
		// Platform data is available. copy its value to local. 
		printk("Have platform data:%d\n", pdata->layout);
		mag->layout = pdata->layout;
	} else {
		// Platform data is not available.
		   //Layout and information should be set by each application. 
		printk("%s: No platform data.\n", __func__);
		//mag->layout = 0;
#ifdef CONFIG_OF
		if(client->dev.of_node)
		{
			Qmc6308_parse_dt(&client->dev);
		}
#endif
	}

	//mag->layout = 3;
	
	mag->client = client;
	i2c_set_clientdata(client, mag);

	mag->msec_delay = 20;

	init_completion(&mag->data_updated);
	init_waitqueue_head(&mag->state_wq);
	mutex_init(&mag->lock);
	
	INIT_DELAYED_WORK(&mag->work, qmc6308_work);

	/* Create input device for qmc6308 */
	err = qmc6308_input_init(mag);
	if (err < 0) {
		dev_err(&client->dev, "error init input dev interface\n");
		goto exit_kfree;
	}

	//jiangdayao remove
	/*err = create_sysfs_interfaces(mag);
	if (err < 0) {
		dev_err(&client->dev, "sysfs register failed\n");
		goto exit_kfree_input;
	}*/

	err = misc_register(&qmc_compass_dev);
	if(err)
		dev_err(&client->dev, "misc register failed\n");
	
	//jiangdayao add start
	err = sysfs_create_group(&mag->input->dev.kobj, &Qmc6308_attribute_group);
	if(err < 0) {
		printk("%s: create group fail!\n", __func__);
		goto exit_sysfs_create;
	}
	//jiangdayao add end

	printk("qmc6308_probe success!!!\n");
	
	return 0;

exit_sysfs_create:
	misc_deregister(&qmc_compass_dev);
//jiangdayao remove	
//exit_kfree_input:
	//input_unregister_device(mag->input);
exit_kfree:
	kfree(mag);
exit:
	return err;
}

static int qmc6308_remove(struct i2c_client *client)
{
	struct qmc6308_data *dev = i2c_get_clientdata(client);
	#if DEBUG
		printk( "QMC6308 driver removing\n");
	#endif
#if 0
	if ( misc_deregister(&qmc_compass_dev) < 0)
		dev_err(&client->dev, "misc deregister failed.");
#else
	misc_deregister(&qmc_compass_dev);
#endif
	input_unregister_device(dev->input);
	kfree(dev);
	
	dev_info(&client->dev, "successfully removed.");
	return 0;
}



#if 1//def CONFIG_PM
static int qmc6308_suspend(struct device *dev)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	#if DEBUG
	printk( "qmc6308_suspend\n");
	#endif
	pr_err("stephen qmc6308_suspend\n");
	qmc6308->state.power_on = (qmc6308->menabled|qmc6308->menabled);
	if (qmc6308->state.power_on) {
		qmc6308_disable(qmc6308);
	}
	/* TO DO */
	return 0;
}

static int qmc6308_resume(struct device *dev)
{
	struct qmc6308_data *qmc6308 = dev_get_drvdata(dev);
	#if DEBUG
	printk( "qmc6308_resume\n");
	#endif
	pr_err("stephen qmc6308_resume\n");
	if (qmc6308->state.power_on) {
		qmc6308_enable(qmc6308);
	}
	return 0;
}
#endif

static const struct dev_pm_ops qmc_compass_pm_ops = {
	.suspend	= qmc6308_suspend,
	.resume		= qmc6308_resume,
};

static const struct i2c_device_id qmc6308_id[] = {
	{ QMC_I2C_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, qmc6308_id);

static struct of_device_id qmc6308_of_match[] = {
	{ .compatible = "qst,qmc6308", },
	{ },
};
MODULE_DEVICE_TABLE(of, qmc6308_of_match);


static struct i2c_driver qmc6308_driver = {
	.probe = qmc6308_probe,
	.remove = qmc6308_remove,
	.id_table = qmc6308_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = QMC_I2C_NAME,
		.of_match_table = qmc6308_of_match,
		.pm		= &qmc_compass_pm_ops,
	},
};

static int __init qmc6308_init(void)
{
	int ret;
	printk("qmc6308_init\n");
	/* add i2c driver for QMC6308 magnetometer */
	ret = i2c_add_driver(&qmc6308_driver);
	printk("qmc6308_init, ret = %d\n", ret);
	return ret;
}

static void __exit qmc6308_exit(void)
{
	#if DEBUG
	printk("QMC6308 exit\n");
	#endif
	i2c_del_driver(&qmc6308_driver);
	return;
}

module_init(qmc6308_init);
module_exit(qmc6308_exit);

MODULE_DESCRIPTION("QMC6308 magnetometer driver");
MODULE_AUTHOR("QST");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.1");


