
#define UREG_DEV_PATH "/dev/comip-ureg"
#define UREG_GPIO_PATH "/sys/class/gpio/gpio"
#define UREG_GPIO_EXPORT "/sys/class/gpio/export"
#define UREG_GPIO_UNEXPORT "/sys/class/gpio/unexport"
#define DIRECTION "/direction"
#define VALUE "/value"
//#define UREG_GPIOX_PATH(x) "/sys/class/gpio"#x

struct ureg_unit {
	unsigned int paddr;
	unsigned int vaddr;
	unsigned int val;
};

struct ureg_i2c {
	int adapter_nr;
	int device_addr;
	int addr;
	int val;
	int flag;
};

struct ureg_gpio {
	int gpio_num;            //0~255
	int pull_up;               //0=disable, 1=enable
	int pull_down;           //0=disable, 1=enable
	int level_status;        //0=low level ,1=high level
	int direction_status;   //0=in, 1=out
	//int driver_status;      //skip
	int muxpin_status;    //0=MUX
	int flag;
};

#define UREG_MAGIC_NUM		'u'
#define UREG_READ_REG		_IOR(UREG_MAGIC_NUM, 0, struct ureg_unit)
#define UREG_WRITE_REG		_IOWR(UREG_MAGIC_NUM, 1, struct ureg_unit)
#define UREG_READ_PMIC		_IOR(UREG_MAGIC_NUM, 2, struct ureg_unit)
#define UREG_WRITE_PMIC	_IOWR(UREG_MAGIC_NUM, 3, struct ureg_unit)
#define UREG_READ_CODEC	_IOR(UREG_MAGIC_NUM, 4, struct ureg_unit)
#define UREG_WRITE_CODEC	_IOWR(UREG_MAGIC_NUM, 5, struct ureg_unit)
#define UREG_READ_I2C		_IOR(UREG_MAGIC_NUM, 6, struct ureg_i2c)
#define UREG_WRITE_I2C		_IOWR(UREG_MAGIC_NUM, 7, struct ureg_i2c)
#define UREG_READ_GPIO		_IOR(UREG_MAGIC_NUM, 8, struct ureg_gpio)
#define UREG_WRITE_GPIO	_IOR(UREG_MAGIC_NUM, 9, struct ureg_gpio)

#define UREG_I2C_DEV_ADDR_7BIT			0x00000001
#define UREG_I2C_DEV_ADDR_10BIT			0x00000002
#define UREG_I2C_ADDR_8BIT			0x00000100
#define UREG_I2C_ADDR_16BIT			0x00000200
#define UREG_I2C_ADDR_24BIT			0x00000400
#define UREG_I2C_ADDR_32BIT			0x00000800
#define UREG_I2C_VAL_8BIT			0x00010000
#define UREG_I2C_VAL_16BIT			0x00020000
#define UREG_I2C_VAL_24BIT			0x00040000
#define UREG_I2C_VAL_32BIT			0x00080000

