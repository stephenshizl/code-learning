#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>		/* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <string.h>

#include "ureg_test.h"

static int ureg_fd = -1;

#define tmpstrlens 100
char tmpstr[tmpstrlens];

int open_ureg_dev(void)
{
	ureg_fd = open(UREG_DEV_PATH, O_RDWR);
	if (ureg_fd == -1) {
		printf("open device : %s failed\n", UREG_DEV_PATH);
		return -1;
	}

	return 0;
}

void close_ureg_dev()
{
	close(ureg_fd);
	ureg_fd = -1;
}

static int xioctl(int fd, int request, void *arg)
{
	int r;
	do {
		r = ioctl(fd, request, arg);
	}
	while (-1 == r && EINTR == errno);

	return r;
}

static int read_normal_register(void)
{
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%8x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_READ_REG, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("reg 0x%.8x = 0x%.8x\n", reg.paddr, reg.val);
	return 0;

}

static int write_normal_register(void)
{
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%8x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);
	printf("Please input reg value in hex : ");
	scanf("%8x", &reg.val);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_WRITE_REG, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	if (xioctl(ureg_fd, UREG_READ_REG, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("reg 0x%.8x = 0x%.8x\n", reg.paddr, reg.val);

	return 0;

}

static int normal_register_set_clear_bit(int set)
{
	struct ureg_unit reg;
	int bit;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%8x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);
	printf("Please input bit to ");
	if (set)
		printf("set:");
	else
		printf("clear:");
	scanf("%d", &bit);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_READ_REG, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("before: reg 0x%.8x = 0x%.8x\n", reg.paddr, reg.val);

	if (set)
		reg.val |= 0x1UL << bit;
	else
		reg.val &= ~(0x1UL << bit);
	if (xioctl(ureg_fd, UREG_WRITE_REG, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	if (xioctl(ureg_fd, UREG_READ_REG, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("after: reg 0x%.8x = 0x%.8x\n", reg.paddr, reg.val);

	return 0;

}

static int read_normal_register_batch(void)
{
	unsigned int start, end;
	unsigned int i;
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg start address in hex : ");
	scanf("%8x", &start);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("reg stop address in hex : ");
	scanf("%8x", &end);
	fgets(tmpstr, tmpstrlens, stdin);

	start &= ~0x3;
	printf("\n --- beign ---\n");
	for (i = start; i <= end; i = i + 4) {
		reg.paddr = i;
		if (xioctl(ureg_fd, UREG_READ_REG, &reg) != 0) {
			fprintf(stderr, "error %d, %s\n", errno,
				strerror(errno));
			return -1;
		}
		printf("0x%.8x = 0x%.8x\n", reg.paddr, reg.val);
	}

	printf(" --- end ---\n");

	return 0;

}

static int read_pmic_register(void)
{
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%2x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_READ_PMIC, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("PMIC reg 0x%.2x = 0x%.2x\n", reg.paddr, reg.val);
	return 0;

}

static int write_pmic_register(void)
{
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%2x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);
	printf("Please input reg value in hex : ");
	scanf("%2x", &reg.val);
	fgets(tmpstr, tmpstrlens, stdin);
	if (xioctl(ureg_fd, UREG_WRITE_PMIC, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	if (xioctl(ureg_fd, UREG_READ_PMIC, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("PMIC reg 0x%.2x = 0x%.2x\n", reg.paddr, reg.val);

	return 0;

}

static int read_pmic_register_batch(void)
{
	unsigned int start, end;
	unsigned int i;
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg start address in hex : ");
	scanf("%2x", &start);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("reg stop address in hex : ");
	scanf("%2x", &end);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("\n --- beign ---\n");
	for (i = start; i <= end; i++) {
		reg.paddr = i;
		if (xioctl(ureg_fd, UREG_READ_PMIC, &reg) != 0) {
			fprintf(stderr, "error %d, %s\n", errno,
				strerror(errno));
			return -1;
		}
		printf("0x%.2x = 0x%.2x\n", reg.paddr, reg.val);
	}

	printf(" --- end ---\n");

	return 0;
}

static int read_codec_register(void)
{
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%2x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_READ_CODEC, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("CODEC reg 0x%.2x = 0x%.2x\n", reg.paddr, reg.val);
	return 0;

}

static int write_codec_register(void)
{
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg address in hex : ");
	scanf("%2x", &reg.paddr);
	fgets(tmpstr, tmpstrlens, stdin);
	printf("Please input reg value in hex : ");
	scanf("%2x", &reg.val);
	fgets(tmpstr, tmpstrlens, stdin);
	if (xioctl(ureg_fd, UREG_WRITE_CODEC, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	if (xioctl(ureg_fd, UREG_READ_CODEC, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}
	printf("CODEC reg 0x%.2x = 0x%.2x\n", reg.paddr, reg.val);

	return 0;

}

static int read_codec_register_batch(void)
{
	unsigned int start, end;
	unsigned int i;
	struct ureg_unit reg;
	reg.paddr = 0x0;
	reg.val = 0x0;
	reg.vaddr = 0x0;

	printf("reg start address in hex : ");
	scanf("%2x", &start);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("reg stop address in hex : ");
	scanf("%2x", &end);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("\n --- beign ---\n");
	for (i = start; i <= end; i++) {
		reg.paddr = i;
		if (xioctl(ureg_fd, UREG_READ_CODEC, &reg) != 0) {
			fprintf(stderr, "error %d, %s\n", errno,
				strerror(errno));
			return -1;
		}
		printf("0x%.2x = 0x%.2x\n", reg.paddr, reg.val);
	}

	printf(" --- end ---\n");

	return 0;
}

static int read_i2c(void)
{
	struct ureg_i2c i2c;
	i2c.adapter_nr = 0;
	i2c.addr = 0x0;
	i2c.device_addr = 0x0;
	i2c.addr = 0x0;
	i2c.val = 0x0;
	i2c.flag = 0x0;
	int addr_width;
	int val_width;

	printf("i2c index (0, 1, 2 ...) : ");
	scanf("%d", &i2c.adapter_nr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("slave device address in hex: ");
	scanf("%x", &i2c.device_addr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address width in bit (8, 16, 24, 32) : ");
	scanf("%d", &addr_width);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address in hex: ");
	scanf("%x", &i2c.addr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register value width in bit (8, 16, 24, 32) : ");
	scanf("%d", &val_width);
	fgets(tmpstr, tmpstrlens, stdin);

	if(addr_width == 8)
		i2c.flag |= UREG_I2C_ADDR_8BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_16BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_24BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_32BIT;
	else
		i2c.flag |= UREG_I2C_ADDR_8BIT;

	if(val_width == 8)
		i2c.flag |= UREG_I2C_VAL_8BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_16BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_24BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_32BIT;
	else
		i2c.flag |= UREG_I2C_VAL_8BIT;

	printf("i2c.adapter_nr=%d, i2c.device_addr=0x%x, i2c.addr=0x%x, i2c.val=0x%x, i2c.flag=0x%x. \n",
		i2c.adapter_nr, i2c.device_addr, i2c.addr, i2c.val, i2c.flag);

	if (xioctl(ureg_fd, UREG_READ_I2C, &i2c) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	printf("I2C_%d: device(0x%.4x) reg 0x%.8x = 0x%.8x \n",
		i2c.adapter_nr, i2c.device_addr, i2c.addr, i2c.val);

	return 0;
}

static int write_i2c(void)
{
	struct ureg_i2c i2c;
	i2c.adapter_nr = 0;
	i2c.addr = 0x0;
	i2c.device_addr = 0x0;
	i2c.addr = 0x0;
	i2c.val = 0x0;
	i2c.flag = 0x0;
	int addr_width;
	int val_width;

	printf("i2c index (0, 1, 2 ...) : ");
	scanf("%d", &i2c.adapter_nr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("slave device address in hex: ");
	scanf("%x", &i2c.device_addr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address width in bit (8, 16, 24, 32) : ");
	scanf("%d", &addr_width);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address in hex: ");
	scanf("%x", &i2c.addr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register value width in bit (8, 16, 24, 32) : ");
	scanf("%d", &val_width);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register value in hex: ");
	scanf("%x", &i2c.val);
	fgets(tmpstr, tmpstrlens, stdin);

	if(addr_width == 8)
		i2c.flag |= UREG_I2C_ADDR_8BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_16BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_24BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_32BIT;
	else
		i2c.flag |= UREG_I2C_ADDR_8BIT;

	if(val_width == 8)
		i2c.flag |= UREG_I2C_VAL_8BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_16BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_24BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_32BIT;
	else
		i2c.flag |= UREG_I2C_VAL_8BIT;

	if (xioctl(ureg_fd, UREG_WRITE_I2C, &i2c) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	if (xioctl(ureg_fd, UREG_READ_I2C, &i2c) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	printf("I2C_%d: device(0x%.4x) reg 0x%.8x = 0x%.8x \n",
		i2c.adapter_nr, i2c.device_addr, i2c.addr, i2c.val);

	return 0;
}

static int read_i2c_batch(void)
{
	struct ureg_i2c i2c;
	i2c.adapter_nr = 0;
	i2c.addr = 0x0;
	i2c.device_addr = 0x0;
	i2c.addr = 0x0;
	i2c.val = 0x0;
	i2c.flag = 0x0;
	int addr_width;
	int val_width;
	unsigned int start, end;
	unsigned int i;

	printf("i2c index (0, 1, 2 ...) : ");
	scanf("%d", &i2c.adapter_nr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("slave device address in hex: ");
	scanf("%x", &i2c.device_addr);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address width in bit (8, 16, 24, 32) : ");
	scanf("%d", &addr_width);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address start in hex: ");
	scanf("%x", &start);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register address end in hex: ");
	scanf("%x", &end);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("device register value width in bit (8, 16, 24, 32) : ");
	scanf("%d", &val_width);
	fgets(tmpstr, tmpstrlens, stdin);

	if(addr_width == 8)
		i2c.flag |= UREG_I2C_ADDR_8BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_16BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_24BIT;
	else if(addr_width == 16)
		i2c.flag |= UREG_I2C_ADDR_32BIT;
	else
		i2c.flag |= UREG_I2C_ADDR_8BIT;

	if(val_width == 8)
		i2c.flag |= UREG_I2C_VAL_8BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_16BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_24BIT;
	else if(val_width == 16)
		i2c.flag |= UREG_I2C_VAL_32BIT;
	else
		i2c.flag |= UREG_I2C_VAL_8BIT;

	printf("\n --- beign ---\n");
	for (i = start; i <= end; i++) {
		i2c.addr = i;
		if (xioctl(ureg_fd, UREG_READ_I2C, &i2c) != 0) {
			fprintf(stderr, "error %d, %s\n", errno,
				strerror(errno));
			return -1;
		}
		printf("I2C_%d: device(0x%.4x) reg 0x%.8x = 0x%.8x \n",
			i2c.adapter_nr, i2c.device_addr, i2c.addr, i2c.val);
	}

	printf(" --- end ---\n");

	return 0;
}

static int read_gpio_status(void)
{
	struct ureg_gpio reg;
	char *str = NULL;
	int fd, num;

	memset(&reg, 0, sizeof(struct ureg_gpio));
	str = (char*)malloc(128);
	printf("NOTE:Please take the spec as reference, the value below just follow the common rules\n");
	printf("Input ID in dec: ");
	scanf("%8d", &reg.gpio_num);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_READ_GPIO, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	printf("0 = MUX, 2 = GPIO, 1,3 = Reserve\n");
	printf("num %d,MUX value = %d\n", reg.gpio_num, reg.muxpin_status);
	printf("pull_down: %s\n",reg.pull_down? "enable" : "disable");
	printf("pull_up: %s\n",reg.pull_up? "enable" : "disable");

	sprintf(str, "echo %d > "UREG_GPIO_EXPORT, reg.gpio_num);
	system(str);

	memset(str, 0, 128);
	sprintf(str, "cat "UREG_GPIO_PATH"%d"DIRECTION, reg.gpio_num);
	system(str);

	memset(str, 0, 128);
	sprintf(str, "cat "UREG_GPIO_PATH"%d"VALUE, reg.gpio_num);
	system(str);

	memset(str, 0, 128);
	sprintf(str, "echo %d > "UREG_GPIO_UNEXPORT, reg.gpio_num);
	system(str);

	return 0;
}
static int read_gpio_status_batch(void)
{
	struct ureg_gpio reg;
	char *str = NULL;
	int fd, num, i, cnt;

	memset(&reg, 0, sizeof(struct ureg_gpio));
	str = (char*)malloc(128);

	printf("NOTE:Please take the spec as reference, the value below just follow the common rules\n");
	printf("Input ID in dec: ");
	scanf("%8d", &reg.gpio_num);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("Input cnt in dec : ");
	scanf("%8d", &cnt);
	fgets(tmpstr, tmpstrlens, stdin);

	if(reg.gpio_num < 0 || reg.gpio_num > 255 || (reg.gpio_num + cnt) > 256) {
		printf("Please input the valid ID and cnt value!\n");
		return 0;
	}

	printf("\n --- beign ---\n");
	for (i = 0; i < cnt; i++) {

		if (xioctl(ureg_fd, UREG_READ_GPIO, &reg) != 0) {
			fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
			return -1;
		}

		printf("0 = MUX, 2 = GPIO, 1,3 = Reserve\n");
		printf("num %d,MUX value = %d\n", reg.gpio_num, reg.muxpin_status);
		printf("pull_down: %s\n",reg.pull_down? "enable" : "disable");
		printf("pull_up: %s\n",reg.pull_up? "enable" : "disable");

		sprintf(str, "echo %d > "UREG_GPIO_EXPORT, reg.gpio_num);
		system(str);

		memset(str, 0, 128);
		sprintf(str, "cat "UREG_GPIO_PATH"%d"DIRECTION, reg.gpio_num);
		system(str);

		memset(str, 0, 128);
		sprintf(str, "cat "UREG_GPIO_PATH"%d"VALUE, reg.gpio_num);
		system(str);

		memset(str, 0, 128);
		sprintf(str, "echo %d > "UREG_GPIO_UNEXPORT, reg.gpio_num);
		system(str);

		reg.gpio_num++;
	}
	printf(" --- end ---\n");

	return 0;

}
static int write_gpio_status()
{
	struct ureg_gpio reg;
	char *str = NULL;
	int fd, num, i;

	memset(&reg, 0, sizeof(struct ureg_gpio));
	str = (char*)malloc(128);

	printf("NOTE:Please take the spec as reference, the value below just follow the common rules\n");
	printf("Input ID in dec: ");
	scanf("%8d", &reg.gpio_num);
	fgets(tmpstr, tmpstrlens, stdin);

	if(reg.gpio_num < 0 || reg.gpio_num > 255 ) {
		printf("Please input the valid ID value!\n");
		return 0;
	}

	printf("Input muxpin configure: \n");
	printf("0 = MUX, 2 = GPIO, 1,3 = Reserve, try 7 to skip\n");
	scanf("%8d", &reg.muxpin_status);
	fgets(tmpstr, tmpstrlens, stdin);

	printf("Input pull configure: \n");
	printf("0 = Both Disable, 1 = DOWN Enable,  2 = UP enable, try 7 to skip\n");
	scanf("%8d", &reg.flag);
	fgets(tmpstr, tmpstrlens, stdin);

	if (xioctl(ureg_fd, UREG_WRITE_GPIO, &reg) != 0) {
		fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
		return -1;
	}

	sprintf(str, "echo %d > "UREG_GPIO_EXPORT, reg.gpio_num);
	system(str);

	printf("Input I/O configure: \n");
	printf("Normally:0 = in ,1 = out, try 7 to skip\n");
	scanf("%8d", &reg.direction_status);
	fgets(tmpstr, tmpstrlens, stdin);

	if (reg.direction_status != 7) {
		memset(str, 0, 128);
		if (reg.direction_status == 0)
			sprintf(str, "echo in > "UREG_GPIO_PATH"%d"DIRECTION, reg.gpio_num);
		if (reg.direction_status == 1)
			sprintf(str, "echo out > "UREG_GPIO_PATH"%d"DIRECTION, reg.gpio_num);
		system(str);
	}
	printf("Input level configure: \n");
	printf("0 = low level ,1 = high level, try 7 to skip\n");
	scanf("%8d", &reg.level_status);
	fgets(tmpstr, tmpstrlens, stdin);

	if (reg.level_status != 7) {
		memset(str, 0, 128);
		sprintf(str, "echo %d > "UREG_GPIO_PATH"%d"VALUE, reg.level_status, reg.gpio_num);
		system(str);
	}

	memset(str, 0, 128);
	sprintf(str, "echo %d > "UREG_GPIO_UNEXPORT, reg.gpio_num);
	system(str);

	return 0;
	
}

int main(int argc, char **argv)
{

	int i;
	int ret = 0;

	ret = open_ureg_dev();
	if (ret)
		goto exit_main;

	while (1) {
		i = -1;
		printf("*** Leadcore Register editor tools ***\n");
		printf("0: Exit\n");

		printf("1: Read Normal Register\n");
		printf("2: Read Normal Register in batch\n");
		printf("3: Write Normal Register\n");
		printf("4: Normal Register set bit\n");
		printf("5: Normal Register clear bit\n");

		printf("6: Read PMIC Register\n");
		printf("7: Write PMIC Register\n");
		printf("8: Read PMIC Register in batch\n");

		printf("9: Read CODEC Register\n");
		printf("10: Write CODEC Register\n");
		printf("11: Read CODEC Register in batch\n");

		printf("12: Read I2C\n");
		printf("13: Write I2C\n");
		printf("14: Read I2C in batch\n");

		printf("15: Read GPIO Status\n");
		printf("16: Read GPIO Status batch\n");
		printf("17: Write GPIO Status\n");

		printf("Please select test case: ");
		scanf("%d", &i);
		switch (i) {
		case 0:
			goto exit_main;

		case 1:
			read_normal_register();
			break;
		case 2:
			read_normal_register_batch();
			break;
		case 3:
			write_normal_register();
			break;
		case 4:
			normal_register_set_clear_bit(1);
			break;
		case 5:
			normal_register_set_clear_bit(0);
			break;
		case 6:
			read_pmic_register();
			break;
		case 7:
			write_pmic_register();
			break;
		case 8:
			read_pmic_register_batch();
			break;
		case 9:
			read_codec_register();
			break;
		case 10:
			write_codec_register();
			break;
		case 11:
			read_codec_register_batch();
			break;
		case 12:
			read_i2c();
			break;
		case 13:
			write_i2c();
			break;
		case 14:
			read_i2c_batch();
			break;
		case 15:
			read_gpio_status();
			break;
		case 16:
			read_gpio_status_batch();
			break;
		case 17:
			write_gpio_status();
			break;

		default:
			printf("Invalid input !\n");
			fgets(tmpstr, tmpstrlens, stdin);
			break;
		}
	}

exit_main:
	close_ureg_dev();
	return 0;
}
