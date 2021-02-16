#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>

int main(){
	struct gpiohandle_request req;
	struct gpiohandle_data data;
	char chrdev_name[20];
	int fd, ret;

	strcpy(chrdev_name, "/dev/gpiochip0");

	fd = open(chrdev_name, 0);
	if (fd == -1) {
		fprintf(stderr, "Failed to open %s\n", chrdev_name);
		return ret;
	}

	req.lineoffsets[0] = 16;
	req.flags = GPIOHANDLE_REQUEST_OUTPUT;
	memcpy(req.default_values, &data, sizeof(req.default_values));
	strcpy(req.consumer_label, "led_gpio");
	req.lines  = 1;

	ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
	if (ret == -1) {
		fprintf(stderr, "Failed to issue GET LINEHANDLE IOCTL (%d)\n",
			ret);
	}
	if (close(fd) == -1)
		perror("Failed to close GPIO character device file");

	usleep(1000000);

	data.values[0] = 1;
	ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
	if (ret == -1) {
		fprintf(stderr, "Failed to issue %s (%d)\n",
				"GPIOHANDLE_SET_LINE_VALUES_IOCTL", ret);
	}

	/*  release line */
	ret = close(req.fd);
	if (ret == -1) {
		perror("Failed to close GPIO LINEHANDLE device file");
	}	

	return 0;
}