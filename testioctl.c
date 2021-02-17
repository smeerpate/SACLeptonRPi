#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>

int main(){
	struct gpiohandle_request req;
	struct gpiohandle_data data;
	char chrdev_name[20] = "/dev/gpiochip0";
	int fd, ret;
	
	while(1){
		fd = open(chrdev_name, 0);
		if (fd == -1) {
			fprintf(stderr, "Failed to open %s\n", chrdev_name);
			return ret;
		}
		fprintf(stderr, "File descriptor %d", fd);

		req.lineoffsets[0] = 16;
		req.flags = GPIOHANDLE_REQUEST_OUTPUT;
		memcpy(req.default_values, &data, sizeof(req.default_values));
		strcpy(req.consumer_label, "led_gpio");
		req.lines = 1;

		ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
		if (ret == -1) {
			fprintf(stderr, "Failed to issue GET LINEHANDLE IOCTL (%d)\n",
				ret);
		}
		if (close(fd) == -1)
			perror("Failed to close GPIO character device file");
		fprintf(stderr, "Req file descriptor %d", req.fd);

		data.values[0] = 1;
		ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
		if (ret == -1) {
			fprintf(stderr, "Failed to issue %s (%d)\n",
					"GPIOHANDLE_SET_LINE_VALUES_IOCTL", ret);
		}

		usleep(1000000);
		
		/*  release line */
		ret = close(req.fd);
		if (ret == -1) {
			perror("Failed to close GPIO LINEHANDLE device file");
		}	
		
		// Sleep to simulate
		usleep(3000000);
	}

	return 0;
}
