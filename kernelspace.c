#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <linux/kernel.h>

int main(){
	int ret = 0;
	
	
	while(1){
		ret = gpio_direction_output(16, 0);
		fprintf(stderr, "gpio_direction_output -> 0 returned %d\n", ret);
		
		usleep(2000000);
		
		ret = gpio_direction_output(16, 1);
		fprintf(stderr, "gpio_direction_output -> 1 returned %d\n", ret);
	}

	return 0;
}
