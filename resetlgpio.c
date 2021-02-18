#include <stdio.h>

#include <lgpio.h>

#define OUT1 16

int main(int argc, char *argv[])
{
   int h;
   int lFlags = 0; /* default line flags */

   /* get a handle to the GPIO */
   h = lgGpiochipOpen(0);

   /* claim some GPIO for OUTPUT */
   lgGpioClaimOutput(h, lFlags, OUT1, 1); /* initial level 1 */

   while (1)
   {
	  usleep(1000000);
	  lgGpioWrite(h, OUT1, 0);
	  usleep(1000000);
	  lgGpioWrite(h, OUT1, 1);
   }
}