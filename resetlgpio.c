#include <stdio.h>

#include <lgpio.h>

#define OUT1 16

int main(int argc, char *argv[])
{
   int h;
   int lFlags = 0; /* default line flags */

   /* get a handle to the GPIO */
   h = lgGpiochipOpen(0);
   
   fprintf(stdout, "Handle: %d", h);

   /* claim some GPIO for OUTPUT */
   int response = lgGpioClaimOutput(h, lFlags, OUT1, 1); /* initial level 1 */
   
   fprintf(stdout, "Claim response: %d\n", response);

   while (1)
   {
	  usleep(1000000);
	  fprintf(stdout, "Low\n");
	  lgGpioWrite(h, OUT1, 0);
	  usleep(1000000);
	  fprintf(stdout, "High\n");
	  lgGpioWrite(h, OUT1, 1);
   }
}