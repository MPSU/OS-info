#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "include/LCD.h"

/*
 *   Sample build command: 
 *   gcc -Wall -o ./main.c ./include/LCD.c ./main -lwiringPi
 * 
 *                      OLED  |  RaspPi 
 *
 *   (Physical: 6)	     VCC ->  3.3    (Physical: 17) 
 *   (Physical: 5)   	 GND ->  GND    (Physical: 25)
 *   (Physical: 2)       MOSI->  MOSI   (Physical: 19)
 *   (Physical: 3)       MISO->  MISO   (Physical: 21)
 *   (Physical: 4)       SCK ->  SCLK   (Physical: 23)
 *   (Physical: 1)       SS  ->  22     (wiringPi: 6)
 *
 * 	Don't forget to change jumpers on JP2 to activate SPI.
 * 
 * 		Revision:	 	E	  |	  any else
 * 		MD0:	 		1	  |	  0
 * 		MD1:	 		0	  |	  1
 * 		MD2:	 		0	  |	  1
 * 
*/

int main(int argc, char *argv[])
{
	
	if(wiringPiSetup() < 0)
	{
		printf("Error occured while GPIO setup.\n");
		return -1;
	}
    
	LCD_begin();
	
				/*List of commands is given in 'LCD.h'*/
	LCD_send_command(LCD_CLEAR_DISPLAY);	/*Clear screen*/
	LCD_send_command(LCD_SET_DISPLAY_MODE, 0);	/*0 - 16x2, 1 - 40x2*/
	LCD_send_command(LCD_SET_CURSOR_POSITION, 0, 2); /*Set cursor at y, x*/
	
	LCD_print("It is working!");		/*Print message*/
	
	LCD_send_command(LCD_SET_CURSOR_POSITION, 1, 4);
	delay(1000);
	
	LCD_print("Greetings!");
	
	printf("Work done.\n");
	return 0;
}
