/*****************************************************************************
*
* File                : LCD.c
* Hardware Environment: Raspberry Pi
* Build Environment   : GCC
* Version             : V0.0.1b
*
*              (c) Copyright 2021, domhathair
*              
******************************************************************************/

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LCD.h"

#define CHANNEL			0
#define CLOCK_SPEED		625000

void LCD_send_command(int n, ...) {
	int* pointer;
	int* buff;
	char* cmd;
	int len;
	int it;
	
	len = 0;
	it = 0;
	buff = (int*)calloc(n, sizeof(int));
	
	for (pointer = &n; n > 0; n--)
	{
		buff[it++] = *(pointer++);
	}
	
	cmd = (char*)calloc(8, sizeof(char));
	
	switch (buff[0]) 
	{
		case 2:
			len = snprintf(cmd, 8, "\x1B[%c", buff[1]);
		break;
		case 3:
			len = snprintf(cmd, 8, "\x1B[%d%c", buff[2], buff[1]);
		break;
		case 4: 
			len = snprintf(cmd, 8, "\x1B[%d;%dH", buff[2], buff[3]);
		break;
	}
	
	digitalWrite(SS, LOW);
	if (len < 9)
	{
		wiringPiSPIDataRW(CHANNEL, (unsigned char*)cmd, len);
	}
	digitalWrite(SS, HIGH);
	
	free(cmd);
	free(buff);
}

int LCD_begin() 
{
	pinMode(SS, OUTPUT);
	
	int fd = wiringPiSPISetupMode(CHANNEL, CLOCK_SPEED, 0);
    if (fd == -1) 
    {
        printf("Failed to init SPI communication.\n");
        return -1;
    }
    printf("SPI communication successfully setup.\n");

    delay(10);
    return 0;
}

void LCD_print(char* string)
{
	digitalWrite(SS, LOW);
	wiringPiSPIDataRW(CHANNEL, (unsigned char*)string, strlen(string));
	digitalWrite(SS, HIGH);
}
