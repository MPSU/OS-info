/*******************************************************************************
 * Copyright (c) 2022 Sergey Balabaev (sergei.a.balabaev@gmail.com)                     *
 *                                                                             *
 * The MIT License (MIT):                                                      *
 * Permission is hereby granted, free of charge, to any person obtaining a     *
 * copy of this software and associated documentation files (the "Software"),  *
 * to deal in the Software without restriction, including without limitation   *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell   *
 * copies of the Software, and to permit persons to whom the Software is       *
 * furnished to do so, subject to the following conditions:                    *
 * The above copyright notice and this permission notice shall be included     *
 * in all copies or substantial portions of the Software.                      *
 *                                                                             *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,             *
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR       *
 * OTHER DEALINGS IN THE SOFTWARE.                                             *
 ******************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "./Adafruit_ADS1X15_RPi/Adafruit_ADS1015.h"

#define err(mess)                                    \
	{                                            \
		fprintf(stderr, "Error: %s.", mess); \
		exit(1);                             \
	}

//***************************//
#define ADC_PIN 1 // GPIO PIN TRIG
//***************************//
#define THRESHOLD 5000

Adafruit_ADS1115 ads;

void help()
{
	printf("    Use this application for reading from light sensor\n");
	printf("    execute format: ./light_detect [-h][-q] TIME \n");
	printf("    return: ADC value, where 0 - minimum light, 65535 - maximum light\n");
	printf("    TIME - pause between writing in ms\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

int main(int argc, char *argv[])
{
	int quiet = 0;
	if (argc <= 1) {
		help();
		return -1;
	} else {
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		} else {
			if ((strcmp(argv[1], "-q") == 0)) {
				quiet = 1;
			}
		}
	}
	if (!quiet)
		printf("\nThe lightsensor application was started\n\n");
	ads.setGain(GAIN_TWO);
	ads.begin();
	uint16_t adc0;
	int argument = 1;
	if (quiet)
		argument++;
	uint16_t pause_time = atoi(argv[argument]);
	while (true) {
		adc0 = ads.readADC_SingleEnded(ADC_PIN);
		if (quiet) {
			printf("%d\n", adc0);
			fflush(stdout);
		} else {
			printf("ADC: %d\n", adc0);
			fflush(stdout);
		}
		usleep(pause_time * 1000);
	}
}