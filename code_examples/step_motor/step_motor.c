/*******************************************************************************
 * Copyright (c) 2022 Sergey Balabaev (sergei.a.balabaev@gmail.com)                    *
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

#include <stdio.h>
#include <bcm2835.h>
#include <stdlib.h>
#include <string.h>
#define STEP_DELAY 1

/*
2000  7 RPM
1500  9 RPM
1000 13 RPM
 900 15 RPM
*/

//***************************//
#define Pin1 8
#define Pin2 11
#define Pin3 26
#define Pin4 27
//***************************//

int step = 0;

void loop()
{
	switch (step) {
	case 0:
		bcm2835_gpio_write(Pin1, LOW);
		bcm2835_gpio_write(Pin2, LOW);
		bcm2835_gpio_write(Pin3, LOW);
		bcm2835_gpio_write(Pin4, HIGH);
		break;
	case 1:
		bcm2835_gpio_write(Pin1, LOW);
		bcm2835_gpio_write(Pin2, LOW);
		bcm2835_gpio_write(Pin3, HIGH);
		bcm2835_gpio_write(Pin4, HIGH);
		break;
	case 2:
		bcm2835_gpio_write(Pin1, LOW);
		bcm2835_gpio_write(Pin2, LOW);
		bcm2835_gpio_write(Pin3, HIGH);
		bcm2835_gpio_write(Pin4, LOW);
		break;
	case 3:
		bcm2835_gpio_write(Pin1, LOW);
		bcm2835_gpio_write(Pin2, HIGH);
		bcm2835_gpio_write(Pin3, HIGH);
		bcm2835_gpio_write(Pin4, LOW);
		break;
	case 4:
		bcm2835_gpio_write(Pin1, LOW);
		bcm2835_gpio_write(Pin2, HIGH);
		bcm2835_gpio_write(Pin3, LOW);
		bcm2835_gpio_write(Pin4, LOW);
		break;
	case 5:
		bcm2835_gpio_write(Pin1, HIGH);
		bcm2835_gpio_write(Pin2, HIGH);
		bcm2835_gpio_write(Pin3, LOW);
		bcm2835_gpio_write(Pin4, LOW);
		break;
	case 6:
		bcm2835_gpio_write(Pin1, HIGH);
		bcm2835_gpio_write(Pin2, LOW);
		bcm2835_gpio_write(Pin3, LOW);
		bcm2835_gpio_write(Pin4, LOW);
		break;
	case 7:
		bcm2835_gpio_write(Pin1, HIGH);
		bcm2835_gpio_write(Pin2, LOW);
		bcm2835_gpio_write(Pin3, LOW);
		bcm2835_gpio_write(Pin4, HIGH);
		break;
	default:
		bcm2835_gpio_write(Pin1, LOW);
		bcm2835_gpio_write(Pin2, LOW);
		bcm2835_gpio_write(Pin3, LOW);
		bcm2835_gpio_write(Pin4, LOW);
		break;
	}
}

void help()
{
	printf("    Use this application for reading from Stepper Motor\n");
	printf("    execute format: sudo ./motor [-h][-q] step_delay angle \n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

int main(int argc, char *argv[])
{
	int arg_N = 1;
	int quiet = 0;
	if (argc > 1) {
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		} else {
			if ((strcmp(argv[1], "-q") == 0)) {
				if (argc < 4) {
					help();
					return (-1);
				}
				quiet = 1;
				arg_N++;
			}
		}
	}
	if (argc < 3) {
		help();
		return (-1);
	}

	if (!quiet)
		printf("\nThe Stepper Motor application was started\n\n");
	int i, rotate_dir, angle;
	int step_delay = STEP_DELAY;

	step_delay = atoi(argv[arg_N]);
	arg_N++;
	angle = atoi(argv[arg_N]);
	if (angle < 0)
		rotate_dir = 0;
	else
		rotate_dir = 1;
	if ((step_delay < 0) || (step_delay > 20))
		step_delay = STEP_DELAY;

	if (!bcm2835_init())
		return 1;

	bcm2835_gpio_fsel(Pin1, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(Pin2, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(Pin3, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(Pin4, BCM2835_GPIO_FSEL_OUTP);

	int intang = abs(angle) * 11.377;
	if (rotate_dir)
		step = 7;
	for (i = 0; i < intang; i++)
		if (rotate_dir) {
			loop();
			step--;
			if (step < 0)
				step = 7;
			bcm2835_delay(step_delay);
		} else {
			loop();
			step++;
			if (step > 7)
				step = 0;
			bcm2835_delay(step_delay);
		}
		bcm2835_close();
}