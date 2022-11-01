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
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pigpio.h>
#include "rotary_encoder.h"

#define GPIO_PIN_A 8
#define GPIO_PIN_B 11
int quiet = 0;
void help()
{
	printf("    Use this application for reading from encoder\n");
	printf("    execute format: ./encoder [-h][-q] \n");
	printf("    return: increment value, when rotate right\n");
	printf("            decrement value, when rotate left\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

void callback(int way)
{
	static int pos = 0;
	pos -= way*360/20;
	if (!quiet)
		printf("angle increment: %d\n", pos);
	if (quiet)
		printf("%d\n", pos);
	fflush(stdout);
}

int main(int argc, char *argv[])
{
	if (argc > 1) {
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
		printf("\nThe encoder application was started\n\n");

	Pi_Renc_t *renc;
	if (gpioInitialise() < 0)
		return 1;
	renc = Pi_Renc(GPIO_PIN_A, GPIO_PIN_B, callback);
	sleep(300);
	Pi_Renc_cancel(renc);
	gpioTerminate();
}
