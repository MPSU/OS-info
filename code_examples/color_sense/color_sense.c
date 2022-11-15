/*******************************************************************************
 * Copyright (c) 2022 Sergey Balabaev (sergei.a.balabaev@gmail.com)            *
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
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define SLEEP_TIME 100

void help()
{
	printf("    Use this application for Color sensor module\n");
	printf("    execute format: ./ColorSens [-h] \n");
	printf("    -h - help\n");
	printf("    return: \n");
	printf("    \tRed color luminance [lux]\n");
	printf("    \tGreen color luminance [lux]\n");
	printf("    \tBlue color luminance [lux]\n");
	printf("    \tIR luminance [lux]\n");
	printf("    \tAmbient Light Luminance [lux]\n");
	printf("    -q - quiet\n");
	printf("    return: \n");
	printf("    \tcode: R G B\n");
}

int main(int argc, char *argv[])
{
	int quiet = 0;
	if (argc > 1) {
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		}
		if ((strcmp(argv[1], "-q") == 0)) {
			quiet = 1;
		}
	}
	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-1";
	if ((file = open(bus, O_RDWR)) < 0) {
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, TCS34725 I2C address is 0x29(41)
	ioctl(file, I2C_SLAVE, 0x29);
	// Select enable register(0x80)
	// Power ON, RGBC enable, wait time disable(0x03)
	char config[2] = { 0 };
	config[0] = 0x80;
	config[1] = 0x03;
	write(file, config, 2);
	// Select ALS time register(0x81)
	// Atime = 700 ms(0x00)
	config[0] = 0x81;
	config[1] = 0x00;
	write(file, config, 2);
	// Select Wait Time register(0x83)
	// WTIME : 2.4ms(0xFF)
	config[0] = 0x83;
	config[1] = 0xFF;
	write(file, config, 2);
	// Select control register(0x8F)
	// AGAIN = 1x(0x00)
	config[0] = 0x8F;
	config[1] = 0x00;
	write(file, config, 2);
	sleep(1);
	// Read 8 bytes of data from register(0x94)
	// cData lsb, cData msb, red lsb, red msb, green lsb, green msb, blue lsb, blue msb

	while (1) {
		char reg[1] = { 0x94 };
		write(file, reg, 1);
		char data[8] = { 0 };
		if (read(file, data, 8) != 8) {
			printf("Erorr : Input/output Erorr \n");
		} else {
			// Convert the data
			int cData = (data[1] * 256 + data[0]);
			int red = (data[3] * 256 + data[2]);
			int green = (data[5] * 256 + data[4]);
			int blue = (data[7] * 256 + data[6]);
			// Calculate luminance
			float luminance = (-0.32466) * (red) +
					  (1.57837) * (green) +
					  (-0.73191) * (blue);
			if (luminance < 0) {
				luminance = 0;
			}
			// Output data to screen
			if (!quiet) {
				printf("Red color luminance : %d lux \n", red);
				printf("Green color luminance : %d lux \n",
				       green);
				printf("Blue color luminance : %d lux \n",
				       blue);
				printf("IR luminance : %d lux \n", cData);
				printf("Ambient Light Luminance : %.2f lux \n",
				       luminance);
				fflush(stdout);
				break;
			} else {
				printf("code: %d %d %d\n", red, green, blue);
				fflush(stdout);
				usleep(SLEEP_TIME * 1000);
			}
		}
	}
}