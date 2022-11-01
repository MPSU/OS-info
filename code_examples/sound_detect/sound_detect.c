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
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define IN 0
#define OUT 1

#define LOW 0
#define HIGH 1

//***************************//
#define GPIO_PIN 8
//***************************//

void Exiting(int);
static int GPIOExport(int pin)
{
#define BUFFER_MAX 3
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open export for writing!\n");
		Exiting(-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return (0);
}

static int GPIOUnexport(int pin)
{
	char buffer[BUFFER_MAX];
	ssize_t bytes_written;
	int fd;

	fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open unexport for writing!\n");
		Exiting(-1);
	}

	bytes_written = snprintf(buffer, BUFFER_MAX, "%d", pin);
	write(fd, buffer, bytes_written);
	close(fd);
	return (0);
}

static int GPIODirection(int pin, int dir)
{
	static const char s_directions_str[] = "in\0out";

#define DIRECTION_MAX 35
	char path[DIRECTION_MAX];
	int fd;

	snprintf(path, DIRECTION_MAX, "/sys/class/gpio/gpio%d/direction", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio direction for writing!\n");
		Exiting(-1);
	}

	if (-1 == write(fd, &s_directions_str[IN == dir ? 0 : 3],
			IN == dir ? 2 : 3)) {
		fprintf(stderr, "Failed to set direction!\n");
		Exiting(-1);
	}

	close(fd);
	return (0);
}

static int GPIORead(int pin)
{
#define VALUE_MAX 30
	char path[VALUE_MAX];
	char value_str[3];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_RDONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for reading!\n");
		Exiting(-1);
	}

	if (-1 == read(fd, value_str, 3)) {
		fprintf(stderr, "Failed to read value!\n");
		Exiting(-1);
	}

	close(fd);

	return (atoi(value_str));
}

void help()
{
	printf("    Use this application for reading from sound sensor\n");
	printf("    execute format: ./sound_detect [-h][-q] \n");
	printf("    return: 'Clap!', when sound detected\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

void Exiting(int parameter)
{
	GPIOUnexport(GPIO_PIN);
	exit(parameter);
}

void Exiting_sig()
{
	GPIOUnexport(GPIO_PIN);
	exit(0);
}

int main(int argc, char *argv[])
{
	signal(SIGINT, Exiting_sig);
	int quiet = 0;
	if (argc > 1){
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		} else {
			if ((strcmp(argv[1], "-q") == 0))
				quiet = 1;
		}
	}
	if (!quiet)
		printf("\nThe soundsensor application was started\n\n");

	GPIOExport(GPIO_PIN);
	GPIODirection(GPIO_PIN, IN);
	sleep(0.05);
	int prev, next;
	prev = 0;
	while (1) {
		next = GPIORead(GPIO_PIN);
		if (prev < next) {
			printf("Clap!\n");
			fflush(stdout);
			usleep(10000);
		}
		prev = next;
	}
	return 0;
}