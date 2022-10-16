/*******************************************************************************
 * Copyright (c) 2022 Sergey Balabaev (sergei.a.balabaev@gmail.com)                      *
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
#define TRIG 11 // GPIO PIN TRIG
#define ECHO 26 // GPIO PIN ECHO
//***************************//

void Exiting(int);

int read_pins_file(char *file)
{
	FILE *f = fopen(file, "r");
	if (f == 0) {
		fprintf(stderr, "ERROR: can't open %s file\n", file);
		return -1;
	}

	char str[32];
	while (!feof(f)) {
		if (fscanf(f, "%s\n", str))
			printf("%s\n", str);
		fflush(stdout);
		sleep(1);
	}
	fclose(f);

	return 0;
}

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

static int GPIOWrite(int pin, int value)
{
	static const char s_values_str[] = "01";

	char path[VALUE_MAX];
	int fd;

	snprintf(path, VALUE_MAX, "/sys/class/gpio/gpio%d/value", pin);
	fd = open(path, O_WRONLY);
	if (-1 == fd) {
		fprintf(stderr, "Failed to open gpio value for writing!\n");
		Exiting(-1);
	}

	if (1 != write(fd, &s_values_str[LOW == value ? 0 : 1], 1)) {
		fprintf(stderr, "Failed to write value!\n");
		Exiting(-1);
	}

	close(fd);
	return (0);
}

void Exiting(int parameter)
{
	GPIOUnexport(TRIG);
	GPIOUnexport(ECHO);
	exit(parameter);
}

void Exiting_sig()
{
	GPIOUnexport(TRIG);
	GPIOUnexport(ECHO);
	exit(0);
}

void help()
{
	printf("    Use this application for reading from rangefinder\n");
	printf("    execute format: ./range TIME \n");
	printf("    return: length in cm\n");
	printf("    TIME - pause between writing in ms\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

#define TIMEOUT_SEC 2

int main(int argc, char *argv[])
{
	int quiet = 0;
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

	if ((quiet && argc != 3) || (!quiet && argc != 2)) {
		help();
		return 0;
	}

	if (!quiet)
		printf("\nThe rangefinder application was started\n\n");
	char *mode = argv[1 + quiet];

	if (strcmp(mode, "-s") == 0) {
		char data[32];
		while (1) {
			scanf("%s", data);
			fflush(stdin);
			printf("%s\n", data);
			fflush(stdout);
		}
	}

	if (strcmp(mode, "-f") == 0) {
		char *file = argv[2 + quiet];
		if (read_pins_file(file) < 0)
			return -1;
		else
			return 0;
	}

	double search_time = 0;
	signal(SIGINT, Exiting_sig);
	GPIOExport(TRIG);
	GPIOExport(ECHO);
	sleep(0.05);
	GPIODirection(TRIG, OUT);
	GPIODirection(ECHO, IN);
	
	int argument = 1;
	if (quiet)
		argument++;

	double sl;
	while (1) {
		GPIOWrite(TRIG, 1);
		usleep(10);
		GPIOWrite(TRIG, 0);
		while (!GPIORead(ECHO)) {
		}
		double start_time = clock();
		int flag = 0;
		while (GPIORead(ECHO)) {
			if (clock() - start_time >
			    TIMEOUT_SEC * CLOCKS_PER_SEC) {
				flag = 1;
				break;
			}
		}
		if (flag) {
			printf("Timeout reached, sleeping for one second\n");
			sleep(1);
			continue;
		}
		double end_time = clock();
		search_time = end_time - start_time;

		sl = atoi(argv[argument]);

		if (!quiet)
			printf("signal_delay: %lf ms\n", search_time);
		else
			printf("%lf\n", search_time);
		fflush(stdout);
		if ((sl > 0) && (sl < 60000))
			usleep(sl * 1000);
		else
			sleep(1);
	}
	return 0;
}
