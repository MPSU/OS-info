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
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

void help();
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
	if (!quiet)
		printf("\nThe Notes application was started\n\n");

	char buf[2];
	while (1) {
		scanf("%s", buf);
		if ((buf[0] == 'A') && (buf[1] != '#')) {
			system("aplay ./notes/A4.wav -q");
			printf("Playing A#");
		}
		if (buf[0] == 'B') {
			system("aplay ./notes/B4.wav -q");
			printf("Playing B");
		}
		if ((buf[0] == 'C') && (buf[1] != '#')) {
			system("aplay ./notes/C4.wav -q");
			printf("Playing C#");
		}
		if ((buf[0] == 'D') && (buf[1] != '#')) {
			system("aplay ./notes/D4.wav -q");
			printf("Playing D#");
		}
		if (buf[0] == 'E') {
			system("aplay ./notes/E4.wav -q");
			printf("Playing E");
		}
		if ((buf[0] == 'F') && (buf[1] != '#')) {
			system("aplay ./notes/F4.wav -q");
			printf("Playing F#");
		}
		if ((buf[0] == 'G') && (buf[1] != '#')) {
			system("aplay ./notes/G4.wav -q");
			printf("Playing G#");
		}
		if ((buf[0] == 'A') && (buf[1] == '#')) {
			system("aplay ./notes/Ad4.wav -q");
			printf("Playing A#");
		}
		if ((buf[0] == 'C') && (buf[1] == '#')) {
			system("aplay ./notes/Cd4.wav -q");
			printf("Playing C#");
		}
		if ((buf[0] == 'D') && (buf[1] == '#')) {
			system("aplay ./notes/Dd4.wav -q");
			printf("Playing D#");
		}
		if ((buf[0] == 'F') && (buf[1] == '#')) {
			system("aplay ./notes/Fd4.wav -q");
			printf("Playing F#");
		}
		if ((buf[0] == 'G') && (buf[1] == '#')) {
			system("aplay ./notes/Gd4.wav -q");
			printf("Playing G#");
		}
		fflush(stdout);
	}

	return 0;
}

void help()
{
	printf("    Use this application for plaing notes\n");
	printf("    execute format: ./notes\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
	printf("    input format (from stdin):\n");
	printf("        NOTE\n");
	printf("    NOTE - note name in scientific pitch notation\n");
	printf("    Example:\n");
	printf("    ./notes -q\n");
	printf("    A\n");
	printf("    playing A\n");
}
