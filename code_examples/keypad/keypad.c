/*******************************************************************************
 * Copyright (c) 2018 Dmitrii Kaleev (kaleev@org.miet.ru)                      *
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

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ROWS 4
#define COLS 3

char pressedKey = '\0';

//int rowPins[ROWS] = { 6, 27, 0, 1 }; // 25, 16, 17, 18; R3, R2, R1, R0
//int colPins[COLS] = { 24, 29, 28 }; // 19 ,21, 20; C2, C1, C0

int rowPins[ROWS] = { 10, 14, 6, 7 }; // 8, 11, 25, 4; R3, R2, R1, R0
int colPins[COLS] = { 21, 22, 26 }; // 5 ,6, 7; C2, C1, C0

char keys[ROWS][COLS] = { { '1', '2', '3' },
			  { '4', '5', '6' },
			  { '7', '8', '9' },
			  { '*', '0', '#' } };

void init_keypad()
{
	for (int c = 0; c < COLS; c++) {
		pinMode(colPins[c], OUTPUT);
		digitalWrite(colPins[c], HIGH);
	}

	for (int r = 0; r < ROWS; r++) {
		pinMode(rowPins[r], INPUT);
	}
	system("raspi-gpio set 4 pu");
	system("raspi-gpio set 8 pu");
	system("raspi-gpio set 11 pu");
	system("raspi-gpio set 25 pu");
}

int findLowRow()
{
	for (int r = 0; r < ROWS; r++) {
		if (digitalRead(rowPins[r]) == LOW)
			return r;
	}
	return -1;
}

char get_key()
{
	int rowIndex;

	for (int c = 0; c < COLS; c++) {
		digitalWrite(colPins[c], LOW);

		rowIndex = findLowRow();
		if (rowIndex > -1) {
			if (!pressedKey)
				pressedKey = keys[rowIndex][c];
			return pressedKey;
		}

		digitalWrite(colPins[c], HIGH);
	}

	pressedKey = '\0';
	return pressedKey;
}

void help()
{
	printf("    Use this application for keypad\n");
	printf("    execute format: ./light_detect [-h] \n");
	printf("    return: enter button\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

int main(int argc, char *argv[])
{
	int quiet = 0;
	if (argc > 1) {
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		} else if ((strcmp(argv[1], "-q") == 0)) {
			quiet = 1;
		} else {
			help();
			return 0;
		}
	}
	wiringPiSetup();
	init_keypad();
	system("clear");
	if (!quiet) {
		while (1) {
			char x = get_key();
			if (x)
				printf("pressed: %c\n", x);
			else
				printf("no key pressed\n");

			delay(250);
			system("clear");
		}
	} else {
		while (1) {
			char x = get_key();
			if (x)
				printf("%c\n", x);
			delay(250);
			system("clear");
		}
	}
	return 0;
}