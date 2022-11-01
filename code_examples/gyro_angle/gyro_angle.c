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
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

// минимальное значение скорости поворота
#define MIN_VAL 500

// значение чувствительности из datasheet
#define SENSITIVITY_250 8.75f
#define SENSITIVITY_500 17.5f
#define SENSITIVITY_2000 70.0f

// значения чувствительности * калибровочный коэффициент
#define MAGIC_CONST_X (SENSITIVITY_2000 * 2)
#define MAGIC_CONST_Y (SENSITIVITY_2000 * 2)
#define MAGIC_CONST_Z (SENSITIVITY_2000 * 2)

// адрес шины i2c
#define I2C "/dev/i2c-1"

// время опроса (ms)
#define TIME 10.0

uint8_t xSign, ySign, zSign;
float xPosition = 0;
float yPosition = 0;
float zPosition = 0;

// Возвращает изменение температуры кристалла. 25 - точка отсчета
void askTemp(int file)
{
	char reg[1];
	char data[1];
	char temp;
	reg[0] = 0x26;
	write(file, reg, 1);
	read(file, data, 1);
	temp = 25 - data[0];
	printf("Temp deviation : %d\n", temp);
	fflush(stdout);
}

void askGiro(int file, char pos, double time)
{
	char reg[1];
	char data[1];
	char data_0, data_1;
	if (pos == 'X') {
		// Чтение значения xGyro lsb из регистра 0x28
		reg[0] = 0x28;
		write(file, reg, 1);
		read(file, data, 1);
		data_0 = data[0];
		// Чтение значения xGyro msb из регистра 0x29
		reg[0] = 0x29;
		write(file, reg, 1);
		read(file, data, 1);
		data_1 = data[0];
		// Получение данных
		int16_t xGyro = (data_1 << 8 | data_0);
		// Обнуляем минимальные значения, чтобы избежать накопления ошибки из-за шумов
		if (xGyro < MIN_VAL && xGyro > -MIN_VAL)
			xGyro = 0;
		if (xGyro > 32767)
			xGyro -= 65536;
		// проверяем первый знаковый бит
		if ((xGyro & 0x8000) == 0)
			xSign = 0;
		else {
			xSign = 1;
			// обнуляем первый знаковый бит
			xGyro &= 0x7FFF;
			xGyro = 0x8000 - xGyro;
		}
		if (xSign == 0)
			xPosition +=
				MAGIC_CONST_X * xGyro * (time / 1000) / 1000;
		else
			xPosition -=
				MAGIC_CONST_X * xGyro * (time / 1000) / 1000;
		printf("X : %lf\n", xPosition);
		return;
	}

	if (pos == 'Y') {
		reg[0] = 0x2A;
		write(file, reg, 1);
		read(file, data, 1);
		data_0 = data[0];
		reg[0] = 0x2B;
		write(file, reg, 1);
		read(file, data, 1);
		data_1 = data[0];
		int16_t yGyro = (data_1 << 8 | data_0);
		if (yGyro < MIN_VAL && yGyro > -MIN_VAL)
			yGyro = 0;
		if (yGyro > 32767)
			yGyro -= 65536;
		if ((yGyro & 0x8000) == 0)
			ySign = 0;
		else {
			ySign = 1;
			yGyro &= 0x7FFF;
			yGyro = 0x8000 - yGyro;
		}
		if (ySign == 0)
			yPosition +=
				MAGIC_CONST_Y * yGyro * (time / 1000) / 1000;
		else
			yPosition -=
				MAGIC_CONST_Y * yGyro * (time / 1000) / 1000;
		printf("Y : %lf\n", yPosition);
		return;
	}

	if (pos == 'Z') {
		reg[0] = 0x2C;
		write(file, reg, 1);
		read(file, data, 1);
		char data_0 = data[0];
		reg[0] = 0x2D;
		write(file, reg, 1);
		read(file, data, 1);
		char data_1 = data[0];
		int16_t zGyro = (data_1 << 8 | data_0);
		if (zGyro < MIN_VAL && zGyro > -MIN_VAL)
			zGyro = 0;
		if (zGyro > 32767)
			zGyro -= 65536;
		if ((zGyro & 0x8000) == 0)
			zSign = 0;
		else {
			zSign = 1;
			zGyro &= 0x7FFF;
			zGyro = 0x8000 - zGyro;
		}
		if (zSign == 0)
			zPosition +=
				MAGIC_CONST_Z * zGyro * (time / 1000) / 1000;
		else
			zPosition -=
				MAGIC_CONST_Z * zGyro * (time / 1000) / 1000;
		printf("Z : %lf\n", zPosition);
		return;
	}
}

void help()
{
	printf("    Use this application for reading from gyroscope\n");
	printf("    execute format: ./gyro [-h][-t][-g] \n");
	printf("    [-h] return: help\n");
	printf("    [-g] return: gyroscope deflection angle in X,Y,Z\n");
	printf("    [-t] return: device temperature change in C deg\n");
}

int main(int argc, char *argv[])
{
	int state = 0;
	if (argc > 1) {
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		} else if ((strcmp(argv[1], "-t") == 0)) {
			state = 1;
		} else if ((strcmp(argv[1], "-g") == 0)) {
			state = 2;
		} else {
			help();
			return 0;
		}
	} else {
		help();
		return 0;
	}

	//
	int file;
	char *bus = I2C;
	if ((file = open(bus, O_RDWR)) < 0) {
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Подключение по I2C. Адрес - 0x69
	ioctl(file, I2C_SLAVE, 0x69);
	// Включить оси X, Y, Z и отключить режим отключения питания (0x0F)
	char config[2] = { 0 };
	config[0] = 0x20;
	config[1] = 0x0F;
	write(file, config, 2);
	// Настройка диапазона измерения запись произвести в config[1]
	//		            2000 dps(0x30/0x20)
	//                  500 dps(0x10)
	//                  250 dps(0x20)
	config[0] = 0x23;
	config[1] = 0x30; // 2000 dps
	write(file, config, 2);
	sleep(1);

	system("clear");
	if (state == 1)
		while (1) {
			system("clear");
			askTemp(file);
			usleep(TIME * 1000);
		}
	if (state == 2)
		while (1) {
			system("clear");
			askGiro(file, 'X', TIME);
			askGiro(file, 'Y', TIME);
			askGiro(file, 'Z', TIME);
			fflush(stdout);
			usleep(TIME * 1000);
		}

	return 0;
}
