// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// L3G4200D
// This code is designed to work with the L3G4200D_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/content/Gyro?sku=L3G4200D_I2CS#tabs-0-product_tabset-2

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

void main() 
{
	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-1";
	if((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, L3G4200D I2C address is 0x68(104)
	ioctl(file, I2C_SLAVE, 0x68);

	// Enable X, Y, Z-Axis and disable Power down mode(0x0F)
	char config[2] = {0};
	config[0] = 0x20;
	config[1] = 0x0F;
	write(file, config, 2);
	// Full scale range, 2000 dps(0x30)
	config[0] = 0x23;
	config[1] = 0x30;
	write(file, config, 2);
	sleep(1);

	// Read 6 bytes of data
	// lsb first
	// Read xGyro lsb data from register(0x28)
	char reg[1] = {0x28};
	write(file, reg, 1);
	char data[1] = {0};
	if(read(file, data, 1) != 1)
	{
		printf("Error : Input/Output error \n");
		exit(1);
	}
	char data_0 = data[0];

	// Read xGyro msb data from register(0x29)
	reg[0] = 0x29;
	write(file, reg, 1);
	read(file, data, 1);
	char data_1 = data[0];

	// Read yGyro lsb data from register(0x2A)
	reg[0] = 0x2A;
	write(file, reg, 1);
	read(file, data, 1);
	char data_2 = data[0];

	// Read yGyro msb data from register(0x2B)
	reg[0] = 0x2B;
	write(file, reg, 1);
	read(file, data, 1);
	char data_3 = data[0];

	// Read zGyro lsb data from register(0x2C)
	reg[0] = 0x2C;
	write(file, reg, 1);
	read(file, data, 1);
	char data_4 = data[0];

	// Read zGyro msb data from register(0x2D)
	reg[0] = 0x2D;
	write(file, reg, 1);
	read(file, data, 1);
	char data_5 = data[0];

	// Convert the data
	int xGyro = (data_1 * 256 + data_0);
	if(xGyro > 32767)
	{
		xGyro -= 65536;
	}

	int yGyro = (data_3 * 256 + data_2);
	if(yGyro > 32767)
	{
		yGyro -= 65536;
	}

	int zGyro = (data_5 * 256 + data_4);
	if(zGyro > 32767)
	{
	zGyro -= 65536;
	}

	// Output data to screen
	printf("Rotation in X-Axis : %d \n", xGyro);
	printf("Rotation in Y-Axis : %d \n", yGyro);
	printf("Rotation in Z-Axis : %d \n", zGyro);
}
