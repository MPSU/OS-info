#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <ads1115.h>

#define	AD_BASE 122
#define AD_ADDR 0x48

void help()
{
	printf("    Use this application for reading from rangefinder\n");
	printf("    execute format: ./range TIME \n");
	printf("    return: length in cm\n");
	printf("    TIME - pause between writing in ms\n");
	printf("    -h - help\n");
	printf("    -q - quiet\n");
}

int clamp(int x, int min, int max) {
    return (x < min) ? min : ((x > max) ? max : x);
}

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

	int argument = 1;
	if (quiet)
		argument++;
    int delay_ms = atoi(argv[argument]);

    wiringPiSetup();
    ads1115Setup(AD_BASE, AD_ADDR);
    digitalWrite(AD_BASE, 0);

    while(1) {
        float voltage = ((float)analogRead(AD_BASE+2) * 0.1875 / 1000.0);
        
        // Power regression approximation
        // Distance is clamped between 20 and 150 cm
        int distance = clamp(round(61.3894*pow(voltage, -1.1076)), 20, 150);

		if (!quiet)
			printf("Length = %d cm\n", distance);
		else
			printf("%d\n", distance);

        fflush(stdout);
        usleep(1000 * delay_ms);
    }

    return 0;
}