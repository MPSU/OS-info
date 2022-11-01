#include <stdio.h>
#include <unistd.h>

int GPIO_setup(unsigned int pin, int value)
{
	FILE *fp;
	char path[128];

	/* If the pin is already exported, unexport first */
	if (access(path, F_OK)) {
		snprintf(path, sizeof(path), "/sys/class/gpio/unexport");
		if ((fp = fopen(path, "w")) == NULL) {
			perror(path);
			return 0;
		}
		fprintf(fp, "%d\n", pin);
		fclose(fp);
	}
	/* Now export the pin */
	snprintf(path, sizeof(path), "/sys/class/gpio/export");
	if ((fp = fopen(path, "w")) == NULL) {
		perror(path);
		return 0;
	}
	fprintf(fp, "%d\n", pin);
	fclose(fp);

	usleep(50000); /* sys needs some time to adjust */
	/* Now set direction (and initial value) */
	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
	if ((fp = fopen(path, "w")) == NULL) {
		perror(path);
		return 0;
	}
	if (value == -1) {
		fprintf(fp, "in\n");
		fclose(fp);
	} else {
		fprintf(fp, "out\n");
		fclose(fp);
		snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value",
			 pin);
		if ((fp = fopen(path, "w")) == NULL) {
			perror(path);
			return 0;
		}
		fprintf(fp, "%d\n", value ? 1 : 0);
		fclose(fp);
	}
	return 1;
}
