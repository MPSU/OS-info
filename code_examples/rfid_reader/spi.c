#include <sys/ioctl.h> // Needed for SPI port
#include <linux/spi/spidev.h> // Needed for SPI port
#include <fcntl.h> // Needed for SPI port
#include <inttypes.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <alloca.h>

int openSPI(const char *device, uint32_t speed)
{
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	int fd;

	/* Device oeffen */
	if ((fd = open(device, O_RDWR)) < 0) {
		perror("Fehler Open Device");
		return -1;
	}

	/* Mode setzen */
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		perror("Fehler Set SPI-Modus");
		close(fd);
		return -1;
	}

	/* Wortlaenge setzen */
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		perror("Fehler Set Wortlaenge");
		close(fd);
		return -1;
	}

	/* Datenrate setzen */
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		perror("Fehler Set Speed");
		close(fd);
		return -1;
	}

	return fd;
}

void spi_transfer(int fd, unsigned char *data, unsigned int length)
{
	struct spi_ioc_transfer *spi;
	int i;
	uint8_t bits;
	uint32_t speed;

	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) {
		perror("SPI_IOC_RD_BITS_PER_WORD");
		return;
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1) {
		perror("SPI_IOC_RD_MAX_SPEED_HZ");
		return;
	}

	spi = alloca(length * sizeof(struct spi_ioc_transfer));
	memset(spi, 0, length * sizeof(spi[0]));

	for (i = 0; i < length; i++) {
		spi[i].tx_buf = (unsigned long)&data[i];
		spi[i].rx_buf = (unsigned long)&data[i];
		spi[i].len = sizeof(unsigned char);
		spi[i].delay_usecs = 0;
		spi[i].speed_hz = speed;
		spi[i].bits_per_word = bits;
		spi[i].cs_change = 0;
	}

	if (ioctl(fd, SPI_IOC_MESSAGE(length), spi) == -1) {
		perror("SPI_IOC_MESSAGE");
		return;
	}
}
