#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "gd.h"

#define GPIO_PATH       "/sys/class/gpio"
#define GPIO_EXPORT     "/export"
#define GPIO_UNEXPORT   "/unexport"
#define GPIO_VALUE      "/value"
#define GPIO_DIRECTION  "/direction"

#define GPIO_DIR_OUT    "out"
#define GPIO_DIR_IN     "in"

#define GPIO_VALUE_HIGH "1"
#define GPIO_VALUE_LOW  "0"

#define SPI_DEVICE      "/dev/spidev1.0"

#define SPI_CHUNK_SIZE  4096
#define SPI_FREQUENCY   50000000

// LCD

#define LCD_MODE_CMD    GPIO_VALUE_LOW
#define LCD_MODE_DATA   GPIO_VALUE_HIGH

#define LCD_PIN_RST     5
#define LCD_PIN_DC      6

#define LCD_WIDTH       480
#define LCD_HEIGHT      320

// Commands

#define CMD_RDPXLFMT    0x0c

#define CMD_SLPIN       0x10
#define CMD_SLPOUT      0x11

#define CMD_INVOFF      0x20
#define CMD_INVON       0x21
#define CMD_DISPOFF     0x28
#define CMD_DISPON      0x29

#define CMD_SETCA       0x2a
#define CMD_SETPA       0x2b
#define CMD_WRMEM       0x2c
#define CMD_RDMEM       0x2e

#define CMD_MADCTL      0x36
#define CMD_IDLOFF      0x38
#define CMD_IDLON       0x39
#define CMD_PXLFMT      0x3a

#define CMD_IFMODE      0xb0

#define CMD_PWRCTLNOR   0xc2
#define CMD_VCOMCTL     0xc5

#define CMD_PGAMCTL     0xe0
#define CMD_NGAMCTL     0xe1

// Origin

#define ORIGIN_UPPER_LEFT           0x28
#define ORIGIN_UPPER_LEFT_MIRRORED  0xA8
#define ORIGIN_LOWER_LEFT           0x48
#define ORIGIN_LOWER_LEFT_MIRRORED  0x08
#define ORIGIN_UPPER_RIGHT          0x88
#define ORIGIN_UPPER_RIGHT_MIRRORED 0xC8
#define ORIGIN_LOWER_RIGHT          0xE8
#define ORIGIN_LOWER_RIGHT_MIRRORED 0x68

static int write_to_file(const char *fname, const char *wdata);
int gpio_export(int gpio_num);
int gpio_unexport(int gpio_num);
int gpio_set_direction(int gpio_num, const char* dir);
int gpio_set_value(int gpio_num, const char* value);

void spi_init();
void spi_deinit();
void spi_write_byte(uint8_t val);
void spi_write_chunk(uint8_t* buffer, uint32_t len);

void lcd_reset();
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);

void lcd_buffer(uint8_t* data, uint32_t len);
void lcd_init();
void lcd_deinit();
void lcd_invert(_Bool state);
void lcd_on();
void lcd_off();

void lcd_set_window(int x0, int y0, int x1, int y1);
void lcd_set_default_window();

uint8_t* flatten_image_array(int** pixels, uint32_t width, uint32_t height, gdImagePtr im);
void display_gd_image(gdImagePtr im);
