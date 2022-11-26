#include "lcd.h"

int spi_file;
struct spi_ioc_transfer trx;
uint32_t scratch32;

// GPIO functions

static int write_to_file(const char *fname, const char *wdata) {
    int fd;

    fd = open(fname, O_WRONLY | O_NONBLOCK);
    if(fd < 0) {
        printf("Could not open file %s\n", fname);
    }
    write(fd, wdata, strlen(wdata));
    close(fd);

    return 0;
}

int gpio_export(int gpio_num) {
    char gpio_str[4];
    sprintf(gpio_str, "%d", gpio_num);
    return write_to_file(GPIO_PATH GPIO_EXPORT, gpio_str);
}

int gpio_unexport(int gpio_num) {
    char gpio_str[4];
    sprintf(gpio_str, "%d", gpio_num);
    return write_to_file(GPIO_PATH GPIO_UNEXPORT, gpio_str);
}

int gpio_set_direction(int gpio_num, const char* dir) {
    char path_str[40];
    sprintf(path_str, "%s/gpio%d%s", GPIO_PATH, gpio_num, GPIO_DIRECTION);
    return write_to_file(path_str, dir);
}

int gpio_set_value(int gpio_num, const char* value) {
    char path_str[40];
    sprintf(path_str, "%s/gpio%d%s", GPIO_PATH, gpio_num, GPIO_VALUE);
    return write_to_file(path_str, value);
}

// SPI functions

void spi_init() {
	int ret;
	
    spi_file = open(SPI_DEVICE, O_RDWR);
    if(spi_file < 0) {
        printf("Could not open the SPI device\n");
        exit(EXIT_FAILURE);
    }

    ret = ioctl(spi_file, SPI_IOC_RD_MODE32, &scratch32);
    if(ret != 0) {
        printf("Could not read SPI mode\n");
        close(spi_file);
        exit(EXIT_FAILURE);
    }

    scratch32 |= SPI_MODE_0;

    ret = ioctl(spi_file, SPI_IOC_WR_MODE32, &scratch32);
    if(ret != 0) {
        printf("Could not write SPI mode\n");
        close(spi_file);
        exit(EXIT_FAILURE);
    }

    ret = ioctl(spi_file, SPI_IOC_RD_MAX_SPEED_HZ, &scratch32);
    if(ret != 0) {
        printf("Could not read the SPI max speed\n");
        close(spi_file);
        exit(EXIT_FAILURE);
    }

    scratch32 = 100000;

    ret = ioctl(spi_file, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
    if(ret != 0) {
        printf("Could not write the SPI max speed...\n");
        close(spi_file);
        exit(EXIT_FAILURE);
    }
}

void spi_deinit() {
    close(spi_file);
}

void spi_write_byte(uint8_t val) {
	uint8_t tx_buffer[4];
	int ret;
	
	tx_buffer[0] = val;
	
    trx.tx_buf = (unsigned long)tx_buffer;
    trx.rx_buf = (unsigned long)NULL;
    trx.bits_per_word = 0;
    trx.speed_hz = SPI_FREQUENCY;
    trx.delay_usecs = 0;
    trx.len = 1;

    ret = ioctl(spi_file, SPI_IOC_MESSAGE(1), &trx);
}


void spi_write_chunk(uint8_t* buffer, uint32_t len) {  
    struct spi_ioc_transfer ctrx;
    ctrx.tx_buf = (unsigned long)buffer;
    ctrx.rx_buf = (unsigned long)NULL;
    ctrx.len = len;
    ctrx.delay_usecs = 0;
    ctrx.speed_hz = SPI_FREQUENCY;
    ctrx.bits_per_word = 0;

    int ret = ioctl(spi_file, SPI_IOC_MESSAGE(1), &ctrx);
}

// LCD functions

void lcd_reset() {
    gpio_set_value(LCD_PIN_RST, GPIO_VALUE_LOW);
    usleep(10000);
    gpio_set_value(LCD_PIN_RST, GPIO_VALUE_HIGH);  
}

void lcd_command(uint8_t cmd) {
    gpio_set_value(LCD_PIN_DC, LCD_MODE_CMD);
    spi_write_byte(cmd);
}

void lcd_data(uint8_t data) {
    gpio_set_value(LCD_PIN_DC, LCD_MODE_DATA);
    spi_write_byte(data);
}

void lcd_buffer(uint8_t* data, uint32_t len) {
    gpio_set_value(LCD_PIN_DC, LCD_MODE_DATA);
    uint32_t num_chunks = len / SPI_CHUNK_SIZE;
    printf("Chunks: %d, total: %d\n", num_chunks, len);
    for(int i = 0; i < num_chunks; i++) {
        spi_write_chunk(data + i * SPI_CHUNK_SIZE, SPI_CHUNK_SIZE);;
    }
    uint32_t bytes_mod = SPI_CHUNK_SIZE * num_chunks;
    uint32_t bytes_remaining = len - bytes_mod;
    if(bytes_remaining) {
        uint8_t* buff = (uint8_t*)malloc(SPI_CHUNK_SIZE * 8);
        memcpy(buff, data + bytes_mod, bytes_remaining);
        memset(buff + bytes_remaining, 0x00, SPI_CHUNK_SIZE - bytes_remaining);
        spi_write_chunk(buff, SPI_CHUNK_SIZE);
    }
}

// Initializes LCD. Call on startup
void lcd_init() {
    // Set gpio pins
    gpio_export(LCD_PIN_RST);
    gpio_export(LCD_PIN_DC);
    gpio_set_direction(LCD_PIN_RST, GPIO_DIR_OUT);
    gpio_set_direction(LCD_PIN_DC, GPIO_DIR_OUT);

    // Reset lcd
    lcd_reset();

    // Init spi
    spi_init();

    lcd_command(CMD_IFMODE);
    lcd_data(0x00);
    usleep(20000);

    lcd_command(CMD_PXLFMT);
    lcd_data(0x66);             // 18 bits per pixel
    lcd_command(CMD_RDPXLFMT);
    lcd_data(0x66);             // 18 bits per pixel    

    lcd_command(CMD_PWRCTLNOR);
    lcd_command(0x44);

    lcd_command(CMD_VCOMCTL);
    lcd_data(0x00); lcd_data(0x00); lcd_data(0x00); lcd_data(0x00);

    lcd_command(CMD_PGAMCTL);
    lcd_data(0x0f); lcd_data(0x1f); lcd_data(0x1c); lcd_data(0x0c); 
    lcd_data(0x0f); lcd_data(0x08); lcd_data(0x48); lcd_data(0x98);
    lcd_data(0x37); lcd_data(0x0a); lcd_data(0x13); lcd_data(0x04);
    lcd_data(0x11); lcd_data(0x0d); lcd_data(0x00);

    lcd_command(CMD_NGAMCTL);
    lcd_data(0x0f); lcd_data(0x32); lcd_data(0x2e); lcd_data(0x0b); 
    lcd_data(0x0d); lcd_data(0x05); lcd_data(0x47); lcd_data(0x75);
    lcd_data(0x37); lcd_data(0x06); lcd_data(0x10); lcd_data(0x03);
    lcd_data(0x24); lcd_data(0x20); lcd_data(0x00);

    lcd_command(CMD_MADCTL);
    lcd_data(ORIGIN_UPPER_LEFT);

    lcd_command(CMD_SLPOUT);
    lcd_command(CMD_DISPON);
}

// Deinitializes LCD. Call this on exit
void lcd_deinit() {
    gpio_unexport(LCD_PIN_RST);
    gpio_unexport(LCD_PIN_DC);
    
    spi_deinit();
}

// Turns on/off screen color inversion
void lcd_invert(_Bool state) {
    if(state) 
        lcd_command(CMD_INVON);
    else
        lcd_command(CMD_INVOFF);
}

// Turns on screen
void lcd_on() {
    lcd_command(CMD_DISPON);
}

// Turns off screen
void lcd_off() {
    lcd_command(CMD_DISPOFF);
}

// Sets pixel address window for proceeding drawing commands
void lcd_set_window(int x0, int y0, int x1, int y1) {
    lcd_command(CMD_SETCA); // Column address
    lcd_data(x0 >> 8);
    lcd_data(x0 & 0xff);
    lcd_data(x1 >> 8);
    lcd_data(x1 & 0xff);
    lcd_command(CMD_SETPA); // Row address (page address)
    lcd_data(y0 >> 8);
    lcd_data(y0 & 0xff);
    lcd_data(y1 >> 8);
    lcd_data(y1 & 0xff);
}

// Set default 480x320 window
void lcd_set_default_window() {
    lcd_set_window(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

uint8_t* flatten_image_array(int** pixels, uint32_t width, uint32_t height, gdImagePtr im) {
    uint32_t buffer_len = height * width * 3;
    uint8_t* buffer = (uint8_t*)malloc(buffer_len);
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            buffer[(i*width + j)*3] = (uint8_t)((gdImageGetTrueColorPixel(im, j, i) >> 16) & 0xff);
            buffer[(i*width + j)*3 + 1] = (uint8_t)((gdImageGetTrueColorPixel(im, j, i) >> 8) & 0xff);
            buffer[(i*width + j)*3 + 2] = (uint8_t)(gdImageGetTrueColorPixel(im, j, i) & 0xff);
        }
    }

    return buffer;
}

// Converst gd image to array and displays it
void display_gd_image(gdImagePtr im) {
    uint8_t* buffer = flatten_image_array(im->tpixels, LCD_WIDTH, LCD_HEIGHT, im);

    lcd_set_default_window();
    lcd_command(CMD_WRMEM);
    lcd_buffer(buffer, LCD_HEIGHT * LCD_WIDTH * 3);    

    free(buffer);
}
