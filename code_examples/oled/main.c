#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "include/ssd1331.h"

/*
 *   Sample build command: 
 *   gcc -Wall -o ./main.c ./include/ssd1331.c ./main -lwiringPi -lm
*/

/*
 *                      OLED  |  RaspPi 
 *
 *   (Physical: 6, 12)   VCC ->  3.3    (Physical: 17) 
 *   (Physical: 5, 11)   GND ->  GND    (Physical: 25)
 *   (Physical: 2)       DIN ->  MOSI   (Physical: 19)
 *   (Physical: 4)       SLK ->  SCLK   (Physical: 14)
 *   (Physical: 1)       CS  ->  24     (wiringPi: 10)
 *   (Physical: 7)       D/C ->  36     (wiringPi: 27)
 *   (Physical: 8)       RES ->  35     (wiringPi: 24)
 *
*/

int abs(int a);
void swap(int a, int b);
void SSD1331_draw_line(int x0, int y0, int x1, int y1, unsigned short hwColor);

int main(int argc, char *argv[])
{
    FILE *file;
    size_t pixel_size;
    unsigned char buff[OLED_WIDTH * OLED_HEIGHT * 3];
    int deg;
    
    file = fopen("bitmap.bmp", "r");
    pixel_size = 3;

    if(wiringPiSetup() < 0)
    {
        printf("Error occured while GPIO setup.\n");
        return -1;
    }
    
    if (file == NULL) 
    {
        printf("File not found.\n");
        return -1;
    }
    
    fseek(file, 54, 0);
    fread(buff, pixel_size, OLED_WIDTH * OLED_HEIGHT, file);
    fclose(file);

    printf("OLED-SPI example.\n");
    SSD1331_begin();
    
    deg = 0;
    
    while (deg > INT_MIN)
    {
        SSD1331_clear_screen(WHITE);
        SSD1331_bitmap24(0, 0, buff, OLED_WIDTH, OLED_HEIGHT);
        SSD1331_string(43, 5, "12", /*text size*/12, 0, WHITE);
        SSD1331_string(69, 26, "3", 12, 0, WHITE);
        SSD1331_string(47, 48, "6", 12, 0, WHITE);
        SSD1331_string(24, 26, "9", 12, 0, WHITE);
        SSD1331_draw_line(49, 32, 
        49 + 24 * cos(deg) /*x coor*/, 
        32 + 24 * sin(deg) /*y coor*/, RED);
        SSD1331_display();
        delay(500);
        deg -= 6;
    }
    
    printf("Work done\n");
    return 0;
}

int abs(int a)
{
    return (a > 0) ? a : -1 * a;
}

void SSD1331_draw_line(int x0, int y0, int x1, int y1, unsigned short hwColor) 
{
    int dx;
    int dy;
    int max;
    
    int foo;
    int bar;
    int baz;
    int bat;
    
    double tg;
    
    dx = x1 - x0;
    dy = y1 - y0;
    
    if (abs(dx) > abs(dy)) 
    {
        tg = (double)dy/(double)dx;
        max = dx;
    } 
    else
    {
        tg = (double)dx/(double)dy;
        max = dy;
    }
    
    for (foo = 0; foo != max; (max > 0) ? foo++ : foo--) 
    {
        bar = (int)((double)foo * tg);
        baz = x0 + ((abs(dx) > abs(dy)) ? foo : bar); 
        bat = y0 + ((abs(dx) > abs(dy)) ? bar : foo);
        if (baz < OLED_WIDTH && baz > -1 && bat < OLED_HEIGHT && bat > -1) 
        {
            SSD1331_draw_point(baz, bat, hwColor);
        }
    }
}
