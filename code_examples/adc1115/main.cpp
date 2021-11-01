#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include<string.h>

#include "./Adafruit_ADS1X15_RPi/Adafruit_ADS1015.h"

#define BUFFSIZE 512
#define err(mess) { fprintf(stderr,"Error: %s.", mess); exit(1); }

void printBits(size_t const size, void const * const ptr);
bool detectLighting(int16_t previous, int16_t current, int16_t threshold);

Adafruit_ADS1115 ads;


int main(int argc, char *argv[])
{
  int fd, n;
  char buf[BUFFSIZE];
  time_t curr_time;
  tm* curr_tm;
  char date_string[100];
  
  mkfifo("ligthing_fifo", 0666);
  if ( (fd = open("ligthing_fifo", O_WRONLY)) < 0)
    err("open")

  uint16_t adc0_prev;
  uint16_t adc0;
  uint32_t threshold = 5000;

  ads.setGain(GAIN_TWO);
  ads.begin();
  
  const char msg[] = "LIGHTING!";
  
  adc0_prev = ads.readADC_SingleEnded(0);
  while (true) {
    adc0 = ads.readADC_SingleEnded(0);
    printBits(sizeof(adc0), &adc0);
    printf(" *** %d\n", adc0);
    if (detectLighting(adc0_prev, adc0, threshold)) {
      time(&curr_time);
	    curr_tm = gmtime(&curr_time);
      strftime(date_string, 50, "%d/%m/%y-%H:%M:%S", curr_tm);
      sprintf(buf, "%s %s\n", date_string, msg);
      printf(buf);
      if ( write(fd, buf, strlen(buf)) != strlen(buf)) { 
        err("write");
      }
    }
    adc0_prev = adc0;
    usleep(100000);
  }
}

//assumes little endian
void printBits(size_t const size, void const * const ptr)
{
  unsigned char *b = (unsigned char*) ptr;
  unsigned char byte;
  int i, j;

  for (i=size-1;i>=0;i--) {
    for (j=7;j>=0;j--) {
      byte = (b[i] >> j) & 1;
      printf("%u", byte);
    }
  }
  //puts("");
}

bool detectLighting(int16_t previous, int16_t current, int16_t threshold)
{
  if (current - previous > threshold)
    return true;
  return false;
}
