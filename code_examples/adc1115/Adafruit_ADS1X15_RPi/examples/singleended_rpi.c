#include <stdio.h>
#include <unistd.h>

#include "../Adafruit_ADS1015.h"

void printBits(size_t const size, void const * const ptr);

Adafruit_ADS1115 ads;

int main(int argc, char *argv[])
{
  uint16_t adc0;

  ads.setGain(GAIN_EIGHT);
  ads.begin();
  while (true) {
    adc0 = ads.readADC_SingleEnded(0);
    printBits(sizeof(adc0), &adc0);
    printf(" *** %d\n", adc0);
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
