#include <stdio.h>
#include <unistd.h>

#include "../Adafruit_ADS1015.h"

void printBits(size_t const size, void const * const ptr);

Adafruit_ADS1115 ads;

int main(int argc, char *argv[])
{
  int16_t adc0;
  double milliVolts;
  double bits2milliVoltsFactor;
  
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.setGain(GAIN_FOUR);
  bits2milliVoltsFactor = 0.03125; // remember to change this according to gain

  ads.begin();
  while (true) {
    adc0 = ads.readADC_Differential_0_1();
    milliVolts = adc0 * bits2milliVoltsFactor; 
    printBits(sizeof(adc0), &adc0);
    printf(" *** %5d *** %f\n", adc0, milliVolts);
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
