#include <iostream>
#include <wiringPi.h>
#include "libSonar.h"

using namespace std;

int trigger = 2;
int echo = 3;

int main()
{
    if (wiringPiSetup() == -1)
        return -1;

    Sonar sonar;
    sonar.init(trigger, echo);

    cout << "Starting..."  << endl;

    while(1){
        cout << "Distance is " << sonar.distance(100000) << " cm." << endl;
        delay(1000);
    }
}
