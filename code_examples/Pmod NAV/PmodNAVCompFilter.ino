/************************************************************************/
/*                                                                      */
/* NAVCompFilter  --  PmodNAV Complimentary Filter demo                 */
/* Combines ACL and Gyro data using a complementary filter and          */
/* outputs the angular position (roll, pitch, yaw) over Serial          */
/*                                                                      */
/************************************************************************/
/*  Author:   Andrew Holzer                                             */
/*  Copyright 2017, Digilent Inc.                                       */
/************************************************************************/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*  1/17/2016(AndrewH): created                                         */
/*                                                                      */
/************************************************************************/

/************************************************************************/
/*  Board Support:                                                      */
/*                                                                      */
/*  chipKit Uno with Pmod Shield:     Header JC                         */
/*   (Note: To use this, download and deploy the Board Variant for      */
/*          chipKIT UNO32 W/Pmod Shield from the Digilent website)      */              
/*  Cerebot Mx3ck:                    Header JE                         */
/*  Cerebot Mx4ck:                    Header JB                         */
/*                   (make sure JP3 is on RB15)                         */
/*  Cerebot Mx7ck:                    Header JD                         */
/************************************************************************/

#include <Nav.h>

#define dt 0.01 // 10 ms, This is set from the Timer1 IRQ indicating a sample should happen

/*
 * VARIABLE DEFINITONS
 */
 
Nav myNAV;

float accData[3]; // Gees in the x, y and z axis respectively
float gyrData[3]; // dps around x, y and z axis respectively

float roll = 0, pitch = 0, yaw = 0; // angle around the x, y and z axis respectively
// NOTE: Yaw is included for educational purposes but needs integration with the magnetometer to fix its reference

volatile int flag = 0;

/*
 * Timer1 interrupt routine
 */
void __attribute__((interrupt)) myISR() {
  flag = 1;
  clearIntFlag(_TIMER_1_IRQ);
}

/*
 * SETUP DEFINITIONS
 */
void setupTimer() {
  //initialize timer hardware
  T1CON = 0; // Stop the timer and clear the control register
  T1CONSET = 1 << 6; // set prescalar to 1:16
  
  TMR1 = 0; // clear the timer
  PR1 = 50000; // set the period. 50000 * 16 cycles / 80MHz = .01 seconds per cycle
    
  T1CONSET = 1 << 15; // enable Timer1
}

void setupInt() {
  setIntVector(_TIMER_1_VECTOR, myISR);
  setIntPriority(_TIMER_1_VECTOR, 4, 0);
  clearIntFlag(_TIMER_1_IRQ);
  setIntEnable(_TIMER_1_IRQ);
}

void setup() {
  Serial.begin(9600);
    
  Serial.println("Setting up NAV and initializing the AG");
  myNAV.begin();
  myNAV.InitAG(1, MODE_INST_AG);

  setupTimer();
  setupInt();
}

/*
 * The algorithm to follow here is:
 *    Sample the accelerometer and gyroscope data
 *    
 *    Integrate the gyroscope data with the sample rate time
 *    and add the results into the respective angular position variable
 *    
 *    Use the accelerometer data to find the respective angle value
 *    
 *    Sum the accel data with the respective angular positionvariable,
 *    but weighing them proportionately
 */
void loop() {
  if (flag == 1) {
    
    myNAV.ReadAccelG(accData[0], accData[1], accData[2]);
    myNAV.ReadGyroDps(gyrData[0], gyrData[1], gyrData[2]);
         
    // integrate gyroscope data
    roll += gyrData[0] * dt; // angle about x axis
    pitch += gyrData[1] * dt; // angle about y axis
    yaw += gyrData[2] * dt; // angle about z axis
    
    int forceMagApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);  
    if (forceMagApprox > 0.5 && forceMagApprox < 2) { // if the accelerometer data is within a 0.5-2G, then we will use that data
      float pitchAcc, rollAcc, yawAcc; // angular positions from the accelerometer
    
      rollAcc = atan2f(-1 * (float) accData[1], (float) abs(accData[2])) * 180 / PI;
      roll = 0.98 * roll + 0.02 * rollAcc;
  
      pitchAcc = atan2f((float) accData[0], (float) abs(accData[2])) * 180 / PI;
      pitch  = 0.98 * pitch + 0.02 * pitchAcc;

      yawAcc = atan2f((float) accData[1], (float) accData[0]) * 180 / PI;
      yaw = 0.98 * yaw + 0.02 * yawAcc;
    }

    // print over Serial for display using the Arduino Serial Plotter
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(yaw);
    
    flag = 0;
  }
}
