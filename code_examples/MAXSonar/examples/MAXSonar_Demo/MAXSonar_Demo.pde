/************************************************************************/
/*                                                                      */
/*  MAXSonarSensor  --  Illustrate Use of PmodMAXSonar with chipKIT MX3 */
/*                                                                      */
/************************************************************************/
/*  Author: Ian Brech                                                   */
/*  Copyright 2014, Digilent Inc, All rights reserved.                  */
/************************************************************************/
/*
  This program is free software; you can redistribute it and/or
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
/*  Module Description:                                                 */
/*                                                                      */
/* This example illustrates using the Digilent MAXSonar library to      */
/* communicate with a PmodMAXSonar from a Cerebot MX3cK.                */
/*                                                                      */
/* This demo illustrates the use of the PmodMAXSonar to retrieve range  */
/* information. The user must specify the mode of communication         */
/* (UART, analog, or PWM) and the units to be read.                     */
/*                                                                      */
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
  /*  05/14/2014(IanB): Created                                         */
/*                                                                      */
/************************************************************************/

/* ------------------------------------------------------------ */
/*                Include File Definitions                      */
/* ------------------------------------------------------------ */
#include <p32xxxx.h>
#include <MAXSonar.h>

/* ------------------------------------------------------------ */
/*                Local Type Definitions                        */
/* ------------------------------------------------------------ */
/* Must be defined for the board being used                     */
/* This demo uses the top row of connector JC on the MX3cK      */
/* PmodMAXSonar pin descriptions:                               */
/*    Pin 1: AN                                                 */
/*    Pin 2: RX                                                 */
/*    Pin 3: TX                                                 */
/*    Pin 4: PW                                                 */
/* ------------------------------------------------------------ */
#define ANpin  16
#define RXpin  17
#define PWpin  19

/* ------------------------------------------------------------ */
/*                Global Variables                              */
/* ------------------------------------------------------------ */
uint16_t data;
MAXSonar sensor;

/* ------------------------------------------------------------ */
/*                Local Variables                               */
/* ------------------------------------------------------------ */
/* Set the values to be used:                                   */
/*      units: INCH: returns measurement in inches              */
/*             CM: returns measurement in centimeters           */
/*             MM: returns measurement in millimeters           */
/*      mode:  UART: retrieves data through UART                */
/*             ANALOG: retrieves data through the analog pin    */
/*             PULSEWIDTH: retrieves data by measuring a        */
/*                         pulsewidth on the PW pin             */
/* ------------------------------------------------------------ */
const UNIT units = INCH;
const MODE mode = UART;
/* ------------------------------------------------------------ */
/*                Forward Declarations                          */
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*                Procedure Definitions                         */
/* ------------------------------------------------------------ */
/*  setup ()
**
**  Parameters:
**    none
**
**  Return Value:
**    none
**
**  Errors:
**    none
**
**  Description:
**    Initialize the system.
*/
void setup()
{
  pinMode(RXpin, OUTPUT);  //Make sensor's RX pin an output from the uC
  digitalWrite(RXpin, LOW);  //write sensor's RX pin low to stop sensor activity
  pinMode(PWpin, INPUT);
  
  Serial.begin(9600);
  delay(250);  //allow for powerup before sending RX command
  digitalWrite(RXpin, HIGH);
  delay(49);  //delay for calibration after RX pin is left open or held high
  delay(49);  //delay for initial range reading; 
  Serial.println("Getting Range Data:");
}

/* ------------------------------------------------------------ */
/***  loop
**
**  Parameters:
**    none
**
**  Return Value:
**    none
**
**  Errors:
**    none
**
**  Description:
**    Main application loop. Retreives sensor data every 500ms
*/
void loop()
{
  switch(mode){
    case UART:
      sensor.begin(UART, Serial1, RXpin);
      data = sensor.getDistance(Serial1, units);
      Serial.print(data);
      sensor.end(Serial1, RXpin);  //End Serial communication to make PmodUS not buffer readings
      break;
    case ANALOG:
      sensor.begin(ANALOG, RXpin);
      data = sensor.getDistance(ANALOG, ANpin, units);
      sensor.end(RXpin);
      Serial.print(data);
      break;
    case PULSEWIDTH:
      sensor.begin(PULSEWIDTH, RXpin);
      data = sensor.getDistance(PULSEWIDTH, PWpin, units);
      sensor.end(RXpin);
      Serial.print(data);
      break;
    default:
      break;
  }
  
  if(units == INCH){
    Serial.println(" inches");
  }
  else if(units == CM){
    Serial.println(" centimeters");
  }
  else if(units == MM){
    Serial.println(" millimeters");
  }
  delay(500);
}

