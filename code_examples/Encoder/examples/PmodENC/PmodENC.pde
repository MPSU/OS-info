/************************************************************************/
/*									*/
/*  PmodENC.pde	-- Example Sketch for PmodENC				*/
/*									*/
/************************************************************************/
/*  Author:	Oliver Jones						*/
/*  Copyright (c) 2011, Digilent Inc.  	    				*/
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
/*  Module Description:							*/
/*									*/
/*  This sketch is an example on on how to read the status of a PmodENC */
/* with the ENC library. This sketch prints "Button is pressed" on the	*/	
/* Serial monitor when the rotary button is pressed and switch LED1     */
/* on and off when the switch on the PmodENC is switched.               */
/************************************************************************/
/*  Revision History:							*/
/*									*/
/*  10/21/2011(Oliver J): Created                                       */
/*									*/
/************************************************************************/


#include <ENC.h>

// pin number of the pin that BTN is attach to 
const int btn = 22;
// pin number of the pin that SWT is attach to 
const int swt = 23;

volatile int i;

void incrementI(int dir);

ENC myENC;

void setup() {
  
    Serial.begin(9600);
    Serial.println("PmodENC Demo");
    
    //set swt and btn as input
    pinMode(swt, INPUT);
    pinMode(btn, INPUT);
    
    //Set LD1 to output
    pinMode(PIN_LED1, OUTPUT);
    
    //Call begin to initialize the change notices for pin A and Pin B
    //in this example CN2 is used for pin A and CN3 for pin B
    myENC.begin(2, 3);
    
   //Assigns the passed in function to the end of the Interrupt Service 
   //Procedure so that every time the encoder turns the function will be 
   //called passing the direction the encoder
    myENC.AttachInterrupt(incrementI);
}

void loop() {
  
  if(digitalRead(btn))
  {
    Serial.println("Button is pressed");
  }
  
  if(digitalRead(swt))
  {
    digitalWrite(PIN_LED1, HIGH);
  }
  else
  {
    digitalWrite(PIN_LED1, LOW);
  }
  
  delay(200);
}

void incrementI(int dir)
{
  i += dir;
  
  Serial.print("I = ");
  Serial.println(i,DEC);
  
}
