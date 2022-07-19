/************************************************************************/
/*																		*/
/*	MAXSonar.cpp  This library supports the Pmod MAXSonic Module,		*/
/*                  in particular, providing an easy means of using     */
/*					the module                                			*/
/*																		*/
/************************************************************************/
/*	Author: 	Ian Brech 												*/
/*	Copyright 2014, Digilent Inc.										*/
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
/*  Module Description: 												*/
/*																		*/
/*	This the Static MAXSonar Module Header file             			*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 4/25/2014(IanB): Created											*/
/*																		*/
/************************************************************************/

#include "MAXSonar.h"

/*****************************************************************************
  Function:
    uint8_t begin()
    
  Description:
	Initializes the serial port being used to receive data. Starts the
	conversion sequence by writing the RX pin HIGH.

  Parameters:
    mode: specifies the method to be used for receiving data. UART is the
			only valid parameter for this instance, to ensure correct use
			of the other library functions.
	serPin: specifies the serial port being used. Valid entries are Serial,
			Serial1, Serial2, etc. Based off of the board defs in MPIDE
	RXPin: the digital pin number of the RX pin for the Pmod
  
  Returns:
	1: if setup is being successfully implemented
	0: if setup is being incorrectly implemented. Will occur if UART mode is
		not specified.

  Remarks:

 ***************************************************************************/
uint8_t MAXSonar::begin(MODE mode, HardwareSerial& serPin, uint8_t RXPin)
{	
	if(mode == UART){
		serPin.begin(9600);
		digitalWrite(RXPin, HIGH);
	}
	else{
		return 0;
	}
	return 1;
}
/*****************************************************************************
  Function:
    uint8_t begin()
    
  Description:
	Starts the conversion sequence by writing the RX pin HIGH.

  Parameters:
    mode: specifies the method to be used for receiving data. UART is an
			invalid parameter for this instance, to ensure correct use
			of the other library functions.
	RXPin: the digital pin number of the RX pin for the Pmod
  
  Returns:
	1: if setup is being successfully implemented
	0: if setup is being incorrectly implemented. Will occur if UART mode is
		specified.
  Remarks:

 ***************************************************************************/
uint8_t MAXSonar::begin(MODE mode, uint8_t RXPin)
{	
	if(mode != UART){
		digitalWrite(RXPin, HIGH);
	}
	else{
		return 0;
	}
	return 1;
}

/*****************************************************************************
  Function:
    uint16_t getDistance()
    
  Description:
	Reads available range data from the Pmod.

  Parameters:
    mode: specifies the method to be used for receiving data. UART is an
			invalid parameter for this instance, to ensure correct use
			of the other library functions.
	pin: the digital pin number of either the AN or PW pin for the Pmod
	unit: specifies whether the output needs to be formatted in inches,
			centimeters, or millimeters.
  
  Returns:
	The two-byte, formatted value of the range. Returns a 0 if an invalid 
	mode is used.

  Remarks:

 ***************************************************************************/
uint16_t MAXSonar::getDistance(MODE mode, uint8_t pin, UNIT unit)
{
  int i = 0;
  uint16_t data = 0;
  
  if(mode == ANALOG){
	data = analogRead(pin)/2;
  }
  else if(mode == PULSEWIDTH){
	  //modified from example at www.maxbotix.com/articles/028.htm#arduino
      while(digitalRead(pin) != HIGH);
      while(1){
        delayMicroseconds(147); //147us = 1 inch;
        i++;
        if((digitalRead(pin) == LOW) || (i > 255)){
          if(i > 255){
			break;
          }
          else{
            data = i;
			break;
          }
        }
      }
  }
  else{
	  data = 0; //illegal mode
  }

  return getUnits(data, unit);
}

/*****************************************************************************
  Function:
    uint16_t getDistance()
    
  Description:
	Reads available range data from the Pmod via serial.

  Parameters:
    serPin: specifies the serial port being used. Valid entries are Serial,
			Serial1, Serial2, etc. Based off of the board defs in MPIDE
	unit: specifies whether the output needs to be formatted in inches,
			centimeters, or millimeters.
  
  Returns:
	The two-byte, formatted value of the range.
	0: if the packet is in error (does not have 'R' or return character
	1: if no object is detected and a timeout error occurs.

  Remarks:

 ***************************************************************************/
uint16_t MAXSonar::getDistance(HardwareSerial& serPin, UNIT unit)
{
	int i = 0;
	uint16_t data = 0;
	char rgbTemp[3];
	//char temp;
	
	while(!serPin.available() && (i < 50)){  //timeout
		delay(1);
		i++;
	}
	if(serPin.available()){
		if(serPin.read() == 'R'){
			for(i = 0; i < 3; i++){
				while(!serPin.available());
				rgbTemp[i] = serPin.read();
			}
			while(!serPin.available());
			if(serPin.read() == 13){	//check for return character
				data = atoi(rgbTemp);
			}
			else{
				data = 0;	//packet error
			}
		} 
		else{
			data = 0;  //packet error
		}
	}
	if(i >= 50){
		data = 1;	//timeout error
	}

	return getUnits(data, unit);
}

/*****************************************************************************
  Function:
    void end()
    
  Description:
	Closes the serial port and writes the RX pin to the inactive LOW state

  Parameters:
    serPin: specifies the serial port being used. Valid entries are Serial,
			Serial1, Serial2, etc. Based off of the board defs in MPIDE
	RXPin: the digital pin number of the RX pin for the Pmod
  Returns:
	NONE

  Remarks:

 ***************************************************************************/
void MAXSonar::end(HardwareSerial& serPin, uint8_t RXPin)
{
	serPin.end();
	digitalWrite(RXPin, LOW);
}

/*****************************************************************************
  Function:
    void end()
    
  Description:
	Writes the RX pin to the inactive LOW state

  Parameters:
	RXPin: the digital pin number of the RX pin for the Pmod
  Returns:
	NONE

  Remarks:

 ***************************************************************************/
void MAXSonar::end(uint8_t RXPin)
{
	digitalWrite(RXPin, LOW);
}

/*****************************************************************************
  Function:
    void getUnits()
    
  Description:
	Closes the serial port and writes the RX pin to the inactive LOW state

  Parameters:
    data: the unformatted range data (in inches) from the sensor.
	unit: specifies whether the output needs to be formatted in inches,
			centimeters, or millimeters.
  Returns:
	The formatted range data.

  Remarks:

 ***************************************************************************/
uint16_t MAXSonar::getUnits(uint16_t data, UNIT unit)
{
	float output;

	switch(unit){
		case INCH:
			output = data;
			break;
		case CM:
			output = 2.54 * data;
			break;
		case MM:
			output = 2.54 * data * 10;
			break;
		default:	//default to inches
			output = data;
			break;
	}

	return output;
}