/************************************************************************/
/*																		*/
/*	MAXSonar.h  This library supports the Pmod Ultrasonic Module,	    */
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
/*	This the Static Ultrasonic Module Header file          				*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 4/25/2014(IanB): Created											*/
/*																		*/
/************************************************************************/

#ifndef MAXSONAR_H
#define MAXSONAR_H

#include "WProgram.h"
#include "HardwareSerial.h"



/***********************************************
 * Module Object Class Type Declarations       *
 **********************************************/
 
typedef enum{
	INCH = 0,
	CM,
	MM
} UNIT;

typedef enum{
	UART = 0,
	ANALOG,
	PULSEWIDTH
} MODE;


/*******************
 * MAXSonar Class
 ******************/

class MAXSonar
{
	public:
	uint8_t begin(MODE mode, HardwareSerial& serPin, uint8_t RXPin);
	uint8_t begin(MODE mode, uint8_t RXPin);
	uint16_t getDistance(MODE mode, uint8_t pin, UNIT unit);
	uint16_t getDistance(HardwareSerial& serPin, UNIT unit);
	void end(HardwareSerial& serPin, uint8_t RXPin);
	void end(uint8_t RXPin);
	
	private:
	uint16_t getUnits(uint16_t data, UNIT unit);
	
};

#endif //MAXSONAR_H