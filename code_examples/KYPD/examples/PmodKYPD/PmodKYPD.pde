/************************************************************************/
/*									*/
/*  PmodKYPD.pde	-- Example Sketch for PmodKYPD			*/
/*									*/
/************************************************************************/
/*  Author:	Michelle Yu				        	*/
/*  Copyright (c) 2011, Digilent Inc.  	    			        */
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
/*  This sketch is an example on how to get the value of a key          */
/* presseed on a PmodKYPD with the KYPD library.                        */
/************************************************************************/
/*  Revision History:							*/
/*									*/
/*  10/17/2011(Michelle Yu): Created                                    */
/*									*/									
/************************************************************************/


/* ------------------------------------------------------------ */
/*		Include File Definitions			*/
/* ------------------------------------------------------------ */

#include <KYPD.h>
/* ------------------------------------------------------------ */
/*		Local Variables					*/
/* ------------------------------------------------------------ */
// initialize key and colrow to -1 indicate no key is pressed
int key = -1;
int colRow = -1;
KYPD myKYPD;

//There are 4 columns and 4 rows for the PmodKYPD
//the upper row on the pmod connector of the PmodKYPD is column and
//the lower row on the pmod connector of the PmodKYPD is row
//the following code initialize two arrays of 4 to the pin number  
//of the board that is connected to the pmod connector
//in this example the pins are assigned to be the pins on
//connector JA of the Cerebot Mcks.
unsigned int col[4]={3, 2, 1, 0};
unsigned int row[4]={7, 6, 5, 4};

//Define keymap to the values on the PmodKYPD
int  keyTable[4][4]={{1, 4, 7, 0},    //col 1
                     {2, 5, 8, 15},    //col 2
                     {3, 6, 9, 14},    //col 3
                     {10, 11, 12, 13}};   //col 4


//main program starts here 
void setup()
{
  Serial.begin(9600);
 
  //set the pins 
  myKYPD.setPins(row, col);
 
  //set the keyMap 
  myKYPD.setKeyMap(keyTable);

  myKYPD.begin();
  
  //OPTIONAL!! Free pins to do other things 
  // myKYPD.end();
}
void loop() 
{
  //Method 1: 
  //GetKey by get the column and row first
   colRow = myKYPD.getColRow();
   if( colRow != -1 )
   {
     Serial.print("col:"); 
     Serial.print(colRow>>16); 
     Serial.print(" row:"); 
     Serial.print(0xFFFF&colRow);
     key = myKYPD.getKey(colRow);
     Serial.print(" is value:"); 
     Serial.println(key);
   }
   
  //Method 2:
  //simply GetKey 
   key = myKYPD.getKey();
   if( key != -1 )
   {
     Serial.println(key);
   }
   
   //time in between keys 100 ms
   delay(100);
}
