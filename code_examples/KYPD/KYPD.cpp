/************************************************************************/
/*																		*/
/*	KYPD.cpp	--	Definition for KYPD library 	    				*/
/*																		*/
/************************************************************************/
/*	Author:		Michelle Yu												*/
/*	Copyright 2011, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*		This file defines functions for KYPD							*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	10/06/2011(MichelleY): created										*/
/*																		*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */
#include "KYPD.h"


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------------- */
/** void  begin (void)
**
**	Parameters:
**		None
**
**	Return Value:
**      None	
**
**	Errors:
**		TBD
**
**	Description:
**		This function sets col for output and row for input
*/
void  KYPD::begin (void)
{    
    // set col to be output
    for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    {
        pinMode(this->colPins[i], OUTPUT) ;
    }
   
    // set row to be input
    for(int j = 0 ; j < KYPD_ROWNUM ; j++)       
    {
        pinMode(this->rowPins[j], INPUT) ;
    }
    

}

/* ------------------------------------------------------------------- */
/** void  end (void)
**
**	Parameters:
**		None
**
**	Return Value:
**      None	
**
**	Errors:
**		TBD
**
**	Description:
**		This function release all pins 
*/
void  KYPD::end (void)
{    
    // set col to be input
     for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    {
        pinMode(this->colPins[i], INPUT) ;
    }
    
}

/* ------------------------------------------------------------------- */
/** void setPins(unsigned int*  row, unsigned int* col)
**
**	Parameters:
**		row - pointer to an array of pins to be row 
**      col - pointer to an array of pins to be col
**
**	Return Value:
**      None	
**
**	Errors:
**		TBD
**
**	Description:
**		This function sets the pins for the row and column
*/
void  KYPD::setPins(unsigned int*  row, unsigned int* col)
{    
    // set col
    for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    {
        this->colPins[i] = col[i];
    }
   
    // set row 
    for(int j = 0 ; j < KYPD_ROWNUM ; j++)       
    {
        this->rowPins[j] = row[j];
    }
    
     
}
/* ------------------------------------------------------------------- */
/** void setKeyMap(int table[KYPD_COLNUM][KYPD_ROWNUM])
**
**	Parameters:
**		table - pointer to an 2D array for the custom key mapping 
**
**	Return Value:
**      None	
**
**	Errors:
**		TBD
**
**	Description:
**		This function maps table in to keymap
*/
void KYPD::setKeyMap(int table[KYPD_COLNUM][KYPD_ROWNUM])
{    

  for(int i = 0 ; i < KYPD_COLNUM ; i++)       
  {
    for(int j = 0 ; j < KYPD_ROWNUM ; j++)        
    {
         this->keyMap[i][j] = table[i][j];
    }
  }    
}
/* ------------------------------------------------------------------- */
/** int getKey(void)
**
**	Parameters:
**		None
**
**	Return Value:
**      int - key pressed, retuens -1 if no key was pressed
**
**	Errors:
**		TBD
**
**	Description:
**		This function returns the corresponding value in the keymap of the pressed 
** key by driving each column low one at a time and check the ouput of the row pins.
*/
int KYPD::getKey(void)
{    
    
    //initialize col to be high
    for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    {
        digitalWrite(this->colPins[i], HIGH);
    }
    
    
    for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    { 
        //start to scan colPins[i]
        digitalWrite(this->colPins[i],LOW);
        for(int j = 0 ; j < KYPD_ROWNUM ; j++)       
        {         
            //row[j]
            if(digitalRead(this->rowPins[j])== 0)
            {
                return this->keyMap[i][j];   
            }
        }
        //stop scanning colPins[i]
        digitalWrite(this->colPins[i],HIGH);
    }
    
    return -1;
    
   
}

/* ------------------------------------------------------------------- */
/** int getKey(int colRow)
**
**	Parameters:
**		colRow - a 32 bit column-row indicator, where the first 16 bit is 
**               column and the last 16 bit is row
**
**	Return Value:
**      int - key pressed
**
**	Errors:
**		TBD
**
**	Description:
**		This function returns the corresponding value in the keymap 
** of the 32bit column-row indicator
*/
int KYPD::getKey(int colRow)
{   
    return this->keyMap[colRow>>16][0xFFFF&colRow];           
}

/* ------------------------------------------------------------------- */
/** int getColRow(void)
**
**	Parameters:
**		None
**
**	Return Value:
**      int - column-row indicator
**            The most significant 16 bit is column and the least significant 16 bit is row
**            retuens -1 if no key were pressed
**
**            
**
**	Errors:
**		TBD
**
**	Description:
**		This function returns the column-row indicator by driving each column
** low one at a time and check the output of the row pins..
*/
int KYPD::getColRow(void)
{    
    int colRow = 0;
    
    //initialize col to be high
    for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    {
        digitalWrite(this->colPins[i], HIGH);
    }
    
   
    for(int i = 0 ; i < KYPD_COLNUM ; i++)       
    { 
         //start to scan colPins[i]
        digitalWrite(this->colPins[i],LOW);
        for(int j = 0 ; j < KYPD_ROWNUM ; j++)       
        { 
          
            //row[j]
            if(digitalRead(this->rowPins[j])== 0)
            {
                return (i<<16|j);
                
            }
        }
        //stop scanning colPins[i]
        digitalWrite(this->colPins[i],HIGH);
    }
    
    return -1;
    
   
}


