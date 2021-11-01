/************************************************************************/
/*																		*/
/*	ENC.cpp	--	Encoder Driver for ENC					                */
/*																		*/
/************************************************************************/
/*	Author: 	Oliver Jones											*/
/*	Copyright 2011, Digilent Inc.										*/
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
/*	This module contains the implementation of the object class that	*/
/*	forms the chipKIT interface to the graphics driver functions for	*/
/*	the OLED display on the Digilent Basic I/O Shield.					*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	10/26/2011(OliverJ): created										*/
/*  12/15/2011(MichelleY): modified to use the variable defined in      */
/*                         board_defs.h                                 */
/*																		*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "ENC.h"
#include <pins_arduino.h>

extern "C" {
  #include <stdint.h>
  #include	<sys\attribs.h>
  #include <wiring.h>
}

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

static ENC * pdEnc = 0;


/* ------------------------------------------------------------ */
/*			Encoder Driver Object Instantiation					*/
/* ------------------------------------------------------------ */
/* Instantiate a single static instance of this object class
*/

/* ------------------------------------------------------------------- */
/** void ENC::ENC()
**
**	Parameters:
**		None
**
**
**	Return Value:
**		None
**
**
**	Description:
**		This is the constructor for the ENC object. Calls populateCN
**  to populate cnMap
*/
ENC::ENC()
{
	this->populateCN();
	
}

/* ------------------------------------------------------------------- */
/** void ENC::begin(unsigned int encA, unsigned int encB)
**
**	Parameters:
**		encA – the change notice number that is attach to pin A
**  	encB – the change notice number that is attach to pin B 	 	
**   e.g.  Using change notice 1 (CN1) on pin A, encA will simply be 1
**
**	Return Value:
**		None
**
**	Description:
**		Initializes the Change Notice interrupts
*/
void ENC::begin(unsigned int encA, unsigned int encB)
{

    this->pinA = cnMap[encA];
    this->pinB = cnMap[encB];    
	
	
	if(this->pinA != -1 && this->pinB != -1)
	{   
        p32_regset *	ipc;	//interrupt priority control register set
	    int				vec_shift;


	    /* The interrupt flag and enable control register addresses and
	    ** the bit numbers for the flag bits can be computed from the
	    ** IRQ number for the Change Notice. The irq parameter specifies the IRQ
	    ** for the CN interrupt. There are 32 IRQ bits in each IFS
	    ** and IEC register. For each IFS register, there is a SET, CLR,
	    ** and INV register, so the distance (in dwords) from IFS0 to IFS1
	    ** is 4.
	    ** The interrupt priorty control register address and the priority bits
	    ** can be computed from the vector number. Each IPC register contains the
	    ** the priority bits for four vectors. Each byte of an IPC registger contains
	    ** the priority and sub-priority bits arranged such that  bits 0-1 are
	    ** the sub-priority, bits 2-4 the priority, and bits 5-7 unused.
	    */
	    this->ifs = ((p32_regset *)&IFS0) + (_CHANGE_NOTICE_IRQ/ 32);	//interrupt flag register set
	    this->iec = ((p32_regset *)&IEC0) + (_CHANGE_NOTICE_IRQ / 32);	//interrupt enable control reg set   

	    /* Compute the address of the interrupt priority control
	    ** registers used by the change notice
	    */
	    ipc = ((p32_regset *)&IPC0) + (_CHANGE_NOTICE_VECTOR / 4);	//interrupt priority control reg set

	    /* Compute the number of bit positions to shift to get to the
	    ** correct position for the priority bits for this IRQ.
	    */
	    vec_shift = 8 * (_CHANGE_NOTICE_VECTOR % 4);

	    /* Set the interrupt privilege level and sub-privilege level
	    */
	    ipc->clr = 	(0x1F << vec_shift);
	    ipc->set = ((_CN_IPL_IPC << 2)|_CN_SPL_IPC) << vec_shift;

	    /* Clear the interrupt flags, and set the interrupt enables for the
	    ** interrupts used by the CN.
	    */
	    this->ifs->clr = 1 << (_CHANGE_NOTICE_IRQ % 32);	//clear CN interrupt flags

        CNCONSET = (1 << ENC_CN_bnOn);
	    CNENSET = (1 << encA) | (1 << encB);

	    this->iec->set = 1 << (_CHANGE_NOTICE_IRQ % 32);	//enable the CN interrupt
	    
	    
	    pinMode(this->pinA, INPUT);
	    pinMode(this->pinB, INPUT);

	    digitalRead(this->pinA);
	    digitalRead(this->pinB);
    
        pdEnc = this;
        
	 }
}
/* ------------------------------------------------------------------- */
/** void ENC::end()
**
**	Parameters:
**		None
**
**
**	Return Value:
**		None
**
**
**	Description:
**	    Turns Change Notice Interrupts off
*/
void ENC::end()
{
	CNCONCLR = (1 << ENC_CN_bnOn);
	iec->clr = 1 << (_CHANGE_NOTICE_IRQ % 32);	//disable the CN interrupt
	this->encInt = 0;
}
/* ------------------------------------------------------------------- */
/** int ENC::getDir()
**
**	Parameters:
**		None
**
**
**	Return Value:
**		Returns 1 of three defined values; left, right, and stationary.
**        ENC_Right	-1
**        ENC_0		0
**        ENC_Left	1
**
**	Description:
**		After returning direction the direction will be set to ENC_0 
**  until the encoder has been moved
*/
int ENC::getDir()
{
	return this->encDir;
}
/* ------------------------------------------------------------------- */
/** void ENC::AttachInterrupt( void(*function)(int) )
**
**	Parameters:
**		A pointer to a function with one parameter of int type
**
**
**	Return Value:
**		None
**
**
**	Description:
**		Assigns the passed in function to the end of the Interrupt Service 
**  Procedure so that every time the encoder turns the function will be 
**  called passing the direction the encoder
*/
void ENC::AttachInterrupt( void(*function)(int) )
{
	this->encInt = true;
	encHandler = function;
}
/* ------------------------------------------------------------------- */
/** void ENC::DettachInterrupt()
**
**	Parameters:
**		A pointer to a function with one parameter of int type
**
**
**	Return Value:
**		None
**
**
**	Description:
**		Removes the function that was attached by AttachInterrupt().
*/
void ENC::DettachInterrupt()
{
	this->encInt = false;
}
/* ------------------------------------------------------------------- */
/** void ENC::doEncInterrupt()
**
**	Parameters:
**		None
**
**
**	Return Value:
**		None
**
**
**	Description:
**		handles the change notice interrupt, check for change in directions
*/
void ENC::doEncInterrupt()
{
	unsigned int A = 0;
	unsigned int B = 0;

	A = digitalRead(this->pinA);
	B = digitalRead(this->pinB);

	if(A && !B && this->prevB) {
		this->encDir = ENC_Right;
	}
	else if(!A && B && this->prevA) {
		this->encDir = ENC_Left;
	}
	else
	{
		this->encDir = ENC_0;
	}

	this->prevA = A;
	this->prevB = B; 

	if(this->encInt && this->encDir)
	{
		encHandler(this->encDir);
	}
}
/* ------------------------------------------------------------------- */
/** void ENC::populateCN()
**
**	Parameters:
**		None
**
**
**	Return Value:
**		None
**
**
**	Description:
**		populate cnMap with the CN pin definitions in Board_Def.h
*/
void ENC::populateCN()
{
    #if defined PIN_CN0
        cnMap[0] = PIN_CN0;
    #else 
        cnMap[0] = -1;
    #endif
    
    #if defined PIN_CN1
        cnMap[1] = PIN_CN1;
    #else 
        cnMap[1] = -1;
    #endif
    
    #if defined PIN_CN2
        cnMap[2] = PIN_CN2;
    #else 
        cnMap[2] = -1;
    #endif  
    
    #if defined PIN_CN3
        cnMap[3] = PIN_CN3;
    #else 
        cnMap[3] = -1;
    #endif
    
    #if defined PIN_CN4
        cnMap[4] = PIN_CN4;
    #else 
        cnMap[4] = -1;
    #endif
    
    #if defined PIN_CN5
        cnMap[5] = PIN_CN5;
    #else 
        cnMap[5] = -1;
    #endif
    
    #if defined PIN_CN6
        cnMap[6] = PIN_CN6;
    #else 
        cnMap[6] = -1;
    #endif
    
    #if defined PIN_CN7
        cnMap[7] = PIN_CN7;
    #else 
        cnMap[7] = -1;
    #endif
    
    #if defined PIN_CN8
        cnMap[8] = PIN_CN8;
    #else 
        cnMap[8] = -1;
    #endif
    
    #if defined PIN_CN9
        cnMap[9] = PIN_CN9;
    #else 
        cnMap[9] = -1;
    #endif
    
    #if defined PIN_CN10
        cnMap[10] = PIN_CN10;
    #else 
        cnMap[10] = -1;
    #endif
    
    #if defined PIN_CN11
        cnMap[11] = PIN_CN11;
    #else 
        cnMap[11] = -1;
    #endif
    
    #if defined PIN_CN12
        cnMap[12] = PIN_CN12;
    #else 
        cnMap[12] = -1;
    #endif  
    
    #if defined PIN_CN13
        cnMap[13] = PIN_CN13;
    #else 
        cnMap[13] = -1;
    #endif
    
    #if defined PIN_CN14
        cnMap[14] = PIN_CN14;
    #else 
        cnMap[14] = -1;
    #endif
    
    #if defined PIN_CN15
        cnMap[15] = PIN_CN15;
    #else 
        cnMap[15] = -1;
    #endif
    
    #if defined PIN_CN16
        cnMap[16] = PIN_CN16;
    #else 
        cnMap[16] = -1;
    #endif
    
    #if defined PIN_CN17
        cnMap[17] = PIN_CN17;
    #else 
        cnMap[17] = -1;
    #endif
    
    #if defined PIN_CN18
        cnMap[18] = PIN_CN18;
    #else 
        cnMap[18] = -1;
    #endif
    
    #if defined PIN_CN19
        cnMap[19] = PIN_CN19;
    #else 
        cnMap[19] = -1;
    #endif  
      
    #if defined PIN_CN20
        cnMap[20] = PIN_CN20;
    #else 
        cnMap[20] = -1;
    #endif
    
    #if defined PIN_CN21
        cnMap[21] = PIN_CN21;
    #else 
        cnMap[21] = -1;
    #endif
    
    #if defined PIN_CN22
        cnMap[22] = PIN_CN22;
    #else 
        cnMap[22] = -1;
    #endif  
    
    #if defined PIN_CN23
        cnMap[23] = PIN_CN23;
    #else 
        cnMap[23] = -1;
    #endif
    
    #if defined PIN_CN24
        cnMap[24] = PIN_CN24;
    #else 
        cnMap[24] = -1;
    #endif
    
    #if defined PIN_CN25
        cnMap[25] = PIN_CN25;
    #else 
        cnMap[25] = -1;
    #endif
    
     #if defined PIN_CN26
        cnMap[26] = PIN_CN26;
    #else 
        cnMap[26] = -1;
    #endif
    
    #if defined PIN_CN27
        cnMap[27] = PIN_CN27;
    #else 
        cnMap[27] = -1;
    #endif
    
    #if defined PIN_CN28
        cnMap[28] = PIN_CN28;
    #else 
        cnMap[28] = -1;
    #endif
    
    #if defined PIN_CN29
        cnMap[29] = PIN_CN29;
    #else 
        cnMap[29] = -1;
    #endif
    
    #if defined PIN_CN30
        cnMap[30] = PIN_CN30;
    #else 
        cnMap[30] = -1;
    #endif
    
    #if defined PIN_CN31
        cnMap[31] = PIN_CN31;
    #else 
        cnMap[31] = -1;
    #endif
    
    #if defined PIN_CN32
        cnMap[32] = PIN_CN32;
    #else 
        cnMap[32] = -1;
    #endif  
}


extern "C" {
/* ------------------------------------------------------------------- */
/** void __ISR(_CHANGE_NOTICE_VECTOR, _CN_IPL_ISR) ChangeNoticeHandler(void)
**
**	Parameters:
**		None
**
**
**	Return Value:
**		None
**
**
**	Description:
**		interrupt service routine for change notice.
*/
void __ISR(_CHANGE_NOTICE_VECTOR, _CN_IPL_ISR) ChangeNoticeHandler(void)
{
    p32_regset *		ifs;	//pointer to interrupt flag register
	if(pdEnc != 0) {
		pdEnc->doEncInterrupt();
	}
    
    ifs = ((p32_regset *)&IFS0) + (_CHANGE_NOTICE_IRQ/ 32);	//interrupt flag register set
    ifs->clr = 1 << (_CHANGE_NOTICE_IRQ % 32);	//clear CN interrupt flags
}

}