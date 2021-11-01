/************************************************************************/
/*																		*/
/*		ENC.h	--	Interface Declarations for ENC.cpp			        */
/*																		*/
/************************************************************************/
/*	Author:		Oliver Jones											*/
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
/*  File Description:													*/
/*																		*/
/*	This header file contains the object class declarations and other	*/
/*	interface declarations need to use the encoder.                		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	10/17/2011(OliverJ): created										*/
/*																		*/
/************************************************************************/

#if !defined(ENC_H)
#define ENC_H

#include "p32_defs.h"

extern "C" {
  #include <stdint.h>
  #include <p32xxxx.h>
}

/* ------------------------------------------------------------ */
/*					Miscellaneous Declarations					*/
/* ------------------------------------------------------------ */

/* Encoder Directions
*/
#define ENC_Right	-1
#define ENC_0		0
#define ENC_Left    1

/* CNCON
*/
// Bits 31-16 Unimplemented
#define ENC_CN_bnOn			15
// Bit 14 Unimplemented
#define ENC_CN_bnSidl13		13
// Bit 12-0 Unimplemented
/*******************************/


/* ------------------------------------------------------------ */
/*					General Type Declarations					*/
/* ------------------------------------------------------------ */

extern "C" {
	void __attribute__((nomips16)) ChangeNoticeHandler(void);
};

/* ------------------------------------------------------------ */
/*					Object Class Declarations					*/
/* ------------------------------------------------------------ */



class ENC
{
	friend		void __attribute__((nomips16)) ChangeNoticeHandler(void);

	public:
		ENC();
		void begin(unsigned int encA, unsigned int encB);
		void end();
		int getDir();
		void AttachInterrupt( void(*function)(int) );
		void DettachInterrupt();

	private:
	    p32_regset *		ifs;	//pointer to interrupt flag register
	    p32_regset *		iec;	//pointer to interrupt enable control register
		bool encInt;
		int encDir;
		int cnMap[33];
		unsigned int prevA;
		unsigned int prevB;
		unsigned int pinA;
		unsigned int pinB;
        
        void populateCN();
		void doEncInterrupt();
		void (*encHandler)(int);
};

#endif