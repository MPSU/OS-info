/************************************************************************/
/*																		*/
/*	KYPD.h	--	Declaration for KYPD library 	    				    */
/*																		*/
/************************************************************************/
/*	Author:		Michelle Yu												*/
/*	Copyright 2011, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
/*		This file declares functions for KYPD						    */
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	10/06/2011(MichelleY): created										*/
/*																		*/
/************************************************************************/
#if !defined(KYPD_H)
#define KYPD_H

#define KYPD_ROWNUM 4
#define KYPD_COLNUM 4

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */
#include <inttypes.h>
#include <WProgram.h>
#include "KYPD.h"


/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

class KYPD {
public:
    void begin(void);
    void end(void);
    void setPins(unsigned int* row, unsigned int* col);
    void setKeyMap(int table[KYPD_COLNUM][KYPD_ROWNUM]);  
    int getKey(void);
    int getKey(int colRow);
    int getColRow(void);
private:
    unsigned int rowPins[KYPD_ROWNUM];
    unsigned int colPins[KYPD_COLNUM];
    int keyMap[KYPD_COLNUM][KYPD_ROWNUM];

};



#endif