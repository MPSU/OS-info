/************************************************************************/
/*																		*/
/*	Nav.cpp	--	Nav Module Driver										*/
/*																		*/
/************************************************************************/
/*	Author: 	Monica Ignat											*/
/*	Copyright 2015, Digilent Inc.										*/
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
/*	forms the chipKIT interface to the driver functions for				*/
/*	the Nav module on the Digilent chipKIT™ Pro MX4 board. The library 	*/
/*	can be succesfully used with any Digilent microcontroller board		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	10/26/2015(MonicaI): created										*/
/*	05/25/2017(CristianF): added parameters to begin function for 		*/
/*	all the NAV signals definition, allowing portability on other 		*/
/*	Digilent microcontroller boards										*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "Nav.h"
#include <inttypes.h>
#include <DSPI.h>


/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Nav Definitions									*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*-------------------------------------------------------------*/
/*	Basic Device Control and Initialization Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*	void Nav::Nav()
**
**	Parameters:
**		
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Default constructor, initializes the SPI access object and some global variables to default values. 
*/
Nav::Nav()
{
	pdspi = NULL;
	m_GRangeLSB = 0;
	m_DPSRangeLSB = 0;
	m_GaussRangeLSB = 0;
}
/* ------------------------------------------------------------ */
/*	void Nav::~Nav()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Destructor. Clean up.
*/
Nav::~Nav()
{
	if ( NULL != pdspi ) {
		delete pdspi;
	}
}

/* ------------------------------------------------------------ */
/*	void Nav::begin(uint8_t prtSpi, uint8_t SSPinAG, uint8_t SSPinMAG, uint8_t SSPinALT, int8_t DRDYPinMAG, int8_t INTPin)
**
**	Parameters:
**		uint8_t prtSpi - the number of the DSPI object to be used to access the desired SPI interface
**		uint8_t SSPinAG - SS pin for A/G instrument, specific to each master board
**		uint8_t SSPinMAG - SS pin for MAG instrument, specific to each master board
**		uint8_t SSPinALT - SS pin for ALT instrument, specific to each master board
**		uint8_t DRDYPinMAG - pin number for data ready signal coming from magnetometer
**		uint8_t INTPin - pin number for INT signal coming from either one of the three instruments
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes the SPI interface and defines the SS pins for each instrument, along with DRDY and INT pin. 
**		When SSPinAG parameter is transmitted with -1 value, the SS for AG instrument will be used as the default SS pin for prtSpi interface.
**		When SSPinMAG is -1, then the SS for MAG instrument will have the value of the SS for AG instrument incremented by 6, according to the position in the Pmod connector.
**		When SSPinALT is -1, then the SS for ALT instrument will have the value of the SS for AG instrument incremented by 7, according to the position in the Pmod connector.
**		When DRDY is -1, then the Data ready pin for the MAG instrument will have the value of the SS for AG instrument incremented by 5, according to the position in the Pmod connector.
**		When INT is -1, then the INT pin comming from either one of the instruments will have the value of the SS for AG instrument incremented by 4, according to the position in the Pmod connector.
*/
void Nav::begin(uint8_t prtSpi, int8_t SSPinAG, int8_t SSPinMAG, int8_t SSPinALT, int8_t DRDYPinMAG, int8_t INTPin)
{
	int8_t iSSPinAGDefault;
	prtSel = prtSpi;
	//initialize global variables with the values of the SS parameters and DRDY, INT pins
	m_GRangeLSB = GetXLRangeLSB(PAR_XL_2G);	// the startup range for the accelerometer is +/- 2g, which corresponds to a LSB value of 0.061mg/LSB
	m_DPSRangeLSB = GetGRangeLSB(PAR_G_245DPS);	// the startup range for the gyro is +/- 245dps, which corresponds to a LSB value of 8.75mdps/LSB
	m_GaussRangeLSB = GetMAGRangeLSB(PAR_MAG_4GAUSS);// the startup range for the magnetometer is +/- 4Gauss, which corresponds to a LSB value of 0.14mGauss/LSB
	Pref = 1013.25; //reference pressure corresponding to sea level
	switch (prtSel) {

#if (NUM_DSPI_PORTS > 3)
		case 3:
			pdspi = new DSPI3();
			iSSPinAGDefault = PIN_DSPI3_SS;	// default SS pin for DSPI3
			break;
#endif

#if (NUM_DSPI_PORTS > 2)
		case 2:
			pdspi = new DSPI2();
			iSSPinAGDefault = PIN_DSPI2_SS;	// default SS pin for DSPI2
			break;
#endif

#if (NUM_DSPI_PORTS > 1)
		case 1:
			pdspi = new DSPI1();
			iSSPinAGDefault = PIN_DSPI1_SS;	// default SS pin for DSPI1
			break;
#endif

		default:
			pdspi = new DSPI0();
			iSSPinAGDefault = PIN_DSPI0_SS;	// default SS pin for DSPI0
			
	}
	m_SSPinAG 		= (SSPinAG != -1) 	? SSPinAG	: iSSPinAGDefault;
	m_SSPinMAG 		= (SSPinMAG != -1) 	? SSPinMAG	: m_SSPinAG + 6;
	m_SSPinALT 		= (SSPinALT != -1) 	? SSPinALT	: m_SSPinAG + 7;
	m_DRDYPinMAG 	= (DRDYPinMAG != -1)? DRDYPinMAG: m_SSPinAG + 5;
	m_INTPin 		= (INTPin != -1)	? INTPin	: m_SSPinAG + 4;
	NavHostInit();
	NavDevInit();
}

/* ------------------------------------------------------------ */
/*	void Nav::begin()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes the SPI interface and defines the SS pins for each instrument, along with DRDY and INT pin. 
**		It uses the most common settings: DSPI0 interface and all pins situated in the Pmod connector.
*/
void Nav::begin()
{
	begin(0, -1, -1, -1, -1, -1);
}
/* ------------------------------------------------------------ */
/*	void Nav::end(void)
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		
**		Performs the power off sequences floats all pins and releases the PIC32 resources used by the Nav interface.
**		
*/
void Nav::end()
{
	NavDevTerm();
	NavHostTerm();
	m_GRangeLSB = 0;
	m_DPSRangeLSB = 0;
	m_GaussRangeLSB = 0;
}

/* ------------------------------------------------------------ */
/*	Nav::NavHostInit()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs SPI interface initialization to prepare for use
**		of the Nav module. Sets the module pins as output or input, according to their functionalities

*/
void Nav::NavHostInit()
{
	
	pdspi->begin();	// this defines SS pin as output, sets SS high
	pdspi->setMode(DSPI_MODE3);
	pdspi->setSpeed(4000000);
	
	//output pins for accelerometer, magnetometer and altimeter SS 
	pinMode(m_SSPinAG, OUTPUT);
	digitalWrite(m_SSPinAG, HIGH);
	pinMode(m_SSPinMAG, OUTPUT);
	digitalWrite(m_SSPinMAG, HIGH);
	pinMode(m_SSPinALT, OUTPUT);
	digitalWrite(m_SSPinALT, HIGH);
	//input pins for data ready and interrupt coming from PmodNAV
	pinMode(m_DRDYPinMAG, INPUT);
	digitalWrite(m_DRDYPinMAG, LOW);
	pinMode(m_INTPin, INPUT);
	digitalWrite(m_INTPin, LOW);

}

/* ------------------------------------------------------------ */
/*	Nav::NavHostTerm()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Releases processor resources used by the library, defaults processor's pins
*/

void Nav::NavHostTerm()
{
	// Make the signal pins be inputs.
	digitalWrite(m_SSPinAG, HIGH);
	pinMode(m_SSPinAG, INPUT);
	digitalWrite(m_SSPinMAG, HIGH);
	pinMode(m_SSPinMAG, INPUT);
	digitalWrite(m_SSPinALT, HIGH);
	pinMode(m_SSPinALT, INPUT);
	if(m_DRDYPinMAG != -1)
	{
		digitalWrite(m_DRDYPinMAG, LOW);
		pinMode(m_DRDYPinMAG, INPUT);
	}
	if(m_INTPin != -1)
	{	
		digitalWrite(m_INTPin, LOW);
		pinMode(m_INTPin, INPUT);
	}	
	pdspi->end();

}
/* ------------------------------------------------------------ */
/*	Nav::NavDevInit()
**
**	Parameters:
**		
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Any initialization needed. 
** 		 
**		
*/
void Nav::NavDevInit()
{
 //Init();
}
/* ------------------------------------------------------------ */
/*	Nav::NavDevTerm()
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Shuts down the Nav hardware
*/
void Nav::NavDevTerm()
{
	//shuts down the accel and gyro instruments
	InitAG(0, MODE_INST_AG);
	//shuts down the magnetometer instrument
	InitMAG(0);
	//shuts down the altimeter instrument 
	InitALT(0);
}
/* ------------------------------------------------------------ */
/*    Nav::Init()
**
**   Parameters:
**		none				
**   Return Values:
**      void 
**
**   Errors:
**		none
**
**   Description:
**		This function initializes all the instruments by calling the specific init functions for each of them 
**		default operating mode for Accel and Gyro is Accl+Gyro						
**
*/
void Nav::Init()
{
//enable only acl work mode or gyro and acl work mode;
	InitAG(1, MODE_INST_AG);
	//init mag instrument
	InitMAG(1);
	//init alt instrument 
	InitALT(1);
}
/* ------------------------------------------------------------ */
/*	Nav::InitAG(bool fInit, uint8_t bModeSel)
**
**	Parameters:
**		fInit			- enables the instrument for initialization or disables it
**		bModeSel		- work mode for the two instruments, Accel and Gyro: Accel only or Gyro+Accel
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes the accelerometer only or both the accel and gyro instruments with the following settings:
**		 for Accel only: Enable all three axes in CTRL_REG5_XL register, set 10 Hz ODR in CTRL_REG6_XL register
**		 for Gyro and Accel: set 10Hz ODR in CTRL_REG1_G register and CTRL_REG6_XL, thus enabling the Gyro functionality as well;
**		 enable the output of all three axes
** 		 
**		
*/
void Nav::InitAG(bool fInit, uint8_t bModeSel)
{
	if (fInit)
	{
		if (bModeSel==MODE_INST_A)
		{
			//enable all three axes
			WriteSPI(m_SSPinAG, CTRL_REG5_XL,0x38);
			//set 10Hz odr for accelerometer
			WriteSPI(m_SSPinAG, CTRL_REG6_XL, 0x20);
		}
		else if(bModeSel==MODE_INST_AG)
		{
			//enable all three axes
			WriteSPI(m_SSPinAG, CTRL_REG5_XL,0x38);
			//set 10Hz odr for accel when used together with gyro
			WriteSPI(m_SSPinAG, CTRL_REG6_XL, 0x20);
			//set 10Hz rate for Gyro
			WriteSPI(m_SSPinAG, CTRL_REG1_G,0x20);
			//enable the axes outputs for Gyro
			WriteSPI(m_SSPinAG, CTRL_REG4,0x38);
		}
	}
	else 
	{
		if (bModeSel==MODE_INST_A)
		{
			//power down accel
			WriteSPI(m_SSPinAG, CTRL_REG5_XL, 0x00);
			WriteSPI(m_SSPinAG, CTRL_REG6_XL, 0x00);
		}
		else if(bModeSel==MODE_INST_AG)
		{
		//power down both the accel and gyro instruments
		WriteSPI(m_SSPinAG, CTRL_REG5_XL, 0x00);
		WriteSPI(m_SSPinAG, CTRL_REG6_XL, 0x00);
		WriteSPI(m_SSPinAG, CTRL_REG9, 0x00);
		
		WriteSPI(m_SSPinAG, CTRL_REG4, 0x00);
		WriteSPI(m_SSPinAG, CTRL_REG1_G, 0x00);
		}
	}
}

/* ------------------------------------------------------------ */
/*	Nav::InitMAG(bool fInit)
**
**	Parameters:
**		fInit			- enables the instrument for initialization or disables it
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes the magnetometer instrument with the following settings:
**		set medium performance mode and 10H ODR for MAG, in register CTRL_REG1_M
**		disable I2C and enable SPI read and write operations, set the operating mode to continuous in CTRL_REG3_M register
** 		 
**		
*/
void Nav::InitMAG(bool fInit)
{
	if (fInit)
	{
		//set medium performance mode for x and y and 10Hz ODR for MAG, 
		WriteSPI(m_SSPinMAG, CTRL_REG1_M, 0x30);
		//set scale to +-4Gauss
		WriteSPI(m_SSPinMAG, CTRL_REG2_M, 0);
		//disable I2C and enable SPI read and write operations, 
		//set the operating mode to continuous conversion
		WriteSPI(m_SSPinMAG, CTRL_REG3_M, 0x00);
		//set medium performance mode for z axis
		WriteSPI(m_SSPinMAG, CTRL_REG4_M, 0x04);
		//cntinuous update of output registers
		WriteSPI(m_SSPinMAG, CTRL_REG5_M, 0x00);
	}
	else 
	{
		//power down the instrument
		WriteSPI(m_SSPinMAG, CTRL_REG3_M,0x03);
	}
}

/* ------------------------------------------------------------ */
/*	InitALT(bool fInit)
**
**	Parameters:
**		fInit			- enables the instrument for initialization or disables it
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes the barometer/altimeter instrument with the following settings:
**		set active mode and 7Hz ODR rate, in register CTRL_REG1, block data update active
** 		 
**		
*/
void Nav::InitALT(bool fInit)
{
	uint8_t status;
	if (fInit)
	{
		//clean start
		WriteSPI(m_SSPinALT, CTRL_REG1, 0x00);
		delay(1);
		//set active the device and ODR to 7Hz
		WriteSPI(m_SSPinALT, CTRL_REG1, 0xA4);
		//increment address during multiple byte access disabled
		WriteSPI(m_SSPinALT, CTRL_REG2, 0x00);
		//no modification to interrupt sources
		WriteSPI(m_SSPinALT, CTRL_REG4, 0x00);
	}
	else 
	{
		//power down the instrument
		WriteSPI(m_SSPinALT, CTRL_REG1,0x00);
	}
}
/* ------------------------------------------------------------ */
/*  Nav::GetDeviceID()
**
**  Parameters:
**		none		
**  Return Value:
**      none
**		
**  Errors:
**		none	
**
**  Description:
**		The function gets the device id for all the instruments, updates the DEV_ID structure members
**
*/
void Nav::GetDeviceID()
{
	ReadRegister(m_SSPinAG, WHO_AM_I, 1, &idData.ag);
	ReadRegister(m_SSPinMAG, WHO_AM_I_M, 1, &idData.mag);
	ReadRegister(m_SSPinALT, WHO_AM_I, 1, &idData.alt);
}
/* ------------------------------------------------------------ */
/*   Nav::GetData()
**
**  Parameters:
**		none			
**
**  Return Values:
**      void 
**
**  Errors:
**		none
**
**  Description:
**		This function calls the read functions of all the instruments and updates the global variables with new data
**							
**
*/
void Nav::GetData()
{
	ReadAccelG(acclData.X, acclData.Y, acclData.Z);
	ReadGyroDps(gyroData.X,gyroData.Y, gyroData.Z);
	ReadMagGauss(magData.X,magData.Y, magData.Z);
	hPa = ReadPressurehPa();
	tempC = ReadTempC();	
}
/*-------------------------------------------------------------*/
/*	SPI Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*	Nav::WriteSPI(int8_t ssPin, uint8_t bAddr, uint8_t bVal)
**
**	Parameters:
**		ssPin			- instrument Chip/Slave Select: Accelerometer/Gyro, Magnetometer or Altimeter selection
**		bAddr			- address to write to via SPI
**		bVal			- byte to be written to SPI
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Sends one byte to SPI.
*/
void Nav::WriteSPI(int8_t ssPin, uint8_t bAddr, uint8_t bVal)
{
	if(pdspi != NULL)
	{
		// make SS active	
		digitalWrite(ssPin, LOW);
		//write first byte indicating the operation will be writing
		pdspi->transfer(bAddr|0x00);
		// write to SPI
		pdspi->transfer(bVal);
		// make SS inactive
		digitalWrite(ssPin, HIGH);
	}
}

/* ------------------------------------------------------------ */
/*	Nav::ReadSPI(int8_t ssPin, uint8_t bAddr)
**
**	Parameters:
**		ssPin			- instrument Chip Select: Accelerometer/Gyro, Magnetometer or Altimeter selection
**		bAddr			- address byte to be read from via SPI
**
**	Return Value:
**		uint8_t - Returns a byte read as a result of the SPI transfer
**
**	Errors:
**		none
**
**	Description:
**		Receives one byte to SPI.
*/
uint8_t Nav::ReadSPI(int8_t ssPin, uint8_t bAddr)
{
	uint8_t bVal;
	if(pdspi != NULL)
	{
		// make SS active	
		digitalWrite(ssPin, LOW);
		//send first byte indicating the operation will be reading
		pdspi->transfer(bAddr|0x80);
		// read from SPI
		bVal = pdspi->transfer(0);
		// make SS inactive
		digitalWrite(ssPin, HIGH);
	}
	return bVal;
}
/* ------------------------------------------------------------ */
/*	Nav::WriteRegister(int8_t ssPin, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData)
**
**	Parameters:
**		ssPin			- instrument Chip Select to be used: Accelerometer/Gyro, Magnetometer or Altimeter
**		bAddr			- register address to start writing bytes to
**		bCntBytes		- number of bytes to be written via SPI interface
**		pData			- pointer to the 16 bit data array to be written
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Sends multiple bytes to SPI.
*/
void Nav::WriteRegister(int8_t ssPin, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData)
{
	int ib;
	if(pdspi != NULL)
	{
		if(ssPin == m_SSPinAG)
		{
			// make SS active	
			digitalWrite(ssPin, LOW);
			//write first byte indicating the operation will be writing
			pdspi->transfer(bAddr|0x00);
			for(ib = 0; ib < bCntBytes; ib++)
			{
				// write to SPI
				pdspi->transfer(pData[ib]);
			}
			// make SS inactive
			digitalWrite(ssPin, HIGH);
		}
		else if((ssPin == m_SSPinMAG)||(ssPin == m_SSPinALT))
		{
			// make SS active	
			digitalWrite(ssPin, LOW);
			//write first byte indicating the operation will be writing
			pdspi->transfer(bAddr|0x40);
			for(ib = 0; ib < bCntBytes; ib++)
			{
				// write to SPI
				pdspi->transfer(pData[ib]);
			}
			// make SS inactive
			digitalWrite(ssPin, HIGH);
		}
	}
}

/* ------------------------------------------------------------ */
/*	Nav::ReadRegister(int8_t ssPin, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData)
**
**	Parameters:
**		ssPin			- instrument Chip Select to be used: Accelerometer/Gyro, Magnetometer or Altimeter
**		bAddr			- register address to start reading bytes from
**		bCntBytes		- number of bytes to be read 
**		pData			- pointer to the 16 bit data array to be read
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Reads multiple bytes from SPI.
*/
void Nav::ReadRegister(int8_t ssPin, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData)
{
	int ib;
	if(pdspi != NULL)
	{
		if(ssPin == m_SSPinAG)
		{
			// make SS active	
			digitalWrite(ssPin, LOW);
			// send first byte indicating the operation will be reading from SPI
			pdspi->transfer(bAddr|0x80);
			for(ib = 0; ib < bCntBytes; ib++)
			{
				// read from SPI the number of bytes given by bCntBytes
				pData[ib] = pdspi->transfer(0x00);
			}
			// make SS inactive
			digitalWrite(ssPin, HIGH);
		}
		else if((ssPin == m_SSPinMAG) || (ssPin == m_SSPinALT))
		{
			// make SS active	
			digitalWrite(ssPin, LOW);
			// send first byte indicating the operation will be reading from SPI
			pdspi->transfer(bAddr|0xC0);
			for(ib = 0; ib < bCntBytes; ib++)
			{
				// read from SPI the number of bytes given by bCntBytes
				pData[ib] = pdspi->transfer(0x00);
			}
			// make SS inactive
			digitalWrite(ssPin, HIGH);
		}
		
	}

}
/*-------------------------------------------------------------*/
/*	Accelerometer Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*   Nav::ReadAccel(int16_t &AclX, int16_t &AclY, int16_t &AclZ)
**
**  Parameters:
**		&AclX	- the output parameter that will receive acceleration on X axis - 16 bits value
**		&AclY	- the output parameter that will receive acceleration on Y axis - 16 bits value
**		&AclZ	- the output parameter that will receive acceleration on Z axis - 16 bits value
**
**  Return Values:
**      none 
**
**  Errors:
**		none	
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the accelerometer. 
**			-	It reads simultaneously the acceleration on three axes in a buffer of 6 bytes using the ReadRegister function
**			-	For each of the three axes, it combines the two bytes in order to get a 16-bit value
*/
void Nav::ReadAccel(int16_t &AclX, int16_t &AclY, int16_t &AclZ)
{
	uint8_t iAclX_L, iAclX_H, iAclY_L, iAclY_H, iAclZ_L, iAclZ_H;
	uint8_t rgwRegVals[6];
	//reads the bytes using the incremeting address functionality of the device.
	ReadRegister(m_SSPinAG, OUT_X_L_XL, 6, (uint8_t *)rgwRegVals);
	iAclX_L = rgwRegVals[0];
	iAclX_H = rgwRegVals[1];
	iAclY_L = rgwRegVals[2];
	iAclY_H = rgwRegVals[3];
	iAclZ_L = rgwRegVals[4];
	iAclZ_H = rgwRegVals[5];
	//combines the read values for each axis to obtain the 16-bit values
	AclX = ((int16_t)iAclX_H << 8) | iAclX_L;
	AclY = ((int16_t)iAclY_H << 8) | iAclY_L;
	AclZ = ((int16_t)iAclZ_H << 8) | iAclZ_L;
}
/* ------------------------------------------------------------ */
/*  Nav::ReadAccelG(float &AclXg, float &AclYg, float &AclZg) 
**
**  Parameters:
**		&AclXg	- the output parameter that will receive acceleration on X axis (in "g")
**		&AclYg	- the output parameter that will receive acceleration on Y axis (in "g")
**		&AclZg	- the output parameter that will receive acceleration on Z axis (in "g")
**
**  Return Values:
**      none 
**
**  Errors:
**		none
**  Description:
**		This function is the main function used for accelerometer values reading, providing the 3 current accelerometer values in “g”.
**		It returns the acceleration measured on the three axes in "g", using the raw values and the conversion function 
**		For each of the three values, converts the 16-bit value to the value expressed in “g”, considering the currently selected g range
*/
void Nav::ReadAccelG(float &AclXg, float &AclYg, float &AclZg) 
{
	int16_t AclX, AclY, AclZ;
	
	ReadAccel(AclX, AclY, AclZ);
	AclXg = ConvertReadingToValueG(AclX);
	AclYg = ConvertReadingToValueG(AclY);
	AclZg = ConvertReadingToValueG(AclZ);
}
/* ------------------------------------------------------------ */
/*   Nav::ConvertReadingToValueG(int16_t rawVal)
**
**  Parameters:
**		rawVal	- the 2 bytes containing the reading.
**
**  Return Values:
**      float - the value of the acceleration in "g" corresponding to the 16 bits reading and the current g range
**
**  Errors:
**		none
**  Description:
**		Converts the value from the 16 bits reading to the float value (in g) corresponding to the acceleration, considering the current selected g range.
*/
float Nav::ConvertReadingToValueG(int16_t rawVal)
{
	//Convert the accelerometer value to G's. 
	float dResult = ((float)rawVal) * m_GRangeLSB;
	return dResult;
}	
/* ------------------------------------------------------------ */
/*  Nav::GetXLRangeLSB(uint8_t bRangeXL)
**
**  Parameters:
**		bRangeXL	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_XL_2G	Parameter g range : +/- 2g
**					1	PAR_XL_4G	Parameter g range : +/- 4g
**					2	PAR_XL_8G	Parameter g range : +/- 8g
**					3	PAR_XL_16G 	Parameter g range : +/- 16g
**
**
**  Return Value:
**     float - the corresponding value of one LSB unit according to the range set
**		
**  Errors:
**	   none		
**
**  Description:
**		The function computes the range LSB based on the set range parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "g" value.
**		If value is outside this range, the default value is set.
*/
float Nav::GetXLRangeLSB(uint8_t bRangeXL)
{
	float gRangeLSB;
	switch(bRangeXL)
	{
		case PAR_XL_2G:
			gRangeLSB = 0.000061;
			break;
		case PAR_XL_4G:
			gRangeLSB = 0.000122;
			break;
		case PAR_XL_8G:
			gRangeLSB = 0.000244;
			break;
		case PAR_XL_16G:
			gRangeLSB = 0.000732;
			break;
		default:
			gRangeLSB = 0.000061;
			break;
	}
	return gRangeLSB;	
}
/* ------------------------------------------------------------ */
/*   Nav::SetRangeXL(uint8_t bRangeXL)
**
**  Parameters:
**		bRangeXL	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_XL_2G	Parameter g range : +/- 2g
**					1	PAR_XL_4G	Parameter g range : +/- 4g
**					2	PAR_XL_8G	Parameter g range : +/- 8g
**					3	PAR_XL_16G  Parameter g range : +/- 16g
**
**
**  Return Value:
**      none
**		
**  Errors:
**		none
**  Description:
**		The function sets the appropriate g range bits in the CTRL_REG6_XL register. 
**
*/
void Nav::SetRangeXL(uint8_t bRangeXL)
{
	m_GRangeLSB = GetXLRangeLSB(bRangeXL);
	SetBitsInRegister(m_SSPinAG, CTRL_REG6_XL, MSK_RANGE_XL, bRangeXL, 3);	
}
/* ------------------------------------------------------------ */
/*   Nav::GetRangeXL()
**
**  Parameters:
**		none
**
**  Return Value:
**      uint8_t - returns the previously selected range from CTRL_REG6_XL register
**		
**  Errors:
**		none		
**  Description:
**		The function reads the g range bits in the CTRL_REG6_XL register and computes the range to be provided to the user. 
**		The accepted argument values are between 0 and 3. If the value is not a valid one, the default value for range is set
**
*/
uint8_t Nav::GetRangeXL()
{
	uint8_t readRange, gRange;
	readRange = GetBitsInRegister(m_SSPinAG, CTRL_REG6_XL, 3, 2);
	switch(readRange)
	{
		case PAR_XL_2G:
			gRange = 2;
			break;
		case PAR_XL_4G:
			gRange = 4;
			break;
		case PAR_XL_8G:
			gRange = 8;
			break;
		case PAR_XL_16G:
			gRange = 16;
			break;
		default: 
			gRange = 2;
			break;
	}
	return gRange;
}
/* ------------------------------------------------------------ */
/*   Nav::DataAvailableXL()
**
**  Parameters:
**		none
**
**  Return Value:
**      uint8_t - returns the data available status in STATUS_REG register, for accel
**		
**  Errors:
**		none		
**
**  Description:
**		The function reads the STATUS_REG register and returns the data available bit in it
**
*/
uint8_t Nav::DataAvailableXL()
{
	uint8_t status;
	ReadRegister(m_SSPinAG, STATUS_REG, 1, &status);
	return (status & (1<<0));
}
/* ------------------------------------------------------------ */
/*   Nav::ConfigIntXL(uint8_t bIntGen, bool aoi, bool latch)
**
**  Parameters:
**		bIntGen	- The parameter indicating the interrupt generator. Can be one of the parameters from the following list
**			MSK_XLIE_XL 				1<<0
**			MSK_XHIE_XL 				1<<1
**			MSK_YLIE_XL 				1<<2
**			MSK_YHIE_XL 				1<<3
**			MSK_ZLIE_XL 				1<<4
**			MSK_ZHIE_XL 				1<<5
**			MSK_GEN_6D 					1<<6
**		aoi - parameter indicating whether the interrupt generators are or-ed or and-ed together
**		latch - parameter that sets or not the latch interrupt request
**  Return Value:
**      none 
**		
**  Errors:
**				
**  Description:
**		The function configures the interrupt register INT_GEN_CFG_XL setting the interrupt generator. 
**		Sets the interrupt events to or-ed or and-ed. Sets the interrupt event to be latched or not.
*/
void Nav::ConfigIntXL(uint8_t bIntGen, bool aoi, bool latch)
{
	uint8_t temp = bIntGen;
	//interrupt events are or-ed or and-ed
	if (aoi) 
	{
		temp |= 0x80;
	}
	WriteSPI(m_SSPinAG, INT_GEN_CFG_XL, temp);
	temp = 0;
	
	temp = ReadSPI(m_SSPinAG, CTRL_REG4);
	//latched interrupt enable is set in CTRL_REG4 register
	if (latch) 
	{
		temp |= 0x02;
	}
	WriteSPI(m_SSPinAG, CTRL_REG4, temp);
}
/* ------------------------------------------------------------ */
/*  Nav::SetIntThresholdXL(float thValX, float thValY, float thValZ, uint8_t intDuration, bool wait)
**
**  Parameters:
**		thValX, thValY, thValZ - Parameters containing the threshold value on each axis
**		intDuration - parameter indicating the duration of the enter/exit interrupt
**		wait - parameter enabling or disabling the wait time before exiting the interrupt routine.
**  Return Value:
**      none 
**		
**  Errors:
**		none		
**  Description:
**		The function sets the interrupt threshold for each axis and also the duration of the enter/exit interrupt. Enables or disables the
**		wait on duration before exiting interrupt.
**
*/
void Nav::SetIntThresholdXL(float thValX, float thValY, float thValZ, uint8_t intDuration, bool wait)
{
	uint8_t bthValX, bthValY, bthValZ;
	//converts the float value in g to raw accel data to be written in INT_GEN_THS_X/Y/Z_XL registers
	bthValX = (uint8_t)(thValX/m_GRangeLSB);
	bthValY = (uint8_t)(thValY/m_GRangeLSB);
	bthValZ = (uint8_t)(thValZ/m_GRangeLSB);
	
	WriteSPI(m_SSPinAG, INT_GEN_THS_X_XL, bthValX);
	WriteSPI(m_SSPinAG, INT_GEN_THS_Y_XL, bthValY);
	WriteSPI(m_SSPinAG, INT_GEN_THS_Z_XL, bthValZ);
	
	// Write duration and wait to INT_GEN_DUR_XL register
	uint8_t temp;
	temp = (intDuration & 0x7F);
	if (wait) temp |= 0x80;
	WriteSPI(m_SSPinAG, INT_GEN_DUR_XL, temp);
}
/*-------------------------------------------------------------*/
/*	Gyroscope Specific Functions
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*  Nav::ReadGyro(int16_t &GX, int16_t &GY, int16_t &GZ)
**
**  Parameters:
**		&GX	- the output parameter that will receive gyro value on X axis - 16 bits value
**		&GY	- the output parameter that will receive gyro value on Y axis - 16 bits value
**		&GZ	- the output parameter that will receive gyro value on Z axis - 16 bits value
**
**  Return Values:
**      none 
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the gyro. 
**			-	It reads simultaneously the gyro value on three axes in a buffer of 6 bytes using the ReadRegister function
**			-	For each of the three axes, combines the two bytes in order to get a 16-bit value
*/
void Nav::ReadGyro(int16_t &GX, int16_t &GY, int16_t &GZ)
{
	uint8_t iGX_L, iGX_H, iGY_L, iGY_H, iGZ_L, iGZ_H;
	uint8_t rgwRegVals[6];
	//reads the bytes using the incremeting address functionality of the device.
	ReadRegister(m_SSPinAG, OUT_X_L_G, 6, (uint8_t *)rgwRegVals);
	iGX_L = rgwRegVals[0];
	iGX_H = rgwRegVals[1];
	iGY_L = rgwRegVals[2];
	iGY_H = rgwRegVals[3];
	iGZ_L = rgwRegVals[4];
	iGZ_H = rgwRegVals[5];
	//combines the read values for each axis to obtain the 16-bits values
	GX = ((int16_t)iGX_H << 8) | iGX_L;
	GY = ((int16_t)iGY_H << 8) | iGY_L;
	GZ = ((int16_t)iGZ_H << 8) | iGZ_L;
}

/* ------------------------------------------------------------ */
/*  Nav::ReadGyroDps(float &GXdps, float &GYdps, float &GZdps) 
**
**  Parameters:
**		&GXdps	- the output parameter that will receive gyro values on X axis (in "dps")
**		&GYdps	- the output parameter that will receive gyro values on Y axis (in "dps")
**		&GZdps	- the output parameter that will receive gyro values on Z axis (in "dps")
**
**  Return Values:
**      none 
**
**  Errors:
**		none
**  Description:
**		This function is the main function used for gyro values reading, providing the 3 current gyro values in “dps”. 
**		It returns the gyro values measured on the three axes in "dps", using the raw values and the conversion function 
**		For each of the three values, converts the 16-bit value to the value expressed in “dps”, considering the currently selected dps range
*/
void Nav::ReadGyroDps(float &GXdps, float &GYdps, float &GZdps) 
{
	int16_t GX, GY, GZ;
	
	ReadGyro(GX, GY, GZ);
	GXdps = ConvertReadingToValueDPS(GX);
	GYdps = ConvertReadingToValueDPS(GY);
	GZdps = ConvertReadingToValueDPS(GZ);
}

/* ------------------------------------------------------------ */
/*  Nav::ConvertReadingToValueDPS(int16_t rawVal)
**
**  Parameters:
**		rawVal	- the 2 bytes containing the reading.
**
**  Return Values:
**      float - the value of the gyro in "dps" corresponding to the 16 bits reading and the current dps range
**
**  Errors:
**		none
**  Description:
**		Converts the value from the 16 bits reading to the float value (in dps) corresponding to the degrees value, considering the current selected dps range.
**	
*/
float Nav::ConvertReadingToValueDPS(int16_t rawVal)
{
	//Convert the gyro value to dps. 
  float dResult = ((float)rawVal) * m_DPSRangeLSB;
  return dResult;
}

/* ------------------------------------------------------------ */
/*  Nav::GetGRangeLSB
**
**   Parameters:
**		bRangeG	- the parameter specifying the dps range. Can be one of the parameters from the following list:
**					0	PAR_G_245DPS	Parameter dps range : +/- 245dps
**					1	PAR_G_500DPS	Parameter dps range : +/- 500dps
**					3	PAR_G_2kDPS		Parameter dps range : +/- 2kdps
**					
**   Return Value:
**      float - corresponding value of one LSB unit according to the range set
**		
**   Errors:
**		none
**   Description:
**		The function computes the range LSB based on the set range parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "dps" value.
**		If value is outside this range, the default value is set.
**
*/
float Nav::GetGRangeLSB(uint8_t bRangeG)
{
	float gRangeLSB;
	switch(bRangeG)
	{
		case PAR_G_245DPS:
			gRangeLSB = 0.00875;
			break;
		case PAR_G_500DPS:
			gRangeLSB = 0.0175;
			break;
		case PAR_G_2kDPS:
			gRangeLSB = 0.07;
			break;
		default:
			gRangeLSB = 0.00875;
			break;
	}
	return gRangeLSB;	
}

/* ------------------------------------------------------------ */
/*  Nav::SetRangeG(uint8_t bRangeG)
**
**  Parameters:
**		bGRangeG	- the parameter specifying the dps range. Can be one of the parameters from the following list:
**					0	PAR_G_245DPS	Parameter dps range : +/- 245dps
**					1	PAR_G_500DPS	Parameter dps range : +/- 500dps
**					3	PAR_G_2000DPS	Parameter dps range : +/- 2000dps
**
**
**  Return Value:
**      none
**		
**  Errors:
**		none		
**  Description:
**		The function sets the appropriate dps range bits in the CTRL_REG1_G register. 
**
*/
void Nav::SetRangeG(uint8_t bRangeG)
{
	m_DPSRangeLSB = GetGRangeLSB(bRangeG);
	SetBitsInRegister(m_SSPinAG, CTRL_REG1_G, MSK_RANGE_G, bRangeG, 3);	
}

/* ------------------------------------------------------------ */
/*   Nav::GetRangeG()
**
**  Parameters:
**			
**  Return Value:
**      float - returns the previously selected range from CTRL_REG1_G register
**		
**  Errors:
**		none		
**  Description:
**		The function reads the g range bits in the CTRL_REG1_G register and computes the range to be provided to the user. 
**		The accepted argument values are between 0 and 3.If the value is not a valid one, the default value for range is set
*/
float Nav::GetRangeG()
{
	uint16_t readRange, gRange;
	readRange = GetBitsInRegister(m_SSPinAG, CTRL_REG1_G, 3, 2);
	switch(readRange)
	{
		case PAR_G_245DPS:
			gRange = 245;
			break;
		case PAR_G_500DPS:
			gRange = 500;
			break;
		case PAR_G_2kDPS:
			gRange = 2000;
			break;
		default: 
			gRange = 245;
			break;
	}
	return gRange;
}

/* ------------------------------------------------------------ */
/*  Nav::DataAvailableG()
**
**  Parameters:
**		none	
**  Return Value:
**      uint8_t - returns the data available status in STATUS_REG register, for gyro
**		
**  Errors:
**		none		
**  Description:
**		The function reads the STATUS_REG register and returns the data available bit in it
**
*/
uint8_t Nav::DataAvailableG()
{
	uint8_t status;
	ReadRegister(m_SSPinAG, STATUS_REG, 1, &status);
	return ((status & (1<<1))>> 1);
}

/* ------------------------------------------------------------ */
/*   Nav::ConfigIntG(uint8_t bIntGen, bool aoi, bool latch)
**
**   Parameters:
**		bIntGen	- The parameter indicating the interrupt generator. Can be one of the parameters from the following list
**			MSK_XLIE_G	 				1<<0
**			MSK_XHIE_G					1<<1
**			MSK_YLIE_G					1<<2
**			MSK_YHIE_G					1<<3
**			MSK_ZLIE_G					1<<4
**			MSK_ZHIE_G					1<<5
**		aoi - parameter indicating whether the interrupt generators are or-ed or and-ed together
**		latch - parameter that sets or not the latch interrupt request
**   Return Value:
**      none 
**		
**   Errors:
**		none		
**
**   Description:
**		The function sets the interrupt threshold for each axis and also the duration of the enter/exit interrupt. Enables or dissables the
**		wait on duration before exiting interrupt.
**
*/
void Nav::ConfigIntG(uint8_t bIntGen, bool aoi, bool latch)
{
	uint8_t temp = bIntGen;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	WriteSPI(m_SSPinAG, INT_GEN_CFG_G, temp);
}
/* ------------------------------------------------------------ */
/*  Nav::SetIntThresholdG(float thVal, uint16_t axis, bool drCntMode, uint8_t intDuration, bool wait)
**
**  Parameters:
**		thVal - Parameters containing the threshold value for one of the axes or for all
**		axis - parameter indicating the axis for which the threshold is set. It can be one of the following values:
**				X_AXIS			0
**				Y_AXIS			1
**				Z_AXIS			2
**		drCntMode - counter mode for interrupt
**		intDuration - parameter indicating the duration of the enter/exit interrupt
**		wait - parameter enabling or disabling the wait time before exiting the interrupt routine.
**  Return Value:
**      none
**		
**  Errors:
**		none
**  Description:
**		The function sets the interrupt threshold for the selected or all axes, the counter mode for interrupt
**		and also the duration of the enter/exit interrupt. Enables or disables the wait on duration 
**		before exiting interrupt.
**
*/
void Nav::SetIntThresholdG(float thVal, uint16_t axis, bool drCntMode, uint8_t intDuration, bool wait)
{
	uint8_t buffer[2];
	uint16_t bthVal;
	
	//convert the float value in g to raw accel data to be written in INT_GEN_THS_XH/XL/YH/YL/ZH/ZL_G registers
	bthVal = (uint16_t)(thVal/m_DPSRangeLSB);
	//split bytes
	buffer[0] = (bthVal & 0x7F00) >> 8;
	buffer[1] = (bthVal & 0x00FF);
	switch(axis)
	{
		case X_AXIS:
		//set the first bit, decrement or reset counter mode for interrupt
			buffer[0] |= drCntMode<<8;
			WriteSPI(m_SSPinAG, INT_GEN_THS_XH_G + (axis*2), buffer[0]);
			WriteSPI(m_SSPinAG, INT_GEN_THS_XH_G + 1 + (axis*2), buffer[1]);
			break;
		case Y_AXIS:
			WriteSPI(m_SSPinAG, INT_GEN_THS_XH_G + (axis*2), buffer[0]);
			WriteSPI(m_SSPinAG, INT_GEN_THS_XH_G + 1 + (axis*2), buffer[1]);
			break;
		case Z_AXIS:
			WriteSPI(m_SSPinAG, INT_GEN_THS_XH_G + (axis*2), buffer[0]);
			WriteSPI(m_SSPinAG, INT_GEN_THS_XH_G + 1 + (axis*2), buffer[1]);	
			break;
		default:
			break;
	}
	// Write duration and wait to INT_GEN_DUR_G
	uint8_t temp;
	temp = (intDuration & 0x7F);
	if (wait) temp |= 0x80;
	WriteSPI(m_SSPinAG, INT_GEN_DUR_G, temp);
	
}
/*-------------------------------------------------------------*/
/*	XL+G Common Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*  Nav::GetIntSrcXLG(uint8_t bInstMode)
**
**  Parameters:
**		bInstMode - parameter selecting between the two instruments, Accel and Gyro:
**			MODE_INST_A 	0 - accelerometer mode
**			MODE_INST_AG 	1 - Gyro 
**  Return Value:
**      uint8_t - returns the content of the INT_GEN_SRC_XL or INT_GEN_SRC_G, depending of bInstMode parameter
**		
**  Errors:
**		none		
**
**  Description:
**		The function returns the source of interrupt for either accel or gyro instruments.
**
*/
uint8_t Nav::GetIntSrcXLG(uint8_t bInstMode)
{
	uint8_t intSrc;
	if (bInstMode==MODE_INST_A)
	{
		intSrc = ReadSPI(m_SSPinAG, INT_GEN_SRC_XL);
		// Check if the IA_XL (interrupt active) bit is set
		if (intSrc & (1<<6))
		{
			return intSrc;
		}
		intSrc &=0x3F;
	}
	
	else if (bInstMode==MODE_INST_AG)
	{
		intSrc = ReadSPI(m_SSPinAG, INT_GEN_SRC_G);
		// Check if the IA_G (interrupt active) bit is set
		if (intSrc & (1<<6))
		{
			return intSrc; 
		}
		intSrc &=0x3F;
	}	
}
/* ------------------------------------------------------------ */
/*  Nav::ConfigInt(uint8_t bIntPin, uint8_t bIntGenMask, uint8_t bActiveType, uint8_t bOutputType)
**
**  Parameters:
**		bIntPin	- The parameter indicating the INT pin number, INT1 or INT2. Can be one of the parameters from the following list
**					0		INT_PIN_1
**					1		INT_PIN_2
**
**		bIntGenMask	- the events that trigger the interrupt. Can be one or more (OR-ed) parameters from the following list
**					MSK_INT1_IG_G				1<<7
**					MSK_INT_IG_XL				1<<6
**					MSK_INT_FSS5				1<<5
**					MSK_INT_OVR					1<<4
**					MSK_INT_FTH					1<<3
**					MSK_INT_Boot				1<<2
**					MSK_INT_DRDY_G				1<<1
**					MSK_INT_DRDY_XL				1<<0	Overrun
**
**					MSK_INT2_INACT				1<<7
**					MSK_INT2_FSS5				1<<5
**					MSK_INT2_OVR				1<<4
**					MSK_INT2_FTH				1<<3
**					MSK_INT2_DRDY_TEMP			1<<2
**					MSK_INT2_DRDY_G				1<<1
**					MSK_INT2_DRDY_XL			1<<0
**
**		bActiveType	– The parameter indicating the interrupt pin is active high or low. Can be one of the parameters from the following list
**					0		PAR_INT_ACTIVEHIGH
**					1		PAR_INT_ACTIVELOW
**		bOutputType	– The parameter indicating the interrupt pin is set pushpull or opendrain. Can be one of the parameters from the following list
**					0		PAR_INT_PUSHPULL
**					1		PAR_INT_OPENDRAIN
**  Return Value:
**      none 
**		
**  Errors:
**		none		
**
**  Description:
**		The function sets the interrupt generator on one of the INT1 or INT2 pins. Also sets the active type of the interrupt and the output type
**
*/
void Nav::ConfigInt(uint8_t bIntPin, uint8_t bIntGenMask,
	uint8_t bActiveType, uint8_t bOutputType)
{
	//set interrupt pin generator
	if (bIntPin == INT_PIN_1)
	{
		SetRegisterBits(m_SSPinAG, INT1_CTRL, bIntGenMask, 1);	
	
	}
	else if (bIntPin == INT_PIN_2)
	{
		SetRegisterBits(m_SSPinAG, INT2_CTRL, bIntGenMask, 1);	
	}
	// Configure CTRL_REG8
	uint8_t temp;
	temp = ReadSPI(m_SSPinAG, CTRL_REG8);
	//set interrupt active low or high
	if (bActiveType ==PAR_INT_ACTIVELOW) 
	{
		temp |= (1<<5);
	}
	else 
	{
		temp &= ~(1<<5);
	}
	//set output type to pushpull or opendrain
	if (bOutputType==PAR_INT_PUSHPULL) 
	{
		temp &= ~(1<<4);
	}
	else 
	{
		temp |= (1<<4);
	}
	WriteSPI(m_SSPinAG, CTRL_REG8, temp);
}
/* ------------------------------------------------------------ */
/*  Nav::ConfigIntForExternalIntUse(uint8_t bIntPin, uint8_t bParExtIntNo, uint8_t bIntGenMask,
**		void (*pfIntHandler)(), uint8_t bActiveType, uint8_t bOutputType)
**
**  Parameters:
**		bIntPin	- The parameter indicating the INT pin number, INT1 or INT2. Can be one of the parameters from the following list
**					0		INT_PIN_1
**					1		INT_PIN_2
**
**		bParExtIntNo - The parameter indicating the external interrupt number. Can be one of the parameters from the following list
**					0		PAR_EXT_INT0
**					1		PAR_EXT_INT1
**					2		PAR_EXT_INT2
**					3		PAR_EXT_INT3
**					4		PAR_EXT_INT4
**
**		bIntGenMask	- the events that trigger the interrupt. Can be one or more (OR-ed) parameters from the following list
**					MSK_INT1_IG_G				1<<7
**					MSK_INT_IG_XL				1<<6
**					MSK_INT_FSS5				1<<5
**					MSK_INT_OVR					1<<4
**					MSK_INT_FTH					1<<3
**					MSK_INT_Boot				1<<2
**					MSK_INT_DRDY_G				1<<1
**					MSK_INT_DRDY_XL				1<<0	Overrun
**
**					MSK_INT2_INACT				1<<7
**					MSK_INT2_FSS5				1<<5
**					MSK_INT2_OVR				1<<4
**					MSK_INT2_FTH				1<<3
**					MSK_INT2_DRDY_TEMP			1<<2
**					MSK_INT2_DRDY_G				1<<1
**					MSK_INT2_DRDY_XL			1<<0
**
**		void (*pfIntHandler)() - pointer to a function that will serve as interrupt handler.
**
**		bActiveType	– The parameter indicating the interrupt pin is active high or low. Can be one of the parameters from the following list
**					0		PAR_INT_ACTIVEHIGH
**					1		PAR_INT_ACTIVELOW
**		bOutputType	– The parameter indicating the interrupt pin is active high or low. Can be one of the parameters from the following list
**					0		PAR_INT_PUSHPULL
**					1		PAR_INT_OPENDRAIN
**  Return Value:
**      none 
**		
**  Errors:
**		none		
**
**  Description:
**		The function configures the interrupt for each of the INT pins of the XL/Gyro instruments. It sets the interrupt generator, 
**		the type  active high or low and the output type pushpull or open drain. 
**		It also attaches the interrupt handler to one of the microcontroller external interrupt pins.
**
*/
void Nav::ConfigIntForExternalIntUse(uint8_t bIntPin, uint8_t bParExtIntNo, uint8_t bIntGenMask,
		void (*pfIntHandler)(), uint8_t bActiveType, uint8_t bOutputType)
{
	if (bIntPin == INT_PIN_1)
	{
		//attach external interrupt to the handler function, set the interrupt generator for INT1 pin
		attachInterrupt(bParExtIntNo, pfIntHandler, RISING);
		//set the interrupt generator in the INT1_CTRL register
		SetRegisterBits(m_SSPinAG, INT1_CTRL, bIntGenMask, 1);	
	
	}
	else if (bIntPin == INT_PIN_2)
	{
		//attach external interrupt to the handler function, set the interrupt generator for INT2 pin
		attachInterrupt(bParExtIntNo, pfIntHandler, RISING);
		//set the interrupt generator in the INT2_CTRL register
		SetRegisterBits(m_SSPinAG, INT2_CTRL, bIntGenMask, 1);	
	}
	// Configure CTRL_REG8
	uint8_t temp;
	temp = ReadSPI(m_SSPinAG, CTRL_REG8);
	 //set interrupt active low or high
	if (bActiveType ==PAR_INT_ACTIVELOW) 
	{
		temp |= (1<<5);
	}
	else 
	{
		temp &= ~(1<<5);
	}
	//set output type to pushpull or opendrain
	if (bOutputType==PAR_INT_PUSHPULL) 
	{
		temp &= ~(1<<4);
	}
	else 
	{
		temp |= (1<<4);
	}
	WriteSPI(m_SSPinAG, CTRL_REG8, temp);	
}

/*-------------------------------------------------------------*/
/*	Magnetometer Specific Functions
/* ------------------------------------------------------------ */
/*   Nav::ReadMag(int16_t &MagX, int16_t &MagY, int16_t &MagZ)
**
**  Parameters:
**		&MagX	- the output parameter that will receive magnetometer value on X axis - 16 bits value
**		&MagY	- the output parameter that will receive magnetometer value on Y axis - 16 bits value
**		&MagZ	- the output parameter that will receive magnetometer value on Z axis - 16 bits value
**
**  Return Values:
**      none 
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the magnetometer. 
**			-	It reads simultaneously the magnetic field value on three axes in a buffer of 6 bytes using the ReadRegister function
**			-	For each of the three axes, combines the two bytes in order to get a 16-bit value
**
*/
void Nav::ReadMag(int16_t &MagX, int16_t &MagY, int16_t &MagZ)
{
	uint8_t iMagX_L, iMagX_H, iMagY_L, iMagY_H, iMagZ_L, iMagZ_H;
	uint8_t status;
	uint8_t rgwRegVals[6];
	do
	{
		ReadRegister(m_SSPinMAG, STATUS_REG_M, 1, &status);
	}
	while(status&0x08==0);
	//reads the bytes using the incremeting address functionality of the device.
	ReadRegister(m_SSPinMAG, OUT_X_L_M, 6, (uint8_t *)rgwRegVals);
	iMagX_L = rgwRegVals[0];
	iMagX_H = rgwRegVals[1];
	iMagY_L = rgwRegVals[2];
	iMagY_H = rgwRegVals[3];
	iMagZ_L = rgwRegVals[4];
	iMagZ_H = rgwRegVals[5];
	//combines the read values for each axis to obtain the 16-bits values
	MagX = ((int16_t)iMagX_H << 8) | iMagX_L;
	MagY = ((int16_t)iMagY_H << 8) | iMagY_L;
	MagZ = ((int16_t)iMagZ_H << 8) | iMagZ_L;	
}
/* ------------------------------------------------------------ */
/*  Nav::ReadMagGauss(float &MagXGauss, float &MagYGauss, float &MagZGauss) 
**
**  Parameters:
**		&MagXGauss	- the output parameter that will receive magnetic value on X axis (in "Gauss")
**		&MagYGauss	- the output parameter that will receive magnetic value on Y axis (in "Gauss")
**		&MagZGauss	- the output parameter that will receive magnetic value on Z axis (in "Gauss")
**
**  Return Values:
**      none 
**
**  Errors:
**		none
**  Description: 
**		This function is the main function used for magnetic field values reading, providing the 3 current magnetometer values in “Gauss”. 
**		It returns the gyro values measured on the three axes in "Gauss", using the raw values and the conversion function 
**		For each of the three values, converts the 16-bit value to the value expressed in “Gauss”, considering the currently selected Gauss range
*/
void Nav::ReadMagGauss(float &MagXGauss, float &MagYGauss, float &MagZGauss) 
{
	int16_t MagX, MagY, MagZ;
	
	ReadMag(MagX, MagY, MagZ);
	MagXGauss = ConvertReadingToValueGauss(MagX);
	MagYGauss = ConvertReadingToValueGauss(MagY);
	MagZGauss = ConvertReadingToValueGauss(MagZ);
}
/* ------------------------------------------------------------ */
/*  Nav::ConvertReadingToValueGauss(int16_t rawVal)
**
**  Parameters:
**		rawVal	- the 2 bytes containing the raw reading.
**
**  Return Values:
**      float - the value of the magnetic field in "gauss" corresponding to the 16 bits reading and the currently selected range
**
**  Errors:
**		none
**  Description:
**		Converts the value from the 16 bits reading to the float value (in gauss) corresponding to the magnetic field value, considering the current selected gauss range.
**
*/
float Nav::ConvertReadingToValueGauss(int16_t rawVal)
{
  float dResult = ((float)rawVal )* m_GaussRangeLSB;
  return dResult;
}

/* ------------------------------------------------------------ */
/*  Nav::GetMAGRangeLSB(uint8_t bRangeMAG)
**
**  Parameters:
**		uint8_t bRangeMAG	- the parameter specifying the gauss range. Can be one of the parameters from the following list:
**					0	PAR_MAG_4GAUSS	Parameter gauss range : +/- 4gauss
**					1	PAR_MAG_8GAUSS	Parameter gauss range : +/- 8gauss
**					2	PAR_MAG_12GAUSS	Parameter gauss range : +/- 12gauss
**					3	PAR_MAG_16GAUSS Parameter gauss range : +/- 16gauss
**
**  Return Value:
**      float  - returns the LSB unit value specific for each available range
**		
**  Errors:
**		none		
**  Description:
**		The function computes the range LSB based on the bRangeMAG parameter. The accepted argument values are between 0 and 3.
**		If the argument is within the accepted values range, it selects the range LSB value for further computations of the "gauss" value.
**		If value is outside this range, the default value is set.
**
*/
float Nav::GetMAGRangeLSB(uint8_t bRangeMAG)
{
	float gRangeLSB;
	switch(bRangeMAG)
	{
		case PAR_MAG_4GAUSS:
			gRangeLSB = 0.00014;
			break;
		case PAR_MAG_8GAUSS:
			gRangeLSB = 0.00029;
			break;
		case PAR_MAG_12GAUSS:
			gRangeLSB = 0.00043;
			break;
		case PAR_MAG_16GAUSS:
			gRangeLSB = 0.00058;
			break;
		default:
			gRangeLSB = 0.00014;
			break;
	}
	return gRangeLSB;	
}
/* ------------------------------------------------------------ */
/*   Nav::SetRangeMAG(uint8_t bRangeMAG)
**
**  Parameters:
**		bRangeMAG	- the parameter specifying the g range. Can be one of the parameters from the following list:
**					0	PAR_MAG_4GAUSS	Parameter g range : +/- 4g
**					1	PAR_MAG_8GAUSS	Parameter g range : +/- 8g
**					2	PAR_MAG_12GAUSS	Parameter g range : +/- 12g
**					3	PAR_MAG_16GAUSS Parameter g range : +/- 16g
**
**  Return Value:              
**		none
**
**  Errors:
**		none		
**
**  Description:
**		The function sets the appropriate gauss range bits in the CTRL_REG2_M register.
**
*/
void Nav::SetRangeMAG(uint8_t bRangeMAG)
{
	m_GaussRangeLSB = GetMAGRangeLSB(bRangeMAG);
	SetBitsInRegister(m_SSPinMAG, CTRL_REG2_M, MSK_RANGE_MAG, bRangeMAG, 5);	
}
/* ------------------------------------------------------------ */
/*  Nav::GetRangeMAG()
**
**  Parameters:
**		none
**  Return Value:
**      uint8_t - returns the previously set range value
**		
**  Errors:
**		none
**
**  Description:
**		The function reads the gauss range bits in the CTRL_REG2_M register and computes the range to be provided to the user. 
**		The accepted argument values are between 0 and 3. If value is outside this range, the default value is set
**
*/
uint8_t Nav::GetRangeMAG()
{
	uint8_t readRange, gRange;
	readRange = GetBitsInRegister(m_SSPinMAG, CTRL_REG2_M, 5, 2);
	switch(readRange)
	{
		case PAR_MAG_4GAUSS:
			gRange = 4;
			break;
		case PAR_MAG_8GAUSS:
			gRange = 8;
			break;
		case PAR_MAG_12GAUSS:
			gRange = 12;
			break;
		case PAR_MAG_16GAUSS:
			gRange = 16;
			break;
		default:
			gRange = 4;
			break;
	}
	return gRange;
}
/* ------------------------------------------------------------ */
/*  Nav::DataAvailableMAG(uint8_t axis)
**
**  Parameters:
**		axis - parameter indicating the axis for which the data availability is checked. It can be one of the values:
**			X_AXIS			0
**			Y_AXIS			1
**			Z_AXIS			2	
**
**  Return Value:
**       uint8_t - returns the data available status in STATUS_REG_M register, for the selected axis
**		
**  Errors:
**		none		
**
**  Description:
**		The function reads the STATUS_REG_M register and returns the data available bit in it, for each of the axes
**
*/
uint8_t Nav::DataAvailableMAG(uint8_t axis)
{
	uint8_t status;
	ReadRegister(m_SSPinMAG, STATUS_REG_M, 1, &status);
	return ((status & (1<<axis)) >> axis);
}
/* ------------------------------------------------------------ */
/*  Nav::ConfigIntMAG(uint8_t bIntGen, uint8_t bActiveType, bool latch)
**
** 	Parameters:
**		bIntGen - interrupt generator sources. It can be one of the following:
**				MSK_ZIEN_MAG				1<<5
**				MSK_YIEN_MAG				1<<6
**				MSK_XIEN_MAG 				1<<7
**		bActiveType - interrupt active low or high parameter:
**				PAR_INT_ACTIVEHIGH		0
**				PAR_INT_ACTIVELOW		1	
**		latch - parameter indicating the interrupt event is latched or not
**  Return Value:
**      none 
**		
**  Errors:
**		none		
**	Description:
**		The function configures the interrupts for magnetometer instruments. It sets the interrupt generator for the three axes
**		It sets the active level, low or high for the interrupt event. It enables/disables the interrupt latching
**
*/
void Nav::ConfigIntMAG(uint8_t bIntGen, uint8_t bActiveType, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (bIntGen & 0xE0);	
	// IEA bit is 0 for active-low, 1 for active-high.
	if (bActiveType == PAR_INT_ACTIVEHIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) bIntGen |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (bIntGen != 0) config |= (1<<0);
	
	WriteSPI(m_SSPinMAG, INT_CFG_M, config);
}
/* ------------------------------------------------------------ */
/*  Nav::SetIntThresholdM(float thVal)
**
**  Parameters:
**		thVal - the threshold value set to all axes
**  Return Value:
**      none 
**		
**   Errors:
**		none	
**
**   Description:
**		The function sets the interrupt threshold for the magnetometer instrument
*/
void Nav::SetIntThresholdM(float thVal)
{
	uint8_t buffer[2];
	uint16_t bthVal;
	//converts the float value in gauss to raw magnetic field data to be written in INT_THS_L_M/INT_THS_H_M registers
	bthVal = (uint16_t)(thVal/m_GaussRangeLSB);
	//split bytes
	//make sure the first bit of the High byte is 0, for correct functionality of the device
	buffer[0] = (bthVal & 0x7F00) >> 8;
	buffer[1] = (bthVal & 0x00FF);
	WriteSPI(m_SSPinMAG, INT_THS_H_M, buffer[0]);
	WriteSPI(m_SSPinMAG, INT_THS_L_M, buffer[1]);
}

/* ------------------------------------------------------------ */
/*   Nav::GetIntSrcMAG()
**
**  Parameters:
**		none
**  Return Value:
**      uint8_t - returns the interrupt sources for magnetometer instrument, after reading INT_SRC_M register
**		
**  Errors:
**		none		
**  Description:
**		The function returns the source of interrupt for magnetometer instrument.
**
*/
uint8_t Nav::GetIntSrcMAG()
{
	uint8_t intSrc;
	intSrc = ReadSPI(m_SSPinMAG, INT_SRC_M);
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc);// & 0xFE);
	}
	intSrc&=0xFE;
}
/* ------------------------------------------------------------ */
/*  Nav::ConvMagToPolar(float mXGauss, float mYGauss, float mZGauss)
**
**  Parameters:
**		mXGauss, mYGauss, mZGauss - the magnetic field values for all the three axes
**  Return Value:
**      POLAR_T - returns the POLAR_T structure members values
**		
**  Errors:
**		none		
**  Description:
**		The function computes the R and D, polar coordinates of the magnetic field. 
**		Updates the POLAR_T structure members D and R with the calculated values -  degrees of declination for D,
**		to further help indicate North, in compass functioning.
**
*/
POLAR_T Nav::ConvMagToPolar(float mXGauss, float mYGauss, float mZGauss)
{
	//update the POLAR_T structure member R with the field resultant
	coord.R = sqrt(pow(mXGauss,2)+ pow(mYGauss,2)+pow(mZGauss,2));
	//calculate the declination using two of the axes values, X and Y and reduce to first quadrant the values
	if (mXGauss == 0)
		coord.D = (mYGauss < 0) ? 90 : 0;
	else 
		coord.D = atan2(mYGauss,mXGauss)*180/PI;
	if (coord.D > 360)
	{
		coord.D -= 360;
	}
	
	else if (coord.D<0)
	{
		coord.D+=360;
	}
}
/*-------------------------------------------------------------*/
/*	Altimeter Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*  Nav::ComputePref(float altitudeMeters)
**
**  Parameters:
**		altitudeMeters	- the parameter used to calibrate the altitude computing, is considered known for 
**		the wanted location
**
**  Return Values:
**      void 
**
**  Errors:
**
**  Description:
**		This function provides the reference pressure computed with a known altitude for the given location
**			-	it performs a pressure reading, then computes the Reference pressure using the altitude parameter. 
**		It needs to be called once for the correct operation of the altitude function, all the following pressure readings
**		being affected by it.
**		This is needed because the current altitude is also affected by the current sea level air pressure, while the barometric 
**		pressure formula used to compute altitude is considering the sea level pressure constant at all times.						
**
*/
void Nav::ComputePref(float altitudeMeters)
{
	float Pinit, altCorrected;
	Pinit = ReadPressurehPa();  /* Measured Pressure  */

	float temp = 1 - (altitudeMeters*3.2808/145366.45);
	Pref= Pinit/(pow(temp,1/0.190284));
}

/*  Nav::ReadPressure()
**
**  Parameters:
**		none			
**
**  Return Values:
**      int32_t - returns the raw measured pressure value later used for converting it in hPa
**
**  Errors:
**		none
**  Description:
**		This function provides the 3 "raw" 16-bit values read from the barometer instrument. 
**			-	It reads the pressure value from the low, middle and high registers using the ReadRegister function
**			-	combines the three registers to obtain a 24-bit value for the pressure
*/
int32_t Nav::ReadPressure()
{
	uint8_t iPress_XL, iPress_L, iPress_H;
	uint8_t rgwRegVals[3];
	int32_t dataPress;
	ReadRegister(m_SSPinALT, PRESS_OUT_XL, 3, (uint8_t *)rgwRegVals);
	iPress_XL = rgwRegVals[0];
	iPress_L = rgwRegVals[1];
	iPress_H = rgwRegVals[2];
	dataPress = (iPress_H << 16)|iPress_L<<8|iPress_XL;
	return dataPress;
}
/* ------------------------------------------------------------ */
/*		Nav::ReadPressurehPa()
**
**   Parameters:
**		none		
**				
**   Return Values:
**      float - returns the value of measured pressure in hPa
**
**   Errors:
**		none
**
**   Description:
**		This function provides the pressure in hPa
**
*/
float Nav::ReadPressurehPa()
{
	uint32_t dataRawFull;
	dataRawFull = ReadPressure(); 
	//check if there is a negative value
	if (dataRawFull & 0x00800000){
            dataRawFull |= 0xFF000000;
    }
	hPa = dataRawFull/4096;
	return hPa;
}
/* ------------------------------------------------------------ */
/*  Nav::ConvPresToAltM(float hPa)
**
**   Parameters:
**		float hPa	- parameter representing the value of pressure in hPa
**				
**   Return Values:
**      float - it returns the current altitude based on the measured pressure and the previously computed reference pressure
**
**   Errors:
**		none
**
**   Description:
**		This function converts the current pressure to altitude using the previously computed Pref as reference pressure.
**		The Pref is computed once and used for further calculations of the altitude. The value returned is in meters
**
*/
float Nav::ConvPresToAltM(float hPa)
{
	float altMeters;
	float altCorrected = ((1-pow(hPa/Pref,0.190284))*145366.45);
	altMeters = altCorrected*0.3048;
	return altMeters;
}
/* ------------------------------------------------------------ */
/*  Nav::ConvPresToAltF(float hPa)
**
**   Parameters:
**		float hPa	- parameter representing the value of pressure in hPa
**				
**   Return Values:
**      float - returns the value of the altitude in feet
**
**   Errors:
**		none
**   Description:
**		This function performs the conversion from meters to feet and returns the value of altitude in feet
**
**
*/
float Nav::ConvPresToAltF(float hPa)
{
	float altfeet;
	altfeet = ((1-pow(hPa/Pref,0.190284))*145366.45);
	return altfeet;
}
/* ------------------------------------------------------------ */
/*   Nav::ReadTempC()
**
**   Parameters:
**		none
**				
**   Return Values:
**      float - the function returns the value of the read temperature in degrees Celsius 
**
**   Errors:
**		none
**   Description:
**		Reads and computes the temperature in degrees Celsius
**
*/
float Nav::ReadTempC()
{
	uint8_t rgwRegVals[2], tempL, tempH;
	ReadRegister(m_SSPinALT, TEMP_OUT_L, 2, (uint8_t *)rgwRegVals);
	tempL = rgwRegVals[0];	
	tempH = rgwRegVals[1];	
	int16_t temp = (int16_t)tempH <<8 | tempL; 
	//datasheet formula used for converting to temperature in degrees Celsius from raw values
	tempC = 42.5 +(temp * 0.002083);
	return tempC;
}
/* ------------------------------------------------------------ */
/*  Nav::ConvTempCToTempF(float tempC)
**
**  Parameters:
**		tempC	- parameter representing the value of temperature expressed in degrees Celsius
**				
**  Return Values:
**      float - returns the value of the temperature in degrees Fahrenheit 
**
**   Errors:
**
**   Description:
**		This function performs the conversion from Celsius to Fahrenheit degrees and returns the value of temperature in F
**
*/
float Nav::ConvTempCToTempF(float tempC)
{
	float tempF = 32 +(tempC * 1.8);
	return tempF;
}
/* ------------------------------------------------------------ */
/*    Nav::DataAvailableALT()
**
**   Parameters:
**		none	
**
**   Return Value:
**      uint8_t - returns the data available status in STATUS_REG register, for altimeter
**		
**   Errors:
**				
**   Description:
**		The function reads the STATUS_REG register and returns the data available bit in it
**
*/
uint8_t Nav::DataAvailableALT()
{
	uint8_t status;
	ReadRegister(m_SSPinALT, STATUS_REG, 1, &status);
	return ((status & (1<<1)) >> 1);
}
/* ------------------------------------------------------------ */
/*    Nav::TempAvailableALT()
**
**   Parameters:
**		none
**
**   Return Value:
**      uint8_t - returns the temperature available status in STATUS_REG register, for altimeter
**		
**   Errors:
**		none		
**   Description:
**		The function reads the STATUS_REG register and returns the temperature available bit in it
**
*/
uint8_t Nav::TempAvailableALT()
{
	uint8_t status;
	ReadRegister(m_SSPinALT, STATUS_REG, 1, &status);
	return (status & (1<<0));
}
/* ------------------------------------------------------------ */
/*  Nav::ConfigIntALT(uint8_t bIntGen, uint8_t bActiveType, uint8_t bOutputType, uint8_t dataSignalVal, 
**			bool intEnable, bool latch, uint8_t intLevel)
**
**   Parameters:
**		bIntGen - interrupt generator sources. It can be one of the following:
**				 MSK_INT_F_EMPTY			1<<3
**				 MSK_INT_F_FTH				1<<2
**				 MSK_INT_F_OVR				1<<1
**				 MSK_INT_DRDY				1<<0
**		bActiveType - interrupt active low or high parameter:
**				PAR_INT_ACTIVEHIGH		0
**				PAR_INT_ACTIVELOW		1	
**		dataSignalVal - INT_S bits value, representing the interrupt configurations, data signal, pressure high, low, or high and low
**			it can be one of the following:	
**				MSK_INT_P_HIGH				0x01
**				MSK_INT_P_LOW				0x02
**				MSK_INT_P_LOW_HIGH			0x03
**		intEnable - enable interrupts parameter
**		bOutputType - output type parameter, one of the following:
**				PAR_INT_OPENDRAIN			1
**				PAR_INT_PUSHPULL			0
**		latch - parameter indicating the interrupt event is latched or not
**		intLevel - set the level active interrupt for differential pressure value, on high or low
**				MSK_INT_LEVEL_HIGH				1<<0
**				MSK_INT_LEVEL_LOW				1<<1
**  Return Value:
**      none 
**		
**  Errors:
**		none		
**	Description:
**		The function configures the interrupts for altimeter instruments. It sets the interrupt generator for the three axes
**		It sets the active level, low or high for the interrupt event, interrupt configurations, output type.
**		It enables/disables the interrupt latching
**
*/
void Nav::ConfigIntALT(uint8_t bIntGen, uint8_t bActiveType, uint8_t bOutputType, uint8_t dataSignalVal, 
bool intEnable, bool latch, uint8_t intLevel)
{
	//enable interrupts in CTRL_REG1 register
	SetRegisterBits(m_SSPinALT, CTRL_REG1, MSK_DIFF_EN_ALT, intEnable);
	//mask out the reserved bits in CTRL_REG3 register
	uint8_t config = (bIntGen & 0xC3);	
	// INT_H_L bit is 1 for active-low, 0 for active-high.
	if (bActiveType == PAR_INT_ACTIVELOW) 
	{
		config |= (1<<7);
	}
	// PP_OD bit is 1 for open drain, 0 for push pull.
	if (bOutputType == PAR_INT_OPENDRAIN) 
	{
		config |= (1<<6);
	}
	config|=dataSignalVal;
	WriteSPI(m_SSPinALT, CTRL_REG3, config);
	config = bIntGen;
	WriteSPI(m_SSPinALT, CTRL_REG4, config);
	if (!latch) config |= (1<<2);
	config|= intLevel;
	WriteSPI(m_SSPinALT, INTERRUPT_CFG, config);
	
}
/* ------------------------------------------------------------ */
/*  Nav::SetIntThresholdALT(float thVal)
**
**   Parameters:
**		thVal - the interrupt threshold parameter for alt instrument
**   Return Value:
**      none 
**		
**   Errors:
**			
**   Description:
**		The function sets the interrupt threshold for the altimeter instrument
**
*/
void Nav::SetIntThresholdALT(float thVal)
{
	uint8_t buffer[2];
	uint16_t bthVal;
	//converts the float value in gauss to raw magnetic field data to be written in INT_THS_L_M/INT_THS_H_M registers
	bthVal = (uint16_t)(thVal*4096);
	//split bytes
	//make sure the first bit of the High byte is 0, for correct functionality of the device
	buffer[0] = (bthVal & 0xFF00) >> 8;
	buffer[1] = (bthVal & 0x00FF);
	WriteSPI(m_SSPinALT, THS_P_H, buffer[0]);
	WriteSPI(m_SSPinALT, THS_P_L, buffer[1]);
}

/* ------------------------------------------------------------ */
/*   Nav::GetIntSrcALT()
**
**  Parameters:
**		
**  Return Value:
**     uint8_t - parameter storing the interrupt source register content
**		
**  Errors:
**		none		
**  Description:
**		The function gets the interrupt sources by reading the INT_SOURCE register
**
*/
uint8_t Nav::GetIntSrcALT()
{
	uint8_t intSrc;
	intSrc = ReadSPI(m_SSPinALT, INT_SOURCE);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<2))
	{
		return intSrc; 
	}
	intSrc &= 0x03;
}
/*-------------------------------------------------------------*/
/*	Common Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*  Nav::SetODR(uint8_t bInstMode, uint8_t odrVal)
**
**    Parameters:
**			bInstMode - parameter representing the instrument to be affected
**			odrVal - the parameter specifying the ODR value for each instrument. Can be one of the parameters from the following list:
**					----for Accelerometer
**					0	ODR_XL_PWR_DWN	Parameter ODR value: power down the device
**					1	ODR_XL_10_HZ	Parameter ODR value: 10 Hz
**					2	ODR_XL_50_HZ	Parameter ODR value: 50 Hz
**					3	ODR_XL_119_HZ	Parameter ODR value: 119 Hz
**					4	ODR_XL_238_HZ	Parameter ODR value: 238 Hz
**					5	ODR_XL_476_HZ	Parameter ODR value: 476 Hz
**					6	ODR_XL_952_HZ	Parameter ODR value: 952 Hz
**					7	ODR_XL_NA		Parameter ODR value: not defined
**					----for Gyro
**					0	ODR_G_PWR_DWN	Parameter ODR value: power down the device
**					1	ODR_G_14_9HZ	Parameter ODR value: 14.9 Hz
**					2	ODR_G_59_5HZ	Parameter ODR value: 59.5 Hz
**					3	ODR_G_119HZ		Parameter ODR value: 119 Hz
**					4	ODR_G_238HZ		Parameter ODR value: 238 Hz
**					5	ODR_G_476HZ		Parameter ODR value: 476 Hz
**					6	ODR_G_952HZ		Parameter ODR value: 952 Hz
**					7	ODR_G_NA		Parameter ODR value: not defined
**					----for Magnetometer
**					0	ODR_M_0_625HZ	Parameter ODR value: 0.625 HZ
**					1	ODR_M_1_25HZ	Parameter ODR value: 1.25 Hz
**					2	ODR_M_2_5HZ		Parameter ODR value: 2.5 Hz
**					3	ODR_M_5HZ		Parameter ODR value: 5 Hz
**					4	ODR_M_10HZ		Parameter ODR value: 10 Hz
**					5	ODR_M_20HZ		Parameter ODR value: 20 Hz
**					6	ODR_M_40HZ		Parameter ODR value: 40 Hz
**					7	ODR_M_80HZ		Parameter ODR value: 80 HZ
**   Return Value:
**      none
**		
**   Errors:
**		none	
**
**   Description:
**		The function sets the ODR value for the selected instrument, in the corresponding register.
**		The accepted argument values are between 0 and 7.
**
*/
void Nav::SetODR(uint8_t bInstMode, uint8_t odrVal)
{
	uint8_t setODR;
	switch(bInstMode)
	{
		case MODE_INST_A:
			//set ODR for accelerometer instrument when used in single mode
			SetBitsInRegister(m_SSPinAG, CTRL_REG6_XL, MSK_ODR_XL, odrVal, 5);
			break;
		case MODE_INST_AG:
			//set ODR for gyro and accel instruments when used together
			SetBitsInRegister(m_SSPinAG, CTRL_REG1_G, MSK_ODR_G, odrVal, 5);
			break;
		case MODE_INST_MAG:
			//set ODR for magnetometer instrument 
			SetBitsInRegister(m_SSPinMAG, CTRL_REG6_XL, MSK_ODR_MAG, odrVal, 3);
			break;
		case MODE_INST_ALT:
			//set ODR for altimeter instrument 
			SetBitsInRegister(m_SSPinALT, CTRL_REG1, MSK_ODR_ALT, odrVal, 4);
			break;
		default:
			break;
	}
}
/* ------------------------------------------------------------ */
/*   Nav::GetODRRaw(uint8_t bInstMode)
**
**  Parameters:
**		bInstMode - parameter representing the instrument to be affected
**  Return Value:
**      uint8_t - returns the ODR raw value for the selected instrument, expressed in hexa
**		
**  Errors:
**		none		
**
**  Description:
**		The function sets the ODR value for the selected instrument, in the corresponding register. 
**		The accepted argument values are between 0 and 3.
**
*/
uint8_t Nav::GetODRRaw(uint8_t bInstMode)
{
	uint8_t getODR;
	switch(bInstMode)
	{
		case MODE_INST_A:
			getODR = GetBitsInRegister(m_SSPinAG, CTRL_REG6_XL, 5, 3);
			break;
		case MODE_INST_AG:
			getODR = GetBitsInRegister(m_SSPinAG, CTRL_REG1_G, 5, 3);
			break;
		case MODE_INST_MAG:
			getODR = GetBitsInRegister(m_SSPinMAG, CTRL_REG6_XL, 3, 3);
			break;
		case MODE_INST_ALT:
			getODR = GetBitsInRegister(m_SSPinALT, CTRL_REG1, 4, 3);
			break;
		default:
			break;
	}
	return getODR;
}

/* ------------------------------------------------------------ */
/*   Nav::GetODR(uint8_t bInstMode)
**
**   Parameters:
**		bInstMode - parameter representing the instrument to be affected
**   Return Value:
**      float - returns the ODR value of the selected instrument, based on the ODR bits reading from the corresponding register 
**		
**   Errors:
**		none		
**
**   Description:
**		The function reads the ODR bits from each instrument register and computes the real ODR to be provided to the user. 
**			
**
*/
float Nav::GetODR(uint8_t bInstMode)
{
	uint8_t odrRead;
	float odrFinal;
	if (bInstMode ==MODE_INST_A)
	{
		odrRead = GetODRRaw(MODE_INST_A);
		switch (odrRead)
		{
			case ODR_XL_PWR_DWN:
				odrFinal = 0;
				break;
			case ODR_XL_10HZ:
				odrFinal = 10;
				break;
			case ODR_XL_50HZ:
				odrFinal = 50;
				break;
			case ODR_XL_119HZ:
				odrFinal = 119;
				break;
			case ODR_XL_238HZ:
				odrFinal = 238;
				break;
			case ODR_XL_476HZ:
				odrFinal = 476;
				break;
			case ODR_XL_952HZ:
				odrFinal = 952;
				break;
			case ODR_XL_NA:
				odrFinal = -1;
				break;
			default:
				odrFinal = 0;
				break;
		}
	}
	//get odr for accel+gyro
	else if (bInstMode ==MODE_INST_AG)
	{
		odrRead = GetODRRaw(MODE_INST_AG);
		switch (odrRead)
		{
			case ODR_G_PWR_DWN:
				odrFinal = 0;
				break;
			case ODR_G_14_9HZ:
				odrFinal = 14.9;
				break;
			case ODR_G_59_5HZ:
				odrFinal = 59.5;
				break;
			case ODR_G_119HZ:
				odrFinal = 119;
				break;
			case ODR_G_238HZ:
				odrFinal = 238;
				break;
			case ODR_G_476HZ:
				odrFinal = 476;
				break;
			case ODR_G_952HZ:
				odrFinal = 952;
				break;
			case ODR_G_NA:
				odrFinal = -1;
				break;
			default:
				odrFinal = 0;
				break;
		}
	}
	//get odr for magnetometer
		else if (bInstMode ==MODE_INST_MAG)
		{
			odrRead = GetODRRaw(MODE_INST_MAG);
			switch (odrRead)
			{
				case ODR_M_0_625HZ:
					odrFinal = 0.625;
					break;
				case ODR_M_1_25HZ:
					odrFinal = 1.25;
					break;
				case ODR_M_2_5HZ:
					odrFinal = 2.5;
					break;
				case ODR_M_5HZ:
					odrFinal = 5;
					break;
				case ODR_M_10HZ:
					odrFinal = 10;
					break;
				case ODR_M_20HZ:
					odrFinal = 20;
					break;
				case ODR_M_40HZ:
					odrFinal = 40;
					break;
				case ODR_M_80HZ:
					odrFinal = 80;
					break;
				default:
					odrFinal = 10;
					break;
			}
		}
		//get odr for altimeter
		else if (bInstMode ==MODE_INST_ALT)
		{
			odrRead = GetODRRaw(MODE_INST_ALT);
			switch (odrRead)
			{
				case ODR_ALT_ONE_SHOT:
					odrFinal = 0;
					break;
				case ODR_ALT_1HZ:
					odrFinal = 1;
					break;
				case ODR_ALT_7HZ:
					odrFinal = 7;
					break;
				case ODR_ALT_12_5HZ:
					odrFinal = 12.5;
					break;
				case ODR_ALT_25HZ:
					odrFinal = 25;
					break;
				default:
					odrFinal = 0;
					break;
			}
		}
	else odrFinal = -1;
	return odrFinal;
}
/*-------------------------------------------------------------*/
/*	FIFO Functions
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*   Nav::FIFOEnable(int8_t ssPin, bool fEnable)
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**		fEnable - the parameter used to enable or disable the FIFO
**							
**   Return Value:
**      none
**		
**   Errors:
**		none
**   Description:
**		The function enables or disables FIFO by writing FIFO_EN bit in CTRL_REG9 register for Accel/Gyro, or CTRL_REG2 for Altimeter instrument
**
*/
void Nav::FIFOEnable(int8_t ssPin, bool fEnable)
{
	uint8_t temp;
	if(ssPin == m_SSPinAG)
	{
		ReadRegister(m_SSPinAG, CTRL_REG9, 1, &temp);
		if (fEnable) temp |= (1<<1);
		else temp &= ~(1<<1);
		WriteSPI(m_SSPinAG, CTRL_REG9, temp);
	}
	else
	{
		if(ssPin == m_SSPinALT)
		{
			ReadRegister(m_SSPinALT, CTRL_REG2, 1, &temp);
			if (fEnable) temp |= (1<<6);
			else temp &= ~(1<<6);
			WriteSPI(m_SSPinALT, CTRL_REG2, temp);
		}	
	}
}
/* ------------------------------------------------------------ */
/*   Nav::SetFIFO(int8_t ssPin, uint8_t parFIFOMode, uint8_t FIFOThreshold);
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**		parFIFOMode - the parameter specifying the FIFO mode for each instrument. Can be one of the parameters from the following list:
**					----for Accelerometer and Gyro instruments, when working together
**					0	FIFO_MODE_XL_G_BYPASS			Parameter FIFO mode value: bypass
**					1	FIFO_MODE_XL_G_FIFO				Parameter FIFO mode value: FIFO normal
**					3	FIFO_MODE_XL_G_CONTINUOUS_FIFO	Parameter FIFO mode value: continuous to FIFO
**					4	FIFO_MODE_XL_G_BYPASS_CONTINUOUS		Parameter FIFO mode value: bypass to continuous
**					6	FIFO_MODE_XL_G_CONTINUOUS		Parameter FIFO mode value: continuous
**					----for Altimeter instrument
**					0	FIFO_MODE_ALT_BYPASS			Parameter FIFO mode value: bypass
**					1	FIFO_MODE_ALT_FIFO				Parameter FIFO mode value: FIFO normal
**					2	FIFO_MODE_ALT_STREAM			Parameter FIFO mode value: stream mode
**					3	FIFO_MODE_ALT_STREAM_TO_FIFO	Parameter FIFO mode value: stream to fifo
**					4	FIFO_MODE_ALT_BYPASS_TO_STREAM	Parameter FIFO mode value: bypass to stream
**					6	FIFO_MODE_ALT_MEAN				Parameter FIFO mode value: mean mode
**					7	FIFO_MODE_ALT_BYPASS_TO_FIFO	Parameter FIFO mode value: bypass to fifo mode
**		FIFOThreshold - FIFO threshold level setting. Can be one of the parameters from the following list:
**					----for Accelerometer and Gyro instruments, when working together. Any value from 0-0x1F is acceptable
**					0			Parameter FIFO mode mean value: 0 samples
**					.
**					.
**					.
**					32			Parameter FIFO mode mean value: 32 samples
**					----for Altimeter instrument
**					0	FIFO_MODE_MEAN_ALT_2SAMPLES		Parameter FIFO mean mode value: 2 samples
**					1	FIFO_MODE_MEAN_ALT_4SAMPLES		Parameter FIFO mean mode value: 4 samples
**					2	FIFO_MODE_MEAN_ALT_8SAMPLES		Parameter FIFO mean mode value: 8 samples
**					3	FIFO_MODE_MEAN_ALT_16SAMPLES	Parameter FIFO mean mode value: 16 samples
**					4	FIFO_MODE_MEAN_ALT_32SAMPLES	Parameter FIFO mean mode value: 32 samples
**   Return Value:
**      none
**		
**   Errors:
**		none
**   Description:
**		The function sets the FIFO control mode and threshold in FIFO_CTRL register for the accel+gyro instruments and for altimeter instrument
**		The magnetometer instrument does not contain FIFO
**
*/
void Nav::SetFIFO(int8_t ssPin, uint8_t parFIFOMode, uint8_t FIFOThreshold)
{
	if(ssPin == m_SSPinAG)
	{

		SetBitsInRegister(m_SSPinAG, FIFO_CTRL, MSK_FIFO_CTL_MODE, parFIFOMode, 5);
		SetBitsInRegister(m_SSPinAG, FIFO_CTRL, MSK_FIFO_THS, FIFOThreshold, 0);
	}
	else
	{
		if(ssPin == m_SSPinALT)
		{
			SetBitsInRegister(m_SSPinALT, FIFO_CTRL, MSK_FIFO_CTL_MODE, parFIFOMode, 5);
			SetBitsInRegister(m_SSPinALT, FIFO_CTRL, MSK_FIFO_THS, FIFOThreshold, 0);
		}
	}
}
/* ------------------------------------------------------------ */
/*   Nav::GetFIFOMode(int8_t ssPin)
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**  Return Value:
**     uint8_t - returns the FIFO mode bits from FIFO_CTRL register for either Accel/Gyro or Altimeter instrument
**		
**  Errors:
**		none		
**  Description:
**		The function reads the FIFO mode bits in the FIFO_CTRL register for accel and gyro and Altimeter instruments 
**		and returns their value. 			
**
*/
uint8_t Nav::GetFIFOMode(int8_t ssPin)
{
	uint8_t getFIFO;
	if(ssPin == m_SSPinAG)
	{
		getFIFO = GetBitsInRegister(m_SSPinAG, FIFO_CTRL, 5, 3);
	}
	else
	{
		if(ssPin == m_SSPinALT)
		{
			getFIFO = GetBitsInRegister(m_SSPinALT, FIFO_CTRL, 5, 3);
		}
	}
	return getFIFO;
}

/* ------------------------------------------------------------ */
/*  Nav::GetFIFOThs(int8_t ssPin)
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter 
**	Return Value
**		uint8_t - specifies the FIFO mean number of samples for each instrument. Can be one of the parameters from the following list:
**					----for Accelerometer and Gyro instruments, when working together
**					0			Parameter FIFO mode mean value: 0 samples
**					.
**					.
**					.
**					32			Parameter FIFO mode mean value: 32 samples
**					----for Altimeter instrument
**					0	FIFO_MODE_MEAN_ALT_2SAMPLES		Parameter FIFO mean mode value: 2 samples
**					1	FIFO_MODE_MEAN_ALT_4SAMPLES		Parameter FIFO mean mode value: 4 samples
**					2	FIFO_MODE_MEAN_ALT_8SAMPLES		Parameter FIFO mean mode value: 8 samples
**					3	FIFO_MODE_MEAN_ALT_16SAMPLES	Parameter FIFO mean mode value: 16 samples
**					4	FIFO_MODE_MEAN_ALT_32SAMPLES	Parameter FIFO mean mode value: 32 samples
**		
**  Errors:
**		none	
**
**  Description:
**		The function returns the FIFO mean/threshold mode bits in FIFO_CTRL register for the accel+gyro instruments or for altimeter instrument
**		The magnetometer instrument does not contain FIFO
**
*/
uint8_t Nav::GetFIFOThs(int8_t ssPin)
{
	uint8_t bFifoThs;
	if(ssPin == m_SSPinAG)
	{
		bFifoThs = GetBitsInRegister(m_SSPinAG, FIFO_CTRL, 0, 5);
	}
	else
	{
		if(ssPin == m_SSPinALT)
		{
			bFifoThs = GetBitsInRegister(m_SSPinALT, FIFO_CTRL, 0, 5);
		}
	}	
	return bFifoThs;
}
/* ------------------------------------------------------------ */
/*   Nav::GetFIFOStatus(int8_t ssPin)
**
**  Parameters:
**		ssPin - parameter for selecting one of the instruments' Slave/Chip Select signal: A/G or Altimeter
**  Return Value:
**     uint8_t - returns the FIFO status FIFO_STATUS or FIFO_SRC register, depending on the selected instrument
**		
**  Errors:
**		none		
**  Description:
**		The function reads the FIFO_SRC register for accel and gyro and FIFO_STATUS register for Altimeter instrument
**		and returns their value. 			
**
*/
uint8_t Nav::GetFIFOStatus(int8_t ssPin)
{
	uint8_t getFIFOSts;
	if(ssPin == m_SSPinAG)
	{
		getFIFOSts = ReadSPI(m_SSPinAG, FIFO_SRC);
	}
	else
	{
		if(ssPin == m_SSPinALT)
		{
			getFIFOSts = ReadSPI(m_SSPinALT, FIFO_STATUS);
		}
	}
	return getFIFOSts;
}
/*-------------------------------------------------------------*/
/*	Bits Specific Functions
/* ------------------------------------------------------------ */
/* ------------------------------------------------------------ */
/*   Nav::SetBitsInRegister(int8_t ssPin, uint8_t bRegAddr, uint8_t bMask, uint8_t bValue, uint8_t startBit)
**
**   Parameters:
**		ssPin				- instrument selection for chip select: AG/MAG/ALT
**		bRegAddr		 	- the address of the register whose bits are set
**		bMask				- the mask indicating which bits are affected
**		bValue				- the byte containing bits values
**		startBit			- start bit of the bits group to be set in register 
**
**   Return Values:
**       none 
**   Errors:
**		none
**   Description:
**		This function sets the value of some bits (corresponding to the bMask) of a register (indicated by bRegAddr) to the value of the corresponding bits from another byte (indicated by bValue)
**		starting from the position indicated by startBit.
**
*/
void Nav::SetBitsInRegister(int8_t ssPin, uint8_t bRegAddr, uint8_t bMask, uint8_t bValue, uint8_t startBit)
{
	uint8_t bRegValue, shiftedValue;
	shiftedValue = (bValue << startBit);
	ReadRegister(ssPin, bRegAddr, 1, &bRegValue);
	// register value: mask out the bits from the mask
	bRegValue &= ~bMask;
	// value: mask out the values outside the mask
	shiftedValue &= bMask;
	// combine the value with the masked register value
	bRegValue |= (shiftedValue & bMask);
	WriteRegister(ssPin, bRegAddr, 1, &bRegValue);
}

/* ------------------------------------------------------------ */
/*  Nav::GetBitsInRegister(int8_t ssPin, uint8_t bRegAddr, uint8_t startBit, uint8_t noBits)
**
**  Parameters:
**		ssPin				- instrument selection for chip select: AG/MAG/ALT
**		bRegAddr		 	- the address of the register whose bits are set
**		startBit			- start bit of the bits group to be set in register 
**		noBits				- number of bits starting from start bit, to be read
**
**   Return Values:
**      none 
**   Errors:
**		none
**   Description:
**		This function gets the value of some bits (given by the startBts and noBits parameters) of a register (indicated by bRegisterAddress).
**
*/
uint8_t Nav::GetBitsInRegister(int8_t ssPin, uint8_t bRegAddr, uint8_t startBit, uint8_t noBits)
{
	uint8_t bRegValue, bResult, bMask;
	ReadRegister(ssPin, bRegAddr, 1, &bRegValue);
	bMask = ((1<<noBits)-1)<< startBit;
	bResult = (bRegValue & bMask) >> startBit;
	
	return bResult;	
}
/* ------------------------------------------------------------ */
/*   Nav::SetRegisterBits(int8_t ssPin, uint8_t bRegAddr, uint8_t bMask, bool fValue)
**
**   Parameters:
**		ssPin				- instrument selection for chip select: AG/MAG/ALT
**		bRegAddr		 	- the address of the register whose bits are set
**		bMask				- the mask indicating which bits are affected
**		fValue				- 1 if the bits are set or 0 if their bits are reset
**
**   Return Values:
**      none 
**   Errors:
**		none
**   Description:
**		This function sets the value of some bits (corresponding to the bMask) of a register (indicated by bRegAddr) to 1 or 0 (indicated by fValue).
**
*/
void Nav::SetRegisterBits(int8_t ssPin, uint8_t bRegAddr, uint8_t bMask, bool fValue)
{
	uint8_t bRegValue;
	ReadRegister(ssPin, bRegAddr, 1, &bRegValue);
	if(fValue)
	{
		// set 1 value to the values that are 1 in the mask
		bRegValue |= bMask;
	}
	else
	{
		// set 0 value to the values that are 1 in the mask
		bRegValue &= ~bMask;
	}
	WriteRegister(ssPin, bRegAddr, 1, &bRegValue);
}

