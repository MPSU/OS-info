/************************************************************************/
/*																		*/
/* Nav.h	--	Interface Declarations for Nav.cpp		*/
/*																		*/
/************************************************************************/
/*	Author:		MonicaI											*/
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
/*  File Description:													*/
/*																		*/
/*	This header file contains the object class declarations and other	*/
/*	interface declarations need to use the PmodNAV driver				*/
/*	for the Digilent microcontroller boards. chipKIT™ Pro MX4 is used	*/
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
/*					Miscellaneous Declarations					*/
/* ------------------------------------------------------------ */


#if !defined(NAV_H)
#define NAV_H

#include <inttypes.h>
#include <DSPI.h>

#define ACT_THS             0B00000100
#define ACT_DUR             0B00000101
#define INT_GEN_CFG_XL      0B00000110
#define INT_GEN_THS_X_XL    0B00000111
#define INT_GEN_THS_Y_XL    0B00001000
#define INT_GEN_THS_Z_XL    0B00001001
#define INT_GEN_DUR_XL      0B00001010
#define REFERENCE_G         0B00001011
#define INT1_CTRL           0B00001100
#define INT2_CTRL           0B00001101

#define WHO_AM_I            0B00001111
#define CTRL_REG1_G         0B00010000
#define CTRL_REG2_G         0B00010001
#define CTRL_REG3_G         0B00010010
#define ORIENT_CFG_G        0B00010011
#define INT_GEN_SRC_G       0B00010100
#define OUT_TEMP_L          0B00010101
#define OUT_TEMP_H          0B00010110
#define STATUS_REG0         0B00010111
#define OUT_X_L_G           0B00011000
#define OUT_X_H_G           0B00011001
#define OUT_Y_L_G           0B00011010
#define OUT_Y_H_G           0B00011011
#define OUT_Z_L_G           0B00011100
#define OUT_Z_H_G           0B00011101
#define CTRL_REG4           0B00011110
#define CTRL_REG5_XL        0B00011111
#define CTRL_REG6_XL        0B00100000
#define CTRL_REG7_XL        0B00100001
#define CTRL_REG8           0B00100010
#define CTRL_REG9           0B00100011
#define CTRL_REG10          0B00100100

#define INT_GEN_SRC_XL      0B00100110
#define STATUS_REG          0B00100111
#define OUT_X_L_XL          0B00101000
#define OUT_X_H_XL          0B00101001
#define OUT_Y_L_XL          0B00101010
#define OUT_Y_H_XL          0B00101011
#define OUT_Z_L_XL          0B00101100
#define OUT_Z_H_XL          0B00101101
#define FIFO_CTRL           0B00101110
#define FIFO_SRC            0B00101111
#define INT_GEN_CFG_G       0B00110000
#define INT_GEN_THS_XH_G    0B00110001
#define INT_GEN_THS_XL_G    0B00110010
#define INT_GEN_THS_YH_G    0B00110011
#define INT_GEN_THS_YL_G    0B00110100
#define INT_GEN_THS_ZH_G    0B00110101
#define INT_GEN_THS_ZL_G    0B00110110
#define INT_GEN_DUR_G       0B00110111

#define OFFSET_X_REG_L_M    0B00000101
#define OFFSET_X_REG_H_M    0B00000110
#define OFFSET_Y_REG_L_M    0B00000111
#define OFFSET_Y_REG_H_M    0B00001000
#define OFFSET_Z_REG_L_M    0B00001001
#define OFFSET_Z_REG_H_M    0B00001010

#define WHO_AM_I_M          0B00001111

#define CTRL_REG1_M         0B00100000
#define CTRL_REG2_M         0B00100001
#define CTRL_REG3_M         0B00100010
#define CTRL_REG4_M         0B00100011
#define CTRL_REG5_M         0B00100100

#define STATUS_REG_M        0B00100111
#define OUT_X_L_M           0B00101000
#define OUT_X_H_M           0B00101001
#define OUT_Y_L_M           0B00101010
#define OUT_Y_H_M           0B00101011
#define OUT_Z_L_M           0B00101100
#define OUT_Z_H_M           0B00101101

#define INT_CFG_M           0B00110000
#define INT_SRC_M           0B00110001
#define INT_THS_L_M         0B00110010
#define INT_THS_H_M         0B00110011

#define REF_P_XL			0B00001000
#define REF_P_L				0B00001001
#define REF_P_H				0B00001010
#define WHO_AM_I			0B00001111
#define RES_CONF			0B00010000

#define CTRL_REG1			0B00100000
#define CTRL_REG2			0B00100001
#define CTRL_REG3			0B00100010
#define CTRL_REG4			0B00100011
#define INTERRUPT_CFG		0B00100100
#define INT_SOURCE			0B00100101

#define STATUS_REG			0B00100111
#define PRESS_OUT_XL		0B00101000
#define PRESS_OUT_L			0B00101001
#define PRESS_OUT_H			0B00101010
#define TEMP_OUT_L			0B00101011
#define TEMP_OUT_H			0B00101100

#define FIFO_CTRL			0B00101110
#define FIFO_STATUS			0B00101111
#define THS_P_L				0B00110000
#define THS_P_H				0B00110001

#define RPDS_L				0B00111001
#define RPDS_H				0B00111010


#define MODE_INST_A		0
#define MODE_INST_AG	1
#define MODE_INST_MAG	2
#define MODE_INST_ALT	3

#define INT_PIN_1		0
#define	INT_PIN_2		1


#define	PAR_XL_2G		0
#define	PAR_XL_4G		2
#define	PAR_XL_8G		3
#define	PAR_XL_16G		1
#define	PAR_G_245DPS	0
#define	PAR_G_500DPS	1	
#define	PAR_G_2kDPS		3
#define PAR_MAG_4GAUSS	0
#define PAR_MAG_8GAUSS	1
#define PAR_MAG_12GAUSS	2
#define PAR_MAG_16GAUSS	3

#define	PAR_INT_ACTIVEHIGH		0
#define	PAR_INT_ACTIVELOW		1
#define PAR_INT_PUSHPULL		0
#define PAR_INT_OPENDRAIN		1
#define	PAR_EXT_INT0			0
#define	PAR_EXT_INT1			1
#define	PAR_EXT_INT2			2
#define	PAR_EXT_INT3			3
#define	PAR_EXT_INT4			4


#define	FIFO_MODE_XL_G_BYPASS			0
#define	FIFO_MODE_XL_G_FIFO				1
#define	FIFO_MODE_XL_G_CONTINUOUS_FIFO	3
#define	FIFO_MODE_XL_G_BYPASS_CONTINUOUS		4
#define	FIFO_MODE_XL_G_CONTINUOUS		6

#define	FIFO_MODE_ALT_BYPASS			0
#define	FIFO_MODE_ALT_FIFO				1
#define	FIFO_MODE_ALT_STREAM			2
#define	FIFO_MODE_ALT_STREAM_TO_FIFO	3
#define	FIFO_MODE_ALT_BYPASS_TO_STREAM	4
#define	FIFO_MODE_ALT_MEAN				6
#define	FIFO_MODE_ALT_BYPASS_TO_FIFO	7

#define	FIFO_MODE_MEAN_ALT_2SAMPLES		0B00001
#define	FIFO_MODE_MEAN_ALT_4SAMPLES		0B00011
#define	FIFO_MODE_MEAN_ALT_8SAMPLES		0B00111
#define	FIFO_MODE_MEAN_ALT_16SAMPLES	0B01111
#define	FIFO_MODE_MEAN_ALT_32SAMPLES	0B11111


#define	ACL_NO_BITS		16
#define	X_AXIS			0
#define	Y_AXIS			1
#define	Z_AXIS			2
#define	ALL_AXIS		3
#define PI				3.1415

#define	MSK_RANGE_XL	0x18
#define	MSK_RANGE_G		0x18
#define MSK_RANGE_MAG	0x60
#define MSK_ODR_XL		0xE0
#define MSK_ODR_G		0xE0
#define MSK_ODR_MAG		0x1C
#define MSK_ODR_ALT		0x70
#define MSK_FIFO_CTL_MODE	0xE0
#define MSK_FIFO_THS	0x1F

#define	MSK_INT1_IG_G				1<<7
#define	MSK_INT1_IG_XL				1<<6
#define	MSK_INT1_FSS5				1<<5
#define	MSK_INT1_OVR				1<<4
#define	MSK_INT1_FTH				1<<3
#define	MSK_INT1_Boot				1<<2
#define	MSK_INT1_DRDY_G				1<<1
#define	MSK_INT1_DRDY_XL			1<<0

#define	MSK_INT2_INACT				1<<7
#define	MSK_INT2_FSS5				1<<5
#define	MSK_INT2_OVR				1<<4
#define	MSK_INT2_FTH				1<<3
#define	MSK_INT2_DRDY_TEMP			1<<2
#define	MSK_INT2_DRDY_G				1<<1
#define	MSK_INT2_DRDY_XL			1<<0

#define MSK_XLIE_XL 				1<<0
#define MSK_XHIE_XL 				1<<1
#define MSK_YLIE_XL 				1<<2
#define MSK_YHIE_XL 				1<<3
#define MSK_ZLIE_XL 				1<<4
#define MSK_ZHIE_XL 				1<<5
#define MSK_GEN_6D 					1<<6

#define MSK_XLIE_G	 				1<<0
#define MSK_XHIE_G					1<<1
#define MSK_YLIE_G					1<<2
#define MSK_YHIE_G					1<<3
#define MSK_ZLIE_G					1<<4
#define MSK_ZHIE_G					1<<5

#define MSK_ZIEN_MAG				1<<5
#define MSK_YIEN_MAG				1<<6
#define MSK_XIEN_MAG 				1<<7

#define MSK_DIFF_EN_ALT 			1<<3
#define MSK_INT_H_L					1<<7
#define MSK_INT_PP_OD				1<<6
#define MSK_INT_S1_S2				0x03

#define MSK_INT_P_HIGH				0x01
#define MSK_INT_P_LOW				0x02
#define MSK_INT_P_LOW_HIGH			0x03
//int configurations for Altimeter
#define MSK_INT_F_EMPTY				1<<3
#define MSK_INT_F_FTH				1<<2
#define MSK_INT_F_OVR				1<<1
#define MSK_INT_DRDY				1<<0

#define MSK_INT_LEVEL_HIGH				1<<0
#define MSK_INT_LEVEL_LOW				1<<1

#define ODR_G_PWR_DWN	0
#define ODR_G_14_9HZ	1
#define ODR_G_59_5HZ	2
#define ODR_G_119HZ		3
#define ODR_G_238HZ		4
#define ODR_G_476HZ		5
#define ODR_G_952HZ		6
#define ODR_G_NA		7
//ODR defined values for Accelerometer
#define ODR_XL_PWR_DWN	0
#define ODR_XL_10HZ		1
#define ODR_XL_50HZ		2
#define ODR_XL_119HZ	3
#define ODR_XL_238HZ	4
#define ODR_XL_476HZ	5
#define ODR_XL_952HZ	6
#define ODR_XL_NA		7
//ODR defined values for magnetometer
#define ODR_M_0_625HZ	0
#define ODR_M_1_25HZ	1
#define ODR_M_2_5HZ		2
#define ODR_M_5HZ		3
#define ODR_M_10HZ		4
#define ODR_M_20HZ		5
#define ODR_M_40HZ		6
#define ODR_M_80HZ		7

#define ODR_ALT_ONE_SHOT 0
#define ODR_ALT_1HZ		1
#define ODR_ALT_7HZ		2
#define ODR_ALT_12_5HZ	3
#define ODR_ALT_25HZ	4


/* ------------------------------------------------------------ */
/*					Global Variable Declarations				*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Object Class Declarations					*/
/* ------------------------------------------------------------ */
struct ACCL_T {
	float X; // Acceleration in G's in X direction
	float Y; // Acceleration in G's in Y direction
	float Z; // Acceleration in G's in Z direction
};

struct GYRO_T {
	float X; // DPS in X direction
	float Y; // DPS in Y direction
	float Z; // DPS in Z direction
};

struct MAGNO_T {
	float X; // Gauss in X direction
	float Y; // Gauss in Y direction
	float Z; // Gauss in Z direction
	
};
struct POLAR_T {
	float R; // the total magnetic field computed from the three axes components;
	float D; // the angle between Z axis and the rezultant of X and Y
};

struct DEV_ID {
	uint8_t ag; // device ID for AG instrument
	uint8_t mag; // device ID for MAG instrument
	uint8_t alt; // device ID for ALT instrument
};
class Nav
{
 private: 
	DSPI *pdspi;
	uint8_t prtSel;
	int8_t m_SSPinAG;
	int8_t m_SSPinMAG;
	int8_t m_SSPinALT;
	int8_t m_DRDYPinMAG;
	int8_t m_INTPin;	
	float m_GRangeLSB;
	float m_DPSRangeLSB;
	float m_GaussRangeLSB;
	float Pref;
public:
	struct ACCL_T acclData;
	struct GYRO_T gyroData;
	struct MAGNO_T magData;
	struct DEV_ID idData;
	struct POLAR_T coord;
	float hPa;
	float tempC;
	

	
private:
	void NavHostTerm();
	void NavHostInit();

	void NavDevInit();
	void NavDevTerm();
	// read and write SPI specific functions - read and write one byte via SPI
	void WriteSPI(int8_t ssPin, uint8_t bAddr, uint8_t bVal);
	uint8_t ReadSPI(int8_t ssPin, uint8_t bAddr);	
	//bits operations functions
	void SetBitsInRegister(int8_t ssPin, uint8_t bRegAddr, uint8_t bMask, uint8_t bValue, uint8_t startBit);
	uint8_t GetBitsInRegister(int8_t ssPin, uint8_t bRegAddr, uint8_t startBit, uint8_t noBits);
	void SetRegisterBits(int8_t ssPin, uint8_t bRegAddr, uint8_t bMask, bool fValue);

	//library internal functions used to convert data from raw values into g, dps, Gauss
	float ConvertReadingToValueG(int16_t rawVal);
	float ConvertReadingToValueDPS(int16_t rawVal);
	float ConvertReadingToValueGauss(int16_t rawVal);
	//library internal functions used to compute the data using the corresponding range and LSB values
	float GetGRangeLSB(uint8_t bRangeG);
	float GetXLRangeLSB(uint8_t bRangeXL);
	float GetMAGRangeLSB(uint8_t bRangeMAG);
	void ComputePref(float altitudeMeters);
public:
	Nav();

	~Nav();
	//--------------------------------------------------- Basic device control functions.
    void begin(uint8_t prtSpi, int8_t SSPinAG, int8_t SSPinMAG, int8_t SSPinALT, int8_t DRDYPinMAG, int8_t INTPin);
	void begin();
	void end(void);
	void GetData();
	void InitALT(bool fInit);
	void InitAG(bool fInit, uint8_t bModeSel);
	void InitMAG(bool fInit);
	void Init();
	void GetDeviceID();
	
	//--------------------------------------------------- SPI specific functions
	void WriteRegister(int8_t ssPin, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData);
	void ReadRegister(int8_t ssPin, uint8_t bAddr, uint8_t bCntBytes, uint8_t *pData);
	
	//--------------------------------------------------accelerometer specific functions
	void ReadAccel(int16_t &AclX, int16_t &AclY, int16_t &AclZ);
	void ReadAccelG(float &AclXg, float &AclYg, float &AclZg); 
	void SetRangeXL(uint8_t bRangeXL);
	uint8_t GetRangeXL();
	uint8_t DataAvailableXL();
	void ConfigIntXL(uint8_t bIntGen, bool aoi, bool latch);
	uint8_t GetIntSrcXLG(uint8_t bInstMode);
	void SetIntThresholdXL(float thValX, float thValY, float thValZ, uint8_t intDuration, bool wait);
	
	//--------------------------------------------------gyro specific functions
	void ReadGyro(int16_t &GX, int16_t &GY, int16_t &GZ);
	void ReadGyroDps(float &GXdps, float &GYdps, float &GZdps);
	void SetRangeG(uint8_t bRangeG);
	float GetRangeG();
	uint8_t DataAvailableG();
	void ConfigIntG(uint8_t bIntGen, bool aoi, bool latch);
	void SetIntThresholdG(float thVal, uint16_t axis, bool drCntMode, uint8_t intDuration, bool wait);
	//configure INT1 and INT2 pins and use external interrupts functions
	void ConfigInt(uint8_t bIntPin, uint8_t bIntGenMask, uint8_t bActiveType, uint8_t bOutputType);
	void ConfigIntForExternalIntUse(uint8_t bIntPin, uint8_t bParExtIntNo, uint8_t bIntGenMask,
		void (*pfIntHandler)(), uint8_t bActiveType, uint8_t bOutputType);
		
	//--------------------------------------------------magmetometer specific functions
	void ReadMag(int16_t &MagX, int16_t &MagY, int16_t &MagZ);
	void ReadMagGauss(float &MagXGauss, float &MagYGauss, float &MagZGauss);
	void SetRangeMAG(uint8_t bRangeMAG);
	uint8_t GetRangeMAG();
	uint8_t DataAvailableMAG(uint8_t axis);
	void ConfigIntMAG(uint8_t bIntGen, uint8_t bActiveType, bool latch);
	void SetIntThresholdM(float thVal);
	uint8_t GetIntSrcMAG();
	
	//--------------------------------------------------altimeter functions
	int32_t ReadPressure();
	float ReadPressurehPa();
	POLAR_T ConvMagToPolar(float magXGauss, float magYGauss, float magZGauss);
	float ConvPresToAltM(float hPa);
	float ConvPresToAltF(float hPa);
	float ConvTempCToTempF(float tempC);
	float ReadTempC();
	uint8_t TempAvailableALT();
	uint8_t DataAvailableALT();
	void ConfigIntALT(uint8_t bIntGen, uint8_t bActiveType, uint8_t bOutputType, uint8_t dataSignalVal, 
		bool intEnable, bool latch, uint8_t intLevel);
	void SetIntThresholdALT(float thVal);
	uint8_t GetIntSrcALT();
	
	//--------------------------------------------------common functions
	void SetODR(uint8_t bInstMode, uint8_t odrVal);
	uint8_t GetODRRaw(uint8_t bInstMode);
	float GetODR(uint8_t bInstMode);
	
	//--------------------------------------------------FIFO specific functions
	void FIFOEnable(int8_t ssPin, bool fEnable);
	void SetFIFO(int8_t ssPin, uint8_t parFIFOMode, uint8_t FIFOThreshold);
	uint8_t GetFIFOMode(int8_t ssPin);
	uint8_t GetFIFOThs(int8_t ssPin);
	uint8_t GetFIFOStatus(int8_t ssPin);
};

/* ------------------------------------------------------------ */

#endif

/************************************************************************/
