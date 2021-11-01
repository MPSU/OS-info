 
/************************************************************************/
/*									*/
/*  NAVDemo  --  NAV demo                               */
/* demostrates some basic ACL, Gyro, Magnetometer and Altimeter functions            */
/*									*/
/************************************************************************/
/*  Author: 	Monica Ignat						*/
/*  Copyright 2015, Digilent Inc.					*/
/************************************************************************/

/************************************************************************/
/*  Revision History:							*/
/*									*/
/*  10/26/2015(MonicaI): created					*/
/*	01/18/2016(MonicaI): added new generic GetData(), GetDeviceID(), Init() functions for perform readings from and settings of all the instruments
/*	05/26/2017(CristianF): modified begin function to accept SS parameters for each instrument, moved the setup from class constructor to begin function			*/
/*					
/************************************************************************/

/************************************************************************/
/*  Board Support:							*/
/*									*/
/*  chipKit Uno with Pmod Shield:     Header JC	                	*/
/*   (Note: To use this, download and deploy the Board Variant for      */
/*          chipKIT UNO32 W/Pmod Shield from the Digilent website)      */  
/* chipKit Uno32 without Pmod Shield connections:
/*		PmodNAV pins     Uno32 pins										*/
/*		1  CS_A/G        J8-05 (44)										*/
/*		2  SDI           J8-04											*/
/*		3  SDO           J8-01											*/
/*		4  SCK           J8-02											*/
/*		9  CS_M          J6-05 (2) - optional, any other digital pin can be used */
/*		10 CS_ALT        J6-07 (3) - optional, any other digital pin can be used */  
/* chipKit uC32 without Pmod Shield connections:
/*		PmodNAV pins     uC32 pins										*/
/*		1  CS_A/G        J8-05 (44)										*/
/*		2  SDI           J8-04											*/
/*		3  SDO           J8-01											*/
/*		4  SCK           J8-02											*/
/*		9  CS_M          J6-05 (2) - optional, any other digital pin can be used */
/*		10 CS_ALT        J6-07 (3) - optional, any other digital pin can be used */       
/*  chipKIT� Pro MX3:                    Header JE                      */
/*  chipKIT� Pro MX4:                    Header JB                      */
/*                   (make sure JP3 is on RB15)                         */
/*  chipKIT� Pro MX7:                    Header JD                      */
/************************************************************************/

#include <DSPI.h>
#include <Nav.h>


Nav navObj;	

void setup()
{
	//setup the SPI communications and basic device initializations
	//navObj.begin(0, 44, 2, 3, -1, -1);//example of use for uC32, Uno32 boards
	//navObj.begin(0, 24, 30, 31, 29, 28);  // example of use for chipKIT� Pro MX7 board
	//navObj.begin(0, 8, 14, 15, -1, -1);	// example of use for chipKIT� Pro MX4 board
	navObj.begin();	// DSPI0, all pimns in the PMod connector
	Serial.begin(9600);
	//generic function to perform basic initializations of all devices: Accel/Gyro, Magnetometer, Altimeter
	navObj.Init();
	/*------------------------------------------------------------*/
	//set the known altitude as reference for future measurements of the altitude
	//uncomment the below line to improve accuracy of the altimeter device by specifying the known altitude for your current location

	//navObj.ComputePref(349); //altitude for Cluj-Napoca town, taken from:http://www.altitude-maps.com/city/175_218,Cluj-Napoca,Cluj,Romania
	/*------------------------------------------------------------*/
	delay(100);
}

void loop()
{
	char strTemp[50];
	//the function reads the ID for each of the devices: Accel/Gyro, Magnetometer and Altimeter from WHO_AM_I registers and stores them in idData
	navObj.GetDeviceID();
	//print the device ID for Accel/Gyro stored in idData.ag struct member
	sprintf(strTemp, "ID ACCEL+GYRO:0x%02X", navObj.idData.ag); 
	Serial.println(strTemp);
	//print the device ID for magnetometer stored in idData.mag struct member
	sprintf(strTemp, "ID MAG:0x%02X", navObj.idData.mag); 
	Serial.println(strTemp);
	//print the device ID for altimeter stored in idData.alt struct member
	sprintf(strTemp, "ID ALT:0x%02X", navObj.idData.alt); 
	Serial.println(strTemp);
	
	/*function used to read data from all the instruments and stores them in structure members and global variables
	it reads the accl, gyro, magnetometer, pressure, temperature and stores them in the corresponding variables:
	- acclData for all three axes of the accelerometer instrument
	- gyroData for all three axes of the gyro instrument
	- magData for all three axes of the magnetometer instrument
	- hPa pressure in hPa for the barometer/altimeter instrument
	- tempC temperature in degrees Celsius read from altimeter instrument 
	*/
	navObj.GetData();
	
	Serial.println("Accelerometer G values");
	//format and serial print the data read from accelerometer instrument, stored in acclData variables and expressed in G
	sprintf(strTemp, "X:%6.3f G, Y:%6.3f G, Z:%6.3f G", navObj.acclData.X, navObj.acclData.Y, navObj.acclData.Z); 
	Serial.println(strTemp);
	
	//format and serial print the data read from Gyro instrument, stored in gryoData variables and expressed in degrees per second
	Serial.println("Gyro dps values");
	sprintf(strTemp, "XG:%6.3f dps, YG:%6.3f dps, ZG:%6.3f dps", navObj.gyroData.X, navObj.gyroData.Y, navObj.gyroData.Z); 
	Serial.println(strTemp);
	
	//format and serial print the data read from magnetometer instrument, stored in magData variables and expressed in Gauss
	Serial.println("Magnetometer Gauss values");
	sprintf(strTemp, "XM:%6.3f Gauss, YM:%6.3f Gauss, ZM:%6.3f Gauss", navObj.magData.X, navObj.magData.Y, navObj.magData.Z); 
	Serial.println(strTemp);
	
	//format and serial print the heading computed in ConvMagToPolar function as arctg(magData.Y/magData.X)
	navObj.ConvMagToPolar(navObj.magData.X, navObj.magData.Y, navObj.magData.Z);
	sprintf(strTemp, "Heading in Degrees: %4.2f", navObj.coord.D );
	Serial.println(strTemp);
	
	if (( 337.25 < navObj.coord.D ) || ( 22.5 > navObj.coord.D )) {
          Serial.println("Direction: North");
        }
        else if ( 292.5 < navObj.coord.D ) {
          Serial.println("Direction: North-West");
        }
        else if ( 247.5 < navObj.coord.D ) {
          Serial.println("Direction: West");
        }
        else if ( 202.5 < navObj.coord.D ) {
          Serial.println("Direction: South-West");
        }
        else if ( 157.5 < navObj.coord.D ) {
          Serial.println("Direction: South");
        }
        else if ( 112.5 < navObj.coord.D ) {
          Serial.println("Direction: South-East");
        }
        else if ( 67.5 < navObj.coord.D ) {
          Serial.println("Direction: East");
        }
        else {
          Serial.println("Direction: North-East");
        }
	
	//format and serial print the temperature in degrees Celsius
	sprintf(strTemp, "Temperature C:%4.2f", navObj.tempC); 
	Serial.println(strTemp);
	//format and serial print the temperature in degrees Farenheit
	float tempF = navObj.ConvTempCToTempF(navObj.tempC);
	sprintf(strTemp, "Temperature F:%4.2f", tempF);
	Serial.println(strTemp);
	
	//format and serial print the data read from altimeter instrument, stored in hPa variable
	sprintf(strTemp, "Pressure hPa:%4.3f", navObj.hPa); 
	Serial.println(strTemp);
	//convert the pressure to altitude in meters and feet
	float altM = navObj.ConvPresToAltM(navObj.hPa);
	float altFeet = navObj.ConvPresToAltF(navObj.hPa);
	//format and serial print the altitude
	sprintf(strTemp, "Altitude in meters:%4.2f", altM);
	Serial.println(strTemp);
	sprintf(strTemp, "Altitude in feet:%4.2f", altFeet);
	Serial.println(strTemp);
	Serial.print("-----------------------------------------------");
	Serial.println();
	delay(1000);
}
