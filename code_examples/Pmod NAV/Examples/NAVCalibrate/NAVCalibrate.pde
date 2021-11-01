 
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
/*									*/
/************************************************************************/

/************************************************************************/
/*  Board Support:							*/
/*									*/
/*  chipKit Uno with Pmod Shield:     Header JC	                	*/
/*   (Note: To use this, download and deploy the Board Variant for      */
/*          chipKIT UNO32 W/Pmod Shield from the Digilent website)      */              
/*  Cerebot Mx3ck:                    Header JE                         */
/*  Cerebot Mx4ck:                    Header JB                         */
/*                   (make sure JP3 is on RB15)                         */
/*  Cerebot Mx7ck:                    Header JD                         */
/************************************************************************/

#include <DSPI.h>
#include <Nav.h>



#define DEBOUNCE_MIN_TIME  10 // debounce minimum time interval is 10 ms
#define DEBOUNCE_NO  5
bool      fBtn1Process;
bool      fBtn2Process;
/*
Button is a class that provides information about the debounced buttons and the changes in their states. 
*/
class Button {
private:
        uint8_t	m_bPinNo;   // pin number corresponding to the used on-board button
        uint8_t	m_stBtn;    // status of the button (pressed or released)
		uint8_t	m_stPrev;   // previous read state of the button
		uint8_t	m_cst;	    // counter for number of consecutive readings of the same button state
        uint32_t m_dwLastTime;  // timestamp of the last state check
        uint8_t m_fChangedState;  // flag indicating that a state change has just been detected 
 
public:
/* ------------------------------------------------------------------- */
/** Button(byte bPinNo)
**
**	Parameters:
**		bPinNo - The pin corresponding to the used button
**
**	Description:
**		Class constructor, performs initialization tasks
**
-----------------------------------------------------------------------*/
Button(byte bPinNo)
{
  m_bPinNo = bPinNo;
  pinMode(m_bPinNo, INPUT);
  digitalWrite(m_bPinNo, HIGH);
  m_stBtn = m_stPrev = digitalRead(m_bPinNo);   // read the button
}

/* ------------------------------------------------------------------- */
/**         Pressed()
**
**	Return value:
**		TRUE - if the button is in the "Pressed" state
**		FALSE - if the button is in the "Un-pressed" state
**
**	Description:
**	  The function returns the current state of the button, regardless of the moment when this state was set
**
-----------------------------------------------------------------------*/
bool Pressed()
{
  return m_stBtn == 1;
}

/* ------------------------------------------------------------------- */
/**         JustPressed()
**
**	Return value:
**		TRUE - if the button was just pressed
**		FALSE - otherwise
**
**	Description:
**	  The function returns TRUE only once, corresponding to the first CheckState() when the button was detected as being pressed.
**        It is suitable to implement actions based on the button pressing action.
**
-----------------------------------------------------------------------*/
bool JustPressed()
{
  return (m_stBtn == 1 && m_fChangedState == 1);
}

/* ------------------------------------------------------------------- */
/**         JustReleased()
**
**	Return value:
**		TRUE - if the button was just released
**		FALSE - otherwise
**
**	Description:
**	  The function returns TRUE only once, corresponding to first CheckState() when the button was detected as released.
**        It is suitable to implement actions based on the button releasing action.
**
-----------------------------------------------------------------------*/
bool JustReleased()
{
  return (m_stBtn == 0 && m_fChangedState == 1);
}

/* ------------------------------------------------------------------- */
/**         CheckState()
**
**
**	Description:
**	  This function is periodically called, in order to determine the internal state of the button.
**        If a button is detected as being in the same state for more than DEBOUNCE_NO times, it is marked to be in that state.
**        Checking a button state is done only if the minimum time (DEBOUNCE_MIN_TIME) has elapsed since the last check.
**        The function sets the flag for changed state immediately after the state is changed, and clears it in all the other situations.
**
-----------------------------------------------------------------------*/
void CheckState()
{
  uint8_t stCur;  // current read state of the button
  stCur = digitalRead(m_bPinNo);   // read the button
  uint32_t dwNow = millis();
  m_fChangedState = 0;   
  if(dwNow < m_dwLastTime)
  {
    // millis - rolled over 
    m_dwLastTime = dwNow; // update the check time
  }
  if(dwNow > m_dwLastTime + DEBOUNCE_MIN_TIME)
  {
    // time to debounce
    if((stCur == m_stPrev) && (stCur != m_stBtn))
    {
      if(++m_cst >= DEBOUNCE_NO)
      {
        m_stBtn = stCur;
        m_fChangedState = 1;
        m_cst = 0;
      }
    }
    else
    {
        m_cst = 0;  // restart counter
    }
    m_stPrev = stCur;
    m_dwLastTime = dwNow; 
  }        
}
};

//Bounce class objects instantiation
Button  btn1 = Button(PIN_BTN1);
Button  btn2 = Button(PIN_BTN2);
Nav navObj;
int16_t magMin[3] = {0, 0, 0};
int16_t magMax[3] = {0, 0, 0};
char strTemp[50];
//function that detects if a button or both have been pressed
bool WaitUntilBtnPressed(boolean *pfBtn1Process, boolean *pfBtn2Process);
void setup()
{
	//setup the SPI communications and basic device initializations
	navObj.begin();
	pinMode(PIN_BTN1, INPUT);
	Serial.begin(9600);
	int16_t mBiasRaw[3];
	float mBias[3];
	int j;
	//generic function to perform basic initializations of all devices: Accel/Gyro, Magnetometer, Altimeter
	navObj.Init();
	Serial.println("wait for button to start calibration");
	fBtn1Process = false;
    fBtn2Process = false;
	WaitUntilBtnPressed(&fBtn1Process, &fBtn2Process);
	if(fBtn1Process == true){
		do{ 
		// if only Btn1 is pressed
			Serial.println("Calibration start");
			for (j = 0; j < 3; j++)
			{
				magOffset(j, 0);
			}
			calibrateMag(1);
			//check if button 2 is pressed
			btn2.CheckState();
			if (btn2.JustPressed()) {
				fBtn2Process = true;
			}
		}
		while(fBtn2Process ==false);	
        
	}
	Serial.println("Calibration end");
	sprintf(strTemp, "XMax:%6d dec, YMax:%6d dec, ZMax:%6d dec", magMax[0], magMax[1], magMax[2]); 
	Serial.println(strTemp);
	sprintf(strTemp, "XMin:%6d dec, YMin:%6d dec, ZMin:%6d dec", magMin[0], magMin[1], magMin[2]); 
	Serial.println(strTemp);
	for (j = 0; j < 3; j++)
	{
		mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		mBias[j] = navObj.ConvertReadingToValueGauss(mBiasRaw[j]);
		magOffset(j, mBiasRaw[j]);
	}
	sprintf(strTemp, "XBias:%6d dec, YBias:%6d dec, ZBias:%6d dec", mBiasRaw[0], mBiasRaw[1], mBiasRaw[2]); 
	Serial.println(strTemp);
	delay(10);
}

void loop()
{
	char strTemp[50];
	int16_t MagX, MagY, MagZ, aMagX[512],aMagY[512],aMagZ[512];
	float gMagX, gMagY, gMagZ, agMagX[512], agMagY[512], agMagZ[512];
	char strM[100];
	int i,k;
	float D, sum, sumy;
	for (int i=0;i<512;i++)
	{
		navObj.ReadMag(MagX, MagY, MagZ);
		aMagX[i] = MagX;
		aMagY[i] = MagY;
		aMagZ[i] = MagZ;
		agMagX[i] = navObj.ConvertReadingToValueGauss(aMagX[i]);
		agMagY[i] = navObj.ConvertReadingToValueGauss(aMagY[i]);
		agMagZ[i] = navObj.ConvertReadingToValueGauss(aMagZ[i]);
	}
		sprintf(strTemp, "XM:%6.3f Gauss float, YM:%6.3f Gauss float, ZM:%6.3f Gauss float", gMagX, gMagY, gMagZ); 
		Serial.println(strTemp);
		sprintf(strTemp, "XM:%6X Gauss hex, YM:%6X Gauss hex, ZM:%6X Gauss hex", MagX, MagY, MagZ); 
		Serial.println(strTemp);
		sprintf(strTemp, "XM:%6d Gauss dec, YM:%6d Gauss dec, ZM:%6d Gauss dec", MagX, MagY, MagZ); 
		Serial.println(strTemp);
		
		for(int k=0;k<128;k++)
		{
			sum = aMagX[k]+aMagX[k+1];
		}
		float newMagX = sum/128;
		for(int k=0;k<128;k++)
		{
			sumy = (aMagY[k]+aMagY[k+1]);
		}
		float newMagY = sumy/128;
		sprintf(strTemp, "media: %4.2f", newMagX);
		Serial.println(strTemp);
		sprintf(strTemp, "media: %4.2f", newMagY);
		Serial.println(strTemp);
		if (newMagX == 0)
			D = (newMagY < 0) ? 90 : 0;
		else 
			D = atan2(newMagY,newMagX)*180/PI;
		if (D > 360)
		{
			D -= 360;
		}	
		else if (D<0)
		{
			D+=360;
		}
		sprintf(strTemp, "Heading in Degrees RAW: %4.2f", D );
		Serial.println(strTemp);
		sprintf(strTemp, "Total Magnetic Field: %4.2f", navObj.coord.R );
		Serial.println(strTemp);	
	if (( 337.25 < D ) || ( 22.5 > D )) {
		Serial.println("Direction: North");
		}
	else if ( 292.5 < D ) {
		Serial.println("Direction: North-West");
		}
	else if ( 247.5 < D ) {
		Serial.println("Direction: West");
		}
	else if ( 202.5 < D ) {
		Serial.println("Direction: South-West");
		}
	else if ( 157.5 < D ) {
		Serial.println("Direction: South");
		}
	else if ( 112.5 < D ) {
		Serial.println("Direction: South-East");
	}
	else if ( 67.5 < D ) {
		Serial.println("Direction: East");
	}
	else {
		Serial.println("Direction: North-East");
	}
	Serial.print("-----------------------------------------------");
	Serial.println();
	delay(500);	
}
void calibrateMag(bool loadIn)
	{
	int i, j;
	float mBias[3];
	int16_t MagX, MagY, MagZ; // The road warrior
	while (!(navObj.DataAvailableMAG(X_AXIS)&&navObj.DataAvailableMAG(Y_AXIS)&&navObj.DataAvailableMAG(Z_AXIS)))
		;
		navObj.ReadMag(MagX, MagY, MagZ);
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = MagX;		
		magTemp[1] = MagY;
		magTemp[2] = MagZ;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
}
void magOffset(uint8_t axis, int16_t offset)
{
	if (axis > 2)
	return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	navObj.WriteSPI(INST_MAG, OFFSET_X_REG_L_M + (2 * axis), lsb);
	navObj.WriteSPI(INST_MAG, OFFSET_X_REG_H_M + (2 * axis), msb);
}
/* ------------------------------------------------------------------- */
/** void  WaitUntilBtnPressed(boolean *pfBtn1Process, boolean *pfBtn2Process)
**
**	Parameters:
**		pfBtn1Process - true for button1 pressed
**		pfBtn2Process - true for button2 pressed
**
**	Return Value:
**		boolean
**
**	Errors:
**		none
**
**	Description:
**		This function returns the state of the two buttons, if they are pressed or not
**
-----------------------------------------------------------------------*/
bool WaitUntilBtnPressed(bool *pfBtn1Process, bool *pfBtn2Process)
{
    *pfBtn1Process = false;
    *pfBtn2Process = false;
    while ((*pfBtn1Process == false) && (*pfBtn2Process == false)) {
		//check the state of the buttons
        btn1.CheckState();
        btn2.CheckState();
        //if either one of them is pressed while the other is released, set the flags for buttons pressed
	if ((btn1.JustPressed() && btn2.Pressed())||
           (btn1.Pressed() && btn2.JustPressed())) 
	    {
	    *pfBtn1Process = true;
	    *pfBtn2Process = true;
  	}
	else{
			//if only button 1 is pressed
			if (btn1.JustPressed()) {
			*pfBtn1Process = true;
	    }
			else {
			//if only button 2 is pressed
			if (btn2.JustPressed()) {
				*pfBtn2Process = true;
			}
			}	
	}
    }
}