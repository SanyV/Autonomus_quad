/*

Sany V.

*/

///////////////////////////////////////////////////////////////////////////////

#include "Config.h"
#include "stdbool.h"
#include "cli.h"
#include "ublox.h"
#include "UKF_lib.h"
#include "PWM_PPM.h"


///////////////////////////////////////////////////////////////////////////////
//Function prototype
void cliTaskCode (void *argument);                                 // thread function
TaskHandle_t cli_handle;
uint16_t (*cliPortNumCharsAvailable)(void);
uint8_t (*cliPortAvailable)(void);
void     (*cliPortClearBuffer)(void);
uint8_t  (*cliPortRead)(void);
void     (*cliPortPrint)(char *str);
void     (*cliPortPrintCh)(uint8_t ch);
void     (*cliPortPrintF)(const char * fmt, ...);
void     (*cliPortPrintBinary)(uint8_t *buf, uint16_t length);
float stringToFloat(const char *p);
void  PrintState(void);
///////////////////////////////////////
extern uint16_t ppm1_buf[10];
///////////////////////////////////////
const char cliHome[] = {0x1b, 0x5b, 0x48};
const char cliClear[] = {0x1b, 0x5b, 0x32, 0x4a};
const char cliClearEOL[] = {0x1b, 0x5b, 0x4b, 0x00};
const char cliClearEOS[] = {0x1b, 0x5b, 0x4a, 0x00};
#define C_RED     "\x1b[31m"
#define C_GREEN   "\x1b[32m"
#define C_YELLOW  "\x1b[33m"
#define C_BLUE    "\x1b[34m"
#define C_MAGENTA "\x1b[35m"
#define C_CYAN    "\x1b[36m"
#define C_RESET   "\x1b[0m"
///////////////////////////////////////
//external variable for printing
extern uint16_t M1_lev_front, M2_prav_back, M3_prav_front, M4_lev_back;
extern float GPS_hAccf;
extern data_ukf DataUKF ;
extern gps_t gps;
extern uint32_t deltaTimeCOMM1, executionTimeCOMM1, previousCOMM1Time;
extern uint32_t deltaTimeCOMM2, executionTimeCOMM2, previousCOMM2Time;
extern uint32_t deltaTimeMPU,  	executionTimeMPU,  	previousMPUTime;
extern uint32_t deltaTimeGPS,  	executionTimeGPS,  	previousGPSTime;
extern uint32_t deltaTimeLED,   executionTimeLED,   previousLEDTime;
extern uint32_t deltaTimeUKF,   executionTimeUKF,   previousUKFTime;
extern uint32_t deltaTimeINS,   executionTimeINS,   previousINSTime;
extern uint32_t deltaTime1Hz,   executionTime1Hz,   previous1HzTime;
extern uint32_t deltaTimeBARO,  executionTimeBARO , previousBAROTime;
extern float dtCOMM1, dtCOMM2, dtMPU, dtGPS, dtLED, dtUKF, dtINS, dtBARO;
float dtCLI;
extern uint32_t deltaTimeMAG,   executionTimeMAG ,   previousMAGTime;
extern float dtMAG;
extern uint32_t deltaTimeUKF,   executionTimeUKF ,   previousUKFTime;
extern float dtUKF;
extern uint32_t deltaTimeGPS,   executionTimeGPS ,   previousGPSTime;
extern uint32_t deltaTimeGPScalib,   executionTimeGPScalib ,   previousGPSTimecalib;
extern uint32_t timerValue1;
extern float q0,q1,q2,q3;
//Полезные данные
extern int16_t mx, my, mz;
extern float mxf, myf, mzf;
extern float Temperature, Altitude, Pressure, altitude;
//данные с инерциальных датчиков
extern int16_t ax, ay, az, gx, gy, gz;//сырые даные в 16 разрядах
extern float temp_gyro;//температура датчика
extern float axf, ayf, azf, gxf, gyf, gzf;//правильные данные с датчиков в g и рад/сек

///////////////////////////////////////
//internal variable for cli
uint8_t cliBusy = false;
static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;
uint32_t deltaTimeCLI,  	executionTimeCLI,  	previousCLITime;
uint8_t sym;
uint8_t gpsDataType = 0;
float fQ[4];
///////////////////////////////////////////////////

#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')

float stringToFloat(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.

    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.

    sign = 1.0;
    if (*p == '-') {
        sign = -1.0;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.

    value = 0.0;
    while (valid_digit(*p)) {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.

    if (*p == '.') {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.

    scale = 1.0;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.

        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.

        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.

        while (expon >= 50) { scale *= 1E50; expon -= 50; }
        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
        while (expon >   0) { scale *= 10.0; expon -=  1; }
    }

    // Return signed and scaled floating point result.

    return sign * (frac ? (value / scale) : (value * scale));
}

///////////////////////////////////////////////////
// Quanterion output for visualisation
/////////////////////////////////////////////
void serialFloatPrint(float f) {
  uint8_t * b = (uint8_t *) &f;
  for(int i=0; i<4; i++) {

    uint8_t b1 = (b[i] >> 4) & 0x0f;
    uint8_t b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    cliPortPrintCh(c1);
    cliPortPrintCh(c2);
  }
}

void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    cliPortPrint(",");
  }
}

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

char *readStringCLI(char *data, uint8_t length)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;

    do
    {
        if (cliPortAvailable() == false)
        {
        	vTaskDelay(10);
            timeout++;
        }
        else
        {
            data[index] = cliPortRead();
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < length));

    data[index] = '\0';

    return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
    uint8_t index    = 0;
    uint8_t timeout  = 0;
    char    data[13] = "";

    cliPortPrint("_");
    do
    {
        if (cliPortAvailable() == false)
        {
        	vTaskDelay(100);
            timeout++;
        }
        else
        {
            data[index] = cliPortRead();
            cliPortPrintCh(data[index]);
            timeout = 0;
            index++;
        }
    }
    while ((index == 0 || data[index-1] != ';') && (timeout < 25) && (index < sizeof(data)-1));

    data[index] = '\0';

    return stringToFloat(data);
}

///
///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	uint8_t  index;
//	char mvlkToggleString[5] = { 0, 0, 0, 0, 0 };

	if (cliPortAvailable())
	{
		cliQuery = cliPortRead() ;
		validCliCommand = true;
	}

	//validCliCommand = false;

    if (cliQuery != '#')
    {
        switch (cliQuery)
        {

			case 'i': // 500 hz Gyros
				cliPortPrintF("Accel raw: %5d, %5d, %5d\r\n", ax,
												   ay,
												   az);

				validCliCommand = false;
				break;

			case 'b': // 500 hz Gyros
				cliPortPrintF("Motor raw: %5d, %5d, %5d, %5d\r\n",M1_lev_front,
						M2_prav_back,
						M3_prav_front,
						M4_lev_back);

				validCliCommand = false;
				break;
			/////////////
		   case 'j': // 500 hz Gyros
			   cliPortPrintF("Gyro raw: %5d, %5d, %5d\r\n",		gx,
														gy,
														gz);
				validCliCommand = false;
				break;
				
			  case 'k': // 500 hz Gyros
				cliPortPrintF("Gyro %9.4f, %9.4f, %9.4f \r\n", 	  gxf * RAD2DEG,
															  gyf * RAD2DEG,
															  gzf * RAD2DEG);
				validCliCommand = false;
				break;
		
				case 'l': // 100 Hz Accels
				cliPortPrintF("Accel %9.4f, %9.4f, %9.4f\r\n", axf,
													     ayf,
													     azf);
				validCliCommand = false;
				break;
			///////////////////////////////
				case 'v': // 100 Hz Accels
    cliPortPrintF("\r\nHCLK->   %6.2f MHz\r\n", (float)  (HAL_RCC_GetHCLKFreq()  / 1000000.0f));
    cliPortPrintF(  "PCLK1->  %6.2f MHz\r\n",   (float)  (HAL_RCC_GetPCLK1Freq()  / 1000000.0f));
    cliPortPrintF(  "PCLK2->  %6.2f MHz\r\n",   (float)  (HAL_RCC_GetPCLK2Freq()  / 1000000.0f));
    cliPortPrintF(  "SYSCLK-> %6.2f MHz\r\n",   (float)  (HAL_RCC_GetSysClockFreq() / 1000000.0f));
				
				cliQuery = 'x';
				validCliCommand = false;
				break;

/*
				case 'c': // 100 Hz Accels
				    cliPortPrintF("\r\nFreeHeapSize =  %6d \r\n", xPortGetFreeHeapSize());
				    cliPortPrintF("MinimumFreeHeapSize =  %6d \r\n", xPortGetMinimumEverFreeHeapSize());

				    cliQuery = 'x';
				validCliCommand = false;
				break;
*/
						///////////////////////////////
				case 'm': // Mag
					cliPortPrintF("%4d, %4d, %4d\r\n",		mx,
															my,
															mz);
					validCliCommand = false;
					break;

				case 't': // Telemetry Start
					cliPortPrint(cliClear);
					cliPortPrint(cliHome);
					cliQuery = 'w';
					validCliCommand = true;
					break;

				case 'w': // Mag
					cliPortPrint(cliHome);
					cliPortPrint ("	  Gx	   Gy	   Gz	   Ax	   Ay	   Az	  Mx	  My	  Mz\r\n");
					cliPortPrintF("RAW:	%5d,	%5d,	%5d,	%5d,	%5d,	%5d,	%5d,	%5d,	%5d    \r\n",gx, gy, gz, ax, ay, az, mx, my, mz);
					cliPortPrintF("Nor:	%6.2f,	%6.2f,	%6.2f,	%6.2f,	%6.2f,	%6.2f,	%6.2f,	%6.2f, %6.2f     \r\n",gxf,gyf,gzf,axf,ayf,azf,mxf,myf,mzf);
					cliPortPrint ("Att:	ROLL	  PITCH	     YAW\r\n");
					cliPortPrintF("	%6.2f, %6.2f, %6.2f \r\n",navUkfData.roll,navUkfData.pitch,navUkfData.yaw);
					cliPortPrint ("Alt:	Baro	True	Pres	GPShacc	GPSpdop	GPSsatnum\r\n");
					cliPortPrintF("	%6.2f, %6.2f, %6.2f, %6.2f, %3d, %2d\r\n",DataUKF.baro_alt,DataUKF.tru_pos_z,DataUKF.pres_alt,GPS_hAccf,gps.pDop,GPS_satellits);
					cliPortPrint ("MOTOR:	M1	M2	M3	M4\r\n");
					cliPortPrintF("	%5d, %5d, %5d, %5d\r\n",M1_lev_front,M2_prav_back,M3_prav_front,M4_lev_back);
					cliPortPrint ("SystemState:\r\n");
					PrintState();
					validCliCommand = true;
					break;

				case 'c': // 500 hz Gyros

			//		cliPortPrintF("Temp=%4.2f,  Alt=%6.2f, Pres=%9.2f, Absolute Alt=%6.2f, UKF Alt=%6.2f \r\n", 	  Temperature, Altitude, Pressure, altitude, DataUKF.pres_alt);

					cliPortPrintF("Baro Alt=%6.2f, Trus=%9.2f, Pres ALt=%6.2f, GPS_hAccf=%6.2f \r\n",
							DataUKF.baro_alt,
						    DataUKF.tru_pos_z,
						    DataUKF.pres_alt,
							GPS_hAccf);
							validCliCommand = false;
							break;
			///////////////////////////////

			case 'n': // 10 Hz Mag Data

				cliPortPrintF("%7.2f, %7.2f, %7.2f\r\n", mxf,
													     myf,
													     mzf);
				validCliCommand = false;


				break;				
				///////////////////////////////
		case 'q': // Not Used
		serialFloatPrint(q0);
        cliPortPrint(",");
	        serialFloatPrint(q1);
	        cliPortPrint(",");
	        serialFloatPrint(q2);
	        cliPortPrint(",");
	        serialFloatPrint(q3);
	        cliPortPrint(",");
/*
		serialFloatPrint(fQ[0]);
	    cliPortPrint(",");
   	    serialFloatPrint(fQ[1]);
   	    cliPortPrint(",");
   	    serialFloatPrint(fQ[2]);
   	    cliPortPrint(",");
 	    serialFloatPrint(fQ[3]);
 	    cliPortPrint(",");
*/
		cliPortPrint("\r\n");
		validCliCommand = false;
		break;

		///////////////////////////////
				case 'Q': // GPS Data Selection
					gpsDataType = (uint8_t)readFloatCLI();

					cliPortPrint("\r\n");

					cliQuery = 'n';
					validCliCommand = false;
					break;
					///////////////////////////////
/*
///////////////////////////////
		case 'q': // Not Used
		serialFloatPrint(q0);
        cliPortPrint(",");
	        serialFloatPrint(q1);
	        cliPortPrint(",");
	        serialFloatPrint(q2);
	        cliPortPrint(",");
	        serialFloatPrint(q3);
	        cliPortPrint(",");
		
		serialFloatPrint(fQ[0]);
	    cliPortPrint(",");
   	    serialFloatPrint(fQ[1]);
   	    cliPortPrint(",");
   	    serialFloatPrint(fQ[2]);
   	    cliPortPrint(",");
 	    serialFloatPrint(fQ[3]);
 	    cliPortPrint(",");

		cliPortPrint("\r\n");
		validCliCommand = false;
		break;
*/
			case 'e': // Loop Delta Times

	/*			cliPortPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n\r", deltaTimeCOMM1,
																			  deltaTimeCOMM2,
																			  deltaTimeMPU,
																			  deltaTimeINS,
																			  deltaTimeGPS,
																			  deltaTimeCLI);
*/
				cliPortPrint(cliClear);
				cliPortPrint(cliHome);
				cliQuery = 'f';
				validCliCommand = true;
/*
				cliPortPrintF("%9.4f, %9.4f, %9.4f, %10.6f, %10.6f, %9.4f, %9.4f, %9.4f, %9.4f\n\r",
																			  dtUKF,
																			  dtCOMM1,
																			  dtCOMM2,
																			  dtMPU,
																			  dtINS,
																			  dtGPS,
																			  dtBARO,
																			  dtMAG,
																			  dtCLI);

*/
				validCliCommand = false;
				break;

			///////////////////////////////

			case 'f': // Loop Execution Times
				cliPortPrint(cliHome);
				cliPortPrint ("	  	UKF	   COM1	   	COM2	   MPU	   INS	   GPS	  BARO	  MAG	  CLI\r\n");
				cliPortPrintF("Exec Time:	%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n\r",
																			  executionTimeUKF,
																			  executionTimeCOMM1,
																			  executionTimeCOMM2,
																			  executionTimeMPU,
																			  executionTimeINS,
																			  executionTimeGPS,
																			  executionTimeBARO,
																			  executionTimeMAG,
																			  executionTimeCLI);

				cliPortPrintF("Delt Time:	%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n\r",
																			  dtUKF,
																			  dtCOMM1,
																			  dtCOMM2,
																			  dtMPU,
																			  dtINS,
																			  dtGPS,
																			  dtBARO,
																			  dtMAG,
																			  dtCLI);
				cliQuery = 'f';
				validCliCommand = true;
				break;

			case 'a': //
				cliPortPrintF("GPS calib ticks= %12d\n\r",
																			  deltaTimeGPScalib);

				validCliCommand = false;
				break;
/*
			///////////////////////////////
			case 'i': // 500 hz Gyros
				cliPortPrintF("%5d, %5d, %5d\r\n", rawAccel[XAXIS].value,
												   rawAccel[YAXIS].value,
												   rawAccel[ZAXIS].value);

				validCliCommand = false;
				break;
			///////////////////////////////
			case 'j': // 500 hz Gyros
				cliPortPrintF("%5d, %5d, %5d\r\n",		rawGyro[ROLL ].value,
														rawGyro[PITCH].value,
														rawGyro[YAW  ].value);
				validCliCommand = false;
				break;
			case 'k': // 500 hz Gyros
				cliPortPrintF("%9.4f, %9.4f, %9.4f \r\n", 	  sensors.gyro500Hz[ROLL ] * R2D,
															  sensors.gyro500Hz[PITCH] * R2D,
															  sensors.gyro500Hz[YAW  ] * R2D);
				validCliCommand = false;
				break;
			case 'l': // 100 Hz Accels
				cliPortPrintF("%9.4f, %9.4f, %9.4f\r\n", sensors.accel500Hz[XAXIS],
													     sensors.accel500Hz[YAXIS],
													     sensors.accel500Hz[ZAXIS]);
				validCliCommand = false;
				break;


				case 'z': // 500 hz Gyros

					cliPortPrint(cliHome);
					cliPortPrint(cliClear);
					cliPortPrint("\r");
					vTaskList(Text_buf);
					//vTaskGetRunTimeStats(Text_buf);
					cliPortPrint(Text_buf);
				//	cliPortPrintF("dt500=%9.7f, Timer= %7d, Timer1=%7d, Gyrobias=%9.4f \r\n", 	  dt500Hz,timerValue, timerValue1,gyroRTBias[0]);
							validCliCommand = false;
							break;
							*/
				case 'r': // GPS Data
					switch (gpsDataType)
					{
						///////////////////////

						case 0:
							cliPortPrintF("%12ld, %12ld, %12ld, %12ld, %12ld, %12ld, %4d, %4d", gps.latitude,
																								  gps.longitude,
																								  gps.hMSL,
																								  gps.velN,
																								  gps.velE,
																								  gps.velD,
																								  gps.fix,
																								  gps.numSats
																								  );
							cliPortPrint("\r\n");
							break;

						case 1:
							cliPortPrintF("%3d: ", gps.numCh);

							for (index = 0; index < gps.numCh; index++)
								cliPortPrintF("%3d  ", gps.chn[index]);

							cliPortPrint("\r\n");

							break;

						///////////////////////

						case 2:
							cliPortPrintF("%3d: ", gps.numCh);

							for (index = 0; index < gps.numCh; index++)
								cliPortPrintF("%3d  ", gps.svid[index]);

							cliPortPrint("\r\n");

							break;

						///////////////////////

						case 3:
							cliPortPrintF("%3d: ", gps.numCh);

							for (index = 0; index < gps.numCh; index++)
								cliPortPrintF("%3d  ", gps.cno[index]);

							cliPortPrint("\r\n");

							break;

						///////////////////////
					}
					validCliCommand = false;
					break;
					///////////////////////////////
			case 's': // Mag
				cliPortPrintF("%5d, %5d, %5d, %5d, %5d, %5d,%5d, %5d, %5d, %5d, \r\n",		ppm1_buf[0],
						ppm1_buf[1],
						ppm1_buf[2],
						ppm1_buf[3],
						ppm1_buf[4],
						ppm1_buf[5],
						ppm1_buf[6],
						ppm1_buf[7],
						ppm1_buf[8],
						ppm1_buf[9]);

					validCliCommand = false;
					break;

			case 'p': // Mode state

				cliPortPrint("Flight Mode=");

				if ( Mode & 1 ) cliPortPrint("ESC_CALIBRATION");
				else 			cliPortPrint("| STAB");
				if ( Mode & 2 ) cliPortPrint("| ALT_HOLD");
				else if ( Mode & 3 ) cliPortPrint("| AUTO");
				if ( Mode & 4 ) cliPortPrint("| HEADFREE");
				if ( Mode & 8 ) cliPortPrint("| POS_HOLD");

				cliPortPrint(",  Supervisor state=");
				if ( supervisorState == 0 ) cliPortPrint("STATE_INITIALIZING");
				if ( supervisorState & 1 ) cliPortPrint("| STATE_CALIBRATION");
				if ( supervisorState & 2 ) cliPortPrint("| STATE_ARMED");
				if ( supervisorState & 4 ) cliPortPrint("| STATE_DISARMED");
				if ( supervisorState & 8 ) cliPortPrint("| STATE_FLYING");
				if ( supervisorState & 16 ) cliPortPrint("| STATE_RADIO_LOSS1");
				if ( supervisorState & 32 ) cliPortPrint("| GPS_GLITCH");
				if ( supervisorState & 64 ) cliPortPrint("| STATE_LOW_BATTERY");
				if ( supervisorState & 128 ) cliPortPrint("| ERR_SENSOR");

				cliPortPrint(",  UKF state=");
				if ( StateNavUKF == 0 ) cliPortPrint("UKF_NORM");
				if ( StateNavUKF & 1 ) cliPortPrint("| UKF_BAD_GPS");
				if ( StateNavUKF & 2 ) cliPortPrint("| UKF_GLITCH_GPS");
				if ( StateNavUKF & 4 ) cliPortPrint("| UKF_NAN");

				cliPortPrint("\r\n");

				validCliCommand = false;
				break;

				case '?': // Command Summary
				cliBusy = true;

				cliPortPrint("\r\n");
				cliPortPrint("'a' Rate PIDs                              'A' Set Roll Rate PID Data   AP;I;D;N\r\n");
				cliPortPrint("'b' Attitude PIDs                          'B' Set Pitch Rate PID Data  BP;I;D;N\r\n");
				cliPortPrint("'c' Velocity PIDs                          'C' Set Yaw Rate PID Data    CP;I;D;N\r\n");
				cliPortPrint("'d' Position PIDs                          'D' Set Roll Att PID Data    DP;I;D;N\r\n");
				cliPortPrint("'e' Loop Delta Times                       'E' Set Pitch Att PID Data   EP;I;D;N\r\n");
				cliPortPrint("'f' Loop Execution Times                   'F' Set Hdg Hold PID Data    FP;I;D;N\r\n");
				cliPortPrint("'g' 500 Hz Accels                          'G' Set nDot PID Data        GP;I;D;N\r\n");
				cliPortPrint("'h' 100 Hz Earth Axis Accels               'H' Set eDot PID Data        HP;I;D;N\r\n");
				cliPortPrint("'i' 500 Hz Gyros                           'I' Set hDot PID Data        IP;I;D;N\r\n");
				cliPortPrint("'j' 10 hz Mag Data                         'J' Set n PID Data           JP;I;D;N\r\n");
				cliPortPrint("'k' Vertical Axis Variable                 'K' Set e PID Data           KP;I;D;N\r\n");
				cliPortPrint("'l' Attitudes                              'L' Set h PID Data           LP;I;D;N\r\n");
				cliPortPrint("\r\n");

				cliPortPrint("Press space bar for more, or enter a command....\r\n");

				while (cliPortAvailable() == false);

				cliQuery = cliPortRead();

				if (cliQuery != ' ')
				{
					validCliCommand = true;
					cliBusy = false;
					return;
				}

				cliPortPrint("\r\n");
				cliPortPrint("'m' Axis PIDs                              'M' MAX7456 CLI\r\n");
				cliPortPrint("'n' GPS Data                               'N' Mixer CLI\r\n");
				cliPortPrint("'o' Battery Voltage                        'O' Receiver CLI\r\n");
				cliPortPrint("'p' Not Used                               'P' Sensor CLI\r\n");
				cliPortPrint("'q' Not Used                               'Q' GPS Data Selection\r\n");
				cliPortPrint("'r' Mode States                            'R' Reset and Enter Bootloader\r\n");
				cliPortPrint("'s' Raw Receiver Commands                  'S' Reset\r\n");
				cliPortPrint("'t' Processed Receiver Commands            'T' Telemetry CLI\r\n");
				cliPortPrint("'u' Command In Detent Discretes            'U' EEPROM CLI\r\n");
				cliPortPrint("'v' Motor PWM Outputs                      'V' Reset EEPROM Parameters\r\n");
				cliPortPrint("'w' Servo PWM Outputs                      'W' Write EEPROM Parameters\r\n");
				cliPortPrint("'x' Terminate Serial Communication         'X' Not Used\r\n");
				cliPortPrint("\r\n");

				cliPortPrint("Press space bar for more, or enter a command....\r\n");

				while (cliPortAvailable() == false);

				cliQuery = cliPortRead();

				if (cliQuery != ' ')
				{
					validCliCommand = true;
					cliBusy = false;
					return;
				}

				cliPortPrint("\r\n");
				cliPortPrint("'y' ESC Calibration/Motor Verification     'Y' ADC CLI\r\n");
				cliPortPrint("'z' ADC Values                             'Z' WMM Test\r\n");
				cliPortPrint("                                           '?' Command Summary\r\n");
				cliPortPrint("\r\n");

				cliQuery = 'x';
				cliBusy = false;
				break;

				///////////////////////////////
		}
    }
}

void  PrintState(void) {
	cliPortPrint("Flight Mode=");

	if ( Mode & 1 ) cliPortPrint("ESC_CALIBRATION");
	else 			cliPortPrint("| STAB");
	if ( Mode & 2 ) cliPortPrint("| ALT_HOLD");
	else if ( Mode & 3 ) cliPortPrint("| AUTO");
	if ( Mode & 4 ) cliPortPrint("| HEADFREE");
	if ( Mode & 8 ) cliPortPrint("| POS_HOLD");

	cliPortPrint(",  Supervisor state=");
	if ( supervisorState == 0 ) cliPortPrint("STATE_INITIALIZING");
	if ( supervisorState & 1 ) cliPortPrint("| STATE_CALIBRATION");
	if ( supervisorState & 2 ) cliPortPrint("| STATE_DISARMED");
	if ( supervisorState & 4 ) cliPortPrint("| STATE_ARMED");
	if ( supervisorState & 8 ) cliPortPrint("| STATE_FLYING");
	if ( supervisorState & 16 ) cliPortPrint("| STATE_RADIO_LOSS1");
	if ( supervisorState & 32 ) cliPortPrint("| GPS_GLITCH");
	if ( supervisorState & 64 ) cliPortPrint("| STATE_LOW_BATTERY");
	if ( supervisorState & 128 ) cliPortPrint("| ERR_SENSOR");

	cliPortPrint(",  UKF state=");
	if ( StateNavUKF == 0 ) cliPortPrint("UKF_NORM");
	if ( StateNavUKF & 1 ) cliPortPrint("| UKF_BAD_GPS");
	if ( StateNavUKF & 2 ) cliPortPrint("| UKF_GLITCH_GPS");
	if ( StateNavUKF & 4 ) cliPortPrint("| UKF_NAN");

	cliPortPrint("\r\n");
}
static volatile unsigned long cliloops;

void cliTaskCode(void *argument) {
static uint32_t currentTime;
while (1) {
	cliloops++;
   // if (!(cliloops % (unsigned int)CLI_RATE )) {
    currentTime       = TIM5->CNT;
    deltaTimeCLI    = currentTime - previousCLITime;
    previousCLITime = currentTime;
    dtCLI = (float)deltaTimeCLI * 0.0000001f;
	cliCom();
    // Calculate execution tick process time
    executionTimeCLI = TIM5->CNT - currentTime;
    vTaskDelay(25);  //25 normal

    //	}
}
}

int CliInit (void) {

	//создание задачи сериал порта
	cliPortPrintF("FreeHeapSize =  %6d, before Cli task \r\n", xPortGetFreeHeapSize());
	xTaskCreate(cliTaskCode,(char*)"CLI", 600, NULL, 4, cli_handle);
//	cliPortPrint("CliInit!\r\n");
	return(0);
}



///////////////////////////////////////////////////////////////////////////////
