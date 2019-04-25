/*
 2017 Alexander Valov ported from AQ32
*/

///////////////////////////////////////////////////////////////////////////////

#include "Config.h"
#include "ublox.h"

//#define AIRBORNE
#define AIRBORNE_QUAD

uint8_t  (*gpsPortAvailable)(void);
void     (*gpsPortClearBuffer)(void);
uint8_t  (*gpsPortRead)(void);
void     (*gpsPortPrint)(char *str);
void     (*gpsPortPrintCh)(uint8_t ch);
void     (*gpsPortPrintF)(const char * fmt, ...);
void     (*gpsPortPrintBinary)(uint8_t *buf, uint16_t length);

uint32_t deltaTimeGPS,  	executionTimeGPS,  	previousGPSTime;
float dtGPS;
//****************************************************//
//Для ОС
TaskHandle_t gps_handle;
uint32_t ovf_gps_stack;
SemaphoreHandle_t xGPS_Semaphore = NULL;
SemaphoreHandle_t xGPS_UKF_Mutex;
//****************************************************//

//****************************************************//
//Микросекунды получения данных, флаги появления данных
uint32_t GPS_Micros_Update;
uint8_t flag_start_home = 0, flag_get_pvt = 0;
//****************************************************//
//Вспомогательные данные
uint16_t GPS_year;
uint8_t GPS_month, GPS_day, GPS_hours, GPS_minutes, GPS_seconds;
uint8_t GPS_satellits;
uint8_t GPS_valid_invalid;
uint8_t GPS_fix_type;
//****************************************************//
//Данные не для расчетов, только для телемтрии
float GPS_LA, GPS_LO;
int32_t GPS_lon, GPS_lat;
uint32_t GPS_hAcc, GPS_vAcc, GPS_sAcc;
uint16_t GPS_vDop, GPS_nDop, GPS_eDop, GPS_tDop;
//****************************************************//


//*******************Главные данные для расчетов*******************************//
//Позиция в м, относительно старта
float GPS_X, GPS_Y;
float GPS_X_home, GPS_Y_home;
float GPS_alt;
float GPS_X_cmp, GPS_Y_cmp;
float GPS_alt_cmp;
//Скорости, курс
float GPS_Y_speed, GPS_X_speed, GPS_Z_speed;
float GPS_vel, GPS_course;
//Показатели качества
float GPS_hAccf = 99.99f, GPS_vAccf = 99.99f, GPS_sAccf = 99.99f;
float GPS_pDopf = 99.99f, GPS_eDopf = 99.99f, GPS_vDopf = 99.99f, GPS_nDopf = 99.99f, GPS_tDopf = 99.99f;
//****************************************************//

///////////////////////////////////////////////////////////////////////////////
// GPS Initialization Variables
///////////////////////////////////////////////////////////////////////////////

void     (*gpsPortClearBuffer)(void);

uint16_t (*gpsPortNumCharsAvailable)(void);

void     (*gpsPortPrintBinary)(uint8_t *buf, uint16_t length);

uint8_t  (*gpsPortRead)(void);

gps_t gps;

///////////////////////////////////////////////////////////////////////////////

uint32_t initBaudRates[5] = {9600,19200,38400,57600,115200};

///////////////////////////////////////

enum ubloxState { WAIT_SYNC1, WAIT_SYNC2, GET_CLASS, GET_ID, GET_LL, GET_LH, GET_DATA, GET_CKA, GET_CKB  } ubloxProcessDataState;

///////////////////////////////////////

static uint8_t ubloxPortConfig38p4[] = {0xB5,0x62,            // Header             Setup UBLOX Comm Port
                                        0x06,0x00,            // ID
                                        0x14,0x00,            // Length
                                        0x01,                 // Port ID
                                        0x00,                 // Reserved 0
                                        0x00,0x00,            // TX Ready
                                        0xD0,0x08,0x00,0x00,  // Mode
                                        0x00,0x96,0x00,0x00,  // Baud Rate
                                        0x03,0x00,            // In Proto Mask
                                        0x01,0x00,            // Out Proto Mask
                                        0x00,0x00,            // Reserved 4
                                        0x00,0x00,            // Reserved 5
                                        0x8D,0x64};           // CK_A, CK_B

///////////////////////////////////////

static uint8_t ubloxInitData[] = {0xB5,0x62,            // Header             Turn Off NMEA GGA Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x00,                 // Msg ID       GGA
                                  0x00,                 // Rate         0
                                  0xFA,0x0F,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA GLL Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x01,                 // Msg ID       GLL
                                  0x00,                 // Rate         0
                                  0xFB,0x11,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA GSA Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x02,                 // Msg ID       GSA
                                  0x00,                 // Rate         0
                                  0xFC,0x13,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA GSV Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x03,                 // Msg ID       GSV
                                  0x00,                 // Rate         0
                                  0xFD,0x15,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA RMC Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x04,                 // Msg ID       RMC
                                  0x00,                 // Rate         0
                                  0xFE,0x17,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn Off NMEA VTG Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0xF0,                 // Msg Class    NMEA
                                  0x05,                 // Msg ID       VTG
                                  0x00,                 // Rate         0
                                  0xFF,0x19,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-POSLLH Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x02,                 // Msg ID       POSLLH
                                  0x01,                 // Rate         1
                                  0x0E,0x47,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-STATUS Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x03,                 // Msg ID       STATUS
                                  0x01,                 // Rate         1
                                  0x0F,0x49,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-DOP Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x04,                 // Msg ID       DOP
                                  0x01,                 // Rate         1
                                  0x10,0x4B,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-SOL Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x06,                 // Msg ID       SOL
                                  0x01,                 // Rate         1
                                  0x12,0x4F,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-VELNED Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x12,                 // Msg ID       VELNED
                                  0x01,                 // Rate         1
                                  0x1E,0x67,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-TIMEUTC Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x21,                 // Msg ID       TIMEUTC
                                  0x01,                 // Rate         1
                                  0x2D,0x85,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Turn On UBLOX NAV-SVINFO Msg
                                  0x06,0x01,            // ID
                                  0x03,0x00,            // Length
                                  0x01,                 // Msg Class    NAV
                                  0x30,                 // Msg ID       SVINFO
                                  0x05,                 // Rate         5
                                  0x40,0xA7,            // CK_A, CK_B

                                  0xB5,0x62,            // Header             Setup SBAS Mode
                                  0x06,0x16,            // ID
                                  0x08,0x00,            // Length
                                  0x01,                 // Mode
                                  0x07,                 // Usage
                                  0x03,                 // Max SBAS
                                  0x00,                 // Scan Mode 2
                                  0x04,0xE0,0x04,0x00,  // Scan Mode 1
                                  0x17,0x8D,            // CK_A, CK_B
/*CFG-NAV5 - 06 24
			 24 00
			 FF FF
			 08
			 03
			 00 00 00 00
			 10 27 00 00
			 05
			 00
			 FA 00
			 FA 00
			 64 00
			 2C 01
			 00
			 3C
			 00 00 00 00
			 C8 00 00 00
			 00 00 00 00
*/
//nav_pvt
#ifdef AIRBORNE_QUAD
								  0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1,
#endif

#ifdef AIRBORNE_4G
    0xB5,0x62,            // Header             Set Navigation Engine Settings
    0x06,0x24,            // ID
    0x24,0x00,            // Length
    0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
    0x08,                 // DynModel             Airborne with < 4g Acceleration
    0x03,                 // FixMode              Auto 2D/3D
    0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
    0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
    0x05,                 // MinElev                5 deg
    0x00,                 // DrLimit                0 s
    0xFA,0x00,            // pDop                  25 scaled at 0.1
    0xFA,0x00,            // tDop                  25 scaled at 0.1
    0x64,0x00,            // pAcc                 100 m
    0x2C,0x01,            // tAcc                 300 m
    0x00,                 // StaticHoldThresh       0 m/s
    0x3C,                 // DgpsTimeOut           60 s
    0x00,0x00,0x00,0x00,  // Reserved2
    0x00,0x00,0x00,0x00,  // Reserved3
    0x00,0x00,0x00,0x00,  // Reserved4
    0x56,0x75,            // CK_A, CK_B
#endif

                                  #ifdef AIRBORNE
                                      0xB5,0x62,            // Header             Set Navigation Engine Settings
                                      0x06,0x24,            // ID
                                      0x24,0x00,            // Length
                                      0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
                                      0x07,                 // DynModel             Airborne with < 2g Acceleration
                                      0x03,                 // FixMode              Auto 2D/3D
                                      0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
                                      0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
                                      0x05,                 // MinElev                5 deg
                                      0x00,                 // DrLimit                0 s
                                      0xFA,0x00,            // pDop                  25 scaled at 0.1
                                      0xFA,0x00,            // tDop                  25 scaled at 0.1
                                      0x64,0x00,            // pAcc                 100 m
                                      0x2C,0x01,            // tAcc                 300 m
                                      0x00,                 // StaticHoldThresh       0 m/s
                                      0x3C,                 // DgpsTimeOut           60 s
                                      0x00,0x00,0x00,0x00,  // Reserved2
                                      0x00,0x00,0x00,0x00,  // Reserved3
                                      0x00,0x00,0x00,0x00,  // Reserved4
                                      0x56,0x75,            // CK_A, CK_B
                                  #endif

                                  #ifdef PEDESTRIAN
                                      0xB5,0x62,            // Header             Set Navigation Engine Settings
                                      0x06,0x24,            // ID
                                      0x24,0x00,            // Length
                                      0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
                                      0x03,                 // DynModel             Pedestrian
                                      0x03,                 // FixMode              Auto 2D/3D
                                      0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
                                      0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
                                      0x05,                 // MinElev                5 deg
                                      0x00,                 // DrLimit                0 s
                                      0xFA,0x00,            // pDop                  25 scaled at 0.1
                                      0xFA,0x00,            // tDop                  25 scaled at 0.1
                                      0x64,0x00,            // pAcc                 100 m
                                      0x2C,0x01,            // tAcc                 300 m
                                      0x00,                 // StaticHoldThresh       0 m/s
                                      0x3C,                 // DgpsTimeOut           60 s
                                      0x00,0x00,0x00,0x00,  // Reserved2
                                      0x00,0x00,0x00,0x00,  // Reserved3
                                      0x00,0x00,0x00,0x00,  // Reserved4
                                      0x52,0xED,            // CK_A, CK_B
                                  #endif

                                  #ifdef PORTABLE
                                      0xB5,0x62,            // Header             Set Navigation Engine Settings
                                      0x06,0x24,            // ID
                                      0x24,0x00,            // Length
                                      0x01,0x00,            // Mask                 Mask to apply only dynamic model setting
                                      0x00,                 // DynModel             Portable
                                      0x03,                 // FixMode              Auto 2D/3D
                                      0x00,0x00,0x00,0x00,  // FixedAlt               0 scaled at 0.01
                                      0x10,0x27,0x00,0x00,  // FixedAltVar            1 scaled at 0.0001
                                      0x05,                 // MinElev                5 deg
                                      0x00,                 // DrLimit                0 s
                                      0xFA,0x00,            // pDop                  25 scaled at 0.1
                                      0xFA,0x00,            // tDop                  25 scaled at 0.1
                                      0x64,0x00,            // pAcc                 100 m
                                      0x2C,0x01,            // tAcc                 300 m
                                      0x00,                 // StaticHoldThresh       0 m/s
                                      0x3C,                 // DgpsTimeOut           60 s
                                      0x00,0x00,0x00,0x00,  // Reserved2
                                      0x00,0x00,0x00,0x00,  // Reserved3
                                      0x00,0x00,0x00,0x00,  // Reserved4
                                      0x4F,0x87,            // CK_A, CK_B
                                  #endif

                                  0xB5,0x62,            // Header             Setup Meaurement Rates, Clock Reference
                                  0x06,0x08,            // ID
                                  0x06,0x00,            // Length
                                  0xC8,0x00,            // Measurement Rate     200 mSec (5 Hz)
                                  0x01,0x00,            // Navigation Rate      1 Measurement Cycle
                                  0x00,0x00,            // Time Reference       UTM Time
                                  0xDD,0x68};           // CK_A, CK_B


extern UART_HandleTypeDef huart2;
extern uint8_t  RX2Buffer[USART2_BUFFER_SIZE] __attribute__((aligned (32)));
///////////////////////////////////////////////////////////////////////////////
// Initialize UBLOX Receiver
///////////////////////////////////////////////////////////////////////////////

void initUBLOX(void)
{
    uint8_t i;
	static char uartState;

    ///////////////////////////////////

    // Going to send the GPS port configuration command at 5 different baud rates,
    // 9600, 19200, 38400, 57600, 115200.  If GPS is not at one of these rates,
    // port configuration will fail and communication between flight control board
    // and GPS board will not be possible.  GPS configuration command sets 38400
    // baud.  NEO-6M defaults to 9600 baud if config pins are left floating.

    for (i = 0; i < (sizeof(initBaudRates) / sizeof(initBaudRates[0])); i++)
    {
    	  huart2.Init.BaudRate = initBaudRates[i];

    	  if (HAL_UART_Init(&huart2) != HAL_OK)
    	  {
    	    _Error_Handler(__FILE__, __LINE__);
    	  }

//    	gpsPortPrintBinary(ubloxPortConfig38p4, sizeof(ubloxPortConfig38p4));
    	HAL_UART_Transmit(&huart2, ubloxPortConfig38p4, sizeof(ubloxPortConfig38p4), 0x1000);

		uartState = HAL_UART_GetState(&huart2);
		  while (uartState!= HAL_UART_STATE_READY)
		  {
			  uartState = HAL_UART_GetState(&huart2);
		  }
    	HAL_Delay(100);  // Delay so DMA buffer can be completely sent before trying next baud rate
    }

	  huart2.Init.BaudRate = 38400;

	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

    //////////////////////////////////
		uartState = HAL_UART_GetState(&huart2);
		  while (uartState!= HAL_UART_STATE_READY)
		  {
			  uartState = HAL_UART_GetState(&huart2);
		  }
	  HAL_UART_Transmit(&huart2, ubloxInitData, sizeof(ubloxInitData), 0x1000);  // Send UBLOX Initialization Data

    ///////////////////////////////////

    ubloxProcessDataState = WAIT_SYNC1;
  //  HAL_UART_Receive_DMA(&huart2, (uint8_t *)RX2Buffer, (uint16_t) USART2_BUFFER_SIZE);

    gpsPortClearBuffer();
}

///////////////////////////////////////////////////////////////////////////////
// GPS Decoding Variables
///////////////////////////////////////////////////////////////////////////////

struct ublox_NAV_POSLLH  // 01 02 (28)
{
	uint32_t iTow;
    int32_t  lon;    // 1e-7 degrees
    int32_t  lat;    // 1e-7 degrees
    int32_t  height; // mm
    int32_t  hMSL;   // mm
    uint32_t hAcc;   // mm
    uint32_t vAcc;   // mm
};

struct ublox_NAV_STATUS  // 01 03 (16)
{
    uint32_t iTow;
    uint8_t  gpsFix;
    uint8_t  flags;
    uint8_t  fixStat;
    uint8_t  flags2;
    uint32_t ttfx;
    uint32_t msss;
};

struct ublox_NAV_DOP  // 01 04 (18)
{
	uint32_t iTow;
	uint16_t gDOP;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t vDOP;
	uint16_t hDOP;
	uint16_t nDOP;
	uint16_t eDOP;
};

struct ublox_NAV_SOL  // 01 06 (52)
{
	uint32_t iTow;
    int32_t  fTow;
    int16_t  week;
    uint8_t  gspFix;
    uint8_t  flags;
    int32_t  ecefX;
    int32_t  ecefY;
    int32_t  ecefZ;
    int32_t  pAcc;
    int32_t  ecefVX;
    int32_t  ecefVY;
    int32_t  ecefVZ;
    int32_t  sAcc;
    uint16_t pDOP;
    uint8_t  res1;
    uint8_t  numSV;
    uint32_t res2;
};

struct ublox_NAV_VELNED  // 01 18 (36)
{
	uint32_t iTow;
    int32_t  velN;    // cm/s
    int32_t  velE;    // cm/s
    int32_t  velD;    // cm/s
    uint32_t speed;   // cm/s
    uint32_t gSpeed;  // cm/s
    int32_t  heading; // deg 1e-5
    uint32_t sAcc;    // cm/s
    uint32_t cAcc;    // deg 1e-5
};

struct ublox_NAV_TIMEUTC  // 01 33 (20)
{
    uint32_t iTOW;  // mSec
    uint32_t tACC;  // nSec
    int32_t  nano;  // nSec
    uint16_t year;  // years
    uint8_t  month; // months
    uint8_t  day;   // days
    uint8_t  hour;  // hours
    uint8_t  min;   // minutes
    uint8_t  sec;   // seconds
    uint8_t  valid;
};

struct ublox_NAV_SVINFO  // 01 48 (608, 8 + 12 * numCh, numCh = 50)
{
	uint32_t iTOW;        // mSec
	uint8_t  numCh;       // number of channels
	uint8_t  globalFlags; // bitmask
	uint16_t reserved2;   // reserved
    uint8_t  svinfo[600];
};

union ublox_message
{
    struct ublox_NAV_POSLLH  nav_posllh;
    struct ublox_NAV_STATUS  nav_status;
    struct ublox_NAV_DOP     nav_dop;
    struct ublox_NAV_VELNED  nav_velned;
    struct ublox_NAV_SOL     nav_sol;
    struct ublox_NAV_TIMEUTC nav_timeutc;
    struct ublox_NAV_SVINFO  nav_svinfo;
    unsigned char raw[608];
}   ubloxMessage;

uint16_t ubloxExpectedDataLength;
uint16_t ubloxDataLength;
uint8_t  ubloxClass,ubloxId;
uint8_t  ubloxCKA,ubloxCKB;
double LAT;

///////////////////////////////////////////////////////////////////////////////
// UBLOX Parse Data
///////////////////////////////////////////////////////////////////////////////
uint16_t GPS_pDop;
void ubloxParseData(void)
{
    uint8_t n;
	static float KX, KY;
	static int32_t GPS_lon_start, GPS_lat_start, GPS_height_start;
	int32_t  GPS_height, GPS_V_N, GPS_V_E, GPS_V_D, GPS_V_G, GPS_head;
//	uint16_t GPS_pDop;

    if (ubloxClass == 1)         // NAV
    {
        ///////////////////////////////
	//	Led3_Toggle;
    	if (ubloxId == 2)        // NAV:POSLLH
        {
        	gps.latitude  = ubloxMessage.nav_posllh.lat;
        	gps.longitude = ubloxMessage.nav_posllh.lon;
        	gps.height    = ubloxMessage.nav_posllh.height;
        	gps.hMSL      = ubloxMessage.nav_posllh.hMSL;
        	gps.hAcc      = ubloxMessage.nav_posllh.hAcc;
			gps.vAcc	  = ubloxMessage.nav_posllh.vAcc;
        	GPS_lon = gps.longitude;
        	GPS_lat = gps.latitude;
        	GPS_height = gps.height;
        	GPS_hAcc = gps.hAcc;
        	GPS_vAcc = gps.vAcc;
        	GPS_LA = ((float)(GPS_lat))/10000000.0f;
        	GPS_LO = ((float)(GPS_lon))/10000000.0f;
        	GPS_hAccf = (float)GPS_hAcc/1000.0f;
        	GPS_vAccf = (float)GPS_vAcc/1000.0f;
        	GPS_Micros_Update = timerMicros() - GPS_LATENCY;
        }

    	///////////////////////////////

    	else if (ubloxId == 3)   // NAV:STATUS
        {
            gps.fix         = ubloxMessage.nav_status.gpsFix;
            gps.statusFlags = ubloxMessage.nav_status.flags;
            GPS_fix_type = gps.fix;
    	//	Led3_Toggle;
        }

    	///////////////////////////////

    	else if (ubloxId == 4)   // NAV:DOP
        {
        	gps.hDop = ubloxMessage.nav_dop.hDOP;
        	gps.vDop = ubloxMessage.nav_dop.vDOP;
        	gps.pDop = ubloxMessage.nav_dop.pDOP;
        	gps.eDop = ubloxMessage.nav_dop.eDOP;
        	gps.nDop = ubloxMessage.nav_dop.nDOP;
        	GPS_pDop = gps.pDop;
        	GPS_pDopf = (float)GPS_pDop/100.0f;
        	//DOP convert
        	GPS_tDop = gps.tDop;
        	GPS_vDop = gps.vDop;
        	GPS_nDop = gps.nDop;
        	GPS_eDop = gps.eDop;
        	xSemaphoreTake(xGPS_UKF_Mutex, portMAX_DELAY);
        	GPS_tDopf = (float)GPS_tDop/100.0f;
        	GPS_vDopf = (float)GPS_vDop/100.0f;
        	GPS_nDopf = (float)GPS_nDop/100.0f;
        	GPS_eDopf = (float)GPS_eDop/100.0f;
        	xSemaphoreGive(xGPS_UKF_Mutex);
		}

    	///////////////////////////////

    	else if (ubloxId == 6)   // NAV:SOL
        {
			gps.numSats = ubloxMessage.nav_sol.numSV;
			gps.sAcc = ubloxMessage.nav_sol.sAcc;
			GPS_sAcc = gps.sAcc;
			GPS_satellits = gps.numSats;
			GPS_sAccf = (float)GPS_sAcc/1000.0f;
        }

    	///////////////////////////////

    	else if (ubloxId == 18)  // NAV:VELNED
        {
    		gps.velN    = ubloxMessage.nav_velned.velN;
    		gps.velE    = ubloxMessage.nav_velned.velE;
    	    gps.velD    = ubloxMessage.nav_velned.velD;
    	    gps.speed   = ubloxMessage.nav_velned.speed;
    	    gps.gSpeed  = ubloxMessage.nav_velned.gSpeed;
    		gps.heading = ubloxMessage.nav_velned.heading;
    		GPS_V_N = gps.velN;
    		GPS_V_E = gps.velE;
    		GPS_V_D = gps.velD;
    		GPS_V_G = gps.gSpeed;
    		GPS_head = gps.heading;
    		GPS_course = (float)GPS_head/100000.0f;
    		GPS_vel = ((float)GPS_V_G)*0.001f;
    		GPS_Y_speed = ((float)GPS_V_N)*0.001f;
    		GPS_X_speed = ((float)GPS_V_E)*0.001f;
    		GPS_Z_speed = -((float)GPS_V_D)*0.001f;
        }

    	///////////////////////////////

    	else if (ubloxId == 33)  // NAV:TIMEUTC
        {
        	gps.iTOW  = ubloxMessage.nav_timeutc.iTOW;
        	gps.year  = ubloxMessage.nav_timeutc.year;
			gps.month = ubloxMessage.nav_timeutc.month;
            gps.day   = ubloxMessage.nav_timeutc.day;
            gps.hour  = ubloxMessage.nav_timeutc.hour;
			gps.min   = ubloxMessage.nav_timeutc.min;
			gps.sec   = ubloxMessage.nav_timeutc.sec;

			GPS_year = gps.year;
			GPS_month = gps.month;
			GPS_day = gps.day;
			GPS_hours = gps.hour;
			GPS_minutes = gps.min;
			GPS_seconds = gps.sec;
		}

    	///////////////////////////////

    	else if (ubloxId == 48)  // NAV:SVINFO
    	{
			gps.numCh = ubloxMessage.nav_svinfo.numCh;

			for (n = 0; n < gps.numCh; n++)
			{
				gps.chn[n]  = ubloxMessage.nav_svinfo.svinfo[0 + 12 * n];
				gps.svid[n] = ubloxMessage.nav_svinfo.svinfo[1 + 12 * n];
				gps.cno[n]  = ubloxMessage.nav_svinfo.svinfo[4 + 12 * n];
			}
		}

		///////////////////////////////
    	if ((GPS_satellits > 8)&&(GPS_pDop<175))
    	    {
    		if (flag_start_home == 0)
    		    {
    			flag_start_home = 1;
    			GPS_height_start = GPS_height;
    			GPS_lon_start = GPS_lon;
    			GPS_lat_start = GPS_lat;
    			LAT = (double)GPS_lat/10000000.0f;
    	//		navUkfCalcEarthRadius(LAT, &KX, &KY);
    		    }


    //		GPS_X = ((float)(GPS_lon-GPS_lon_start))*K_Y_GPS*k_x;
    //		GPS_Y = ((float)(GPS_lat-GPS_lat_start))*K_Y_GPS;
    		xSemaphoreTake(xGPS_UKF_Mutex, portMAX_DELAY);
    		GPS_X = ((float)(GPS_lon-GPS_lon_start))*KX;
    		GPS_Y = ((float)(GPS_lat-GPS_lat_start))*KY;
    		GPS_alt = ((float)(GPS_height-GPS_height_start))*0.001f;

    		xSemaphoreGive(xGPS_UKF_Mutex);

    		StateNavUKF &= ~UKF_BAD_GPS;

    		flag_get_pvt = 1;
    		//Led4_On;
    	    }
    	else
    	    {
    		StateNavUKF |= UKF_BAD_GPS;
    	    }

    }
}

///////////////////////////////////////////////////////////////////////////////
// Decode UBLOX Message
///////////////////////////////////////////////////////////////////////////////

uint8_t decodeUbloxMsg(void)
{
    uint8_t  data;
    uint8_t  parsed = false;
    uint16_t i;
    uint16_t numberOfChars;

    numberOfChars = gpsPortNumCharsAvailable();

    for (i = 0; i < numberOfChars; i++)
    {
		data = gpsPortRead();

        switch (ubloxProcessDataState)
        {
            ///////////////////////////

            case WAIT_SYNC1:
                if (data == 0xb5)
                    ubloxProcessDataState = WAIT_SYNC2;

                break;

            ///////////////////////////

            case WAIT_SYNC2:
                if (data == 0x62)
                    ubloxProcessDataState = GET_CLASS;
                else
                    ubloxProcessDataState = WAIT_SYNC1;

                break;

            ///////////////////////////

            case GET_CLASS:
                ubloxClass            = data;
                ubloxCKA              = data;
                ubloxCKB              = data;
                ubloxProcessDataState = GET_ID;

                break;

            ///////////////////////////

            case GET_ID:
                ubloxId               = data;
                ubloxCKA             += data;
                ubloxCKB             += ubloxCKA;
                ubloxProcessDataState = GET_LL;

                break;

            ///////////////////////////

            case GET_LL:
                ubloxExpectedDataLength = data;
                ubloxCKA               += data;
                ubloxCKB               += ubloxCKA;
                ubloxProcessDataState   = GET_LH;

                break;

            ///////////////////////////

            case GET_LH:
                ubloxExpectedDataLength += data << 8;
                ubloxDataLength          = 0;
                ubloxCKA                += data;
                ubloxCKB                += ubloxCKA;

                if (ubloxExpectedDataLength <= sizeof(ubloxMessage))
                    ubloxProcessDataState = GET_DATA;
                else
                    // discard overlong message
                    ubloxProcessDataState = WAIT_SYNC1;

                break;

            ///////////////////////////

            case GET_DATA:
                ubloxCKA += data;
                ubloxCKB += ubloxCKA;

                // next will discard data if it exceeds our biggest known msg
                if (ubloxDataLength < sizeof(ubloxMessage))
                    ubloxMessage.raw[ubloxDataLength++] = data;

                if (ubloxDataLength >= ubloxExpectedDataLength)
                    ubloxProcessDataState = GET_CKA;

                break;

            ///////////////////////////

            case GET_CKA:
                if (ubloxCKA != data)
                    ubloxProcessDataState = WAIT_SYNC1;
	            else
                    ubloxProcessDataState = GET_CKB;

                break;

            ///////////////////////////

            case GET_CKB:
                if (ubloxCKB == data)
                {
                    parsed = 1;
                    ubloxParseData();
                }

                ubloxProcessDataState = WAIT_SYNC1;

                break;
        }
    }
    return parsed;
}

///////////////////////////////////////////////////////////////////////////////
void GPS_Processing(void *argument)
{
	static uint32_t currentTime;
	while (1)
	{
        currentTime       = TIM5->CNT;
        deltaTimeGPS    = currentTime - previousGPSTime;
        previousGPSTime = currentTime;
        dtGPS = (float)deltaTimeGPS * 0.0000001f;

	if (gpsPortAvailable())
		{
		decodeUbloxMsg();
		//Led3_Toggle;
		}
	vTaskDelay(50);
    // Calculate execution tick process time
    executionTimeGPS = TIM5->CNT - currentTime;
	}
}
extern uint8_t  RX2Buffer[USART2_BUFFER_SIZE] __attribute__((aligned (32)));
void GPS_init(void)
{
	initUBLOX();
	vSemaphoreCreateBinary(xGPS_Semaphore);
	cliPortPrintF("FreeHeapSize =  %6d, before GPS_task \r\n", xPortGetFreeHeapSize());
	xTaskCreate(GPS_Processing,(char*)"GPS", 600, NULL, 3, gps_handle);

	HAL_UART_Receive_DMA(&huart2, (uint8_t *)RX2Buffer, (uint16_t) USART2_BUFFER_SIZE);
	 serial2ClearBuffer();
//	 TX2BufferFull = false;;
//	gpsPortPrint("Test gps port.\r\n");
}
