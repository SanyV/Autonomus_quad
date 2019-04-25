/*
 2017 Alexander Valov ported from AQ32
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once
//***************************************************************************************************//
#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))

#define GPS_LATENCY             75000       // us (comment out to use uBlox timepulse)

//*******************Главные данные для расчетов*******************************//
//Позиция в м, относительно старта
extern float GPS_X, GPS_Y;
extern float GPS_X_home, GPS_Y_home;
extern float GPS_alt;
extern float GPS_X_cmp, GPS_Y_cmp;
extern float GPS_alt_cmp;
//Скорости, курс
extern float GPS_Y_speed, GPS_X_speed, GPS_Z_speed;
extern float GPS_vel, GPS_course;
//Показатели качества
extern float GPS_hAccf, GPS_vAccf, GPS_sAccf;
extern float GPS_pDopf, GPS_eDopf, GPS_vDopf, GPS_nDopf, GPS_tDopf;


//Экспорт
//Для персчета позиции и скорости, в случае положения ЖПС не в центре
#define GPS_FRAME_POS_X		(float)(-10.5f*0.01f)
#define GPS_FRAME_POS_Y		(float)(11.0f*0.01f)
#define GPS_FRAME_POS_Z		(float)(13.5f*0.01f)

//вкл компенсацию
#define USE_GPS_FRAME_POS_CMPNST
#define USE_GPS_FRAME_VEL_CMPNST

extern SemaphoreHandle_t xGPS_UKF_Mutex;

//Счетчики цикла
extern uint32_t cnt_cycle_gps, cnt_ticks_gps;
//****************************************************//
//Микросекунды получения данных, флаги появления данных
extern uint32_t GPS_Micros_Update;
extern uint8_t flag_start_home, flag_get_pvt;
//****************************************************//
//****************************************************//
//Вспомогательные данные
extern uint16_t GPS_year;
extern uint8_t GPS_month, GPS_day, GPS_hours, GPS_minutes, GPS_seconds;
extern uint8_t GPS_satellits;
extern uint8_t GPS_valid_invalid;
extern uint8_t GPS_fix_type;
//****************************************************//
//Данные не для расчетов, только для телемтрии
extern float GPS_LA, GPS_LO;
extern int32_t GPS_lon, GPS_lat;
extern uint32_t GPS_hAcc, GPS_vAcc, GPS_sAcc;
extern uint16_t GPS_vDop, GPS_nDop, GPS_eDop, GPS_tDop;
//****************************************************//


//*******************Главные данные для расчетов*******************************//
//Позиция в м, относительно старта
extern float GPS_X, GPS_Y;
extern float GPS_X_home, GPS_Y_home;
extern float GPS_alt;
//Скорости, курс
extern float GPS_Y_speed, GPS_X_speed, GPS_Z_speed;
extern float GPS_vel, GPS_course;
//Показатели качества
extern float GPS_hAccf, GPS_vAccf, GPS_sAccf;
extern float GPS_pDopf, GPS_eDopf, GPS_vDopf, GPS_nDopf, GPS_tDopf;
//****************************************************//

///////////////////////////////////////////////////////////////////////////////

#define NO_FIX         0x00
#define DEAD_RECKONING 0x01
#define FIX_2D         0x02
#define FIX_3D         0x03

#define GPS_FIX_OK     0x01

///////////////////////////////////////////////////////////////////////////////

extern void     (*gpsPortClearBuffer)(void);

extern uint16_t (*gpsPortNumCharsAvailable)(void);

extern void     (*gpsPortPrintBinary)(uint8_t *buf, uint16_t length);

extern uint8_t  (*gpsPortRead)(void);

//////////////////////////////////////////////////////////////////////////////
typedef struct gps_t
{
	uint32_t iTOW;
	int32_t  latitude;     // 1e-7 degrees
	int32_t  longitude;    // 1e-7 degrees
	int32_t  height;       // mm above ellipsoid
	int32_t  hMSL;         // mm above mean sea level
	int32_t	 hAcc;    	   // horizontal accuracy est (m)
	int32_t	 vAcc;         // vertical accuracy est (m)
	int32_t  velN;         // cm/s
	int32_t  velE;         // cm/s
	int32_t  velD;         // cm/s
	uint32_t speed;        // cm/s
	uint32_t gSpeed;       // cm/s
	int32_t  heading;      // deg 1e-5
	int32_t  sAcc;     // speed accuracy est (m/s)
	int32_t  cAcc;     // course accuracy est (deg)
    uint8_t  numSats;
    uint8_t  fix;
    uint8_t  statusFlags;
 //   uint32_t iTOW;         // mSec
    uint16_t year;         // years
    uint8_t  month;        // months
    uint8_t  day;          // days
    uint16_t pDop;     // Position Dilution of Precision
    uint16_t hDop;
    uint16_t vDop;
    uint16_t tDop;
    uint16_t nDop;     // Northing DOP
    uint16_t eDop;     // Easting DOP
    uint16_t gDop;     // Geometric DOP
    uint8_t  numCh;
    uint8_t  chn[50];      // channel number
    uint8_t  svid[50];     // satellite ID
    uint8_t  cno[50];      // carrier to noise ratio (signal strength)
    uint8_t  updated;
    uint8_t  hour;  // hours
    uint8_t  min;   // minutes
    uint8_t  sec;   // seconds

    unsigned long TPtowMS;    // timepulse time of week (ms)
    unsigned long lastReceivedTPtowMS;
    unsigned long lastPosUpdate;
    unsigned long lastVelUpdate;
    unsigned long lastMessage;
} gps_t;

typedef struct gpsdata_p
{
	float  latitude;     // 1e-7 degrees
	float  longitude;    // 1e-7 degrees
	float  height;       // mm above ellipsoid
} gpsdata_p;

typedef struct gpsdata_v
{
	float  velN;         // cm/s
	float  velE;         // cm/s
	float  velD;         // cm/s
} gpsdata_v;


///////////////////////////////////////////////////////////////////////////////
// Initialize UBLOX Receiver
///////////////////////////////////////////////////////////////////////////////

void initUBLOX(void);

///////////////////////////////////////////////////////////////////////////////
// Decode UBLOX Message
///////////////////////////////////////////////////////////////////////////////

uint8_t decodeUbloxMsg(void);

///////////////////////////////////////////////////////////////////////////////
void GPS_init(void);
