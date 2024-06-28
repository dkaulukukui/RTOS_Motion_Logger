#include <FreeRTOS_SAMD21.h>
#include <avr/dtostrf.h>   //hack to get dtostrf() to work ???? idk why???

#include <RTClib.h>
//**************************************************************************
// Configuration Option Defines
// - Use this section to easily enable or disable functions/features
//**************************************************************************

#define SERIAL_LOGGING  //leave on 

#define SD_LOGGING //will break program if enabled

#define BNO085_ON

#define GPS_ON  //enable GPS and GPS thread

#define RTC_ON

//#define TASK_MON  //enable task monitor thread

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define  ERROR_LED_PIN  13 //Led Pin: Typical Arduino Board
//#define  ERROR_LED_PIN  2 //Led Pin: samd21 xplained board

#define ERROR_LED_LIGHTUP_STATE  HIGH // the state that makes the led light up on your board, either low or high

// Select the serial port the project should use and communicate over
// Some boards use SerialUSB, some use Serial
//#define SERIAL          SerialUSB //Sparkfun Samd21 Boards
#define SERIAL          Serial //Adafruit, other Samd21 Boards

#define DEFAULT_STACK_SIZE 256

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_monitorTask;

const char LOG_SEPARATOR = '\t';  //character to use for serial logging seperation


//**************************************************************************
// SD LOGGING 
//**************************************************************************
#ifdef SD_LOGGING

  #include <SPI.h>
  #include <SD.h>

  Sd2Card SDcard;

  #define SD_chipSelect 10

  char filename[20] =  "DD-hh-mm.csv";
  //char filename[20] =  "test2.csv";

  #define WIFI_CS_PIN 8

#endif

//**************************************************************************
// BNO085 defines
//**************************************************************************

#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
//#define BNO08X_CS 10
//#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// BNO Queue
size_t BNO_Tail_Q = 0;
size_t BNO_Head_Q = 0;
const size_t BNO_ARRAY_SIZE = 20;          // size of records
int BNO_error = 0;             // count of overrun error_senses
SemaphoreHandle_t BNO_Data_SemaphorHandle;   // count of data records
SemaphoreHandle_t BNO_Space_SemaphorHandle;  // count of freequ buffers

struct BNO_DATA {
  //float Accel_X;
  //float Accel_Y;
  //float Accel_Z;
  //float Gyro_X;
  //float Gyro_Y;
  //float Gyro_Z;
  //float Mag_X;
  //float Mag_Y;
  //float Mag_Z;
  float LinearAccel_X;
  float LinearAccel_Y;
  float LinearAccel_Z;
  //float Grav_X;
  //float Grav_Y;
  //float Grav_Z;
  float RotVec_Real;
  float RotVec_I;
  float RotVec_J;
  float RotVec_K;
  //float GeoMagVec_Real;
  //float GeoMagVec_I;
  //float GeoMagVec_J;
  //float GeoMagVec_K;
  //float GameRotVec_Real;
  //float GameRotVec_I;
  //float GameRotVec_J;
  //float GameRotVec_K;
  //int16_t RawAccel_X;
  //int16_t RawAccel_Y;
  //int16_t RawAccel_Z;
  //int16_t RawGyro_X;
  //int16_t RawGyro_Y;
  //int16_t RawGyro_Z;
  //int16_t RawMag_X;
  //int16_t RawMag_Y;
  //int16_t RawMag_Z;
  DateTime log_time;
};                          //structure of BNO data

BNO_DATA BNO_Array[BNO_ARRAY_SIZE]; //Array to hold BNO data
BNO_DATA BNO_Transmit[BNO_ARRAY_SIZE]; 

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

long reportIntervalUs = 20000; // 50Hz

// Vars to hold most recent report values
float yaw, pitch, roll, heading, rot_accuracy;
int acc_status;


//const float MAGNETIC_DECLINATION = 9.48; //Honolulu
const float MAGNETIC_DECLINATION = 0;

//**************************************************************************
// RTC defines
//**************************************************************************

#ifdef RTC_ON

  RTC_PCF8523 rtc;

  DateTime time;

  unsigned long last_millis = 0;

#endif

//**************************************************************************
// GPS defines
//**************************************************************************

#ifdef GPS_ON
  
  #include <Adafruit_GPS.h>

  #define GPSSerial Serial1

  // Connect to the GPS on the hardware port
  Adafruit_GPS GPS(&GPSSerial);

  // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
  // Set to 'true' if you want to debug and listen to the raw GPS sentences
  #define GPSECHO false
  //#define GPSECHO true


  uint32_t gps_timer = millis();

  TaskHandle_t Handle_gpsTask;

  // GPS Queue
  size_t GPS_Tail_Q = 0;
  size_t GPS_Head_Q = 0;
  const size_t GPS_ARRAY_SIZE = 20;          // size of records
  int GPS_error = 0;             // count of overrun error_senses

  SemaphoreHandle_t GPS_SemaphorHandle;   // count of data records

  /*struct GPS_DATA {
    u_int8_t year;
    u_int8_t month;
    u_int8_t day;
    u_int8_t hour;
    u_int8_t minute;
    u_int8_t sec;
    u_int16_t millisec;
    u_int8_t fixquality;
    uint8_t satellites;
    uint8_t antenna_status;
    float latitude;
    char lat;
    float longitude;
    char lon;
    float speed; //knots
    float cog; //course over ground ref to True North
    float altitude; 
  };      
  */

    //GPS_DATA GPS_ARRAY[GPS_ARRAY_SIZE]; //Array to hold GPS data

#endif

//**************************************************************************
// Display defines
//**************************************************************************
