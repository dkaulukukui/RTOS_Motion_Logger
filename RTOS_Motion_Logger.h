//**************************************************************************
// FreeRtos on Samd21
// By Scott Briscoe
//
// Project is a simple example of how to get FreeRtos running on a SamD21 processor
// Project can be used as a template to build your projects off of as well
//
//**************************************************************************

#include <FreeRTOS_SAMD21.h>
#include <avr/dtostrf.h>   //hack to get dtostrf() to work ???? idk why???

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

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;
TaskHandle_t Handle_monitorTask;

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account processor speed
// Use these instead of delay(...) in rtos tasks


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
  float Accel_X;
  float Accel_Y;
  float Accel_Z;
  float Gyro_X;
  float Gyro_Y;
  float Gyro_Z;
  float Mag_X;
  float Mag_Y;
  float Mag_Z;
  float LinearAccel_X;
  float LinearAccel_Y;
  float LinearAccel_Z;
  float Grav_X;
  float Grav_Y;
  float Grav_Z;
  float RotVec_Real;
  float RotVec_I;
  float RotVec_J;
  float RotVec_K;
  float GeoMagVec_Real;
  float GeoMagVec_I;
  float GeoMagVec_J;
  float GeoMagVec_K;
  float GameRotVec_Real;
  float GameRotVec_I;
  float GameRotVec_J;
  float GameRotVec_K;
  int16_t RawAccel_X;
  int16_t RawAccel_Y;
  int16_t RawAccel_Z;
  int16_t RawGyro_X;
  int16_t RawGyro_Y;
  int16_t RawGyro_Z;
  int16_t RawMag_X;
  int16_t RawMag_Y;
  int16_t RawMag_Z;
};                          //structure of BNO data

BNO_DATA BNO_Array[BNO_ARRAY_SIZE]; //Array to hold BNO data
BNO_DATA BNO_Transmit[BNO_ARRAY_SIZE]; 