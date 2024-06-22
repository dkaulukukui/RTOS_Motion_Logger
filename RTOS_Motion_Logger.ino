//**************************************************************************
// RTOS MOTION LOGGER using BNO085
//
//To Do List:
//  - (done) get data and calculate heading
//  - (done) get data and calculate pitch, yaw and roll
//  - implement SD card logging
//  - implement GPS library
//  - implement RTC library
//  - implement OLED display
//  - figure out calibration procedure for BNO085
//**************************************************************************


//#include <FreeRTOS_SAMD21.h>
#include "RTOS_Motion_Logger.h"

//**************************************************************************
void setReports(void) {
  //bno08x.enableReport(SH2_ACCELEROMETER, reportIntervalUs);
  //bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  //bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
  //bno08x.enableReport(SH2_LINEAR_ACCELERATION, reportIntervalUs);
  //bno08x.enableReport(SH2_GRAVITY);
  bno08x.enableReport(SH2_ROTATION_VECTOR, reportIntervalUs);
  //bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR);
  //bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  //bno08x.enableReport(SH2_RAW_ACCELEROMETER);
  //bno08x.enableReport(SH2_RAW_GYROSCOPE);
  //bno08x.enableReport(SH2_RAW_MAGNETOMETER);
}

//**************************************************************************
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void Normalized_quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  // Normalise quaternion before conversion.
  float qlength = sqrt(sq(qr) + sq(qi) + sq(qj) + sq(qk));
  qr /= qlength;
  qi /= qlength;
  qj /= qlength;
  qk /= qlength;
  
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw =  atan2( 2.0 * (qi * qj + qk * qr),  (sqi - sqj - sqk + sqr));
  //ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  //ypr->roll = atan2( 2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  ypr->roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));  //swapped for default orientation
  ypr->pitch = atan2( 2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));  //swapped for default orientation

  if (degrees) {
    ypr->yaw   *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll  *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float getHeading() {
  // Y axis facing north, X facing east 
  //default yaw is reported in degrees from north, rotating west is posistive, rotatin east is negative
  // South is either 180 or -180
  // Returns heading in True 0-359.9

  if (yaw > 0 && yaw < 180) { // Quadrants III, IV
    return (360 - yaw);
  } else {                      // Quadrant I and II
    return -(yaw);
  } 
}

//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );  
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

//*****************************************************************
// Create a thread that prints out A to the screen every two seconds
// this task will delete its self after printing out afew messages
//*****************************************************************
static void threadA( void *pvParameters )  //Data Getting task
{
  
  SERIAL.println(F("Thread A: Started"));
  TickType_t tick_count = xTaskGetTickCount();
  //uint16_t p_sens = 0.5 * configTICK_RATE_HZ;  // 1 second period * 1000 ms/s
  uint16_t p_sens = 100;

    //**************************************************************************
  // BNO085 setup
  //**************************************************************************
  //Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  while (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    //while (1) {
      delay(100);
    //}
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  //**************************************************************************
  // RTC setup
  //**************************************************************************
 
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

    // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  rtc.start();

  while(1) {
    
    //vTaskDelayUntil(&tick_count, p_sens);
    myDelayMs(100);

    if (bno08x.wasReset()) {
      Serial.println("sensor was reset ");
      setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) { 

      // get a buffer
      if (xSemaphoreTake(BNO_Space_SemaphorHandle, 0) != pdTRUE) {
        BNO_error++;  // fifo full - indicate missed point
        Serial.println("buffer full");
        continue;
      }

      acc_status = sensorValue.status;

      BNO_DATA *bno_data = &BNO_Array[BNO_Head_Q]; //grab the first data set in Q

      switch (sensorValue.sensorId) {

        case SH2_ACCELEROMETER:
          bno_data->Accel_X = sensorValue.un.accelerometer.x;
          bno_data->Accel_Y = sensorValue.un.accelerometer.y;
          bno_data->Accel_Z = sensorValue.un.accelerometer.z;
          break;
        case SH2_GYROSCOPE_CALIBRATED:
          bno_data->Gyro_X = sensorValue.un.gyroscope.x;
          bno_data->Gyro_Y = sensorValue.un.gyroscope.y;
          bno_data->Gyro_Z = sensorValue.un.gyroscope.z;
          break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
          bno_data->Mag_X = sensorValue.un.magneticField.x;
          bno_data->Mag_Y = sensorValue.un.magneticField.y;
          bno_data->Mag_Z = sensorValue.un.magneticField.z;
          break;
        case SH2_LINEAR_ACCELERATION:
          bno_data->LinearAccel_X = sensorValue.un.linearAcceleration.x;
          bno_data->LinearAccel_Y = sensorValue.un.linearAcceleration.y;
          bno_data->LinearAccel_Z = sensorValue.un.linearAcceleration.z;
          break;
        case SH2_GRAVITY:
          bno_data->Grav_X = sensorValue.un.gravity.x;
          bno_data->Grav_Y = sensorValue.un.gravity.y;
          bno_data->Grav_Z = sensorValue.un.gravity.z;
          break;
        case SH2_ROTATION_VECTOR:
          bno_data->RotVec_Real = sensorValue.un.rotationVector.real;
          bno_data->RotVec_I = sensorValue.un.rotationVector.i;
          bno_data->RotVec_J = sensorValue.un.rotationVector.j;
          bno_data->RotVec_K = sensorValue.un.rotationVector.k;

          Normalized_quaternionToEuler(bno_data->RotVec_Real, bno_data->RotVec_I, bno_data->RotVec_J, bno_data->RotVec_K, &ypr, true); // degrees
          yaw = ypr.yaw;
          pitch = ypr.pitch;
          roll = ypr.roll;
          heading = getHeading();
          rot_accuracy = sensorValue.un.rotationVector.accuracy;
          break;

        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
          bno_data->GeoMagVec_Real = sensorValue.un.geoMagRotationVector.real;
          bno_data->GeoMagVec_I = sensorValue.un.geoMagRotationVector.i;
          bno_data->GeoMagVec_J = sensorValue.un.geoMagRotationVector.j;
          bno_data->GeoMagVec_K = sensorValue.un.geoMagRotationVector.k;
          break;

        case SH2_GAME_ROTATION_VECTOR:
          bno_data->GameRotVec_Real = sensorValue.un.gameRotationVector.real;
          bno_data->GameRotVec_I = sensorValue.un.gameRotationVector.i;
          bno_data->GameRotVec_J = sensorValue.un.gameRotationVector.j;
          bno_data->GameRotVec_K = sensorValue.un.gameRotationVector.k;
          break;

        case SH2_RAW_ACCELEROMETER:
          bno_data->RawAccel_X = sensorValue.un.rawAccelerometer.x;
          bno_data->RawAccel_Y = sensorValue.un.rawAccelerometer.y;
          bno_data->RawAccel_Z = sensorValue.un.rawAccelerometer.z;
          break;
        case SH2_RAW_GYROSCOPE:
          bno_data->RawGyro_X = sensorValue.un.rawGyroscope.x;
          bno_data->RawGyro_Y = sensorValue.un.rawGyroscope.y;
          bno_data->RawGyro_Z = sensorValue.un.rawGyroscope.z;
          break;
        case SH2_RAW_MAGNETOMETER:
          bno_data->RawMag_X = sensorValue.un.rawMagnetometer.x;
          bno_data->RawMag_Y = sensorValue.un.rawMagnetometer.y;
          bno_data->RawMag_Z = sensorValue.un.rawMagnetometer.z;
          break;
      }

      bno_data->log_time = rtc.now();  // time data is logged

      BNO_error = 0;
      BNO_Head_Q = BNO_Head_Q < (BNO_ARRAY_SIZE -1) ? BNO_Head_Q +1 : 0; //advance fifo index, if true increment, else reset
      xSemaphoreGive(BNO_Data_SemaphorHandle);  //signal new data

    }

  }

}

//*****************************************************************
// Create a thread that prints out B to the screen every second
// this task will run forever
//*****************************************************************
static void threadB( void *pvParameters )  //Data Output
{
  SERIAL.println("Thread B: Started");


  //create temporary buffers to hold stuff for printing
  const uint8_t BUFLEN = 100;
  char buf[BUFLEN] = "";

  const uint8_t FLT_STR_LEN = 10;
  
  char Log_Time[20] = "";
  //char Gyro_X[FLT_STR_LEN] = ""; 
  //char Gyro_Y[FLT_STR_LEN] = "";
  //char Gyro_Z[FLT_STR_LEN] = "";
  //char Accel_X[FLT_STR_LEN] = ""; 
  //char Accel_Y[FLT_STR_LEN] = "";
  //char Accel_Z[FLT_STR_LEN] = "";
  //char YAW[FLT_STR_LEN] = ""; 
  char PITCH[FLT_STR_LEN] = "";
  char ROLL[FLT_STR_LEN] = "";
  char HDG[FLT_STR_LEN] = "";
  char ROT_ACC[FLT_STR_LEN] = "";
  char FRAC_SEC[FLT_STR_LEN] = "";

  double frac_sec = 0;
  uint8_t current_sec = rtc.now().second();


  BNO_DATA *bno_data; // = &BNO_Array[BNO_Tail_Q];

  while(1) { 

    //clear all temp buffers
    strcpy(buf,""); //reset buffer

    strcpy(Log_Time,"");
    //strcpy(Accel_X,""); //reset buffer
    //strcpy(Accel_Y,""); //reset buffer
    //strcpy(Accel_Z,""); //reset buffer
    //strcpy(YAW,""); //reset buffer
    strcpy(PITCH,""); //reset buffer
    strcpy(ROLL,""); //reset buffer
    strcpy(HDG,""); //reset buffer
    strcpy(ROT_ACC,""); //reset buffer
    strcpy(FRAC_SEC,"");

    xSemaphoreTake(BNO_Data_SemaphorHandle, portMAX_DELAY);  // wait for next data record
    bno_data = &BNO_Array[BNO_Tail_Q];

    time = rtc.now();

    if(time.second() != current_sec) {  //reset millis delta whenever the seconds changes
      last_millis = millis();
      current_sec = time.second();
    }
    //last_millis = millis();

    frac_sec = (millis() - last_millis)*0.001;

    //snprintf(Log_Time, 20, "%02d:%02d:%02d %02d/%02d/%02d",  bno_data->log_time.hour(),  bno_data->log_time.minute(),  bno_data->log_time.second(), bno_data->log_time.day(),  bno_data->log_time.month(),  bno_data->log_time.year()); 
    //bno_data->log_time.timestamp().toCharArray(Log_Time,20);
    time.timestamp().toCharArray(Log_Time,20);

    //dtostrf(bno_data->Gyro_X, 5,2, Gyro_X);
    //dtostrf(bno_data->Gyro_Y, 5,2, Gyro_Y);
    //dtostrf(bno_data->Gyro_Z, 5,2, Gyro_Z);
    //dtostrf(bno_data->LinearAccel_X, 5,2, Accel_X);
    //dtostrf(bno_data->LinearAccel_Y, 5,2, Accel_Y);
    //dtostrf(bno_data->LinearAccel_Z, 5,2, Accel_Z);
    //dtostrf(yaw, 5,2, YAW);
    dtostrf(pitch, 5,2, PITCH);
    dtostrf(roll, 5,2, ROLL);
    dtostrf(heading, 5,2, HDG);
    dtostrf(rot_accuracy, 5,2, ROT_ACC);
    dtostrf(frac_sec,3,3,FRAC_SEC);

    strcat(buf, Log_Time);
    //strcat(buf, "\t");
    strcat(buf, FRAC_SEC+1); //skip the whole number portion of the string
    strcat(buf, "\t");
    strcat(buf, HDG);
    strcat(buf, "\t");
    //strcat(buf, YAW);
    //strcat(buf, "\t");
    strcat(buf, PITCH);
    strcat(buf, "\t");
    strcat(buf, ROLL);
    strcat(buf, "\t");
    strcat(buf, ROT_ACC);
    strcat(buf, "\t");
    //strcat(buf, Accel_X);
    //strcat(buf, "\t");
    //strcat(buf, Accel_Y);
    //strcat(buf, "\t");
    //strcat(buf, Accel_Z);
    //snprintf(buf, BUFLEN,"%d\t%d\t%d",bno_data->RawGyro_X, bno_data->RawGyro_Y, bno_data->RawGyro_Z);

    SERIAL.println(buf);

    BNO_Tail_Q = BNO_Tail_Q < (BNO_ARRAY_SIZE-1) ? BNO_Tail_Q +1 :0;
    xSemaphoreGive(BNO_Space_SemaphorHandle);

  }

}

//*****************************************************************
// Task will periodically print out useful information about the tasks running
// Is a useful tool to help figure out stack sizes being used
// Run time stats are generated from all task timing collected since startup
// No easy way yet to clear the run time stats yet
//*****************************************************************
static char ptrTaskList[400]; //temporary string buffer for task stats

void taskMonitor(void *pvParameters)
{
    int x;
    int measurement;
    
    SERIAL.println("Task Monitor: Started");

    // run this task afew times before exiting forever
    while(1) {
    	myDelayMs(10000); // print every 10 seconds

    	SERIAL.flush();
		  SERIAL.println("");			 
    	SERIAL.println("****************************************************");
    	SERIAL.print("Free Heap: ");
    	SERIAL.print(xPortGetFreeHeapSize());
    	SERIAL.println(" bytes");

    	SERIAL.print("Min Heap: ");
    	SERIAL.print(xPortGetMinimumEverFreeHeapSize());
    	SERIAL.println(" bytes");
    	SERIAL.flush();

    	SERIAL.println("****************************************************");
    	SERIAL.println("Task             ABS             %Util");
    	SERIAL.println("****************************************************");

    	vTaskGetRunTimeStats(ptrTaskList); //save stats to char array
    	SERIAL.println(ptrTaskList); //prints out already formatted stats
    	SERIAL.flush();

      SERIAL.println("****************************************************");
      SERIAL.println("Task            State   Prio    Stack   Num     Core");
      SERIAL.println("****************************************************");

      vTaskList(ptrTaskList); //save stats to char array
      SERIAL.println(ptrTaskList); //prints out already formatted stats
      SERIAL.flush();

      SERIAL.println("****************************************************");
      SERIAL.println("[Stacks Free Bytes Remaining] ");

      measurement = uxTaskGetStackHighWaterMark( Handle_aTask );
      SERIAL.print("Thread A: ");
      SERIAL.println(measurement);

      measurement = uxTaskGetStackHighWaterMark( Handle_bTask );
      SERIAL.print("Thread B: ");
      SERIAL.println(measurement);

      measurement = uxTaskGetStackHighWaterMark( Handle_monitorTask );
      SERIAL.print("Monitor Stack: ");
      SERIAL.println(measurement);

      SERIAL.println("****************************************************");
      SERIAL.flush();

    }

    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    SERIAL.println("Task Monitor: Deleting");
    vTaskDelete( NULL );

}


//*****************************************************************

void setup() 
{

  SERIAL.begin(115200);

  delay(1000); // prevents usb driver crash on startup, do not omit this
  while (!SERIAL) ;  // Wait for serial terminal to open port before starting program

  //**************************************************************************
  BNO_Data_SemaphorHandle = xSemaphoreCreateCounting(BNO_ARRAY_SIZE,0);
  BNO_Space_SemaphorHandle = xSemaphoreCreateCounting(BNO_ARRAY_SIZE,BNO_ARRAY_SIZE);

  // 
  SERIAL.println("");
  SERIAL.println("******************************");
  SERIAL.println("        Program start         ");
  SERIAL.println("******************************");
  SERIAL.flush();

  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  // sets the serial port to print errors to when the rtos crashes
  // if this is not set, serial information is not printed by default
  vSetErrorSerial(&SERIAL);

  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
  xTaskCreate(threadA,     "Task A",       256, NULL, tskIDLE_PRIORITY + 3, &Handle_aTask);
  xTaskCreate(threadB,     "Task B",       512, NULL, tskIDLE_PRIORITY + 2, &Handle_bTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while(1)
  {
	  SERIAL.println("Scheduler Failed! \n");
	  SERIAL.flush();
	  delay(1000);
  }

}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    // Optional commands, can comment/uncomment below
    //SERIAL.print("."); //print out dots in terminal, we only do this when the RTOS is in the idle state
    //SERIAL.flush();
    //delay(100); //delay is interrupt friendly, unlike vNopDelayMS
}


//*****************************************************************

