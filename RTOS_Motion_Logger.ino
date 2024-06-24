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
//***********************GPS Functions*********************** */
void printGPS(){
    Serial.print(F("\nTime: "));
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print(F("00"));
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print(F("0"));
    }
    Serial.println(GPS.milliseconds);
    Serial.print(F("Date: "));
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print(F("/20"));
    Serial.println(GPS.year, DEC);
    Serial.print(F("Fix: ")); Serial.print((int)GPS.fix);
    Serial.print(F(" quality: ")); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print(F("Location: "));
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(F(", "));
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print(F("Speed (knots): ")); Serial.println(GPS.speed);
      Serial.print(F("Angle: ")); Serial.println(GPS.angle);
      Serial.print(F("Altitude: ")); Serial.println(GPS.altitude);
      Serial.print(F("Satellites: ")); Serial.println((int)GPS.satellites);
      Serial.print(F("Antenna status: ")); Serial.println((int)GPS.antenna);
    }
}

//**********Utility Functions****************************************************************
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

void append_float_to_log(char* s, float f, int length, int frac, char seperator){
  int len = strlen(s);

  char float_buffer[length+1] = "";
  char sep_buffer[2] = "";
  sep_buffer[0] = seperator;

  dtostrf(f,length,frac,float_buffer);
  strcat(s,float_buffer);
  strcat(s,sep_buffer);
}

// blink out an error code
void error(uint8_t errnum) {
  while(1) {
    uint8_t i;
    for (i=0; i<errnum; i++) {
      digitalWrite(ERROR_LED_PIN, HIGH);
      delay(100);
      digitalWrite(ERROR_LED_PIN, LOW);
      delay(100);
      yield();
    }
    for (i=errnum; i<10; i++) {
      delay(200);
      yield();
    }
  }
}

//*****************************************************************
// Create a thread to intialize and collect dataf rom BNO085
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
    Serial.println(F("Failed to find BNO08x chip"));
    //while (1) {
      delay(100);
    //}
    error(3);
  }
  Serial.println(F("BNO08x Found!"));

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print(F("Part "));
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(F(": Version :"));
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(F("."));
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(F("."));
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(F(" Build "));
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  //**************************************************************************
  // RTC setup
  //**************************************************************************
 
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    //while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println(F("RTC is NOT running, let's set the time!"));
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

    //**************************************************************************
    // BNO085 data collection
    //**************************************************************************

    if (bno08x.wasReset()) {
      Serial.println(F("sensor was reset "));
      setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) { 

      // get a buffer
      if (xSemaphoreTake(BNO_Space_SemaphorHandle, 0) != pdTRUE) {
        BNO_error++;  // fifo full - indicate missed point
        Serial.println(F("BNO buffer full"));
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
  SERIAL.println(F("Thread B: Started"));


  //create temporary buffers to hold stuff for printing
  const uint8_t BUFLEN = 100;
  char buf[BUFLEN] = "";

  const uint8_t FLT_STR_LEN = 10;
  
  char Log_Time[20] = "";
  char FRAC_SEC[FLT_STR_LEN] = "";
  
  double frac_sec = 0;
  uint8_t current_sec = rtc.now().second();


  BNO_DATA *bno_data; // = &BNO_Array[BNO_Tail_Q];

  while(1) { 

    //clear all temp buffers
    strcpy(buf,""); //reset buffer

    strcpy(Log_Time,"");
    strcpy(FRAC_SEC,"");

    xSemaphoreTake(BNO_Data_SemaphorHandle, portMAX_DELAY);  // wait for next data record
    bno_data = &BNO_Array[BNO_Tail_Q];

    time = rtc.now();

    if(time.second() != current_sec) {  //reset millis delta whenever the seconds changes
      last_millis = millis();
      current_sec = time.second();
    }

    frac_sec = (millis() - last_millis)*0.001;

    //snprintf(Log_Time, 20, "%02d:%02d:%02d %02d/%02d/%02d",  bno_data->log_time.hour(),  bno_data->log_time.minute(),  bno_data->log_time.second(), bno_data->log_time.day(),  bno_data->log_time.month(),  bno_data->log_time.year()); 
    //bno_data->log_time.timestamp().toCharArray(Log_Time,20);
    time.timestamp().toCharArray(Log_Time,20);
    dtostrf(frac_sec,3,3,FRAC_SEC);
    strcat(buf, Log_Time);
    //strcat(buf, "\t");
    strcat(buf, FRAC_SEC+1); //skip the whole number portion of the string
    strcat(buf, "\t");

    append_float_to_log(buf,heading,5,2,LOG_SEPARATOR);
    append_float_to_log(buf,pitch,5,2,LOG_SEPARATOR);
    append_float_to_log(buf,roll,5,2,LOG_SEPARATOR);
    append_float_to_log(buf,rot_accuracy,5,2,LOG_SEPARATOR);

    append_float_to_log(buf,GPS.latitude,10,4, GPS.lat);
    strcat(buf, "\t");
    append_float_to_log(buf,GPS.longitude,10,4, GPS.lon);
    strcat(buf, "\t");
    append_float_to_log(buf,GPS.speed, 5, 2, LOG_SEPARATOR);
    append_float_to_log(buf,GPS.angle, 5, 2, LOG_SEPARATOR);
    append_float_to_log(buf,(float)GPS.fixquality, 5, 0, LOG_SEPARATOR);

    BNO_Tail_Q = BNO_Tail_Q < (BNO_ARRAY_SIZE-1) ? BNO_Tail_Q +1 :0;
    xSemaphoreGive(BNO_Space_SemaphorHandle);

    xSemaphoreTake(GPS_SemaphorHandle,0);


    xSemaphoreGive(GPS_SemaphorHandle); 

    #ifdef SERIAL_LOGGING 
      SERIAL.println(buf);
      //SERIAL.flush();
    #endif

    #ifdef SD_LOGGING
      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open(filename, FILE_WRITE); 

        // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(buf);
        dataFile.close();
        // print to the serial port too:
      // Serial.println(dataString);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("error opening datalog.txt");
      }
    #endif

  }

}
//*****************************************************************
// Create a thread that read and parse the GPS NMEA data 
// this task will run forever
//*****************************************************************
static void GPSthread( void *pvParameters )  //Data Getting task
{
  
  SERIAL.println(F("GPS Thread: Started"));
  TickType_t tick_count = xTaskGetTickCount();
  //uint16_t p_sens = 0.5 * configTICK_RATE_HZ;  // 1 second period * 1000 ms/s
  uint16_t p_sens = 100;

  //**************************************************************************
  // GPS setup
  //**************************************************************************
   // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  //xSemaphoreGive(GPS_SemaphorHandle); //release GPS data


  while(1) {
    
    //vTaskDelayUntil(&tick_count, p_sens);
    myDelayMs(5);

    xSemaphoreTake(GPS_SemaphorHandle,0); //reserve the GPS data

      // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...

    if (GPS.newNMEAreceived()) {
      //Serial.println("nmea received");

      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trying to print out data
      //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
      if (!GPS.parse(GPS.lastNMEA())){ // this also sets the newNMEAreceived() flag to false
        //return; // we can fail to parse a sentence in which case we should just wait for another
      }
    }

    //GPS_DATA *gps_data = &GPS_ARRAY[GPS_Head_Q]; //grab the first gps data in q

    xSemaphoreGive(GPS_SemaphorHandle); //release GPS data
    
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

  #ifdef SERIAL_LOGGING
    SERIAL.begin(115200);

    delay(1000); // prevents usb driver crash on startup, do not omit this
    while (!SERIAL) ;  // Wait for serial terminal to open port before starting program
  #endif

  #ifdef SD_LOGGING
  //*************SD Card Setup**************** */

    // set up variables using the SD utility library functions:

    time = rtc.now();
    time.toString(filename);
    Serial.println(filename);

      // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
      Serial.println(F("Card failed, or not present"));
      // don't do anything more:
      error(2);
      //while (1);
    }
    Serial.println(F("card initialized."));
  

  #endif

  //**************************************************************************
  BNO_Data_SemaphorHandle = xSemaphoreCreateCounting(BNO_ARRAY_SIZE,0);
  BNO_Space_SemaphorHandle = xSemaphoreCreateCounting(BNO_ARRAY_SIZE,BNO_ARRAY_SIZE);
  GPS_SemaphorHandle = xSemaphoreCreateCounting(GPS_ARRAY_SIZE,1);

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
  xTaskCreate(threadA,     "Task A",       512, NULL, tskIDLE_PRIORITY + 4, &Handle_aTask);
  xTaskCreate(threadB,     "Task B",       512, NULL, tskIDLE_PRIORITY + 3, &Handle_bTask);
  //xTaskCreate(taskMonitor, "Task Monitor", 256, NULL, tskIDLE_PRIORITY + 1, &Handle_monitorTask);
  xTaskCreate(GPSthread,  "GPS Task", 256, NULL, tskIDLE_PRIORITY + 2, &Handle_gpsTask);

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

