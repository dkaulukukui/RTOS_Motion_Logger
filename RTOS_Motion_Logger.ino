//**************************************************************************
// FreeRtos on Samd21
// By Scott Briscoe
//
// Project is a simple example of how to get FreeRtos running on a SamD21 processor
// Project can be used as a template to build your projects off of as well
//
//**************************************************************************

//#include <FreeRTOS_SAMD21.h>
#include "RTOS_Motion_Logger.h"

//**************************************************************************
void setReports(void) {
  bno08x.enableReport(SH2_ACCELEROMETER);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  //bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
  //bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  //bno08x.enableReport(SH2_GRAVITY);
  //bno08x.enableReport(SH2_ROTATION_VECTOR);
  //bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR);
  //bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  //bno08x.enableReport(SH2_RAW_ACCELEROMETER);
  //bno08x.enableReport(SH2_RAW_GYROSCOPE);
  //bno08x.enableReport(SH2_RAW_MAGNETOMETER);
}

//**************************************************************************


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
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
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

      BNO_error = 0;
      xSemaphoreGive(BNO_Data_SemaphorHandle);  //signal new data
      BNO_Head_Q = BNO_Head_Q < (BNO_ARRAY_SIZE -1) ? BNO_Head_Q +1 : 0; //advance fifo index

    }



  }

}

//*****************************************************************
// Create a thread that prints out B to the screen every second
// this task will run forever
//*****************************************************************
static void threadB( void *pvParameters )  //Data printing task
{
  SERIAL.println("Thread B: Started");

  while(1) { 
    const uint8_t BUFLEN = 160;
    char buf[BUFLEN] = "";

    xSemaphoreTake(BNO_Data_SemaphorHandle, portMAX_DELAY);  // wait for next data record
    BNO_DATA *bno_data = &BNO_Array[BNO_Tail_Q];

    const uint8_t FLT_STR_LEN = 10;
    char Gyro_X[FLT_STR_LEN] = ""; 
    char Gyro_Y[FLT_STR_LEN] = "";
    char Gyro_Z[FLT_STR_LEN] = "";
    char Accel_X[FLT_STR_LEN] = ""; 
    char Accel_Y[FLT_STR_LEN] = "";
    char Accel_Z[FLT_STR_LEN] = "";


    dtostrf(bno_data->Gyro_X, 5,2, Gyro_X);
    dtostrf(bno_data->Gyro_Y, 5,2, Gyro_Y);
    dtostrf(bno_data->Gyro_Z, 5,2, Gyro_Z);
    dtostrf(bno_data->Accel_X, 5,2, Accel_X);
    dtostrf(bno_data->Accel_Y, 5,2, Accel_Y);
    dtostrf(bno_data->Accel_Z, 5,2, Accel_Z);

    strcat(buf, Gyro_X);
    strcat(buf, "\t");
    strcat(buf, Gyro_Y);
    strcat(buf, "\t");
    strcat(buf, Gyro_Z);
    strcat(buf, "\t");
    strcat(buf, Accel_X);
    strcat(buf, "\t");
    strcat(buf, Accel_Y);
    strcat(buf, "\t");
    strcat(buf, Accel_Z);

    //snprintf(buf, BUFLEN,"%d\t%d\t%d",bno_data->RawGyro_X, bno_data->RawGyro_Y, bno_data->RawGyro_Z);

    SERIAL.println(buf);

    xSemaphoreGive(BNO_Space_SemaphorHandle);
    BNO_Tail_Q = BNO_Tail_Q < (BNO_ARRAY_SIZE-1) ? BNO_Tail_Q +1 :0;

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
  xTaskCreate(threadB,     "Task B",       256, NULL, tskIDLE_PRIORITY + 2, &Handle_bTask);
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

