#include "src/esp32-cam-webserver/esp32-cam-webserver.h"
#include "src/readingIMUdata/readingIMUdata.h"

#define WEBCAM_SERVER

// define two tasks for Blink & AnalogRead
#ifdef WEBCAM_SERVER
void TaskSetupCamWebserver( void *pvParameters );
#endif

void TaskSetupReadingIMUdata( void *pvParameters );

void setup() {

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  
  // Now set up two tasks to run independently.
  #ifdef WEBCAM_SERVER
  xTaskCreatePinnedToCore(
    TaskSetupCamWebserver
    ,  "TaskSetupCamWebserver"   // A name just for humans
    ,  1024*10  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0); //ARDUINO_RUNNING_CORE
    #endif

  xTaskCreatePinnedToCore(
    TaskSetupReadingIMUdata
    ,  "TaskSetupReadingIMUdata"
    ,  1024*2  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  1); //ARDUINO_RUNNING_CORE

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
    
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10000);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

#ifdef WEBCAM_SERVER
void TaskSetupCamWebserver(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize task setup
  setupCamWebserver();

  for (;;) // A Task shall never return or exit.
  {
    loopCamWebserver();
  }
}
#endif

void TaskSetupReadingIMUdata(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  setupReadingIMUdata();

  for (;;)
  {
    loopReadingIMUdata();
    //vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}
