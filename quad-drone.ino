#include "src/esp32-cam-webserver/esp32-cam-webserver.h"
#include "src/readingIMUdata/readingIMUdata.h"
#include "src/Arduino-PID-Library/PID_v1.h"

//define WEBCAM_SERVER

// define two tasks for Blink & AnalogRead
#ifdef WEBCAM_SERVER
void TaskSetupCamWebserver( void *pvParameters );
#endif

void TaskSetupReadingIMUdata( void *pvParameters );

void TaskPIDController( void *pvParameters );

//Define Variables we'll be connecting to
// The setpoints and inputs come in degrees, the output goes from 0 to 255 for PWM
double yawSetpoint, yawInput, yawOutput; 
double pitchSetpoint, pitchInput,pitchOutput;
double rollSetpoint, rollInput, rollOutput;

//Specify the links and initial tuning parameters
double yawKp   = 0, yawKi   = 0, yawKd   = 0;
double pitchKp = 2, pitchKi = 5, pitchKd = 1;
double rollKp  = 2, rollKi  = 5, rollKd  = 1;

PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawKp, yawKi, yawKd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, rollKp, rollKi, rollKd, DIRECT);

float yaw, pitch, roll;

int motor1, motor2, motor3, motor4;

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
    ,  1024*4  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  1); //ARDUINO_RUNNING_CORE


   xTaskCreatePinnedToCore(
    TaskPIDController
    ,  "TaskPIDController"
    ,  1024*4  // Stack size
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

  float pitchOffset = -1.35;
  float rollOffset = -.15;
  int counter = 0;

  for (;;)
  {
    loopReadingIMUdata(&yaw, &pitch, &roll);
    yawInput = yaw;
    pitchInput = pitch + pitchOffset;
    rollInput = roll + rollOffset;
    
//    if(counter == 10000){
//    Serial.print("Orientation: ");
//    Serial.print(yaw);
//    Serial.print(", ");
//    Serial.print(pitchInput);
//    Serial.print(", ");
//    Serial.println(rollInput);
//    }
//    else {
//      counter ++;
//    }

    
    
    //vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}


void TaskPIDController(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize task setup

  //HARDCODED!!!!!!!!!!!!!
  yawSetpoint = 0;
  pitchSetpoint = 0;
  rollSetpoint = 0;

  motor1 = 0;
  motor2 = 0;
  motor3 = 0;
  motor4 = 0;

  //Simulate turning on the drone
  float throttle = 100;

  int counter = 0;


  //turn the PID on
  yawPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  yawPID.SetOutputLimits(-50, 50);
  pitchPID.SetOutputLimits(-50, 50);
  rollPID.SetOutputLimits(-50, 50);

  for (;;) // A Task shall never return or exit.
  {
    yawPID.Compute();
    pitchPID.Compute();
    rollPID.Compute();

  //if(yawPID.Compute() && pitchPID.Compute() && rollPID.Compute()){
  //    Serial.print("PID-Outputs: ");
  //    Serial.print(yawOutput);
  //    Serial.print(", ");
  //    Serial.print(pitchOutput);
  //    Serial.print(", ");
  //    Serial.println(rollOutput);
  //    Serial.println();
  
      //Calculate motor power output
      motor1 = throttle + yawOutput + pitchOutput - rollOutput;
      motor2 = throttle - yawOutput + pitchOutput + rollOutput;
      motor3 = throttle - yawOutput - pitchOutput - rollOutput;
      motor4 = throttle + yawOutput - pitchOutput + rollOutput;
      
      if(motor1 < 0){
        motor1 = 0;
      }
      else if(motor1 > 255){
        motor1 = 0;
      }
      
      
      if(motor2 < 0){
        motor2 = 0;
      }
      else if(motor2 > 255){
        motor2 = 0;
      }
      
      
      if(motor3 < 0){
        motor3 = 0;
      }
      else if(motor3 > 255){
        motor3 = 0;
      }
      
      
      if(motor4 < 0){
        motor4 = 0;
      }
      else if(motor4 > 255){
        motor4 = 0;
      }

      
  if(counter == 1000){
      Serial.print("Motor-Outputs: ");
      Serial.print(motor1);
      Serial.print(", ");
      Serial.print(motor2);
      Serial.print(", ");
      Serial.print(motor3);
      Serial.print(", ");
      Serial.println(motor4);

      //counter reset
      counter = 0;
  }
  else {
    counter ++;
  }

    //vTaskDelay(10);
    
//  }
  }
}
