/* 
    BuPiZu Sketch --  This sketch listens for commands from serial1. the commands are either motor commands
                      or a command to send a sensor report. Sensor report includes raw readings from IMU 
                      sensors, as well as a bool flag indicating if a puck is in the gripper or not. 
    ------------------------------------------------------------------------------------------------

    Nicholi Shiell
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include <string.h>

/* Serial port baud rate */
#define BAUDRATE     57600

/* Variable initialization */

Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
LSM303 compass;
L3G gyro;
Zumo32U4ProximitySensors proxSensors;

String cmdString;
bool robotOn = false;
bool sendReportFlag = false;
bool continuousReportFlag = true;
int motorLinearValue = 50;
int motorRotateValue = 100;
char report[120];
char currentCmd = 'S';
int nLeft = 0;
int nRight = 0;
int nFront = 0;

int counter = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// SETUP FUNC. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void setup() {
   Wire.begin();
    Serial1.begin(BAUDRATE);
    Serial1.setTimeout(10);
    
  if (!compass.init()){
    // Failed to detect the compass.
    ledRed(1);
    while(1){
      Serial1.println(F("Failed to detect the compass."));
      delay(100);
    }
  }

  compass.enableDefault();

  if (!gyro.init())
  {
    // Failed to detect the gyro.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect gyro."));
      delay(100);
    }
  }
  
  proxSensors.initThreeSensors();

  randomSeed(analogRead(0));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// This function interperts the command string recieved over serial1.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int runCommand() {
  
  char *cmd = new char[cmdString.length() + 1];
  strcpy(cmd, cmdString.c_str());
 
  // Bounce back msgs
  //Serial1.println("Recieved Command: " + cmdString);
  
  int i = 0;
  if(cmd[i] == 'F'){
    motors.setSpeeds(motorLinearValue, motorLinearValue);
  }
  else if(cmd[i] == 'B'){
    motors.setSpeeds(-1*motorLinearValue, -1*motorLinearValue);
  }
  else if(cmd[i] == 'R'){
    motors.setSpeeds(motorRotateValue, 0/*-1*motorRotateValue*/);
    //Serial.println("Right!");
  }
    else if(cmd[i] == 'E'){
    motors.setSpeeds(75, -75);
    //Serial.println("Right!");
  }
  else if(cmd[i] == 'L'){
    motors.setSpeeds(/*-1*motorRotateValue*/0, motorRotateValue);
    //Serial.println("Left!");
  } 
    else if(cmd[i] == 'K'){
    motors.setSpeeds(-75, 75);
    //Serial.println("Left!");
  } 
  else if(cmd[i] == 'S'){
    motors.setSpeeds(0, 0);
    //Serial.println("Stop!");
  }
  else if(cmd[i] == 'T'){
    sendReportFlag = true;
  } 
  else if(cmd[i] == 'C'){
    if(continuousReportFlag) continuousReportFlag = false;
    else continuousReportFlag = true;
  } 
  else{
    //Serial.println("Huh.");
    motors.setSpeeds(0,0);    
  }
  
  delete [] cmd;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// This code is the main loop. It is only called  after the A button has been pushed on the zumo.
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void RobotOn(){
  
  // Check if a command has been recieved over serial1.
  while (Serial1.available() > 0) {
    cmdString = Serial1.readString();
    runCommand();     
  }

  // If the command was to send a sensor report then do it!
  if( sendReportFlag ||  continuousReportFlag  ){
      
      /*snprintf_P(report, sizeof(report),
                  PSTR("A: %6d %6d %6d    M: %6d %6d %6d    G: %6d %6d %6d    P: %d %d %d"),
                  compass.a.x, compass.a.y, compass.a.z,
                  compass.m.x, compass.m.y, compass.m.z,
                  gyro.g.x, gyro.g.y, gyro.g.z,
                  nFront, nLeft, nRight);*/
      
      // Report string is comma seperated with the following order of the data.
      // a.x a.y a.z m.x m.y m.z p.f p.l p.r 
      snprintf_P(report, sizeof(report),
                  PSTR("%d"),
                  //,%d,%d,%d,%d,%d,%d,%d,%d
                  //if nfront < 12 then the robot has a puck
                  nFront
                  //, proxSensors.countsFrontWithLeftLeds(), proxSensors.countsFrontWithRightLeds(),
                  //nLeft, proxSensors.countsLeftWithLeftLeds(), proxSensors.countsLeftWithRightLeds(),
                  //nRight, proxSensors.countsRightWithLeftLeds(), proxSensors.countsRightWithRightLeds()
                  );
      
      
      Serial1.println(report); 
      
      sendReportFlag = false;
  }
   
}

void ReadProxSensors(){
  // put your main code here, to run repeatedly:
    proxSensors.read();
    nLeft = proxSensors.countsLeftWithLeftLeds()+proxSensors.countsLeftWithRightLeds();   
    nFront = proxSensors.countsFrontWithLeftLeds()+proxSensors.countsFrontWithRightLeds();  
    nRight = proxSensors.countsRightWithLeftLeds()+proxSensors.countsRightWithRightLeds();
}

void loop() {
  counter++;
  if(counter > 10){
    counter = 0;
  }
  
  if(buttonA.getSingleDebouncedPress()){
      if(robotOn) robotOn = false;
      else robotOn = true;
      sendReportFlag = true;
  }

  // If the robot is "off" stop the motors and send a msg over serial!
  if(!robotOn){
      motors.setSpeeds(0, 0);
      snprintf_P(report, sizeof(report), PSTR("DEACTIVATED. PRESS 'A' BUTTON ACTIVATE."));
      Serial1.println(report); 
  }
  
  // If robot is "on" do stuff
  if (robotOn){

    ReadProxSensors(); 
    RobotOn();
  }
  
  delay(10);
}


