// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>

#include "robot.h"
#include "StateEstimator.h"
#include "Storage.h"
#include "Stats.h"
#include "LineTracker.h"
#include "comm.h"
#include "src/op/op.h"
#include "RunningMedian.h"
#include "pinman.h"
#include "ble.h"
#include "motor.h"
#include "src/driver/AmRobotDriver.h"
#include "src/driver/MpuDriver.h"
#include "battery.h"
#include "gps.h"
#include "src/ublox/ublox.h"
#include "helper.h"
#include "buzzer.h"
#include "map.h"
#include "config.h"
#include "reset.h"
#include "cpu.h"
#include "i2c.h"
#include "bumper.h"
#include "events.h"

// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};


  MpuDriver imuDriver;

  AmRobotDriver robotDriver;
  AmMotorDriver motorDriver;
  AmBatteryDriver batteryDriver;
  AmBumperDriver bumperDriver;
  AmStopButtonDriver stopButton;
  AmBuzzerDriver buzzerDriver;
Motor motor;
Battery battery;
PinManager pinMan;

  UBLOX gps;

BLEConfig bleConfig;
Buzzer buzzer;
Bumper bumper;
Map maps;
TimeTable timetable;

int stateButton = 0;  
int stateButtonTemp = 0;
unsigned long stateButtonTimeout = 0;

OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor

unsigned long controlLoops = 0;
String stateOpText = "";  // current operation as text
String gpsSolText = ""; // current gps solution as text
float stateTemp = 20; // degreeC
//float stateHumidity = 0; // percent
unsigned long stateInMotionLastTime = 0;
bool stateChargerConnected = false;
bool stateInMotionLP = false; // robot is in angular or linear motion? (with motion low-pass filtering)

unsigned long lastFixTime = 0;
int fixTimeout = 0;
bool absolutePosSource = false;
double absolutePosSourceLon = 0;
double absolutePosSourceLat = 0;
float lastGPSMotionX = 0;
float lastGPSMotionY = 0;
unsigned long nextGPSMotionCheckTime = 0;

bool finishAndRestart = false;

unsigned long nextBadChargingContactCheck = 0;
unsigned long linearMotionStartTime = 0;
unsigned long angularMotionStartTime = 0;
unsigned long overallMotionTimeout = 0;
unsigned long nextControlTime = 0;
unsigned long lastComputeTime = 0;

unsigned long nextLedTime = 0;
unsigned long nextImuTime = 0;
unsigned long nextTempTime = 0;
unsigned long imuDataTimeout = 0;
unsigned long nextSaveTime = 0;
unsigned long nextTimetableTime = 0;
unsigned long nextGenerateGGATime = 0;

//##################################################################################
unsigned long loopTime = millis();
int loopTimeNow = 0;
int loopTimeMax = 0;
float loopTimeMean = 0;
int loopTimeMin = 99999;
unsigned long loopTimeTimer = 0;
String psOutput = "";
unsigned long wdResetTimer = millis();
//##################################################################################

int motorErrorCounter = 0;

// must be defined to override default behavior
void watchdogSetup (void){} 


// reset linear motion measurement
void resetLinearMotionMeasurement(){
  linearMotionStartTime = millis();  
  //stateGroundSpeed = 1.0;
}

// reset angular motion measurement
void resetAngularMotionMeasurement(){
  angularMotionStartTime = millis();
}

// reset overall motion timeout
void resetOverallMotionTimeout(){
  overallMotionTimeout = millis() + 10000;      
}

void updateGPSMotionCheckTime(){
  nextGPSMotionCheckTime = millis() + GPS_MOTION_DETECTION_TIMEOUT * 1000;     
}



void sensorTest(){
  CONSOLE.println("testing sensors for 60 seconds...");
  unsigned long stopTime = millis() + 60000;  
  unsigned long nextMeasureTime = 0;
  while (millis() < stopTime){
    bumper.run();
    robotDriver.run();   
    if (millis() > nextMeasureTime){
      nextMeasureTime = millis() + 1000;      
      if (BUMPER_ENABLE){
        CONSOLE.print("bumper (left,right,triggered,nearObstacle): ");
        CONSOLE.print(((int)bumper.testLeft()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.testRight()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.obstacle()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.nearObstacle()));        
        CONSOLE.print("\t");       
      } 
      CONSOLE.println();  
      watchdogReset();
    }
  }
  CONSOLE.println("end of sensor test - please ignore any IMU/GPS errors");
}

// check for RTC module
bool checkAT24C32() {
  byte b = 0;
  int r = 0;
  unsigned int address = 0;
  Wire.beginTransmission(AT24C32_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(AT24C32_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(AT24C32_ADDRESS, 1);
      while (Wire.available() > 0 && r < 1) {        
        b = (byte)Wire.read();        
        r++;
      }
    }
  }
  
    return (r == 1);
  
}


void outputConfig(){
  #ifdef ENABLE_PASS
    CONSOLE.println("ENABLE_PASS");
  #endif 

  CONSOLE.print("FREEWHEEL_IS_AT_BACKSIDE: ");
  CONSOLE.println(FREEWHEEL_IS_AT_BACKSIDE);
  CONSOLE.print("WHEEL_BASE_CM: ");
  CONSOLE.println(WHEEL_BASE_CM);
  CONSOLE.print("WHEEL_DIAMETER: ");
  CONSOLE.println(WHEEL_DIAMETER);

  CONSOLE.print("ENABLE_ODOMETRY_ERROR_DETECTION: ");
  CONSOLE.println(ENABLE_ODOMETRY_ERROR_DETECTION);
  CONSOLE.print("TICKS_PER_REVOLUTION: ");
  CONSOLE.println(TICKS_PER_REVOLUTION);
  
  CONSOLE.print("MOTOR_FAULT_CURRENT: ");
  CONSOLE.println(MOTOR_FAULT_CURRENT);
  CONSOLE.print("MOTOR_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOTOR_OVERLOAD_CURRENT);
  CONSOLE.print("USE_LINEAR_SPEED_RAMP: ");
  CONSOLE.println(USE_LINEAR_SPEED_RAMP);
  CONSOLE.print("MOTOR_PID_KP: ");
  CONSOLE.println(MOTOR_PID_KP);
  CONSOLE.print("MOTOR_PID_KI: ");
  CONSOLE.println(MOTOR_PID_KI);
  CONSOLE.print("MOTOR_PID_KD: ");
  CONSOLE.println(MOTOR_PID_KD);
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    CONSOLE.println("MOTOR_LEFT_SWAP_DIRECTION");
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    CONSOLE.println("MOTOR_RIGHT_SWAP_DIRECTION");
  #endif
  #ifdef MAX_MOW_PWM
    CONSOLE.print("MAX_MOW_PWM: ");
    CONSOLE.println(MAX_MOW_PWM);
  #endif
  CONSOLE.print("MOW_FAULT_CURRENT: ");
  CONSOLE.println(MOW_FAULT_CURRENT);
  CONSOLE.print("MOW_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOW_OVERLOAD_CURRENT);
  CONSOLE.print("ENABLE_OVERLOAD_DETECTION: ");
  CONSOLE.println(ENABLE_OVERLOAD_DETECTION);
  CONSOLE.print("ENABLE_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_FAULT_DETECTION);
  CONSOLE.print("ENABLE_FAULT_OBSTACLE_AVOIDANCE: ");
  CONSOLE.println(ENABLE_FAULT_OBSTACLE_AVOIDANCE);
  CONSOLE.print("ENABLE_RPM_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_RPM_FAULT_DETECTION);
  CONSOLE.print("BUMPER_ENABLE: ");
  CONSOLE.println(BUMPER_ENABLE);  
  CONSOLE.print("BUMPER_DEADTIME: ");
  CONSOLE.println(BUMPER_DEADTIME);
  CONSOLE.print("BUMPER_TRIGGER_DELAY: ");
  CONSOLE.println(BUMPER_TRIGGER_DELAY);
  CONSOLE.print("BUMPER_MAX_TRIGGER_TIME: ");
  CONSOLE.println(BUMPER_MAX_TRIGGER_TIME);  
  CONSOLE.print("CURRENT_FACTOR: ");
  CONSOLE.println(CURRENT_FACTOR);
  CONSOLE.print("GO_HOME_VOLTAGE: ");
  CONSOLE.println(GO_HOME_VOLTAGE);
  CONSOLE.print("BAT_FULL_VOLTAGE: ");
  CONSOLE.println(BAT_FULL_VOLTAGE);
  CONSOLE.print("BAT_FULL_CURRENT: ");
  CONSOLE.println(BAT_FULL_CURRENT);
  CONSOLE.print("BAT_SWITCH_OFF_IDLE: ");
  CONSOLE.println(BAT_SWITCH_OFF_IDLE);
  CONSOLE.print("BAT_SWITCH_OFF_UNDERVOLTAGE: ");
  CONSOLE.println(BAT_SWITCH_OFF_UNDERVOLTAGE);
  CONSOLE.print("REQUIRE_VALID_GPS: ");
  CONSOLE.println(REQUIRE_VALID_GPS);
  CONSOLE.print("GPS_SPEED_DETECTION: ");
  CONSOLE.println(GPS_SPEED_DETECTION);
  CONSOLE.print("GPS_MOTION_DETECTION: ");
  CONSOLE.println(GPS_MOTION_DETECTION);
  CONSOLE.print("GPS_REBOOT_RECOVERY: ");
  CONSOLE.println(GPS_REBOOT_RECOVERY);
  CONSOLE.print("GPS_CONFIG: ");
  CONSOLE.println(GPS_CONFIG);
  CONSOLE.print("GPS_CONFIG_FILTER: ");
  CONSOLE.println(GPS_CONFIG_FILTER);
  CONSOLE.print("CPG_CONFIG_FILTER_MINELEV: ");
  CONSOLE.println(CPG_CONFIG_FILTER_MINELEV);
  CONSOLE.print("CPG_CONFIG_FILTER_NCNOTHRS: ");
  CONSOLE.println(CPG_CONFIG_FILTER_NCNOTHRS);
  CONSOLE.print("CPG_CONFIG_FILTER_CNOTHRS: ");
  CONSOLE.println(CPG_CONFIG_FILTER_CNOTHRS);
  CONSOLE.print("ALLOW_ROUTE_OUTSIDE_PERI_METER: ");
  CONSOLE.println(ALLOW_ROUTE_OUTSIDE_PERI_METER);
  CONSOLE.print("OBSTACLE_DETECTION_ROTATION: ");
  CONSOLE.println(OBSTACLE_DETECTION_ROTATION);
  CONSOLE.print("DOCKING_STATION: ");
  CONSOLE.println(DOCKING_STATION);
  CONSOLE.print("DOCK_IGNORE_GPS: ");
  CONSOLE.println(DOCK_IGNORE_GPS);
  CONSOLE.print("DOCK_AUTO_START: ");
  CONSOLE.println(DOCK_AUTO_START);
  CONSOLE.print("TARGET_REACHED_TOLERANCE: ");
  CONSOLE.println(TARGET_REACHED_TOLERANCE);
  CONSOLE.print("STANLEY_CONTROL_P_NORMAL: ");
  CONSOLE.println(STANLEY_CONTROL_P_NORMAL);
  CONSOLE.print("STANLEY_CONTROL_K_NORMAL: ");
  CONSOLE.println(STANLEY_CONTROL_K_NORMAL);
  CONSOLE.print("STANLEY_CONTROL_P_SLOW: ");
  CONSOLE.println(STANLEY_CONTROL_P_SLOW);
  CONSOLE.print("STANLEY_CONTROL_K_SLOW: ");
  CONSOLE.println(STANLEY_CONTROL_K_SLOW);
  CONSOLE.print("BUTTON_CONTROL: ");
  CONSOLE.println(BUTTON_CONTROL);
  #ifdef BUZZER_ENABLE
    CONSOLE.println("BUZZER_ENABLE");    
  #endif
}


// robot start routine
void start(){    
  loopTime = millis();
  pinMan.begin();         
  // keep battery switched ON
  batteryDriver.begin();  
  CONSOLE.begin(CONSOLE_BAUDRATE);    
  buzzerDriver.begin();
  buzzer.begin();      
    
  Wire.begin();      
  analogReadResolution(12);  // configure ADC 12 bit resolution
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout){
    if (!checkAT24C32()){
      CONSOLE.println(F("PCB not powered ON or RTC module missing"));      
      I2Creset();  
      Wire.begin();    
      #ifdef I2C_SPEED
        Wire.setClock(I2C_SPEED);     
      #endif
    } else break;
  }  
  
  // give Arduino IDE users some time to open serial console to actually see very first console messages
 delay(1500);
  
  
  logResetCause();
  
  CONSOLE.println(VER);          
  CONSOLE.print("compiled for: ");
  CONSOLE.println(BOARD);
  
  robotDriver.begin();
  CONSOLE.print("robot id: ");
  String rid = "";
  robotDriver.getRobotID(rid);
  CONSOLE.println(rid);
  motorDriver.begin();;  
  battery.begin();      
  stopButton.begin();

  bleConfig.run();   
      
  motor.begin();

  bumper.begin();
 

  outputConfig();      
  
  CONSOLE.print("SERIAL_BUFFER_SIZE=");
  CONSOLE.print(SERIAL_BUFFER_SIZE);
  CONSOLE.println(" (increase if you experience GPS checksum errors)");
  //CONSOLE.println("-----------------------------------------------------");
  //CONSOLE.println("NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:");
  //CONSOLE.println("1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom");
  //CONSOLE.println("2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h");
  //CONSOLE.println("   for Grand Central M4 'packages/adafruit/hardware/samd/xxxxx/cores/arduino/RingBuffer.h");  
  //CONSOLE.println("change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024");
  CONSOLE.println("-----------------------------------------------------");
  watchdogEnable(20000L);  
  
 
  gps.begin(GPS, GPS_BAUDRATE);   

  maps.begin();      
  
  startIMU(false);        
  
  buzzer.sound(SND_READY);  
  battery.resetIdle();        
  loadState();

}



// should robot move?
bool robotShouldMove(){
  /*CONSOLE.print(motor.linearSpeedSet);
  CONSOLE.print(",");
  CONSOLE.println(motor.angularSpeedSet / PI * 180.0);  */
  return ( fabs(motor.linearSpeedSet) > 0.001 );
}


bool robotShouldMoveForward(){
   return ( motor.linearSpeedSet > 0.001 );
}

// should robot rotate?
bool robotShouldRotate(){
  return ( (fabs(motor.linearSpeedSet) < 0.001) &&  (fabs(motor.angularSpeedSet) > 0.001) );
}

// should robot be in motion? NOTE: function ignores very short motion pauses (with motion low-pass filtering)
bool robotShouldBeInMotion(){  
  if (robotShouldMove() || (robotShouldRotate())) {
    stateInMotionLastTime = millis();
    stateInMotionLP = true;    
  }
  if (millis() > stateInMotionLastTime + 2000) {
    stateInMotionLP = false;
  }
  return stateInMotionLP;
}


// drive reverse if robot cannot move forward
void triggerObstacle(){
  activeOp->onObstacle();
}


// detect sensor malfunction
void detectSensorMalfunction(){  
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if (motor.odometryError){
      CONSOLE.println("odometry error!");    
      activeOp->onOdometryError();
      return;      
    }
  }
  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > 20000){
      // one motor is taking too much current over a long time (too high gras etc.) and we should stop mowing
      CONSOLE.println("overload!");    
      activeOp->onMotorOverload();
      return;
    }  
  }
  if (ENABLE_FAULT_OBSTACLE_AVOIDANCE){
    // there is a motor error (either unrecoverable fault signal or a malfunction) and we should try an obstacle avoidance
    if (motor.motorError){
      CONSOLE.println("motor error!");
      activeOp->onMotorError();
      return;      
    }  
  }
}

// returns true, if obstacle detected, otherwise false
bool detectObstacle(){   
  if (! ((robotShouldMoveForward()) || (robotShouldRotate())) ) return false;        

  if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (bumper.obstacle()) ){  
    CONSOLE.println("bumper obstacle!");    
    Logger.event(EVT_BUMPER_OBSTACLE);
    statMowBumperCounter++;
    triggerObstacle();    
    return true;
  }
   
  // check if GPS motion (obstacle detection)  
  if ((millis() > nextGPSMotionCheckTime) || (millis() > overallMotionTimeout)) {        
    updateGPSMotionCheckTime();
    resetOverallMotionTimeout(); // this resets overall motion timeout (overall motion timeout happens if e.g. 
    // motion between anuglar-only and linar-only toggles quickly, and their specific timeouts cannot apply due to the quick toggling)
    float dX = lastGPSMotionX - stateX;
    float dY = lastGPSMotionY - stateY;
    float delta = sqrt( sq(dX) + sq(dY) );    
    if (delta < 0.05){
      if (GPS_MOTION_DETECTION){
        if (stateLocalizationMode == LOC_GPS) {
          CONSOLE.println("gps no motion => obstacle!");
          Logger.event(EVT_NO_ROBOT_MOTION_OBSTACLE);    
          statMowGPSMotionTimeoutCounter++;
          triggerObstacle();
          return true;
        }
      }
    }
    lastGPSMotionX = stateX;      
    lastGPSMotionY = stateY;      
  }    
  return false;
}

// stuck rotate avoidance (drive forward if robot cannot rotate)
void triggerObstacleRotation(){
  activeOp->onObstacleRotation();
}

// stuck rotate detection (e.g. robot cannot due to an obstacle outside of robot rotation point)
// returns true, if stuck detected, otherwise false
bool detectObstacleRotation(){  
  if (!robotShouldRotate()) {
    return false;
  }  
  if (!OBSTACLE_DETECTION_ROTATION) return false; 
  if (millis() > angularMotionStartTime + 15000) { // too long rotation time (timeout), e.g. due to obstacle
    CONSOLE.println("too long rotation time (timeout) for requested rotation => assuming obstacle");
    Logger.event(EVT_ANGULAR_MOTION_TIMEOUT_OBSTACLE);
    statMowRotationTimeoutCounter++;
    triggerObstacleRotation();
    return true;
  }
  /*if (BUMPER_ENABLE){
    if (millis() > angularMotionStartTime + 500) { // FIXME: do we actually need a deadtime here for the freewheel sensor?        
      if (bumper.obstacle()){  
        CONSOLE.println("bumper obstacle!");    
        statMowBumperCounter++;
        triggerObstacleRotation();    
        return true;
      }
    }
  }*/
  if (imuDriver.imuFound){
    if (millis() > angularMotionStartTime + 3000) {                  
      if (fabs(stateDeltaSpeedLP) < 3.0/180.0 * PI){ // less than 3 degree/s yaw speed, e.g. due to obstacle
        CONSOLE.println("no IMU rotation speed detected for requested rotation => assuming obstacle");    
        statMowImuNoRotationSpeedCounter++;
        Logger.event(EVT_IMU_NO_ROTATION_OBSTACLE);    
        triggerObstacleRotation();
        return true;      
      }
    }
    if (diffIMUWheelYawSpeedLP > 10.0/180.0 * PI) {  // yaw speed difference between wheels and IMU more than 8 degree/s, e.g. due to obstacle
      CONSOLE.println("yaw difference between wheels and IMU for requested rotation => assuming obstacle");            
      statMowDiffIMUWheelYawSpeedCounter++;
      Logger.event(EVT_IMU_WHEEL_DIFFERENCE_OBSTACLE);            
      triggerObstacleRotation();
      return true;            
    }    
  }
  return false;
}




// robot main loop
void run(){  
  
  robotDriver.run();
  buzzer.run();
  buzzerDriver.run();
  stopButton.run();
  battery.run();
  batteryDriver.run();
  motorDriver.run();
  motor.run();
  maps.run();  
  bumper.run();
  
  // state saving
  if (millis() >= nextSaveTime){  
    nextSaveTime = millis() + 5000;
    saveState();
  }
  
  // IMU
  if (millis() > nextImuTime){
    int ims = 750 / IMU_FIFO_RATE;
    nextImuTime = millis() + ims;        
    //imu.resetFifo();    
    if (imuIsCalibrating) {
      activeOp->onImuCalibration();             
    } else {
      readIMU();    
    }
  }

  // LED states
  if (millis() > nextLedTime){
    nextLedTime = millis() + 1000;
    robotDriver.ledStateGpsFloat = (gps.solution == SOL_FLOAT);
    robotDriver.ledStateGpsFix = (gps.solution == SOL_FIXED);
    robotDriver.ledStateError = (stateOp == OP_ERROR);            
  }

  gps.run();

  if (millis() > nextTimetableTime){
    nextTimetableTime = millis() + 30000;
    gps.decodeTOW();
    timetable.setCurrentTime(gps.hour, gps.mins, gps.dayOfWeek);
    timetable.run();
  }

  calcStats();  
  
  
  if (millis() >= nextControlTime){        
    nextControlTime = millis() + 20; 
    controlLoops++;    
    
    computeRobotState();
    if (!robotShouldMove()){
      resetLinearMotionMeasurement();
      updateGPSMotionCheckTime();  
    }
    if (!robotShouldRotate()){
      resetAngularMotionMeasurement();
    }
    if (!robotShouldBeInMotion()){
      resetOverallMotionTimeout();
      lastGPSMotionX = 0;
      lastGPSMotionY = 0;
    }

    /*if (gpsJump) {
      // gps jump: restart current operation from new position (restart path planning)
      CONSOLE.println("restarting operation (gps jump)");
      gpsJump = false;
      motor.stopImmediately(true);
      setOperation(stateOp, true);    // restart current operation
    }*/
    
    if (battery.chargerConnected() != stateChargerConnected) {    
      stateChargerConnected = battery.chargerConnected(); 
      if (stateChargerConnected){      
        // charger connected event        
        activeOp->onChargerConnected();                
      } else {
        activeOp->onChargerDisconnected();
      }            
    }
    if (millis() > nextBadChargingContactCheck) {
      if (battery.badChargerContact()){
        nextBadChargingContactCheck = millis() + 60000; // 1 min.
        activeOp->onBadChargingContactDetected();
      }
    } 

    if (battery.underVoltage()){
      activeOp->onBatteryUndervoltage();
    } 
    else {         
      if (battery.shouldGoHome()){
        if (DOCKING_STATION){
           activeOp->onBatteryLowShouldDock();
        }
      }   
       
      if (battery.chargerConnected()){
        if (battery.chargingHasCompleted()){
          activeOp->onChargingCompleted();
        }
      }        
    } 
    activeOp->checkStop();
    activeOp->run();     
      
    // process button state
    if (stateButton == 5){
      stateButton = 0; // reset button state
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_DOCK, false);
    } else if (stateButton == 6){ 
      stateButton = 0; // reset button state        
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_MOW, false);
    } 
    //else if (stateButton > 0){  // stateButton 1 (or unknown button state)        
    else if (stateButton == 1){  // stateButton 1                   
      stateButton = 0;  // reset button state
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_IDLE, false);                             
    } else if (stateButton == 9){
      stateButton = 0;  // reset button state
      stateSensor = SENS_STOP_BUTTON;
      cmdSwitchOffRobot();
    } else if (stateButton == 12){
      stateButton = 0; // reset button state
      stateSensor = SENS_STOP_BUTTON;
    }

    // update operation type      
    stateOp = activeOp->getGoalOperationType();  
            
  }   // if (millis() >= nextControlTime)
    
  // ----- read serial input (BT/console) -------------
  processComm();
  outputConsole();    

  //##############################################################################

  if(millis() > wdResetTimer + 1000){
    watchdogReset();
  }   

  //CONSOLE.println(millis());
  //CONSOLE.println(loopTime); 
  loopTimeNow = millis() - loopTime;
  //CONSOLE.println(loopTimeNow);  
  //delay(5000);  
  loopTimeMin = min(loopTimeNow, loopTimeMin); 
  loopTimeMax = max(loopTimeNow, loopTimeMax);
  loopTimeMean = 0.99 * loopTimeMean + 0.01 * loopTimeNow; 
  loopTime = millis();

  if(millis() > loopTimeTimer + 10000){
    if(loopTimeMax > 500){
      CONSOLE.print("WARNING - LoopTime: ");
    }else{
      CONSOLE.print("Info - LoopTime(ms) now=");
    }
    CONSOLE.print(loopTimeNow);
    CONSOLE.print(" min=");
    CONSOLE.print(loopTimeMin);
    CONSOLE.print(" mean=");
    CONSOLE.print(loopTimeMean);
    CONSOLE.print(" max=");
    CONSOLE.print(loopTimeMax);
    CONSOLE.println();
    if (psOutput != "") CONSOLE.println(psOutput);

    loopTimeMin = 99999; 
    loopTimeMax = 0;
    psOutput = "";
    loopTimeTimer = millis();
  }   
  //##############################################################################

  // compute button state (stateButton)
  if (BUTTON_STOP){  // should we use the stop/emergency button?
    bool buttonTriggered = stopButton.triggered();
    if (BUTTON_INVERT) buttonTriggered = !buttonTriggered; 
    if (buttonTriggered){
      if ((stateOp != OP_IDLE) && (stateOp != OP_CHARGE)) {   // if not in idle or charge state
        // stop all pendings actions if button pressed 
        CONSOLE.println("BUTTON triggered, going IDLE");
        stateSensor = SENS_STOP_BUTTON;  
        setOperation(OP_IDLE, false);  // go into idle-state
      } 
      if (BUTTON_CONTROL){     
        // additional button features (start mowing, docking etc.)
        if (millis() > stateButtonTimeout){
          stateButtonTimeout = millis() + 1000;
          stateButtonTemp++; // next state
          buzzer.sound(SND_READY, true);
          CONSOLE.print("BUTTON ");
          CONSOLE.print(stateButtonTemp);
          CONSOLE.println("s");                                     
        }
      }                          
    } else {
      if (stateButtonTemp > 0){
        // button released => set stateButton
        stateButtonTimeout = 0;
        stateButton = stateButtonTemp;
        stateButtonTemp = 0;
        CONSOLE.print("stateButton ");
        CONSOLE.println(stateButton);
      }
    }
  }    
}        



// set new robot operation
void setOperation(OperationType op, bool allowRepeat){  
  if ((stateOp == op) && (!allowRepeat)) return;  
  CONSOLE.print("setOperation op=");
  CONSOLE.println(op);
  stateOp = op;  
  activeOp->changeOperationTypeByOperator(stateOp);
  saveState();
}
