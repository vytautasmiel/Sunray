// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#ifndef ROBOT_H
#define ROBOT_H

#include "motor.h"
#include "config.h"
#include "src/driver/AmRobotDriver.h"
#include "src/driver/MpuDriver.h"
#include "battery.h"
#include "ble.h"
#include "pinman.h"
#include "bumper.h"
#include "buzzer.h"
#include "map.h"   
#include "src/ublox/ublox.h"
#include "timetable.h"


#define VER "Sunray,1.0.324"

// operation types
enum OperationType {
      OP_IDLE,      // idle
      OP_MOW,       // mowing
      OP_CHARGE,    // charging
      OP_ERROR,     // serious error
      OP_DOCK,      // go to docking
};    

// sensor errors
enum Sensor {
      SENS_NONE,              // no error
      SENS_BAT_UNDERVOLTAGE,  // battery undervoltage
      SENS_OBSTACLE,          // obstacle triggered
      SENS_GPS_FIX_TIMEOUT,   // gps fix timeout
      SENS_IMU_TIMEOUT,       // imu timeout  
      SENS_IMU_TILT,          // imut tilt
      SENS_KIDNAPPED,         // robot has been kidnapped (is no longer on planned track)
      SENS_OVERLOAD,          // motor overload
      SENS_MOTOR_ERROR,       // motor error
      SENS_GPS_INVALID,       // gps is invalid or not working
      SENS_ODOMETRY_ERROR,    // motor odometry error
      SENS_MAP_NO_ROUTE,      // robot cannot find a route to next planned point
      SENS_MEM_OVERFLOW,      // cpu memory overflow
      SENS_BUMPER,            // bumper triggered
      SENS_SONAR,             // ultrasonic triggered
      SENS_LIFT,              // lift triggered
      SENS_RAIN,              // rain sensor triggered
      SENS_STOP_BUTTON,       // emergency/stop button triggered
      SENS_TEMP_OUT_OF_RANGE, // temperature out-of-range triggered
};


  #define FILE_CREATE  (O_WRITE | O_CREAT)

extern OperationType stateOp; // operation
extern Sensor stateSensor; // last triggered sensor
extern String stateOpText;  // current operation as text
extern String gpsSolText; // current gps solution as text
extern int stateButton;  // button state
extern float stateTemp;  // current temperature

extern float setSpeed; // linear speed (m/s)
extern int fixTimeout;
extern bool finishAndRestart; // auto-restart when mowing finished?
extern bool absolutePosSource;
extern double absolutePosSourceLon;
extern double absolutePosSourceLat;

extern unsigned long linearMotionStartTime;
extern unsigned long angularMotionStartTime;
extern bool stateInMotionLP; // robot is in angular or linear motion? (with motion low-pass filtering)

extern unsigned long lastFixTime;

extern bool hasClient;

extern unsigned long controlLoops;
extern int motorErrorCounter;

  extern AmRobotDriver robotDriver;
  extern AmMotorDriver motorDriver;
  extern AmBatteryDriver batteryDriver;
  extern AmBumperDriver bumperDriver;
  extern AmStopButtonDriver stopButton;
  extern AmBuzzerDriver buzzerDriver;
  extern MpuDriver imuDriver;

extern Motor motor;
extern Battery battery;
extern BLEConfig bleConfig;
extern Bumper bumper;
extern Buzzer buzzer;
extern PinManager pinMan;
extern Map maps;
extern TimeTable timetable;

  extern UBLOX gps;

int freeMemory();
void start();
void run();
void setOperation(OperationType op, bool allowRepeat = false);
void triggerObstacle();
void sensorTest();
void updateStateOpText();
void detectSensorMalfunction();
bool detectObstacle();
bool detectObstacleRotation();



#endif
