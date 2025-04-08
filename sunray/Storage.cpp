// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "Storage.h"
#include "StateEstimator.h"
#include "robot.h"
#include "map.h"
#include "config.h"
#include "reset.h"
#include <Arduino.h>

double stateCRC = 0;


double calcStateCRC(){
 return (stateOp *10 + maps.mowPointsIdx + maps.dockPointsIdx + maps.freePointsIdx + ((byte)maps.wayMode) 
   + fixTimeout + setSpeed +
   + ((byte)absolutePosSource) + absolutePosSourceLon + absolutePosSourceLat + motor.pwmMaxMow 
   + ((byte)finishAndRestart) + ((byte)motor.motorMowForwardSet) + ((byte)battery.docked)
   + timetable.crc() );
}


void dumpState(){
  CONSOLE.print("dumpState: ");
  CONSOLE.print(" X=");
  CONSOLE.print(stateX);
  CONSOLE.print(" Y=");
  CONSOLE.print(stateY);
  CONSOLE.print(" delta=");
  CONSOLE.print(stateDelta);
  CONSOLE.print(" mapCRC=");
  CONSOLE.print(maps.mapCRC);
  CONSOLE.print(" mowPointsIdx=");
  CONSOLE.print(maps.mowPointsIdx);
  CONSOLE.print(" dockPointsIdx=");
  CONSOLE.print(maps.freePointsIdx);
  CONSOLE.print(" freePointsIdx=");
  CONSOLE.print(maps.freePointsIdx);
  CONSOLE.print(" wayMode=");
  CONSOLE.print(maps.wayMode);
  CONSOLE.print(" op=");
  CONSOLE.print(stateOp);
  CONSOLE.print(" sensor=");
  CONSOLE.print(stateSensor);
  CONSOLE.print(" fixTimeout=");
  CONSOLE.print(fixTimeout);
  CONSOLE.print(" absolutePosSource=");
  CONSOLE.print(absolutePosSource);
  CONSOLE.print(" lon=");
  CONSOLE.print(absolutePosSourceLon);
  CONSOLE.print(" lat=");
  CONSOLE.print(absolutePosSourceLat);
  CONSOLE.print(" pwmMaxMow=");
  CONSOLE.print(motor.pwmMaxMow);
  CONSOLE.print(" finishAndRestart=");
  CONSOLE.print(finishAndRestart);
  CONSOLE.print(" motorMowForwardSet=");
  CONSOLE.println(motor.motorMowForwardSet);  
}

void updateStateOpText(){
  switch (stateOp){
    case OP_IDLE: stateOpText = "idle"; break;
    case OP_MOW: stateOpText = "mow"; break;
    case OP_CHARGE: stateOpText = "charge"; break;
    case OP_ERROR: 
      stateOpText = "error (";
      switch (stateSensor){
        case SENS_NONE: stateOpText += "none)"; break;
        case SENS_BAT_UNDERVOLTAGE: stateOpText += "unvervoltage)"; break;            
        case SENS_OBSTACLE: stateOpText += "obstacle)"; break;      
        case SENS_GPS_FIX_TIMEOUT: stateOpText += "fix timeout)"; break;
        case SENS_IMU_TIMEOUT: stateOpText += "imu timeout)"; break;
        case SENS_IMU_TILT: stateOpText += "imu tilt)"; break;
        case SENS_KIDNAPPED: stateOpText += "kidnapped)"; break;
        case SENS_OVERLOAD: stateOpText += "overload)"; break;
        case SENS_MOTOR_ERROR: stateOpText += "motor error)"; break;
        case SENS_GPS_INVALID: stateOpText += "gps invalid)"; break;
        case SENS_ODOMETRY_ERROR: stateOpText += "odo error)"; break;
        case SENS_MAP_NO_ROUTE: stateOpText += "no map route)"; break;
        case SENS_MEM_OVERFLOW: stateOpText += "mem overflow)"; break;
        case SENS_BUMPER: stateOpText += "bumper)"; break;
        case SENS_SONAR: stateOpText += "sonar)"; break;
        case SENS_LIFT: stateOpText += "lift)"; break;
        case SENS_RAIN: stateOpText += "rain)"; break;
        case SENS_STOP_BUTTON: stateOpText += "stop button)"; break;
        default: stateOpText += "unknown)"; break;
      }
      break;
    case OP_DOCK: stateOpText = "dock"; break;
    default: stateOpText = "unknown"; break;
  }
  switch (gps.solution){
    case SOL_INVALID: gpsSolText = "invalid"; break;
    case SOL_FLOAT: gpsSolText = "float"; break;
    case SOL_FIXED: gpsSolText ="fixed"; break;
    default: gpsSolText = "unknown";      
  }
}


bool loadState(){
  return true;
}


bool saveState(){   
  bool res = true;
  return res; 
}



