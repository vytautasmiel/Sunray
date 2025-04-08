// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "bumper.h"
#include "config.h"
#include "robot.h"
#include <Arduino.h>

volatile bool inputLeftPressed = false;
volatile bool inputRightPressed = false;

volatile bool outputLeftPressed = false;
volatile bool outputRightPressed = false;

unsigned long leftPressedOnDelay = 0; // on delay timer (BUMPER_TRIGGER_DELAY) for the bumper inputs
unsigned long rightPressedOnDelay = 0;

unsigned long bumperStayActivTime = 0;    // duration, the bumper stays triggered
unsigned long lastCallBumperObstacle = 0; // last call for bumper.obstacle


void Bumper::begin(){
  bumperDriver.begin();

  leftPressedOnDelay  = millis();
  rightPressedOnDelay = millis();
}

void Bumper::run() {
  bumperDriver.run();
    
  inputLeftPressed   = bumperDriver.getLeftBumper();
  inputRightPressed  = bumperDriver.getRightBumper();

    outputLeftPressed = inputLeftPressed;
    outputRightPressed = inputRightPressed;  
    lastCallBumperObstacle = millis();

}


bool Bumper::obstacle(){
    return (outputLeftPressed || outputRightPressed);
}

bool Bumper::nearObstacle(){
  return bumperDriver.nearObstacle();
}

// send separated signals without delay to sensortest
bool Bumper::testLeft(){
  return (inputLeftPressed);
}

bool Bumper::testRight(){
  return (inputRightPressed);
}
