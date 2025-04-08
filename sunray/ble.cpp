// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


// Bluetooth BLE/4.0 module (HC-08/HM-10 ZS-040)
// https://www.marotronics.de/HM-10-Bluetooth-BLE-BT-40-CC2541-CC2540-fuer-Arduino-iOS-Android-Low-Energy
// docs:  http://denethor.wlu.ca/arduino/MLT-BT05-AT-commands-TRANSLATED.pdf

// to send/receive data from this module, use BLE characteristic write/read:  
// UART_CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb";



#include "ble.h"
#include <Arduino.h>
#include "config.h"
#include "robot.h"
#include "comm.h"

bool bleConnected = false;
unsigned long bleConnectedTimeout = 0;


String BLEConfig::read(){
  String res;    
  unsigned long timeout = millis() + 2000;  
  while (millis() < timeout){
    while (BLE.available()){
      timeout = millis() + 200; 
      char ch = BLE.read();
      CONSOLE.print(ch);
      res += ch;
    }
  }
  return res;
}

String BLEConfig::exec(String cmd, bool doRetry){
  String res;
  delay(500);    
  for (int retry=0; retry < 3; retry++){
    CONSOLE.print("BLE: ");
    CONSOLE.print(cmd);
    BLE.print(cmd);
    res = read();
    if ((res != "") || (!doRetry)) break;
    CONSOLE.print("retry ");
    CONSOLE.print(retry+1);
    CONSOLE.print("  ");
  }
  return res;
}

void BLEConfig::run(){   
  BLE.begin(BLE_BAUDRATE);    
  return;
}


// process Bluetooth input
void processBLE(){
  char ch;   
  if (BLE.available()){
    battery.resetIdle();  
    bleConnected = true;
    bleConnectedTimeout = millis() + 5000;
    while ( BLE.available() ){    
      ch = BLE.read();      
      if ((ch == '\r') || (ch == '\n')) {   
        #ifdef VERBOSE
          CONSOLE.print("BLE:");     
          CONSOLE.println(cmd);        
        #endif
        processCmd(true, true);              
        BLE.print(cmdResponse);    
        cmd = "";
      } else if (cmd.length() < 500){
        cmd += ch;
      }
    }    
  } else {
    if (millis() > bleConnectedTimeout){
      bleConnected = false;
    }
  }  
}  
