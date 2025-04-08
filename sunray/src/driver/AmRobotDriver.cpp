// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)




#include "AmRobotDriver.h"
#include "../../config.h"
#include "../../helper.h"
#include "../../robot.h"
#include "../../pinman.h"
#include "../../cpu.h"
#include "../../ArduinoUniqueID.h"

#if defined(_SAM3XA_)
  #include "../due/DueTimer.h"
#else
  #include "../agcm4/Adafruit_ZeroTimer.h"    // __SAMD51__
#endif


//#define SUPER_SPIKE_ELIMINATOR 1  // advanced spike elimination  (experimental, comment out to disable)


volatile int odomTicksLeft  = 0;
volatile int odomTicksRight = 0;
volatile int odomTicksMow = 0;

volatile unsigned long motorLeftTicksTimeout = 0;
volatile unsigned long motorRightTicksTimeout = 0;
volatile unsigned long motorMowTicksTimeout = 0;

volatile unsigned long motorLeftTransitionTime = 0;
volatile unsigned long motorRightTransitionTime = 0;
volatile unsigned long motorMowTransitionTime = 0;

volatile float motorLeftDurationMax = 0;
volatile float motorRightDurationMax = 0;
volatile float motorMowDurationMax = 0;

volatile bool leftPressed = false;
volatile bool rightPressed = false;


volatile boolean tone_pin_state = false;


void toneHandler(){  
  digitalWrite(pinBuzzer, tone_pin_state= !tone_pin_state);  
}


#if defined(__SAMD51__)
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(3);

void TC3_Handler() {
  Adafruit_ZeroTimer::timerHandler(3);
}
#endif 




void AmRobotDriver::begin(){
  CONSOLE.println("using robot driver: AmRobotDriver");
}



void AmRobotDriver::run(){
}


bool AmRobotDriver::getRobotID(String &id){
  id = "";
  for(size_t i = 0; i < UniqueIDsize; i++){
      byte v = UniqueID[i];
      if (v <= 0xF) id += F("0");
      id += String(v, HEX);  
      if (i + 1 < UniqueIDsize) id += ":";
  }
  return true;
}

bool AmRobotDriver::getMcuFirmwareVersion(String &name, String &ver){
  name = "XX";
  ver = "XX";
  return true;
}

float AmRobotDriver::getCpuTemperature(){
  return GetCPUTemperature();  
}


// ------------------------------------------------------------------------------------


// odometry signal change interrupt

void OdometryMowISR(){			  
  if (digitalRead(pinMotorMowRpm) == LOW) return;
  if (millis() < motorMowTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorMowTransitionTime;
    if (duration > 5) duration = 0;
    motorMowTransitionTime = millis();
    motorMowDurationMax = 0.7 * max(motorMowDurationMax, ((float)duration));
    motorMowTicksTimeout = millis() + motorMowDurationMax;
  #else
    motorMowTicksTimeout = millis() + 1;
  #endif
  odomTicksMow++;    
}


void OdometryLeftISR(){			  
  if (digitalRead(pinOdometryLeft) == LOW) return;
  if (millis() < motorLeftTicksTimeout) return; // eliminate spikes  
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorLeftTransitionTime;
    if (duration > 5) duration = 0;
    motorLeftTransitionTime = millis();
    motorLeftDurationMax = 0.7 * max(motorLeftDurationMax, ((float)duration));
    motorLeftTicksTimeout = millis() + motorLeftDurationMax;
  #else
    motorLeftTicksTimeout = millis() + 1;
  #endif
  odomTicksLeft++;    
}

void OdometryRightISR(){			
  if (digitalRead(pinOdometryRight) == LOW) return;  
  if (millis() < motorRightTicksTimeout) return; // eliminate spikes
  #ifdef SUPER_SPIKE_ELIMINATOR
    unsigned long duration = millis() - motorRightTransitionTime;
    if (duration > 5) duration = 0;  
    motorRightTransitionTime = millis();
    motorRightDurationMax = 0.7 * max(motorRightDurationMax, ((float)duration));  
    motorRightTicksTimeout = millis() + motorRightDurationMax;
  #else
    motorRightTicksTimeout = millis() + 1;
  #endif
  odomTicksRight++;        

}


AmMotorDriver::AmMotorDriver(){

  // default values for all motor drivers (for parameters description, see AmRobotDriver.h)
  
  // MC33926 (https://www.nxp.com/docs/en/data-sheet/MC33926.pdf) - PwmFreqMax=20 khz
  MC33926.driverName = "MC33926";
  MC33926.forwardPwmInvert = false;
  MC33926.forwardDirLevel = LOW;
  MC33926.reversePwmInvert = true;
  MC33926.reverseDirLevel = HIGH;
  MC33926.usePwmRamp = false;
  MC33926.faultActive = LOW;
  MC33926.resetFaultByToggleEnable = true;
  MC33926.enableActive = HIGH;
  MC33926.disableAtPwmZeroSpeed=false;  
  MC33926.keepPwmZeroSpeed = true;
  MC33926.minPwmSpeed = 0;
  MC33926.maxPwmSpeed = 255;
  MC33926.pwmFreq = PWM_FREQ_3900;
  MC33926.adcVoltToAmpOfs = 0;
  MC33926.adcVoltToAmpScale = 1.905 * 2; // ADC voltage to amp for 2 drivers connected in parallel
  MC33926.adcVoltToAmpPow = 1.0;

  // your custom brushed/brushless driver (ACT-8015A, JYQD_V7.3E3, etc.)
  CUSTOM.driverName = "CUSTOM";    // just a name for your driver
  CUSTOM.forwardPwmInvert = false; // invert PWM signal for forward? (false or true)
  CUSTOM.forwardDirLevel = LOW;    // logic level for forward (LOW or HIGH)
  CUSTOM.reversePwmInvert = false; // invert PWM signal for reverse? (false or true)
  CUSTOM.reverseDirLevel = HIGH;   // logic level for reverse (LOW or HIGH)
  CUSTOM.usePwmRamp = false;       // use a ramp to get to PWM value?    
  CUSTOM.faultActive = LOW;        // fault active level (LOW or HIGH)
  CUSTOM.resetFaultByToggleEnable = false; // reset a fault by toggling enable? 
  CUSTOM.enableActive = LOW;       // enable active level (LOW or HIGH)
  CUSTOM.disableAtPwmZeroSpeed=false;  // disable driver at PWM zero speed? (brake function)
  CUSTOM.keepPwmZeroSpeed = false;  // keep PWM zero value (disregard minPwmSpeed at zero speed)?
  CUSTOM.minPwmSpeed = 0;          // minimum PWM speed your driver can operate
  CUSTOM.maxPwmSpeed = 255;          
  CUSTOM.pwmFreq = PWM_FREQ_29300;  // choose between PWM_FREQ_3900 and PWM_FREQ_29300 here   
  CUSTOM.adcVoltToAmpOfs = -1.65;      // ADC voltage to amps (offset)        // brushless-adapter: 0A=1.65V, resolution 132mV/A
  CUSTOM.adcVoltToAmpScale = 7.57; // ADC voltage to amps (scale)
  CUSTOM.adcVoltToAmpPow = 1.0;    // ADC voltage to amps (power of number)
}
    

void AmMotorDriver::begin(){      


    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS: no");    
    mowDriverChip = MC33926;
    gearsDriverChip = MC33926;



  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);
  pinMode(pinMotorLeftSense, INPUT);
  pinMode(pinMotorLeftFault, INPUT);

  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT);
  pinMode(pinMotorRightSense, INPUT);
  pinMode(pinMotorRightFault, INPUT);

  // mower motor
  pinMode(pinMotorMowDir, OUTPUT);
  pinMode(pinMotorMowPWM, OUTPUT);
  pinMode(pinMotorMowSense, INPUT);
  pinMode(pinMotorMowRpm, INPUT);
  pinMode(pinMotorMowRpm, INPUT_PULLUP);  
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, mowDriverChip.enableActive);
  pinMode(pinMotorMowFault, INPUT);

  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);

  pinMode(pinOdometryRight, INPUT_PULLUP);




  // enable interrupts
  attachInterrupt(pinOdometryLeft, OdometryLeftISR, CHANGE);  
  attachInterrupt(pinOdometryRight, OdometryRightISR, CHANGE);  
  attachInterrupt(pinMotorMowRpm, OdometryMowISR, CHANGE);  
    
	//pinMan.setDebounce(pinOdometryLeft, 100);  // reject spikes shorter than usecs on pin
	//pinMan.setDebounce(pinOdometryRight, 100);  // reject spikes shorter than usecs on pin	
  
  leftSpeedSign = rightSpeedSign = mowSpeedSign = 1;
  lastRightPwm = lastLeftPwm = lastMowPwm = 0;
}


void AmMotorDriver::run(){
}

// brushed/brushless motor driver
//(8-bit PWM=255, 10-bit PWM=1023)
// example logic:
//   IN1 PinPWM         IN2 PinDir
//   PWM                L     Forward
//   PWM                H     Reverse

void AmMotorDriver::setMotorDriver(int pinDir, int pinPWM, int speed, DriverChip &chip, int speedSign) {
  //DEBUGLN(speed);
  bool reverse = (speedSign < 0);    
  
  if ((speed == 0) && (chip.keepPwmZeroSpeed)) {
    // driver does not require periodic signal at zero speed, we can output 'silence' for zero speed    
  } else {
    // verhindert dass das PWM Signal 0 wird. Der Driver braucht einen kurzen Impuls um das PWM zu erkennen.
    // Wenn der z.B. vom max. PWM Wert auf 0 bzw. das Signal auf Low geht, behält er den vorherigen Wert bei und der Motor stoppt nicht
    if (abs(speed) < chip.minPwmSpeed) speed = chip.minPwmSpeed * speedSign;
    if (abs(speed) > chip.maxPwmSpeed) speed = chip.maxPwmSpeed * speedSign;  
  }

  if (reverse) {  

    digitalWrite(pinDir, chip.reverseDirLevel) ;
    if (chip.reversePwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM

  } else {

    digitalWrite(pinDir, chip.forwardDirLevel) ;
    if (chip.forwardPwmInvert) 
      pinMan.analogWrite(pinPWM, 255 - ((byte)abs(speed)), chip.pwmFreq);  // nPWM (inverted pwm)
    else 
      pinMan.analogWrite(pinPWM, ((byte)abs(speed)), chip.pwmFreq);       // PWM
  }  
}
    
void AmMotorDriver::setMotorPwm(int leftPwm, int rightPwm, int mowPwm){  
  // remember speed sign during zero-transition
  if (leftPwm < 0) leftSpeedSign = -1;
  if (leftPwm > 0) leftSpeedSign = 1;
  if (rightPwm < 0) rightSpeedSign = -1;
  if (rightPwm > 0) rightSpeedSign = 1;
  if (mowPwm < 0) mowSpeedSign = -1;
  if (mowPwm > 0) mowSpeedSign = 1;   
  
  // limit pwm to ramp if required
  if (gearsDriverChip.usePwmRamp){
    int deltaLeftPwm = leftPwm-lastLeftPwm;
    leftPwm = leftPwm + min(1, max(-1, deltaLeftPwm));    
    int deltaRightPwm = rightPwm-lastRightPwm;
    rightPwm = rightPwm + min(1, max(-1, deltaRightPwm));    
  }
  if (mowDriverChip.usePwmRamp){
    int deltaMowPwm = mowPwm-lastMowPwm;
    mowPwm = mowPwm + min(1, max(-1, deltaMowPwm));      
  }  

  // remember last PWM values
  lastLeftPwm = leftPwm;  
  lastRightPwm = rightPwm;
  lastMowPwm = mowPwm;

  // apply motor PWMs
  setMotorDriver(pinMotorLeftDir, pinMotorLeftPWM, leftPwm, gearsDriverChip, leftSpeedSign);
  setMotorDriver(pinMotorRightDir, pinMotorRightPWM, rightPwm, gearsDriverChip, rightSpeedSign);
  setMotorDriver(pinMotorMowDir, pinMotorMowPWM, mowPwm, mowDriverChip, mowSpeedSign);
  
  // disable driver at zero speed (brake function)    
  bool enableGears = gearsDriverChip.enableActive;
  bool enableMow = mowDriverChip.enableActive;  
  if (gearsDriverChip.disableAtPwmZeroSpeed){  
    if ((leftPwm == 0) && (rightPwm == 0)){
      enableGears = !gearsDriverChip.enableActive;                
    }
    digitalWrite(pinMotorEnable, enableGears);
  }
  if (mowDriverChip.disableAtPwmZeroSpeed){ 
    if (mowPwm == 0) {
      if (mowDriverChip.disableAtPwmZeroSpeed){
        enableMow = !mowDriverChip.enableActive;
      }
    }      
    digitalWrite(pinMotorMowEnable, enableMow);
  }  
}


void AmMotorDriver::getMotorFaults(bool &leftFault, bool &rightFault, bool &mowFault){ 
  if (digitalRead(pinMotorLeftFault) == gearsDriverChip.faultActive) {
    leftFault = true;
  }
  if  (digitalRead(pinMotorRightFault) == gearsDriverChip.faultActive) {
    rightFault = true;
  }
  if (digitalRead(pinMotorMowFault) == mowDriverChip.faultActive) {
    mowFault = true;
  }
}

void AmMotorDriver::resetMotorFaults(){  
  if (digitalRead(pinMotorLeftFault) == gearsDriverChip.faultActive) {
    if (gearsDriverChip.resetFaultByToggleEnable){
      digitalWrite(pinMotorEnable, !gearsDriverChip.enableActive);
      digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
    }
  }
  if  (digitalRead(pinMotorRightFault) == gearsDriverChip.faultActive) {
    if (gearsDriverChip.resetFaultByToggleEnable){
      digitalWrite(pinMotorEnable, !gearsDriverChip.enableActive);
      digitalWrite(pinMotorEnable, gearsDriverChip.enableActive);
    }
  }
  if (digitalRead(pinMotorMowFault) == mowDriverChip.faultActive) {
    if (mowDriverChip.resetFaultByToggleEnable){
      digitalWrite(pinMotorMowEnable, !mowDriverChip.enableActive);
      digitalWrite(pinMotorMowEnable, mowDriverChip.enableActive);
    }
  }
}

void AmMotorDriver::getMotorCurrent(float &leftCurrent, float &rightCurrent, float &mowCurrent){
  // current (amps)= ((ADCvoltage + ofs)^pow) * scale
  float ValuePosCheck	= 0;
  ValuePosCheck = (((float)ADC2voltage(analogRead(pinMotorLeftSense))) + gearsDriverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  leftCurrent = pow(
      ValuePosCheck, gearsDriverChip.adcVoltToAmpPow
      )  * gearsDriverChip.adcVoltToAmpScale;

  ValuePosCheck = (((float)ADC2voltage(analogRead(pinMotorRightSense))) + gearsDriverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  rightCurrent = pow(
      ValuePosCheck, gearsDriverChip.adcVoltToAmpPow
      )  * gearsDriverChip.adcVoltToAmpScale;

  ValuePosCheck = (((float)ADC2voltage(analogRead(pinMotorMowSense))) + gearsDriverChip.adcVoltToAmpOfs);
  if (ValuePosCheck < 0) ValuePosCheck = 0;	// avoid negativ numbers
  mowCurrent = pow(
            ValuePosCheck, mowDriverChip.adcVoltToAmpPow
      )  * mowDriverChip.adcVoltToAmpScale;
}

void AmMotorDriver::getMotorEncoderTicks(int &leftTicks, int &rightTicks, int &mowTicks){
  leftTicks = odomTicksLeft;
  rightTicks = odomTicksRight;  
  mowTicks = odomTicksMow;
  // reset counters
  odomTicksLeft = odomTicksRight = odomTicksMow = 0;
}    




// ------------------------------------------------------------------------------------


// --- battery switch off circuit --------------------
// JP8 Dauer-ON : automatic switch off circuit disabled
// JP8 Autom.   : automatic switch off circuit enabled
// Note: to increase hardware switch-off time increase capacitor C12  (under DC/DC module)


void AmBatteryDriver::begin(){
  // keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);    
  digitalWrite(pinBatterySwitch, HIGH);  
  batteryFactor = (100+10) / 10;    // ADC voltage to battery voltage

  //INA169:  A precision amplifier measures the voltage across the Rs=0.1 ohm, 1% sense resistor. 
  //The Rs is rated for 2W continuous so you can measure up to +5A continuous. 
  //The output is a current that is drawn through the on-board RL=10K+10K=20K resistors so that the 
  // output voltage is 2V per Amp. So for 1A draw, the output will be 2V. You can change the 
  // load resistor RL to be smaller by soldering the bridge If you solder the bridge (RL=10K resistor) 
  // you'll get 1V per Amp.   
  //
  // Is = Vout * 1k / (Rs * RL)

  // PCB1.3 (external INA module)
  //   a) bridged      RL=10K:    Is = 1V * 1k / (0.1*10K)  = 1A
  //   b) non-bridged  RL=20k:    Is = 1V * 1k / (0.1*20K)  = 0.5A
  // PCB1.4 (INA soldered on main PCB)
  //   a) bridged      RL=6.8K:   Is = 1V * 1k / (0.05*6.8K)  = 2.941A
  //   b) non-bridged  RL=10.1k:  Is = 1V * 1k / (0.05*10.1K)  = 1.98A
  
  currentFactor = CURRENT_FACTOR;         // ADC voltage to current ampere  (0.5 for non-bridged)

  pinMode(pinChargeRelay, OUTPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinChargeVoltage, INPUT);
  pinMode(pinChargeCurrent, INPUT);  
}


void AmBatteryDriver::run(){
}

    
float AmBatteryDriver::getBatteryVoltage(){
  float voltage = ((float)ADC2voltage(analogRead(pinBatteryVoltage))) * batteryFactor;
  return voltage;  
}

float AmBatteryDriver::getChargeVoltage(){
  float voltage = ((float)ADC2voltage(analogRead(pinChargeVoltage))) * batteryFactor;
  return voltage;
}


float AmBatteryDriver::getChargeCurrent(){    
  float amps = ((float)ADC2voltage(analogRead(pinChargeCurrent))) * currentFactor;    
	return amps;
}

void AmBatteryDriver::enableCharging(bool flag){
  digitalWrite(pinChargeRelay, flag);      
}

void AmBatteryDriver::keepPowerOn(bool flag){
  digitalWrite(pinBatterySwitch, flag);
}


// ------------------------------------------------------------------------------------
void BumperLeftInterruptRoutine(){
  leftPressed = (digitalRead(pinBumperLeft) == LOW);  
}

void BumperRightInterruptRoutine(){
  rightPressed = (digitalRead(pinBumperRight) == LOW);  
}


void AmBumperDriver::begin(){	
  pinMode(pinBumperLeft, INPUT_PULLUP);                   
  pinMode(pinBumperRight, INPUT_PULLUP);                   
  attachInterrupt(pinBumperLeft, BumperLeftInterruptRoutine, CHANGE);
	attachInterrupt(pinBumperRight, BumperRightInterruptRoutine, CHANGE);
}

void AmBumperDriver::getTriggeredBumper(bool &leftBumper, bool &rightBumper){
  leftBumper = leftPressed;
  rightBumper = rightPressed;
}

bool AmBumperDriver::nearObstacle(){
  return false;
}

bool AmBumperDriver::obstacle(){
  return (leftPressed || rightPressed);
}
    
bool AmBumperDriver::getLeftBumper(){
  return (leftPressed);
}

bool AmBumperDriver::getRightBumper(){
  return (rightPressed);
}

void AmBumperDriver::run(){  
}


// ------------------------------------------------------------------------------------


void AmStopButtonDriver::begin(){
  nextControlTime = 0;
  pressed = false;  
  pinMode(pinButton, INPUT_PULLUP);  
}

void AmStopButtonDriver::run(){
  unsigned long t = millis();
  if (t < nextControlTime) return;
  nextControlTime = t + 100;                                       // save CPU resources by running at 10 Hz
  pressed = (digitalRead(pinButton)== LOW);
}

bool AmStopButtonDriver::triggered(){
  return pressed;
}


void AmBuzzerDriver::begin(){  
  pinMode(pinBuzzer, OUTPUT);                
  digitalWrite(pinBuzzer, LOW);
}

void AmBuzzerDriver::run(){  
}

void AmBuzzerDriver::noTone(){  
  #ifdef _SAM3XA_
    Timer1.stop();  
    digitalWrite(pinBuzzer, LOW);
  #elif __SAMD51__  // __SAMD51__
    //::noTone(pinBuzzer);     
    zerotimer.enable(false);
    digitalWrite(pinBuzzer, LOW);
  #endif     
}

void AmBuzzerDriver::tone(int freq){  
  #ifdef _SAM3XA_
    pinMode(pinBuzzer, OUTPUT);
    Timer1.attachInterrupt(toneHandler).setFrequency(freq).start();   
  #elif __SAMD51__      // __SAMD51__
    //::tone(pinBuzzer, freq);    

    // Set up the flexible divider/compare
    uint8_t divider  = 1;
    uint16_t compare = 0;
    tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;
    
    divider = 16;
    prescaler = TC_CLOCK_PRESCALER_DIV16;
    compare = (48000000/16)/freq;   
    
    zerotimer.enable(false);
    zerotimer.configure(prescaler,       // prescaler
            TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
            TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
            );

    zerotimer.setCompare(0, compare);
    zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, toneHandler);
    zerotimer.enable(true);
  #endif     
}