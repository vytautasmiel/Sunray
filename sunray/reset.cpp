#include "reset.h"
#include "config.h"
#include <Arduino.h>

#if defined(__SAMD51__)


// C:\Users\alex\AppData\Local\Arduino15\packages\arduino\tools\CMSIS-Atmel\1.2.0\CMSIS\Device\ATMEL\samd51\include\component\rstc.h
#pragma push_macro("WDT")
#undef WDT    // Required to be able to use '.bit.WDT'. Compiler wrongly replace struct field with WDT define
ResetCause getResetCause() {
  RSTC_RCAUSE_Type resetCause;  
  resetCause.reg = REG_RSTC_RCAUSE;
  if (resetCause.bit.POR)                                   return RST_POWER_ON;
  else if (resetCause.bit.EXT)                              return RST_EXTERNAL;
  else if (resetCause.bit.BODCORE || resetCause.bit.BODVDD) return RST_BROWN_OUT;
  else if (resetCause.bit.WDT)                              return RST_WATCHDOG;
  else if (resetCause.bit.SYST || resetCause.bit.NVM)       return RST_SOFTWARE;
  else if (resetCause.bit.BACKUP)                           return RST_BACKUP;
  return RST_UNKNOWN;
}
#pragma pop_macro("WDT")


#else
ResetCause getResetCause() {  
      // Read the Reset Controller Status Register
      uint32_t reset_status = RSTC->RSTC_SR;

      // Extract the reset type value using the Mask and Position definitions
      uint32_t reset_type_value = (reset_status & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;
  
      Serial.print("Last reset reason code: ");
      Serial.print(reset_type_value);
      Serial.print(" - ");

      switch (reset_type_value) {
        case 0:
          Serial.println("General Reset (Power-on / Brown-out / User Button)");
          break;
        case 1:
          Serial.println("Backup Reset");
          break;
        case 2:
          Serial.println("Watchdog Reset");
          break;
        case 3:
          Serial.println("Software Reset");
          break;
        case 4:
           // As noted before, this might appear as General Reset (0) after bootloader.
          Serial.println("User Reset (NRST Pin - may show as General)");
          break;
        default:
          Serial.print("Unknown Reset Type (Status Reg: 0x");
          Serial.print(reset_status, HEX);
          Serial.println(")");
          break;
      }
  
  return RST_UNKNOWN;
} 
   
#endif


void logResetCause(){
  switch (getResetCause()){
    case RST_UNKNOWN: CONSOLE.println("unknown"); break;
    case RST_POWER_ON : CONSOLE.println("power-on"); break;
    case RST_EXTERNAL : CONSOLE.println("external"); break;
    case RST_BROWN_OUT : CONSOLE.println("brown-out"); break;
    case RST_WATCHDOG : CONSOLE.println("watchdog"); break;
    case RST_SOFTWARE : CONSOLE.println("software"); break;
    case RST_BACKUP: CONSOLE.println("backup"); break;
  }
}


// get free memory
// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int freeMemory() {

  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));

}

