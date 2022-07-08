#ifndef DEBUG_LEVEL_H
#define DEBUG_LEVEL_H

#define rColor "\e[0m"
#define gColor "\e[1;32m"
#define eColor "\e[1;31m"
#define wColor "\e[1;33m"
#define iColor "\e[1;34m"
#define dColor "\e[1;36m"

#if DEBUG_L == 3
  #define DEBUG_LEVEL
#elif DEBUG_L == 2
  #define INFO_LEVEL
#elif DEBUG_L == 1
  #define WARNING_LEVEL
#elif DEBUG_L == 0
  #define ERROR_LEVEL
#endif

#ifdef ERROR_LEVEL
  #define printMessage(x) Serial.print(gColor + String(x) + rColor)
  #define printMessageLn(x) Serial.println(gColor + String(x) + rColor)
  #define printError(x) Serial.print(eColor + String(x) + rColor)
  #define printErrorLn(x) Serial.println(eColor + String(x) + rColor)
  #define printWarning(x) 
  #define printWarningLn(x) 
  #define printInfo(x) 
  #define printInfoLn(x) 
  #define printDebug(x) 
  #define printDebugLn(x) 
#elif defined(WARNING_LEVEL)
  #define printMessage(x) Serial.print(gColor + String(x) + rColor)
  #define printMessageLn(x) Serial.println(gColor + String(x) + rColor)
  #define printError(x) Serial.print(eColor + String(x) + rColor)
  #define printErrorLn(x) Serial.println(eColor + String(x) + rColor)
  #define printWarning(x) Serial.print(wColor + String(x) + rColor)
  #define printWarningLn(x) Serial.println(wColor + String(x) + rColor)
  #define printInfo(x)
  #define printInfoLn(x)
  #define printDebug(x)
  #define printDebugLn(x)
#elif defined(INFO_LEVEL)
  #define printMessage(x) Serial.print(gColor + String(x) + rColor)
  #define printMessageLn(x) Serial.println(gColor + String(x) + rColor)
  #define printError(x) Serial.print(eColor + String(x) + rColor)
  #define printErrorLn(x) Serial.println(eColor + String(x) + rColor)
  #define printWarning(x) Serial.print(wColor + String(x) + rColor)
  #define printWarningLn(x) Serial.println(wColor + String(x) + rColor)
  #define printInfo(x) Serial.print(wColor + String(x) + rColor)
  #define printInfoLn(x) Serial.println(wColor + String(x) + rColor)
  #define printDebug(x)
  #define printDebugLn(x)
#elif defined(DEBUG_LEVEL)
  #define printMessage(x) Serial.print(gColor + String(x) + rColor)
  #define printMessageLn(x) Serial.println(gColor + String(x) + rColor)
  #define printError(x) Serial.print(eColor + String(x) + rColor)
  #define printErrorLn(x) Serial.println(eColor + String(x) + rColor)
  #define printWarning(x) Serial.print(wColor + String(x) + rColor)
  #define printWarningLn(x) Serial.println(wColor + String(x) + rColor)
  #define printInfo(x) Serial.print(wColor + String(x) + rColor)
  #define printInfoLn(x) Serial.println(wColor + String(x) + rColor)
  #define printDebug(x) Serial.print(wColor + String(x) + rColor)
  #define printDebugLn(x) Serial.println(wColor + String(x) + rColor)
#endif




#endif