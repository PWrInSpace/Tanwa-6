#ifndef TANWA_CONFIG_H
#define TANWA_CONFIG_H

#include <Arduino.h>

//#define __DEBUG

#ifdef __DEBUG
  #define DEBUGL(x) Serial.println(x)
  #define DEBUG(x) Serial.print(x)
#else
  #define DEBUGL(x) {}
  #define DEBUG(x) {}
#endif

//WEIGHTING
#define BIT_TO_GRAM_RATIO_RCK 53.13
#define BIT_TO_GRAM_RATIO_TANK 37.78// USA 32.73
#define OFFSET_TANK 1
#define STABILIZNG_TIME 4000

#endif