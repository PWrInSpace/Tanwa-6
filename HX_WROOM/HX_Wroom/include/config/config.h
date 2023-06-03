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


#define DATA_PREFIX "HX_RCK;"
#define DATA_PREFIX_BTL "HX_BTL;"
//NOW QUEUE
#define ESP_NOW_QUEUE_LENGTH 100
//EEPROM
#define EEPROM_SIZE 50
//WEIGHTING
#define BIT_TO_GRAM_RATIO_RCK 53.13
#define BIT_TO_GRAM_RATIO_TANK 37.78// USA 32.73
#define OFFSET_TANK 1
#define STABILIZNG_TIME 4000


#define NOTHING 255
#define TARE_ 70
#define CALIBRATE_ 71
#define SET_CAL_FACTOR_ 72
#define SET_OFFSET_ 73
#define SOFT_RESTART_ 99

#endif