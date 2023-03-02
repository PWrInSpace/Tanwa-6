#ifndef PINOUT_H
#define PINOUT_H

#include "freertos/FreeRTOS.h"
#include "Arduino.h"

//HX711
#define HX_SDA 15
#define HX_SCL 2


#define TX 1
#define RX 3

#define BOOT 0 //????????


void pinInit();

#endif