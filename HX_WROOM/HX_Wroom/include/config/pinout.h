#ifndef PINOUT_H
#define PINOUT_H

#include "freertos/FreeRTOS.h"
#include "Arduino.h"

//HX711
#define HX_SDA 15
#define HX_SCL 2

#define CALIBRATION_PIN 13

#define TX 16
#define RX 17

#define BOOT 0 //????????


void pinInit();

#endif