#ifndef PINOUT_H
#define PINOUT_H

#include "freertos/FreeRTOS.h"
#include "Arduino.h"

//HX711
#define HX_SDA 15
#define HX_SCL 2

#define CALIBRATION_PIN 13
//CROSS!!!!!!!!!
#define CAN_TX 17
#define CAN_RX 16

#define BOOT 0 //????????


void pinInit();

#endif