#ifndef HARDWARE_MANAGMENT_HH
#define HARDWARE_MANAGMENT_HH


#include "freertos/FreeRTOS.h"
#include <SPI.h>
#include <Wire.h>

struct SoftwareToolsManagment{

  TaskHandle_t dataTask;
  QueueHandle_t espNowRxQueue;

  //SemaphoreHandle_t i2cMutex;
 // SemaphoreHandle_t spiMutex;

  //TwoWire i2c = TwoWire(0);
};


#endif