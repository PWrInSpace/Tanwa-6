#ifndef HARDWARE_MANAGMENT_HH
#define HARDWARE_MANAGMENT_HH


#include "freertos/FreeRTOS.h"
#include <SPI.h>
#include <Wire.h>
#include "freertos/semphr.h"

struct SoftwareToolsManagment{
  TaskHandle_t loraTask;
  TaskHandle_t canTask;

  TaskHandle_t sdTask;
  TaskHandle_t dataTask;
  TaskHandle_t stateTask;
  TaskHandle_t buzzerTask;
  TaskHandle_t rxHandlingTask;
  //... inne taski
  QueueHandle_t sdQueue;
  QueueHandle_t sdQueue_lastWeight;
  QueueHandle_t loraTxQueue;
  QueueHandle_t loraRxQueue;
  QueueHandle_t espNowRxQueueHxRck;
  QueueHandle_t espNowRxQueueHxBtl;
  QueueHandle_t espNowRxQueueObc;

  QueueHandle_t canRxQueueHxRck;

  SemaphoreHandle_t i2cMutex;
  SemaphoreHandle_t spiMutex;
  SemaphoreHandle_t canMutex;

  TwoWire i2c = TwoWire(0);
  SPIClass spi = SPIClass(VSPI);
};


#endif