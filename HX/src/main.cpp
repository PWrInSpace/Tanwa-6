#include <Arduino.h>
#include <SPI.h>
#include "../include/tasks/tasks.h"
#include "../include/config/config.h"
#include <esp_wifi.h>

//TODO delete one hx - due to single hx on board at individual hx interface

SoftwareToolsManagment stm;
// HX711
HX711_api HxWeight;


void setup() {

  Serial.begin(115200);
  // pinInit();
  
  //stm.i2c.begin(I2C_SDA, I2C_SCL, 100E3);
  //stm.i2c.setTimeOut(20);


  //stm.i2cMutex = xSemaphoreCreateMutex();


  vTaskDelay(25 / portTICK_PERIOD_MS);

  // xTaskCreatePinnedToCore(dataTask, "Data task", 20000, NULL, 3, &stm.dataTask, APP_CPU_NUM);

  //if(stm.i2cMutex == NULL){
    //ESP.restart();
  //}
  
  // vTaskDelete(NULL);

  
}

void loop() {Serial.print("ROCKET WEIGHT: ");vTaskDelay(2500 / portTICK_PERIOD_MS);}

