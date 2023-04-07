#include <Arduino.h>
#include <SPI.h>
#include "../include/tasks/tasks.h"
#include "../include/config/config.h"
#include <esp_wifi.h>
#include "../include/com/now.h"
#include <EEPROM.h>
//TODO delete one hx - due to single hx on board at individual hx interface

SoftwareToolsManagment stm;
// HX711
HX711_api HxWeight;
float temp_cal_factor;

void setup() {

  Serial.begin(115200);
  pinInit();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA , adressHxRck);


  EEPROM.begin(EEPROM_SIZE);
  
  int eeprom_temp_tab[6];

  for (int i = 0; i<6; i++){
    eeprom_temp_tab[i] = EEPROM.read(i);
    Serial.print("EEPROM TAB  "); Serial.println(eeprom_temp_tab[i]);
  }


  temp_cal_factor =(10*eeprom_temp_tab[4]+1*eeprom_temp_tab[3]+0.1*eeprom_temp_tab[2]+0.01*eeprom_temp_tab[1] + 0.001*eeprom_temp_tab[0]);

  if(eeprom_temp_tab[5]==1)
    temp_cal_factor = temp_cal_factor *(-1);


  Serial.println("EEPROM   "); Serial.println(temp_cal_factor,3);
  
  
   vTaskDelay(1000 / portTICK_PERIOD_MS);




  nowInit();
  nowAddPeer(adressTanwa, 0);

  // stm.espNowRxQueue = xQueueCreate(ESP_NOW_QUEUE_LENGTH, sizeof(TxData));
  // stm.i2c.begin(I2C_SDA, I2C_SCL, 100E3);
  //stm.i2c.setTimeOut(20);


  //stm.i2cMutex = xSemaphoreCreateMutex();


  vTaskDelay(25 / portTICK_PERIOD_MS);

  xTaskCreatePinnedToCore(dataTask, "Data task", 20000, NULL, 3, &stm.dataTask, APP_CPU_NUM);

  //if(stm.i2cMutex == NULL){
    //ESP.restart();
  //}
  
  vTaskDelete(NULL);

  
}

void loop() {}

