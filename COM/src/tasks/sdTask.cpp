#include "../include/tasks/tasks.h"
float lastWeight = 0;
float lastWeightBtl = 0;

void sdTask(void *arg){
  //TanWaControl * tc = static_cast<TanWaControl*>(arg);
  char data[SD_FRAME_SIZE] = {};
  String dataPath = dataFileName;
  String dataPath_lastWeightRck = dataFileName_lastWeightRck;
  String dataPath_lastWeightBtl = dataFileName_lastWeightBtl;
  uint32_t sd_i = 0;
  uint32_t sd_rck = 0;
  uint32_t sd_btl = 0;
  
  vTaskDelay(100 / portTICK_RATE_MS);

  SDCard mySD(stm.spi, SD_CS);

  xSemaphoreTake(stm.spiMutex, portMAX_DELAY);

  while(!mySD.init()){
    Serial.println("SD INIT ERROR!");
    xSemaphoreGive(stm.spiMutex);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xSemaphoreTake(stm.spiMutex, portMAX_DELAY);
  }
  

  while(mySD.fileExists(dataPath + String(sd_i) + ".txt")){
    sd_i++;
  }
  dataPath = dataPath + String(sd_i) + ".txt";

//############## ROCKET #########################

  while(mySD.fileExists(dataPath_lastWeightRck + String(sd_rck) + ".txt")){
    sd_rck++;
  }
  Serial.print("SD path RCK == "); Serial.println(dataPath_lastWeightRck + String(sd_rck-1) + ".txt");

  File file = SD.open(dataPath_lastWeightRck + String(sd_rck-1) + ".txt", FILE_READ);
  String data2 = file.readStringUntil(';');
  Serial.println(data2.toFloat());
  file.close();

  if(data2.toFloat()==0){
    File file = SD.open(dataPath_lastWeightRck + String(sd_rck-2) + ".txt", FILE_READ);
    String data3 = file.readStringUntil(';');
    Serial.println(data3.toFloat());
    file.close();
    lastWeight = data3.toFloat();
  } 
  else 
    lastWeight = data2.toFloat();

//############# BTL ######################

  while(mySD.fileExists(dataPath_lastWeightBtl + String(sd_btl) + ".txt")){
    sd_btl++;
  }
  Serial.print("SD path BTL == "); Serial.println(dataPath_lastWeightBtl + String(sd_btl-1) + ".txt");

  File file_btl = SD.open(dataPath_lastWeightBtl + String(sd_btl-1) + ".txt", FILE_READ);
  String data2_btl = file_btl.readStringUntil(';');
  Serial.println(data2_btl.toFloat());
  file_btl.close();

  if(data2_btl.toFloat()==0){
    File file_btl = SD.open(dataPath_lastWeightBtl + String(sd_btl-2) + ".txt", FILE_READ);
    String data3_btl = file_btl.readStringUntil(';');
    Serial.println(data3_btl.toFloat());
    file_btl.close();
    lastWeightBtl = data3_btl.toFloat();
  } 
  else 
    lastWeightBtl = data2_btl.toFloat();
    
  xTaskNotifyGive(stm.dataTask);
  xTaskNotifyGive(stm.canTask);
  
  dataPath_lastWeightRck = dataPath_lastWeightRck + String(sd_rck) + ".txt";
  dataPath_lastWeightBtl = dataPath_lastWeightBtl + String(sd_btl) + ".txt";
  
  vTaskDelay(200 / portTICK_PERIOD_MS);

  xSemaphoreGive(stm.spiMutex);

  vTaskDelay(100 / portTICK_PERIOD_MS);

  while(1){
    if(xQueueReceive(stm.sdQueue, (void*)&data, 0) == pdTRUE){
      
      xSemaphoreTake(stm.spiMutex, portMAX_DELAY);  

      if(!mySD.write(dataPath, data, "a")){
        //SD_WRITE_ERROR
      }
        
      DEBUG("ZAPIS NA SD");
      //xSemaphoreGive(tc->spiMutex);
      xSemaphoreGive(stm.spiMutex);
    }


    if(xQueueReceive(stm.sdQueue_lastWeightRck, (void*)&data, 0) == pdTRUE){
      
      xSemaphoreTake(stm.spiMutex, portMAX_DELAY);  

      strcat(data, ";");
      if(!mySD.write(dataPath_lastWeightRck, data, FILE_WRITE)){
        //SD_WRITE_ERROR
      }
        
      DEBUG("ZAPIS NA SD");
      //xSemaphoreGive(tc->spiMutex);
      xSemaphoreGive(stm.spiMutex);
    }

      if(xQueueReceive(stm.sdQueue_lastWeightBtl, (void*)&data, 0) == pdTRUE){
      
      xSemaphoreTake(stm.spiMutex, portMAX_DELAY);  

      strcat(data, ";");
      if(!mySD.write(dataPath_lastWeightBtl, data,  FILE_WRITE)){
        //SD_WRITE_ERROR
      }
        
      DEBUG("ZAPIS NA SD");
      //xSemaphoreGive(tc->spiMutex);
      xSemaphoreGive(stm.spiMutex);
    }
   
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}