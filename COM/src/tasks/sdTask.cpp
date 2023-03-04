#include "../include/tasks/tasks.h"

void sdTask(void *arg){
  //TanWaControl * tc = static_cast<TanWaControl*>(arg);
  char data[SD_FRAME_SIZE] = {};
  String dataPath = dataFileName;
  String dataPath_lastWeight = dataFileName_lastWeight;
  uint32_t sd_i = 0;
  uint32_t sd_j = 0;
  
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


   while(mySD.fileExists(dataPath_lastWeight + String(sd_j) + ".txt")){
    sd_j++;
  }
  dataPath_lastWeight = dataPath_lastWeight + String(sd_j) + ".txt";

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


    if(xQueueReceive(stm.sdQueue_lastWeight, (void*)&data, 0) == pdTRUE){
      
      xSemaphoreTake(stm.spiMutex, portMAX_DELAY);  

      if(!mySD.write(dataPath_lastWeight, data, "w")){
        //SD_WRITE_ERROR
      }
        
      DEBUG("ZAPIS NA SD");
      //xSemaphoreGive(tc->spiMutex);
      xSemaphoreGive(stm.spiMutex);
    }
   
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}