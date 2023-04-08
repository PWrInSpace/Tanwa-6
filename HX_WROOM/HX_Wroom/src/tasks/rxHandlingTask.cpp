#include "../include/tasks/tasks.h"
#include <string>
#include <iostream>

using namespace std;
RxData rxData;


void rxHandlingTask(void* arg){
  TxData espNowCommand;
  
  

  while(1){
    //ESP NOW
    
    if(xQueueReceive(stm.espNowRxQueue, (void*)&rxData, 0) == pdTRUE){

      Serial.print("ESP NOW FROM COM: ");
      Serial.print("REQUEST:   ");Serial.println(rxData.request);
      Serial.print("OFFSET:   ");Serial.println((uint32_t)rxData.offset);
      Serial.print("COMMAND:   ");Serial.println(rxData.command);
      
    }  
 
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}