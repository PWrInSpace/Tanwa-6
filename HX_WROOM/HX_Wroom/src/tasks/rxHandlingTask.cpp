#include "../include/tasks/tasks.h"
#include <EEPROM.h>
#include <string>
#include <iostream>

using namespace std;
RxData rxData;
bool esp_flag = true;
bool can_flag = false;


void rxHandlingTask(void* arg){
  
  while(1){
    //ESP NOW
    
    if(xQueueReceive(stm.espNowRxQueue, (void*)&rxData, 0) == pdTRUE){

      Serial.print("ESP NOW FROM COM: ");
      Serial.print("REQUEST:   ");Serial.println(rxData.request);
      Serial.print("OFFSET:   ");Serial.println((uint32_t)rxData.offset);
      Serial.print("COMMAND:   ");Serial.println(rxData.command);
      
      switch(rxData.command){
        case TARE_:
          HxWeight.tare();
          break;
        
        case CALIBRATE_:
        {     
          rxData.command = NOTHING;    
          float temp_cal_factor2;
          temp_cal_factor2 = HxWeight.calibration(rxData.offset);
          
          Serial.println("temp cal factor2  "); Serial.println(temp_cal_factor2,3);
          vTaskDelay(2000 / portTICK_PERIOD_MS);

          int x_temp = 0;
          
          temp_cal_factor2 = temp_cal_factor2*1000;
            
          //Serial.println("temp cal factor*1000  "); Serial.println(temp_cal_factor2);
          if(abs(temp_cal_factor2)<=99999){    
            int tab[6];
            if(temp_cal_factor2<0)
              tab[5] = 1;
            else
              tab[5] = 0;
            for (int i=0; i<5; i++){
              x_temp=abs(temp_cal_factor2)/10;
              tab[i] = abs(temp_cal_factor2) - 10*x_temp;
              temp_cal_factor2 = x_temp;
              Serial.println("##################  CONVERTED VALUE:  #####################"); Serial.println(tab[i]);
              EEPROM.write(i, tab[i]);
              EEPROM.commit();
            } 
            Serial.println("is negative: "); Serial.println(tab[5]);
            EEPROM.write(5, tab[5]);
            EEPROM.commit();
          }

          vTaskDelay(5000 / portTICK_PERIOD_MS); 
          break;
        }

        case SET_CAL_FACTOR_:{
          float temp_cal;
          temp_cal = rxData.offset;
          temp_cal = temp_cal/1000;
          HxWeight.set_scale(temp_cal);

          int x_temp = 0;
          
          temp_cal = temp_cal*1000;
            
          //Serial.println("temp cal factor*1000  "); Serial.println(temp_cal_factor2);
          if(abs(temp_cal)<=99999){    
            int tab[6];
            if(temp_cal<0)
              tab[5] = 1;
            else
              tab[5] = 0;
            for (int i=0; i<5; i++){
              x_temp=abs(temp_cal)/10;
              tab[i] = abs(temp_cal) - 10*x_temp;
              temp_cal = x_temp;
              Serial.println("##################  CONVERTED VALUE:  #####################"); Serial.println(tab[i]);
              EEPROM.write(i, tab[i]);
              EEPROM.commit();
            } 
            Serial.println("is negative: "); Serial.println(tab[5]);
            EEPROM.write(5, tab[5]);
            EEPROM.commit();
          }
          break;
        }

        case SET_OFFSET_:{
          int dd = HxWeight.get_scale() * rxData.offset; 
          HxWeight.set_offset(HxWeight.get_offset()-dd);
          Serial.print("###### ### OFFSET = "); Serial.println(HxWeight.get_offset());
          break;
        }
        
        case SOFT_RESTART_:{
          esp_restart();
          break;
        }

        case ESP_INTERFACE_:{
          esp_flag = true;
          can_flag = false;
          Serial.println("####### ESP INTERFACE #########");
          break;
        }

        case CAN_INTERFACE_:{
          esp_flag = false;
          can_flag = true;
          Serial.println("####### CAN INTERFACE #########");
          break;
        }

        default:
          break;
      }
    }  
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}