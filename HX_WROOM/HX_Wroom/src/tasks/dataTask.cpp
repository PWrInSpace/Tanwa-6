#include "../include/tasks/tasks.h"
#include <EEPROM.h>
char data[256] = {};
extern float temp_cal_factor;

void dataTask(void *arg){

  uint16_t HxWeight_raw;
  uint16_t HxWeight_unit;
  DataFrame dataFrame;

  Serial.print("ROCKET WEIGHT: \n");
    //HX711
  HxWeight.begin(HX_SDA, HX_SCL);

   while (!HxWeight.wait_ready_retry())
  {
    Serial.print("WAIT\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  //HxWeight.set_gain(128);
  //HxWeight.wait_ready_timeout(); //start without tare
  HxWeight.set_scale(temp_cal_factor);
  Serial.print("ROCKET CAL FAC  "); Serial.println(HxWeight.get_scale(),3);
  // HxWeight.set_offset(OFFSET_RCK);
  HxWeight.tare();
  Serial.print("ROCKET OFFSET  "); Serial.println(HxWeight.get_offset());

  //  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  // int dd = rckWeight.get_scale() * lastWeight; // TODO 1000 = last saved measurement! from SD
  // rckWeight.set_offset(rckWeight.get_offset()-dd);

  vTaskDelay(3000 / portTICK_PERIOD_MS);


  vTaskDelay(100 / portTICK_PERIOD_MS);
 
   
  while(1){

    if(digitalRead(CALIBRATION_PIN) == HIGH){

        float temp_cal_factor2;
        temp_cal_factor2 = HxWeight.calibration(1000);
        
        Serial.println("temp cal factor2  "); Serial.println(temp_cal_factor2,3);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        int x_temp = 0, y_temp = 0;
        
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
            Serial.println("CONVERTED VALUE: "); Serial.println(tab[i]);
            EEPROM.write(i, tab[i]);
            EEPROM.commit();
          } 
          Serial.println("is negative: "); Serial.println(tab[5]);
          EEPROM.write(5, tab[5]);
          EEPROM.commit();
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); 


    }
    
    HxWeight_unit = HxWeight.get_units(1);
    HxWeight_raw = (uint32_t) HxWeight.get_value(1);

    Serial.print("ROCKET WEIGHT: "); Serial.println(HxWeight_unit);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(HxWeight_raw);

    dataFrame.weight = HxWeight_unit;
    dataFrame.weight_raw = (uint32_t)HxWeight_raw;
    dataFrame.temperature = 0;

    createDataFrame(dataFrame, data);
     Serial.print("DATA SENT:  "); Serial.println(data);
    esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(DataFrame));
    perror("esp_now_send");

   

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}