#include "../include/tasks/tasks.h"

void dataTask(void *arg){

  uint16_t HxWeight_raw;
  uint16_t HxWeight_unit;

    //HX711
  HxWeight.begin(HX_SDA, HX_SCL);
  //HxWeight.set_gain(128);
  //HxWeight.wait_ready_timeout(); //start without tare
  HxWeight.set_scale(BIT_TO_GRAM_RATIO_RCK);
  // HxWeight.set_offset(OFFSET_RCK);



  while (HxWeight.wait_ready_retry())
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
 
   
  while(1){
    
    HxWeight_unit = HxWeight.get_units(1);
    HxWeight_raw = (uint32_t) HxWeight.get_value(1);

    Serial.print("ROCKET WEIGHT: "); Serial.println(HxWeight_unit);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(HxWeight_raw);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}