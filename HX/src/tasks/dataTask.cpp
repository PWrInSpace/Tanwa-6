#include "../include/tasks/tasks.h"

//kod w tym tasku jest tylko do debugu 
void dataTask(void *arg){

  uint16_t tankWeight_raw;
  uint16_t rckWeight_raw;
  
  uint16_t tankWeight_unit;
  uint16_t rckWeight_unit;
    //HX711
  rckWeight.begin(HX1_SDA, HX1_SCL);
  //rckWeight.set_gain(128);
  //rckWeight.wait_ready_timeout(); //start without tare
  rckWeight.set_scale(BIT_TO_GRAM_RATIO_RCK);
  // rckWeight.set_offset(OFFSET_RCK);

  tankWeight.begin(HX2_SDA, HX2_SCL);
  //tankWeight.set_gain(128);
  //tankWeight.wait_ready_timeout(); //start without tare
  tankWeight.set_scale(BIT_TO_GRAM_RATIO_TANK);
  // tankWeight.set_offset(OFFSET_TANK);

  while (tankWeight.wait_ready_retry() && rckWeight.wait_ready_retry())
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
 
   
  while(1){
   
    
    tankWeight_unit = tankWeight.get_units(1);
    tankWeight_raw = (uint32_t) tankWeight.get_value(1);
    
    rckWeight_unit = rckWeight.get_units(1);
    rckWeight_raw = (uint32_t) rckWeight.get_value(1);



    Serial.print("TANK WEIGHT: "); Serial.println(tankWeight_unit);
    Serial.print("TANK WEIGHT RAW: "); Serial.println(tankWeight_raw);

    Serial.print("ROCKET WEIGHT: "); Serial.println(rckWeight_unit);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(rckWeight_raw);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}