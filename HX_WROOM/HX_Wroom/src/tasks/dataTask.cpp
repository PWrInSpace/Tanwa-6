#include "../include/tasks/tasks.h"
#include <EEPROM.h>
char data[256] = {};
extern float temp_cal_factor;
extern RxData rxData;

void dataTask(void *arg){

  uint16_t HxWeight_raw;
  float HxWeight_unit;
  TxData dataFrame;


  Serial.print("ROCKET WEIGHT: \n");
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
  Serial.print("ROCKET OFFSET  "); Serial.println(HxWeight.get_offset(),2);

  //TODO add ASK request for offset to COM
  while (rxData.request != ANSWER) {

    dataFrame.request = ASK;
    esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(TxData));
    Serial.println("SENDING ASK");
    Serial.println(dataFrame.request);
    vTaskDelay(5000 / portTICK_PERIOD_MS);

  }
  Serial.println(" OUUUUUUUUUUUUUUUUUUUUTTTTTTTTTT LOOP");
  int dd = HxWeight.get_scale() * rxData.offset; 
  //DEBUG
    // int dd = HxWeight.get_scale() * 1000;
  //DEBUG END 
  HxWeight.set_offset(HxWeight.get_offset()-dd);
  Serial.print("OFFSET = "); Serial.println(HxWeight.get_offset());
  dataFrame.request = WORK;
  esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(TxData));


  //  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  vTaskDelay(3000 / portTICK_PERIOD_MS);
 
   
  while(1){

    HxWeight_unit = HxWeight.get_units(10);
    HxWeight_raw = (uint32_t) HxWeight.get_value(10);

    Serial.print("ROCKET WEIGHT: "); Serial.println(HxWeight_unit);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(HxWeight_raw);

    dataFrame.weight = HxWeight_unit;
    dataFrame.weight_raw = (uint32_t)HxWeight_raw;
    dataFrame.temperature = 0;

    createDataFrame(dataFrame, data);
    Serial.print("DATA SENT:  "); Serial.println(data);
    esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(TxData));
    // perror("esp_now_send");


    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}