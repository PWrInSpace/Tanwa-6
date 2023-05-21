#include "../include/tasks/tasks.h"
#include <eHaJo_LM75.h>
#include <EEPROM.h>

char data[256] = {};
extern float temp_cal_factor;
extern RxData rxData;

RxData rxData_CAN;
TxData txData_CAN;
EHAJO_LM75 tempsensor;


void dataTask(void *arg){



  uint16_t HxWeight_raw;
  float HxWeight_unit;
  float HxWeight_temp;
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


  // while (rxData.request != ANSWER) {

  //   dataFrame.request = ASK;
  //   txData_CAN.request = ASK;
  //   canSend();
  //   esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(TxData));
  //   Serial.println("SENDING ASK");
  //   Serial.println(dataFrame.request);
  //   vTaskDelay(5000 / portTICK_PERIOD_MS);

  // }
  Serial.println(" OUUUUUUUUUUUUUUUUUUUUTTTTTTTTTT LOOP");
  int dd = HxWeight.get_scale() * rxData.offset; 

  HxWeight.set_offset(HxWeight.get_offset()-dd);
  Serial.print("OFFSET = "); Serial.println(HxWeight.get_offset());
  dataFrame.request = WORK;
  txData_CAN.request = WORK;
  esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(TxData));


  //  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  vTaskDelay(3000 / portTICK_PERIOD_MS);
 
   
  while(1){

    HxWeight_temp = tempsensor.getTemp();
    HxWeight_unit = HxWeight.get_units(10);
    HxWeight_raw = (uint32_t) HxWeight.get_value(10);

    Serial.print("Temperature = ");Serial.print(tempsensor.getTemp());Serial.println(" C");
    Serial.print("ROCKET WEIGHT: "); Serial.println(HxWeight_unit);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(HxWeight_raw);

    txData_CAN.weight = HxWeight_unit;
    txData_CAN.weight_raw = (uint32_t)HxWeight_raw;
    txData_CAN.temperature = 0;

    dataFrame.weight = HxWeight_unit;
    dataFrame.weight_raw = (uint32_t)HxWeight_raw;
    dataFrame.temperature = HxWeight_temp;

    createDataFrame(dataFrame, data);
    Serial.print("DATA SENT:  "); Serial.println(data);
    esp_now_send(adressTanwa, (uint8_t*) &dataFrame, sizeof(TxData));
    canSend();
    // perror("esp_now_send");


    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}