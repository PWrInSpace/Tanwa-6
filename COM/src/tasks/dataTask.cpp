#include "../include/tasks/tasks.h"
#include <EEPROM.h>
#include "lora.pb.h"


LoRaFrameTanwa loraFrameTanwa;
char data[SD_FRAME_SIZE] = {};
PWRData pwrData;
bool lastWeightFlag = false;
extern float temp_cal_factor;
extern float lastWeight;
extern float lastWeightBtl;
extern RxData_Hx rxDataRck;
extern RxData_Hx rxDataBtl;
extern TxData_Hx txDataRck;
extern TxData_Hx txDataBtl;

extern RxData_Hx rxDataBtl_CAN;
extern RxData_Hx rxDataRck_CAN;
extern PWRData pwrData_CAN;
extern TxData_Hx txDataRck_CAN;
extern TxData_Hx txDataBtl_CAN;
extern TxData txData_CAN;

extern bool can_rck_interface;
extern bool esp_now_rck_interface;
extern bool can_btl_interface;
extern bool esp_now_btl_interface;

void dataTask(void *arg){
  Serial.println("DATA TASK");
  uint32_t abort_count = 0;
  int turnVar = 0;
  int SD_cont = 0;
  int abrtButton = 0;
  DataFrame dataFrame;

  xSemaphoreTake(stm.i2cMutex, pdTRUE);
  expander.setPinMode(0,B,INPUT); //input for abort button
  xSemaphoreGive(stm.i2cMutex);

//TODO
  // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // Serial.println("BEFORE RCK REQUEST");
  // Serial.println(rxDataRck.request);
  // while(rxDataRck.request != WORK && rxDataRck.request == ASK){
  //   // lastWeightFlag = true;
  //   txDataRck.request = ANSWER;
  //   txDataRck.offset = lastWeight;
  //   // esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
  //   perror("esp_now_send RCK");
  //   Serial.println("SENDING ANSWER RCK");
  //   Serial.println(txDataRck.request);
  //   Serial.println(txDataRck.offset);
  //   Serial.println(txDataRck.command);
  //   vTaskDelay(2000 / portTICK_PERIOD_MS);
  // }
  // Serial.println("BEFORE BTL REQUEST");
  // Serial.println(rxDataBtl.request);
  // while(rxDataBtl.request != WORK && rxDataBtl.request == ASK){
  //     // lastWeightFlag = true;
  //     txDataBtl.request = ANSWER;
  //     txDataBtl.offset = lastWeightBtl;
  //     // esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
  //     perror("esp_now_send BTL");
  //     Serial.println("SENDING ANSWER BTL");
  //     Serial.println(txDataBtl.request);
  //     Serial.println(txDataBtl.offset);
  //     Serial.println(txDataBtl.command);
  //     vTaskDelay(2000 / portTICK_PERIOD_MS);
  // }
  Serial.println("OUT OF LOOP");
  

  vTaskDelay(300 / portTICK_PERIOD_MS);

  ledcWriteTone(0, 5000);
  for(int i = 0; i <2; i++){
  ledcWrite(0, 255);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ledcWrite(0, 0);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  }


  // !!!//DEBUG
  //InternalI2C<PWRData, TxData> i2cCOM(&stm.i2c, COM_ADRESS);

  lastWeightFlag = true;
  while(1){

    txDataRck.request = WORK;
    txDataRck.offset = 0;
    txDataRck.command = NOTHING;

    txDataBtl.request = WORK;
    txDataBtl.offset = 0;
    txDataBtl.command = NOTHING;
    
    Serial.println("LOOP");
    xSemaphoreTake(stm.i2cMutex, pdTRUE);
    pwrCom.getData(&pwrData);
    xSemaphoreGive(stm.i2cMutex);

    xSemaphoreTake(stm.i2cMutex, pdTRUE);
    expander.setPinX(2,B,OUTPUT, turnVar);
    xSemaphoreGive(stm.i2cMutex);

    if(turnVar == 1)
      turnVar = 0;
    else
      turnVar = 1;
    //########## ESP NOW STRUCT #######################

    if(can_btl_interface == false && esp_now_btl_interface == true){
      Serial.println("ESP NOW BTL");
      loraFrameTanwa.tankWeight_blink = rxDataBtl.blink;
      loraFrameTanwa.tankWeight_temp = rxDataBtl.temperature;
      loraFrameTanwa.tankWeight_val = rxDataBtl.weight;
      loraFrameTanwa.tankWeightRaw_val = (uint32_t) rxDataBtl.weight_raw;
    }else{ //########## CAN STRUCT ######################
      Serial.println("CAN BTL");
      loraFrameTanwa.tankWeight_blink = rxDataBtl_CAN.blink;
      loraFrameTanwa.tankWeight_temp = rxDataBtl_CAN.temperature;
      loraFrameTanwa.tankWeight_val = rxDataBtl_CAN.weight;
      loraFrameTanwa.tankWeightRaw_val = (uint32_t) rxDataBtl_CAN.weight_raw;

    }
      //########## ESP NOW STRUCT #######################
    if(can_rck_interface == false && esp_now_rck_interface == true){
      Serial.println("ESP NOW RCK");
      loraFrameTanwa.rocketWeight_blink = rxDataRck.blink;
      loraFrameTanwa.rocketWeight_temp = rxDataRck.temperature;
      loraFrameTanwa.rocketWeight_val = rxDataRck.weight;
      loraFrameTanwa.rocketWeightRaw_val = (uint32_t) rxDataRck.weight_raw;
    }else{ //########## CAN STRUCT ######################
      Serial.println("CAN RCK");
      loraFrameTanwa.rocketWeight_blink = rxDataRck_CAN.blink;
      loraFrameTanwa.rocketWeight_temp = rxDataRck_CAN.temperature;
      loraFrameTanwa.rocketWeight_val = rxDataRck_CAN.weight;
      loraFrameTanwa.rocketWeightRaw_val = (uint32_t) rxDataRck_CAN.weight_raw;

    }
    snprintf(data, sizeof(data), "%.2f", loraFrameTanwa.rocketWeight_val);
    // Serial.print("DATA TO BE SAVED: "); Serial.println(data);
    xQueueSend(stm.sdQueue_lastWeightRck, (void*)data, 0);

    snprintf(data, sizeof(data), "%.2f", loraFrameTanwa.tankWeight_val);
    // Serial.print("DATA TO BE SAVED: "); Serial.println(data);
    xQueueSend(stm.sdQueue_lastWeightBtl, (void*)data, 0);
    

    loraFrameTanwa.vbat = voltageMeasure(VOLTAGE_MEASURE);
    //### memcpy(dataFrame.motorState, pwrData.motorState, sizeof(uint8_t[5]));
    loraFrameTanwa.motorState_1 = pwrData.motorState[0];
    loraFrameTanwa.motorState_2 = pwrData.motorState[1];
    loraFrameTanwa.motorState_3 = pwrData.motorState[2];
    loraFrameTanwa.motorState_4 = pwrData.motorState[3];

  

    loraFrameTanwa.tanWaState = StateMachine::getCurrentState();
    // Serial.println(analogRead(IGN_TEST_CON_1));
    // Serial.println(analogRead(IGN_TEST_CON_2));
    loraFrameTanwa.igniterContinouity_1 = analogRead(IGN_TEST_CON_1) > 1000;
    loraFrameTanwa.igniterContinouity_2 = analogRead(IGN_TEST_CON_2) > 1000;

    if(rxDataRck.request == ANSWER){
        loraFrameTanwa.hxRequest_RCK = 1;
    }else if(rxDataRck.request == ANSWER){
        loraFrameTanwa.hxRequest_RCK = 2;
    }else if(rxDataRck.request == WORK){
        loraFrameTanwa.hxRequest_RCK = 3;
    }else{
        loraFrameTanwa.hxRequest_RCK = 0;
    }

    if(rxDataBtl.request == ANSWER){
        loraFrameTanwa.hxRequest_TANK = 1;
    }else if(rxDataBtl.request == ANSWER){
        loraFrameTanwa.hxRequest_TANK = 2;
    }else if(rxDataBtl.request == WORK){
        loraFrameTanwa.hxRequest_TANK = 3;
    }else{
        loraFrameTanwa.hxRequest_TANK = 0;
    }

    dataFrame.tanWaState = loraFrameTanwa.tanWaState;
    dataFrame.pressureSensor = loraFrameTanwa.pressureSensor;
    dataFrame.solenoid_fill = loraFrameTanwa.solenoid_fill;
    dataFrame.solenoid_depr = loraFrameTanwa.solenoid_depr;
    dataFrame.abortButton = loraFrameTanwa.abortButton;
    dataFrame.igniterContinouity_1 = loraFrameTanwa.igniterContinouity_1;
    dataFrame.igniterContinouity_2 = loraFrameTanwa.igniterContinouity_2;
    dataFrame.hxRequest_RCK =  loraFrameTanwa.hxRequest_RCK;
    dataFrame.hxRequest_TANK =  loraFrameTanwa.hxRequest_TANK;
    dataFrame.vbat = loraFrameTanwa.vbat;
    dataFrame.motorState_1 = loraFrameTanwa.motorState_1;
    dataFrame.motorState_2 = loraFrameTanwa.motorState_2;
    dataFrame.motorState_3 = loraFrameTanwa.motorState_3;
    dataFrame.motorState_4 = loraFrameTanwa.motorState_4;
    dataFrame.tankWeight_blink = rxDataBtl.blink;
    dataFrame.rocketWeight_blink = rxDataRck.blink;
    dataFrame.tankWeight_temp = loraFrameTanwa.tankWeight_temp;
    dataFrame.rocketWeight_temp = loraFrameTanwa.rocketWeight_temp;
    dataFrame.rocketWeight_val = loraFrameTanwa.rocketWeight_val;
    dataFrame.tankWeight_val = loraFrameTanwa.tankWeight_val;
    dataFrame.rocketWeightRaw_val = loraFrameTanwa.rocketWeightRaw_val;
    dataFrame.tankWeightRaw_val = loraFrameTanwa.tankWeightRaw_val;

    createDataFrame(dataFrame, data);
    // Serial.println(data);
    //TODO  lora command send on
    // xQueueSend(stm.loraTxQueue, (void*)&loraFrameTanwa, 0);

    xQueueSend(stm.sdQueue, (void*)data, 0); 
    //TODO check polarity of pin ABORT on esp pull up or down
    xSemaphoreTake(stm.i2cMutex, pdTRUE);
    abrtButton = !expander.getPin(0,B);
    SD_cont = !expander.getPin(7,A);
    xSemaphoreGive(stm.i2cMutex);

    //TODO UNCOMMENT + change pin or solder correct pull up
    // if(abrtButton == 1){
    if( analogRead(IGN_TEST_CON_1) > 1000){

      Serial.println("############## CALIBRATE RCK ################");
      
      xSemaphoreTake(stm.i2cMutex, pdTRUE);
      expander.setPinX(5,A,OUTPUT, OFF);
      xSemaphoreGive(stm.i2cMutex);

      txDataRck.request = ASK;
      txDataRck.offset = 4100;
      txDataRck.command = CALIBRATE_HX;
      esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
      // if(digitalRead(HX1_SCL) == HIGH){
      if( analogRead(IGN_TEST_CON_2) > 1000){

      Serial.println("############ CALIBRATE BTL ##############");
      
      xSemaphoreTake(stm.i2cMutex, pdTRUE);
      expander.setPinX(5,A,OUTPUT, OFF);
      xSemaphoreGive(stm.i2cMutex);

      txDataBtl.request = ASK;
      txDataBtl.offset = 4100;
      txDataBtl.command = CALIBRATE_HX;
      esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    // // //################### real abort content ###################
    // //   // abort_count++;
    // //   // Serial.println("==================");
    // //   // Serial.println("ABORT ++");
    // //   // Serial.println("==================");
    // //   // if(abort_count>=3){
    // //   //   xSemaphoreTake(stm.i2cMutex, pdTRUE);
    // //   //   expander.setPin(4,A,OFF);
    // //   //   xSemaphoreGive(stm.i2cMutex);
    // //   //   StateMachine::changeStateRequest(States::ABORT);
    // //   //   Serial.println("ABORT BUTTON CONFIRMATION");
    // //   //}
    // //   //##########################################################
    // }
    // else{
    //   abort_count = 0;
    //   xSemaphoreTake(stm.i2cMutex, pdTRUE);
    //   expander.setPin(4,A,ON);
    //   expander.setPinX(5,A,OUTPUT, ON);
    //   xSemaphoreGive(stm.i2cMutex);
    // }
    // // canSend();


      
    // Serial.println("RESeeeeeeeeeeeeeeeET");
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // xSemaphoreTake(stm.i2cMutex, pdTRUE);
    // expander.setPinX(4,A,OUTPUT,ON);
    // xSemaphoreGive(stm.i2cMutex);
    //DEBUG(data);
    // xSemaphoreTake(stm.i2cMutex, pdTRUE);
    // i2cCOM.getData(&pwrData);
    // xSemaphoreGive(stm.i2cMutex);
    
    Serial.println("\nCOM DATA:");
    Serial.print("TANWA STATE: "); Serial.println(loraFrameTanwa.tanWaState);
    Serial.print("SD status: "); Serial.println(SD_cont);
    Serial.print("ABORT BUTTON: "); Serial.println(abrtButton);
    Serial.print("BLINK: "); Serial.println(pwrData.tick);
    Serial.print("LAST COMMAND: "); Serial.println(pwrData.lastDoneCommandNum);
    Serial.print("MOTOR FILL COMMAND: "); Serial.println(pwrData.motorState[0]);
    Serial.print("MOTOR FILL ADC: "); Serial.println(pwrData.adcValue[1]);
    Serial.print("MOTOR DEPR COMMAND: "); Serial.println(pwrData.motorState[1]);
    Serial.print("MOTOR DEPR ADC: "); Serial.println(pwrData.adcValue[2]);
    Serial.print("MOTOR STATE QUICK DISCONNECT: "); Serial.println(pwrData.motorState[2]);
    

    Serial.print("PRESSURE bit: "); Serial.println(pwrData.adcValue[0]);

    loraFrameTanwa.pressureSensor = map(pwrData.adcValue[0],450, 4096, 0, 80);
    Serial.print("PRESSURE in bars: "); Serial.println(loraFrameTanwa.pressureSensor);
    Serial.print("TANWA VOLTAGE: "); Serial.println(voltageMeasure(VOLTAGE_MEASURE));

    Serial.print("TANK BLNK:  "); Serial.print(loraFrameTanwa.tankWeight_blink); //TODO chabge to lora frame
    Serial.print("\t\t\tROCKET BLNK:  "); Serial.println(loraFrameTanwa.rocketWeight_blink); // TODO change to lora frame
    Serial.print("TANK WEIGHT: "); Serial.print(loraFrameTanwa.tankWeight_val);
    Serial.print("\t\tROCKET WEIGHT: "); Serial.println(loraFrameTanwa.rocketWeight_val);
    Serial.print("TANK WEIGHT RAW: "); Serial.print(loraFrameTanwa.tankWeightRaw_val);
    Serial.print("\t\tROCKET WEIGHT RAW: "); Serial.println(loraFrameTanwa.rocketWeightRaw_val);
    Serial.print("TANK TEMPERATURE: "); Serial.print(loraFrameTanwa.tankWeight_temp);
    Serial.print("\t\tROCKET TEMPERATURE: "); Serial.println(loraFrameTanwa.rocketWeight_temp);
    // Serial.print("ROCKET WEIGHT OFFSET: "); Serial.println(rckWeight.get_offset());

    Serial.print("HX REQUEST TANK: "); Serial.print(rxDataBtl.request);
    Serial.print("\t\tHX REQUEST RCK: "); Serial.println(rxDataRck.request);
    Serial.print("continuity 1 "); Serial.println(loraFrameTanwa.igniterContinouity_1);
    Serial.print("continuity 2 "); Serial.println(loraFrameTanwa.igniterContinouity_2);
   
    Serial.println();


    esp_now_send(adressObc, (uint8_t*) &dataFrame, sizeof(dataFrame));
    // esp_now_send(adressObc, (uint8_t*) &dataFrame, sizeof(DataFrame)); //DO NOT USE FOR OBC due hx request type STRING
    
    vTaskDelay(557/ portTICK_PERIOD_MS); 
  }
}