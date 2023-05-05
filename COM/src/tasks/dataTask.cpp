#include "../include/tasks/tasks.h"
#include <EEPROM.h>
#include "lora.pb.h"

 LoRaFrameTanwa loraFrameTanwa;
 char data[SD_FRAME_SIZE] = {};
 PWRData pwrData;
 bool lastWeightFlag = false;
 extern float temp_cal_factor;
 extern float lastWeight;
 extern RxData_Hx rxDataRck;
 extern RxData_Hx rxDataBtl;
 extern TxData_Hx txDataRck;


 void dataTask(void *arg){
  uint32_t abort_count = 0;
  int turnVar = 0;
  int SD_cont = 0;
  int abrtButton = 0;
  DataFrame dataFrame;

  xSemaphoreTake(stm.i2cMutex, pdTRUE);
  expander.setPinMode(0,B,INPUT); //input for abort button
  xSemaphoreGive(stm.i2cMutex);


  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  while(rxDataRck.request != WORK && rxDataRck.request == ASK){
    // lastWeightFlag = true;
    txDataRck.request = ANSWER;
    txDataRck.offset = lastWeight;
    esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
    perror("esp_now_send");
    Serial.println("SENDING ANSWER");
    Serial.println(txDataRck.request);
    Serial.println(txDataRck.offset);
    Serial.println(txDataRck.command);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  Serial.println("OUT OF LOOP");
  

  vTaskDelay(3000 / portTICK_PERIOD_MS);


 


  // !!!//DEBUG
  //InternalI2C<PWRData, TxData> i2cCOM(&stm.i2c, COM_ADRESS);

  vTaskDelay(100 / portTICK_PERIOD_MS);
 
  lastWeightFlag = true;
  while(1){

    txDataRck.request = WORK;
    txDataRck.offset = 0;
    txDataRck.command = NOTHING;
    
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

    loraFrameTanwa.tankWeight_val = rxDataBtl.weight;
    loraFrameTanwa.tankWeightRaw_val = (uint32_t) rxDataBtl.weight_raw;
    
    loraFrameTanwa.rocketWeight_val = rxDataRck.weight;
    loraFrameTanwa.rocketWeightRaw_val = (uint32_t) rxDataRck.weight_raw;

    //TODO dodać zapis butli
    snprintf(data, sizeof(data), "%.2f", loraFrameTanwa.rocketWeight_val);
    Serial.print("DATA TO BE SAVED: "); Serial.println(data);
    xQueueSend(stm.sdQueue_lastWeight, (void*)data, 0);
    

    loraFrameTanwa.vbat = voltageMeasure(VOLTAGE_MEASURE);
    //### memcpy(dataFrame.motorState, pwrData.motorState, sizeof(uint8_t[5]));
    loraFrameTanwa.motorState_1 = pwrData.motorState[0];
    loraFrameTanwa.motorState_2 = pwrData.motorState[1];
    loraFrameTanwa.motorState_3 = pwrData.motorState[2];
    loraFrameTanwa.motorState_4 = pwrData.motorState[3];

  

    loraFrameTanwa.tanWaState = StateMachine::getCurrentState();
    Serial.println(analogRead(IGN_TEST_CON_1));
    Serial.println(analogRead(IGN_TEST_CON_2));
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

    dataFrame.tanWaState = loraFrameTanwa.tanWaState;
    dataFrame.pressureSensor = loraFrameTanwa.pressureSensor;
    dataFrame.solenoid_fill = loraFrameTanwa.solenoid_fill;
    dataFrame.solenoid_depr = loraFrameTanwa.solenoid_depr;
    dataFrame.abortButton = loraFrameTanwa.abortButton;
    dataFrame.igniterContinouity_1 = loraFrameTanwa.igniterContinouity_1;
    dataFrame.igniterContinouity_2 = loraFrameTanwa.igniterContinouity_2;
    dataFrame.hxRequest_RCK = rxDataRck.request;
    dataFrame.vbat = loraFrameTanwa.vbat;
    dataFrame.motorState_1 = loraFrameTanwa.motorState_1;
    dataFrame.motorState_2 = loraFrameTanwa.motorState_2;
    dataFrame.motorState_3 = loraFrameTanwa.motorState_3;
    dataFrame.motorState_4 = loraFrameTanwa.motorState_4;
    dataFrame.rocketWeight_val = loraFrameTanwa.rocketWeight_val;
    dataFrame.tankWeight_val = loraFrameTanwa.tankWeight_val;
    dataFrame.rocketWeightRaw_val = loraFrameTanwa.rocketWeightRaw_val;
    dataFrame.tankWeightRaw_val = loraFrameTanwa.tankWeightRaw_val;

    createDataFrame(dataFrame, data);
    Serial.println(data);

    xQueueSend(stm.loraTxQueue, (void*)&loraFrameTanwa, 0);

    xQueueSend(stm.sdQueue, (void*)data, 0); 
      //TODO check polarity of pin ABORT on esp pull up or down
      xSemaphoreTake(stm.i2cMutex, pdTRUE);
      abrtButton = !expander.getPin(0,B);
      SD_cont = !expander.getPin(7,A);
      xSemaphoreGive(stm.i2cMutex);

      //TODO UNCOMMENT + change pin or solder correct pull up
      if(!abrtButton){// ABORT BUTTON

        xSemaphoreTake(stm.i2cMutex, pdTRUE);
        expander.setPinX(5,A,OUTPUT, OFF);
        xSemaphoreGive(stm.i2cMutex);

        txDataRck.request = ASK;
        txDataRck.offset = 1000;
        txDataRck.command = CALIBRATE_HX;
        esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
        vTaskDelay(10000 / portTICK_PERIOD_MS);

      // //################### real abort content ###################
      //   // abort_count++;
      //   // Serial.println("==================");
      //   // Serial.println("ABORT ++");
      //   // Serial.println("==================");
      //   // if(abort_count>=3){
      //   //   xSemaphoreTake(stm.i2cMutex, pdTRUE);
      //   //   expander.setPin(4,A,OFF);
      //   //   xSemaphoreGive(stm.i2cMutex);
      //   //   StateMachine::changeStateRequest(States::ABORT);
      //   //   Serial.println("ABORT BUTTON CONFIRMATION");
      //   //}
      //   //##########################################################
      }
      else{
        abort_count = 0;
        xSemaphoreTake(stm.i2cMutex, pdTRUE);
        expander.setPin(4,A,ON);
        xSemaphoreGive(stm.i2cMutex);
      }

      xSemaphoreTake(stm.i2cMutex, pdTRUE);
      expander.setPinX(5,A,OUTPUT, ON);
      xSemaphoreGive(stm.i2cMutex);
      // canSend();


      
    // Serial.println("RESeeeeeeeeeeeeeeeET");
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // xSemaphoreTake(stm.i2cMutex, pdTRUE);
    // expander.setPinX(4,A,OUTPUT,ON);
    // xSemaphoreGive(stm.i2cMutex);
    //DEBUG(data);
    // xSemaphoreTake(stm.i2cMutex, pdTRUE);
    // i2cCOM.getData(&pwrData);
    // xSemaphoreGive(stm.i2cMutex);
    
    Serial.println("\n\n\nCOM DATA:");
    Serial.print("SD status: "); Serial.println(SD_cont);
    Serial.print("ABORT: "); Serial.println(abrtButton);
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
 

    Serial.print("TANK WEIGHT: "); Serial.println(loraFrameTanwa.tankWeight_val);
    Serial.print("ROCKET WEIGHT: "); Serial.println(loraFrameTanwa.rocketWeight_val);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(loraFrameTanwa.rocketWeightRaw_val);
    // Serial.print("ROCKET WEIGHT OFFSET: "); Serial.println(rckWeight.get_offset());
    Serial.print("continuity 1 "); Serial.println(loraFrameTanwa.igniterContinouity_1);
    Serial.print("continuity 2 "); Serial.println(loraFrameTanwa.igniterContinouity_2);
    Serial.print("HX REQUEST "); Serial.println(loraFrameTanwa.hxRequest_RCK);


    // esp_now_send(adressObc, (uint8_t*) &loraFrameTanwa, sizeof(loraFrameTanwa));
    esp_now_send(adressObc, (uint8_t*) &dataFrame, sizeof(DataFrame));
    
    vTaskDelay(500 / portTICK_PERIOD_MS); 
  }
 }