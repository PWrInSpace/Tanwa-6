 #include "../include/tasks/tasks.h"
#include <EEPROM.h>

 char data[SD_FRAME_SIZE] = {};
 PWRData pwrData;
 bool lastWeightFlag = false;
 extern float temp_cal_factor;
 extern float lastWeight;
 extern RxData_Hx rxDataRck;
 extern RxData_Hx rxDataBtl;
 extern TxData_Hx txDataRck;
 extern TxData_Hx txDataRck;

 void dataTask(void *arg){
  uint32_t abort_count = 0;
  int turnVar = 0;
  DataFrame dataFrame;

   
  xSemaphoreTake(stm.i2cMutex, pdTRUE);
  expander.setPinMode(0,B,INPUT); //input for abort button
  xSemaphoreGive(stm.i2cMutex);
 
  //HX711
  // rckWeight.begin(HX1_SDA, HX1_SCL);
  //rckWeight.set_gain(128);
  //rckWeight.wait_ready_timeout(); 
  // rckWeight.set_scale(temp_cal_factor);
  

  // Serial.print("ROCKET CAL FAC  "); Serial.println(rckWeight.get_scale(),3);
 
  // rckWeight.tare();
  // Serial.print("ROCKET OFFSET  "); Serial.println(rckWeight.get_offset());

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



  // tankWeight.begin(HX2_SDA, HX2_SCL); //UWAGA!!!!!!!!!!!! 
  //tankWeight.set_gain(128);
  //tankWeight.wait_ready_timeout(); 
  // tankWeight.set_scale(BIT_TO_GRAM_RATIO_TANK);
  // tankWeight.set_offset(OFFSET_TANK);
  // while (!tankWeight.wait_ready_retry() && !rckWeight.wait_ready_retry())
  // {
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }

  // while (!tankWeight.wait_ready_retry())
  // {
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }






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

    
    dataFrame.tankWeight = rxDataBtl.weight;
    dataFrame.tankWeightRaw = (uint32_t) rxDataBtl.weight_raw;
    
    dataFrame.rocketWeight = rxDataRck.weight;
    dataFrame.rocketWeightRaw = (uint32_t) rxDataRck.weight_raw;

    //TODO dodać zapis butli
    snprintf(data, sizeof(data), "%.2f", dataFrame.rocketWeight);
    Serial.print("DATA TO BE SAVED: "); Serial.println(data);
    xQueueSend(stm.sdQueue_lastWeight, (void*)data, 0);
    

    dataFrame.vbat = voltageMeasure(VOLTAGE_MEASURE);
    //### memcpy(dataFrame.motorState, pwrData.motorState, sizeof(uint8_t[5]));
    dataFrame.motorState_1 = pwrData.motorState[0];
    dataFrame.motorState_2 = pwrData.motorState[1];
    dataFrame.motorState_3 = pwrData.motorState[2];
    dataFrame.motorState_4 = pwrData.motorState[3];

  

    dataFrame.tanWaState = StateMachine::getCurrentState();
    Serial.println(analogRead(IGN_TEST_CON_1));
    Serial.println(analogRead(IGN_TEST_CON_2));
    dataFrame.igniterContinouity_1 = analogRead(IGN_TEST_CON_1) > 1000;
    dataFrame.igniterContinouity_2 = analogRead(IGN_TEST_CON_2) > 1000;
    dataFrame.hxRequest = rxDataRck.request;

    createDataFrame(dataFrame, data);
    Serial.println(data);
    xQueueSend(stm.loraTxQueue, (void*)data, 0);

    xQueueSend(stm.sdQueue, (void*)data, 0); 
      //TODO check polarity of pin ABORT on esp pull up or down
      xSemaphoreTake(stm.i2cMutex, pdTRUE);
      int abrtButton = expander.getPin(0,B);
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
    Serial.print("SD status: "); Serial.println(!expander.getPin(7,A));
    Serial.print("ABORT: "); Serial.println(!expander.getPin(0,B));
    Serial.print("BLINK: "); Serial.println(pwrData.tick);
    Serial.print("LAST COMMAND: "); Serial.println(pwrData.lastDoneCommandNum);
    Serial.print("MOTOR FILL COMMAND: "); Serial.println(pwrData.motorState[0]);
    Serial.print("MOTOR FILL ADC: "); Serial.println(pwrData.adcValue[1]);
    Serial.print("MOTOR DEPR COMMAND: "); Serial.println(pwrData.motorState[1]);
    Serial.print("MOTOR DEPR ADC: "); Serial.println(pwrData.adcValue[2]);
    Serial.print("MOTOR STATE QUICK DISCONNECT: "); Serial.println(pwrData.motorState[2]);
    

    Serial.print("PRESSURE bit: "); Serial.println(pwrData.adcValue[0]);

    dataFrame.pressureSensor = map(pwrData.adcValue[0],450, 4096, 0, 80);
    Serial.print("PRESSURE in bars: "); Serial.println(dataFrame.pressureSensor);
    Serial.print("TANWA VOLTAGE: "); Serial.println(voltageMeasure(VOLTAGE_MEASURE));
 

    Serial.print("TANK WEIGHT: "); Serial.println(dataFrame.tankWeight);
    Serial.print("ROCKET WEIGHT: "); Serial.println(dataFrame.rocketWeight);
    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(dataFrame.rocketWeightRaw);
    // Serial.print("ROCKET WEIGHT OFFSET: "); Serial.println(rckWeight.get_offset());
    Serial.print("continuity 1 "); Serial.println(dataFrame.igniterContinouity_1);
    Serial.print("continuity 2 "); Serial.println(dataFrame.igniterContinouity_2);
    Serial.print("HX REQUEST "); Serial.println(dataFrame.hxRequest);

  
    // esp_now_send(adressObc, (uint8_t*) &dataFrame, sizeof(DataFrame));
    
    vTaskDelay(500 / portTICK_PERIOD_MS); 
  }
 }