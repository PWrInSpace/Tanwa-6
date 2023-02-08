#include "../include/tasks/tasks.h"

char data[SD_FRAME_SIZE] = {};
 PWRData pwrData;
//kod w tym tasku jest tylko do debugu 
void dataTask(void *arg){
  uint32_t abort_count = 0;
  int turnVar = 0;
  DataFrame dataFrame;
 
  expander.setPinMode(0,B,INPUT); //input for abort button

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
;
  while (tankWeight.wait_ready_retry() && rckWeight.wait_ready_retry())
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }



  // !!!//DEBUG
  //InternalI2C<PWRData, TxData> i2cCOM(&stm.i2c, COM_ADRESS);

  vTaskDelay(100 / portTICK_PERIOD_MS);
 
   
  while(1){
   
    xSemaphoreTake(stm.i2cMutex, pdTRUE);
    pwrCom.getData(&pwrData);
    xSemaphoreGive(stm.i2cMutex);


    xSemaphoreTake(stm.i2cMutex, pdTRUE);
    expander.setPinPullUp(2,B,turnVar);
    xSemaphoreGive(stm.i2cMutex);


    if(turnVar == 1)
      turnVar = 0;
    else
      turnVar = 1;

    
    dataFrame.tankWeight = tankWeight.get_units(1);
    dataFrame.tankWeightRaw = (uint32_t) tankWeight.get_value(1);
    
    dataFrame.rocketWeight = rckWeight.get_units(1);
    dataFrame.rocketWeightRaw = (uint32_t) rckWeight.get_value(1);
    

    dataFrame.vbat = voltageMeasure(VOLTAGE_MEASURE);
    // memcpy(dataFrame.motorState, pwrData.motorState, sizeof(uint8_t[5]));
    dataFrame.motorState_1 = pwrData.motorState[0];
    dataFrame.motorState_2 = pwrData.motorState[1];
    dataFrame.motorState_3 = pwrData.motorState[2];
    dataFrame.motorState_4 = pwrData.motorState[3];

  

    dataFrame.tanWaState = StateMachine::getCurrentState();
    
    dataFrame.igniterContinouity_1 = analogRead(IGN_TEST_CON_1) > 1000;
    dataFrame.igniterContinouity_2 = analogRead(IGN_TEST_CON_2) > 1000;


    createDataFrame(dataFrame, data);

    Serial.println(data);
    // xQueueSend(stm.loraTxQueue, (void*)data, 0);

    xQueueSend(stm.sdQueue, (void*)data, 0); 
      //TODO check polarity of pin ABORT on esp pull up or down
      // xSemaphoreTake(stm.i2cMutex, pdTRUE);
      if(digitalRead(ABORT_ESP)==0){// ABORT BUTTON
        abort_count++;
        Serial.println("==================");
        Serial.println("ABORT ++");
        Serial.println("==================");
        if(abort_count>=3){
          xSemaphoreTake(stm.i2cMutex, pdTRUE);
          expander.setPinPullUp(1,B,ON);
          xSemaphoreGive(stm.i2cMutex);
          StateMachine::changeStateRequest(States::ABORT);
          Serial.println("ABORT BUTTON CONFIRMATION");
        }
      }
      else{
        abort_count = 0;
        xSemaphoreTake(stm.i2cMutex, pdTRUE);
        expander.setPinPullUp(1,B,OFF);
        xSemaphoreGive(stm.i2cMutex);
      }
       

      // xSemaphoreGive(stm.i2cMutex);

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
    Serial.print("continuity 1 "); Serial.println(dataFrame.igniterContinouity_1);
    Serial.print("continuity 2 "); Serial.println(dataFrame.igniterContinouity_2);

  
    esp_now_send(adressObc, (uint8_t*) &dataFrame, sizeof(DataFrame));
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}