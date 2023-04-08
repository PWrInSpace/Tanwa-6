 #include "../include/tasks/tasks.h"
#include <EEPROM.h>

 char data[SD_FRAME_SIZE] = {};
 PWRData pwrData;

 extern float temp_cal_factor;
 extern float lastWeight;

 void dataTask(void *arg){
  uint32_t abort_count = 0;
  int turnVar = 0;
  DataFrame dataFrame;
 
  expander.setPinMode(0,B,INPUT); //input for abort button

  //HX711
  rckWeight.begin(HX1_SDA, HX1_SCL);
  //rckWeight.set_gain(128);
  //rckWeight.wait_ready_timeout(); 
  rckWeight.set_scale(temp_cal_factor);
  

  Serial.print("ROCKET CAL FAC  "); Serial.println(rckWeight.get_scale(),3);
 
  rckWeight.tare();
  Serial.print("ROCKET OFFSET  "); Serial.println(rckWeight.get_offset());

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  int dd = rckWeight.get_scale() * lastWeight; // TODO 1000 = last saved measurement! from SD
  rckWeight.set_offset(rckWeight.get_offset()-dd);

  vTaskDelay(3000 / portTICK_PERIOD_MS);



  tankWeight.begin(HX2_SDA, HX2_SCL); //UWAGA!!!!!!!!!!!! 
  //tankWeight.set_gain(128);
  //tankWeight.wait_ready_timeout(); 
  tankWeight.set_scale(BIT_TO_GRAM_RATIO_TANK);
  // tankWeight.set_offset(OFFSET_TANK);
;
  // while (!tankWeight.wait_ready_retry() && !rckWeight.wait_ready_retry())
  // {
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }

  while (!tankWeight.wait_ready_retry())
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
    expander.setPin(2,B,turnVar);
    xSemaphoreGive(stm.i2cMutex);


    if(turnVar == 1)
      turnVar = 0;
    else
      turnVar = 1;

    
    dataFrame.tankWeight = tankWeight.get_units(10);
    dataFrame.tankWeightRaw = (uint32_t) tankWeight.get_value(10);
    
    dataFrame.rocketWeight = rckWeight.get_units(10);
    dataFrame.rocketWeightRaw = (uint32_t) rckWeight.get_value(10);

    snprintf(data, sizeof(data), "%.2f", dataFrame.rocketWeight);
    // Serial.print("DATA TO BE SAVED: "); Serial.println(data);
    xQueueSend(stm.sdQueue_lastWeight, (void*)data, 0);
    

    dataFrame.vbat = voltageMeasure(VOLTAGE_MEASURE);
    //### memcpy(dataFrame.motorState, pwrData.motorState, sizeof(uint8_t[5]));
    dataFrame.motorState_1 = pwrData.motorState[0];
    dataFrame.motorState_2 = pwrData.motorState[1];
    dataFrame.motorState_3 = pwrData.motorState[2];
    dataFrame.motorState_4 = pwrData.motorState[3];

  

    dataFrame.tanWaState = StateMachine::getCurrentState();
    
    dataFrame.igniterContinouity_1 = analogRead(IGN_TEST_CON_1) > 1000;
    dataFrame.igniterContinouity_2 = analogRead(IGN_TEST_CON_2) > 1000;


    createDataFrame(dataFrame, data);

    Serial.println(data);
    xQueueSend(stm.loraTxQueue, (void*)data, 0);

    xQueueSend(stm.sdQueue, (void*)data, 0); 
      //TODO check polarity of pin ABORT on esp pull up or down
      xSemaphoreTake(stm.i2cMutex, pdTRUE);
      //TODO UNCOMMENT + change pin or solder correct pull up
      if(!expander.getPin(0,B)){// ABORT BUTTON
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
      //   //DEBUG
      //   float temp_cal_factor2;
      //   temp_cal_factor2 = rckWeight.calibration(1000);
        
      //   Serial.println("temp cal factor2  "); Serial.println(temp_cal_factor2,3);
      //   vTaskDelay(2000 / portTICK_PERIOD_MS);

      //   int x_temp = 0, y_temp = 0;
        
      //   temp_cal_factor2 = temp_cal_factor2*1000;
          
      //   //Serial.println("temp cal factor*1000  "); Serial.println(temp_cal_factor2);
      //   if(abs(temp_cal_factor2)<=99999){    
      //     int tab[6];
      //     if(temp_cal_factor2<0)
      //       tab[5] = 1;
      //     else
      //       tab[5] = 0;
      //     for (int i=0; i<5; i++){
      //       x_temp=abs(temp_cal_factor2)/10;
      //       tab[i] = abs(temp_cal_factor2) - 10*x_temp;
      //       temp_cal_factor2 = x_temp;
      //       Serial.println("CONVERTED VALUE: "); Serial.println(tab[i]);
      //       EEPROM.write(i, tab[i]);
      //       EEPROM.commit();
      //     } 
      //     Serial.println("is negative: "); Serial.println(tab[5]);
      //     EEPROM.write(5, tab[5]);
      //     EEPROM.commit();
      //   }

      //   vTaskDelay(5000 / portTICK_PERIOD_MS); 


        
        
      }
      else{
        abort_count = 0;
        xSemaphoreTake(stm.i2cMutex, pdTRUE);
        expander.setPin(4,A,ON);
        xSemaphoreGive(stm.i2cMutex);
      }
       

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
    
    // Serial.println("\n\n\nCOM DATA:");
    // Serial.print("SD status: "); Serial.println(!expander.getPin(7,A));
    // Serial.print("ABORT: "); Serial.println(!expander.getPin(0,B));
    // Serial.print("BLINK: "); Serial.println(pwrData.tick);
    // Serial.print("LAST COMMAND: "); Serial.println(pwrData.lastDoneCommandNum);
    // Serial.print("MOTOR FILL COMMAND: "); Serial.println(pwrData.motorState[0]);
    // Serial.print("MOTOR FILL ADC: "); Serial.println(pwrData.adcValue[1]);
    // Serial.print("MOTOR DEPR COMMAND: "); Serial.println(pwrData.motorState[1]);
    // Serial.print("MOTOR DEPR ADC: "); Serial.println(pwrData.adcValue[2]);
    // Serial.print("MOTOR STATE QUICK DISCONNECT: "); Serial.println(pwrData.motorState[2]);
    

    // Serial.print("PRESSURE bit: "); Serial.println(pwrData.adcValue[0]);

    // dataFrame.pressureSensor = map(pwrData.adcValue[0],450, 4096, 0, 80);
    // Serial.print("PRESSURE in bars: "); Serial.println(dataFrame.pressureSensor);
    // Serial.print("TANWA VOLTAGE: "); Serial.println(voltageMeasure(VOLTAGE_MEASURE));
 

    // Serial.print("TANK WEIGHT: "); Serial.println(dataFrame.tankWeight);
    // Serial.print("ROCKET WEIGHT: "); Serial.println(dataFrame.rocketWeight);
    // Serial.print("ROCKET WEIGHT RAW: "); Serial.println(dataFrame.rocketWeightRaw);
    // Serial.print("ROCKET WEIGHT OFFSET: "); Serial.println(rckWeight.get_offset());
    // Serial.print("continuity 1 "); Serial.println(dataFrame.igniterContinouity_1);
    // Serial.print("continuity 2 "); Serial.println(dataFrame.igniterContinouity_2);

  
    // esp_now_send(adressObc, (uint8_t*) &dataFrame, sizeof(DataFrame));
    
    vTaskDelay(500 / portTICK_PERIOD_MS); 
  }
 }