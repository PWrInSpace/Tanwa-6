#include <Arduino.h>
#include <SPI.h>
#include "../include/tasks/tasks.h"
#include "lora.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "../include/config/config.h"
#include "../include/structs/SoftToolsManagment.h"
#include "../include/structs/commStructs.h"
#include "../include/com/now.h"
#include "../include/com/can.h"
#include <esp_wifi.h>
#include <MCP23017.h>
#include <EEPROM.h>

SoftwareToolsManagment stm;
InternalI2C<PWRData, TxData> pwrCom(&stm.i2c, COM_ADRESS);
// HX711
HX711_api rckWeight;
HX711_api tankWeight;

MCP23017 expander = MCP23017(&stm.i2c,MCP_ADDRESS,RST);

float temp_cal_factor;

void setup() {

  Serial.begin(115200);
  pinInit();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA , adressTanwa);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  //ledcSetup(0,2000,8);// PWM FOR BUZZER
 // ledcAttachPin(BUZZER, 0);
  
  
  stm.i2c.begin(I2C_SDA, I2C_SCL, 100E3);
  stm.i2c.setTimeOut(20);
  stm.spi.begin();
  //DEBUG
//#######################################
  while(!expander.Init()){
    vTaskDelay(500);
    Serial.println("Not connected!1");
    expander.softReset();
  }

    if(!expander.Init())
      Serial.println("Not connected!");

    else{
  
      expander.softReset(); //WARNING - EXPANDER ON SECOND PCB NEEDED THIS!!!!!! CHECK BEHAVIOUR FOR 1ST ONE

      Serial.println("CONNECTED");


    expander.setPinX(0,B,INPUT,ON); //input for abort button
    expander.setPinX(7,A,INPUT,ON); //input for SD card connection check

    // all leds off
    expander.setPinX(1,B,OUTPUT, OFF);
    expander.setPinX(2,B,OUTPUT, ON);
    expander.setPinX(3,B,OUTPUT, ON);
    expander.setPinX(4,B,OUTPUT, ON);
    expander.setPinX(5,B,OUTPUT, ON);
    expander.setPinX(6,B,OUTPUT, ON);
    expander.setPinX(7,B,OUTPUT, ON);
    expander.setPinX(6,A,OUTPUT, ON);
    expander.setPinX(5,A,OUTPUT, ON);
    expander.setPinX(4,A,OUTPUT, ON);
      // expander.setPinPullUp(1,B,ON);// LED1
      // expander.setPinPullUp(2,B,OFF);// LED2
      // expander.setPinPullUp(3,B,OFF);// LED3
      // expander.setPinPullUp(4,B,OFF);// LED4
      // expander.setPinPullUp(5,B,OFF);// LED5
      // expander.setPinPullUp(6,B,OFF);// LED6
      // expander.setPinPullUp(7,B,OFF);// LED7
      // expander.setPinPullUp(6,A,OFF);// LED8
      // expander.setPinPullUp(5,A,ON);// LED9
      // expander.setPinPullUp(4,A,OFF);// LED10
    }

 //############################
  nowInit();
  nowAddPeer(adressObc, 0);
  nowAddPeer(adressHxRck, 0);
  nowAddPeer(adressHxBtl, 0);
  canInit();


  //ledcWriteTone(0, 1000);
 // ledcWrite(0, 255);
 // vTaskDelay(100 / portTICK_PERIOD_MS);
 // ledcWrite(0, 0);

  stm.sdQueue = xQueueCreate(SD_QUEUE_LENGTH, sizeof(char[SD_FRAME_SIZE]));
  stm.sdQueue_lastWeight = xQueueCreate(SD_QUEUE_LENGTH, sizeof(char[SD_FRAME_SIZE]));
  stm.loraTxQueue = xQueueCreate(LORA_TX_QUEUE_LENGTH, sizeof(uint8_t[LORA_TX_FRAME_SIZE]));
  stm.loraRxQueue = xQueueCreate(LORA_RX_QUEUE_LENGTH, sizeof(LoRaFrameTanwa[LORA_RX_FRAME_SIZE]));
  stm.espNowRxQueueObc = xQueueCreate(ESP_NOW_QUEUE_LENGTH, sizeof(RxData_OBC));
  stm.espNowRxQueueHxRck = xQueueCreate(10, sizeof(RxData_Hx));
  stm.espNowRxQueueHxBtl =  xQueueCreate(10, sizeof(RxData_Hx)); 

  stm.i2cMutex = xSemaphoreCreateMutex();
  stm.spiMutex = xSemaphoreCreateMutex();

  vTaskDelay(25 / portTICK_PERIOD_MS);


//  xTaskCreatePinnedToCore(canTask, "CAN task", 20000, NULL, 3, &stm.canTask, APP_CPU_NUM);
//  xTaskCreatePinnedToCore(loraTask, "LoRa task", 20000, NULL, 3, &stm.loraTask, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(rxHandlingTask, "Rx handling task", 20000, NULL, 5, &stm.rxHandlingTask, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(sdTask,   "SD task",   20000, NULL, 3, &stm.sdTask,   APP_CPU_NUM);
  xTaskCreatePinnedToCore(dataTask, "Data task", 40000, NULL, 3, &stm.dataTask, APP_CPU_NUM);
  xTaskCreatePinnedToCore(stateTask, "State task", 20000, NULL, 10, &stm.stateTask, APP_CPU_NUM);
 // xTaskCreatePinnedToCore(buzzerTask, "Buzzer task", 20000, NULL, 1, &stm.buzzerTask, APP_CPU_NUM);


   if(stm.sdQueue == NULL || stm.loraTxQueue == NULL){
     ESP.restart();
   }

   if(stm.spiMutex == NULL || stm.i2cMutex == NULL){
     ESP.restart();
   }

  //   if(stm.i2cMutex == NULL){
  //   ESP.restart();
  // }

  /*
  if(stm.loraTask == NULL || stm.sdTask == NULL || stm.dataTask == NULL){
   ESP.restart();
  }*/
  StateMachine::changeStateRequest(States::IDLE);
  
  vTaskDelete(NULL);
}

void loop() {}