#include "../include/tasks/tasks.h"
#include <string>
#include <iostream>
#include "lora.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

using namespace std;
RxData_Hx rxDataRck;
RxData_Hx rxDataBtl;

TxData_Hx txDataRck;
TxData_Hx txDataBtl;
// extern MCP23017 expander;
uint8_t data_buff[SD_FRAME_SIZE];
extern bool lastWeightFlag;

void rxHandlingTask(void* arg){

  RxData_Hx rxData_temp;
  RxData_OBC rxDataOBC_temp;
  LoRaCommandTanwa loraCommandTanwa_Rx = LoRaCommandTanwa_init_zero;

  while(1){
    Serial.println("RX TASK");
    // ESPNOW
    if(xQueueReceive(stm.espNowRxQueueObc, (void*)&rxDataOBC_temp, 0) == pdTRUE){
      Serial.print("ESP NOW: ");
      Serial.println(rxDataOBC_temp.command);
      Serial.println(rxDataOBC_temp.commandValue);

      switch(rxDataOBC_temp.command){
       
        case STATE_ESP:{
          StateMachine::changeStateRequest(static_cast<States>(rxDataOBC_temp.commandValue));
          Serial.println("STATE CHANGE REQUEST");
          break;
        }

        case ABORT_ESP:{
          Serial.println("#################### ABOR T###########################");
          StateMachine::changeStateRequest(States::ABORT);
          break;
        }

        case HOLD_IN_ESP:{
          Serial.println("hold in");
          if(StateMachine::getCurrentState() != States::HOLD)
          {
            if(StateMachine::changeStateRequest(States::HOLD) == false)
              Serial.println("ERROR HOLD IN");
          }else
            Serial.println("ERROR HOLD IN");
          break;
        }

        case HOLD_OUT_ESP:{
          Serial.println("hold out");
          if(StateMachine::getCurrentState() == States::HOLD)
          {
            if(StateMachine::changeStateRequest(States::HOLD) == false)
              Serial.println("ERROR HOLD OUT");
          }else
            Serial.println("ERROR HOLD OUT");
           
          break;
        }

        case FILL_ESP:{
            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_FILL, rxDataOBC_temp.commandValue);
            xSemaphoreGive(stm.i2cMutex);
            printf("MOTOOOR FILL\n");
            break;
        }
          
        case FILL_TIME_ESP:{
          xSemaphoreTake(stm.i2cMutex, pdTRUE);
          pwrCom.sendCommandMotor(MOTOR_FILL, TIMED_OPEN_VALVE, rxDataOBC_temp.commandValue);
          xSemaphoreGive(stm.i2cMutex);
          printf("MOTOOOR FILL TIME OPEN\n");
          break;
        }
        
        case DEPR_ESP:{
          xSemaphoreTake(stm.i2cMutex, pdTRUE);
          pwrCom.sendCommandMotor(MOTOR_DEPR, rxDataOBC_temp.commandValue);
          xSemaphoreGive(stm.i2cMutex);
          printf("MOTOOOR DEPR\n");
          break;
        }
        
        case QD_ESP:{
          xSemaphoreTake(stm.i2cMutex, pdTRUE);
          pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, rxDataOBC_temp.commandValue);
          xSemaphoreGive(stm.i2cMutex);
          printf("MOTOOOR QD\n");
          break;
        }

        case SOFT_ARM_ESP:{
          digitalWrite(ARM_PIN, HIGH);
          break;
        }

        case SOFT_DISARM_ESP:{
          digitalWrite(ARM_PIN, LOW);
          break;
        }
        
        case FIRE_ESP:{
          digitalWrite(FIRE1, HIGH);
          digitalWrite(FIRE2, HIGH);
          break;
        }
        
        case SOFT_RESTART_ESP_ESP:{
          esp_restart();
          break;
        }

        case SOFT_RESTART_STM_ESP:{
          xSemaphoreTake(stm.i2cMutex, pdTRUE);
          expander.setPinX(4,A,OUTPUT,OFF);
          xSemaphoreGive(stm.i2cMutex);
    
          Serial.println("################# RESET STM ####################");
          vTaskDelay(100 / portTICK_PERIOD_MS);
          xSemaphoreTake(stm.i2cMutex, pdTRUE);
          expander.setPinX(4,A,OUTPUT,ON); //TODO CHECK THIS PIN!
          xSemaphoreGive(stm.i2cMutex);
          break;
        }
        
        case CALIBRATE_RCK_ESP:{
          txDataRck.request = ASK;
          txDataRck.offset = rxDataOBC_temp.commandValue;
          txDataRck.command = CALIBRATE_HX;
          esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
          vTaskDelay(10000 / portTICK_PERIOD_MS);
          break;
        }

        case TARE_RCK_ESP:{
          txDataRck.request = ASK;
          txDataRck.command = TARE_HX;
          esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          break;
        }
        
        case SET_CAL_FACTOR_RCK_ESP:{
          txDataRck.request = ASK;
          txDataRck.offset = rxDataOBC_temp.commandValue;
          txDataRck.command = SET_CAL_FACTOR_HX;
          esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
          vTaskDelay(10000 / portTICK_PERIOD_MS);
          break;
        }

        case SET_OFFSET_RCK_ESP:{
          txDataRck.request = ASK;
          txDataRck.offset = rxDataOBC_temp.commandValue;
          txDataRck.command = SET_OFFSET_HX;
          esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
          vTaskDelay(10000 / portTICK_PERIOD_MS);
          break;
        }
        
        case CALIBRATE_TANK_ESP:{
          txDataBtl.request = ASK;
          txDataBtl.offset = rxDataOBC_temp.commandValue;
          txDataBtl.command = CALIBRATE_HX;
          esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
          vTaskDelay(10000 / portTICK_PERIOD_MS);
          break;
        }

        case TARE_TANK_ESP:{
          txDataBtl.request = ASK;
          txDataBtl.command = TARE_HX;
          esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          break;
        }

        case SET_CAL_FACTOR_TANK_ESP:{
          txDataBtl.request = ASK;
          txDataBtl.offset = rxDataOBC_temp.commandValue;
          txDataBtl.command = SET_CAL_FACTOR_HX;
          esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          break;
        }

        case SET_OFFSET_TANK_ESP:{
          txDataBtl.request = ASK;
          txDataBtl.offset = rxDataOBC_temp.commandValue;
          txDataBtl.command = SET_OFFSET_HX;
          esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          break;
        }

        case PLSS_ESP:{
          LoRaFrameTanwa loraFrameTanwa_local = LoRaFrameTanwa_init_zero;
          pb_ostream_t stream = pb_ostream_from_buffer(data_buff, sizeof(data_buff));
          bool status = pb_encode(&stream, LoRaFrameTanwa_fields, &loraFrameTanwa_local);
          if (!status)
          {
              printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
          }
          if(xQueueSend(stm.loraTxQueue, (void*)&data_buff, 0) == pdTRUE){

            Serial.print("ESP NOW SEND VIA LORA: ");
            // Serial.println(data);//debug
          }  
          break;
        }

        case INTERFACE_RCK_ESP:{
          if(rxDataOBC_temp.commandValue == INTERFACE_CAN_ESP)
          {
            ////
          }else if(rxDataOBC_temp.commandValue == INTERFACE_ESPNOW_ESP)
          {
              ////
          }
          break;
        }

        
        case INTERFACE_TANK_ESP:{
          if(rxDataOBC_temp.commandValue == INTERFACE_CAN_ESP)
          {
            /////
          }else if(rxDataOBC_temp.commandValue == INTERFACE_ESPNOW_ESP)
          {
            ////
          }
        
          break;
        }
      
        case INTERFACE_MCU_ESP:{
          if(rxDataOBC_temp.commandValue == INTERFACE_CAN_ESP)
          {
           ////
          }else if(rxDataOBC_temp.commandValue == INTERFACE_I2C_ESP)
          {
              /////
          }
        
          break;
        }

        case LORA_FREQ_ESP:{
          ////
          break;
        }

        case LORA_TIME_ESP:{
          ////
          break;
        }

    
        default:{
          // xSemaphoreTake(stm.i2cMutex, pdTRUE);
          // pwrCom.sendCommand(&espNowCommand);
          // xSemaphoreGive(stm.i2cMutex);
          break;
        }
      }
    }

      //TODO FLAG FROM HX THAT THERE IS NO COMMUNICATION
     if(xQueueReceive(stm.espNowRxQueueHxRck, (void*)&rxData_temp, 0) == pdTRUE){

      Serial.print("ESP NOW FROM HX: ");
      Serial.print("REQ:   ");Serial.println(rxData_temp.request);

      if(rxData_temp.request == ASK && lastWeightFlag == true){
        rxDataRck.request = rxData_temp.request;
        txDataRck.request = ANSWER;
        txDataRck.offset = rxDataRck.weight;
        esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
        perror("esp_now_send");
        
      }
      else{
        memcpy(&rxDataRck, &rxData_temp, sizeof(rxData_temp));
      }
      // Serial.print("WEIGHT:   ");Serial.println(rxDataRck.weight);
      // Serial.print("WEIGHT RAW:   ");Serial.println((uint32_t)rxDataRck.weight_raw);
      // Serial.print("TEMP:   ");Serial.println(rxDataRck.temperature);
     }


    //LORA 
    if(xQueueReceive(stm.loraRxQueue, (void*)&loraCommandTanwa_Rx, 0) == pdTRUE){
      Serial.print("LORAAAAAAAAAAAAAAAAAAAAAAA: ");
      uint32_t lora_command;
      uint32_t lora_payload;
    
      if(loraCommandTanwa_Rx.lora_dev_id == 0x04 || loraCommandTanwa_Rx.lora_dev_id == 0x05 || loraCommandTanwa_Rx.lora_dev_id == 0x00){  
        int i = 1;
    
        lora_command = loraCommandTanwa_Rx.command;
        lora_payload = loraCommandTanwa_Rx.payload;

  
        switch(lora_command){

          case STATE_ESP:{
            StateMachine::changeStateRequest(static_cast<States>(lora_payload));
            Serial.println("STATE CHANGE REQUEST");
            break;
          }

          case ABORT_ESP:{
            Serial.println("####################  ABORT  ###########################");
            StateMachine::changeStateRequest(States::ABORT);
            break;
          }

          case HOLD_IN_ESP:{
            Serial.println("hold in");
            if(StateMachine::getCurrentState() != States::HOLD)
            {
              if(StateMachine::changeStateRequest(States::HOLD) == false)
                Serial.println("ERROR HOLD IN");
            }else
              Serial.println("ERROR HOLD IN");
            break;
          }

          case HOLD_OUT_ESP:{
            Serial.println("hold out");
            if(StateMachine::getCurrentState() == States::HOLD)
            {
              if(StateMachine::changeStateRequest(States::HOLD) == false)
                Serial.println("ERROR HOLD OUT");
            }else
              Serial.println("ERROR HOLD OUT");
            
            break;
          }

          case FILL_ESP:{
              xSemaphoreTake(stm.i2cMutex, pdTRUE);
              pwrCom.sendCommandMotor(MOTOR_FILL, lora_payload);
              xSemaphoreGive(stm.i2cMutex);
              printf("MOTOOOR FILL\n");
              break;
          }
          
          case FILL_TIME_ESP:{
            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_FILL, TIMED_OPEN_VALVE, lora_payload);
            xSemaphoreGive(stm.i2cMutex);
            printf("MOTOOOR FILL TIME OPEN\n");
            break;
          }
          
          case DEPR_ESP:{
            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_DEPR, lora_payload);
            xSemaphoreGive(stm.i2cMutex);
            printf("MOTOOOR DEPR\n");
            break;
          }
          
          case QD_ESP:{
            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, lora_payload);
            xSemaphoreGive(stm.i2cMutex);
            printf("MOTOOOR QD\n");
            break;
          }

          case SOFT_ARM_ESP:{
            digitalWrite(ARM_PIN, HIGH);
            break;
          }

          case SOFT_DISARM_ESP:{
            digitalWrite(ARM_PIN, LOW);
            break;
          }
          
          case FIRE_ESP:{
            digitalWrite(FIRE1, HIGH);
            digitalWrite(FIRE2, HIGH);
            break;
          }
          
          case SOFT_RESTART_ESP_ESP:{
            esp_restart();
            break;
          }

          case SOFT_RESTART_STM_ESP:{
            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            expander.setPinX(4,A,OUTPUT,OFF);
            xSemaphoreGive(stm.i2cMutex);
      
            Serial.println("#################RESET STM####################");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            expander.setPinX(4,A,OUTPUT,ON); //TODO CHECK THIS PIN!
            xSemaphoreGive(stm.i2cMutex);
            break;
          }
          
          case CALIBRATE_RCK_ESP:{
            txDataRck.request = ASK;
            txDataRck.offset = lora_payload;
            txDataRck.command = CALIBRATE_HX;
            esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            break;
          }

          case TARE_RCK_ESP:{
            txDataRck.request = ASK;
            txDataRck.command = TARE_HX;
            esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
          }
          
          case SET_CAL_FACTOR_RCK_ESP:{
            txDataRck.request = ASK;
            txDataRck.offset = lora_payload;
            txDataRck.command = SET_CAL_FACTOR_HX;
            esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            break;
          }

          case SET_OFFSET_RCK_ESP:{
            txDataRck.request = ASK;
            txDataRck.offset = lora_payload;
            txDataRck.command = SET_OFFSET_HX;
            esp_now_send(adressHxRck, (uint8_t*) &txDataRck, sizeof(TxData_Hx));
            vTaskDelay(10000 / portTICK_PERIOD_MS);
            break;
          }
          
          case CALIBRATE_TANK_ESP:{
              txDataBtl.request = ASK;
              txDataBtl.offset = lora_payload;
              txDataBtl.command = CALIBRATE_HX;
              esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
              vTaskDelay(10000 / portTICK_PERIOD_MS);
              break;
          }

          case TARE_TANK_ESP:{
            txDataBtl.request = ASK;
            txDataBtl.command = TARE_HX;
            esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
          }

          case SET_CAL_FACTOR_TANK_ESP:{
            txDataBtl.request = ASK;
            txDataBtl.offset = lora_payload;
            txDataBtl.command = SET_CAL_FACTOR_HX;
            esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
          }

          case SET_OFFSET_TANK_ESP:{
            txDataBtl.request = ASK;
            txDataBtl.offset = lora_payload;
            txDataBtl.command = SET_OFFSET_HX;
            esp_now_send(adressHxBtl, (uint8_t*) &txDataBtl, sizeof(TxData_Hx));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            break;
          }

          case PLSS_ESP:{
            LoRaFrameTanwa loraFrameTanwa_local = LoRaFrameTanwa_init_zero;
            pb_ostream_t stream = pb_ostream_from_buffer(data_buff, sizeof(data_buff));
            bool status = pb_encode(&stream, LoRaFrameTanwa_fields, &loraFrameTanwa_local);
            if (!status)
            {
                printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
            }
            if(xQueueSend(stm.loraTxQueue, (void*)&data_buff, 0) == pdTRUE){

              Serial.print("ESP NOW SEND VIA LORA: ");
              // Serial.println(data);//debug
            }  
            break;
          }

          case INTERFACE_RCK_ESP:{
            if(lora_payload == INTERFACE_CAN_ESP)
            {
              ////
            }else if(lora_payload == INTERFACE_ESPNOW_ESP)
            {
                ////
            }
            break;
          }

          
          case INTERFACE_TANK_ESP:{
            if(lora_payload == INTERFACE_CAN_ESP)
            {
              /////
            }else if(lora_payload == INTERFACE_ESPNOW_ESP)
            {
              ////
            }
          
            break;
          }
        
          case INTERFACE_MCU_ESP:{
            if(lora_payload == INTERFACE_CAN_ESP)
            {
            ////
            }else if(lora_payload == INTERFACE_I2C_ESP)
            {
                /////
            }
          
            break;
          }

          case LORA_FREQ_ESP:{
            ////
            break;
          }

          case LORA_TIME_ESP:{
            ////
            break;
          }

      
          default:{
            // xSemaphoreTake(stm.i2cMutex, pdTRUE);
            // pwrCom.sendCommand(&espNowCommand);
            // xSemaphoreGive(stm.i2cMutex);
            break;
          }
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}