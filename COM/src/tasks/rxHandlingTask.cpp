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



enum FrameStates {
    PLSS_,
    TANK_,
    DEPR_,
    QD_,
    ARM_,
    DISARM_,
    IGNITER_,
    TARE_RCK_,
    CALIBRATE_RCK_,
    TARE_TANK_,
    CALIBRATE_TANK_,
    SOFT_RESTART_ESP_,
    SOFT_RESTART_STM_,
    SET_CAL_FACTOR_,
    STAT_,
    HOLD_IN_,
    HOLD_OUT_,
    ABORT_,
    INVALID
};


FrameStates resolveOption(uint32_t input) {
    if( input == 0x11 ) return PLSS_;
    if( input ==  0x22) return TANK_;
    if( input == 0x33 ) return DEPR_;
    if( input == 0x44 ) return QD_;
    if( input == 0x55 ) return ARM_;
    if( input == 0x66 ) return DISARM_;
    if( input == 0x77 ) return IGNITER_;
    if( input == 0x88 ) return TARE_RCK_;
    if( input == 0x99 ) return CALIBRATE_RCK_;
    if( input == 0xAA ) return TARE_TANK_;
    if( input == 0xBB ) return CALIBRATE_TANK_;
    if( input == 0xCC ) return SOFT_RESTART_ESP_;
    if( input == 0xDD ) return SOFT_RESTART_STM_;
    if( input == 0xEE ) return SET_CAL_FACTOR_;
    if( input == 0xFF) return STAT_;
    if( input == 0x990) return HOLD_IN_;
    if( input == 0x100 ) return HOLD_OUT_;
    if( input == 0x420 ) return ABORT_;
    //...
    return INVALID;
 }


void rxHandlingTask(void* arg){

  RxData_Hx rxData_temp;
  LoRaCommandTanwa loraCommandTanwa_Rx = LoRaCommandTanwa_init_zero;

  uint8_t loraRx[LORA_RX_FRAME_SIZE];

  while(1){
    // Serial.println("RX TASK");
    //ESPNOW
    // if(xQueueReceive(stm.espNowRxQueue, (void*)&espNowCommand, 0) == pdTRUE){
    //   Serial.print("ESP NOW: ");
    //   Serial.println(espNowCommand.command);
    //   Serial.println(espNowCommand.commandValue);

    //   switch(espNowCommand.command){
    //     case PLSS_:
    //       if(xQueueSend(stm.loraTxQueue, (void*)data, 0) == pdTRUE){
    //         Serial.print("ESP NOW SEND VIA LORA: ");
    //         // Serial.println(data);
    //       }
    //       break;

    //     case IGNITER:
    //       digitalWrite(FIRE1, HIGH);
    //       digitalWrite(FIRE2, HIGH);
    //       break;

    //     case TARE_RCK:
    //       rckWeight.tare();
    //       break;
        
    //     case CALIBRATE_RCK:
    //       rckWeight.calibration(espNowCommand.commandValue);
    //       break;

    //     case TARE_TANK:
    //       tankWeight.tare();
    //       break;

    //     case CALIBRATE_TANK:
    //       tankWeight.calibration(espNowCommand.commandValue);
    //       break;

    //     case SOFT_ARM:
    //       digitalWrite(ARM_PIN, HIGH);
    //       break;

    //     case SOFT_DISARM:
    //       digitalWrite(ARM_PIN, LOW);
    //       break;
        
    //     case SOFT_RESTART:
    //       //RESET ESP COMMAND
    //       ESP.restart();
    //       //RESET STM
    //       // pwrCom.sendCommandMotor(0, RESET_COMMAND);
    //       break;

    //     default:
    //       xSemaphoreTake(stm.i2cMutex, pdTRUE);
    //       pwrCom.sendCommand(&espNowCommand);
    //       xSemaphoreGive(stm.i2cMutex);
    //       break;
    //   }
    // }

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
   


      
      // Serial.println(loraRx);
    //parser
      // string frame_array [50];
      // string loraRx_frame = loraRx;
      // string delimiter = ";";
      // string frame_elem;
      uint32_t frame_function;
      
      // frame_elem = loraRx_frame.substr(0, loraRx_frame.find(delimiter));
      // frame_array[0] = frame_elem;
      // loraRx_frame = loraRx_frame.substr(frame_elem.length());
      // loraRx_frame = loraRx_frame.erase(0, 1);
     

      if(loraCommandTanwa_Rx.lora_dev_id == 0x04 || loraCommandTanwa_Rx.lora_dev_id == 0x05 || loraCommandTanwa_Rx.lora_dev_id == 0x00){  
         int i = 1;
        // do {//decomposition of the frame
          
        //       frame_elem = loraRx_frame.substr(0, loraRx_frame.find(delimiter));
        //       frame_array[i] = frame_elem;
        //       loraRx_frame = loraRx_frame.substr(frame_elem.length());
        //       loraRx_frame = loraRx_frame.erase(0, 1);
        //       i++;
        //       cout<<"lora = "<<frame_elem<<endl;

        //   } while (loraRx_frame.compare("") != 0);

          frame_function = loraCommandTanwa_Rx.command;

          switch(resolveOption(frame_function)){

            case PLSS_:{
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

            case TANK_:{
              xSemaphoreTake(stm.i2cMutex, pdTRUE);
              // pwrCom.sendCommandMotor(MOTOR_FILL /*TODO add payload decoder*/);
              xSemaphoreGive(stm.i2cMutex);
              printf("MOTOOOR FILL\n");
              break;
            }
            case DEPR_:
              xSemaphoreTake(stm.i2cMutex, pdTRUE);
              // pwrCom.sendCommandMotor(MOTOR_DEPR /*TODO add payload decoder*/);
              xSemaphoreGive(stm.i2cMutex);
              printf("MOTOOOOR DEPR\n");
              break;

            case QD_:{
              xSemaphoreTake(stm.i2cMutex, pdTRUE);
              // pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT /*TODO add payload decoder*/);
              xSemaphoreGive(stm.i2cMutex);
              printf("MOTOOOR QUICK DISCONNECT\n");
              break;
            }
            case ARM_:{
              digitalWrite(ARM_PIN, HIGH);
              break;
            }

            case DISARM_:{
              digitalWrite(ARM_PIN, LOW);
              break;
            }

            case IGNITER_:{
              digitalWrite(FIRE1, HIGH);
              digitalWrite(FIRE2, HIGH);
              break;
            }

             case TARE_RCK_:{
              rckWeight.tare();
              break;
            }
            
            case CALIBRATE_RCK_:{
              // rckWeight.calibration(/*TODO add payload decoder*/);
              Serial.print("CAL FACTOR ROCKET: "); 
              Serial.println(rckWeight.get_scale());
              break;
            }

            case TARE_TANK_:{
              tankWeight.tare();
              break;
            }

            case CALIBRATE_TANK_:{
              // tankWeight.CustomCalibration(atoi(frame_array[3].c_str()),0);
              // tankWeight.CustomCalibration(atoi(frame_array[2].c_str()));


              // tankWeight.calibration(/*TODO add payload decoder*/);
              Serial.print("CAL FACTOR TANK: "); 
              Serial.println(tankWeight.get_scale());

              break;
            }

            case SOFT_RESTART_ESP_:{
              //RESET ESP COMMAND
              ESP.restart();
              break;
            }

            case SOFT_RESTART_STM_:{
              //RESET STM
              // pwrCom.sendCommandMotor(0, RESET_COMMAND);
              xSemaphoreTake(stm.i2cMutex, pdTRUE);
              expander.setPinX(4,A,OUTPUT,OFF);
              xSemaphoreGive(stm.i2cMutex);
        
              Serial.println("RESeeeeeeeeeeeeeeeET");
              vTaskDelay(100 / portTICK_PERIOD_MS);
              xSemaphoreTake(stm.i2cMutex, pdTRUE);
              expander.setPinX(4,A,OUTPUT,ON);
              xSemaphoreGive(stm.i2cMutex);

              break;
            }

          case SET_CAL_FACTOR_:{
            // if(/*TODO add payload decoder*/=="RCK"){
            //   rckWeight.set_scale(/*TODO add payload decoder*/);
            //   Serial.print("CAL FACTOR RCK = "); Serial.println(/*TODO add payload decoder*/);
            // }
            // else if (/*TODO add payload decoder*/=="TANK"){
            //   tankWeight.set_scale(atoi(/*TODO add payload decoder*/);
            //   Serial.print("CAL FACTOR TANK = "); Serial.println(/*TODO add payload decoder*/);
            // }
            
            break;
          }

            case STAT_:{
              // StateMachine::changeStateRequest(static_cast<States>(/*TODO add payload decoder*/);
              Serial.println("STATE CHANGE REQUEST");
              break;
            }
              
            case HOLD_IN_:{
              Serial.println("hold in");
              if(StateMachine::getCurrentState() != States::HOLD){
                if(StateMachine::changeStateRequest(States::HOLD) == false)
                  Serial.println("ERROR HOLD IN");
              }else
                Serial.println("ERROR HOLD IN");
              break;
            }
          
            case HOLD_OUT_:{
              Serial.println("hold out");
              if(StateMachine::getCurrentState() == States::HOLD){
                if(StateMachine::changeStateRequest(States::HOLD) == false)
                  Serial.println("ERROR HOLD OUT");
              }else
                Serial.println("ERROR HOLD OUT");
              break;
            }

            case ABORT_:{
              Serial.println("ABORT");
              StateMachine::changeStateRequest(States::ABORT);
              break;
            }

            default:
              break;
          }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}