#include "../include/tasks/tasks.h"
#include "../include/com/can.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <CAN.h>

void onReceive(int packetSize);
void canSend();


 extern RxData_Hx rxDataBtl_CAN;
 extern RxData_Hx rxDataRck_CAN;
extern PWRData pwrData_CAN;
 extern TxData_Hx txDataRck_CAN;
 extern TxData_Hx txDataBtl_CAN;
 extern TxData txData_CAN;

bool canInit(){
    

    CAN.setPins(CAN_RX, CAN_TX);
    if (!CAN.begin(500E3)) {
      Serial.println("Starting CAN failed!");
      while (1);
    }
      
    CAN.onReceive(onReceive);
    return true;
}


void canSend(){
    int DataSizeRck = 100;
    int DataSizeBtl = 100;
    int DataSize = 100;
    char* DataRck = new char[DataSizeRck];
    char* DataBtl = new char[DataSizeBtl];
    char* Data = new char[DataSize];
    
    DataSizeBtl = snprintf( DataBtl, "%s;%f;%d", &txDataBtl_CAN.request, &txDataBtl_CAN.offset, &txDataBtl_CAN.command);
    DataSizeRck = snprintf( DataRck, "%s;%f;%d", &txDataRck_CAN.request, &txDataRck_CAN.offset, &txDataRck_CAN.command);
    DataSize = snprintf( Data, "%d;%d", &txData_CAN.command, &txData_CAN.commandValue);
    int id = 0xa00011;

    uint8_t *buff =(uint8_t*)DataRck;
    for(int i = 0; i < DataSizeRck; i = i + 8){
        CAN.beginExtendedPacket(id);
        CAN.write(&buff[i], 8);
        //or
        for(int j = 0; j < 8; j++){
        //    CAN.write(buff[i+j]);
        }
        CAN.endPacket();
        id++;
    }
    free(buff);

    id = 0x000012;
    uint8_t *buff =(uint8_t*)DataBtl;
    for(int i = 0; i < DataSizeBtl; i = i + 8){
        CAN.beginExtendedPacket(id);
        CAN.write(&buff[i], 8);
        //or
        for(int j = 0; j < 8; j++){
        //    CAN.write(buff[i+j]);
        }
        CAN.endPacket();
        id++;
    }
    free(buff);

    id = 0x000013;
    uint8_t *buff =(uint8_t*)Data;
    for(int i = 0; i < DataSize; i = i + 8){
      CAN.beginExtendedPacket(id);
      CAN.write(&buff[i], 8);
      //or
      for(int j = 0; j < 8; j++){
      //    CAN.write(buff[i+j]);
      }
      CAN.endPacket();
      id++;
    }
    free(buff);

    free(DataRck);
    free(DataBtl);
    free(Data);
}


/// @brief 
/// @param packetSize 
void onReceive(int packetSize) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
        // Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
        // Remote transmission request, packet contains no data
        Serial.print("RTR ");
    }

    //   Serial.print("packet with id 0x");
    //   Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
        // Serial.print(" and requested length ");
        // Serial.println(CAN.packetDlc());
    } else {
        // Serial.print(" and length ");
        // Serial.println(packetSize);

        // only print packet data for non-RTR packets
        if(CAN.packetId() == 0xb00011){ //HX RCK
          
          int i = 0;
          char* Data = new char[100];
          while (CAN.available()) {    
              Serial.print((char)CAN.read());
              Data[i] = (char)CAN.read();
          }
          // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);

          sscanf( Data,  "%s;%0.2f;%d;%0.2f;", &rxDataRck_CAN.request, &rxDataRck_CAN.weight, &rxDataRck_CAN.weight_raw, &rxDataRck_CAN.temperature);
          //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;
          free(Data);
          Serial.println();

        }
        else if(CAN.packetId() == 0xb00012){ //HX Btl
              
          int i = 0;
          char* Data = new char[100];
          while (CAN.available()) {    
              Serial.print((char)CAN.read());
              Data[i] = (char)CAN.read();
          }
          // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);

          sscanf( Data,  "%s;%0.2f;%d;%0.2f;", &rxDataBtl_CAN.request, &rxDataBtl_CAN.weight, &rxDataBtl_CAN.weight_raw, &rxDataBtl_CAN.temperature);
          //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;
          free(Data);
          Serial.println();

        }
        else if(CAN.packetId() == 0xb00013){ //PWR

          int i = 0;
          char* Data = new char[100];
          while (CAN.available()) {    
              Serial.print((char)CAN.read());
              Data[i] = (char)CAN.read();
          }
          // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);

          sscanf( Data,  "%d;%d;%d;%d;%d;%d;%d;%d;%d;", &pwrData_CAN.tick, &pwrData_CAN.lastDoneCommandNum, &pwrData_CAN.motorState[0], &pwrData_CAN.motorState[1], &pwrData_CAN.motorState[2], &pwrData_CAN.motorState[3], &pwrData_CAN.adcValue[0], &pwrData_CAN.adcValue[1], &pwrData_CAN.adcValue[2]);
          //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;
          free(Data);
          Serial.println();

        }

    }

    Serial.println();
}

