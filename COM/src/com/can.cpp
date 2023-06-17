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


RxData_Hx rxDataBtl_CAN;
RxData_Hx rxDataRck_CAN;
PWRData pwrData_CAN;
TxData_Hx txDataRck_CAN;
TxData_Hx txDataBtl_CAN;
TxData txData_CAN;

int start_frame_flag = 0;
int end_frame_flag = 0;
char Data1[8] = {'\0'};

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
    // size_t DataSizeRck;
    // size_t DataSizeBtl;
    // size_t DataSize;

    // DataSizeBtl = snprintf(NULL, 0, "%s;%f;%d", txDataBtl_CAN.request.c_str(), txDataBtl_CAN.offset, txDataBtl_CAN.command) + 1;
    // DataSizeRck = snprintf(NULL, 0 , "%s;%f;%d", txDataRck_CAN.request.c_str(), txDataRck_CAN.offset, txDataRck_CAN.command) + 1;
    // DataSize = snprintf(NULL, 0, "%d;%d", txData_CAN.command, txData_CAN.commandValue) + 1;


    // char* DataRck = new char[DataSizeRck];
    // char* DataBtl = new char[DataSizeBtl];
    // char* Data = new char[DataSize];

    // snprintf(DataRck, DataSizeRck, "%s;%f;%d", txDataBtl_CAN.request.c_str(), txDataBtl_CAN.offset, txDataBtl_CAN.command);
    // snprintf(DataBtl, DataSizeBtl,"%s;%f;%d", txDataRck_CAN.request.c_str(), txDataRck_CAN.offset, txDataRck_CAN.command);
    // snprintf(Data, DataSize, "%d;%d", txData_CAN.command, txData_CAN.commandValue);

    // int id = 0xa00011;

    // uint8_t *buff =(uint8_t*)DataRck;
    // for(int i = 0; i < DataSizeRck; i = i + 8){
    //     CAN.beginExtendedPacket(id);
    //     CAN.write(&buff[i], 8);
    //     //or
    //     for(int j = 0; j < 8; j++){
    //     //    CAN.write(buff[i+j]);
    //     }
    //     CAN.endPacket();
    //     id++;
    // }
    // free(buff);

    // id = 0x000012;
    // uint8_t *buff2 =(uint8_t*)DataBtl;
    // for(int i = 0; i < DataSizeBtl; i = i + 8){
    //     CAN.beginExtendedPacket(id);
    //     CAN.write(&buff2[i], 8);
    //     //or
    //     for(int j = 0; j < 8; j++){
    //     //    CAN.write(buff[i+j]);
    //     }
    //     CAN.endPacket();
    //     id++;
    // }
    // free(buff2);

    // id = 0x000013;
    // uint8_t *buff3 =(uint8_t*)Data;
    // for(int i = 0; i < DataSize; i = i + 8){
    //   CAN.beginExtendedPacket(id);
    //   CAN.write(&buff3[i], 8);
    //   //or
    //   for(int j = 0; j < 8; j++){
    //   //    CAN.write(buff[i+j]);
    //   }
    //   CAN.endPacket();
    //   id++;
    // }
    // free(buff3);

    // free(DataRck);
    // free(DataBtl);
    // free(Data);
}


/// @brief 
/// @param packetSize 
void onReceive(int packetSize) {
    
    if (CAN.packetRtr()) {
       
    } else {
        // Serial.println("ID: ###########################################################");
        //   Serial.println(CAN.packetId(), HEX);
        // Serial.print(" and length ");
        // Serial.println(packetSize);

        // only print packet data for non-RTR packets
        if(CAN.packetId() == 0xb00011){ //HX RCK
            // Serial.println(CAN.packetId(), HEX);
            int i = 0;
            char Data1[100] = {'\0'};
            //  Serial.println("\n\n\n\n\n CAAAAAAAAAAAAAN\n\n\n");
            //       Serial.print((char)CAN.read());
            //     Serial.println(CAN.packetId(), HEX);
           
            // Serial.println("RCK   :");

            while (CAN.available() && CAN.packetId() == 0xb00011) { 
                
                //   Serial.print((char)CAN.read());
                // Serial.println(CAN.packetId(), HEX);
                
                char temp_read = (char)CAN.read();
              
                Data1[i] = temp_read;
                xQueueSend(stm.canRxQueueHxRck, (void*)&Data1[i], 0);
                i++;
                // Serial.print("i  = ");
                // Serial.println(i); 
            }
        }
        else if(CAN.packetId() == 0xb00012){ //HX Btl
              
            int i = 0;
            char Data2[100] = {'\0'};
           
            // Serial.println("RCK   :");

            while (CAN.available() && CAN.packetId() == 0xb00012) { 
                // Serial.println("\n\n\n\n\n CAAAAAAAAAAAAAN\n\n\n");
                //   Serial.print((char)CAN.read());
                // Serial.println(CAN.packetId(), HEX);
                
                char temp_read = (char)CAN.read();
              
                Data2[i] = temp_read;
                xQueueSend(stm.canRxQueueHxBtl, (void*)&Data2[i], 0);
                i++;
                // Serial.print("i  = ");
                // Serial.println(i); 
            }
        }
        else if(CAN.packetId() == 0xb00013){ //PWR

            int i = 0;
            char* Data = new char[100];
            while (CAN.available()) {    
                //   Serial.print((char)CAN.read());
                if(CAN.packetId() == 0xb00013)
                Data[i] = (char)CAN.read();
            }
        }
    }

    // Serial.println();
}

