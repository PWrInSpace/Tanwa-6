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
    // received a packet
    // Serial.print("Received ");

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
            // Serial.println(CAN.packetId(), HEX);
            int i = 0;
            char Data1[100] = {'\0'};
           
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

            // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);

           
            //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;
            
            // Serial.print("DATA RCK = ");Serial.println(Data1);
            // free(Data1);
            

        }
        // else if(CAN.packetId() == 0xb00012){ //HX Btl
              
        //   int i = 0;
        //   char* Data = new char[100];
        //   while (CAN.available()) {    
        //     //   Serial.print((char)CAN.read());
        //     if(CAN.packetId() == 0xb00012)
        //       Data[i] = (char)CAN.read();
        //   }
        //   // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);

        //   sscanf( Data,  "%s;%0.2f;%d;%0.2f", &rxDataBtl_CAN.request, &rxDataBtl_CAN.weight, &rxDataBtl_CAN.weight_raw, &rxDataBtl_CAN.temperature);
        //   //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;
          
        //   Serial.print("DATA BTL = ");Serial.println(Data);
        //   free(Data); 
        // }
        // else if(CAN.packetId() == 0xb00013){ //PWR

        //   int i = 0;
        //   char* Data = new char[100];
        //   while (CAN.available()) {    
        //     //   Serial.print((char)CAN.read());
        //     if(CAN.packetId() == 0xb00013)
        //       Data[i] = (char)CAN.read();
        //   }

        //   // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);
        // Serial.print("DATA PWR = ");Serial.println(Data);
        //   sscanf( Data,  "%d;%d;%d;%d;%d;%d;%d;%d;%d", &pwrData_CAN.tick, &pwrData_CAN.lastDoneCommandNum, &pwrData_CAN.motorState[0], &pwrData_CAN.motorState[1], &pwrData_CAN.motorState[2], &pwrData_CAN.motorState[3], &pwrData_CAN.adcValue[0], &pwrData_CAN.adcValue[1], &pwrData_CAN.adcValue[2]);
        //   //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;
        //   free(Data);
        //   Serial.println();

        // }

    }

    // Serial.println();
    // vTaskDelay(1 / portTICK_PERIOD_MS);
}

