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

extern RxData rxData_CAN;
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
    size_t DataSize;
    
    
    DataSize = snprintf(NULL, 0, "!%s;%0.2f;%d;%0.2f;?",txData_CAN.request.c_str(),txData_CAN.weight, txData_CAN.weight_raw, txData_CAN.temperature) + 1;
    char* Data = new char[DataSize];
    snprintf(Data, DataSize, "!%s;%0.2f;%d;%0.2f;?",txData_CAN.request.c_str(),txData_CAN.weight, txData_CAN.weight_raw, txData_CAN.temperature);
    Serial.print("CAN SEND: ");
    Serial.println(Data);
    

    // int id = 0xb00011;//HX RCK
    int id = 0xb00012;//HX BTL

    uint8_t buff [8] = {0};
    // Serial.print("BUFF: "); Serial.println(String(*buff));
    for(int i = 0; i < DataSize; i = i + 8){
        CAN.beginExtendedPacket(id);
        // CAN.write(&buff[i], 8);
      
        //or
        // Serial.print("BUFF: ");
        for(int j = 0; j < 8; j++){

            buff[j] = Data[i+j];
            
            // Serial.print((String)buff[j]);
        }
        // Serial.print("\n");
        CAN.write(buff, 8);
        //  sscanf(Data, "%s;%0.2f;%d;%0.2f;",&txData_CAN.request,&txData_CAN.weight, &txData_CAN.weight_raw, &txData_CAN.temperature);
        // Serial.println("SCANF"); Serial.println(txData_CAN.request.c_str());
        // Serial.println(txData_CAN.weight);
        // Serial.println(txData_CAN.weight_raw);
        // Serial.println(txData_CAN.temperature);
        // Serial.println();
        CAN.endPacket();
        // id++;
    }
    free(Data);
}


void onReceive(int packetSize) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetRtr()) {
        // Serial.print(" and requested length ");
        // Serial.println(CAN.packetDlc());
    } else {
        // Serial.print(" and length ");
        // Serial.println(packetSize);
        // if(CAN.packetId() == 0x000011){//HX RCK
        if(CAN.packetId() == 0x000012){//HX BTL
            
        
            // only print packet data for non-RTR packets
            int i = 0;
            char* Data = new char[100];
            while (CAN.available()) {    
                Serial.print((char)CAN.read());
                Data[i] = (char)CAN.read();
            }
            // test data: (Data, 100, "%0.2f;%d;%0.2f;%0.2f;%d;%0.2f;",weight, weight_raw, temperature, weight+1, weight_raw+1, temperature+1);

            sscanf( Data, "%s;%f;%d;", &rxData_CAN.request, &rxData_CAN.offset, &rxData_CAN.command);
            //std::cout<< weight_r1 << ';' << weight_raw_r1 << ';' << temperature_r1 << ';' << weight_r2 << ';' << weight_raw_r2 << ';' << temperature_r2;

            Serial.println();
        }
    }

    Serial.println();
}

