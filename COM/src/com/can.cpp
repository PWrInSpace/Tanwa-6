#include "../include/com/can.h"

void onReceive(int packetSize);
void canSend();

bool canInit(){
    

    CAN.setPins(CAN_RX, CAN_TX);
    if (!CAN.begin(500E3)) {
      Serial.println("Starting CAN failed!");
      while (1);
    }
      
    CAN.onReceive(onReceive);
    // while(1){
    //     // Serial.println("CAN task");
    //     int packetSize = CAN.parsePacket();
    //     Serial.println(packetSize);
    //     if (packetSize || CAN.packetId() != -1) {
    //         // received a packet
    //         Serial.print("Received ");

    //         if (CAN.packetExtended()) {
    //         Serial.print("extended ");
    //         }

    //         if (CAN.packetRtr()) {
    //         // Remote transmission request, packet contains no data
    //         Serial.print("RTR ");
    //         }

    //         Serial.print("packet with id 0x");
    //         Serial.print(CAN.packetId(), HEX);

    //         if (CAN.packetRtr()) {
    //         Serial.print(" and requested length ");
    //         Serial.println(CAN.packetDlc());
    //         } else {
    //         Serial.print(" and length ");
    //         Serial.println(packetSize);

    //         // only print packet data for non-RTR packets
    //         while (CAN.available()) {
    //             Serial.print((char)CAN.read());
    //         }
    //         Serial.println();
    //         }

    //         Serial.println();
    //     }
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    return true;
}

/**************************************************************************/
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
    while (CAN.available()) {
      Serial.print((char)CAN.read());
    }
    Serial.println();
  }

  Serial.println();
}
/**************************************************************************/
//can parse data to be sent

void canSend(){

    // while(1){
        Serial.println("Sending packet ... ");

        CAN.beginPacket(0x12);
        // Serial.println("Sending packet ... ");
       
        CAN.write('h');
        CAN.write('e');
        CAN.write('l');
        CAN.write('l');
        CAN.write('o');
        CAN.endPacket();

        Serial.println("done");

       vTaskDelay(1000 / portTICK_PERIOD_MS);

        // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
        Serial.print("Sending extended packet ... ");

        CAN.beginExtendedPacket(0xabcdef);
        uint8_t *buff =(uint8_t*)( "sample text");
        CAN.write(buff, 8);
        // CAN.write('w');
        // CAN.write('o');
        // CAN.write('r');
        // CAN.write('l');
        // CAN.write('d');
        CAN.endPacket();

        Serial.println("done");

       vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}