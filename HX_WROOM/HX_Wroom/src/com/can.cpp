// #include "../include/tasks/tasks.h"

#include "../include/com/can.h"
// #include <CAN.h>

void onReceive(int packetSize);
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
  Serial.print("CAN Received ");

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
