#include "../include/tasks/tasks.h"

void canTask(void *arg){

    while(1){
        // Serial.println("CAN task");
        int packetSize = CAN.parsePacket();
        Serial.println(packetSize);
        if (packetSize || CAN.packetId() != -1) {
            // received a packet
            Serial.print("Received ");

            if (CAN.packetExtended()) {
            Serial.print("extended ");
            }

            if (CAN.packetRtr()) {
            // Remote transmission request, packet contains no data
            Serial.print("RTR ");
            }

            Serial.print("packet with id 0x");
            Serial.print(CAN.packetId(), HEX);

            if (CAN.packetRtr()) {
            Serial.print(" and requested length ");
            Serial.println(CAN.packetDlc());
            } else {
            Serial.print(" and length ");
            Serial.println(packetSize);

            // only print packet data for non-RTR packets
            while (CAN.available()) {
                Serial.print((char)CAN.read());
            }
            Serial.println();
            }

            Serial.println();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}