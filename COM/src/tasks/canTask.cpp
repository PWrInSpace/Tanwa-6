#include "../include/tasks/tasks.h"

void canTask(void *arg){

    while(1){
        Serial.println("Sending packet ... ");

        CAN.beginPacket(0x12);
        Serial.println("Sending packet ... ");
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
        CAN.write('w');
        CAN.write('o');
        CAN.write('r');
        CAN.write('l');
        CAN.write('d');
        CAN.endPacket();

        Serial.println("done");

       vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}