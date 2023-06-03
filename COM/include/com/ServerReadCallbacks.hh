#include <BLEServer.h>
#include <BLEUtils.h>
#include <Arduino.h>
#include "../include/tasks/tasks.h"

extern std::string message;
// extern SoftwareToolsManagment stm;
// extern InternalI2C<PWRData, TxData> pwrCom(&stm.i2c, COM_ADRESS);

static bool messageReceivedComplete;

class ServerReadCallbacks : public BLECharacteristicCallbacks
{
public:
        void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
        {
            Serial.print(rxValue[i]);
        }
        Serial.println();


        if(rxValue == "RST"){
            Serial.println("RST");
            ESP.restart();
        }
        else if(rxValue =="FILL_OPEN"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_FILL, OPEN_VALVE);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR FILL ###########\n");
        }

       
    }
};