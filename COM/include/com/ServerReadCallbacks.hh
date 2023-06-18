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
            printf("######## MOTOOOR FILL OPEN ###########\n");
        }
          else if(rxValue =="FILL_CLOSE"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_FILL, CLOSE_VALVE);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR FILL CLOSE ###########\n");
        }
         else if(rxValue =="DEPR_OPEN"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_DEPR, OPEN_VALVE);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR DEPR OPEN ###########\n");
        }
          else if(rxValue =="DEPR_CLOSE"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_DEPR, CLOSE_VALVE);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR DEPR CLOSE ###########\n");
        } 
        else if(rxValue =="QD_OPEN"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, OPEN_VALVE);
            perror("QD_OPEN1");
            xSemaphoreGive(stm.i2cMutex);

            vTaskDelay(500 / portTICK_PERIOD_MS);

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, 7);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR QD OPEN ###########\n");
        }
          else if(rxValue =="QD_CLOSE"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, CLOSE_VALVE);
            perror("QD_CLOSE1");
            xSemaphoreGive(stm.i2cMutex);

            vTaskDelay(500 / portTICK_PERIOD_MS);

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, 7);
            perror("QD_CLOSE2");
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR QD CLOSE ###########\n");
        }
          else if(rxValue =="QD_STOP"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT, 7);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR QD STOP ###########\n");
        }
        else if(rxValue == "FIRE"){

            Serial.println("################ ARMING ######################");
            digitalWrite(ARM_PIN, HIGH);
            delay(1000);
            Serial.println("############### FIRE #################");
            digitalWrite(FIRE1, HIGH);
            digitalWrite(FIRE2, HIGH);
            delay(1000);
            digitalWrite(FIRE1, LOW);
            digitalWrite(FIRE2, LOW);
        }  else if(rxValue =="QD2_OPEN"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT_2, OPEN_VALVE);
            perror("QD2_OPEN1");
            xSemaphoreGive(stm.i2cMutex);

            vTaskDelay(1000 / portTICK_PERIOD_MS);

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT_2, 7);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR QD2 OPEN ###########\n");
        }
          else if(rxValue =="QD2_CLOSE"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT_2, CLOSE_VALVE);
            perror("QD2_CLOSE1");
            xSemaphoreGive(stm.i2cMutex);

            vTaskDelay(1000 / portTICK_PERIOD_MS);

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT_2, 7);
            perror("QD2_CLOSE2");
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR QD2 CLOSE ###########\n");
        }
          else if(rxValue =="QD2_STOP"){

            xSemaphoreTake(stm.i2cMutex, pdTRUE);
            pwrCom.sendCommandMotor(MOTOR_QUICK_DISCONNECT_2, 7);
            xSemaphoreGive(stm.i2cMutex);
            printf("######## MOTOOOR QD2 STOP ###########\n");
        }

       
    }
};