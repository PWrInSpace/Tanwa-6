#include "../include/config/pinout.h"
#include <Wire.h>

float voltageMeasure(uint8_t batteryPin)
{
    // reverseDividerVal = (R1 + R2) / R2 = 58/11
    // return (analogReadMilliVolts(batteryPin) / 4095) * 3300 * (58/11);//TODO sprawdizc rezystor
    return (analogReadMilliVolts(batteryPin)*0.00428571429);
    
}


void beepBoop(int time, int howManyTimes, int devicePin){
    uint32_t delay = time / howManyTimes;
    for(int i=0; i < howManyTimes; ++i){
        digitalWrite(devicePin, 1);
        vTaskDelay(delay / portTICK_PERIOD_MS);
        digitalWrite(devicePin, 0);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}


void pinInit()
{
    //igniter
    pinMode(IGN_TEST_CON_1, INPUT); 
    pinMode(IGN_TEST_CON_2, INPUT);
    pinMode(ARM_PIN, OUTPUT);
    pinMode(FIRE1, OUTPUT);
    pinMode(FIRE2, OUTPUT);
    digitalWrite(ARM_PIN, LOW);
    digitalWrite(FIRE1, LOW);
    digitalWrite(FIRE2, LOW);

    //VOLTAGE
    pinMode(VOLTAGE_MEASURE, INPUT);

    //LORA


    //BUZZER & SPEAKER
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);


    //RESET STM
    pinMode(RST, OUTPUT);
    digitalWrite(RST, HIGH);


    //I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    //calibrate btl
    pinMode(HX1_SCL, INPUT);

}