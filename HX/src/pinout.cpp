#include "../include/config/pinout.h"
#include <Wire.h>
//TODO change pins according to pico!!

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


    //I2C
    Wire.begin(I2C_SDA, I2C_SCL);

}