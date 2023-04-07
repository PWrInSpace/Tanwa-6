#include "../include/config/pinout.h"
#include <Wire.h>
//TODO change pins according to pico!!

void pinInit()
{
    //igniter
    pinMode(CALIBRATION_PIN, INPUT); 
    // pinMode(IGN_TEST_CON_2, INPUT);
    // pinMode(ARM_PIN, OUTPUT);
    // pinMode(FIRE1, OUTPUT);
    // pinMode(FIRE2, OUTPUT);
    // digitalWrite(ARM_PIN, LOW);
    // digitalWrite(FIRE1, LOW);
    // digitalWrite(FIRE2, LOW);

}