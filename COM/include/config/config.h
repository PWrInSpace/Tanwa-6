#ifndef TANWA_CONFIG_H
#define TANWA_CONFIG_H

#include <Arduino.h>

//#define __DEBUG

#ifdef __DEBUG
  #define DEBUGL(x) Serial.println(x)
  #define DEBUG(x) Serial.print(x)
#else
  #define DEBUGL(x) {}
  #define DEBUG(x) {}
#endif

//RTOS QUEUE
#define SD_FRAME_SIZE 256
#define SD_QUEUE_LENGTH 10
#define LORA_TX_FRAME_SIZE 256
#define LORA_TX_QUEUE_LENGTH 10
#define ESP_NOW_QUEUE_LENGTH 100
#define LORA_RX_QUEUE_LENGTH 10
#define LORA_RX_FRAME_SIZE 512
#define DATA_PREFIX "R4T;"

//SD FILE PATH
static String dataFileName= "/data_";
static String dataFileName_lastWeightRck= "/lastWeightRck_";
static String dataFileName_lastWeightBtl= "/lastWeightBtl_";
//I2C COM COMMAND
#define OPEN_VALVE 0x01
#define CLOSE_VALVE 0x00
#define STOP_VALVE 0x22
#define TIMED_OPEN_VALVE 0x03
#define MOTOR_FILL 0x01
#define MOTOR_DEPR 0x02
#define MOTOR_QUICK_DISCONNECT 0x03 //remember about that there are 2 of them


#define LORA_FREQ_MHZ 915    //927

//WEIGHTING
#define BIT_TO_GRAM_RATIO_RCK 53.13
#define BIT_TO_GRAM_RATIO_TANK 37.78// USA 32.73
#define OFFSET_TANK 1
#define STABILIZNG_TIME 4000

#define COM_ADRESS 0x1A //26 dec 0x1A

//ESPNOW COMMANDS OBC TANWA

#define STATE_ESP 0x00
#define ABORT_ESP 0x01
#define HOLD_IN_ESP 0x02
#define HOLD_OUT_ESP 0x03

#define FILL_ESP 0x10
#define FILL_TIME_ESP 0x15
#define DEPR_ESP 0x20
#define QD_ESP 0x30
#define VALVE_OPEN_ESP 0x01
#define VALVE_CLOSE_ESP 0x00

#define SOFT_ARM_ESP 0x04
#define SOFT_DISARM_ESP 0x05

#define FIRE_ESP 0x88

#define SOFT_RESTART_ESP_RCK 0x61
#define SOFT_RESTART_ESP_BTL 0x62
#define SOFT_RESTART_ESP_ESP 0x06
#define SOFT_RESTART_STM_ESP 0x07

#define CALIBRATE_RCK_ESP 0x40
#define TARE_RCK_ESP 0x41
#define SET_CAL_FACTOR_RCK_ESP 0x42
#define SET_OFFSET_RCK_ESP 0x43

#define CALIBRATE_TANK_ESP 0x50
#define TARE_TANK_ESP 0x51
#define SET_CAL_FACTOR_TANK_ESP 0x52
#define SET_OFFSET_TANK_ESP 0x53

#define PLSS_ESP 0x60

#define INTERFACE_RCK_ESP 0x44
#define INTERFACE_TANK_ESP 0x54
#define INTERFACE_ESPNOW_ESP 0x11
#define INTERFACE_CAN_ESP 0x12

#define INTERFACE_MCU_ESP 0x70
#define INTERFACE_I2C_ESP 0x11

#define LORA_FREQ_ESP 0x80
#define LORA_TIME_ESP 0x81

//ESPNOW COMMNANDS FOR HX
#define NOTHING 255
#define TARE_HX 70
#define CALIBRATE_HX 71
#define SET_CAL_FACTOR_HX 72
#define SET_OFFSET_HX 73
#define SET_HX_ESP_INTERFACE 80
#define SET_HX_CAN_INTERFACE 90
#define SOFT_RESTART_HX 99

//Expander 
#define MCP_ADDRESS 0x20 

//EEPROM
#define EEPROM_SIZE 50

#endif