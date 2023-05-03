#ifndef DATA_HH
#define DATA_HH

#include "../config/config.h"
#include "../structs/stateMachine.h"

#define WORK "WRK"
#define ASK "ASK"
#define ANSWER "ANS"


enum ValveState {
  ValveStateClose = 0, 
  ValveStateOpen = 1, 
  ValveStateIDK = 2 , 
  ValveStateVent = 3, 
  ValveStateAttemptToOpen = 4, 
  ValveStateAttemptToClose = 5
};


struct TxData{
  uint8_t command;
  uint16_t commandValue;
};

struct TxData_Hx{
  String request;
  float offset;
  uint8_t command;

  TxData_Hx():
    request(""),
    offset(0),
    command(-1)
  {}
};


struct RxData_Hx{
  String request;
  float weight;
  uint32_t weight_raw;
  float temperature;

  RxData_Hx():
    request(""),
    weight(0),
    weight_raw(0),
    temperature(0)
  {}
};

struct PWRData{
  bool tick = {};
  uint8_t lastDoneCommandNum = {};
  uint8_t motorState[4] = {};
  int16_t adcValue[3] = {}; 
};

struct Options{
  uint16_t loraFreqMHz;
  uint32_t bitToGramsRatioTank;
  uint32_t bitToGramsRatioRocket;

  Options():
    loraFreqMHz(LORA_FREQ_MHZ),
    bitToGramsRatioTank(BIT_TO_GRAM_RATIO_TANK),
    bitToGramsRatioRocket(BIT_TO_GRAM_RATIO_RCK)
  {}
};

struct DataFrame{
  uint8_t tanWaState;
  uint16_t pressureSensor;
  uint16_t solenoid_fill; //pin pa0 10 pin stm adc
  uint16_t solenoid_depr; // pin pa2 12 pin stm adc
  bool abortButton : 1;
  bool igniterContinouity_1;
  bool igniterContinouity_2;
  String hxRequest_RCK;
  String hxRequest_TANK;
  float vbat;
  uint8_t motorState_1;
  uint8_t motorState_2;
  uint8_t motorState_3;
  uint8_t motorState_4;
  float rocketWeight_val;
  float tankWeight_val; 
  uint32_t rocketWeightRaw_val;
  uint32_t tankWeightRaw_val;
  bool interface_rck;
  bool interface_tank;
  bool interface_mcu;


  DataFrame():
    tanWaState(States::INIT),
    pressureSensor(0),
    solenoid_fill(0),
    solenoid_depr(0),
    abortButton(0),
    igniterContinouity_1(0),
    igniterContinouity_2(0),
    hxRequest_RCK(""),
    hxRequest_TANK(""),
    vbat(0),
    motorState_1(0),
    motorState_2(0),
    motorState_3(0),
    motorState_4(0),
    rocketWeight_val(0),
    tankWeight_val(0),
    rocketWeightRaw_val(0),
    tankWeightRaw_val(0),
    interface_rck(0),
    interface_tank(0),
    interface_mcu(0)

  {}
};

void createDataFrame(DataFrame dataFrame, char *data);

#endif