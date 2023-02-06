#ifndef DATA_HH
#define DATA_HH

#include "../config/config.h"
#include "../structs/stateMachine.h"

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

struct PWRData{
  bool tick = {};
  uint8_t lastDoneCommandNum = {};
  uint8_t motorState[4] = {};
  int16_t adcValue[2] = {}; //tanwa voltage adc[4]
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
  bool tankHeating : 1;
  bool abortButton : 1;
  bool armButton : 1;
  bool igniterContinouity_1;
  bool igniterContinouity_2;
  float vbat;
  uint8_t motorState_1;
  uint8_t motorState_2;
  uint8_t motorState_3;
  uint8_t motorState_4;
  float rocketWeight;
  float tankWeight; 
  uint32_t rocketWeightRaw;
  uint32_t tankWeightRaw;


  DataFrame():
    tanWaState(States::INIT),
    tankHeating(0),
    abortButton(0),
    armButton(0),
    igniterContinouity_1(0),
    igniterContinouity_2(0),
    vbat(0),
    motorState_1(0),
    motorState_2(0),
    motorState_3(0),
    motorState_4(0),
    rocketWeight(0),
    tankWeight(0),
    rocketWeightRaw(0),
    tankWeightRaw(0),
    pressureSensor(0),
    solenoid_fill(0),
    solenoid_depr(0)
  {}
};

void createDataFrame(DataFrame dataFrame, char *data);

#endif