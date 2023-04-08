#ifndef DATA_HH
#define DATA_HH

#include "../config/config.h"


// struct TxData{
//   uint8_t command;
//   uint16_t commandValue;
// };

struct Options{
  uint32_t bitToGramsRatioTank;
  uint32_t bitToGramsRatioRocket;

  Options():
    bitToGramsRatioTank(BIT_TO_GRAM_RATIO_TANK),
    bitToGramsRatioRocket(BIT_TO_GRAM_RATIO_RCK)
  {}
};

struct TxData{
  String request;
  float weight;
  uint32_t weight_raw;
  float temperature;


  TxData():
  request(""),
    weight(0),
    weight_raw(0),
    temperature(0)
  {}
};

void createDataFrame(TxData dataFrame, char *data);

#endif