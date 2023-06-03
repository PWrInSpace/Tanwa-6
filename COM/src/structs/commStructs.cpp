#include "../include/structs/commStructs.h"

void createDataFrame(DataFrame df, char *data){
  size_t loraDataSize;

  loraDataSize = snprintf(NULL, 0, "%d;%d;%d;%d;%d;%d;%d;%s;%s;%0.2f;%d;%d;%d;%d;%0.2f;%0.2f;%0.2f;%0.2f;%d;%d;%d;%d;%d",
    df.tanWaState, df.pressureSensor, df.solenoid_fill, df.solenoid_depr, df.abortButton, df.igniterContinouity_1,
    df.igniterContinouity_2, df.hxRequest_RCK.c_str(), df.hxRequest_TANK.c_str(), df.vbat, df.motorState_1, df.motorState_2,
    df.motorState_3, df.motorState_4, df.tankWeight_temp, df.rocketWeight_temp, df.rocketWeight_val, df.tankWeight_val, df.rocketWeightRaw_val, 
    df.tankWeightRaw_val, df.interface_rck, df.interface_tank, df.interface_mcu) + 1;
  
  char loraFrame[loraDataSize];

  
  snprintf(loraFrame, loraDataSize,  "%d;%d;%d;%d;%d;%d;%d;%s;%s;%0.2f;%d;%d;%d;%d;%0.2f;%0.2f;%0.2f;%0.2f;%d;%d;%d;%d;%d",
  df.tanWaState, df.pressureSensor, df.solenoid_fill, df.solenoid_depr, df.abortButton, df.igniterContinouity_1,
    df.igniterContinouity_2, df.hxRequest_RCK.c_str(), df.hxRequest_TANK.c_str(), df.vbat, df.motorState_1, df.motorState_2,
    df.motorState_3, df.motorState_4, df.tankWeight_temp, df.rocketWeight_temp, df.rocketWeight_val, df.tankWeight_val, df.rocketWeightRaw_val, 
    df.tankWeightRaw_val, df.interface_rck, df.interface_tank, df.interface_mcu);//10
  
  strcpy(data, DATA_PREFIX);
  strcat(data, loraFrame);
  strcat(data, "\n");
}