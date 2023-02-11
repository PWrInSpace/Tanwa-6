#include "../include/structs/commStructs.h"

void createDataFrame(DataFrame df, char *data){
  size_t loraDataSize;

  loraDataSize = snprintf(NULL, 0, "%d;%d;%d;%d;%0.2f;%d;%d;%d;%d;%d;%d;%0.2f;%0.2f;%d;%d",
    df.tanWaState, df.pressureSensor, df.solenoid_fill, df.solenoid_depr, df.vbat, df.igniterContinouity_1,
    df.igniterContinouity_2, df.motorState_1, df.motorState_2,
    df.motorState_3, df.motorState_4, df.rocketWeight, df.tankWeight, df.rocketWeightRaw, 
    df.tankWeightRaw) + 1;
  
  char loraFrame[loraDataSize];

  
  snprintf(loraFrame, loraDataSize, "%d;%d;%d;%d;%0.2f;%d;%d;%d;%d;%d;%d;%0.2f;%0.2f;%d;%d",
    df.tanWaState, df.pressureSensor,df.solenoid_fill, df.solenoid_depr, df.vbat, df.igniterContinouity_1,
    df.igniterContinouity_2, df.motorState_1, df.motorState_2,
    df.motorState_3, df.motorState_4, df.rocketWeight, df.tankWeight, df.rocketWeightRaw, 
    df.tankWeightRaw);//10
  
  strcpy(data, DATA_PREFIX);
  strcat(data, loraFrame);
  strcat(data, "\n");
}