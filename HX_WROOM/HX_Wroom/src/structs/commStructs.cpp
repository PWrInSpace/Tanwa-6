#include "../include/structs/commStructs.h"

void createDataFrame(TxData df, char *data){
  size_t loraDataSize;

  loraDataSize = snprintf(NULL, 0, "%0.2f;%d;%0.2f",
    df.weight, df.weight_raw, df.temperature) + 1;
  
  char loraFrame[loraDataSize];

  
  snprintf(loraFrame, loraDataSize,  "%0.2f;%d;%0.2f",
    df.weight, df.weight_raw, df.temperature);//10
  
  // strcpy(data, DATA_PREFIX_BTL);
  strcpy(data, DATA_PREFIX);
  strcat(data, loraFrame);
  strcat(data, "\n");
}