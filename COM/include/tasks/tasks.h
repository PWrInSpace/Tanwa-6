#ifndef TASKS_HH
#define TASKS_HH

#include "../structs/SoftToolsManagment.h"
#include "../config/pinout.h"
#include "LoRa.h"
#include "../config/config.h"
#include "../components/SDcard.h"
//#include <HX711_ADC.h>
#include "HX711.h"
#include "../components/hx711.h"
#include "../structs/stateMachine.h"
#include "../com/internalCommunication.h"
#include "../structs/commStructs.h"
#include "../com/now.h"
#include <MCP23017.h>

extern SoftwareToolsManagment stm;
extern InternalI2C<PWRData, TxData> pwrCom;
extern MCP23017 expander;
extern HX711 rckWeight;
extern HX711 tankWeight;
extern HX711 ADC2_China;


//pro_cpu
void loraTask(void *arg);
void rxHandlingTask(void *arg);


//app_cpu
void sdTask(void *arg);
void dataTask(void *arg);
void stateTask(void *arg);
void buzzerTask(void *arg);


#endif