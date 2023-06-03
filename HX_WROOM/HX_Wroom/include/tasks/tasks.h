#ifndef TASKS_HH
#define TASKS_HH

#include "../structs/SoftToolsManagment.h"

#include "../config/pinout.h"
#include "../config/config.h"
#include "../components/hx711_api.h"
#include "../com/now.h"
#include "../com/can.h"


extern HX711_api HxWeight;
//app_cpu
void dataTask(void *arg);
void rxHandlingTask(void *arg);
#endif