#ifndef TASKS_HH
#define TASKS_HH

#include "../structs/SoftToolsManagment.h"
#include "../config/pinout.h"
#include "../config/config.h"
#include "../components/hx711_api.h"


extern HX711_api rckWeight;
extern HX711_api tankWeight;

//app_cpu
void dataTask(void *arg);


#endif