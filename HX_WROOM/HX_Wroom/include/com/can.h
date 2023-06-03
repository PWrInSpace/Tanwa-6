#include <CAN.h>
#include "../config/config.h"
#include "../config/pinout.h"


bool canInit();
void canSend();
void onReceive(int packetSize);
