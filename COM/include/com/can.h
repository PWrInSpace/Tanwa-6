#include <CAN.h>
#include "../config/config.h"
#include "../config/pinout.h"


bool canInit();
void onReceive(int packetSize);
void canSend();
