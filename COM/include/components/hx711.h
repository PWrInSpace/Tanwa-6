#ifndef HX711_HH
#define HX711_HH

#include "HX711.h"

class Hx711:public HX711{

    public:
    Hx711(uint8_t dout, uint8_t sck):HX711(dout, sck){};
    float calibration(int known_mass);
};

#endif
