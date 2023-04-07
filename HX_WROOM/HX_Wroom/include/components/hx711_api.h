#ifndef HX711_HH
#define HX7111_HH

#include "HX711.h"

class HX711_api: public HX711{

    public:

    float calibration(int known_mass);
};

#endif
