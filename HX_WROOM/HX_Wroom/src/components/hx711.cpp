#include "../include/components/hx711_api.h"
#include <Arduino.h>

float HX711_api::calibration(int known_mass){


    set_scale();    
    Serial.print("CALIBRATE WITH = "); Serial.print(known_mass); Serial.println("g");
    Serial.println("################   Tare... remove any weights from the scale.   #############");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    tare();
	
    Serial.println("################# Tare done... #######################");
    Serial.println(" ############## Place a known weight on the scale... ###############");
    vTaskDelay(4500 / portTICK_PERIOD_MS);

    float cal_factor = get_units(10)/known_mass;

    //get value/scale = known -> scale = value/known
    Serial.print(" ###############CAAAL FAAACTOR: ############## "); Serial.println(cal_factor,3);
    Serial.println(" #####################  REMOVE OBJECT FROM THE SCLAE!  ####################");
    vTaskDelay(4500 / portTICK_PERIOD_MS);
    set_scale(cal_factor);

    return cal_factor;

}
