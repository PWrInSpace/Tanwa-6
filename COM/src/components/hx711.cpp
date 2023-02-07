#include "../include/components/hx711_api.h"
#include <Arduino.h>

void HX711_api::calibration(int known_mass){


    set_scale();    
    Serial.println("Tare... remove any weights from the scale.");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    tare();
	
    Serial.println("Tare done...");
    Serial.println("Place a known weight on the scale...");
    vTaskDelay(45000 / portTICK_PERIOD_MS);

    float cal_factor = get_units(10)/known_mass;
    Serial.print("CAAAL FAAACTOR: "); Serial.println(cal_factor,3);
    Serial.println("REMOVE OBJECT FROM THE SCLAE!");
    vTaskDelay(45000 / portTICK_PERIOD_MS);
    set_scale(cal_factor);

}
