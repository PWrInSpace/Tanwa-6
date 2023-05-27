#include "../include/tasks/tasks.h"


extern RxData_Hx rxDataBtl_CAN;
extern RxData_Hx rxDataRck_CAN;
extern PWRData pwrData_CAN;
extern TxData_Hx txDataRck_CAN;
extern TxData_Hx txDataBtl_CAN;
extern TxData txData_CAN;


void canTask(void *arg){

    

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    

    canInit();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    char Data_RCK_temp = '\0';
    char Data_RCK[100] = {'\0'};
    int start_frame_flag = 0;
    int end_frame_flag = 0;
    int i = 0;

    while(1){
      
        if(xQueueReceive(stm.canRxQueueHxRck, (void*)&Data_RCK_temp, 0) == pdTRUE){
            if(Data_RCK_temp == '!'){
                Serial.println();
                Serial.println("START FLAG");
                Serial.print("TASK DATA = ");
                start_frame_flag = 1;
                end_frame_flag = 0;
                i = 0;
            }
        }

        while(start_frame_flag == 1 && end_frame_flag == 0){

            if(xQueueReceive(stm.canRxQueueHxRck, (void*)&Data_RCK_temp, 0) == pdTRUE){
                
                if(Data_RCK_temp == '?'){
                    Serial.println("\nEND FLAG");
                    end_frame_flag = 1;
                    start_frame_flag = 0;
                    sscanf( Data_RCK,  "%s;%0.2f;%d;%0.2f", rxDataRck_CAN.request.c_str(), rxDataRck_CAN.weight, rxDataRck_CAN.weight_raw, rxDataRck_CAN.temperature);
                    Serial.print("ROCKET REQUEST: "); Serial.println(rxDataRck_CAN.request);
                    Serial.print("ROCKET WEIGHT: "); Serial.println(rxDataRck_CAN.weight);
                    Serial.print("ROCKET WEIGHT RAW: "); Serial.println(rxDataRck_CAN.weight_raw);
                    Serial.print("ROCKET TEMPERATURE: "); Serial.println(rxDataRck_CAN.temperature);
                    break;
                }

                Data_RCK[i] = Data_RCK_temp;
                Serial.print(Data_RCK);
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);

    }
}