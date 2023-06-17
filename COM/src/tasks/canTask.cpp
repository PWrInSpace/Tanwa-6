#include "../include/tasks/tasks.h"
#include <string.h>
#include <string>
#include <iostream>

extern RxData_Hx rxDataBtl_CAN;
extern RxData_Hx rxDataRck_CAN;
extern PWRData pwrData_CAN;
extern TxData_Hx txDataRck_CAN;
extern TxData_Hx txDataBtl_CAN;
extern TxData txData_CAN;

extern bool can_rck_interface;
extern bool esp_now_rck_interface;
extern bool can_btl_interface;
extern bool esp_now_btl_interface;


void canTask(void *arg){

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    canInit();
    vTaskDelay(200 / portTICK_PERIOD_MS);

    char Data_RCK_temp = '\0';
    char Data_RCK[100] = {'\0'};
    char Data_Btl_temp = '\0';
    char Data_Btl[100] = {'\0'};
    int start_frame_flag_rck = 0;
    int end_frame_flag_rck = 0;
    int start_frame_flag_btl = 0;
    int end_frame_flag_btl = 0;
    int i = 0;
    int j = 0;
    char *ptr_rck = NULL;
    char *ptr_btl = NULL;
    char *strings_rck[4];
    char *strings_btl[4];
    int index_rck = 0;
    int index_btl = 0;

    while(1){
      
        if(xQueueReceive(stm.canRxQueueHxRck, (void*)&Data_RCK_temp, 0) == pdTRUE && can_rck_interface == true){
            if(Data_RCK_temp == '!'){
                // Serial.println();
                // Serial.println("START FLAG");
                // Serial.print("TASK DATA = ");
                start_frame_flag_rck = 1;
                end_frame_flag_rck = 0;
                i = 0;
            }
        }

        while(start_frame_flag_rck == 1 && end_frame_flag_rck == 0){

            if(xQueueReceive(stm.canRxQueueHxRck, (void*)&Data_RCK_temp, 0) == pdTRUE && can_rck_interface == true){
                
                if(Data_RCK_temp == '?'){
                    Data_RCK[i] = '\0';
                    // Serial.println("\nEND FLAG");
                    Serial.println(Data_RCK);
                    end_frame_flag_rck = 1;
                    start_frame_flag_rck = 0;

                    String str(Data_RCK);

                
                    ptr_rck = strtok(Data_RCK, ";");

                    while (ptr_rck != NULL && index_rck < 4)
                    {
                        strings_rck[index_rck] = ptr_rck;
                        index_rck++;
                        ptr_rck = strtok(NULL, ";");
                    }

                    rxDataRck_CAN.request = strings_rck[0];
                    // Serial.print("ROCKET REQUEST: "); Serial.println(rxDataRck_CAN.request);

                    rxDataRck_CAN.weight = atof(strings_rck[1]);
                    // Serial.print("ROCKET WEIGHT: "); Serial.println(rxDataRck_CAN.weight);

                    rxDataRck_CAN.weight_raw = atoi(strings_rck[2]);
                    // Serial.print("ROCKET WEIGHT RAW: "); Serial.println(rxDataRck_CAN.weight_raw);
            
                    rxDataRck_CAN.temperature = atof(strings_rck[3]);
                    // Serial.print("ROCKET TEMPERATURE: "); Serial.println(rxDataRck_CAN.temperature);

                    strcpy(Data_RCK, "");
                    ptr_rck = NULL;
                    index_rck = 0;
                    break;
                }

                Data_RCK[i] = Data_RCK_temp;
                i++;
            }
        }

//#################################################### BTL ####################################################
        if(xQueueReceive(stm.canRxQueueHxBtl, (void*)&Data_Btl_temp, 0) == pdTRUE && can_btl_interface == true ){
            if(Data_Btl_temp == '!'){
                // Serial.println();
                // Serial.println("START FLAG");
                // Serial.print("TASK DATA = ");
                start_frame_flag_btl = 1;
                end_frame_flag_btl = 0;
                j = 0;
            }
        }

        while(start_frame_flag_btl == 1 && end_frame_flag_btl == 0){

            if(xQueueReceive(stm.canRxQueueHxBtl, (void*)&Data_Btl_temp, 0) == pdTRUE && can_btl_interface == true){
                
                if(Data_Btl_temp == '?'){
                    Data_Btl[j] = '\0';
                    // Serial.println("\nEND FLAG");
                    Serial.println(Data_Btl);
                    end_frame_flag_btl = 1;
                    start_frame_flag_btl = 0;

                    String str(Data_Btl);

                
                    ptr_btl = strtok(Data_Btl, ";");

                    while (ptr_btl != NULL && index_btl < 4)
                    {
                        strings_btl[index_btl] = ptr_btl;
                        index_btl++;
                        ptr_btl = strtok(NULL, ";");
                    }

                    rxDataBtl_CAN.request = strings_btl[0];
                    // Serial.print("Tank REQUEST: "); Serial.println(rxDataBtl_CAN.request);

                    rxDataBtl_CAN.weight = atof(strings_btl[1]);
                    // Serial.print("Tank  WEIGHT: "); Serial.println(rxDataBtl_CAN.weight);

                    rxDataBtl_CAN.weight_raw = atoi(strings_btl[2]);
                    // Serial.print("Tank WEIGHT RAW: "); Serial.println(rxDataBtl_CAN.weight_raw);
            
                    rxDataBtl_CAN.temperature = atof(strings_btl[3]);
                    // Serial.print("Tank  TEMPERATURE: "); Serial.println(rxDataBtl_CAN.temperature);

                    strcpy(Data_Btl, "");
                    ptr_btl = NULL;
                    index_btl = 0;
                    break;
                }

                Data_Btl[j] = Data_Btl_temp;
                j++;
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);

    }
}