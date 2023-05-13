#include "../include/com/now.h"

bool adressCompare(const uint8_t *addr1, const uint8_t *addr2);

/**********************************************************************************************/

bool nowInit() {

    WiFi.mode(WIFI_STA);
    if (esp_now_init())
        return false;

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    return true;
}

/**********************************************************************************************/

bool nowAddPeer(const uint8_t *address, uint8_t channel) {

    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, address, 6);
    peerInfo.channel = channel;

    if (esp_now_add_peer(&peerInfo))
        return false;
    return true;
}

/**********************************************************************************************/

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

}

/**********************************************************************************************/

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    TxData txData;
    RxData_OBC rxDataOBC;
    RxData_Hx rxDataRck;
    RxData_Hx rxDataBtl;
    if (adressCompare(mac, adressHxRck)) {
      // Serial.println("Data HXHXHXHXHHXHXHH");


      memcpy((void*) &rxDataRck, (uint16_t *)incomingData, sizeof(RxData_Hx));
  

      if(xQueueSend(stm.espNowRxQueueHxRck, (void*)&rxDataRck, 0) == pdFALSE){
        //TODO ERROR HANDLING
        Serial.println("esp now queue error!");
        xQueueReset(stm.espNowRxQueueHxRck);
      }
    }
    else if(adressCompare(mac, adressObc)) {
      if (len == sizeof(uint8_t)) {
        // TO DO: zmiana stanu
          // StateMachine::changeStateRequest(static_cast<States>(*incomingData));
          // Serial.println("STATE CHANGE REQUEST");

        return;       
      }
      Serial.println("Data OBCOBCOBCOBCOBCOBCOBCOC");
        
        
        memcpy((void*) &rxDataOBC, incomingData, sizeof(RxData_OBC));
        if(xQueueSend(stm.espNowRxQueueObc, (void*)&rxDataOBC, 0) == pdFALSE){
        //TODO ERROR HANDLING
        Serial.println("esp now queue error!");
      }
    }
    else if(adressCompare(mac, adressHxBtl)) {
      Serial.println("Data HXHXHXHXHHXHXHH22222222222222222222222");
        memcpy((void*) &rxDataBtl, (uint16_t *)incomingData, sizeof(RxData_Hx));
        if(xQueueSend(stm.espNowRxQueueHxBtl, (void*)&rxDataBtl, 0) == pdFALSE){
        //TODO ERROR HANDLING
        Serial.println("esp now queue error!");
      }

    }

}

/**********************************************************************************************/

bool adressCompare(const uint8_t *addr1, const uint8_t *addr2) {

    for (int8_t i = 0; i < 6; i++) {

        if (addr1[i] != addr2[i])
            return false;
    }

    return true;
}