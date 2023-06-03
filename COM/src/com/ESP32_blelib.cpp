#include "../include/com/ESP32_blelib.hh"

void ESP32_blelib::init(
    BLECharacteristic *pCharacteristicFILL,
    BLECharacteristic *pCharacteristicDEPR,
    BLECharacteristic *pCharacteristicQD
  )
{
    Serial.println("inside blelib init");
    deviceConnected = false;

    BLEDevice::init("ESP32BLE");
    BLEServer *pServer = BLEDevice::createServer();
    // BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x1700));
    // BLEService *pService2 = pServer->createService(BLEUUID((uint16_t)0x1701));

    BLEService *pServiceVALVES = pServer->createService(UUID_SERVICE_VALVES);

    pServer->setCallbacks(new EchoServerCallbacks());


    pServiceVALVES->addCharacteristic(pCharacteristicFILL);
    pServiceVALVES->addCharacteristic(pCharacteristicDEPR);
    pServiceVALVES->addCharacteristic(pCharacteristicQD);



    pCharacteristicFILL->addDescriptor(new BLE2902());
    pCharacteristicFILL->setCallbacks(new ServerReadCallbacks());
    pCharacteristicDEPR->addDescriptor(new BLE2902());
    pCharacteristicQD->addDescriptor(new BLE2902());
 

    pServer->getAdvertising()->addServiceUUID(pServiceVALVES->getUUID());

    // pService->start();
    // pService2->start();
    pServiceVALVES->start();

    pServer->getAdvertising()->start();

    Serial.println("Waiting for a Client to connect...");
}
