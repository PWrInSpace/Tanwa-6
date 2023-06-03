#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include "ServerReadCallbacks.hh"
#include "EchoServerCallbacks.hh"

#ifndef BLUETOOTH_LE_READY_LIB
#define BLUETOOTH_LE_READY_LIB

#define UUID_FILL BLEUUID((uint16_t)0x1700)
#define UUID_DEPR BLEUUID((uint16_t)0x1701)
#define UUID_QD BLEUUID((uint16_t)0x1702)
#define UUID_SERVICE_VALVES BLEUUID((uint16_t)0x1703)


#define LED_BUILTIN 2

extern std::string message;


static BLECharacteristic pCharacteristicFILL(
    UUID_FILL,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR|
        BLECharacteristic::PROPERTY_NOTIFY);
    
static BLECharacteristic pCharacteristicDEPR(
    UUID_DEPR,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR|
        BLECharacteristic::PROPERTY_NOTIFY);

static BLECharacteristic pCharacteristicQD(
    UUID_QD,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR|
        BLECharacteristic::PROPERTY_NOTIFY);


static BLECharacteristic *characTable[3] =
{
    &pCharacteristicFILL,
    &pCharacteristicDEPR,
    &pCharacteristicQD

};

class ESP32_blelib
{
public:
    static std::string prevMessege;
    static void init(
        BLECharacteristic *pCharacteristicFILL,
        BLECharacteristic *pCharacteristicDEPR,
        BLECharacteristic *pCharacteristicQD
        );

private:
    ESP32_blelib() {}
};

#endif
