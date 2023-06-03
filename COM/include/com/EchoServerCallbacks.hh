#include <BLEServer.h>
#include <BLEUtils.h>
#include <../include/config/pinout.h>

static bool deviceConnected;

class EchoServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        // stop advt?
        Serial.println("Connected!");
        // digitalWrite(LED_BUILTIN, HIGH);
    };

    void onDisconnect(BLEServer *pServer)
    {
        // restart advt?
        deviceConnected = false;
        Serial.println("Disconnected!");
        pServer->getAdvertising()->start();
        // digitalWrite(LED_BUILTIN, LOW);
    }
};