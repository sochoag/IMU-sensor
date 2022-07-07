#ifndef WIFI_HELPER
#define WIFI_HELPER

#include "WiFi.h"
#include "secrets.h"

#define WIFI_TIMEOUT_MS 20000

void connectToWifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(1000);
    }
    Serial.println(WiFi.localIP());
}
#endif