#ifndef WIFI_HELPER
#define WIFI_HELPER

#include "WiFi.h"
#include "secrets.h"
#include "debuglevels/debug.h"

#define WIFI_TIMEOUT_MS 20000

void connectToWifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    printInfo("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        printInfo('.');
        delay(1000);
    }
    printInfoLn('.');
    printMessage("Wifi IP address:");
    printMessageLn(WiFi.localIP().toString());
}
#endif