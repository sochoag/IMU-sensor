#ifndef JSON_HELPER
#define JSON_HELPER

#include <ArduinoJson.h>

String json;

String GetJsonString(unsigned int sensorID, float  pitch, float roll, float yaw)
{
    StaticJsonDocument<64> doc;

    doc["sensorID"] = sensorID;
    doc["yaw"] = pitch;
    doc["pitch"] = roll;
    doc["roll"] = yaw;

    char docBuff[100];

    serializeJson(doc, docBuff);

    return docBuff;
}

#endif