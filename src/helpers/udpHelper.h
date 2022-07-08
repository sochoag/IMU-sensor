#ifndef UDP_HELPER
#define UDP_HELPER

#include "AsyncUDP.h"
#include "debuglevels/debug.h"

#define SERVER "192.168.1.121"
#define PORT 3002

AsyncUDP udpClient; //Async UDPClient object

void UDPConnect()
{
    IPAddress ipAddress = IPAddress();
    ipAddress.fromString(SERVER);
    udpClient.connect(ipAddress, PORT);
    printInfoLn("UPD Server Connected");
    printMessage("UDP Server:");
    printMessageLn(String(SERVER)+":"+String(PORT));
}

void UDPSendData(String message)
{
    char charBuffer[100];
    message.toCharArray(charBuffer,100);

    if(udpClient.connected())
    {
        printDebugLn("Data sent successfully!");
        udpClient.broadcastTo(charBuffer, PORT);
    }
    else
    {
        printErrorLn("Server not connected");
        UDPConnect();
    }
}

#endif