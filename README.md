# DPS5020 TCP Simulator
 ESP8266 (ESP-12) emulates the operation of the Riden DPS5020 power supply over WiFi
 The simpliest (all in one file) Arduino / VSCode example.
 ESP8266 works as Modbus TCP server over WiFi
    
## How it works
1. An external TCP client (DPSmaster - Windows PC application) sends a Modbus request to the ESP8266's local IP address.
2. The ESP8266 transmits the response to the TCP client via WiFi.

## How to use
+ Visual Studio Code project in the **VSCode project** folder.
+ Arduino IDE project in the **Adruino project** folder.

In the DPSmaster application select "TCP" and enter correct ESP8266's local IP address.

Enter your Wi-Fi credentials.
```
WiFi.begin("ssidname","password");
```
## How to find out the local Wi-Fi IP address of ESP8266.
Open the router's web page in a browser. View the list of clients.

## How to debug
Use the built-in USB-COM serial port at the selected baud rate:
```
Serial.begin(115200);
```
Uncomment the line if you need debug Modbus messages:
```
#define MB_DEBUG
```
