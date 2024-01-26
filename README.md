# RD DPS5020 TCP Simulator
 ESP8266 (ESP-12) emulates the operation of the RD DPS5020 power supply over WiFi.
 This project is just an example of Modbus TCP server.
 The simpliest (all in one file) Arduino / VSCode example.
 ESP8266 works as Modbus TCP server over WiFi

## What is RD DPS5020
<details>
<summary>Show RD DPS5020</summary>
RD DPS5020 is an inexpensive Chinese power supply sold on Aliexpress.
RD official store page: https://aliexpress.ru/item/32821185351.html
<image src="/Pictures/dps5020.jpg" alt="DPS5020">
</details>

## How it works
1. An external TCP client (DPSmaster - Windows PC application) sends a Modbus request to the ESP8266's local IP address over WiFi.
2. The ESP8266 answers the TCP client.

## How to use
+ Visual Studio Code project in the **VSCode project** folder.
+ Arduino IDE project in the **Adruino project** folder.

Enter your Wi-Fi credentials.
```
WiFi.begin("ssidname","password");
```

You can use any board with ESP8266. I use this one:
<details>
<summary>Show the  board</summary>
Aliexpress page: https://aliexpress.ru/item/4000550036826.html
<image src="/Pictures/NodeMCU.jpg" alt="NodeMCU">
</details>

In the DPSmaster application select "TCP" and enter correct ESP8266's local IP address.

Download page: https://profimaxblog.ru/dpsmaster/

![DPSmaster](/Pictures/DPSmaster.jpg)

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
