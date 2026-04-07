# CAN-Receiver-Transmitter

**WIP**
**The Program is developed using help from claude code.**

Web based can interface for receiving and transmitting can data

This programs helps in receiving and transmitting can frame. It is based on ESP32, MCP2515. ESP32 opens up a wifi connection and hosts its own webpage locally. Connect to the wife, and open the webpage at http://192.168.4.1, upload the DBC file to see the signals and messages, provides manual or automatiic control for setting the signal values. Each signal and message can be individually disabled/enabled. 



MCP2515-Pin	      ESP32-Pin	    Description
CS	              GPIO 5	      SPI Chip Select
INT	              GPIO 4	      Interrupt (goes LOW when CAN message received)
SCK	              GPIO 18	      SPI Clock (default ESP32 SPI)
MOSI	            GPIO 23	      SPI Master Out Slave In
MISO	            GPIO 19	      SPI Master In Slave Out
VCC	              3.3V or 5V	  Power (check your module specs)
GND	              GND	          Ground


Futurer plans
-> Integrate MF4 replay as well for can injection
-> Fix bugs in receiver
-> Improve the esp32 connectivity issues
-> Look into UI for improvements and more refined signal control
