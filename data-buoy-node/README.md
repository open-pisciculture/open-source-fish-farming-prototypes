# Data buoy prototype

This is a prototype for remote monitoring of temperature, pH and dissolved oxygen variables for applications in fish farming.

## Overview

The device is a data buoy prototype. It uses sensors from Atlas-Scientific to measure temperature, pH and dissolved oxygen variables and transmits the data using LoRaWAN. This section includes the program code as an STM32CubeIDE project, the Altium project and Gerber files for the printed circuit board and the mechanical parts for assembly of the buoy. 

## Instructions

Before assembling the data buoy, make sure to follow the initial instructions for setting up a LoRaWAN gateway and end device available [here](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/general). Live data can be monitored through the console in The Things Network if everything is working correctly.

Edit the following in the main.c file:
- Specify the sampling period in the SLEEP_SECONDS constant in seconds
- Follow the [Setting LoRaWAN configuration for each device](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/general) section to add the device-specific information from The Things Network

For instructions to program the board if the ST-Link programmer was removed refer to this [section](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/general).

## Usage
1. Turn on the device. Each sensor circuit will briefly light up and blink blue while the microcontroller reads each value.
2. The microcontroller led will turn on while reading the sensors and transmitting the measurements. Once the packets have been successfully transmitted, the microcontroller will sleep.
3. After the specified sampling period has passed, the microcontroller will wake up, read the sensors, transmit the values and enter sleep mode. This cycle will repeat. 

## Additional
![Software flowchart](images/flowchart.png)
