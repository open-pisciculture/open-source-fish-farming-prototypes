# Data buoy prototype

This is a prototype for remote monitoring of temperature, pH and dissolved oxygen variables for applications in fish farming.

## Overview

The device is

## Instructions

Before assembling the data buoy, make sure to follow the initial instructions for setting up a LoRaWAN gateway and node available [here](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/general) and also configure the services in AWS detailed [here](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/data-buoy-node/cloud).

- Specify the sampling period in the SLEEP_SECONDS constant. 

## Usage
1. Turn on the device. Each sensor circuit will briefly light up and blink blue while the microcontroller reads each value.
2. The microcontroller led will turn on while reading the sensors and transmitting the measurements. Once the packets have been successfully transmitted, the microcontroller will sleep.
3. After the specified sampling period has passed, the microcontroller will wake up, read the sensors, transmit the values and enter sleep mode. This cycle will repeat. 

## Additional
![Software flowchart](images/flowchart.png)
