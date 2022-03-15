# GPS LoRaWAN node
This is a device for transmitting gps coordinates using LoRaWAN each time the user presses a button. 

## Overview
<p align="left">
  <img src="images/breadboard_circuit.jpeg" width="70%" />
</p>

### Component list
- A NUCLEO-L476RG microcontroller board
- Program code as an STM32CubeIDE project
- A GPS module GY-NEO6MV2
- An RFM95W module for LoRaWAN transmissions. For the breakout board used, refer to this [section of the repository](https://github.com/open-pisciculture/temp-open-fish-farming/tree/main/rfm95w-breakout)
- A 3.3V power supply with 3 AA batteries in series and an MCP1700 voltage regulator 
- A simple 3d printed box to organize the circuit

## Prerequisites

Refer to [this section](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/general) for intructions to remove the ST-Link programmer and to program the Nucleo board with the removed programmer. In addition, the breakout board used for the RFM95W module is available [here](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/rfm95w-breakout). The device as presented here uses this breakout board to easily connect the module with a breadboard.

## Instructions

### Circuit schematics
![circuit schematics](images/schematics.png)

### Pin connections

#### GPS Module NEO-6MV2 connections
| Nucleo pin  | NEO-6MV2 pin |
|-------------|--------------|
| CN10 Pin 6  | RX           |
| CN10 Pin 34 | TX           |

#### RFM95W connections
| NUCLEO pin | RFM95W breakout pin | RFM95W module pin |
|:----------:|:-------------------:|-------------------|
|     D12    |          2          | MISO              |
|     D11    |          3          | MOSI              |
|     D3     |          4          | SCK               |
|     D10    |          5          | NSS               |
|     A0     |          6          | RESET             |
|     A3     |          14         | DIO0              |
|     D7     |          15         | DIO1              |
|     A1     |          16         | DIO2              |

## Usage

1. Turn on the device and wait until the GPS is sending data. The GPS module has a red led that will blink once it's ready.
2. Wait until the device has finished the join process. The board led will start blinking once transmission has been completed.
3. Press the user button (blue button on the board) to transmit the current coordinates. The led will stop blinking and the user button will be disabled while transmission is in process. It will start blinking again once transmission has been completed, at which point the user button can be pressed again to initiate a new transmission.

## Troubleshooting

The device will wait until it receives data from the GPS module before initiating the join procedure. If the GPS is not sending data, you can check for the following:
- First, verify that the red led from the module is blinking. It may take a bit (~30 seconds or maybe more), so make sure to wait an appropiate amount of time.
- Try using the device outside. The GPS module may have trouble receiving the signal indoors.
- The GPS module integrates a voltage regulator. Make sure that the battery voltage is connected to the module and not the 3.3V output from the MCP1700.

## Additional

### Program flowchart
![Program flowchart](images/flowchart.png)
