# GPS LoRaWAN node

## Usage

1. Turn on the device and wait until the GPS is sending data. The module has a red led that will blink once it's ready.
2. Wait until the device has finished the join process. The board led will start blinking once transmission has been completed.
3. Press the user button (blue button on the board) to transmit the current coordinates. The led will stop blinking while transmission and the user button will be disabled while transmission is in process. It will start blinking again once transmission has been completed, at which point the user button can be pressed again to initiate a new transmission.

## Troubleshooting

The device will wait until it receives data from the GPS module before initiating the join procedure. If the GPS is not sending data, you can check for the following:
- First, verify that the red led from the module is blinking. It may take a bit (~30 seconds or maybe more), so make sure to wait an appropiate amount of time.
- Try using the device outside. The GPS module may have trouble recieving the signal indoors.
