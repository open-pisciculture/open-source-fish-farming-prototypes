# Open-source fish farming prototypes

This repository presents 3 prototypes for fish farming applications based on LoRaWAN.

## Prototypes

### Data buoy node
![imagen de la boya](imagen.png)

This is a device for remote monitoring of temperature, pH and dissolved oxygen variables in fish farming. The data buoy floats on the fish pond and periodically transmits these three values to an on-site gateway that forwards the data to an AWS database. The software, hardware and mechanical structure files are available under permissive open-source licenses and instructions for building your own can be found inside the [data-buoy-node folder](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/data-buoy-node).

### GPS coordinates node
![imagen del nodo gps?](imagen.png)

This system transmits GPS coordinates each time the user presses a button to an on-site gateway and saves the data in an AWS database alongisde LoRaWAN packet information. This device can be used for LoRaWAN coverage sampling. The software and circuit schematics are available inside the [gps-node folder](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/gps-node).

### RFM95W breakout board
![imagen del breakout?](imagen.png)

A simple breakout board for an RFM95W module for easier prototyping with a breadboard. Hardware files are available inside the [rfm95w-breakout folder](https://github.com/open-pisciculture/open-source-fish-farming-prototypes/tree/main/rfm95w-breakout).

## Article

LINK A PLOSONE

PARA CITAR: 
[CITA]

Bibtex

## Licenses

### Software
The software is licensed under an [MIT License](https://opensource.org/licenses/MIT). A copy of the license has been included in the repository and can be found [here](https://github.com/open-pisciculture/temp-open-fish-farming/blob/main/LICENSE-MIT.txt).

### Hardware
The hardware design files are licensed under a CERN Open Source Hardware license version 2 CERN-OHL-P. Details of the license can be found [here](https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-version-2) and a copy of the license has been included [here](https://github.com/open-pisciculture/temp-open-fish-farming/blob/main/LICENSE-CERN-OHL-P.txt).

### Documentation
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.
