# Assessment of LoRa and IEEE802.15.4 suitability for asset tracking across an urban environment.
This investigation was completed and submitted on 11 November 2020 at the [University of Cape Town](www.uct.ac.za).

## General Details

This repository holds all the code for the investigations conducted on the effectiveness of IoT devices in localisation. The intended use case was for an urban environment or for a multisite factory that has multiple bases. This system may be implemented to keep track of stock and track deliveries across multiple locations that are within range.

## Using this repository
The code was written using the [STMCubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html). Using this code will be much easier if this IDE is used.
There are 3 main folders:
### Lorabee
This contains code for a transceiver that will use both the RFM95W and the MRF24J40 radios with an STMF0xx microcontroller. The pinout is obtained in the Lorabee.ioc file found in the folder.  It simply will transmit a message containing only a DEVICE_ID every second or check its radios for an incoming message every second. It will also display the data collected through a serial port.
  
### Lorabee Master
This contains code for a receiver that will use both the RFM95W and the MRF24J40 radios with an STMF0xx microcontroller. The pinout is obtained in the Lorabee Master.ioc file found in the folder.  It has multiple modes that are used to obtain different metrics from the radios. These modes are: 
**Joint**: obtains a single packet and returns the RSSI.
**Tower**: obtains packets sent by towers with `DEVICE_ID = A, B or C` and displays them.
**Count**: counts the number of packets received by both radios.
**Live**: Displays packet RSSI's as they are received.

### Zigbee
This contains code for a transceiver that will use the MRF24J40 radio with an STMF0xx microcontroller. The pinout is obtained in the Zigbee.ioc file found in the folder.  It simply will transmit a message containing only a DEVICE_ID every second or check its radio for an incoming message every 0.2seconds. It will also display the data collected through a serial port.

## Abstract
This report will investigate the effectiveness of IEEE802.15.4 and LoRa localisation in an urban environment. This was done by dynamic mapping using Received Signal Strength Indicator(RSSI) values obtained by sending packets in a four-node personal area network. The purpose behind this investigation is to see if the Internet of Things (IoT) networks used would be able to produce results that are as accurate as those given by GPS receivers in an urban environment such as a city or town both indoors and outdoors. Other metrics that will be evaluated will be the cost of the infrastructure and the power consumption compared to the GPS network. The proposed system in this report may easily be implemented in existing networks to allow the location of the nodes in that network as well.
## Acknowledgements
The library used for the RFM95W radio was written by Konstantin Belyalov. It can be found here: [STM32 HAL Libraries](https://github.com/belyalov/stm32-hal-libraries)
The library used for the MRF24J40 was written by Karl Palsson. It can be found here:  [MRF24J40 driver for STM32](https://false.ekta.is/2012/05/mrf24j40-driver-for-stm32/)