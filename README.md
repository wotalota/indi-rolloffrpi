# indi-rolloffrpi
Roll off roof driver for Raspberry Pi 

### Description

An Observatory roll off roof driver to automate the opening and closing of a roll off roof in the INDI environment.
The driver uses pigpiod to access the Raspberry Pi pins. Because of this it can only be built and used by a Raspberry Pi. So it is not expected to be suitable to be included as an INDI 3rdparty driver.

RollOff rpi is a roll off roof driver that is specific to a Raspberry Pi. It uses the Raspberry Pi GPIO pins to open and close the roof. The roof driver GPIO pins are directly connected to relays and sensors that activate and monitor a roof controller such as a gate opener of garage door opener. The driver provides an interface sufficient to open, close and determine if the roof is fully opened or fully closed.
[More information is in the documentation file.](doc/rolloffrpi.md)

### Driver
```
Driver:       RollOff rpi
Executable:   indi_rolloffrpi
Minimum INDI: 2.0.1
```

### Installation


### Building


### Usage


### Source Files


