# indi-rolloffrpi
Roll off roof driver for Raspberry Pi 

### Description

An Observatory roll off roof driver to automate the opening and closing of a roll off roof in the INDI environment.
This driver for personal experimenting with a Raspberry Pi. It is not the intention to submit for inclusion as a INDI 3rdparty driver.

RollOff rpi is a roll off roof driver that is specific to a Raspberry Pi. It uses the Raspberry Pi GPIO pins to open and close the roof. The roof driver GPIO pins are directly connected to relays and sensors that activate and monitor a roof controller such as a gate opener of garage door opener. The driver provides an interface sufficient to open, close and determine if the roof is fully opened or fully closed.
[More information is in the documentation file.](doc/rolloffrpi.md)

### Driver
```
Driver:       RollOff rpi
Executable:   indi_rolloffrpi
Minimum INDI: 2.0.1
```

### Installation

The driver will be installed and ready for use after an indi-full install which includes 3rdparty drivers.

### Building

To build The driver from source, refer to the directions in the parent directory under the topic "Building".  https://github.com/indilib/indi-3rdparty. The directions describe the required build environment, include the installation of libindi-dev. In the section "Building individual 3rd Party Drivers" replace references to indi-eqmod with indi-rolloffino. 

### Usage

Available for selection in a KStars/Ekos INDI client. The driver requires successful communication to an already running Arduino to complete the on-line connection stage. The driver can be manually started from the command line by the indiserver:
`$ indiserver indi_rolloffino`.

### Source Files

Refer to the general documentation in the parent directory for 3rdparty drivers https://github.com/indilib/indi-3rdparty [ indi-rolloffino ].

The source files can best be obtained by cloning the 3rdparty repo as described in the general documentation in the parent directory for 3rdparty drivers https://github.com/indilib/indi-3rdparty. Navigate to the indi-rolloffino directory. Documentation can be read and individual Arduino examples obtained by using a web browser accessing https://github.com/indilib/indi-3rdparty. The Green "Code" button provides one way to get repository files. Alternatively navigate to indi-rolloffino folder and right click an individual file to save it in a local directory.

