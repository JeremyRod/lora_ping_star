# README #

DOG proof of concept firmware.  
Basic idea is transmitting ping and alarm data from several nodes to a base station via LoRa.  
A node user triggers the alarm and the base station user receives an indication of an alarm.

### Build Requirements ###

* STM32CubeIDE (designed with v1.13.2)
* STM32CubeWL for subghz radio libraries

### Breakout board ###
* Seeed Studio Wio-E5 LoRa
* Micro is STM32WLE5

### Programming and preparation notes ###
Use STM32cube programmer  

Erasing factory firmware:  
* Hot plug connection
* Hold the reset pin when conencting, then release on connection.
* In OB tab (optional bytes) read the RDP, should be 0xBB, set it to 0xAA, apply and read to confirm.
* Disconnection devkit and programmer from USB.
* Reconnect, and program connect.
* Read RDP confirm 0xAA, set it to 0xBB, it may show an error.
* Disconnection devkit and programmer from USB.
* Reconnect, and program connect.
* Read RDP confirm 0xFF, set it back to 0xAA. Read. 
* If not 0xFF try again.
* Try full chip erase and should complete without error.

Enabling SPI comms:  
* In REG tab, set device to STM32WL5X_cm4, choose FLASH registers.
* Go to SFR register, SUBGHSPISD, if it is 0 set it to 1, apply and read to confirm.
