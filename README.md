# nunchuk_mod

The code for the MCU on the custom PCB for my wireless nunchuk. It uses a NRF24L01+ radio and a stm32f100c8t6 microcontroller.

To upload the code, I'm using stm32flash in the upload rule of the makefile.

https://code.google.com/p/stm32flash/

* DTR is connected to the reset pin
* RTS is connected to the boot0 pin
* RX and TX are connected as usual.

Just type

    make upload

to upload the firmware.

