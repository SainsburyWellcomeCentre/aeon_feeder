
# Build Firmware Development Environment

Hold Boot sel button on the board and plug in the USB cable to your computer. The board will appear as a USB drive named `RPI-RP2`. Copy the latest [MicroPython UF2 firmware](https://micropython.org/download/SEEED_XIAO_RP2040/) for your board to the drive. The board will reboot and appear as a serial device.

## Install Dependencies

Install the board-side dependencies before flashing or running the firmware.
The firmware depends on the MicroPython USB CDC package plus the project library packages for the Harp protocol and Dynamixel support:

```bash
mpremote connect COMx  # replace COMx with your board's serial port
mpremote mip install github:SainsburyWellcomeCentre/micropython-microharp
mpremote mip install github:SainsburyWellcomeCentre/micropython-dynamixel
```

## copy the firmware to the board

```bash
mpremote connect COMx  # replace COMx with your board's serial port
mpremote fs cp -r firmware/src :
```
