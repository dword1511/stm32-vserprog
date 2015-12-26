# stm32-vserprog:
## flashrom serprog programmer based on STM32F103 MCU & USB CDC protocol.
**This project deprecates previous [serprog-stm32vcp](https://github.com/dword1511/serprog-stm32vcp) project, which uses STMicro's proprietary firmware library. Most functions are the same.**

* * *
### Features
* Fully opensource: now with [libopencm3](https://github.com/libopencm3/libopencm3) instead of STMicro's proprietary firmware library.
* Affordable and simple hardware:
  * A STM32F103C8T6 MCU, a 8MHz crystal, a 3.3V 1117 LDO, some 0805 capacitors, resistors and LEDs along with dedicated [PCB available on OSH Park](https://oshpark.com/shared_projects/08Rj6sSm)
  * Or some general-purpose STM32F103 development boards, just with minor modifications in the source code to assign correct GPIO for USB D+ pullup and LEDs.
  * Hardware USB2.0 FullSpeed and efficient virtual COM port with USB CDC protocol eliminates the need of USB-to-UART bridges and the headache that comes with them, operates at any baud rates.
  * *Ironically, you will still have to buy or borrow a USB-to-UART bridge (not RS-232 but TTL level) to program the programmer itself.*
* Hardware full-duplex SPI with DMA, multiple clock speeds available:
  * 36MHz *(Default)*
  * 18MHz
  * 9MHz
  * 4.5MHz
  * 2.25MHz
  * 1.125MHz
  * 562.5kHz
  * 281.25kHz
* 2 status LEDs (controlled with 1 GPIO):
  * Busy is lit during operation.
  * Different operation causes different pattern of blinking: mostly busy = reading, both on = writing, flashing alternatively = erasing.
  * Busy blinks along if a hard fault or NMI is detected. Please unplug and replug the programmer. If situation persists, please consider [opening a new ticket](https://github.com/dword1511/stm32-vserprog/issues).
* flashrom serprog protocol:
  * Operates under all major platforms (see [flashrom wiki](http://www.flashrom.org/Flashrom)).
  * Driver-free under Linux. On Windows may require ST's VCP driver.
* Interrupt-free except for low-level USB handling -- easy to debug and modify.
* Fast:
  * Read speed up to 850KiB/s @ 36MHz SPI operation.
  * Dumps a EN25Q64 in 11 seconds and programs in 85 seconds (including flashrom calibration time).
  * Way faster than many commercial CH341-based USB programmers, especially for reading.
* Support 25 and 26 series SPI flash chips. 45 series is **NOT** supported by flashrom.

* * *
### Installation
1. Install stm32flash and gcc-arm-none-eabi toolchain.

  On Debian, simply do the following:

   ```bash
   sudo apt-get install stm32flash gcc-arm-none-eabi
   ```

1. Clone and compile.

  Simply type:

   ```bash
   git clone https://github.com/dword1511/stm32-vserprog.git
   make
   ```
1. Program.

  On dedicated PCB, solder the "PROG ENABLE" jumper, then connect it to a USB slot nearby the one that USB-to-UART bridge is using, then connect TXD and RXD lines with wires.
  For other boards, find the BOOT0 jumper or ISP switch, put it into high or enabled, then connect your board's UART to your computer with the USB-to-UART bridge and apply power to your board.
  Then type:

   ```bash
   make flash
   ```
1. Done!

  Throw the USB-to-UART bridge away and enjoy. Do not forget to desolder the jumper / pull BOOT0 low before resetting / replugging the board.

* * *
### Usage
The following assuming Linux platform, and the programmer appears as /dev/ttyACM0.

1. To read a flash chip:
  * Connect an 25 type SPI Flash to the board's DIP-8 slot or according to the schematics.
  * Connect your board to your PC via USB.
  * Type:

   ```bash
   flashrom -p serprog:dev=/dev/ttyACM0:4000000 -r file-to-save.bin
   ```
  * Some times flashrom will ask you to choose a chip, add something like:

   ```bash
   -c SST25VF040B
   ```
   This is because sometimes multiple devices with different timing requirements can only be distinguished by the device code, however currently flashrom will not read it. Besides, some flash chips support more than one instruction sets.
1. To erase a flash chip:
  * Erase is automatically done during writing. However, if you simply want an empty chip, you will need to erase manually.
  * Type:

   ```bash
   flashrom -p serprog:dev=/dev/ttyACM0:4000000 -E
   ```
  * For certain chips like MX25L6445E, first pass could fail with old flashrom version, but if you do a second pass, everything will be alright. Seems that the first block needs more delay to be erased.
  * Flash status are verified to be empty automatically.
  * The whole process can take a few minutes.
1. To write a flash chip:
  * Type:

   ```bash
   flashrom -p serprog:dev=/dev/ttyACM0:4000000 -w file-to-load.bin
   ```
  * Flash chips are checked and blocks that are not empty are automatically erased.
  * Images are verified after writing automatically.
  * The whole process can take a few minutes.

* * *
### Problems?
1. If encountered something like "Error: Cannot open serial port: Device or resource busy", please try to stop or remove modemmanager.
1. Check your wirings and flashrom version. Do not forget to power the flash chip itself.
1. If you are sure it is caused by something wrong in the programmer's firmware,
   please [open a new ticket](https://github.com/dword1511/stm32-vserprog/issues) and provide details such as the board you are using and flashrom's output. I appreciate your feedback.
