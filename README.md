# stm32-vserprog:
## flashrom serprog programmer based on STM32 MCU & USB CDC protocol.
**This project deprecates previous [`serprog-stm32vcp`](https://github.com/dword1511/serprog-stm32vcp) project,
  which uses STMicro's proprietary firmware library. Most functions are the same.**

* * *
### Features
* Fully open-source: now with [`libopencm3`](https://github.com/libopencm3/libopencm3) instead of STMicro's proprietary firmware library.
* Affordable and simple hardware:
  * An _STM32F103C8T6_ MCU, an 8MHz crystal, a 3.3V _1117_ LDO, some _0805_ capacitors, resistors and LEDs
    along with dedicated PCB available on OSH Park ([V2](https://oshpark.com/shared_projects/08Rj6sSm)
    or [V3](https://oshpark.com/shared_projects/vKn08YZG))
  * Or some general-purpose _STM32_ development boards,
  just add a new header file under "boards" folder to assign correct GPIO for USB D+ pull-up (_STM32F1_ only) and LEDs.
  * Hardware USB 2.0 Full-Speed and efficient virtual COM port with USB CDC protocol operates at any baud rates,
  eliminates the need of USB-to-UART bridges and the headache that comes with them.
  * **Ironically, you will still have to buy or borrow a USB-to-UART bridge (not RS-232 but TTL level) to program the programmer itself,**
  unless you are using an _STM32F0x2_ device (which has USB ISP capability) or boards with embedded _ST-Link_ (see @FabianInostroza's fork).
* Hardware full-duplex SPI with DMA, multiple clock speeds available (default at the one closest to but under 10MHz), _e.g._ on STM32F103 targets:
  * 36MHz
  * 18MHz
  * 9MHz *(Default)*
  * 4.5MHz
  * 2.25MHz
  * 1.125MHz
  * 562.5kHz
  * 281.25kHz
* 2 status LEDs (controlled with 1 GPIO):
  * Busy is lit during operation (inverse for boards with only one LED).
  * Different operations cause different patterns of blinking:
  mostly busy = reading, both on = writing, flashing alternatively = erasing.
  * Busy blinks alone if a hard fault or NMI is detected. Please unplug and replug the programmer.
  If the situation persists, please consider [opening a new ticket](https://github.com/dword1511/stm32-vserprog/issues).
* `flashrom` "serprog" protocol:
  * Operates on all major platforms (see [`flashrom` wiki](http://www.flashrom.org/Flashrom)).
  * Driver-free on Linux. On Windows may require ST's VCP driver
  (or just hack the `.inf` file of other CDC drivers, not confirmed).
* Firmware is interrupt-free except for low-level USB handling -- robust, easy to debug and modify.
* Fast:
  * Read speed up to 850KiB/s @ 36MHz SPI operation.
  * Dumps an _EN25Q64_ in 11 seconds and programs in 85 seconds (including `flashrom`'s calibration time).
  * Way faster than many commercial _CH341_-based USB programmers, especially for reading.
* Supports 25 and 26 series SPI flash chips.
  **45 series flash chips have different pinouts. Thus for the V2 PCB mentioned above, you will need an adapter.**

* * *
### Installation
1. Install `stm32flash` and the `gcc-arm-none-eabi` toolchain.

  On Debian, simply do the following:

   ```bash
   sudo apt-get install stm32flash gcc-arm-none-eabi
   ```

1. Clone and compile.

  Simply type (change the board name accordingly, for details see the header of the `Makefile` or just type `make`):

   ```bash
   git clone https://github.com/dword1511/stm32-vserprog.git
   make BOARD=stm32-vserprog-v2
   ```
1. Program.

  Connect the USB-to-UART bridge to your computer first. Then connect _TXD_ and _RXD_ lines to your _STM32_ board with wires.
  On V2 or V3 boards mentioned above, connect _BOOT0_ (labeled _BT0_ on boards) to the 3.3V output of the USB-to-UART bridge.
  For other boards, find the _BOOT0_ jumper or ISP switch, put it into high or enabled.
  On some boards, you will also need to ensure _BOOT1_ is pulled low.
  Then, connect your _STM32_ board to your computer via USB to power it up.
  Finally, type:

   ```bash
   make BOARD=stm32-vserprog-v2 flash-uart
   ```
1. Done!

  Throw the USB-to-UART bridge away and enjoy.
  Do not forget to pull _BOOT0_ low before resetting or replugging the board.

* * *
### Usage
The following assumes Linux platform, and that the programmer appears as `/dev/ttyACM0`.

1. To read a flash chip:
  * Connect a 25 type SPI Flash to the board's DIP-8 slot or according to the schematics.
  * Connect your board to your PC via USB.
  * Type:

   ```bash
   flashrom -p serprog:dev=/dev/ttyACM0:4000000 -r file-to-save.bin
   ```
  * Sometimes `flashrom` will ask you to choose a chip, add something like:

   ```bash
   -c SST25VF040B
   ```
   This is because sometimes different devices with distinct timing requirements can only be distinguished by the device code,
   however currently `flashrom` will not read it as it is not returned by the `RDID` command.
1. To erase a flash chip:
  * Erase is automatically done during writing. However, if you simply want an empty chip, you will need to erase manually.
  * Type:

   ```bash
   flashrom -p serprog:dev=/dev/ttyACM0:4000000 -E
   ```
  * For certain chips like _MX25L6445E_, the first pass could fail with some old `flashrom` version,
    but if you try a second pass, everything will be alright. Seems that the first block needs more delay to be erased.
  * Flash contents are verified to be empty automatically.
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
1. If you encountered something like "`Error: Cannot open serial port: Device or resource busy`", please try to stop or remove `ModemManager`.
1. Check your wirings and `flashrom` version. Do not forget to power the flash chip itself if operating on a breadboard or prototype board.
1. If you are sure it is caused by bugs in the programmer's firmware,
   please [open a new ticket](https://github.com/dword1511/stm32-vserprog/issues) and provide details,
   _e.g._ the board you are using and `flashrom`'s output. I appreciate your feedback.
