
# BMC - Board Management controller based on MCU STM32F042

The STM32CubeF0 MCU Package composed of the STM32Cube hardware abstraction layer (HAL) and the low-layer (LL) APIs
was provided by [ST Micro](https://www.st.com/en/embedded-software/stm32cubef0.html).

HAL and LL APIs are available under open-source BSD license for user convenience.

## multi-UART USB CDC for STM32F042 :
* ACM0 - MCU UART1
* ACM1 - MCU UART2
* ACM2 - MCU VCP ( Virtual com port)

### VCP Command controll :
- reset : reset the board
- power : power ON/OFF
- sensors : show temp
- ...

### Reading SPI ID example :
- echo "spi_sw_stm-" > /dev/ttyACM1
- echo "spi_id" > /dev/ttyACM1

### Turning the board on and off :
To control the power button IO from STM32 use the pwrbtn_low and pwrbtn_high
commands that asserts and deasserts the power button signal.
```
echo "powerbtn_low" > /dev/ttyACM1
echo "powerbtn_high" > /dev/ttyACM1
```
The poweron and off has some automation in the stm32 code.
```
echo "poweroff" > /dev/ttyACM1
echo "poweron" > /dev/ttyACM1
```
## Build Requirements
- ARM GCC (arm-none-eabi-gcc-)
- Alternatively you can use the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) stm tools

## Generate the code project :
```
cd ./src
make clean && make
```
The binaries output will be under ./src/build (stm32cdc.bin, .elf, .hex)

## Flashing the binares to your MCU :
Select usb boot mode (put the jumper on the MCU) and connect your PC to your board by plugging the micro USB cable to a USB port on your laptop/computer and plugging the other end to MicroUSB port on MCU (STM32).
You can use `dfu-util` - Device firmware update (DFU) USB programmer to flash the FW to the MCU STM32,
[dfu tool descriptor]( http://manpages.ubuntu.com/manpages/xenial/man1/dfu-util.1.html)
Then run the command bellow  :
```
# flashing the binaries and boot the MCU
dfu-util  -l -d 0483:df11 -a 0 -s 0x08000000:leave -D ./build/stm32cdcuart.bin
```
The USB device `idVendor=048 , idProduct=df11`
#### Note: reconnect the mUSB when you need to reflash the MCU.

## DMA-accelerated multi-UART USB CDC for STM32F042 microcontroller

- `config.h` has a `NUM_OF_CDC_UARTS` value that is used throughout the code to control the number of CDC UARTs.
- The **ommand and Data Interface** numbers in the USB descriptor in `usbd_desc.c` must be continguous and start from **zero**.
- The `UARTconfig` array in `stm32f0xx_hal_msp.c` must be customized to suit the pin-mapping used in your application.
- An understanding of USB descriptors is important when modifying `usb_desc.c`. This data conveys the configuration of the device (including endpoint, etc.) to the host PC.
- The DMA IRQ handlers in `usbd_cdc.c` must be consistent with the `UARTconfig` array in `stm32f0xx_hal_msp.c`.
- USB transfers are handled via a distinct section of memory called **PMA**. Read the ST documentation on this. At most, there is 1k Bytes that must be shared across all endpoints. Consider the usage of this PMA memory when scaling up the number of UARTs and buffer sizes.

## Testing was done with:
- Linux and MCU STM32F042K4U6
- toolchain arm-none-eabi-gcc-7.3.1
- dfu-util v0.9
