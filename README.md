
# BMC - Board Management controller based on MCU STM32F042

The STM32CubeF0 MCU Package composed of the STM32Cube hardware abstraction layer (HAL) and the low-layer (LL) APIs
was provided by [ST Micro](https://www.st.com/en/embedded-software/stm32cubef0.html).

HAL and LL APIs are available under open-source BSD license for user convenience.

## Multi-UART USB CDC for STM32F042

* ACM0 - MCU UART1 - connected to COM express SER0 console
* ACM1 - MCU VCP ( Virtual com port)

### VCP Command control :

In order to use the commands; open a terminal emulator like minicom, putty and point to /dev/ttyACM1 (assuming first and second ACM are /dev/ttyACM0 and /dev/ttyACM1).

The following commands can be executed :
```
	re - MCU Reset
	ra - Assert reset signal
	rd - Deassert reset signal
	fl - Assert force recovery signal (BIOS_DIS0# signal)
	ff - Deassert force recovery signal
	pl - Assert power button
	ph - Deassert power button (float)
	vn - Connect vbat transistor
	vf - Disconnect vbat transistor
	cu - Report current measurement
	vo - Report voltage measurement
	ss - Connect SPI ROM to STM32
	sc - Connect SPI ROM to COM express
	si - Report SPI ID
	su - Report SPU unique id
	sr - SPI Read byte - syntax is 'sr 0x0'
	sw - SPI Write byte - syntax is 'sw 0x0 0x0'
	sp - SPI Page Write - syntax is 'sp 0x0 0x11223344556677..' - the 0x0 is the full address and the value is up to 256Bytes (full page)
	se - Erase SPI flash
  	ee - Enable STM32 commands echo
  	ed - Disable STM32 commands echo (useful when flashing SPI image)
```

### Reading SPI ID example

In the terminal emulator execute the following command :

```
si
```

### Turning the board on and off

To control the power button, set it low and then high by running the following two commands :

```
pl
ph
```

### Flashing the SPI flash and booting the COM

- In order to flash the board; run the following commands on the virtual terminal :

```
ss - Makes sure that the SPI MUX connects the SPI flash to the STM32
se - Erases the whole flash
ed - Set STM32 commands echo to be disabled
```
- Prepare the SPI image; if you are using LX2 COM then you can use first 4MByte lx2160acex7_xspi_2000_700_3200_8_5_2.img built by lx2160a_build project :
	```
	dd if=lx2160acex7_xspi_2000_700_3200_8_5_2.img of=lx2_spi_4MB.img bs=1M count=4
	```

- Compile the below write.c example and run it. If using minicom then exit the virtual terminal, if using putty you can keep it open when running the below command :
	```
	./write lx2_spi_4MB.img /dev/ttyACM1
	```
	> **Note**: Flashing the whole 4MByte (SPI flash size on HoneyComb) will take approximately 2 minutes.

- The following commands will boot the COM :

	```
	ee - Enable STM32 commands echo
	sc - Set the SPI mux to be connected to the COM
	fl - Set the BIOS_DIS0# signal low to force the COM to boot from external SPI flash
	pl - Assert the power button to enable power on the COM
	pf - Deassert the power button
	```

### Build Requirements

- ARM GCC (arm-none-eabi-gcc-) Recommended to use latested toolchain to better optimize for space
- Alternatively you can use the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) stm tools but this wasn't tested

## Generate the code project

```
cd ./src
make clean && make
```
The binaries output will be under ./src/build (stm32cdc.bin, .elf, .hex)

#### Elf2dfuse

This tool is a possible aid for STM32 developers who want to generate a DfuSe image directly from a STM32 ELF object file.

The source code for the `elf2dfuse` here <https://github.com/majbthrd/elf2dfuse>.

## Flashing the binaries to your MCU

If using HoneyComb / ClearFog CX board; perform the following :

- Place a jumper J5017 (near the micro USB connectors). This jumper will force the STM32 to boot in DFU mode.
- Connect your PC to your board by plugging a USB to micro USB cable from your PC to the micro USB connector marked as 'Management'.
- You can use `dfu-util` - Device Firmware Update (DFU) USB programmer to flash the FW to the MCU STM32,
[dfu tool man page]( http://manpages.ubuntu.com/manpages/xenial/man1/dfu-util.1.html)

Then run the command below  :

```
# flashing the binaries and boot the MCU
dfu-util  -l -d 0483:df11 -a 0 -s 0x08000000:leave -D ./build/stm32cdcuart.bin
```
The USB device `idVendor=048 , idProduct=df11`

> **Note:** You can reflash the MCU by replugging it; or issuing a `re` command that will reset the MCU in DFU mode (if the jumper is plugged).

## DMA-accelerated multi-UART USB CDC for STM32F042 microcontroller

- `config.h` has a `NUM_OF_CDC_UARTS` value that is used throughout the code to control the number of CDC UARTs.
- The **Command and Data Interface** numbers in the USB descriptor in `usbd_desc.c` must be continguous and start from **zero**.
- The `UARTconfig` array in `stm32f0xx_hal_msp.c` must be customized to suit the pin-mapping used in your application.
- An understanding of USB descriptors is important when modifying `usb_desc.c`. This data conveys the configuration of the device (including endpoint, etc.) to the host PC.
- The DMA IRQ handlers in `usbd_cdc.c` must be consistent with the `UARTconfig` array in `stm32f0xx_hal_msp.c`.
- USB transfers are handled via a distinct section of memory called **PMA**. Read the ST documentation on this. At most, there is 1k Bytes that must be shared across all endpoints. Consider the usage of this PMA memory when scaling up the number of UARTs and buffer sizes.

## Testing was done with

- Linux and MCU STM32F042K4U6
- toolchain gcc-arm-none-eabi-9-2020-q2-update
- dfu-util v0.9

## Patching dfu-util

There seems to be an issue with `dfu-util` when instructing the MCU to jump to the downloaded code. The symptom is that `lsusb` shows the MCU as being in DFU mode after running the above `dfu-util` command.

The following workaround improves the failure rate but doesn't resolve the issue :

```
diff --git a/src/dfuse.c b/src/dfuse.c
index 527ac54..62ebd94 100644
--- a/src/dfuse.c
+++ b/src/dfuse.c
@@ -26,6 +26,7 @@
 #include <stdlib.h>
 #include <errno.h>
 #include <string.h>
+#include <time.h>

 #include "portable.h"
 #include "dfu.h"
@@ -280,6 +281,7 @@ int dfuse_dnload_chunk(struct dfu_if *dif, unsigned char *data, int size,
 		return ret;
 	}
 	bytes_sent = ret;
+	usleep(100000);

 	do {
 		ret = dfu_get_status(dif, &dst);
```

### Known issues

- After SPI page write(s) SPI read returns `0x0`; to workaround that the MCU needs to be reset after completing all the required SPI page writes
- The dfu-util issue as discussed above
- Hangs when extensively working wih `/dev/ttyACM0` and `/dev/ttyACM1` simultaneously
- Since the STM32 flash is 16KB; not all commands are implemented

### SPI write.c example

Following is an example code that reads a binary and executes a SPI page writes :

```
#include <stdio.h>
#include <unistd.h>
#include <time.h>

void print_help (void) {
	printf ("rwv <file name> <stm32 ttyACM>\n");
}

int main(int argc, char **argv) {
	FILE *fin, *stm;
	int addr = 0, addr2 = 0;
	int len = 0, mark_break = 0, i;
	unsigned char buffer[256];
	if (argc != 3) {
		print_help();
		return 0;
	}

	fin = fopen (argv[1], "r");
	if (!fin) {
		printf ("Can't open %s\n",argv[1]);
		return -1;
	}
	stm = fopen (argv[2], "a+");
	if (!stm) {
		printf ("Can't open STM ACM %s\n",argv[2]);
		return -1;
	}
	len = 0;
	while (1) {
		int input = fgetc(fin);
		if (input == EOF) mark_break = 1;
		else {
			buffer[len] = (unsigned char) input;
			len ++;
		}
		if (mark_break && !len) break;
		if ((len == 256) || (mark_break)) {
			char str[550];
			sprintf (str, "sp 0x%08x 0x",addr & 0xffffff00);
			for (i = 0 ; i < len ; i++) {
				sprintf (&str[8+8+i*2],"%02x",buffer[i]);
			}
			printf ("%s\n",str);
			len = 0;
			fprintf(stm, "%s\n",str);
			fflush(stm);
		}
		if (mark_break) break;
		addr ++;
	}
	fclose (stm);
	fclose (fin);
}
```
