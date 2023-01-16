Standard boot-up sequence before BIOS is loaded

```
NOTICE:  BL2: v2.4(release):v2.0-4471-g865d93d7e
NOTICE:  BL2: Built : 22:21:42, Dec 28 2022
NOTICE:  UDIMM M474A4G43AB1-CVF
NOTICE:  DDR4 UDIMM with 2-rank 64-bit bus (x8)

NOTICE:  64 GB DDR4, 64-bit, CL=22, ECC on, 256B, CS0+CS1
NOTICE:  BL2: Booting BL31
NOTICE:  BL31: v2.4(release):v2.0-4471-g865d93d7e
NOTICE:  BL31: Built : 22:21:47, Dec 28 2022
NOTICE:  Welcome to lx2160acex7 BL31 Phase

UEFI firmware built at 01:40:14 on Dec 28 2022. version:
202105
SOC: LX2160ACE Rev2.0 (0x87360020)
UEFI firmware (version 202105 built at 01:40:24 on Dec 28 2022)
```

## Compiler Requirements

## Code Repo Checkouts
```
git clone https://github.com/em-winterschon/project-limbs.git
cd project-limbs
git submodule update --init --recursive

```

## Steps to flash the ESP32 based BMC board

1. Use the jumper link to force the ESP32 BMC on the Honeycomb to DFU mode and power up the board.

- Jumper is named "J5017" on the PCB directly in front of the micro-usb connectors.

2. More instructions here...

44. Unset the jumper and cycle the power.



## References:
- STM32 USB shows as DFU device: https://community.solid-run.com/t/stm32-usb-shows-as-dfu-device/184/2
- Repo for BMC firmware fork: https://github.com/em-winterschon/project-limbs#vcp-command-control-
- Project-Limbs PR to add `-flto` to `CFLAGS` compile time options, "Add -flto to reduce binary size": https://github.com/SolidRun/project-limbs/pull/1

