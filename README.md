# gtaxl-loader

A small arm64 loader for the Samsung Galaxy Tab A 10.1 (2016) Wi-Fi
(SM-T580, Exynos 7870), started by S-BOOT, Samsung's signed bootloader,
from the BOOT and RECOVERY partitions. S-BOOT only starts Samsung boot
images, ignores their command line and refuses ramdisks over 14 MiB, so
the image "kernel" is this loader, followed by a payload with the real
kernel, the DTB, the ramdisk and the command line. The device tree
(`device/samsung/gtaxlwifi`) builds the images.

## What it does

- sets INFORM3 back to "normal boot" (S-BOOT remembers the last mode) and
  INFORM2 to "normal reset";
- takes `androidboot.serialno` from S-BOOT's command line;
- adds `androidboot.mode=charger` when the cable switched the tablet on
  (cold start and PMIC power-on source, read over HSI2C);
- sets the compatible of the panel node to the panel fitted on the tablet,
  one of three, from the `lcdtype` S-BOOT reads off the panel;
- reads the Wi-Fi and Bluetooth addresses of the tablet from Samsung's EFS,
  read-only, and puts them in the DTB;
- passes an entropy seed (the one Android keeps in the vendor space of
  `misc`, plus timing jitter, hashed);
- inflates the kernel to 0x50000000, fills `/chosen` of the DTB at 0x4a000000
  and leaves the ramdisk in place;
- goes to the recovery when the payload is bad or the partition table has no
  `super`;
- leaves any error, and the `lcdtype` of a panel other than the default
  one, in `/chosen/gtaxl,loader-log`.

## Payload

`loader.bin` is loaded by S-BOOT at 0x40080000. At 0x100000
(`PAYLOAD_OFFSET`) from its start comes the payload header: magic
`GXPAYLD1`, flags (bit 0: RECOVERY), offset and size of each piece. The
pieces are 4 KiB aligned: the kernel `Image` as an LZ4 legacy frame
(`lz4 -l`), the DTB, the ramdisk and the command line as text.

## Build

    ./build.sh <output.bin> [<top of the Android tree>]

It uses the tree's clang and lld and libfdt from `external/dtc`. The
Android build runs it from `external/gtaxl-loader`.

| file | what |
|---|---|
| `start.S`, `linker.lds` | entry and layout |
| `loader.c`, `loader.h` | payload, command line, DTB, jump to Linux |
| `mmu.c` | identity map with caches, for the LZ4 inflate |
| `lz4.c` | LZ4 legacy frame decoder |
| `pmic.c` | power-on source from the S2MPU05 PMIC over HSI2C |
| `emmc.c`, `ext4.c` | read-only eMMC (DW-MMC, IDMAC) and ext4, for the EFS |
| `sha256.c` | entropy seed hashing |
| `libc.c`, `include/` | the few libc functions needed |

## License

GPL-2.0-only, see `COPYING`.
