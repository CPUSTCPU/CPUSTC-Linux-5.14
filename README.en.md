# CPUSTC LoongArch32 Linux

[中文](README.md) | English

Linux 5.14 for the CPUSTC FPGA SoC and its LoongArch32 CPU. The tree keeps the
upstream kernel layout and adds CPUSTC architecture support, device trees, and
drivers for the display, input, network, storage, and accelerator blocks.

## Pipeline

```text
CPUSTCore / FPGA SoC
        -> LoongArch32 Linux and device tree
        -> CPUSTC drivers and standard kernel interfaces
        -> initramfs or an external root filesystem
        -> U-Boot / Buildroot boot
```

See [platform and hardware](docs/platform.md) for registers, interfaces,
configuration, build details, and board validation.

## Requirements

| Tool | Purpose |
| --- | --- |
| LoongArch32 `ilp32s` cross toolchain | Kernel build |
| GNU make, binutils, and `nproc` | Configuration and build |
| An initramfs or bootable root filesystem | Boot testing |

## Quick Start

```sh
export ARCH=loongarch
export CROSS_COMPILE=loongarch32r-linux-gnusf-
make O=la_build la32_defconfig
make O=la_build -j"$(nproc)" vmlinux
```

The repository script `./la_build.sh` also accepts the toolchain, CPU frequency,
initramfs, and output settings described in [platform and hardware](docs/platform.md).

## Repository Structure

```text
arch/loongarch/                     LoongArch32 and CPUSTC platform code
arch/loongarch/boot/dts/loongson/   CPUSTC device trees
drivers/                            CPUSTC device drivers
include/uapi/                       CPUSTC DRM and TensorCore interfaces
Documentation/                      Upstream Linux documentation and bindings
docs/platform.md                    CPUSTC platform, build, and validation notes
```

## Documentation

- [Chinese documentation index](docs/README.md)
- [Platform and hardware](docs/platform.md)
- [Upstream Linux documentation](Documentation/)
- [Change and validation notes](note.md)

## Validation

The current source has been validated on the target FPGA board. Details for a
particular device tree, configuration, or FPGA design are recorded in
[platform and hardware](docs/platform.md).

## License

Linux licensing and per-file SPDX declarations remain those of upstream Linux:
the kernel is GPL-2.0-only, while UAPI files may use the Linux syscall exception.
See [COPYING](COPYING) and [LICENSES](LICENSES/). CPUSTC additions do not change
the upstream licensing boundary.
