# CPUSTC LoongArch32 Linux

[中文](README.md) | [English](README.en.md)

面向 CPUSTC FPGA SoC 的 LoongArch32 Linux 5.14 内核。仓库保留上游内核结构，新增的
CPUSTC 架构适配、设备树和驱动用于连接 CPUSTCore、显示、输入、网络、存储及加速器。

## Pipeline

```text
CPUSTCore / FPGA SoC
        -> LoongArch32 Linux + 设备树
        -> CPUSTC 驱动与标准内核接口
        -> initramfs 或外部根文件系统
        -> U-Boot / Buildroot 启动
```

平台寄存器、驱动接口、配置和板级验证方法见
[平台与硬件适配](docs/platform.md)。

## Requirements

| 工具 | 用途 |
| --- | --- |
| LoongArch32 `ilp32s` 交叉工具链 | 编译内核 |
| GNU make、binutils、`nproc` | 配置和构建 |
| initramfs 或可启动根文件系统 | 启动测试 |

## Quick Start

```sh
export ARCH=loongarch
export CROSS_COMPILE=loongarch32r-linux-gnusf-
make O=la_build la32_defconfig
make O=la_build -j"$(nproc)" vmlinux
```

需要使用项目脚本时，可执行 `./la_build.sh`；工具链、initramfs、CPU 频率和输出位置
等参数见 [平台与硬件适配](docs/platform.md)。

## Repository Structure

```text
arch/loongarch/                     LoongArch32 架构与 CPUSTC 平台
arch/loongarch/boot/dts/loongson/   CPUSTC 设备树
drivers/                            CPUSTC 显示、输入、网络、存储和加速器驱动
include/uapi/                       CPUSTC DRM、TensorCore 等用户接口
Documentation/                      Linux 上游文档与绑定
docs/platform.md                    CPUSTC 平台说明、构建与验证
```

## Documentation

- [文档索引](docs/README.md)
- [平台与硬件适配](docs/platform.md)
- [Linux 上游文档](Documentation/)
- [变更与验证记录](note.md)

## Validation

当前源码已完成上板验证。

## License

Linux 内核继续沿用上游许可证和逐文件 SPDX 声明：主体代码为 GPL-2.0-only，UAPI
文件可带 Linux syscall exception。详见 [COPYING](COPYING) 与 [LICENSES](LICENSES/)。
CPUSTC 新增代码不改变上游内核的许可证边界。
