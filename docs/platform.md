# CPUSTC LoongArch32 Linux 平台说明

[返回项目 README](../README.md)

## 平台配置

| 平台 | 配置 | 设备树 |
| --- | --- | --- |
| CPUSTC-LS FPGA SoC | `la32_defconfig` | `loongson32_ls.dts` |
| la32r-QEMU | `la32_defconfig` | 按 QEMU 设备选择 |
| BX 参考 SoC | `la32_bx_defconfig` | `loongson32_bx.dts` |

CPUSTC-LS 默认配置：LA32R、单核、4 KiB 页、MMU、soft-float、250 Hz tick、非一致
DMA、8 MiB CMA、内嵌设备树。CPU 定时器默认 50 MHz，APB 外设时钟 33 MHz，内存
128 MiB。`CONFIG_INITRAMFS_SOURCE` 为空，启动时需要外部根文件系统或单独提供
initramfs。

## 硬件接口

| 硬件 | Linux 接口 | 地址/设备节点 |
| --- | --- | --- |
| 16550 UART | `8250_of`、`ttyS0` | `serial@1fe001e0`，33 MHz |
| VGA | `cpustc-vga` DRM/KMS | 固定 640x480、25 MHz、RGB565 |
| LCD | `cpustc-lcd` DRM/KMS | 480x800 RGB565、NT35510 |
| GT9147 | Goodix、evdev | I2C `0x5d`、APB 输入 0 |
| USB | OHCI platform | `0x1fe02000`、CPU HWI5 |
| Ethernet | `dmfe` | `eth0`、10/100M |
| SD | LiteX MMC | `/dev/mmcblkN` |
| NAND | `cpustc_nand`、MTD/UBI | 默认 DTS 禁用 |
| 矩阵键盘 | `cpustc_confreg_keypad` | `/dev/input/eventX` |
| 8x8 点阵 | `cpustc_dotmatrix` | `/dev/cpustc-dotmatrix` |
| TensorCore | `cpustc_tensor` | `/dev/cpustc-tensor` |

CPU HWI7 连接 CPUSTC APB 级联控制器；级联输入 0 为触摸，1 为 LCD，2 为 VGA，
3 为 TensorCore，4 为 SDIO。

### DMA 与 Cache

CPUSTCore 的 DMA 不与 CPU Cache 硬件一致。驱动使用 DMA API，平台代码负责
L1/L2 writeback、invalidate、uncached 映射和内存屏障。VGA、LCD、DMFE、NAND、
TensorCore 都使用这套路径。

### VGA 与 LCD

- VGA 驱动提供 atomic modeset、fbdev emulation、vblank、CMA dumb buffer、硬件
  cursor 和 2D UAPI；接口定义在 `include/uapi/drm/cpustc_drm.h`。
- LCD 驱动提供 480x800 RGB565、damage 区域 DMA、背光和恢复属性；面板没有读回和
  物理 vblank，平台设备属性可从 `/sys/bus/platform/devices` 查找。

### USB 鼠标轮询

`usbhid.mousepoll` 的单位是毫秒，默认值为 `0`。此时驱动不使用统一周期覆盖设备
配置，而是让每只鼠标采用自身中断端点描述符中的轮询周期；当前上板测试中，这一
默认策略的综合效果最好，因此不把固定轮询频率作为主要方案。

上述结论的测试环境为配置 512 KiB L2 Cache 的 CPUSTC FPGA SoC；使用其他 L2
容量或系统负载时，需要重新验证合适的轮询周期。

强制周期测试表明：全速鼠标在 32 ms（31.25 Hz）下可能出现位移报告积压和明显的
停止尾迹，低速鼠标在 4 ms（250 Hz）下表现较差。遇到特定鼠标兼容性或系统负载
问题时，可使用 `mousepoll` 辅助覆盖设备周期；其中 8 ms（125 Hz）是兼顾已测试
全速和低速鼠标的折中取值。

用户可根据鼠标和负载调整运行时参数：

```sh
echo 0  > /sys/module/usbhid/parameters/mousepoll  # 使用设备描述符周期，默认值
echo 4  > /sys/module/usbhid/parameters/mousepoll  # 250 Hz
echo 8  > /sys/module/usbhid/parameters/mousepoll  # 125 Hz，可选折中值
echo 16 > /sys/module/usbhid/parameters/mousepoll  # 62.5 Hz
echo 32 > /sys/module/usbhid/parameters/mousepoll  # 31.25 Hz
```

参数会在 USB HID 输入 URB 创建时读取；运行时修改后需要拔插鼠标，或者重新绑定对应
的 `usbhid` 接口。需要永久调整时，在内核命令行加入
`usbhid.mousepoll=<毫秒数>`。Full-Speed 和 Low-Speed 中断周期会由 USB core 向下
取整到不大于配置值的 2 的幂；当前 OHCI 驱动还会把大于 32 ms 的周期限制为 32 ms。

### TensorCore

`CPUSTC_TENSOR_IOC_RUN` 接收 `4x4` FP32 矩阵，`K` 范围为 1 至 256，返回结果和
硬件周期数。UAPI 定义在 `include/uapi/linux/cpustc_tensor.h`。

## 构建

### 手工构建

```sh
export ARCH=loongarch
export CROSS_COMPILE=loongarch32r-linux-gnusf-
make O=la_build la32_defconfig
scripts/config --file la_build/.config \
    --set-str INITRAMFS_SOURCE /path/to/rootfs.cpio.gz
make O=la_build olddefconfig
make O=la_build -j"$(nproc)" vmlinux
```

产物：`la_build/vmlinux` 和 `la_build/arch/loongarch/boot/dts/loongson/loongson32_ls.dtb`。

### `la_build.sh`

```sh
CPUSTC_TOOLCHAIN_PATH=/path/to/loongarch-toolchain \
CPUSTC_VMLINUX_OUTPUT="$PWD/la_build/vmlinux.stripped" \
./la_build.sh
```

可用变量：

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `CPUSTC_TOOLCHAIN_PATH` | 脚本默认目录 | 工具链根目录 |
| `CPUSTC_CPU_FREQ_HZ` | `50000000` | CPU/constant timer 频率 |
| `CPUSTC_VMLINUX_OUTPUT` | `/tftpboot/vmlinux` | strip 后的输出路径 |
| `CPUSTC_BUILD_LOG` | 空 | 追加构建日志的路径 |

## 启动与检查

U-Boot 从网络加载：

```text
tftpboot 0xa3000000 vmlinux
bootelf 0xa3000000 earlycon console=ttyS0,115200n8 rdinit=/init
```

系统启动后：

```sh
uname -a
cat /proc/cpuinfo
cat /proc/interrupts
dmesg | grep -Ei 'cache|dma|cpustc|dmfe|ohci|goodix|mmc'
ls -l /dev/dri /dev/input/event* /dev/video* 2>/dev/null
cat /sys/class/drm/card*-*/modes 2>/dev/null
cat /proc/partitions
```

GT9147 ID：

```sh
i2ctransfer -y 0 w2@0x5d 0x81 0x40 r4
```

点阵测试：

```sh
printf '\201B$\030\030$B\201' > /dev/cpustc-dotmatrix
```

## 源码导航

| 内容 | 路径 |
| --- | --- |
| 默认配置 | `arch/loongarch/configs/la32_defconfig` |
| CPUSTC-LS 设备树 | `arch/loongarch/boot/dts/loongson/loongson32_ls.dts` |
| CPU 中断分发 | `arch/loongarch/loongson32/irq.c` |
| 非一致 DMA | `arch/loongarch/loongson32/dma-noncoherent.c` |
| Cache | `arch/loongarch/mm/cache.c` |
| APB irqchip | `drivers/irqchip/irq-cpustc-apb.c` |
| VGA | `drivers/gpu/drm/tiny/cpustc-vga.c` |
| LCD | `drivers/gpu/drm/tiny/cpustc-lcd.c` |
| Ethernet | `drivers/net/dmfe.c` |
| NAND | `drivers/mtd/nand/raw/cpustc_nand.c` |
| TensorCore | `drivers/misc/cpustc_tensor.c` |

## 已知限制

- VGA 固定 640x480；LCD 没有物理 vblank/TE。
- OHCI Full-Speed 限制 USB 摄像头和 Wi-Fi 带宽。
- NAND、SPI Flash 默认关闭。
- TensorCore completion 当前由状态寄存器轮询完成。
- `poweroff`/`reboot` 没有固件服务时不会控制 FPGA 板物理电源。
