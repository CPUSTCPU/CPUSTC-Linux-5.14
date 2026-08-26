#!/bin/bash
set -euo pipefail


TOOLCHAIN_ROOT=${CPUSTC_TOOLCHAIN_PATH:-/opt/loongarch-toolchain}
export CROSS_COMPILE="${CPUSTC_CROSS_COMPILE:-${TOOLCHAIN_ROOT%/}/bin/loongarch32r-linux-gnusf-}"
export ARCH=loongarch
OUT=la_build
CPUSTC_CPU_FREQ_HZ=${CPUSTC_CPU_FREQ_HZ:-50000000}
CPUSTC_VMLINUX_OUTPUT=${CPUSTC_VMLINUX_OUTPUT:-/tftpboot/vmlinux}

case "${CPUSTC_CPU_FREQ_HZ}" in
    ''|*[!0-9]*)
        echo "CPUSTC_CPU_FREQ_HZ must be a positive integer in Hz" >&2
        exit 2
        ;;
esac
if [ "${CPUSTC_CPU_FREQ_HZ}" -eq 0 ]; then
    echo "CPUSTC_CPU_FREQ_HZ must be greater than zero" >&2
    exit 2
fi
export CPUSTC_CPU_FREQ_HZ

mkdir -p "${OUT}"
make la32_defconfig O="${OUT}"

echo "----------------output ${OUT}----------------"
echo "CPUSTC CPU timer frequency: ${CPUSTC_CPU_FREQ_HZ} Hz"

make olddefconfig O="${OUT}"
if [ -n "${CPUSTC_BUILD_LOG:-}" ]; then
    make vmlinux -j"$(nproc)" O="${OUT}" 2>&1 | tee -a "${CPUSTC_BUILD_LOG}"
else
    make vmlinux -j"$(nproc)" O="${OUT}"
fi

${CROSS_COMPILE}strip -o "${CPUSTC_VMLINUX_OUTPUT}" "${OUT}/vmlinux"
