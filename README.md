# touch-inject-kernel

Kernel module for FTS touch injection on Xiaomi 13 (fuxi).

Built via GitHub Actions using GKI kernel build system (in-tree compilation) to produce a module compatible with CFI/LTO enabled GKI 5.15 kernels.

## Usage

1. Download `touch_inject.ko` from the latest [Actions run](../../actions)
2. First load CFI_PASS: `insmod CFI-5.15-13.ko`
3. Then load: `insmod touch_inject.ko`
4. Test: `echo "T 540 1200" > /dev/touch_inject`
