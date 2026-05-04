# EmbeddedCI generic kernel (BR2_EXTERNAL)

Minimal Buildroot external tree for kernel-only builds. Architecture is chosen by defconfig, not by directory name.

## Layout

- **external.desc**, **external.mk**, **Config.in** — Required BR2_EXTERNAL glue.
- **configs/** — Buildroot defconfigs (e.g. `arm_defconfig`, `arm64_defconfig`). Add more `*_defconfig` files as needed; set pack **`defconfig`** to the corresponding make target name. `system/buildroot-kernel` enables `BR2_LINUX_KERNEL=y` and, when board **dtbs:** is non-empty, also forces `BR2_LINUX_KERNEL_DTS_SUPPORT=y` and sets `BR2_LINUX_KERNEL_INTREE_DTS_NAME` to the DTS basenames before running the build.

## Usage

In `embeddedci-buildroot.yaml`:

```yaml
- id: system/buildroot-kernel
  config:
    source: git+https://github.com/embeddedci-com/examples.git//buildroot-external/generic-kernel?ref=main
    defconfig: arm_defconfig   # or any *_defconfig name under configs/
    ref: v6.12
    arch: arm64
```

Kernel images and DTBs installed by Buildroot under `$O/images/` are staged into `kernel/` (and `kernel/dtbs/`) for artifact upload.
