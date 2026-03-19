# EmbeddedCI generic kernel (BR2_EXTERNAL)

Minimal Buildroot external tree for kernel-only builds. Architecture is chosen by defconfig, not by directory name.

## Layout

- **external.desc**, **external.mk**, **Config.in** — Required BR2_EXTERNAL glue.
- **configs/** — Defconfigs: `arm_defconfig`, `arm64_defconfig`.

## Usage

In `embeddedci-buildroot.yaml`:

```yaml
- id: system/buildroot-kernel
  config:
    source: git+https://github.com/embeddedci-com/examples.git//buildroot-external/generic-kernel?ref=main
    defconfig: arm_defconfig   # or arm64_defconfig
    ref: v6.12
    arch: arm64
```

Kernel image naming and staging are handled by the `system/buildroot-kernel` pack script (`Image` for arm64, `zImage` for arm), with optional DTBs copied into `kernel/dtbs/`.
