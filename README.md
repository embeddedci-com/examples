# EmbeddedCI Examples

This repository contains small, focused examples used by EmbeddedCI workflows.

## Examples by directory

- `buildroot-external/generic-kernel`  
  Minimal Buildroot `BR2_EXTERNAL` tree for kernel-only builds.  
  Includes architecture-specific defconfigs (`arm_defconfig`, `arm64_defconfig`) and the required Buildroot external glue files.

- `buildroot-external/generic-rootfs`  
  Minimal Buildroot `BR2_EXTERNAL` tree for building a generic Linux root filesystem image.  
  Includes defconfigs plus rootfs overlays under `board/generic-rootfs/overlays` (`common` and `debug`).

- `selftest`  
  Very small Linux userspace app example (`selftest.c`) with a simple `Makefile`.  
  Builds a `selftest` binary that prints basic runtime info and an `APP_OK` marker for CI checks.

- `selftest-stm32`  
  Bare-metal STM32 example targeting STM32F446 (`arm-none-eabi` toolchain).  
  Produces `.elf`, `.bin`, and `.hex` artifacts using STM32Cube HAL sources.

- `yocto-kas-image`  
  Yocto/KAS manifest example (`kas.yml`) for building a `core-image-minimal` image (BeagleBone machine) with pinned layer revisions.

## Notes

- The Buildroot examples contain their own local `README.md` files with usage snippets.
- These examples are intentionally minimal and are meant to be referenced from CI/buildpack configs.
