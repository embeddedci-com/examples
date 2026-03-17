# EmbeddedCI generic rootfs (BR2_EXTERNAL)

Minimal Buildroot external tree for building generic Linux rootfs images. Architecture is chosen by defconfig, not by directory name.

## Layout

- **external.desc**, **external.mk**, **Config.in** — Required BR2_EXTERNAL glue.
- **configs/** — Defconfigs: `generic_rootfs_defconfig` (arm64), `arm_defconfig`, `arm64_defconfig`.
- **board/generic-rootfs/overlays/** — Rootfs overlay dirs (common, debug); copied into target rootfs.

## Usage

In `embeddedci.yaml`:

```yaml
- id: system/buildroot-rootfs
  config:
    source: examples/buildroot-external/generic-rootfs
    defconfig: generic_rootfs_defconfig   # or arm_defconfig / arm64_defconfig
    overlays:
      - board/generic-rootfs/overlays/common
      - board/generic-rootfs/overlays/debug
```

Overlays contain static files only (e.g. `etc/motd`, `etc/profile.d/debug.sh`). App binaries are injected by the buildpack’s post-build script when `app_artifacts` is set.
