# IMX294/IMX492 Shared Raspberry Pi Driver

This directory builds one kernel module, `imx294_imx492.ko`, from one shared
V4L2 I2C sensor driver source: `imx294_imx492.c`.

The module advertises exact device-tree compatibles for both sensors:

- `sony,imx294`
- `sony,imx492`

Variant data selects the IMX294 or IMX492 mode tables, media-bus formats, and
streaming sequence policy behind the shared `imx29x` implementation. The common
core owns register I/O, controls, format negotiation, stream start/stop,
runtime PM, endpoint validation, and subdev registration.

Fun fact: The reason why we can do this is because essentially IMX294 = QuadBayer and IMX492 = Normal Bayer and both are the same silicon. So they not only share a lot of the code but you can run either one with the other drivers. That brings some fun to how to use it, see Non-standard or hardware-specific modes section for more details.  

The code is mostly generated from my IMX294 and IMX492 driver with the help of codex and iterated on RPI5, but I do wanna say there is a catch - you need to enable RP1 overclock to enable the full potential of this sensor, additionally because RPI's CFE has a bug for 14bit and 16bit, libcamera's RPI pipeline fixed the bug in the software so the 14bit mode may be bottlenecked by the CPU speed abd not reach the top speed stablely.
<img width="1873" height="753" alt="image" src="https://github.com/user-attachments/assets/1ce5708c-4402-4ce4-adf4-e73a97b83a9f" />

## Features

- Shared IMX294/IMX492 V4L2 sub-device driver.
- Single image source pad; no embedded metadata pad is advertised.
- Mode selection is derived from the V4L2 subdev active state.
- DKMS install flow through `setup.sh`.
- Device-tree overlays for both sensors: `imx294.dtbo` and `imx492.dtbo`.
- Four-lane CSI-2 at `1728 Mbps/lane` with `link-frequencies = <864000000>`.
- XCLK-dependent PLRD setup rows for `6`, `12`, `18`, and `24 MHz`.
- Test-pattern, exposure, analogue-gain, blanking, and framerate controls.

Non-standard or hardware-specific modes stay gated by dtoverlay properties:

- `quad-bayer-modes` exposes the experimental IMX294 quad-Bayer full-resolution
  12-bit modes. These still use standard Bayer media-bus codes because Linux
  has no standard 4x4 quad-Bayer code.
- `color-binned-modes` exposes the IMX492 color binned mode, which is useful
  for geometry/timing testing but is expected to collapse color information.
- `mono` exposes IMX492 Y10/Y12 formats, and the mono 12-bit binned mode, for
  monochrome hardware. The overlay parameter maps to the driver's
  `mono-mode` device-tree property.

## Install on Raspberry Pi 5

Install the build prerequisites once:

```bash
sudo apt update
sudo apt install build-essential dkms device-tree-compiler raspberrypi-kernel-headers git
```

Clone or copy this repository to the Pi, then run:

```bash
cd imx294_imx492
chmod +x setup.sh
./setup.sh
```

The setup script copies the source to `/usr/src/imx294_imx492-0.0.1`, installs
the module through DKMS, and builds and installs `imx294.dtbo` and
`imx492.dtbo`. It intentionally does not edit boot config.

Edit `/boot/firmware/config.txt` on Bookworm/RPi5, or `/boot/config.txt` on
older Raspberry Pi OS images. Disable camera autodetect and add the overlays
for your wiring. The current bench uses IMX492 on CAM0 and IMX294 on CAM1:

```ini
camera_auto_detect=0
dtoverlay=imx492,cam0,color-binned-modes
dtoverlay=imx294,quad-bayer-modes
```

Common overlay options:

- `cam0`: move that sensor overlay from CAM1 to CAM0.
- `always-on`: keep the camera regulator enabled for hardware debugging.
- `quad-bayer-modes`: expose the IMX294 full-resolution quad-Bayer modes.
- `color-binned-modes`: expose the IMX492 color-binned mode.
- `mono`: expose IMX492 monochrome media-bus codes and the mono 12-bit binned
  mode; this sets the driver `mono-mode` property.

Reboot after installation and check enumeration:

```bash
sudo reboot
rpicam-hello --list-cameras
dmesg | grep -E 'imx294|imx492|imx294_imx492'
```

## Manual Build

For quick local testing without DKMS:

```bash
make clean
make
dtc -Wno-interrupts_property -Wno-unit_address_vs_reg -@ \
  -I dts -O dtb -o imx294.dtbo imx294-overlay.dts
dtc -Wno-interrupts_property -Wno-unit_address_vs_reg -@ \
  -I dts -O dtb -o imx492.dtbo imx492-overlay.dts
```

Install the resulting `.ko` and `.dtbo` files manually only if you need to
test outside the DKMS flow.
