# IMX294/IMX492 Shared Driver

This directory builds one kernel module, `imx294_imx492.ko`, from one shared
V4L2 I2C sensor driver source: `imx294_imx492.c`.

The module advertises exact device-tree compatibles for both sensors:

- `sony,imx294`
- `sony,imx492`

Variant data selects the IMX294 or IMX492 mode tables, media-bus formats, and
streaming sequence policy behind the shared `imx29x` implementation. The common
core owns register I/O, controls, format negotiation, stream start/stop,
runtime PM, endpoint validation, and subdev registration.

Non-standard modes stay gated by dtoverlay properties:

- `quad-bayer-modes` exposes the experimental IMX294 quad-Bayer full-resolution
  12-bit modes. These still use standard Bayer media-bus codes because Linux
  has no standard 4x4 quad-Bayer code.
- `color-binned-modes` exposes the IMX492 color binned mode, which is useful
  for geometry/timing testing but is expected to collapse color information.
- `mono-mode` exposes IMX492 Y10/Y12 formats for monochrome hardware or testing.

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
`imx492.dtbo`.

Edit `/boot/firmware/config.txt` on Bookworm/RPi5, or `/boot/config.txt` on
older Raspberry Pi OS images. Disable camera autodetect and add the overlays
for your wiring. The current bench uses IMX492 on CAM0 and IMX294 on CAM1:

```ini
camera_auto_detect=0
dtoverlay=imx492,cam0,color-binned-modes
dtoverlay=imx294,quad-bayer-modes
```

Reboot after installation and check enumeration:

```bash
sudo reboot
rpicam-hello --list-cameras
dmesg | grep -E 'imx294|imx492|imx294_imx492'
```
