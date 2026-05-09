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
