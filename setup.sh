#!/usr/bin/env bash

set -euo pipefail

PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH"

DRV_NAME=imx294_imx492
DRV_VERSION=0.0.1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="/usr/src/${DRV_NAME}-${DRV_VERSION}"

usage() {
	cat <<EOF
Usage: ./setup.sh

Build and install the merged IMX294/IMX492 driver with DKMS and install the
device-tree overlays. Edit /boot/firmware/config.txt or /boot/config.txt
manually for your camera wiring.
EOF
}

while [ "$#" -gt 0 ]; do
	case "$1" in
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "Unknown option: $1" >&2
			usage >&2
			exit 2
			;;
	esac
	shift
done

need_cmd() {
	if ! command -v "$1" >/dev/null 2>&1; then
		echo "Missing required command: $1" >&2
		echo "Install prerequisites with:" >&2
		echo "  sudo apt update" >&2
		echo "  sudo apt install build-essential dkms device-tree-compiler raspberrypi-kernel-headers git" >&2
		exit 1
	fi
}

install_source_tree() {
	echo "Installing DKMS source to ${SRC_DIR}"
	sudo rm -rf "$SRC_DIR"
	sudo mkdir -p "$SRC_DIR"
	sudo cp -a "$SCRIPT_DIR"/. "$SRC_DIR"/
	sudo rm -rf "$SRC_DIR/.git"
	sudo find "$SRC_DIR" -type f \( \
		-name '*.ko' -o \
		-name '*.o' -o \
		-name '*.mod' -o \
		-name '*.mod.c' -o \
		-name '.*.cmd' -o \
		-name 'Module.symvers' -o \
		-name 'modules.order' -o \
		-name '*.dtbo' \
	\) -delete
	sudo chown -R root:root "$SRC_DIR"
}

remove_old_dkms() {
	echo "Removing old DKMS installs if present"
	for old_name in \
		"$DRV_NAME" \
		imx294-imx492 \
		imx294-imx492-dkms \
		imx294 \
		imx294-dkms \
		imx492 \
		imx492-dkms
	do
		sudo dkms remove -m "$old_name" -v "$DRV_VERSION" --all >/dev/null 2>&1 || true
	done
}

build_install_dkms() {
	echo "Building and installing ${DRV_NAME}/${DRV_VERSION}"
	sudo dkms add -m "$DRV_NAME" -v "$DRV_VERSION"
	sudo dkms build -m "$DRV_NAME" -v "$DRV_VERSION"
	sudo dkms install -m "$DRV_NAME" -v "$DRV_VERSION"

	# Run explicitly as well as via DKMS POST_INSTALL so a manual reinstall
	# refreshes overlays even on DKMS versions that ignore POST_INSTALL.
	sudo sh "$SRC_DIR/dkms.postinst"
}

main() {
	need_cmd sudo
	need_cmd make
	need_cmd dkms
	need_cmd dtc

	if [ ! -d "/lib/modules/$(uname -r)/build" ]; then
		echo "Missing kernel headers for $(uname -r)." >&2
		echo "Install them with:" >&2
		echo "  sudo apt install raspberrypi-kernel-headers" >&2
		exit 1
	fi

	sudo modprobe -r imx294_imx492 >/dev/null 2>&1 || true
	sudo modprobe -r imx294 >/dev/null 2>&1 || true
	sudo modprobe -r imx492 >/dev/null 2>&1 || true

	remove_old_dkms
	install_source_tree
	build_install_dkms

	echo
	echo "Install complete. Reboot, then run:"
	echo "  rpicam-hello --list-cameras"
}

main "$@"
