#!/bin/bash
# setup_usb_devices.sh
# Run this script on the host (WSL2 or Jetson) to configure USB device symlinks.
# This must be run with sudo.
#
# Usage:
#   sudo ./setup_usb_devices.sh
#
# It will:
#   1. List current USB devices to help you identify the Mia Hand and Dynamixel adapter
#   2. Prompt for vendor/product IDs
#   3. Install udev rules
#   4. Reload udev rules

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UDEV_DIR="${SCRIPT_DIR}/udev"

echo "=== USB Device Setup for Mia Hand + Wrist Motor ==="
echo ""
echo "Listing current USB devices:"
echo ""
lsusb 2>/dev/null || echo "(lsusb not available - install usbutils package)"
echo ""
echo "Listing current serial devices:"
echo ""
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(No ttyUSB/ttyACM devices found)"
echo ""
echo "-------------------------------------------------------"
echo "Please identify the Mia Hand and Dynamixel wrist motor"
echo "devices from the list above. You need the USB Vendor ID"
echo "and Product ID (4-digit hex) for each device."
echo ""
echo "If lsusb shows: Bus 001 Device 005: ID 0403:6001 FTDI"
echo "  Then Vendor ID = 0403, Product ID = 6001"
echo ""
echo "If you don't know the IDs yet, press Enter to skip."
echo "You can also use by-id paths which don't require udev rules."
echo "-------------------------------------------------------"
echo ""

read -p "Enter Mia Hand USB Vendor ID (e.g., 0403) or press Enter to skip: " MIA_VID
read -p "Enter Mia Hand USB Product ID (e.g., 6001) or press Enter to skip: " MIA_PID
read -p "Enter Wrist Motor USB Vendor ID (e.g., 10c4) or press Enter to skip: " DXL_VID
read -p "Enter Wrist Motor USB Product ID (e.g., ea60) or press Enter to skip: " DXL_PID

# Create Mia Hand udev rule
MIA_RULE="/etc/udev/rules.d/99-mia-hand.rules"
if [ -n "$MIA_VID" ] && [ -n "$MIA_PID" ]; then
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${MIA_VID}\", ATTRS{idProduct}==\"${MIA_PID}\", SYMLINK+=\"mia_hand\"" | sudo tee "$MIA_RULE" > /dev/null
    echo "Created $MIA_RULE"
else
    echo "Skipping Mia Hand udev rule (no vendor/product ID provided)"
    echo "You can set it up later by editing: ${UDEV_DIR}/99-mia-hand.rules"
fi

# Create Wrist Motor udev rule
DXL_RULE="/etc/udev/rules.d/99-wrist-motor.rules"
if [ -n "$DXL_VID" ] && [ -n "$DXL_PID" ]; then
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${DXL_VID}\", ATTRS{idProduct}==\"${DXL_PID}\", SYMLINK+=\"wrist_motor\"" | sudo tee "$DXL_RULE" > /dev/null
    echo "Created $DXL_RULE"
else
    echo "Skipping Wrist Motor udev rule (no vendor/product ID provided)"
    echo "You can set it up later by editing: ${UDEV_DIR}/99-wrist-motor.rules"
fi

# Reload udev rules
echo ""
echo "Reloading udev rules..."
sudo udevadm control --reload-rules 2>/dev/null || true
sudo udevadm trigger 2>/dev/null || true

echo ""
echo "Done. Checking for symlinks:"
ls -la /dev/mia_hand 2>/dev/null || echo "  /dev/mia_hand not found (device may not be plugged in)"
ls -la /dev/wrist_motor 2>/dev/null || echo "  /dev/wrist_motor not found (device may not be plugged in)"

echo ""
echo "=== WSL2 USB Passthrough Instructions ==="
echo "If running on WSL2, you also need to attach USB devices from Windows:"
echo ""
echo "  1. Install usbipd-win on Windows (PowerShell, admin):"
echo "     winget install usbipd"
echo ""
echo "  2. List devices on Windows (PowerShell, admin):"
echo "     usbipd list"
echo ""
echo "  3. Attach each device (PowerShell, admin):"
echo "     usbipd bind --busid <BUSID>"
echo "     usbipd attach --wsl --busid <BUSID>"
echo ""
echo "  4. Verify in WSL2:"
echo "     lsusb"
echo "     ls -la /dev/ttyUSB* /dev/ttyACM*"
echo ""
echo "=== Jetson Native Instructions ==="
echo "If running on Jetson Orin Nano, devices are directly accessible."
echo "Just ensure the udev rules are installed and replug the devices."
