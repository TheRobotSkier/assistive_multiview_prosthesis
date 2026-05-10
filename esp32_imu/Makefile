# ESP32 Dual IMU Firmware Build
# ==============================
# Requires: podman (or docker)
#
# Targets:
#   make build       Build firmware in Docker (→ build/esp32_dual_imu.bin)
#   make flash       Build + flash to ESP32 via USB
#   make monitor     Build + flash + serial monitor
#   make clean       Remove build artifacts
#
# Config: edit config.yaml to change pins, rates, offsets.

FIRMWARE_DIR  := firmware
BUILD_DIR     := $(FIRMWARE_DIR)/build
DOCKER_IMAGE  := esp32-imu-builder
SERIAL_PORT   ?= /dev/ttyUSB0

.PHONY: build flash monitor clean docker-image

# ── Build in Docker ─────────────────────────────────────────────────────
docker-image:
	podman build -f docker/Dockerfile -t $(DOCKER_IMAGE) .

build: docker-image
	podman run --rm -v $(PWD)/$(FIRMWARE_DIR):/project $(DOCKER_IMAGE) idf.py build

# ── Flash directly from host (requires esptool.py) ─────────────────────
flash: build
	python3 -m esptool --chip esp32 --port $(SERIAL_PORT) --baud 921600 \
		--before default_reset --after hard_reset write_flash \
		-z --flash_mode dio --flash_freq 40m --flash_size detect \
		0x1000 $(BUILD_DIR)/bootloader/bootloader.bin \
		0x8000 $(BUILD_DIR)/partition_table/partition-table.bin \
		0x10000 $(BUILD_DIR)/esp32_dual_imu.bin

# ── Flash via ESP-IDF in Docker ────────────────────────────────────────
flash-docker: build
	podman run --rm --device $(SERIAL_PORT):$(SERIAL_PORT) \
		-v $(PWD)/$(FIRMWARE_DIR):/project \
		$(DOCKER_IMAGE) idf.py -p $(SERIAL_PORT) flash

# ── Serial monitor ─────────────────────────────────────────────────────
monitor:
	podman run --rm -it --device $(SERIAL_PORT):$(SERIAL_PORT) \
		-v $(PWD)/$(FIRMWARE_DIR):/project \
		$(DOCKER_IMAGE) idf.py -p $(SERIAL_PORT) monitor

# ── Clean ──────────────────────────────────────────────────────────────
clean:
	rm -rf $(BUILD_DIR)
