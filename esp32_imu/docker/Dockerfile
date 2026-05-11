# ESP-IDF build environment for ESP32 dual IMU firmware.
# Usage:
#   podman build -f esp32_imu/docker/Dockerfile -t esp32-imu-builder .
#   podman run --rm -v $(pwd)/esp32_imu/firmware:/project \
#       esp32-imu-builder idf.py build
FROM docker.io/espressif/idf:release-v5.4

WORKDIR /project
ENTRYPOINT ["/opt/esp/entrypoint.sh"]
CMD ["idf.py", "build"]
