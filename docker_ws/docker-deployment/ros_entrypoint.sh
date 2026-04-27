#!/bin/bash
# for use in debug mode: set -ex

# By default, do not create placeholder tty devices. Fake nodes can mask
# missing USB passthrough and lead to confusing serial errors at runtime.
create_tty_placeholders="${MIA_CREATE_TTY_PLACEHOLDERS:-true}"
user_name="$(id -u -n)"

if [ "${create_tty_placeholders}" = "true" ]; then
  for port in {0..9}
  do
    if ( ! [ -c /dev/ttyUSB${port} ] ); then
      sudo mknod /dev/ttyUSB${port} c 188 ${port}
      sudo chown ${user_name}:dialout /dev/ttyUSB${port}
    fi
    if ( ! [ -c /dev/ttyACM${port} ] ); then
      sudo mknod /dev/ttyACM${port} c 166 ${port}
      sudo chown ${user_name}:dialout /dev/ttyACM${port}
    fi
  done
else
  echo "[ros_entrypoint] MIA_CREATE_TTY_PLACEHOLDERS=false, skipping placeholder /dev/ttyUSB* and /dev/ttyACM* creation."
fi

echo "[ros_entrypoint] Visible serial devices:"
ls -la /dev/mia_hand /dev/wrist_motor /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || \
  echo "[ros_entrypoint] No matching serial devices are visible in this container."

exec "$@"