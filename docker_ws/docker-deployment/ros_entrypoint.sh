#!/bin/bash
# for use in debug mode: set -ex

# Creating enough /dev/ttyUSB nodes to allow Mia Hand plugging/unplugging during
# container running.

user_name="$(id -u -n)"
for port in {0..9}
do
  if ( ! [ -c /dev/ttyUSB${port} ] ); then
    sudo mknod /dev/ttyUSB${port} c 188 ${port}
    sudo chown ${user_name}:dialout /dev/ttyUSB${port}
  fi
done

exec "$@"