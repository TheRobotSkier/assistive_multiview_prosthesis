#!/usr/bin/env bash
# Prosthesis container entrypoint
# Fixes ownership of named Docker volumes (build/, install/, log/) which are
# created as root:root by Docker.  Without this, colcon build fails with
# PermissionError when trying to write log directories.
set -e

VOLUME_DIRS=(/prosthesis_ws/build /prosthesis_ws/install /prosthesis_ws/log)

for dir in "${VOLUME_DIRS[@]}"; do
    if [ -d "$dir" ] && [ "$(stat -c '%U' "$dir")" = "root" ]; then
        echo "[entrypoint] Fixing ownership of $dir -> prosthesis:prosthesis"
        chown -R prosthesis:prosthesis "$dir"
    fi
done

# Drop root privileges — run as the prosthesis user
exec gosu prosthesis "$@"
