# syntax=docker/dockerfile:1
FROM docker.io/ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        g++

# /sdk  — bind-mounted from host: docker_ws/dev/mindrove/mindroveSDK/mindrovesdk-5.3.0-linux-x64/
# /src  — bind-mounted from host: docker_ws/dev/mindrove_cpp/
# Both are mounted at runtime via docker-compose volumes; build happens in entrypoint.

WORKDIR /build

CMD bash -c " \
    cmake /src \
      -DSDK_RELEASE_DIR=/sdk \
      -DCMAKE_INSTALL_PREFIX=/build \
    && cmake --build . --parallel \
    && cmake --install . \
    && LD_LIBRARY_PATH=/build/bin exec /build/bin/get_data --board-id 0 \
"
