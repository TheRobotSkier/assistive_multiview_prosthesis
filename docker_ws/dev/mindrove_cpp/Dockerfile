FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    && rm -rf /var/lib/apt/lists/*

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
