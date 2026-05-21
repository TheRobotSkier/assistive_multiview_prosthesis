# Prosthesis main container
# Based on ROS Jazzy desktop — includes RViz, rqt, ros2-control, xacro, tf2-ros
#
# Multi-stage build:
#   base        — system deps, Python deps, user setup, rosdep (no source/build)
#   production  — copies source + runs colcon build (used by test service)
#
# For development, the base image is used with source mounted as volumes.

# ── Base stage: dependencies only ────────────────────────────────────────────
FROM docker.io/osrf/ros:jazzy-desktop AS base

ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        chrony \
        libserial-dev \
        python3-pip \
        python3-scipy \
        python3-pytest \
        usbutils \
        ros-jazzy-cyclonedds \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-librealsense2 \
        ros-jazzy-realsense2-camera \
        ros-jazzy-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ──────────────────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --break-system-packages \
        dynamixel-sdk \
        requests

# ── User setup ───────────────────────────────────────────────────────────────
# The osrf/ros:jazzy-desktop image already has a 'ubuntu' user with UID 1000.
# We reuse that user by renaming it instead of creating a conflicting one.
ARG USER_UID=1000
ARG USER_GID=1000

RUN if id -u ${USER_UID} >/dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd ${USER_UID} | cut -d: -f1) && \
        EXISTING_GROUP=$(getent group ${USER_GID} | cut -d: -f1) && \
        groupmod -n prosthesis "$EXISTING_GROUP" 2>/dev/null || true && \
        usermod -l prosthesis -d /home/prosthesis -m "$EXISTING_USER" 2>/dev/null || true; \
    else \
        groupadd -g ${USER_GID} prosthesis 2>/dev/null || true && \
        useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash prosthesis; \
    fi \
    && usermod -aG dialout prosthesis 2>/dev/null || true \
    && echo "prosthesis ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/prosthesis

# ── Workspace ────────────────────────────────────────────────────────────────
WORKDIR /prosthesis_ws

# Install ROS dependencies from package.xml files (source needed for rosdep).
# This layer is rebuilt when package.xml files change — rare compared to code changes.
COPY src/ src/
RUN . /opt/ros/jazzy/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src/ --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Remove the source copy — development mounts it as a volume.
RUN rm -rf /prosthesis_ws/src

# ── Runtime ──────────────────────────────────────────────────────────────────
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV LD_LIBRARY_PATH="/prosthesis_ws/install/grasp_preshaping/lib:${LD_LIBRARY_PATH}"
ENV GRASP_CONFIG_PATH="/prosthesis_ws/config/grasp_preshaping.yaml"
ENV GRASP_PRESHAPING_HOME="/prosthesis_ws/src/grasp_preshaping"

USER prosthesis

# Source the workspace on shell entry (install/setup.bash may not exist yet
# on first run with empty volumes — that's fine, it will after `make build`).
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/prosthesis/.bashrc \
    && echo 'if [ -f /prosthesis_ws/install/setup.bash ]; then source /prosthesis_ws/install/setup.bash; fi' >> /home/prosthesis/.bashrc

# Auto-build on first shell entry if install/ is empty (fresh named volumes).
RUN echo 'if [ ! -f /prosthesis_ws/install/setup.bash ] && [ -d /prosthesis_ws/src ]; then' >> /home/prosthesis/.bashrc \
    && echo '    echo "First run — building workspace...";' >> /home/prosthesis/.bashrc \
    && echo '    cd /prosthesis_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release;' >> /home/prosthesis/.bashrc \
    && echo '    source /prosthesis_ws/install/setup.bash;' >> /home/prosthesis/.bashrc \
    && echo 'fi' >> /home/prosthesis/.bashrc

# Disable ROS2 CLI daemon — it hangs in rootless podman with --network host
# because the XML-RPC localhost connection between daemon subprocess and
# client times out. Must run as root to modify /opt/ros files.
USER root
RUN sed -i 's/use_daemon = not getattr(args, '\''no_daemon'\'', False)/use_daemon = False  # patched: daemon hangs in rootless podman/' \
    /opt/ros/jazzy/lib/python3.12/site-packages/ros2cli/node/strategy.py
USER prosthesis

CMD ["/bin/bash"]

# ── Production stage: copies source for test service ────────────────────────
# colcon build is NOT run here — it happens at runtime inside the container
# where userns/keep-id gives the prosthesis user correct write permissions.
FROM base AS production

USER root

# Copy everything — test service will build at runtime
COPY src/ src/
COPY config/ config/
COPY rviz/ rviz/
COPY scripts/ scripts/
COPY tests/ tests/

RUN chmod +x scripts/*.sh scripts/*.py 2>/dev/null || true

USER prosthesis
