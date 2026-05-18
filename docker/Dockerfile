# Prosthesis main container
# Based on ROS Jazzy desktop — includes RViz, rqt, ros2-control, xacro, tf2-ros
FROM docker.io/osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
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

# Copy everything — colcon will only build packages in src/
COPY src/ src/
COPY config/ config/
COPY rviz/ rviz/
COPY scripts/ scripts/
COPY tests/ tests/

RUN chmod +x scripts/*.sh scripts/*.py 2>/dev/null || true
RUN chown -R ${USER_UID}:${USER_GID} tests/

# Install ROS dependencies and build
RUN . /opt/ros/jazzy/setup.sh \
    && apt-get update \
    && rosdep install --from-paths src/ --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Runtime ──────────────────────────────────────────────────────────────────
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV LD_LIBRARY_PATH="/prosthesis_ws/install/grasp_preshaping/lib:${LD_LIBRARY_PATH}"
ENV GRASP_CONFIG_PATH="/prosthesis_ws/config/grasp_preshaping.yaml"
ENV GRASP_PRESHAPING_HOME="/prosthesis_ws/src/grasp_preshaping"

USER prosthesis

# Source the workspace on shell entry
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/prosthesis/.bashrc \
    && echo "source /prosthesis_ws/install/setup.bash" >> /home/prosthesis/.bashrc

CMD ["/bin/bash"]
