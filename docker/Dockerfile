# Prosthesis main container
# Based on ROS Jazzy desktop — includes RViz, rqt, ros2-control, xacro, tf2-ros
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        libserial-dev \
        python3-pip \
        usbutils \
        ros-jazzy-cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ──────────────────────────────────────────────────────
RUN python3 -m pip install --no-cache-dir --break-system-packages \
        dynamixel-sdk \
        requests

# ── User setup ───────────────────────────────────────────────────────────────
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd -g ${USER_GID} prosthesis || true \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash prosthesis \
    && usermod -aG dialout prosthesis

# ── Workspace ────────────────────────────────────────────────────────────────
WORKDIR /prosthesis_ws

# Copy everything — colcon will only build packages in src/
COPY src/ src/
COPY config/ config/
COPY launch/ launch/
COPY rviz/ rviz/

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

USER prosthesis

# Source the workspace on shell entry
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/prosthesis/.bashrc \
    && echo "source /prosthesis_ws/install/setup.bash" >> /home/prosthesis/.bashrc

ENTRYPOINT ["/bin/bash"]
