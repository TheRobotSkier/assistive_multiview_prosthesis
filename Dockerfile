ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG MUJOCO_VERSION=3.6.0
ARG MUJOCO_PLATFORM=linux-x86_64

ENV ROS_DISTRO=${ROS_DISTRO}
ENV MUJOCO_DIR=/opt/mujoco
ENV LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH}
ENV PATH=${MUJOCO_DIR}/bin:${PATH}

# Install basic tools and ROS2 build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libglfw3-dev \
    neofetch \
    build-essential \
    curl \
    libclang-dev \
    python3-pip \
    python3-colcon-common-extensions \
    tar \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo Installation
RUN set -eux; \
    archive="mujoco-${MUJOCO_VERSION}-${MUJOCO_PLATFORM}.tar.gz"; \
    base_url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}"; \
    curl -fL "${base_url}/${archive}" -o "/tmp/${archive}"; \
    curl -fL "${base_url}/${archive}.sha256" -o "/tmp/${archive}.sha256"; \
    cd /tmp; \
    echo "$(cat ${archive}.sha256 | cut -d' ' -f1) ${archive}" > ${archive}.sha256; \
    sha256sum -c "${archive}.sha256"; \
    tar -xf "${archive}" -C /opt; \
    ln -sfn "/opt/mujoco-${MUJOCO_VERSION}" "${MUJOCO_DIR}"; \
    rm -f "/tmp/${archive}" "/tmp/${archive}.sha256"

# Rust Installation
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:$PATH

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path \
    && chmod -R a+w $RUSTUP_HOME $CARGO_HOME

# Python packages
RUN if pip3 install --help | grep -q -- '--break-system-packages'; then \
      pip3 install --break-system-packages --upgrade \
        colcon-cargo colcon-ros-bundle mujoco pin h5py \
        "numpy>=2.2,<2.3" "matplotlib>=3.9"; \
    else \
      pip3 install --upgrade \
        colcon-cargo colcon-ros-bundle mujoco pin h5py \
        "numpy>=2.2,<2.3" "matplotlib>=3.9"; \
    fi

# Ubuntu's system matplotlib may drop a namespace .pth that preloads /usr/lib
# mpl_toolkits ahead of pip's Matplotlib, which breaks 3D axes imports.
RUN rm -f /usr/lib/python3/dist-packages/matplotlib-*-nspkg.pth \
    && python3 - <<'PY'
from pathlib import Path
import site

for base in site.getsitepackages():
    if base.startswith('/usr/local'):
        p = Path(base) / 'mpl_toolkits' / '__init__.py'
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(
            'from pkgutil import extend_path\n__path__ = extend_path(__path__, __name__)\n',
            encoding='utf-8'
        )
        print(f'Wrote {p}')
        break
else:
    raise RuntimeError('Could not find /usr/local site-packages path')
PY

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joint-limits \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# # Install build dependencies
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     build-essential cmake libeigen3-dev liburdfdom-dev python3-dev \
#     && rm -rf /var/lib/apt/lists/*

# # Build pinocchio from source with casadi support
# RUN git clone https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio && \
#     cd /tmp/pinocchio && \
#     mkdir build && cd build && \
#     cmake .. -DBUILD_WITH_CASADI_SUPPORT=ON \
#              -DPYTHON_EXECUTABLE=$(which python3) \
#              -DBUILD_PYTHON_INTERFACE=ON \
#              -DBUILD_WITH_EXAMPLE_ROBOT_DATA_SUPPORT=OFF && \
#     make -j$(nproc) && \
#     make install && \
#     cd / && rm -rf /tmp/pinocchio

WORKDIR /laptop_ws

# Shell setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /laptop_ws/install/setup.bash ]; then source /laptop_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]