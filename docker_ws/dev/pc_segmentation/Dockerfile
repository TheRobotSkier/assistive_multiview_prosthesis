# ROS 2 Jazzy + CUDA container for pointcloud segmentation.
# InterObject3D inference runs here alongside the ROS 2 segmentation node.
#
# Build: docker compose run --build --rm pc_segmentation
ARG PYTORCH="1.12.0"
ARG CUDA="11.3"
ARG CUDNN="8"
FROM pytorch/pytorch:${PYTORCH}-cuda${CUDA}-cudnn${CUDNN}-devel

ENV DEBIAN_FRONTEND=noninteractive

# ROS 2 Jazzy
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-sensor-msgs \
        ros-jazzy-sensor-msgs-py \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-pip \
        git ninja-build cmake build-essential libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

# MinkowskiEngine (sparse tensor backend for InterObject3D)
ENV MAX_JOBS=4
ENV TORCH_CUDA_ARCH_LIST="6.0 6.1 6.2 7.0 7.2 7.5 8.0 8.6"
RUN git clone --recursive https://github.com/NVIDIA/MinkowskiEngine /opt/MinkowskiEngine \
    && cd /opt/MinkowskiEngine \
    && python setup.py install --blas=openblas

WORKDIR /app
COPY . /app/

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && python /app/nodes/segmentation_ros_node.py"]
