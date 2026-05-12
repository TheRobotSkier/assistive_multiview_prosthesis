COMPOSE := docker_ws/docker-deployment/docker-compose.yml
SUDO := echo robotlab | sudo -S

.PHONY: up cameras kill build

build:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) build miahand_ros2

up:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) run -T --rm miahand_ros2 bash -c 'source /miahand_ws/install/setup.bash 2>/dev/null || true; cd /miahand_ws; colcon build --packages-select sensor_fusion_bringup 2>\0461; source install/setup.bash; ros2 launch sensor_fusion_bringup robotlab_bringup.launch.py'

kill:
	$(SUDO) docker compose -f $(COMPOSE) down -t 1 2>/dev/null || true
	$(SUDO) docker kill $$( $(SUDO) docker ps -q ) 2>/dev/null || true

cameras:
	$(SUDO) docker rm -f cameras_test 2>/dev/null || true
	$(SUDO) docker run -d \
		--name cameras_test \
		--network host \
		--ipc host \
		--runtime nvidia \
		--privileged \
		--group-add video \
		--group-add plugdev \
		--group-add dialout \
		--group-add i2c \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/docker_ws:/miahand_ws/src \
		-v $(CURDIR)/docker_ws/docker-deployment/cyclonedds_robotlab.xml:/tmp/cyclonedds_peer.xml:ro \
		-v /dev:/dev \
		-v /run/udev:/run/udev:ro \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		docker-deployment-miahand_ros2 \
		bash -c 'source /miahand_ws/install/setup.bash 2>/dev/null; source /opt/ros/jazzy/setup.bash; ros2 launch sensor_fusion_bringup dual_d435i.launch.py'
