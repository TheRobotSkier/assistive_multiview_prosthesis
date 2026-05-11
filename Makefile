# Robotlab Jetson Orin Nano — easy bringup commands
# Usage: make build | make up | make shell | make kill

COMPOSE := docker_ws/docker-deployment/docker-compose.yml
SUDO := echo robotlab | sudo -S

.PHONY: build up shell kill clean rebuild

build:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) build miahand_ros2

up:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) run --rm miahand_ros2 bash -c '\
		source /miahand_ws/install/setup.bash 2>/dev/null || true; \
		cd /miahand_ws && colcon build --packages-select sensor_fusion_bringup 2>&1; \
		source install/setup.bash; \
		ros2 launch sensor_fusion_bringup robotlab_bringup.launch.py'

shell:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) run --rm miahand_ros2 bash

kill:
	$(SUDO) docker kill $$( $(SUDO) docker ps -q ) 2>/dev/null || true

rebuild: build kill up

clean:
	$(SUDO) docker system prune -af
