# Robotlab Jetson Orin Nano — easy bringup commands
# Usage: make up | make kill | make build | make shell

COMPOSE := docker_ws/docker-deployment/docker-compose.yml
SUDO := echo robotlab | sudo -S

.PHONY: up kill build shell clean

build:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) build miahand_ros2

up:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) up -d discovery 2>&1
	sleep 2
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) run -T --rm 		-e ROS_DISCOVERY_SERVER=127.0.0.1:11811 		miahand_ros2 bash -c 'source /miahand_ws/install/setup.bash 2>/dev/null || true; cd /miahand_ws && colcon build --packages-select sensor_fusion_bringup 2>&1; source install/setup.bash; ros2 launch sensor_fusion_bringup robotlab_bringup.launch.py'

shell:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) run --rm miahand_ros2 bash

kill:
	$(SUDO) docker compose -f $(COMPOSE) down -t 1 2>/dev/null || true
	$(SUDO) docker kill $$( $(SUDO) docker ps -q ) 2>/dev/null || true

clean:
	$(SUDO) docker system prune -af
