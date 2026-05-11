COMPOSE := docker_ws/docker-deployment/docker-compose.yml
SUDO := echo robotlab | sudo -S

.PHONY: up kill build

build:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) build miahand_ros2

up:
	cd $(CURDIR) && $(SUDO) docker compose -f $(COMPOSE) run -T --rm -v /tmp/start_bringup.sh:/start_bringup.sh:ro miahand_ros2 /start_bringup.sh

kill:
	$(SUDO) docker compose -f $(COMPOSE) down -t 1 2>/dev/null || true
	$(SUDO) docker kill $$( $(SUDO) docker ps -q ) 2>/dev/null || true
