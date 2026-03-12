.PHONY: help build up down logs shell clean rebuild ros-build ros-test exec

# Auto-load .env and build compose file list based on LINUX and NVIDIA flags
-include .env
export

COMPOSE_FILES := -f docker-compose.yml
ifeq ($(LINUX),1)
  COMPOSE_FILES += -f docker-compose.linux.yml
endif
ifeq ($(NVIDIA),1)
  COMPOSE_FILES += -f docker-compose.nvidia.yml
endif

help:
	@echo "Available commands:"
	@echo "  make build          - Build the Docker image"
	@echo "  make up             - Start the container (reads LINUX/NVIDIA from .env)"
	@echo "  make down           - Stop the container"
	@echo "  make shell          - Open a shell in the running container"
	@echo "  make rebuild        - Rebuild the image and restart container"
	@echo "  make clean          - Remove containers, images, and build artifacts"
	@echo "  make ros-build      - Build ROS2 workspace (colcon build)"
	@echo "  make ros-test       - Run ROS2 tests (colcon test)"
	@echo "  make exec CMD=...   - Run commands in the container"

build:
	docker compose $(COMPOSE_FILES) build

up:
	docker rm -f mv_prosthesis_dev 2>/dev/null || true
	docker compose $(COMPOSE_FILES) up -d --force-recreate ros
	@echo "Container started. Use 'make shell' to enter."

down:
	docker compose $(COMPOSE_FILES) down

shell:
	docker compose $(COMPOSE_FILES) exec ros bash

logs:
	docker compose $(COMPOSE_FILES) logs -f ros

rebuild: down clean build up
	@echo "Rebuild complete!"

clean:
	docker compose $(COMPOSE_FILES) down -v
	docker rmi mv_prosthesis:latest 2>/dev/null || true

ros-build:
	docker compose $(COMPOSE_FILES) exec ros bash -c "cd /ros_container && colcon build --symlink-install"

ros-test:
	docker compose $(COMPOSE_FILES) exec ros bash -c "cd /ros_container && colcon test"

# Run arbitrary command in container
exec:
	docker compose $(COMPOSE_FILES) exec ros bash -c "$(CMD)"
