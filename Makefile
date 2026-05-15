# Prosthesis — common commands
# Works with podman-compose or docker compose (auto-detected)

COMPOSE_DIR := docker
DOCKER_CMD ?= docker

# Auto-detect compose command
PODMAN_COMPOSE := $(shell which podman-compose 2>/dev/null)
DOCKER_COMPOSE := $(shell which docker 2>/dev/null)

ifdef PODMAN_COMPOSE
  COMPOSE := podman-compose
else
  COMPOSE := docker compose
endif

.PHONY: build build-prosthesis build-segmentation build-jazzy-rviz rebuild up up-hw up-grasp-test down-grasp-test logs-grasp-test up-digital-twin down-digital-twin logs-digital-twin test-digital-twin test shell clean logs logs-cameras ros2-ethernet-shell ros2-listen-jetson ros2-pub-host ros2-topic-list ros2-node-list rviz rviz-kill robotlab-view robotlab-stop

# ── Build ──────────────────────────────────────────────────────────────────
build:
	cd $(COMPOSE_DIR) && $(COMPOSE) build

build-prosthesis:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis

build-segmentation:
	cd $(COMPOSE_DIR) && $(COMPOSE) build segmentation

build-jazzy-rviz:
	$(DOCKER_CMD) build -f docker/Dockerfile.jazzy-rviz -t localhost/ros2-jazzy-rviz:latest .

rebuild:
	cd $(COMPOSE_DIR) && $(COMPOSE) build --no-cache

# ── Run ────────────────────────────────────────────────────────────────────
up:
	cd $(COMPOSE_DIR) && $(COMPOSE) up -d

up-hw:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile hardware up -d

# ── Grasp Test ──────────────────────────────────────────────────────────────
up-grasp-test:
	$(COMPOSE) rm -f grasp_test 2>/dev/null || true
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test up grasp_test -d

down-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test down

logs-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test logs -f

# ── Digital Twin ────────────────────────────────────────────────────────────
up-digital-twin:
	$(COMPOSE) rm -f digital_twin 2>/dev/null || true
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile digital_twin up digital_twin -d

down-digital-twin:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile digital_twin down

logs-digital-twin:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile digital_twin logs -f

test-digital-twin:
	@echo "=== Pointcloud Health Test ==="
	podman exec digital_twin bash /prosthesis_ws/scripts/test_pointcloud_health.sh || \
		podman exec grasp_test bash /prosthesis_ws/scripts/test_pointcloud_health.sh || \
		echo "No running container found — start one first with 'make up-digital-twin' or 'make up-grasp-test'"

# ── Test ───────────────────────────────────────────────────────────────────
test:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis && $(COMPOSE) run --rm test

# ── Shell into running container ──────────────────────────────────────────
shell:
	cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash

# ── Cleanup ───────────────────────────────────────────────────────────────
down:
	cd $(COMPOSE_DIR) && $(COMPOSE) down

clean:
	cd $(COMPOSE_DIR) && $(COMPOSE) down --rmi local --volumes

logs:
	cd $(COMPOSE_DIR) && $(COMPOSE) logs -f

# ── Camera logs (D435i — hardware profile) ────────────────────────────────
logs-cameras:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile hardware logs -f prosthesis-hw

# ── Robotlab RViz (view Jetson camera data on host via Ethernet) ──────
ros2-ethernet-shell:
	./scripts/ros2_ethernet_hello_host.sh shell

ros2-listen-jetson:
	./scripts/ros2_ethernet_hello_host.sh listen-jetson

ros2-pub-host:
	./scripts/ros2_ethernet_hello_host.sh pub-host

ros2-topic-list:
	./scripts/ros2_ethernet_hello_host.sh topic-list

ros2-node-list:
	./scripts/ros2_ethernet_hello_host.sh node-list

rviz:
	@echo "Launching RViz on host with Docker (connects to robotlab via Ethernet ROS network)"
	@test -f rviz/phase2_dual_openvins_head_preview.rviz || { echo "Missing rviz/phase2_dual_openvins_head_preview.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	./scripts/ros2_ethernet_hello_host.sh rviz

rviz-kill:
	-$(DOCKER_CMD) rm -f ros2-jazzy-host-rviz 2>/dev/null
	@echo "RViz stopped."

robotlab-view: rviz
	@echo "Robotlab view ready. Jetson pointclouds streaming to RViz over Ethernet."

robotlab-stop: rviz-kill
	@echo "Robotlab view stopped."
