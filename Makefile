# Prosthesis — common commands
#
# Container backend selection:
#   Set CONTAINER_BACKEND=docker or CONTAINER_BACKEND=podman to choose explicitly.
#   Default: auto-detect podman-compose first, then docker compose.
#
# Segmentation backends:
#   make build-segmentation-cpu    # CPU-only image (no CUDA dependencies)
#   make build-segmentation-cuda   # CUDA image with GPU runtime support
#   make segmentation-cpu          # run CPU segmentation service
#   make segmentation-cuda         # run CUDA segmentation service (requires GPU)
#   make down-segmentation         # stop any segmentation service
#
# Docker Compose GPU runtime is enabled via compose override for the CUDA service.
# Podman GPU runtime uses --device nvidia.com/gpu=all via a compose override.
#
# See docs/segmentation-backends.md for full platform and workflow docs.

COMPOSE_DIR := docker
DOCKER_CMD ?= podman

# ── Explicit backend selection ───────────────────────────────────────────
CONTAINER_BACKEND ?=

ifeq ($(CONTAINER_BACKEND),docker)
  COMPOSE := docker compose
  DOCKER_CMD := docker
  $(shell docker version >/dev/null 2>&1 || { echo "ERROR: CONTAINER_BACKEND=docker but 'docker' is not available"; exit 1; })
else ifeq ($(CONTAINER_BACKEND),podman)
  COMPOSE := podman-compose
  DOCKER_CMD := podman
  $(shell podman-compose version >/dev/null 2>&1 || { echo "ERROR: CONTAINER_BACKEND=podman but 'podman-compose' is not available"; exit 1; })
else
  # Auto-detect
  PODMAN_COMPOSE := $(shell which podman-compose 2>/dev/null)
  DOCKER := $(shell which docker 2>/dev/null)
  ifdef PODMAN_COMPOSE
    COMPOSE := podman-compose
    DOCKER_CMD := podman
  else ifdef DOCKER
    COMPOSE := docker compose
    DOCKER_CMD := docker
  else
    $(error No container backend found. Install docker or podman-compose, or set CONTAINER_BACKEND explicitly.)
  endif
endif

# Container lifetime limits (seconds). Adjust here to change all host containers.
# 1800 = 30 minutes
HOST_CONTAINER_LIFETIME := 1800

# Compose files for segmentation variants
COMPOSE_SEGMENTATION_CPU := -f docker-compose.yml
COMPOSE_SEGMENTATION_CUDA := -f docker-compose.yml -f docker-compose.segmentation.cuda.yml
COMPOSE_SEGMENTATION_CUDA_PODMAN := -f docker-compose.yml -f docker-compose.segmentation.podman-gpu.yml

# Select the correct GPU runtime compose override based on the detected backend.
# DOCKER_CMD is always set correctly (by explicit selection or auto-detect),
# unlike CONTAINER_BACKEND which is only set when explicitly provided.
ifeq ($(DOCKER_CMD),podman)
  COMPOSE_CUDA_RUNTIME := $(COMPOSE_SEGMENTATION_CUDA_PODMAN)
else
  COMPOSE_CUDA_RUNTIME := $(COMPOSE_SEGMENTATION_CUDA)
endif

.PHONY: build build-prosthesis build-segmentation-cuda build-segmentation-cpu build-jazzy-rviz rebuild dev dev-shell segmentation segmentation-cuda segmentation-cpu up up-prosthesis up-hw test shell down down-segmentation clean clean-volumes logs segmentation-status segmentation-logs rviz rviz-kill rviz-openvins rviz-openvins-kill rviz-static rviz-static-kill robotlab-connect robotlab-view robotlab-stop timesync timesync-host timesync-check jetson-setup jetson-sync jetson-cameras jetson-cameras-stop jetson-cameras-logs jetson-list-cameras jetson-openvins jetson-openvins-stop jetson-openvins-logs jetson-imu-test-single jetson-imu-test-dual jetson-imu-test-stop jetson-imu-test-logs rviz-imu-test-single rviz-imu-test-dual rviz-imu-test-kill ros2-ethernet-shell ros2-listen-jetson ros2-pub-host ros2-topic-list ros2-node-list validate-segmentation validate-segmentation-config

# ── Build ──────────────────────────────────────────────────────────────────
build:
	cd $(COMPOSE_DIR) && $(COMPOSE) build

build-prosthesis:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis

# Compose service names: segmentation-cuda and segmentation-cpu
build-segmentation-cuda:
	cd $(COMPOSE_DIR) && $(COMPOSE) build segmentation-cuda

build-segmentation-cpu:
	cd $(COMPOSE_DIR) && $(COMPOSE) build segmentation-cpu

build-jazzy-rviz:
	$(DOCKER_CMD) build -f docker/Dockerfile.jazzy-rviz -t localhost/ros2-jazzy-rviz:latest .

rebuild:
	cd $(COMPOSE_DIR) && $(COMPOSE) build --no-cache

# ── Development ────────────────────────────────────────────────────────────
# Primary workflow: make dev → make shell → (inside container) make build

dev:
	cd $(COMPOSE_DIR) && $(COMPOSE) up -d prosthesis

dev-shell: dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash

# Segmentation services (explicit backend variants)
# Use the appropriate compose override based on detected backend for CUDA GPU support.
segmentation-cuda:
ifeq ($(DOCKER_CMD),podman)
	@test -f /var/run/cdi/nvidia.yaml || { echo "Regenerating NVIDIA CDI spec..."; sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml; }
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_CUDA_RUNTIME) up -d segmentation-cuda
else
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_CUDA_RUNTIME) --profile segmentation-cuda up -d segmentation-cuda
endif

segmentation-cpu:
ifeq ($(DOCKER_CMD),podman)
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CPU) up -d segmentation-cpu
else
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CPU) --profile segmentation-cpu up -d segmentation-cpu
endif

# Legacy alias: starts the CUDA variant (preserves existing behavior)
segmentation: segmentation-cuda

down-segmentation:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile segmentation-cuda --profile segmentation-cpu down

# ── Run ────────────────────────────────────────────────────────────────────
up:
ifeq ($(DOCKER_CMD),podman)
	@test -f /var/run/cdi/nvidia.yaml || { echo "Regenerating NVIDIA CDI spec..."; sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml; }
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_CUDA_RUNTIME) up -d prosthesis segmentation-cuda
else
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_CUDA_RUNTIME) --profile segmentation-cuda up -d prosthesis segmentation-cuda
endif

up-prosthesis: dev

up-hw:
	cd $(COMPOSE_DIR) && $(COMPOSE) -f docker-compose.yml -f docker-compose.hw.yml up -d prosthesis

# ── Tonight host validation gates ────────────────────────────────────────────
# These run the in-container Makefile targets from the host checkout. They keep
# hardware disabled unless the target name explicitly says otherwise.

TONIGHT_TARGETS := tonight tonight-build tonight-clean tonight-raw-check tonight-imu-check tonight-tf tonight-tf-check tonight-fusion tonight-fusion-check tonight-segmentation-check tonight-twist-check tonight-grasp-check tonight-gates

.PHONY: $(TONIGHT_TARGETS)

tonight-segmentation-check tonight-grasp-check: segmentation

$(TONIGHT_TARGETS): dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash -lc 'make $@'

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

clean-volumes:
	-$(DOCKER_CMD) volume rm prosthesis-build prosthesis-install prosthesis-log segmentation-weights 2>/dev/null || true
	@echo "Named volumes removed. Next 'make dev' will trigger a fresh build."

logs:
	cd $(COMPOSE_DIR) && $(COMPOSE) logs -f

# ── Segmentation diagnostics ──────────────────────────────────────────────
segmentation-status:
	@echo "=== Segmentation container status ==="
	@$(DOCKER_CMD) ps -a --filter name=segmentation --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null || echo "No segmentation containers found."
	@echo ""
	@echo "=== Health check ==="
	@curl -sf http://127.0.0.1:5678/health && echo "" || echo "Inference server NOT reachable on port 5678"
	@echo ""
	@echo "=== Weights volume ==="
	@$(DOCKER_CMD) volume inspect segmentation-weights --format '{{.Mountpoint}} ({{.CreatedAt}})' 2>/dev/null || echo "Volume 'segmentation-weights' not found."

segmentation-logs:
	@$(DOCKER_CMD) logs --tail 100 -f $$( $(DOCKER_CMD) ps -a --filter name=segmentation --format "{{.Names}}" | head -1 ) 2>/dev/null || echo "No segmentation container found."

# ── Validation ────────────────────────────────────────────────────────────
# Backend matrix validation for segmentation container config and health.

validate-segmentation-config:
	@echo "=== Validating segmentation compose config ==="
	@echo "--- CPU config ---"
	@grep -q "profiles:" docker/docker-compose.yml && grep -q "segmentation-cpu" docker/docker-compose.yml || { echo "FAIL: segmentation-cpu profile missing"; exit 1; }
	@cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CPU) --profile segmentation-cpu config >/dev/null || { echo "FAIL: CPU compose config invalid"; exit 1; }
	@echo "CPU config OK"
	@echo "--- CUDA config (Docker) ---"
	@grep -q "profiles:" docker/docker-compose.yml && grep -q "segmentation-cuda" docker/docker-compose.yml || { echo "FAIL: segmentation-cuda profile missing"; exit 1; }
	@cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA) --profile segmentation-cuda config >/dev/null || { echo "FAIL: CUDA compose config invalid"; exit 1; }
	@echo "CUDA config OK"
ifeq ($(DOCKER_CMD),podman)
	@echo "--- CUDA config (Podman) ---"
	@cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA_PODMAN) --profile segmentation-cuda config >/dev/null || { echo "FAIL: Podman CUDA compose config invalid"; exit 1; }
	@echo "Podman CUDA config OK"
endif
	@echo "All compose config validations passed."

validate-segmentation: validate-segmentation-config
	@echo "=== Segmentation backend matrix validation ==="
	@echo "Build validation (dry-run):"
	$(MAKE) -n build-segmentation-cpu >/dev/null && echo "  build-segmentation-cpu: OK"
	$(MAKE) -n build-segmentation-cuda >/dev/null && echo "  build-segmentation-cuda: OK"
	@echo "Run validation (dry-run):"
	$(MAKE) -n segmentation-cpu >/dev/null && echo "  segmentation-cpu: OK"
	$(MAKE) -n segmentation-cuda >/dev/null && echo "  segmentation-cuda: OK"
	@echo "All backend matrix validations passed."

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
	# Use helper script which detects docker/podman and configures the container
	./scripts/ros2_ethernet_hello_host.sh rviz

rviz-kill:
	-$(DOCKER_CMD) rm -f ros2-jazzy-host-rviz 2>/dev/null
	@echo "RViz stopped."

# ── Robotlab RViz + static TF (both cameras at world origin) ─────────
rviz-static:
	@echo "Starting static TF publisher (both cameras → world origin)"
	@test -f rviz/robotlab_cameras_static_tf.rviz || { echo "Missing rviz/robotlab_cameras_static_tf.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	-podman rm -f static-tf-robotlab -t 1 2>/dev/null
	podman run --rm -d --name rviz-robotlab \
		--network host \
		--ipc host \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/robotlab_cameras_static_tf.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && timeout $(HOST_CONTAINER_LIFETIME) rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "RViz started. Kill both with: make rviz-static-kill"
	podman run --rm -d --name static-tf-robotlab \
		--network host \
		--ipc host \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && \
		  timeout $(HOST_CONTAINER_LIFETIME) bash -c "\
		    ros2 run tf2_ros static_transform_publisher \
		      --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
		      --frame-id world --child-frame-id d435i_head_depth_optical_frame & \
		    ros2 run tf2_ros static_transform_publisher \
		      --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
		      --frame-id world --child-frame-id d435i_arm_depth_optical_frame & \
		    wait"'
	@echo "Static TF container started (static-tf-robotlab)"

rviz-static-kill:
	-podman kill rviz-robotlab 2>/dev/null
	-podman rm rviz-robotlab 2>/dev/null
	-podman kill static-tf-robotlab 2>/dev/null
	-podman rm static-tf-robotlab 2>/dev/null
	@echo "RViz and static TF publisher stopped."

robotlab-view: rviz
	@echo "Robotlab view ready. Jetson pointclouds streaming to RViz over Ethernet."

robotlab-stop: rviz-kill
	@echo "Robotlab view stopped."

# ── Robotlab ethernet connection ──────────────────────────────────────────
# Ensures the USB-to-ethernet adapter is up and the Jetson is reachable.

ROBOTLAB_CONNECT_SCRIPT := scripts/robotlab_connect.sh

robotlab-connect:
	@test -f $(ROBOTLAB_CONNECT_SCRIPT) || { echo "Missing $(ROBOTLAB_CONNECT_SCRIPT)"; exit 1; }
	@$(ROBOTLAB_CONNECT_SCRIPT)

# ── Clock sync (chrony) between host and Jetson ──────────────────────────
# Prevents TF2 "extrapolation into the past" errors caused by clock skew.
# Requires: chrony installed on both machines, SSH access to Jetson.
#
# timesync         — configure and start chrony on both sides
# timesync-check   — verify clocks are in sync
# timesync-host    — configure chrony on host only (no Jetson access needed)

timesync: robotlab-connect
	@echo "Configuring chrony on host..."
	@which chronyd >/dev/null 2>&1 || { echo "ERROR: chrony not installed on host. Install with: sudo apt install chrony"; exit 1; }
	sudo cp config/chrony-host.conf /etc/chrony/chrony.conf
	sudo systemctl restart chronyd 2>/dev/null || sudo systemctl restart chrony 2>/dev/null || { echo "WARNING: Could not restart chronyd on host"; }
	@echo "Configuring chrony on Jetson..."
	ssh $(JETSON_HOST) 'which chronyd >/dev/null 2>&1 || (echo robotlab | sudo -S apt install -y chrony); echo robotlab | sudo -S tee /etc/chrony/chrony.conf > /dev/null' < config/chrony-jetson.conf
	ssh $(JETSON_HOST) 'echo robotlab | sudo -S systemctl restart chronyd 2>/dev/null || echo robotlab | sudo -S systemctl restart chrony 2>/dev/null || echo "WARNING: Could not restart chronyd on Jetson"'
	@echo "Chrony configured on both sides. Use 'make timesync-check' to verify."

timesync-host:
	@echo "Configuring chrony on host only (Jetson not configured)..."
	@which chronyd >/dev/null 2>&1 || { echo "ERROR: chrony not installed on host. Install with: sudo apt install chrony"; exit 1; }
	sudo cp config/chrony-host.conf /etc/chrony/chrony.conf
	sudo systemctl restart chronyd 2>/dev/null || sudo systemctl restart chrony 2>/dev/null || { echo "WARNING: Could not restart chronyd"; }
	@echo "Host chrony configured. The Jetson will sync to this machine if it has chrony with server 10.42.0.1."

timesync-check: robotlab-connect
	@echo "Host chrony sources:"
	@chronyc sources 2>/dev/null || echo "chronyc not available on host"
	@echo ""
	@echo "Jetson chrony sources:"
	@ssh $(JETSON_HOST) 'chronyc sources 2>/dev/null || echo "chronyc not available on Jetson"'
	@echo ""
	@echo "Clock offset (host → Jetson):"
	@ssh $(JETSON_HOST) 'chronyc tracking 2>/dev/null | grep "Last offset" || echo "chrony tracking not available"'

# ── Jetson deploy (git-push based sync over Ethernet) ────────────────────
# JETSON_HOST must be reachable via SSH (see ~/.ssh/config for 'robotlab').
# The Jetson pulls from a bare repo here via the post-receive hook.

JETSON_HOST       := robotlab
JETSON_DEPLOY_DIR := /home/robotlab/multiview_prosthesis
JETSON_BARE_REPO  := /home/robotlab/multiview_prosthesis.git
JETSON_BRANCH     := full_test_implementation

# One-time setup: creates bare repo + checkout hook on Jetson, adds git remote.
jetson-setup: robotlab-connect
	@echo "Setting up git deploy repo on Jetson (one-time)..."
	ssh $(JETSON_HOST) 'mkdir -p $(JETSON_DEPLOY_DIR) && git init --bare $(JETSON_BARE_REPO)'
	ssh $(JETSON_HOST) 'printf "#!/bin/bash\nGIT_WORK_TREE=$(JETSON_DEPLOY_DIR) git --git-dir=$(JETSON_BARE_REPO) checkout -f $(JETSON_BRANCH)\n" > $(JETSON_BARE_REPO)/hooks/post-receive && chmod +x $(JETSON_BARE_REPO)/hooks/post-receive'
	git remote add jetson $(JETSON_HOST):$(JETSON_BARE_REPO) 2>/dev/null || git remote set-url jetson $(JETSON_HOST):$(JETSON_BARE_REPO)
	git push jetson $(JETSON_BRANCH)
	@echo "Jetson deploy ready. Use 'make jetson-sync' to push future changes."

# Push committed changes on this branch to the Jetson (triggers checkout).
jetson-sync: robotlab-connect
	git push jetson $(JETSON_BRANCH)

# Sync → start cameras on Jetson → start RViz locally.
# Commit your changes before running this.
jetson-cameras: jetson-sync
	ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make cameras"
	$(MAKE) rviz-static

jetson-cameras-stop: robotlab-connect
	-ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make cameras-stop"
	$(MAKE) rviz-static-kill

jetson-cameras-logs: robotlab-connect
	ssh $(JETSON_HOST) "echo robotlab | sudo -S docker logs -f cameras_test"

jetson-list-cameras: robotlab-connect
	ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make list-cameras"

# ── Jetson OpenVINS (cameras + VIO containers + host RViz) ───────────────────
# Syncs the repo, starts both Jetson containers, and opens the Phase 2 RViz.
# Requires the overlay to already be at /home/robotlab/openvins_overlay/install_overlay/
# on the Jetson (rsynced separately — it is not in git).
jetson-openvins: jetson-sync
	ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make cameras && make openvins"
	$(MAKE) rviz-openvins

jetson-openvins-stop: robotlab-connect
	-ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make openvins-stop && make cameras-stop"
	$(MAKE) rviz-openvins-kill

jetson-openvins-logs: robotlab-connect
	ssh $(JETSON_HOST) "echo robotlab | sudo -S docker logs -f openvins"

# ── Phase 2 RViz (OpenVINS TF + pointclouds) ─────────────────────────────────
rviz-openvins:
	@echo "Launching Phase 2 RViz (OpenVINS marker_map frame)"
	@test -f rviz/phase2_dual_openvins.rviz || { echo "Missing rviz/phase2_dual_openvins.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	podman run --rm -d --name rviz-openvins \
		--network host \
		--ipc host \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/phase2_dual_openvins.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && timeout $(HOST_CONTAINER_LIFETIME) rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "Phase 2 RViz started (rviz-openvins). Kill with: make rviz-openvins-kill"

rviz-openvins-kill:
	-podman kill rviz-openvins 2>/dev/null
	-podman rm rviz-openvins 2>/dev/null
	@echo "Phase 2 RViz stopped."

# ── Jetson IMU dead reckoning test ────────────────────────────────────────────
# Use jetson-imu-test-single or jetson-imu-test-dual depending on how many
# cameras are connected. The Jetson container is the same in both cases;
# only the RViz config differs.

jetson-imu-test-single: jetson-sync
	ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make imu-test"
	$(MAKE) rviz-imu-test-single

jetson-imu-test-dual: jetson-sync
	ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make imu-test"
	$(MAKE) rviz-imu-test-dual

jetson-imu-test-stop: robotlab-connect
	-ssh $(JETSON_HOST) "cd $(JETSON_DEPLOY_DIR)/jetson && make imu-test-stop"
	$(MAKE) rviz-imu-test-kill

jetson-imu-test-logs: robotlab-connect
	ssh $(JETSON_HOST) "echo robotlab | sudo -S docker logs -f imu_test"

# ── IMU test RViz (local PC) ──────────────────────────────────────────────────
rviz-imu-test-single:
	@test -f rviz/imu_test_single.rviz || { echo "Missing rviz/imu_test_single.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	podman run --rm -d --name rviz-imu-test \
		--network host \
		--ipc host \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/imu_test_single.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && timeout $(HOST_CONTAINER_LIFETIME) rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "IMU test RViz (single cam) started. Kill with: make rviz-imu-test-kill"

rviz-imu-test-dual:
	@test -f rviz/imu_test_dual.rviz || { echo "Missing rviz/imu_test_dual.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	podman run --rm -d --name rviz-imu-test \
		--network host \
		--ipc host \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=all \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/imu_test_dual.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && timeout $(HOST_CONTAINER_LIFETIME) rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "IMU test RViz (dual cam) started. Kill with: make rviz-imu-test-kill"

rviz-imu-test-kill:
	-podman kill rviz-imu-test 2>/dev/null
	-podman rm rviz-imu-test 2>/dev/null
	@echo "IMU test RViz stopped."
