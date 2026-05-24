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

.PHONY: build build-prosthesis build-segmentation build-segmentation-cpu build-segmentation-cuda build-jazzy-rviz rebuild dev dev-shell segmentation segmentation-cuda segmentation-cpu up up-prosthesis up-hw test shell down down-segmentation clean clean-volumes logs rviz rviz-kill rviz-openvins rviz-openvins-kill rviz-static rviz-static-kill rviz-twist-propagation rviz-twist-propagation-kill robotlab-connect robotlab-view robotlab-stop jetson-setup jetson-sync jetson-cameras jetson-cameras-stop jetson-cameras-logs jetson-list-cameras jetson-openvins jetson-openvins-stop jetson-openvins-logs jetson-imu-test-single jetson-imu-test-dual jetson-imu-test-stop jetson-imu-test-logs rviz-imu-test-single rviz-imu-test-dual rviz-imu-test-kill ros2-ethernet-shell ros2-listen-jetson ros2-pub-host ros2-topic-list ros2-node-list validate-segmentation validate-segmentation-config up-grasp-test down-grasp-test logs-grasp-test up-grasp-test-train up-emg-test-train test-static-grasp print-force build-emg-experiments emg-default emg-sklearn-imu emg-slew emg-sticky emg-naviflame emg-pipeline emg-validate

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

build-segmentation:
	cd $(COMPOSE_DIR) && SEGMENTATION_CPU_ONLY=1 $(COMPOSE) build segmentation

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
ifeq ($(CONTAINER_BACKEND),podman)
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA_PODMAN) --profile segmentation-cuda up -d segmentation-cuda
else
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA) --profile segmentation-cuda up -d segmentation-cuda
endif

segmentation-cpu:
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CPU) --profile segmentation-cpu up -d segmentation-cpu

# Legacy alias: starts the CUDA variant (preserves existing behavior)
segmentation: segmentation-cuda

down-segmentation:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile segmentation-cuda --profile segmentation-cpu down

# ── Run ────────────────────────────────────────────────────────────────────
up:
	cd $(COMPOSE_DIR) && $(COMPOSE) up -d prosthesis segmentation-cuda

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
ifeq ($(CONTAINER_BACKEND),podman)
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

# ── Trajectory Prediction RViz (twist_propagation visualisation) ────────────────
rviz-twist-propagation:
	@echo "Launching Trajectory Prediction RViz (twist_propagation visualisation)"
	@test -f rviz/twist_propagation.rviz || { echo "Missing rviz/twist_propagation.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	podman run --rm -d --name rviz-twist-propagation \
		--network host \
		--ipc host \
		--device /dev/dri \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/twist_propagation.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && timeout $(HOST_CONTAINER_LIFETIME) rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "Trajectory Prediction RViz started. Kill with: make rviz-twist-propagation-kill"

rviz-twist-propagation-kill:
	-podman kill rviz-twist-propagation 2>/dev/null
	-podman rm rviz-twist-propagation 2>/dev/null
	@echo "Trajectory Prediction RViz stopped."

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

# ── Grasp Test ──────────────────────────────────────────────────────────────
up-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test rm -f grasp_test 2>/dev/null || true
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test up -d grasp_test
	@echo ""
	@echo "grasp_test container started. Python packages will rebuild then the EMG grasp test launches."
	@echo "Follow progress: make logs-grasp-test"
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test logs -f grasp_test

down-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test down

logs-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test logs -f grasp_test

# ── Train + grasp test (interactive: collect → train → launch) ────────────────
# Runs in the foreground so the collect_data ENTER prompts reach your terminal.
# The container is removed automatically on Ctrl-C.
#
# Recording knobs (override on the command line):
#   EMG_REPS     — repetitions per gesture      (default: 3)
#   EMG_DURATION — recording duration per rep s (default: 5)
#
# Example:
#   make up-grasp-test-train EMG_REPS=5 EMG_DURATION=7

EMG_REPS     ?= 3
EMG_DURATION ?= 5

up-grasp-test-train:
	@mkdir -p $(CURDIR)/data $(CURDIR)/models
	@echo "═══════════════════════════════════════════════════════════"
	@echo "  EMG Train + Test"
	@echo "═══════════════════════════════════════════════════════════"
	@echo ""
	@echo "Phase 1–2: collect data + train model (interactive)"
	@echo "Phase 3:   live EMG grasp test (press ENTER after training to launch)"
	@echo ""
	$(DOCKER_CMD) run --rm -it --name grasp_test_train \
		--network host \
		--privileged \
		--ipc host \
		--userns=keep-id \
		-e DISPLAY=$${DISPLAY:-:0} \
		-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
		-e ROS_DOMAIN_ID=0 \
		-e EMG_REPS=$(EMG_REPS) \
		-e EMG_DURATION=$(EMG_DURATION) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		-v $(CURDIR)/src:/prosthesis_ws/src:rw \
		-v $(CURDIR)/config:/prosthesis_ws/config:rw \
		-v $(CURDIR)/rviz:/prosthesis_ws/rviz:rw \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:rw \
		-v $(CURDIR)/tests:/prosthesis_ws/tests:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-v $(CURDIR)/data:/prosthesis_ws/data:rw \
		-v $(CURDIR)/Makefile.workspace:/prosthesis_ws/Makefile:ro \
		-v prosthesis-build:/prosthesis_ws/build \
		-v prosthesis-install:/prosthesis_ws/install \
		-v prosthesis-log:/prosthesis_ws/log \
		$$(test -e /dev/ttyUSB0 && echo '--device /dev/ttyUSB0:/dev/ttyUSB0' || true) \
		$$(test -e /dev/ttyUSB1 && echo '--device /dev/ttyUSB1:/dev/ttyUSB1' || true) \
		prosthesis:latest \
		/bin/bash /prosthesis_ws/scripts/emg_train_and_test.sh
	@echo ""
	@printf "Press ENTER to launch the live EMG grasp test... "
	@read -r dummy
	@echo ""
	@echo "Launching live EMG grasp test..."
	@echo ""
	$(MAKE) up-grasp-test

up-emg-test-train:
	@mkdir -p $(CURDIR)/data $(CURDIR)/models
	@echo "  EMG Train + Terminal Inference"
	@echo ""
	@echo "Phase 1: collect data (interactive)"
	@echo "Phase 2: train model on the latest collected session"
	@echo "Phase 3: continue directly into live terminal inference"
	@echo ""
	$(DOCKER_CMD) run --rm -it --name emg_test_train \
		--network host \
		--privileged \
		--ipc host \
		--userns=keep-id \
		-e DISPLAY=$${DISPLAY:-:0} \
		-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
		-e ROS_DOMAIN_ID=0 \
		-e EMG_REPS=$(EMG_REPS) \
		-e EMG_DURATION=$(EMG_DURATION) \
		-e EMG_POST_TRAIN_MODE=classifier \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		-v $(CURDIR)/src:/prosthesis_ws/src:rw \
		-v $(CURDIR)/config:/prosthesis_ws/config:rw \
		-v $(CURDIR)/rviz:/prosthesis_ws/rviz:rw \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:rw \
		-v $(CURDIR)/tests:/prosthesis_ws/tests:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-v $(CURDIR)/data:/prosthesis_ws/data:rw \
		-v $(CURDIR)/Makefile.workspace:/prosthesis_ws/Makefile:ro \
		-v prosthesis-build:/prosthesis_ws/build \
		-v prosthesis-install:/prosthesis_ws/install \
		-v prosthesis-log:/prosthesis_ws/log \
		$$(test -e /dev/ttyUSB0 && echo '--device /dev/ttyUSB0:/dev/ttyUSB0' || true) \
		$$(test -e /dev/ttyUSB1 && echo '--device /dev/ttyUSB1:/dev/ttyUSB1' || true) \
		prosthesis:latest \
		/bin/bash /prosthesis_ws/scripts/emg_train_and_test.sh

# ── EMG Experiment Mode Launch Targets ───────────────────────────────────────
# Each target launches run_classifier with a pre-made experiment config.
#
# Default mode (no experiments — standard sklearn EMG-only):
#   make emg-default
#
# Quick experiment modes (all run in base prosthesis container except NaviFlame):
#   make emg-sklearn-imu    # EMG + IMU features with sklearn
#   make emg-slew           # Proportional slew limiting
#   make emg-sticky         # Sticky gesture hysteresis
#   make emg-naviflame      # NaviFlame backend (separate container, see below)
#
# Full pipeline with experiment config:
#   make emg-pipeline MODE=naviflame

EMG_CONFIG_DIR = config
EMG_MODELS_DIR = models

# ── NaviFlame container (Python 3.10 + TF 2.12, separate from ROS) ───────────
build-emg-experiments:
	podman build -f docker/Dockerfile.emg-experiments -t emg-experiments .

emg-naviflame: build-emg-experiments
	@echo "=== NaviFlame Backend ==="
	@test -f $(CURDIR)/$(EMG_CONFIG_DIR)/emg_experiment_config.yaml || \
		{ echo "Error: $(EMG_CONFIG_DIR)/emg_experiment_config.yaml not found."; exit 1; }
	podman run --rm -it \
		--network host \
		--userns=keep-id \
		-v $(CURDIR)/$(EMG_CONFIG_DIR):/prosthesis_ws/config:ro \
		-v $(CURDIR)/$(EMG_MODELS_DIR):/prosthesis_ws/models:ro \
		emg-experiments \
		--model-dir /prosthesis_ws/models \
		--config /prosthesis_ws/config/emg_experiment_config.yaml

# ── Baseline mode (runs in ROS container) ────────────────────────────────────
emg-default:
	@echo "=== EMG Classifier (default sklearn) ==="
	$(DOCKER_CMD) run --rm -it --name emg-default \
		--network host --privileged --ipc host --userns=keep-id \
		-e DISPLAY=$${DISPLAY:-:0} \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:rw \
		-v $(CURDIR)/config:/prosthesis_ws/config:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-v prosthesis-build:/prosthesis_ws/build \
		-v prosthesis-install:/prosthesis_ws/install \
		prosthesis:latest \
		python3 /prosthesis_ws/src/emg_bridge/scripts/run_classifier.py --model-dir /prosthesis_ws/models

# ── Experiment modes (all run in ROS container, toggled via --config) ────────
emg-sklearn-imu:
	@echo "=== EMG Classifier (sklearn + IMU) ==="
	$(DOCKER_CMD) run --rm -it --name emg-sklearn-imu \
		--network host --privileged --ipc host --userns=keep-id \
		-e DISPLAY=$${DISPLAY:-:0} \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:rw \
		-v $(CURDIR)/config:/prosthesis_ws/config:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-v prosthesis-build:/prosthesis_ws/build \
		-v prosthesis-install:/prosthesis_ws/install \
		prosthesis:latest \
		python3 /prosthesis_ws/src/emg_bridge/scripts/run_classifier.py \
			--model-dir /prosthesis_ws/models \
			--config /prosthesis_ws/config/emg_experiment_config_sklearn_imu.yaml

emg-slew:
	@echo "=== EMG Classifier (proportional slew limiting) ==="
	$(DOCKER_CMD) run --rm -it --name emg-slew \
		--network host --privileged --ipc host --userns=keep-id \
		-e DISPLAY=$${DISPLAY:-:0} \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:rw \
		-v $(CURDIR)/config:/prosthesis_ws/config:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-v prosthesis-build:/prosthesis_ws/build \
		-v prosthesis-install:/prosthesis_ws/install \
		prosthesis:latest \
		python3 /prosthesis_ws/src/emg_bridge/scripts/run_classifier.py \
			--model-dir /prosthesis_ws/models \
			--config /prosthesis_ws/config/emg_experiment_config_slew.yaml

emg-sticky:
	@echo "=== EMG Classifier (sticky gesture selection) ==="
	$(DOCKER_CMD) run --rm -it --name emg-sticky \
		--network host --privileged --ipc host --userns=keep-id \
		-e DISPLAY=$${DISPLAY:-:0} \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e ROS_DOMAIN_ID=0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:rw \
		-v $(CURDIR)/config:/prosthesis_ws/config:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-v prosthesis-build:/prosthesis_ws/build \
		-v prosthesis-install:/prosthesis_ws/install \
		prosthesis:latest \
		python3 /prosthesis_ws/src/emg_bridge/scripts/run_classifier.py \
			--model-dir /prosthesis_ws/models \
			--config /prosthesis_ws/config/emg_experiment_config_sticky.yaml

# Full pipeline launch with experiment config override
# Usage: make emg-pipeline MODE=naviflame
emg-pipeline:
	@echo "=== EMG Pipeline (mode: $(MODE)) ==="
	cd $(COMPOSE_DIR) && $(COMPOSE) run --rm --name emg-pipeline-$(MODE) \
		prosthesis \
		ros2 launch prosthesis_launch pipeline.launch.py \
			emg:=true \
			emg_config:=/prosthesis_ws/config/emg_experiment_config_$(MODE).yaml \
			mia_hand:=false wrist:=false camera:=false rviz:=false

# ── Hardware validation checklist ─────────────────────────────────────────────
# Requires: MindRove WiFi armband connected and trained models in models/
emg-validate:
	@echo "=== EMG Experiment Mode Validation ==="
	@echo ""
	@echo "1. Baseline (EMG-only sklearn):"
	@echo "   python scripts/run_classifier.py --model-dir models/"
	@echo "   Verify /emg/gesture_label, /emg/confidence, /emg/proportional publish."
	@echo ""
	@echo "2. IMU mode:"
	@echo "   python scripts/collect_data.py --include-imu --output-dir data/"
	@echo "   python scripts/train.py --classifier-backend sklearn_imu --data-dir data/"
	@echo "   python scripts/run_classifier.py --config config/emg_experiment_config.yaml"
	@echo "   Edit config to set classifier_backend: sklearn_imu + imu_features.enabled: true"
	@echo ""
	@echo "3. Proportional limiter:"
	@echo "   Set proportional_slew.enabled: true in config."
	@echo "   Verify /emg/proportional ramps smoothly (no sudden jumps)."
	@echo ""
	@echo "4. Sticky gestures:"
	@echo "   Set gesture_stability.enabled: true in config."
	@echo "   Induce brief ambiguous transitions — label should hold."
	@echo ""
	@echo "5. NaviFlame (separate container):"
	@echo "   Requires Python 3.10 + tensorflow==2.12.0."
	@echo "   Set classifier_backend: naviflame, naviflame.enabled: true."
	@echo ""
	@echo "Record exact configs, command lines, and observations."

print-force:
	@echo "=== MIA Hand Force Monitor ==="
	@echo "Hand device: $${MIA_PORT:-/dev/ttyUSB0}"
	@test -f scripts/print_force.sh || { echo "Missing scripts/print_force.sh"; exit 1; }
	-podman rm -f mia-force-print 2>/dev/null
	podman run --rm -it --name mia-force-print \
		--network host \
		--device $${MIA_PORT:-/dev/ttyUSB0}:/dev/ttyUSB0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:ro \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:ro \
		-v $(CURDIR)/config:/prosthesis_ws/config:ro \
		-e MIA_PORT=/dev/ttyUSB0 \
		prosthesis:latest \
		bash /prosthesis_ws/scripts/print_force.sh

test-static-grasp:
	@echo "=== MIA Hand Static Grasp Test ==="
	@echo "Hand device: $${MIA_PORT:-/dev/ttyUSB0}"
	@test -f scripts/static_grasp_test.sh || { echo "Missing scripts/static_grasp_test.sh"; exit 1; }
	@test -f config/static_grasp_test.yaml || { echo "Missing config/static_grasp_test.yaml"; exit 1; }
	-podman rm -f mia-static-grasp 2>/dev/null
	podman run --rm -it --name mia-static-grasp \
		--network host \
		--device $${MIA_PORT:-/dev/ttyUSB0}:/dev/ttyUSB0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:ro \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:ro \
		-v $(CURDIR)/config:/prosthesis_ws/config:ro \
		-e MIA_PORT=/dev/ttyUSB0 \
		-e GRASP_TEST_CONFIG=/prosthesis_ws/config/static_grasp_test.yaml \
		prosthesis:latest \
		python3 /prosthesis_ws/scripts/static_grasp_test.sh
