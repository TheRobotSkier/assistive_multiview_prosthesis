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

.PHONY: build build-prosthesis build-segmentation build-segmentation-cuda build-segmentation-cpu build-jazzy-rviz rebuild dev dev-shell segmentation segmentation-cuda segmentation-cpu up up-prosthesis up-hw test shell down down-segmentation clean clean-volumes logs segmentation-status segmentation-logs rviz rviz-kill rviz-openvins rviz-openvins-kill rviz-static rviz-static-kill rviz-twist-propagation rviz-twist-propagation-kill robotlab-connect robotlab-view robotlab-stop timesync timesync-host timesync-check jetson-setup jetson-sync jetson-cameras jetson-cameras-stop jetson-cameras-logs jetson-list-cameras jetson-openvins jetson-openvins-stop jetson-openvins-logs jetson-imu-test-single jetson-imu-test-dual jetson-imu-test-stop jetson-imu-test-logs rviz-imu-test-single rviz-imu-test-dual rviz-imu-test-kill ros2-ethernet-shell ros2-listen-jetson ros2-pub-host ros2-topic-list ros2-node-list validate-segmentation validate-segmentation-config up-grasp-test down-grasp-test logs-grasp-test test-static-grasp emg-force-grasp emg-grasp-test print-force emg-infer

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
	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash

# Segmentation services (explicit backend variants)
# Use the appropriate compose override based on detected backend for CUDA GPU support.
segmentation-cuda:
ifeq ($(CONTAINER_BACKEND),podman)
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA_PODMAN) up -d segmentation-cuda
else
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA) --profile segmentation-cuda up -d segmentation-cuda
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
	cd $(COMPOSE_DIR) && $(COMPOSE) up -d prosthesis segmentation-cpu

up-prosthesis: dev

up-hw:
	cd $(COMPOSE_DIR) && $(COMPOSE) -f docker-compose.yml -f docker-compose.hw.yml up -d prosthesis segmentation-cpu

# ── Tonight host validation gates ────────────────────────────────────────────
# These run the in-container Makefile targets from the host checkout. They keep
# hardware disabled unless the target name explicitly says otherwise.

TONIGHT_TARGETS := tonight tonight-build tonight-clean tonight-raw-check tonight-imu-check tonight-tf tonight-tf-check tonight-fusion tonight-fusion-check tonight-segmentation-check tonight-twist-check tonight-grasp-check tonight-gates

.PHONY: $(TONIGHT_TARGETS)

tonight-segmentation-check tonight-grasp-check: segmentation

$(TONIGHT_TARGETS): dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec prosthesis /bin/bash -lc 'make $@'

# ── Launch proxies (from host) ─────────────────────────────────────────────
run: up-hw
	make timesync-check
	@DETECTED=$$(bash scripts/detect_usb_host.sh) && eval "$$DETECTED" && \
	echo "[host] Detected: MIA=$$DETECTED_MIA_PORT  WRIST=$$DETECTED_WRIST_PORT" && \
	cd $(COMPOSE_DIR) && $(COMPOSE) exec \
		-e MIA_SERIAL_PORT="$$DETECTED_MIA_PORT" \
		-e WRIST_SERIAL_PORT="$$DETECTED_WRIST_PORT" \
		prosthesis /bin/bash -lc 'make setup-usb' && \
	$(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make run'

camera-test: dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make camera-test'

collect-data: dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make collect-data'

train: dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make train'

emg-infer: dev
	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash -lc 'make run-classifier'

# ── Test ───────────────────────────────────────────────────────────────────
test:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis && $(COMPOSE) run --rm test

# ── Shell into running container ──────────────────────────────────────────
shell:
	cd $(COMPOSE_DIR) && $(COMPOSE) exec --user prosthesis prosthesis /bin/bash

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
	@echo "=== Weights ==="
	@if [ -f "${HOME}/prosthesis_data/weights/weights_exp14_14.pth" ]; then \
		ls -lh ${HOME}/prosthesis_data/weights/weights_exp14_14.pth | awk '{print "  " $$5, $$9}'; \
	else \
		echo "  Weights not found at ${HOME}/prosthesis_data/weights/"; \
	fi

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

# ── Clock sync between host and Jetson ────────────────────────────────────
# Prevents TF2 "extrapolation into the past" errors caused by clock skew.
#
# Two-phase sync:
#   1. One-shot SSH date sync (sets Jetson clock to host clock immediately)
#   2. Chrony NTP on both sides for ongoing drift correction (sub-ms precision)
#
# NOTE: On WSL2, w32time (Windows Time service) must be disabled first —
#   it holds UDP 123 and prevents chrony in WSL2 from binding.
#   Run from elevated PowerShell: Stop-Service w32time; Set-Service w32time -StartupType Disabled
#
# timesync         — one-shot sync + configure chrony on host + Jetson
# timesync-check   — compare clocks, show chrony status on both sides
# timesync-host    — configure chrony on host only

timesync: robotlab-connect
	@echo "=== Configuring chrony on host ==="
	@which chronyd >/dev/null 2>&1 || { echo "ERROR: chrony not installed on host. Run: sudo apt install -y chrony"; exit 1; }
	-sudo systemctl stop systemd-timesyncd 2>/dev/null || true
	-sudo systemctl disable systemd-timesyncd 2>/dev/null || true
	@sudo cp config/chrony-host.conf /etc/chrony/chrony.conf
	@grep -q 'SYNC_IN_CONTAINER="yes"' /etc/default/chrony 2>/dev/null || sudo sed -i 's/^SYNC_IN_CONTAINER=.*/SYNC_IN_CONTAINER="yes"/' /etc/default/chrony 2>/dev/null || echo 'SYNC_IN_CONTAINER="yes"' | sudo tee -a /etc/default/chrony > /dev/null
	@sudo systemctl restart chronyd 2>/dev/null || sudo systemctl restart chrony 2>/dev/null || { echo "WARNING: Could not restart chronyd on host"; }
	@echo "Host chrony configured and running."
	@echo ""
	@echo "=== One-shot clock sync (host -> Jetson) ==="
	@HOST_EPOCH="$$(date +%s.%N)" && \
		echo "Host time:     $$(date)" && \
		echo "Jetson before: $$(ssh $(JETSON_HOST) date)" && \
		ssh -T $(JETSON_HOST) "echo robotlab | sudo -S date -s @$${HOST_EPOCH}" && \
		echo "Jetson after:  $$(ssh $(JETSON_HOST) date)"
	@echo ""
	@echo "=== Configuring chrony on Jetson for ongoing NTP sync ==="
	@CHRONY_B64="$$(base64 -w0 config/chrony-jetson.conf)" && \
		ssh -T $(JETSON_HOST) "echo robotlab | sudo -S -v 2>/dev/null && \
			{ which chronyd >/dev/null 2>&1 || sudo apt install -y chrony; } && \
			{ sudo systemctl stop systemd-timesyncd 2>/dev/null || true; } && \
			{ sudo systemctl disable systemd-timesyncd 2>/dev/null || true; } && \
			echo '$$CHRONY_B64' | base64 -d | sudo tee /etc/chrony/chrony.conf > /dev/null && \
			{ sudo systemctl restart chronyd 2>/dev/null || sudo systemctl restart chrony 2>/dev/null || true; } && \
			echo 'Reconnecting WiFi (chrony Breaks: network-manager)...' && \
			sleep 2 && \
			sudo nmcli connection up eduroam 2>/dev/null || true"
	@echo ""
	@echo "Clock sync complete. Use 'make timesync-check' to verify NTP is active."

timesync-host:
	@echo "Configuring chrony on host only (Jetson not configured)..."
	@which chronyd >/dev/null 2>&1 || { echo "ERROR: chrony not installed on host. Install with: sudo apt install chrony"; exit 1; }
	-sudo systemctl stop systemd-timesyncd 2>/dev/null || true
	-sudo systemctl disable systemd-timesyncd 2>/dev/null || true
	@sudo cp config/chrony-host.conf /etc/chrony/chrony.conf
	@grep -q 'SYNC_IN_CONTAINER="yes"' /etc/default/chrony 2>/dev/null || sudo sed -i 's/^SYNC_IN_CONTAINER=.*/SYNC_IN_CONTAINER="yes"/' /etc/default/chrony 2>/dev/null || echo 'SYNC_IN_CONTAINER="yes"' | sudo tee -a /etc/default/chrony > /dev/null
	@sudo systemctl restart chronyd 2>/dev/null || sudo systemctl restart chrony 2>/dev/null || { echo "WARNING: Could not restart chronyd"; }
	@echo "Host chrony configured. Jetson can now sync via NTP to this machine."

timesync-check: robotlab-connect
	@echo "=== Clock comparison ==="
	@echo "Host time:   $$(date)"
	@echo "Jetson time: $$(ssh $(JETSON_HOST) date)"
	@echo ""
	@echo "=== Host chrony status ==="
	@chronyc sources 2>/dev/null || echo "  chronyc not available on host"
	@echo ""
	@chronyc tracking 2>/dev/null | grep -E "(Reference|Stratum|Last offset|RMS offset)" || echo "  chrony tracking not available on host"
	@echo ""
	@echo "=== Jetson chrony status ==="
	@ssh $(JETSON_HOST) 'chronyc sources 2>/dev/null || echo "  chronyc not available on Jetson"'
	@echo ""
	@ssh $(JETSON_HOST) 'chronyc tracking 2>/dev/null | grep -E "(Reference|Stratum|Last offset|RMS offset)" || echo "  chrony tracking not available on Jetson"'

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
	$(COMPOSE) rm -f grasp_test 2>/dev/null || true
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test up grasp_test

down-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test down

logs-grasp-test:
	cd $(COMPOSE_DIR) && $(COMPOSE) --profile grasp_test logs -f

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
	@mkdir -p build install log
	-podman rm -f mia-static-grasp 2>/dev/null
	podman run --rm -it --name mia-static-grasp \
		--network host \
		--userns keep-id \
		--device $${MIA_PORT:-/dev/ttyUSB0}:/dev/ttyUSB0 \
		-v $(CURDIR)/src:/prosthesis_ws/src:ro \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:ro \
		-v $(CURDIR)/config:/prosthesis_ws/config:ro \
		-v $(CURDIR)/build:/prosthesis_ws/build:rw \
		-v $(CURDIR)/install:/prosthesis_ws/install:rw \
		-v $(CURDIR)/log:/prosthesis_ws/log:rw \
		-e MIA_PORT=$${MIA_PORT:-/dev/ttyUSB0} \
		-e GRASP_TEST_CONFIG=/prosthesis_ws/config/static_grasp_test.yaml \
		-e GRASP_TEST_START_CONTROLLER=$${GRASP_TEST_START_CONTROLLER:-true} \
		-e GRASP_TEST_USE_MOCK_HARDWARE=$${GRASP_TEST_USE_MOCK_HARDWARE:-false} \
		-e GRASP_TEST_MAX_CLOSING_DURATION_S=$${GRASP_TEST_MAX_CLOSING_DURATION_S:-20} \
		-e GRASP_TEST_FORCE_THRESHOLD=$${GRASP_TEST_FORCE_THRESHOLD:-} \
		-e GRASP_TEST_FORCE_HOLD_TARGET=$${GRASP_TEST_FORCE_HOLD_TARGET:-} \
		-e GRASP_TEST_FORCE_HOLD_DURATION_S=$${GRASP_TEST_FORCE_HOLD_DURATION_S:-} \
		prosthesis:latest \
		bash -lc 'set -e; source /opt/ros/jazzy/setup.bash; if [ -f /prosthesis_ws/install/setup.bash ]; then source /prosthesis_ws/install/setup.bash; echo "Updating MIA hand packages for static grasp test..."; colcon build --packages-up-to mia_hand_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release; else echo "Building MIA hand packages for static grasp test..."; colcon build --packages-up-to mia_hand_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release; fi; source /prosthesis_ws/install/setup.bash; python3 /prosthesis_ws/scripts/static_grasp_test.sh'

emg-force-grasp: ## EMG force grasp + wrist: collect (if needed) → train → launch (1min auto-stop)
	@echo "=== EMG Force Grasp + Wrist Controller ==="
	@echo ""
	@echo "Env overrides (optional):"
	@echo "  EMG_DATA_DIR=$${EMG_DATA_DIR:-/prosthesis_ws/data}     Training data directory"
	@echo "  EMG_MODEL_DIR=$${EMG_MODEL_DIR:-/prosthesis_ws/models}  Model output directory"
	@echo "  MIA_PORT=$${MIA_PORT:-/dev/ttyUSB0}             Mia hand serial port"
	@echo "  WRIST_PORT=$${WRIST_PORT:-/dev/ttyUSB1}         Wrist Dynamixel port"
	@echo "  CONFIG_PATH=$${CONFIG_PATH:-tests/emg_grasp/emg_grasp_test.yaml}"
	@echo "  FORCE_RETRAIN=$${FORCE_RETRAIN:-false}          Re-train even if model exists"
	@echo "  LOG_LEVEL=$${LOG_LEVEL:-info}                   ROS 2 log level"
	@echo "  AUTO_KILL_S=$${AUTO_KILL_S:-60}                 Auto-stop after N seconds"
	@echo ""
	@test -f scripts/emg_force_grasp.sh || { echo "Missing scripts/emg_force_grasp.sh"; exit 1; }
	@mkdir -p build install log data models
	@# Map serial devices
	@DEVICE_ARGS=""; \
	if [ -e "$${MIA_PORT:-/dev/ttyUSB0}" ]; then \
		DEVICE_ARGS="--device $${MIA_PORT:-/dev/ttyUSB0}:$${MIA_PORT:-/dev/ttyUSB0}"; \
	fi; \
	if [ "$${WRIST_ENABLE:-true}" = "true" ] && [ -e "$${WRIST_PORT:-/dev/ttyUSB1}" ]; then \
		DEVICE_ARGS="$$DEVICE_ARGS --device $${WRIST_PORT:-/dev/ttyUSB1}:$${WRIST_PORT:-/dev/ttyUSB1}"; \
	fi; \
	if [ -z "$$DEVICE_ARGS" ]; then \
		echo "WARNING: No serial devices found. MOCK_HARDWARE=true by default."; \
		export MOCK_HARDWARE=true; \
	fi
	-podman rm -f emg-force-grasp 2>/dev/null
	podman run --rm -it --name emg-force-grasp \
		--network host \
		--userns keep-id \
		--group-add keep-groups \
		--device /dev/ttyUSB0:/dev/ttyUSB0 \
		--device /dev/ttyUSB1:/dev/ttyUSB1 \
		$$DEVICE_ARGS \
		-v $(CURDIR)/src:/prosthesis_ws/src:ro \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:ro \
		-v $(CURDIR)/config:/prosthesis_ws/config:ro \
		-v $(CURDIR)/tests:/prosthesis_ws/tests:ro \
		-v $(CURDIR)/build:/prosthesis_ws/build:rw \
		-v $(CURDIR)/install:/prosthesis_ws/install:rw \
		-v $(CURDIR)/log:/prosthesis_ws/log:rw \
		-v $(CURDIR)/data:/prosthesis_ws/data:rw \
		-v $(CURDIR)/models:/prosthesis_ws/models:rw \
		-e EMG_DATA_DIR="$${EMG_DATA_DIR:-/prosthesis_ws/data}" \
		-e EMG_MODEL_DIR="$${EMG_MODEL_DIR:-/prosthesis_ws/models}" \
		-e MIA_PORT="$${MIA_PORT:-/dev/ttyUSB0}" \
		-e WRIST_PORT="$${WRIST_PORT:-/dev/ttyUSB1}" \
		-e CONFIG_PATH="$${CONFIG_PATH:-/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml}" \
		-e WRIST_ENABLE="$${WRIST_ENABLE:-true}" \
		-e FORCE_RETRAIN="$${FORCE_RETRAIN:-false}" \
		-e MOCK_HARDWARE="$${MOCK_HARDWARE:-false}" \
		-e LOG_LEVEL="$${LOG_LEVEL:-info}" \
		-e AUTO_KILL_S="$${AUTO_KILL_S:-60}" \
		prosthesis:latest \
		bash /prosthesis_ws/scripts/emg_force_grasp.sh

emg-grasp-test: ## EMG-driven grasp test: collect → train → launch (set MOCK_HARDWARE=true for CI)
	@echo "=== EMG-Driven Grasp Test ==="
	@echo ""
	@echo "Env overrides (optional):"
	@echo "  EMG_DEVICE=$${EMG_DEVICE:-<auto-discover>}   MindRove board IP/host"
	@echo "  EMG_DATA_DIR=$${EMG_DATA_DIR:-/app/data}     Recording output directory"
	@echo "  EMG_MODEL_DIR=$${EMG_MODEL_DIR:-/app/models}  Trained model directory"
	@echo "  MIA_PORT=$${MIA_PORT:-/dev/ttyUSB0}           Mia hand serial port"
	@echo "  CONFIG_PATH=$${CONFIG_PATH:-tests/emg_grasp/emg_grasp_test.yaml}"
	@echo "  WRIST_ENABLE=$${WRIST_ENABLE:-false}          Enable wrist Dynamixel"
	@echo "  MOCK_HARDWARE=$${MOCK_HARDWARE:-false}        Skip collect/train, use mock HW"
	@echo ""
	@test -f scripts/emg_grasp_test.sh || { echo "Missing scripts/emg_grasp_test.sh"; exit 1; }
	@test -f $(CURDIR)/tests/emg_grasp/emg_grasp_test.yaml || { echo "Missing tests/emg_grasp/emg_grasp_test.yaml"; exit 1; }
	@# Determine serial device mapping
	@if [ "$${MOCK_HARDWARE:-false}" = "true" ]; then \
		echo "MOCK MODE: no physical device mapping needed"; \
		DEVICE_ARGS=""; \
	else \
		DEV="$${MIA_PORT:-/dev/ttyUSB0}"; \
		if [ ! -e "$$DEV" ]; then \
			echo "ERROR: Mia hand device not found at $$DEV"; \
			echo "  - Connect the USB cable" \
			echo "  - Or set MIA_PORT=/dev/ttyUSB1" \
			echo "  - Or use MOCK_HARDWARE=true"; \
			exit 1; \
		fi; \
		echo "Hand device: $$DEV"; \
		DEVICE_ARGS="--device $$DEV:$$DEV"; \
	fi
	-podman rm -f emg-grasp-test 2>/dev/null
	podman run --rm -it --name emg-grasp-test \
		--network host \
		$$DEVICE_ARGS \
		-v $(CURDIR)/src:/prosthesis_ws/src:ro \
		-v $(CURDIR)/scripts:/prosthesis_ws/scripts:ro \
		-v $(CURDIR)/config:/prosthesis_ws/config:ro \
		-v $(CURDIR)/tests:/prosthesis_ws/tests:ro \
		-e EMG_DEVICE="$${EMG_DEVICE:-}" \
		-e EMG_DATA_DIR="$${EMG_DATA_DIR:-/app/data}" \
		-e EMG_MODEL_DIR="$${EMG_MODEL_DIR:-/app/models}" \
		-e MIA_PORT="$${MIA_PORT:-/dev/ttyUSB0}" \
		-e CONFIG_PATH="$${CONFIG_PATH:-/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml}" \
		-e WRIST_ENABLE="$${WRIST_ENABLE:-false}" \
		-e MOCK_HARDWARE="$${MOCK_HARDWARE:-false}" \
		prosthesis:latest \
		bash /prosthesis_ws/scripts/emg_grasp_test.sh
