# Prosthesis — common commands
# Works with podman-compose or docker compose (auto-detected)

COMPOSE_DIR := docker

# Auto-detect compose command
PODMAN_COMPOSE := $(shell which podman-compose 2>/dev/null)
DOCKER_COMPOSE := $(shell which docker 2>/dev/null)

ifdef PODMAN_COMPOSE
  COMPOSE := podman-compose
else
  COMPOSE := docker compose
endif

.PHONY: build build-prosthesis build-segmentation rebuild up up-hw up-grasp-test down-grasp-test logs-grasp-test up-digital-twin down-digital-twin logs-digital-twin test-digital-twin test shell clean logs logs-cameras rviz rviz-kill rviz-openvins rviz-openvins-kill rviz-static rviz-static-kill robotlab-connect robotlab-view robotlab-stop jetson-setup jetson-sync jetson-cameras jetson-cameras-stop jetson-cameras-logs jetson-list-cameras jetson-openvins jetson-openvins-stop jetson-openvins-logs jetson-imu-test-single jetson-imu-test-dual jetson-imu-test-stop jetson-imu-test-logs rviz-imu-test-single rviz-imu-test-dual rviz-imu-test-kill

# ── Build ──────────────────────────────────────────────────────────────────
build:
	cd $(COMPOSE_DIR) && $(COMPOSE) build

build-prosthesis:
	cd $(COMPOSE_DIR) && $(COMPOSE) build prosthesis

build-segmentation:
	cd $(COMPOSE_DIR) && $(COMPOSE) build segmentation

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
rviz:
	@echo "Launching RViz on host (connects to robotlab via Ethernet ROS network)"
	@test -f rviz/robotlab_cameras.rviz || { echo "Missing rviz/robotlab_cameras.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	podman run --rm -d --name rviz-robotlab \
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
		-v $(CURDIR)/rviz/robotlab_cameras.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "RViz container started (rviz-robotlab). Kill with: make rviz-kill"

rviz-kill:
	-podman kill rviz-robotlab 2>/dev/null
	-podman rm rviz-robotlab 2>/dev/null
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
		--device /dev/dri \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/robotlab_cameras_static_tf.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz' 2>&1 &
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
		  ros2 run tf2_ros static_transform_publisher \
		    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
		    --frame-id world --child-frame-id d435i_head_depth_optical_frame & \
		  ros2 run tf2_ros static_transform_publisher \
		    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
		    --frame-id world --child-frame-id d435i_arm_depth_optical_frame & \
		  wait'
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
		--device /dev/dri \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/phase2_dual_openvins.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz' 2>&1 &
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
		--device /dev/dri \
		--userns=keep-id \
		-e DISPLAY=$(DISPLAY) \
		-e XAUTHORITY=/tmp/.xauth \
		-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		-e CYCLONEDDS_URI=/tmp/cyclonedds_peer.xml \
		-e ROS_DOMAIN_ID=0 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v $(XAUTHORITY):/tmp/.xauth:ro \
		-v $(CURDIR)/rviz/imu_test_single.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "IMU test RViz (single cam) started. Kill with: make rviz-imu-test-kill"

rviz-imu-test-dual:
	@test -f rviz/imu_test_dual.rviz || { echo "Missing rviz/imu_test_dual.rviz"; exit 1; }
	@test -f config/cyclonedds_peer.xml || { echo "Missing config/cyclonedds_peer.xml"; exit 1; }
	xhost +
	podman run --rm -d --name rviz-imu-test \
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
		-v $(CURDIR)/rviz/imu_test_dual.rviz:/rviz_config.rviz:ro \
		-v $(CURDIR)/config/cyclonedds_peer.xml:/tmp/cyclonedds_peer.xml:ro \
		localhost/rviz-robotlab \
		bash -c 'source /opt/ros/jazzy/setup.bash && rviz2 -d /rviz_config.rviz' 2>&1 &
	@sleep 3
	@echo "IMU test RViz (dual cam) started. Kill with: make rviz-imu-test-kill"

rviz-imu-test-kill:
	-podman kill rviz-imu-test 2>/dev/null
	-podman rm rviz-imu-test 2>/dev/null
	@echo "IMU test RViz stopped."
