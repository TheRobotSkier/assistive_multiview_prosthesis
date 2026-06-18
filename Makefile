COMPOSE_DIR := $(CURDIR)/docker_ws/docker-deployment
PKG         := sensor_fusion_bringup

.PHONY: help build run run-log run-log-debug

help:
	@echo "Jetson (multiview_prosthesis-jetson_docker) commands:"
	@echo ""
	@echo "  make build            Rebuild overlay (sensor_fusion_bringup, symlink-install)"
	@echo "  make build PKG=ov_msckf   Rebuild a specific package"
	@echo "  make run              Launch dynamic_id2_arm_update_live pipeline"
	@echo "  make run-log          Same, with log capture to logs/"
	@echo "  make run-log-debug    Same, with debug logging + sysmon"

# Rebuild the ROS 2 overlay with --symlink-install so that future edits to
# launch files, scripts, and config are picked up automatically (no rebuild
# needed for those).  Must be run after changing Python scripts or launch files
# that were not present when the overlay was last built.
#
# Usage:
#   make build                      # rebuild sensor_fusion_bringup (default)
#   make build PKG=ov_msckf         # rebuild OpenVINS instead
#   make build PKG="sensor_fusion_bringup ov_msckf"  # rebuild both
build:
	@if [ ! -d $(COMPOSE_DIR) ]; then \
		echo "ERROR: $(COMPOSE_DIR) not found" >&2; \
		exit 1; \
	fi
	docker compose --project-directory $(COMPOSE_DIR) run --rm --name openvins_build realsense_camera \
		'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && \
		 MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 \
		 colcon --log-base log_overlay build --symlink-install \
		   --build-base build_overlay --install-base install_overlay \
		   --executor sequential --parallel-workers 1 \
		   --packages-select $(PKG)'

run:
	docker compose --project-directory $(CURDIR)/docker_ws/docker-deployment run --rm --name openvins_pc realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=true pointcloud_decimation_magnitude:=4 pointcloud_max_range_m:=0.0 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=false relay_pc_decimate:=false relay_hz:=5.0 image.downsample_factor:=2 marker_detection_rate_hz:=5.0'

run-log:
	mkdir -p $(CURDIR)/logs && \
	LOGFILE=$(CURDIR)/logs/run-jetson-$$(date +%Y%m%d_%H%M%S).txt && \
	echo "=== Logging to $$LOGFILE ===" && \
	docker compose --project-directory $(CURDIR)/docker_ws/docker-deployment run --rm --name openvins_pc realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=true pointcloud_decimation_magnitude:=4 pointcloud_max_range_m:=0.0 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=false relay_pc_decimate:=false relay_hz:=5.0 image.downsample_factor:=2 marker_detection_rate_hz:=5.0' 2>&1 | tee $$LOGFILE

run-log-debug:
	@if [ ! -d $(CURDIR)/docker_ws/docker-deployment ]; then \
		echo "ERROR: $(CURDIR)/docker_ws/docker-deployment not found" >&2; \
		exit 1; \
	fi
	mkdir -p $(CURDIR)/logs && \
	TS=$$(date +%Y%m%d_%H%M%S) && \
	LOGFILE=$(CURDIR)/logs/run-jetson-debug-$$TS.txt && \
	SYSMONFILE=$(CURDIR)/logs/run-jetson-debug-$$TS.jsonl && \
	echo "=== Logging to $$LOGFILE (debug) ===" && \
	echo "=== Sysmon to $$SYSMONFILE ===" && \
	{ python3 $(CURDIR)/scripts/sysmon-jetson.py --output $$SYSMONFILE & } && \
	SYSMON_PID=$$! && \
	docker compose --project-directory $(CURDIR)/docker_ws/docker-deployment run --rm --name openvins_pc \
		-e RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}][{time}][{name}]: {message}' \
		-e RCL_LOG_LEVEL=debug \
		realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=true pointcloud_decimation_magnitude:=4 pointcloud_max_range_m:=0.0 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=false relay_pc_decimate:=false relay_hz:=5.0 image.downsample_factor:=2 marker_detection_rate_hz:=5.0' 2>&1 | tee $$LOGFILE; \
	EXIT_CODE=$$?; \
	kill $$SYSMON_PID 2>/dev/null; wait $$SYSMON_PID 2>/dev/null; \
	exit $$EXIT_CODE
