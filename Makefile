run:
	docker compose --project-directory $(CURDIR)/docker_ws/docker-deployment run --rm --name openvins_pc realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=false pointcloud_max_range_m:=0.0 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=true relay_pc_decimate:=true relay_hz:=10.0 image.downsample_factor:=2'

run-log:
	mkdir -p $(CURDIR)/logs && \
	LOGFILE=$(CURDIR)/logs/run-jetson-$$(date +%Y%m%d_%H%M%S).txt && \
	echo "=== Logging to $$LOGFILE ===" && \
	docker compose --project-directory $(CURDIR)/docker_ws/docker-deployment run --rm --name openvins_pc realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=false pointcloud_max_range_m:=0.0 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=true relay_pc_decimate:=true relay_hz:=10.0 image.downsample_factor:=2' 2>&1 | tee $$LOGFILE

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
		realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py enable_pointclouds:=true enable_marker_map_pointclouds:=false pointcloud_decimation_enable:=false pointcloud_max_range_m:=0.0 start_preview:=false start_rviz:=false enable_pointcloud_neon_fix:=true relay_pc_decimate:=true relay_hz:=10.0 image.downsample_factor:=2' 2>&1 | tee $$LOGFILE; \
	EXIT_CODE=$$?; \
	kill $$SYSMON_PID 2>/dev/null; wait $$SYSMON_PID 2>/dev/null; \
	exit $$EXIT_CODE
