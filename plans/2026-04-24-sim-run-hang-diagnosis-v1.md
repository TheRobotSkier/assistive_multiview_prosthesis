# Diagnosis: sim-run Hangs After Rebuild (MuJoCo Interactive Window Never Opens)

## Objective

Diagnose and fix the issue where `sim-run` hangs after a container rebuild — the MuJoCo interactive simulation window never appears, and the process blocks indefinitely.

---

## Root Cause Analysis

### What the log shows

The `podman run` command in your log shows the container starts, the entrypoint runs, `colcon build` succeeds, and then the `ros2 launch` command is invoked. The process then **blocks forever** — you had to `^C` to kill it, and the traceback confirms the podman-compose `subprocess.wait()` was still pending.

### Primary suspect: `podman compose` vs `podman-compose` (two different tools)

Your `.zshrc` aliases use **`podman compose`** (the native podman v4+ compose plugin, a Go binary):

```
alias sim-run='podman compose -f ~/multiview_prosthesis/docker_ws/docker-deployment/docker-compose.yml run --rm mujoco_interactive'
```

But the log output says:

```
>>>> Executing external compose provider "/usr/bin/podman-compose". Please refer to the documentation for details. <<<<
podman-compose version: 1.0.6
```

**This means your system is falling back to the Python-based `podman-compose` 1.0.6 instead of using the native `podman compose` plugin.** This is a critical clue. The native `podman compose` and the external `podman-compose` Python script handle TTY, interactive, and `--rm` flags differently. The Python `podman-compose` 1.0.6 has known issues with:

1. **TTY passthrough**: It may not properly forward the TTY to the container, which means GLFW cannot open an X11 window.
2. **`--rm` + interactive**: The container cleanup and interactive session handling differs from the native plugin.

### Why this might have changed recently

If you recently added services to `docker-compose.yml`, it's possible that:
- A `podman-compose` cache or config was regenerated
- A system update changed the `podman-compose` package or its priority
- The `podman compose` plugin became unavailable or was uninstalled, causing the fallback

### Secondary suspect: X11 display forwarding

The container needs `DISPLAY=:0` and access to `/tmp/.X11-unix` to render the MuJoCo GLFW window. Looking at the compose file:

- `DISPLAY: $DISPLAY` — this is evaluated on the **host** at compose-parse time
- Volume: `/tmp/.X11-unix:/tmp/.X11-unix` — present
- `QT_X11_NO_MITSHM: 1` — present

If `$DISPLAY` is empty when `podman-compose` evaluates the YAML, the container gets `DISPLAY=` (empty), and GLFW's `glfwCreateWindow()` will fail or hang. The `podman-compose` Python tool may not handle environment variable interpolation the same way as the native plugin.

### Tertiary suspect: `--tty` and `--entrypoint` interaction

The log shows podman-compose passes `--entrypoint ["/ros_entrypoint.sh"]` but the compose file has `entrypoint: /ros_entrypoint.sh` (on the base `miahand_ros2` service). The `mujoco_interactive` service extends `miahand_ros2`. The `command` field in compose is passed as arguments to the entrypoint. If `podman-compose` 1.0.6 mishandles the entrypoint/command combination, the bash command may not execute correctly, or the interactive TTY may not be attached, causing the MuJoCo render thread to block waiting for a display that's not connected.

---

## Implementation Plan

- [ ] **Step 1. Verify which compose provider is actually being used.** Run `podman compose version` and `podman-compose --version` separately to confirm which is installed and which is active. If `podman compose` (native plugin) returns an error, that confirms the fallback to the Python script.
- [ ] **Step 2. Ensure the native `podman compose` plugin is installed and used.** If it's missing, install it (`sudo dnf install podman-compose` or download the plugin binary for your distro). The native plugin handles TTY, env vars, and `--rm` correctly. Alternatively, verify that `/usr/bin/podman-compose` is not shadowing the native plugin.
- [ ] **Step 3. Test with explicit DISPLAY.** Run `echo $DISPLAY` on the host before `sim-run`. If it's empty or not `:0`, that's the problem. Try: `DISPLAY=:0 sim-run` to force it.
- [ ] **Step 4. Test with `podman run` directly (bypass compose).** Run the container manually with the same flags to isolate whether the issue is compose or the container itself:
  ```
  podman run --rm -it \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e MUJOCO_OBJECT=sphere \
    -e MUJOCO_PC_MODE=object \
    -v /home/daniel/multiview_prosthesis/docker_ws:/miahand_ws/src \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    --userns keep-id \
    -u 1000:1000 \
    --entrypoint /ros_entrypoint.sh \
    docker-deployment_mujoco_interactive \
    bash -c "source /opt/ros/jazzy/setup.bash && cd /miahand_ws && colcon build --packages-select grasp_preshaping mia_hand_mujoco && source install/setup.bash && ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py scene:=sphere depth_target_geom_name:=target_sphere depth_pc_mode:=object enable_depth_publisher:=true hardware_plugin:=mia_hand_mujoco/InteractiveSystemInterface; /bin/bash"
  ```
  If this works, the issue is definitively in the compose layer.
- [ ] **Step 5. Check for stale containers.** After the `^C` kill, there may be leftover containers. Run `podman ps -a` and remove any stuck containers from previous runs. A leftover container holding the same name can cause `podman-compose` to hang waiting for cleanup.
- [ ] **Step 6. Check `xhost` permissions.** Ensure the container user can connect to the X server: `xhost +local:podman` or `xhost +SI:localuser:$(whoami)`. Without this, GLFW window creation silently fails or hangs inside the container.

---

## Verification Criteria

- [ ] `podman compose version` returns a version string (not an error about external provider)
- [ ] `sim-run` opens the MuJoCo interactive window within ~30 seconds
- [ ] No `>>>> Executing external compose provider` message appears in the output
- [ ] The colcon build output scrolls by, followed by ROS2 launch output

## Potential Risks and Mitigations

1. **Native plugin not available for your distro**
   Mitigation: Uninstall the Python `podman-compose` and install the native plugin, or fix the Python version's TTY handling by upgrading: `pip install --upgrade podman-compose`

2. **X11 security policy blocks container connections**
   Mitigation: Use `xhost +local:` to allow local connections, or configure Xauthority properly (the commented-out lines in your compose file)

3. **Container image needs rebuild after compose changes**
   Mitigation: Run `sim-build` again after fixing the compose provider to ensure a clean image

## Alternative Approaches

1. **Force native plugin via alias**: Change the alias to explicitly call the plugin binary path if it exists but is being shadowed: `alias sim-run='/usr/bin/podman compose ...'`
2. **Use `docker-compose` (standalone)**: If available, use the official `docker-compose` v2 binary instead of `podman-compose`
3. **Add EGL/offscreen rendering**: If X11 forwarding continues to be problematic, configure MuJoCo to use EGL for offscreen rendering and skip the interactive window entirely (not suitable for your use case since you need the interactive GUI)
