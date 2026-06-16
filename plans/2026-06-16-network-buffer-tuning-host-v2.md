# Network Buffer Tuning & QoS Optimization

## Objective

Resolve connection saturation between the Jetson and the host/laptop by applying three layers of defense: kernel-level UDP buffer tuning (both machines), CycloneDDS socket buffer configuration (both machines), and QoS relaxation for monitoring tools (host side). These changes complement the Jetson-side topic-lightening work already in progress.

## Why Host-Only Changes Help (Even Without Jetson Changes)

The connection saturation manifests as crashes on the **receiving** side — `ros2 topic hz` and `ros2 topic echo` dying because the host kernel cannot reassemble fragmented UDP datagrams fast enough. Each 5-8 MB PointCloud2 from the Jetson arrives as ~120 RTPS fragments. The host kernel must:

1. Receive each fragment into a socket buffer (`rmem_max` limits this)
2. Queue fragments in the IP reassembly cache (`ipfrag_high_thresh` limits this)
3. Reassemble the full datagram before the fragment timeout expires (`ipfrag_time`)

All three of these are **receiver-side kernel operations**. The Jetson's send path is a simple `sendto()` — it doesn't reassemble anything. So host-side kernel tuning alone eliminates the crash even if the Jetson is completely untouched.

The Jetson does benefit from `wmem_max` and `ipfrag_high_thresh` on its side too (in case it receives fused pointclouds or service responses from the host), but the immediate crashes are a host-receive problem.

## Background & Diagnosis

### Root Cause
The host receives two RealSense D435i PointCloud2 streams (640×480 depth clouds, 2-10 MB each) plus VIO odometry from the Jetson over a direct Ethernet link via CycloneDDS UDP unicast. Large PointCloud2 messages are fragmented at the RTPS layer (default `MaxMessageSize=65536` → ~150 fragments per cloud). The receiving socket must reassemble these fragments within kernel IP fragment buffers and within timeout windows.

### Current State (Host)
- **No kernel buffer tuning**: `net.core.rmem_max` is at Linux default (typically 212992 bytes ≈ 208 KB). Two incoming PointCloud2 streams at 15-30 Hz each with 150+ fragments overwhelm this immediately.
- **CycloneDDS config** (`config/cyclonedds_peer.xml`) has no `<Internal>` socket buffer settings — falls back to 512 KB default.
- **Monitoring tools** (`ros2 topic echo`, `ros2 topic hz`) subscribe with **default QoS** (which inherits publisher RELIABLE), generating ACK/NACK traffic that further saturates the link.
- **Containers use `network_mode: host`**, so kernel settings on the host apply to all ROS2 traffic.
- **Documented QoS mismatches** between BEST_EFFORT subscribers and RELIABLE publishers have been fixed in pipeline nodes (see `plans/2026-05-20-pointcloud-fusion-debugging-retrospective-v1.md:107-119`), but monitoring tools were never updated.

### Current State (Jetson)
- Same kernel defaults — no `wmem_max`, `rmem_max`, or `ipfrag_*` tuning.
- CycloneDDS config on the Jetson is deployed via `make jetson-sync` from this repo (or managed manually).
- The Jetson is primarily a publisher, so `wmem_max` (send buffers) is more relevant than `rmem_max` for its role. However, `rmem_max` and `ipfrag_high_thresh` matter if Jetson-side tools subscribe to host-published topics like `/fused_pointcloud`.

### Topics Most Affected
| Topic | Size Est. | Rate | Fragments/msg |
|-------|-----------|------|---------------|
| `/head/d435i_head/depth/color/points` | 5-8 MB | 15-30 Hz | ~80-120 |
| `/arm/d435i_arm/depth/color/points` | 5-8 MB | 15-30 Hz | ~80-120 |
| `/head/d435i_head/color/image_raw` | 0.9 MB | 15-30 Hz | ~15 |
| `/arm/d435i_arm/color/image_raw` | 0.9 MB | 15-30 Hz | ~15 |
| `/fused_pointcloud` | 8-15 MB | ~15 Hz | ~120-230 |

## Implementation Plan

### Phase 1: Kernel Buffer Tuning — Host Side

These changes apply to the bare-metal host OS, not inside containers. Since all containers use `network_mode: host`, the kernel settings affect all ROS2 UDP traffic.

- [ ] Task 1.1 **Create persistent sysctl configuration file** at `/etc/sysctl.d/99-prosthesis-udp.conf`

  Set four critical kernel parameters:
  - `net.core.rmem_max=2147483647` — Maximum socket receive buffer (2 GB). This allows CycloneDDS (or any app) to call `setsockopt(SO_RCVBUF, ...)` with values up to 2 GB. Currently at the kernel default (~208 KB), which cannot hold even a single unreassembled PointCloud2.
  - `net.core.wmem_max=2147483647` — Maximum socket send buffer (2 GB). Symmetrical tuning for the host side, which also publishes `/fused_pointcloud` (8-15 MB) back toward Jetson subscribers (e.g., RViz).
  - `net.ipv4.ipfrag_high_thresh=134217728` — IP fragment reassembly memory threshold (128 MB). When the total memory used by all in-flight IP fragments exceeds this, the kernel starts dropping fragments aggressively. The default (typically 4 MB) cannot handle 150+ fragments from two simultaneous clouds.
  - `net.ipv4.ipfrag_time=3` — Maximum time to hold IP fragments waiting for reassembly (3 seconds). Reducing from the default (30s) prevents stale fragments from consuming buffer memory when a fragment is lost, and matches the expected cloud arrival cadence (~33-66 ms inter-frame).

  **Rationale**: These are the values recommended in the user's research. They are well-established in high-throughput DDS/ROS2 deployments. The `rmem_max` of 2 GB is the ceiling — actual memory usage is bounded by `SocketReceiveBufferSize` × number of sockets (typically 1-2 MB each). The `ipfrag_high_thresh` of 128 MB ensures room for ~8 simultaneous partially-reassembled PointCloud2 messages (each ~8 MB fragmented into ~120×64KB pieces with overhead).

- [ ] Task 1.2 **Create host-side apply script** at `scripts/tune_network_host.sh`

  Script should:
  - Apply the four sysctl values immediately via `sudo sysctl -w`
  - Copy the conf file to `/etc/sysctl.d/` for persistence across reboots
  - Run `sudo sysctl --system` to load all sysctl.d configs
  - Verify the values are active with `sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_time`
  - Check for potential conflicts (e.g., `net.core.rmem_default` should be ≤ `rmem_max`)

  **Rationale**: A dedicated script ensures consistent application, provides verification output, and can be called from `make network-tune`. The persistence via `/etc/sysctl.d/` means these settings survive reboots without manual intervention.

- [ ] Task 1.3 **Add `make network-tune` target** to the host Makefile for the host side

  Add a new target to `Makefile` that:
  - Calls `scripts/tune_network_host.sh`
  - Prints the before/after values
  - Is idempotent — safe to run multiple times

  **Rationale**: Makes the tuning discoverable via `make help` and provides a single command for host-side setup.

### Phase 2: Kernel Buffer Tuning — Jetson Side (Remote via SSH)

Following the same pattern as `make timesync` (`Makefile:553-582`), which SSHs into the Jetson to configure chrony, these tasks push and apply the sysctl configuration remotely.

- [ ] Task 2.1 **Create Jetson-side sysctl configuration script** at `scripts/tune_network_jetson.sh`

  This is a self-contained script that is scp'd to the Jetson and executed there. It should:
  - Write the same four sysctl values to `/etc/sysctl.d/99-prosthesis-udp.conf` on the Jetson
  - Apply them immediately via `sysctl -w`
  - Run `sysctl --system` for persistence
  - Print verification output

  Note: The Jetson may have different sudo requirements — the timesync target uses `echo robotlab | sudo -S` for passwordless sudo. This script should follow the same convention.

  **Rationale**: Keeps the Jetson-side logic in a versioned script rather than inline SSH commands, making it testable and maintainable. The same four values apply — both machines need identical kernel buffer configuration because both can send and receive large fragmented UDP payloads.

- [ ] Task 2.2 **Add `make network-tune-jetson` target** to the host Makefile

  Following the `make timesync` pattern (`Makefile:553`):
  - Requires `make robotlab-connect` as a prerequisite (ensures Ethernet link is up)
  - scp's `scripts/tune_network_jetson.sh` to the Jetson
  - SSHs in and executes it with sudo
  - Prints verification output from the Jetson

  **Rationale**: A single command from the host configures both machines. Users don't need to SSH into the Jetson manually. The `robotlab-connect` prerequisite ensures the Jetson is reachable.

- [ ] Task 2.3 **Add `make network-tune-all` convenience target**

  A meta-target that calls both `network-tune` (host) and `network-tune-jetson` (Jetson):
  ```makefile
  network-tune-all: network-tune network-tune-jetson
  ```
  **Rationale**: One command to configure both machines. The host target runs first because it doesn't depend on the Jetson being reachable.

### Phase 3: CycloneDDS Socket Buffer Configuration

CycloneDDS requests socket buffer sizes from the kernel via `setsockopt(SO_RCVBUF)`. The requested size is capped by `net.core.rmem_max`. Without explicit configuration, CycloneDDS uses 512 KB defaults from its XML schema, which is insufficient for reassembling ~150-fragment PointCloud2 messages.

**Important**: The `config/cyclonedds_peer.xml` in this repo is deployed to the Jetson via `make jetson-sync`. However, the Jetson may have its own local variant of this file (the current XML hardcodes `address="10.42.0.1"` which is the host IP). This plan updates the repo copy; the user will manually apply the same `<Internal>` block to the Jetson's local CycloneDDS config.

- [ ] Task 3.1 **Add `<Internal>` socket buffer settings to `config/cyclonedds_peer.xml`**

  Add the following to the existing `<CycloneDDS><Domain>` block:

  ```xml
  <Internal>
    <SocketReceiveBufferSize>8388608</SocketReceiveBufferSize>
    <SocketSendBufferSize>8388608</SocketSendBufferSize>
    <MaximumMessageSize>65536</MaximumMessageSize>
    <FragmentSize>16384</FragmentSize>
  </Internal>
  ```

  **Rationale for each value**:
  - `SocketReceiveBufferSize=8388608` (8 MB): Large enough to hold one complete PointCloud2 message (~5-8 MB) plus overhead. CycloneDDS will request this via `SO_RCVBUF`; the kernel will grant it because `rmem_max=2GB`. Values above 8 MB provide diminishing returns — the kernel double-buffers, so actual allocation is 2× this value (16 MB per socket).
  - `SocketSendBufferSize=8388608` (8 MB): Symmetrical for the host's `/fused_pointcloud` publisher.
  - `MaximumMessageSize=65536` (64 KB): This is the CycloneDDS default and is appropriate — it's the RTPS message size limit. Larger values risk IP-level fragmentation. PointCloud2 messages above this size are fragmented at the RTPS layer into 64 KB messages.
  - `FragmentSize=16384` (16 KB): This is the CycloneDDS default for UDP. It keeps each fragment well under the typical Ethernet MTU (1500 bytes) after IP/UDP headers, avoiding IP-level fragmentation. Lower values increase fragment count but improve reliability on lossy links.

  **Alternative considered**: Setting `SocketReceiveBufferSize` to 0 (let OS decide) — rejected because the OS default is too small for this workload. Setting it to the 2 GB maximum — rejected because it wastes kernel memory without benefit; 8 MB per socket is the sweet spot for ~8 MB PointCloud2 messages.

- [ ] Task 3.2 **Document the CycloneDDS `<Internal>` options in the XML with inline comments**

  Add XML comments explaining each setting so future maintainers understand the tuning rationale.

  **Rationale**: Self-documenting configuration reduces maintenance burden. The CycloneDDS XML schema is not well-known outside DDS specialists.

- [ ] Task 3.3 **User manual step: Apply same `<Internal>` block to Jetson's CycloneDDS config**

  The Jetson's active CycloneDDS config (wherever it lives — likely a local copy with `address="10.42.0.2"`) needs the same `<Internal>` block. This is a manual step because the Jetson's config location and contents may differ from the repo copy.

  **Rationale**: The host and Jetson need identical CycloneDDS socket buffer settings for symmetric performance. The Jetson uses these buffers when publishing large PointCloud2s (send buffers) and when subscribing to `/fused_pointcloud` (receive buffers).

### Phase 4: QoS Relaxation for Monitoring Tools (Host Side Only)

Monitoring tools (`ros2 topic echo`, `ros2 topic hz`, `ros2 topic bw`) don't need reliable delivery — they're sampling, and a dropped sample is acceptable. Using `--qos-reliability best_effort` eliminates ACK/NACK traffic back to the publisher, reducing network load by up to 30% per subscriber.

**Important constraint** (from `plans/2026-05-20-pointcloud-fusion-debugging-retrospective-v1.md:107-119`): Pipeline nodes MUST use RELIABLE QoS because BEST_EFFORT subscribers silently drop data from RELIABLE publishers. This QoS relaxation is **only for monitoring/diagnostic tools**, never for pipeline data-flow nodes.

- [ ] Task 4.1 **Update `scripts/ros2_ethernet_hello_host.sh` to use best_effort for topic echo**

  In the `listen-jetson` case (`line 133`), change:
  ```
  ros2 topic echo /jetson_hello std_msgs/msg/String
  ```
  to:
  ```
  ros2 topic echo --qos-reliability best_effort /jetson_hello std_msgs/msg/String
  ```
  This is the lightweight test topic, but establishing the pattern here propagates to all monitoring use.

  **Rationale**: Sets a consistent precedent. Even for small topics, best_effort eliminates unnecessary ACK round-trips over the Ethernet link.

- [ ] Task 4.2 **Update `Makefile.workspace` "tonight" validation gates to use best_effort**

  Find the "tonight" targets that call `ros2 topic echo` for raw cloud rate checking and add `--qos-reliability best_effort`. These targets are defined in `Makefile.workspace` and include:
  - `tonight-raw-check` — echoes PointCloud2 headers for rate verification
  - `tonight-fusion-check` — echoes fused cloud headers
  - `tonight-imu-check` — echoes IMU/odom headers

  **Rationale**: These validation gates run during development and testing. Using best_effort prevents the monitoring itself from becoming a source of network congestion that could mask real issues.

- [ ] Task 4.3 **Add QoS guidance comment to key monitoring scripts**

  Add a comment block at the top of relevant scripts documenting the best_effort pattern:
  ```bash
  # NETWORK NOTE: When monitoring topics over the Jetson Ethernet link,
  # always append --qos-reliability best_effort to ros2 topic echo/hz/bw
  # to avoid saturating the link with RELIABLE ACK/NACK traffic.
  # Pipeline nodes MUST use RELIABLE; monitoring tools should use BEST_EFFORT.
  ```

  **Rationale**: Ensures future monitoring scripts follow the pattern. The distinction between monitoring (best_effort OK) and pipeline data-flow (RELIABLE required) is critical and should be documented in context.

### Phase 5: Validation

- [ ] Task 5.1 **Verify kernel buffer values are active on both machines**

  After applying sysctl changes, confirm on both host and Jetson:
  ```bash
  sysctl net.core.rmem_max net.core.wmem_max net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_time
  ```
  Expected output: All values match the configured settings on both machines.

- [ ] Task 5.2 **Verify CycloneDDS picks up new XML config**

  Launch the prosthesis container and check:
  ```bash
  # Inside container
  cat /tmp/cyclonedds_peer.xml  # Confirm Internal block is present
  ```
  CycloneDDS applies the XML at process start; a container restart picks up the changes.

- [ ] Task 5.3 **Run a full pipeline with host-side monitoring and check for drops**

  Use the existing validation gates (`make tonight-gates` from host) to verify:
  - Cloud rates are stable (no drops)
  - `ros2 topic hz --qos-reliability best_effort` on `/head/d435i_head/depth/color/points` shows expected rate
  - No "connection saturation" errors in logs

## Verification Criteria

- [ ] `sysctl net.core.rmem_max` returns `2147483647` on **both** host and Jetson
- [ ] `sysctl net.ipv4.ipfrag_high_thresh` returns `134217728` on **both** host and Jetson
- [ ] `sysctl net.ipv4.ipfrag_time` returns `3` on **both** host and Jetson
- [ ] `config/cyclonedds_peer.xml` contains `<SocketReceiveBufferSize>8388608</SocketReceiveBufferSize>` (host)
- [ ] Jetson's CycloneDDS config contains the same `<Internal>` block (manual step)
- [ ] `make tonight-gates` completes without topic echo crashes
- [ ] `ros2 topic hz --qos-reliability best_effort /head/d435i_head/depth/color/points` shows stable rate
- [ ] No "failed to receive" or "connection lost" errors in CycloneDDS logs during pipeline operation

## Potential Risks and Mitigations

1. **Risk: Setting `ipfrag_high_thresh=128MB` could starve other kernel memory consumers on a RAM-constrained machine**
   Mitigation: This is a threshold, not an allocation — only in-use fragments consume memory. Two PointCloud2 messages in-flight (~16 MB) plus overhead (~2 MB) ≈ 18 MB typical. The 128 MB threshold is headroom. The Jetson Orin Nano has 8 GB RAM, so 128 MB is safe. If either machine has < 4 GB RAM, reduce to 67 MB (`67108864`). Document this in the script comments.

2. **Risk: `ipfrag_time=3` may be too low if Jetson publishes clouds with >3s gaps**
   Mitigation: Normal cloud publish rate is 15-30 Hz (33-66 ms between frames). Even a 1 Hz slow mode would be 1s between frames. 3 seconds provides a 3× safety margin. If the pipeline uses very low rates (< 0.3 Hz), increase to 5 seconds. This is configurable per deployment.

3. **Risk: BEST_EFFORT monitoring subscribers may miss data, giving false impression of pipeline failure**
   Mitigation: BEST_EFFORT subscribers CAN receive data from RELIABLE publishers (the issue was the reverse — BEST_EFFORT publishers + RELIABLE subscribers). The monitoring tools use RELIABLE→BEST_EFFORT which is compatible. A dropped monitoring sample is acceptable — the tool is sampling, not storing. Pipeline data-flow subscribers remain RELIABLE.

4. **Risk: CycloneDDS `SocketReceiveBufferSize=8MB` may not be sufficient for fused_pointcloud (8-15 MB)**
   Mitigation: The fused cloud is published from the host (localhost), not over the Jetson link, so it doesn't traverse the Ethernet bottleneck. Host-local CycloneDDS uses shared memory or loopback, which doesn't fragment at the IP level. If the fused cloud is ever sent over the link, increase `SocketReceiveBufferSize` to `16777216` (16 MB).

5. **Risk: SSH to Jetson may fail or require different credentials**
   Mitigation: The `make network-tune-jetson` target follows the exact same SSH pattern as `make timesync` (`Makefile:553-582`), which uses `$(JETSON_HOST)` (set to `robotlab`) and `echo robotlab | sudo -S`. If timesync works, network-tune-jetson will also work. The `robotlab-connect` prerequisite ensures the Ethernet link is up.

## Alternative Approaches

1. **Use FastRTPS instead of CycloneDDS**: FastRTPS has different buffer management and may handle large fragmented UDP payloads more gracefully. Trade-offs: Major change to the entire ROS2 stack, requires revalidating all QoS configurations, breaks the CycloneDDS-specific tf_static workaround.

2. **Reduce PointCloud2 size on the Jetson side** (already in progress): Downsampling, voxel filtering, or ROI cropping before publishing. This is complementary to the host-side changes — lighter topics reduce the urgency of buffer tuning but don't eliminate the need for it entirely. The user is already working on this.

3. **Increase CycloneDDS `MaxMessageSize` beyond 65536**: Would reduce RTPS-level fragmentation (fewer, larger fragments), but increases risk of IP-level fragmentation if messages exceed the path MTU. Trade-offs: Fewer fragments but larger; may trigger IP fragment reassembly issues if the MTU is standard 1500 bytes. The current 64 KB `MaxMessageSize` with 16 KB `FragmentSize` is the recommended balance.

4. **Use TCP transport for CycloneDDS** instead of UDP: TCP handles fragmentation and reassembly natively without kernel buffer issues. Trade-offs: Higher latency due to in-order delivery and head-of-line blocking; CycloneDDS TCP support is less mature than UDP; would require regenerating discovery config.

## Files to Create

| File | Purpose |
|------|---------|
| `scripts/tune_network_host.sh` | Apply and persist sysctl settings on the host |
| `scripts/tune_network_jetson.sh` | Apply and persist sysctl settings on the Jetson (executed remotely) |
| `/etc/sysctl.d/99-prosthesis-udp.conf` | Persistent kernel buffer configuration (created by the scripts on each machine) |

## Files to Modify

| File | Change |
|------|--------|
| `config/cyclonedds_peer.xml` | Add `<Internal>` block with socket buffer and fragment settings |
| `Makefile` (host) | Add `network-tune`, `network-tune-jetson`, `network-tune-all` targets |
| `scripts/ros2_ethernet_hello_host.sh` | Add `--qos-reliability best_effort` to topic echo |
| `Makefile.workspace` (in-container) | Add `--qos-reliability best_effort` to tonight validation echo commands |
| Jetson's local CycloneDDS config | Add identical `<Internal>` block (manual step by user) |

## Dependencies

- **Requires `sudo` on host** to apply sysctl settings and write to `/etc/sysctl.d/`
- **Requires `sudo` on Jetson** via SSH (same mechanism as `make timesync`)
- **Requires `robotlab-connect`** (Ethernet link up, Jetson reachable at `10.42.0.2`) for Jetson-side changes
- **No changes needed inside the prosthesis container** — the CycloneDDS XML is already mounted as a read-only volume at `docker/docker-compose.yml:52`
- **No new packages or dependencies** — all changes use existing kernel and CycloneDDS features
