# Segmentation Container Backend Workflow

This document describes how to build and run the segmentation inference server
container with either CPU or CUDA backend support.

## Quick Reference

| Make target | Compose service | Image tag | GPU required |
|-------------|-----------------|-----------|--------------|
| `make build-segmentation-cuda` | `segmentation-cuda` | `segmentation:cuda` | No (build) |
| `make build-segmentation-cpu` | `segmentation-cpu` | `segmentation:cpu` | No |
| `make segmentation-cuda` | `segmentation-cuda` | `segmentation:cuda` | **Yes** (runtime) |
| `make segmentation-cpu` | `segmentation-cpu` | `segmentation:cpu` | No |
| `make down-segmentation` | — | — | — |

**Legacy alias:** `make segmentation` is an alias for `make segmentation-cuda`.

## Container Backend Selection

The Makefile auto-detects the container backend:

1. If `podman-compose` is installed → uses `podman-compose` + `podman`
2. Otherwise, if `docker` is installed → uses `docker compose` + `docker`
3. Otherwise → fails with a clear error

You can override auto-detection explicitly:

```bash
# Force Docker
CONTAINER_BACKEND=docker make segmentation-cuda

# Force Podman
CONTAINER_BACKEND=podman make segmentation-cuda
```

If the selected backend is unavailable, the Makefile fails immediately with:

```
ERROR: CONTAINER_BACKEND=docker but 'docker' is not available
```

## CPU vs CUDA Images

The two variants are **distinguishable and non-overwriting**:

- **CPU image (`segmentation:cpu`)**
  - Base: `ubuntu:20.04`
  - PyTorch: CPU-only wheels (`torch==1.12.1+cpu`)
  - MinkowskiEngine: built with `--cpu_only`
  - Runs on any x86_64 or aarch64 host without NVIDIA support
  - Container size: ~2.5 GB smaller than CUDA variant

- **CUDA image (`segmentation:cuda`)**
  - Base: `nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04`
  - PyTorch: CUDA 11.6 wheels (`torch==1.12.1+cu116`)
  - MinkowskiEngine: built with `--force_cuda`
  - Requires NVIDIA Container Toolkit at runtime for GPU access
  - Falls back to CPU inference on hosts without a GPU **only if** CUDA runtime libs are present (they are, in the image)

Because the image tags are different (`segmentation:cpu` vs `segmentation:cuda`),
building the CPU target **cannot** silently replace the CUDA runtime image.

## GPU Prerequisites

### Docker

- NVIDIA driver >= **450.80.02**
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- Docker >= 19.03 (or nvidia-docker2)

The CUDA compose override uses:

```yaml
runtime: nvidia
deploy.resources.reservations.devices:  # Docker Compose v2 GPU syntax
```

### Podman

- Podman >= 4.0
- [NVIDIA container device support](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#podman)
- CDI (Container Device Interface) or `libnvidia-container` configured
- The `nvidia.com/gpu=all` device must be available

The Podman compose override uses:

```yaml
devices:
  - nvidia.com/gpu=all
```

## Health Check

Once a segmentation container is running, verify its backend mode:

```bash
# Check health endpoint (returns JSON)
curl -s http://localhost:5678/health | python -m json.tool
```

Expected output on a CUDA host:

```json
{
    "status": "ok",
    "device": "cuda",
    "torch_cuda_available": true,
    "torch_cuda_version": "11.6",
    "minkowski_engine_version": "0.5.4",
    "minkowski_engine_cuda": true
}
```

Expected output on a CPU-only host:

```json
{
    "status": "ok",
    "device": "cpu",
    "torch_cuda_available": false,
    "torch_cuda_version": null,
    "minkowski_engine_version": "0.5.4",
    "minkowski_engine_cuda": false
}
```

## Validation

Run the backend matrix validation from the repository root:

```bash
# Validate compose config (profiles and services exist)
make validate-segmentation-config

# Full validation (dry-run build + run targets)
make validate-segmentation
```

These checks verify:
- `segmentation-cpu` and `segmentation-cuda` profiles/services are defined
- Compose config renders without errors for both Docker and Podman GPU overrides
- Make targets resolve to valid compose commands

## Platform Compatibility

| Platform | CPU backend | CUDA backend |
|----------|-------------|--------------|
| x86_64 Linux (desktop/server) | Supported | Supported |
| x86_64 Linux (WSL2) | Supported | Supported (with WSL CUDA) |
| **Jetson / L4T (aarch64)** | **Supported** | **Explicitly NOT supported** |

### Why Jetson CUDA is not supported

This CUDA image is built for **x86_64** with NVIDIA's desktop CUDA toolkit
(`nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04`). Jetson devices require
L4T-based images (e.g., `nvcr.io/nvidia/l4t-pytorch`) and a different
MinkowskiEngine build process. Use the CPU backend on Jetson, or build a
separate Jetson-specific CUDA image if needed.

### Pinned versions

| Component | Version | Notes |
|-----------|---------|-------|
| CUDA base image | 11.8.0-cudnn8-devel-ubuntu20.04 | Desktop x86_64 only |
| PyTorch (CUDA) | 1.12.1+cu116 | CUDA 11.6 wheels |
| PyTorch (CPU) | 1.12.1+cpu | CPU-only wheels |
| TorchVision (CUDA) | 0.13.1+cu116 | Matches PyTorch CUDA |
| TorchVision (CPU) | 0.13.1+cpu | Matches PyTorch CPU |
| Python | 3.8 | Required by MinkowskiEngine |
| MinkowskiEngine | 0.5.4 (source build) | `--force_cuda` or `--cpu_only` |
| Open3D | >= 0.12.0 | Headless runtime |
| InterObject3D | commit `827b350` | Pinned known-good |

## Troubleshooting

### "Could not select device driver "" with capabilities: [[gpu]]"

The NVIDIA Container Toolkit is not installed or not configured for your backend.
See [GPU Prerequisites](#gpu-prerequisites).

### "nvidia.com/gpu=all: no such file or directory"

Podman cannot find the NVIDIA CDI device. Ensure the NVIDIA Container Toolkit
hook is registered for Podman:

```bash
sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml
```

### CPU image still references CUDA

Make sure you are passing `BACKEND=cpu` at build time. The Makefile does this
automatically for `build-segmentation-cpu`.

### `torch.cuda.is_available()` is false inside the CUDA container

- Verify the host has a working GPU: `nvidia-smi`
- Verify the container runtime is NVIDIA-enabled: `docker run --rm --runtime=nvidia nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi`
- For Podman: `podman run --rm --device nvidia.com/gpu=all nvidia/cuda:11.8.0-base-ubuntu20.04 nvidia-smi`
