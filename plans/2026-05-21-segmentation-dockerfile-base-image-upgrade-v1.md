# Segmentation Dockerfile: Switch to a Heavier Pre-Built Base Image

## Objective

Replace the current bare CUDA base image (`nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04`) with a richer pre-built base that already includes PyTorch, CUDA toolkit, Python, and build tools. This eliminates the slowest rebuild steps (PyTorch wheel download, CUDA toolkit, system packages) and resolves the CUDA version mismatch between the base image (11.8) and PyTorch wheels (cu116).

## Current State Analysis

### What the current Dockerfile installs from scratch (slow rebuild layers)

| Layer | Time Impact | Lines |
|-------|-------------|-------|
| `apt-get install` (python3.8, cmake, ninja, build-essential, openblas, etc.) | ~30-60s | `docker/Dockerfile.segmentation:26-41` |
| `pip install numpy<2` | ~5s | `docker/Dockerfile.segmentation:48` |
| `pip install torch==1.12.1+cu116 torchvision==0.13.1+cu116` | **~3-5 min** (large download ~2GB) | `docker/Dockerfile.segmentation:52-55` |
| MinkowskiEngine source build | **~5-15 min** (CUDA compilation) | `docker/Dockerfile.segmentation:77-85` |
| `pip install` inference deps (open3d, trimesh, flask, etc.) | ~30-60s | `docker/Dockerfile.segmentation:88-99` |
| `git clone` InterObject3D | ~10s | `docker/Dockerfile.segmentation:102-103` |

### The CUDA version mismatch problem

- Base image: CUDA **11.8** toolkit (`nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04`)
- PyTorch wheels: CUDA **11.6** (`torch==1.12.1+cu116`)
- MinkowskiEngine requires: **matching CUDA between PyTorch and nvcc**

## Recommended Approach: Use `pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel`

### Why this specific image

The official `pytorch/pytorch` Docker images include **all of the following pre-installed**:
- PyTorch (with matching CUDA)
- TorchVision
- Python 3.x
- CUDA toolkit + cuDNN (matching PyTorch's CUDA)
- ninja, cmake, build-essential
- numpy, pip
- Various system libraries

Available tags with CUDA 11.6 (matching the current `cu116` wheels):

| Tag | Size | Includes |
|-----|------|----------|
| `pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel` | ~9.3 GB | PyTorch 1.13.1, CUDA 11.6, cuDNN 8, Python 3.10, ninja, cmake, build tools |
| `pytorch/pytorch:1.12.1-cuda11.3-cudnn8-devel` | ~7.1 GB | PyTorch 1.12.1, CUDA 11.3, cuDNN 8, Python 3.8, ninja, cmake, build tools |

### Trade-off analysis

**Option 1: `pytorch/pytorch:1.12.1-cuda11.3-cudnn8-devel`**
- PyTorch 1.12.1 (exact same version as current)
- CUDA 11.3 (different from current cu116 wheels, but self-consistent within the image)
- Python 3.8 (matches current)
- Size: ~7.1 GB (larger initial pull, but faster rebuilds)
- Risk: **LOW** — same PyTorch version, just CUDA 11.3 instead of 11.6. CUDA 11.3 and 11.6 are compatible; the image bundles everything consistently.

**Option 2: `pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel`** (RECOMMENDED)
- PyTorch 1.13.1 (minor bump from 1.12.1)
- CUDA 11.6 (exact match to current cu116 wheels)
- Python 3.10 (upgrade from 3.8)
- Size: ~9.3 GB
- Risk: **LOW-MEDIUM** — PyTorch 1.13 is API-compatible with 1.12. MinkowskiEngine 0.5.4 works with PyTorch 1.13. Python 3.10 is compatible.

**Option 3: `pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel`**
- PyTorch 2.0.1 (major version jump)
- CUDA 11.7
- Risk: **HIGH** — MinkowskiEngine 0.5.4 may not compile against PyTorch 2.0 due to breaking changes in `torch.utils.cpp_extension`.

### Recommendation: Option 2

`pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel` is the best balance because:
1. CUDA 11.6 matches what you currently use (`cu116`)
2. PyTorch 1.13.1 is the last 1.x release — stable, well-tested with MinkowskiEngine
3. The image is self-consistent (PyTorch's CUDA == system nvcc CUDA)
4. You eliminate the ~3-5 minute PyTorch download and ~30-60 second apt-get layer

## Implementation Plan

- [x] **Task 1.** Update the CUDA base image in `docker/Dockerfile.segmentation:17`
  - Change from: `FROM docker.io/nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04 AS base-cuda`
  - Change to: `FROM docker.io/pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel AS base-cuda`
  - Rationale: This single change gives us PyTorch, CUDA toolkit, Python, ninja, cmake, build-essential, and numpy all pre-installed with matching CUDA versions.

- [x] **Task 2.** Remove the redundant `apt-get install` block at `docker/Dockerfile.segmentation:26-41`
  - The pytorch base already includes: python3, python3-pip, ninja-build, cmake, build-essential, and most system libraries.
  - Keep only the packages NOT in the pytorch image: `libopenblas-dev`, `wget`, `libgl1`, `libglib2.0-0`, `libsm6`, `libxext6`, `libxrender1` (needed for Open3D headless).
  - Remove `python3.8`, `python3.8-dev` — the pytorch image has Python 3.10 pre-installed and `python`/`pip` already point to it.
  - Rationale: Eliminates a ~30-60s rebuild layer while keeping Open3D dependencies.

- [x] **Task 3.** Remove the `update-alternatives` block at `docker/Dockerfile.segmentation:44-45`
  - The pytorch image already sets up `python` and `pip` correctly.
  - Rationale: These commands would fail or be unnecessary in the pytorch base.

- [x] **Task 4.** Remove the `pip install numpy<2` line at `docker/Dockerfile.segmentation:48`
  - The pytorch base image already has numpy installed.
  - If a specific pin is needed, change to `pip install --no-cache-dir "numpy<2"` only if build failures occur. PyTorch 1.13.1 ships with numpy 1.2x which is fine.
  - Rationale: Eliminates unnecessary layer.

- [x] **Task 5.** Remove the entire `pytorch-cuda` build stage at `docker/Dockerfile.segmentation:51-55`
  - Change the Dockerfile structure so the CUDA variant goes directly from `base-cuda` to `final` without a separate PyTorch install stage.
  - The pytorch base already has `torch` and `torchvision` installed.
  - Rationale: This is the biggest time saver — eliminates the ~3-5 minute PyTorch wheel download.

- [x] **Task 6.** Keep the `pytorch-cpu` stage at `docker/Dockerfile.segmentation:57-61` unchanged
  - The CPU variant still uses `ubuntu:20.04` as its base, so it still needs pip-installed PyTorch.
  - Rationale: No change needed for CPU path.

- [x] **Task 7.** Update the Dockerfile header comments at `docker/Dockerfile.segmentation:1-12`
  - Update from "Python 3.8 + PyTorch 1.12" to reflect the new versions.
  - Update CUDA compatibility note from "CUDA 11.8" to "CUDA 11.6".
  - Rationale: Keep documentation accurate.

- [x] **Task 8.** Update `docs/segmentation-backends.md` pinned versions table at lines 163-168
  - CUDA base image: `pytorch/pytorch:1.13.1-cuda11.6-cudnn8-devel`
  - PyTorch (CUDA): `1.13.1` (pre-installed in base)
  - TorchVision (CUDA): `0.14.1` (pre-installed in base)
  - Python: `3.10`
  - Rationale: Documentation must match the new configuration.

- [x] **Task 9.** Update `docs/segmentation-backends.md` CUDA image description at line 55
  - Change "PyTorch: CUDA 11.6 wheels (`torch==1.12.1+cu116`)" to reflect PyTorch 1.13.1.
  - Rationale: Documentation consistency.

- [x] **Task 10.** Update compose override comments in `docker/docker-compose.segmentation.cuda.yml:10`
  - Change "Host GPU driver compatible with CUDA 11.8" to "Host GPU driver compatible with CUDA 11.6".
  - Rationale: Driver requirement changes with the CUDA version.

- [x] **Task 11.** Update compose override comments in `docker/docker-compose.segmentation.podman-gpu.yml:10`
  - Same CUDA version update as Task 10.
  - Rationale: Consistency.

- [x] **Task 12.** Update the health check expected output in `docs/segmentation-backends.md:108`
  - Change `"torch_cuda_version": "11.6"` stays the same (it was already 11.6 from the cu116 wheels).
  - Rationale: Verify documentation matches runtime behavior.

- [~] **Task 13.** Verify MinkowskiEngine 0.5.4 builds against PyTorch 1.13.1
  - Test: run `make build-segmentation-cuda` and confirm the MinkowskiEngine compilation succeeds.
  - MinkowskiEngine 0.5.4's `setup.py` uses `torch.utils.cpp_extension.BuildExtension` and `CUDAExtension`, which are stable across PyTorch 1.12-1.13.
  - Rationale: This is the key validation gate before considering the change complete.

## Verification Criteria

- [ ] `make build-segmentation-cuda` completes successfully with the new base image
- [ ] `make build-segmentation-cpu` still works (CPU path unchanged)
- [ ] `make validate-segmentation-config` passes
- [ ] Health endpoint at `/health` returns `"torch_cuda_available": true` and `"torch_cuda_version": "11.6"`
- [ ] Rebuilding after a trivial change (e.g., touching inference_server.py) is noticeably faster because the PyTorch and system package layers are cached in the base image
- [ ] No CUDA version mismatch warnings during MinkowskiEngine compilation

## Potential Risks and Mitigations

1. **PyTorch 1.13.1 incompatibility with MinkowskiEngine 0.5.4**
   - Likelihood: Low. PyTorch 1.13 is a minor release from 1.12 with no breaking changes in the C++ extension API.
   - Mitigation: Test build before committing. If it fails, fall back to `pytorch/pytorch:1.12.1-cuda11.3-cudnn8-devel` (Option 1).

2. **Python 3.10 incompatibility with InterObject3D or other dependencies**
   - Likelihood: Low. InterObject3D is pure Python + PyTorch, no C extension tied to Python version.
   - Mitigation: If issues arise, add `update-alternatives` to install Python 3.8 alongside, or use the 1.12.1 image which has Python 3.8.

3. **Larger initial image pull (~9.3 GB vs ~7 GB for current CUDA base)**
   - Impact: One-time cost on first build. Subsequent rebuilds are faster.
   - Mitigation: This is an acceptable trade-off for faster iteration cycles.

4. **Open3D wheel compatibility with Python 3.10 + PyTorch 1.13**
   - Likelihood: Low. Open3D >= 0.12 supports Python 3.10.
   - Mitigation: Pin `open3d==0.16.x` or later if the latest has issues.

5. **numpy version conflict**
   - Likelihood: Low. PyTorch 1.13 ships with numpy 1.2x (< 2).
   - Mitigation: Add `"numpy<2"` pin back if needed.

## Alternative Approaches

1. **`pytorch/pytorch:1.12.1-cuda11.3-cudnn8-devel`** (Option 1 from analysis)
   - Smaller image (~7.1 GB), same PyTorch version as current, Python 3.8.
   - Trade-off: CUDA 11.3 instead of 11.6 (still self-consistent, but older).

2. **Multi-stage build with pytorch runtime image as final stage**
   - Use the `devel` image for building MinkowskiEngine, then copy artifacts to a `runtime` image for smaller final size.
   - Trade-off: More complex Dockerfile, but final image could be ~4 GB instead of ~9 GB.
   - This is a good follow-up optimization after the initial base image switch.

3. **Keep current approach but fix the CUDA mismatch**
   - Downgrade base to `nvidia/cuda:11.6.2-cudnn8-devel-ubuntu20.04` (as suggested in previous analysis).
   - Trade-off: Doesn't address the rebuild speed concern. Still slow due to PyTorch download.
