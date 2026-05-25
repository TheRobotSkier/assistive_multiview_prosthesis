# Persist EMG Data and Models Across Container Restarts

## Objective

Fix the issue where `make down` destroys EMG training data (`/app/data`) and trained classifier models (`/app/models`) inside the container, requiring re-collection and re-training every time the container is recreated.

## Root Cause

The EMG pipeline writes to two paths inside the container:
- `/app/data` — training data `.npz` files (from `collect_data`)
- `/app/models` — trained classifier `classifier.pkl` (from `train`)

These paths are created at image build time (`Dockerfile:78-79`):
```dockerfile
RUN mkdir -p /app/data && chown -R prosthesis:prosthesis /app
```

But `docker/docker-compose.yml` has **no volume mounts for `/app/data` or `/app/models`**. They live only in the container's ephemeral writable layer. When `make down` runs `docker compose down`, the container is removed and all data in `/app/data` and `/app/models` is lost.

The build artifacts (`build/`, `install/`, `log/`) already use named Docker volumes (lines 63-66) which persist across `down`/`up` cycles. The EMG data needs the same treatment.

## Implementation Plan

- [ ] **1. Add bind mounts for `/app/data` and `/app/models` in `docker/docker-compose.yml`**

  In the `prosthesis` service `volumes` section (after line 66), add two bind mounts pointing to the host `data/` and `models/` directories:

  ```yaml
  # Build artifacts — named volumes persist across container restarts
  - prosthesis-build:/prosthesis_ws/build
  - prosthesis-install:/prosthesis_ws/install
  - prosthesis-log:/prosthesis_ws/log
  # EMG training data and trained models — persist across container restarts
  - ../data:/app/data:rw
  - ../models:/app/models:rw
  ```

  Rationale: The host `data/` and `models/` directories already exist in the project root. Bind mounting them means:
  - Data written inside the container at `/app/data` and `/app/models` appears on the host at `data/` and `models/`
  - Data persists on the host filesystem across `make down` / `make dev` cycles
  - The same pattern is already used for segmentation weights (`${HOME}/prosthesis_data/weights:/weights`)

- [ ] **2. Add `.gitignore` entries for EMG data and model files**

  The `data/` and `models/` directories contain user-specific training data (`.npz` files) and trained models (`.pkl` files) that should not be committed. Add to `.gitignore` after the `# Rosbags` block (line 67):

  ```
  # EMG training data and trained models (persist via bind mounts)
  data/*.npz
  models/*.pkl
  models/*.npz
  ```

  Rationale: Prevents accidentally committing binary training data and model files while keeping the directories themselves tracked (they may contain `.gitkeep` or README files).

- [ ] **3. Create the `models/` directory on the host if it doesn't already have content**

  Verify the host `models/` directory exists (it does — visible in project file list). If empty, add a `.gitkeep` to ensure the bind mount target exists:

  ```bash
  mkdir -p models
  touch models/.gitkeep
  ```

  Rationale: Docker will auto-create the host directory if it doesn't exist, but the directory ownership may be `root`. Pre-creating ensures the host user owns it.

## Verification Criteria

- [ ] Run `make dev` to start the container
- [ ] Inside the container, run `make collect-data` to record some training data
- [ ] Verify data appears on the host: `ls data/*.npz`
- [ ] Run `make train` inside the container
- [ ] Verify model appears on the host: `ls models/classifier.pkl`
- [ ] Run `make down` to stop and remove the container
- [ ] Run `make dev` to start a fresh container
- [ ] Verify data still exists on the host: `ls data/*.npz` and `ls models/classifier.pkl`
- [ ] Run `make emg-infer` and confirm the classifier loads the persisted model successfully

## Potential Risks and Mitigations

1. **File permissions (UID mismatch)**
   The bind mount preserves host file ownership. The container's `prosthesis` user is UID 1000 (same as host user `daniel`). If the host UID differs, the container user may not be able to write.
   Mitigation: The Dockerfile already uses `ARG USER_UID=1000` and the compose file passes `USER_UID: ${USER_UID:-1000}`. The entrypoint (`entrypoint.sh`) also fixes volume ownership via `chown`.

2. **Stale model after code changes**
   If the classifier code changes but the old `classifier.pkl` persists, inference may fail or produce wrong results.
   Mitigation: The `train` target overwrites the model file. Users should re-run `make train` after code changes to the classifier.

3. **Large data files on host**
   Training data `.npz` files can accumulate over time.
   Mitigation: The `.gitignore` entries prevent them from being committed. Users can manually clean `data/` when needed.

## Alternative Approaches

1. **Named Docker volumes** (like `prosthesis-build`): Could use `prosthesis-data:/app/data` and `prosthesis-models:/app/models` instead of bind mounts. However, named volumes are harder to inspect/backup and don't show files on the host filesystem. Bind mounts are preferable here because users can see and manage their training data directly.

2. **Symlinks inside the container**: Could change the EMG scripts to write to `/prosthesis_ws/data` instead of `/app/data`, which is already on a mounted path. This would require changing default paths in multiple Python scripts and the Makefile — more invasive and breaks the existing `/app/data` convention documented in the scripts.
