# Fix `CARGO_MANIFEST_DIR` Hardcoded Paths for Docker Compatibility

## Objective

The Rust `.so` (`libgrasp_preshaping.so`) is pre-built on the host machine where `CARGO_MANIFEST_DIR` is `/home/daniel/multiview_prosthesis/src/grasp_preshaping`. This path is baked into the binary at compile time via `env!()`. Inside Docker, the source lives at `/prosthesis_ws/src/grasp_preshaping/`, so the hardcoded paths don't exist, causing panics when the LUT data file is loaded.

## Affected Locations

Three files use `CARGO_MANIFEST_DIR`:

| File | Line | Usage | Severity |
|------|------|-------|----------|
| `src/grasp_preshaping/src/c_api.rs` | 333 | `concat!(env!("CARGO_MANIFEST_DIR"), "/data/finger_contact_lut.npz")` | **Critical** — panics, kills service calls |
| `src/grasp_preshaping/src/debug_export.rs` | 293 | `env!("CARGO_MANIFEST_DIR")` for debug output directory | Low — only when debug_visualization=true |
| `src/grasp_preshaping/src/runtime_config.rs` | 206 | `option_env!("CARGO_MANIFEST_DIR")` for config path | None — already has `GRASP_CONFIG_PATH` env var fallback |

## Root Cause

`env!("CARGO_MANIFEST_DIR")` is a compile-time macro in Rust. It embeds the literal string from the build environment into the binary. It cannot be overridden at runtime. When the `.so` is moved to a different filesystem layout (Docker), the path is wrong.

## Strategy

Create a shared helper function that resolves the "crate root" directory at runtime, with fallbacks:

1. **`GRASP_PRESHAPING_HOME` env var** — explicit override (set in Dockerfile)
2. **`/prosthesis_ws/src/grasp_preshaping`** — Docker default path
3. **`CARGO_MANIFEST_DIR`** — compile-time fallback (works on host)

This matches the pattern already used in `runtime_config.rs` for config loading, and extends it to the LUT and debug paths.

## Implementation Plan

- [ ] **1. Add `GRASP_PRESHAPING_HOME` env var to Dockerfile.** In `docker/Dockerfile`, add `ENV GRASP_PRESHAPING_HOME=/prosthesis_ws/src/grasp_preshaping` to the runtime ENV block (near line 56-58). This provides the Docker-specific override.

- [ ] **2. Create a `crate_root()` helper function.** In `src/grasp_preshaping/src/c_api.rs`, add a function that resolves the crate root directory at runtime:
  - First checks `std::env::var("GRASP_PRESHAPING_HOME")` — if set, use it
  - Then checks if the compile-time `CARGO_MANIFEST_DIR` path exists on disk — if so, use it
  - Falls back to `CARGO_MANIFEST_DIR` as a last resort

- [ ] **3. Update `get_lut()` in `c_api.rs:333`.** Replace the `concat!(env!(...))` with a call to the new `crate_root()` helper, constructing the data path at runtime: `format!("{}/data/finger_contact_lut.npz", crate_root())`.

- [ ] **4. Update `debug_output_path()` in `debug_export.rs:293`.** Replace `env!("CARGO_MANIFEST_DIR")` with the same `crate_root()` helper. Since `debug_export.rs` can't directly call a function from `c_api.rs` (circular module dependency risk), either:
  - Move the helper to a shared module (e.g., `runtime_config.rs` which already handles path resolution), or
  - Duplicate the small helper in `debug_export.rs`
  - **Recommended**: Add the helper to `runtime_config.rs` as `pub fn crate_root() -> &'static str` (using `OnceLock` for single initialization), then call it from both `c_api.rs` and `debug_export.rs`.

- [ ] **5. Rebuild the Rust `.so`.** After code changes, rebuild on the host:
  ```bash
  cd src/grasp_preshaping
  cargo build --release --lib
  cp target/release/libgrasp_preshaping.so lib/
  ```

- [ ] **6. Rebuild Docker image.** `make build && make up`

- [ ] **7. Verify inside container.** Run the smoke test or the Tier B test:
  ```bash
  python3 /prosthesis_ws/tests/test1_software_verification/run_tier_b.py --method service
  ```
  Using `--method service` avoids the EMG path and directly calls the compute service, which is the simplest way to verify the `.so` works.

## Verification Criteria

- [ ] `strings /prosthesis_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so | grep finger_contact_lut` still shows the compile-time path (it's in the binary), but the runtime code no longer relies solely on it
- [ ] Inside the container, `python3 -c "import ctypes; so = ctypes.CDLL('/prosthesis_ws/install/grasp_preshaping/lib/libgrasp_preshaping.so'); print(so.grasp_preshaping_api_version())"` works (library loads)
- [ ] Inside the container, the Tier B test with `--method service` returns latency values instead of TIMEOUT
- [ ] On the host (outside Docker), the Tier A test `python3 tests/test1_software_verification/run.py` still works (no regression)

## Potential Risks and Mitigations

1. **`OnceLock` ordering**: The `crate_root()` function uses `OnceLock` and is called from `get_lut()` which is also behind a `OnceLock`. No deadlock risk since they are independent locks.
   Mitigation: Keep `crate_root()` simple — just string resolution, no I/O beyond `std::env::var`.

2. **Path with trailing slash**: `std::env::var` might return a path with or without trailing slash.
   Mitigation: Use `std::path::PathBuf` join instead of string concatenation for robustness.

3. **Debug export path**: If `debug_visualization=true` and the debug output path resolves to a non-writable directory inside Docker, the export will fail.
   Mitigation: The `export_npz` function already handles `create_dir_all` errors gracefully. The Dockerfile sets `user: prosthesis` which owns `/prosthesis_ws/`.

4. **Host regression**: Changing from compile-time to runtime path resolution adds a small overhead (one env var lookup, one path existence check).
   Mitigation: `OnceLock` ensures this happens exactly once. The overhead is negligible.

## Alternative Approaches

1. **Build the `.so` inside Docker**: Add Rust toolchain to the Dockerfile and build during `colcon build`. This makes `CARGO_MANIFEST_DIR` correct inside the container.
   - Pros: Eliminates the path mismatch entirely
   - Cons: Significantly increases Docker build time (Rust compilation is slow), increases image size (Rust toolchain), and the current design explicitly avoids this (see CMakeLists.txt comment: "No Rust toolchain is needed in the Docker image")

2. **Symlink inside Docker**: Create a symlink from the host path to the Docker path.
   - Pros: No code changes
   - Cons: Fragile, requires knowing the exact host path at Docker build time, breaks if host path changes

3. **Use `GRASP_DATA_DIR` env var for LUT path only**: Instead of a general crate root resolver, add a specific env var just for the LUT file.
   - Pros: Minimal change
   - Cons: Doesn't fix `debug_export.rs`, and adding more env vars per path is not scalable
