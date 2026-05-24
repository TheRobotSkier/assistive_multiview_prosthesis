# Fix `make test` Exit Code 126 — ENTRYPOINT Conflict

## Objective

Fix the `make test` command which fails with exit code 126 (`/bin/bash: cannot execute binary file`) by resolving a Docker ENTRYPOINT/command conflict.

## Root Cause Analysis

The failure is caused by an interaction between `ENTRYPOINT` and `command`:

1. `docker/Dockerfile:65` sets `ENTRYPOINT ["/bin/bash"]`
2. `docker/docker-compose.yml:70` sets `command: ["/bin/bash", "/prosthesis_ws/scripts/run_tests.sh"]`
3. Docker/Podman concatenates ENTRYPOINT + command, producing the effective command:
   ```
   /bin/bash /bin/bash /prosthesis_ws/scripts/run_tests.sh
   ```
4. The first `/bin/bash` (from ENTRYPOINT) runs and tries to open the second `/bin/bash` (first element of command) as a script file. Since `/bin/bash` is a compiled ELF binary, not a text script, it fails with `cannot execute binary file` (exit code 126).

## Why `CMD` is the Correct Fix

- `CMD` is **overridden** by `command:` in docker-compose — so the test service gets exactly the command it specifies.
- `ENTRYPOINT` is **prepended** to `command:` — which is what causes the bug.
- For an interactive development container, `CMD ["/bin/bash"]` is the standard pattern. It gives a shell by default when no command is specified (`make up`, `make shell`), but allows `command:` to fully replace it (`make test`).

## Implementation Plan

- [ ] Change `ENTRYPOINT ["/bin/bash"]` to `CMD ["/bin/bash"]` on line 65 of `docker/Dockerfile`
- [ ] Rebuild the prosthesis image: `make build-prosthesis`
- [ ] Run tests: `make test`

## Verification Criteria

- [ ] `make test` exits with code 0 (all smoke tests pass)
- [ ] `make up` still starts an interactive container with a bash shell
- [ ] `make shell` still opens a bash session in the running container

## Potential Risks and Mitigations

1. **`make shell` uses `exec` which passes the command to the running container's shell, not through ENTRYPOINT.** No risk — `make shell` is unaffected by this change.
2. **`make up` with `stdin_open: true` and `tty: true`** still gets an interactive bash shell via the default CMD. No risk — behavior is identical.

## Alternative Approaches

1. **Keep ENTRYPOINT, change the test command to `-c "..."`**: e.g., `command: ["-c", "/prosthesis_ws/scripts/run_tests.sh"]`. This works but is fragile — every compose service that overrides the command must remember to drop the leading `/bin/bash`. Worse developer experience.
2. **Add an `entrypoint:` override to the test service**: e.g., `entrypoint: []` in docker-compose.yml. This clears the ENTRYPOINT for the test service only. Works, but leaves the underlying anti-pattern in the Dockerfile for any future service.
