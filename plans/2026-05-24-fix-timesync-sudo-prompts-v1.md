# Fix `make timesync` sudo Password Prompts Being Ignored

## Objective

Fix the `make timesync` target so that sudo authentication on the Jetson works reliably, instead of showing password prompts that are ignored and failing silently.

## Root Cause

The `timesync` target makes **4 separate SSH connections** to the Jetson, each running `echo robotlab | sudo -S` commands (6+ sudo calls total). This fails because:

1. **Per-session sudo caching**: Each `ssh` opens a new session with a new TTY. `sudo` caches credentials per-TTY, so the auth from session 1 does not carry over to sessions 2-4.
2. **TTY vs pipe conflict**: SSH allocates a pseudo-terminal by default. `sudo` tries to read the password from `/dev/tty` (the terminal device) rather than stdin (the pipe). The piped password `robotlab` is never consumed by sudo, and the user can't type it interactively because stdin is the pipe.
3. **Silent failures**: `2>/dev/null` on line 325 hides all error output, making the failure invisible.

## Implementation Plan

- [x] **1. Consolidate all Jetson SSH commands into a single session**

  Replace lines 321-332 in `Makefile` with a single `ssh -T` invocation using a heredoc. This ensures:
  - Only one sudo authentication needed (cached for subsequent calls in the same session)
  - `-T` prevents TTY allocation, forcing `sudo -S` to read from stdin (the pipe)
  - All commands share the same sudo timestamp

  The new `timesync` recipe should be structured as:
  - First `ssh -T` call: single-session script that does all Jetson-side work (date sync, chrony install, config, restart)
  - The `scp` for chrony config must happen *before* the consolidated SSH call, or the config can be embedded inline via `cat` heredoc on the SSH session itself

  Key design: use `ssh -T $(JETSON_HOST) 'bash -s' <<'REMOTE_SCRIPT'` with `sudo -S -v` as the first authenticated command, then all subsequent `sudo` calls without `-S` (they reuse the cached credential).

- [x] **2. Remove `2>/dev/null` from the date-sync SSH command (line 325)**

  Replace with targeted suppression of only the sudo password prompt noise, e.g., `2>&1 | grep -v 'password for'` or just let errors show. Silent failures are worse than noisy successes.

- [x] **3. Handle the chrony config transfer without a separate SSH session**

  Instead of `scp` + separate `ssh` to copy, embed the chrony config content directly in the SSH heredoc:
  ```
  cat config/chrony-jetson.conf | ssh -T $(JETSON_HOST) 'cat > /tmp/chrony-jetson.conf && sudo -S ...'
  ```
  Or use a single heredoc that includes both the file content and the commands.

- [x] **4. Add error handling to the consolidated script**

  Add `set -e` at the top of the remote script so any failed command aborts the whole operation, rather than continuing with a partially-configured state.

- [x] **5. Keep the `robotlab-connect` prerequisite**

  The `robotlab-connect` dependency on line 320 should remain as-is — it verifies network connectivity before attempting SSH.

## Verification Criteria

- `make timesync` completes without any interactive password prompts
- `make timesync-check` shows host and Jetson clocks within ~1 second of each other
- Running `make timesync` twice in a row both succeed (idempotent)
- If the Jetson is unreachable, the target fails with a clear error message (not silently)

## Potential Risks and Mitigations

1. **`sudo -S -v` may not work on all sudo versions**
   Mitigation: `sudo -S -v` is standard across all modern sudo versions (1.7+). The Jetson runs Ubuntu which ships a compatible version.

2. **Heredoc variable expansion in Makefile**
   Mitigation: Use `<<'REMOTE_SCRIPT'` (quoted) to prevent the local shell/Make from expanding variables inside the heredoc. Pass the epoch timestamp as an argument or environment variable to the remote script instead.

3. **`ssh -T` may break some sudo configurations that require a TTY**
   Mitigation: Use `ssh -T` with `sudo -S` (which explicitly reads from stdin). If `requiretty` is set in sudoers, this would fail, but that's uncommon on Ubuntu/Debian.

4. **Hardcoded password visibility**
   Mitigation: This is an existing issue, not introduced by this fix. A future improvement could use `SSH_ASKPASS` or sudoers NOPASSWD for the specific commands.

## Alternative Approaches

1. **Use `ssh -T` on each individual call (minimal change)**: Add `-T` to each `ssh` call. This fixes the TTY-vs-pipe issue but still requires re-authentication per session. Simpler but less reliable.
2. **Configure sudoers NOPASSWD for robotlab user**: Add `robotlab ALL=(ALL) NOPASSWD: /bin/date, /usr/bin/systemctl, ...` on the Jetson. Most secure and reliable, but requires one-time Jetson-side configuration.
3. **Use `SSH_ASKPASS` environment variable**: Point to a script that echoes the password. Works with `ssh -T` and avoids the pipe issue entirely, but more complex to set up.
