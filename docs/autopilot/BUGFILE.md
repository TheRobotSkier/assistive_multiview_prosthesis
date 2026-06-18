# Autopilot Bugfile

Repository-specific debugging memory for Autopilot-managed work.

Each entry records error-message fixes that should help future agents avoid rediscovering the same root cause.

## Entry format

```markdown
### <symptom or error text>

- Date: YYYY-MM-DD
- Batch: <autopilot batch number>
- Beads: <bead ids>
- Components/files: `<path>`, `<path>`
- Root cause: <why it failed>
- Fix: <what changed>
- Verification: `<command>` → <observed result>
- Related: <similar entries or none>
```

## Entries

No Autopilot bugfix entries recorded yet.
