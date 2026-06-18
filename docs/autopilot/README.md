# Autopilot Agent Memory

This directory stores repository-specific knowledge maintained by the OMP Autopilot workflow.

## Files

- `BUGFILE.md`: aggregated error-message fixes, root causes, and verification evidence from Autopilot batches.

## Maintenance contract

- Update after each accepted Autopilot worker batch through the `autopilot-librarian` agent.
- Record only accepted/merged behavior as current docs.
- Preserve exact error text when available.
- Aggregate similar fixes instead of duplicating entries.
- Link entries to bead ids, batch number, files, and verification commands.
