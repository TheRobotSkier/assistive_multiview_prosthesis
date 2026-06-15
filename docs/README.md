# Multiview Prosthesis Documentation

This directory replaces the old report-style and handoff-style documentation with a single maintained documentation system.

Start here:

- `docs/architecture/overview.md`: End-to-end system architecture and runtime flow.
- `docs/architecture/runtime-entrypoints.md`: Main launch files, configs, and execution modes.
- `docs/architecture/perception.md`: Camera, TF, fusion, segmentation, and twist-based target selection.
- `docs/architecture/control-and-actuation.md`: Pipeline manager, planning, proximity control, hand, wrist, force control, and haptics.
- `docs/architecture/packages.md`: Package-by-package reference across `src/`.
- `docs/reference/directory-structure.md`: Directory map for fast repo navigation.
- `docs/reference/validation-and-workflows.md`: Build, test, and run entry points.
- `docs/reference/documentation-maintenance.md`: Required process for keeping these docs current.

Documentation rules:

- Treat `docs/` as the canonical human/agent documentation for the repo.
- When behavior, topology, or important file locations change, update `docs/` in the same change.
- Keep docs code-based: verify against launch files, package manifests, configs, and node code.
