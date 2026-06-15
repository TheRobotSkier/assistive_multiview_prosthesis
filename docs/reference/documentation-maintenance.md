# Documentation Maintenance Procedure

This file defines the required process for updating documentation after code changes.

## When Documentation Must Be Updated

Update documentation in the same change whenever any of the following changes:

- launch composition
- package responsibilities
- major runtime flow
- topics, services, actions, or TF frames used across package boundaries
- main configuration file structure or important parameters
- build, test, or operator workflows
- directory layout used for navigation

## Required Update Procedure

Follow these steps in order.

1. Identify the changed subsystem.
   Use `docs/reference/directory-structure.md` to find the right area.

2. Verify behavior in code, not old docs.
   Read the relevant launch files, package manifests, config, and node implementations.

3. Update the subsystem architecture file.
   Usually one of:
   - `docs/architecture/overview.md`
   - `docs/architecture/runtime-entrypoints.md`
   - `docs/architecture/perception.md`
   - `docs/architecture/control-and-actuation.md`
   - `docs/architecture/packages.md`

4. Update the navigation map if file locations or ownership changed.
   Edit `docs/reference/directory-structure.md`.

5. Update validation guidance if workflows changed.
   Edit `docs/reference/validation-and-workflows.md`.

6. Update repo instructions for agents if the change affects how future agents should work in the repo.
   Edit `agents.md`.

7. Cross-check references.
   Confirm that file paths, launch filenames, package names, and important topic/service names match the code.

8. Remove stale statements instead of stacking add-on notes.
   Do not leave old behavior documented next to new behavior unless both are intentionally supported.

9. Verify docs changed with the code.
   Before finishing, inspect the diff and confirm the documentation updates cover the modified behavior.

## Minimum Files To Review Before Finishing A Feature

For most behavior changes, review at least:

- `docs/architecture/overview.md`
- one subsystem file under `docs/architecture/`
- `docs/reference/directory-structure.md`
- `agents.md`

## Documentation Style Rules

- Prefer architecture facts over historical narrative.
- Prefer file paths over vague descriptions.
- Keep package ownership explicit.
- Keep docs short enough to navigate quickly, but concrete enough to act on.
- If code and docs conflict, fix the docs immediately.
