# Fix Invalid Legend Color in visualize_grasp_debug.py

## Objective
Fix the `ValueError: Invalid color name or hex string: red/lime/dodgerblue` crash in the pyvista legend by replacing the single compound-color legend entry with three separate entries, each with a valid single color.

## Root Cause
At `scripts/visualize_grasp_debug.py:1547`, the legend entry `("Signs: in/surf/out", "red/lime/dodgerblue")` passes a slash-separated string of three color names as a single color value. pyvista's `add_legend` calls `Color(color)` which cannot parse this compound string. This is different from line 822 where `cmap=["red", "lime", "dodgerblue"]` is a valid list of colors for a colormap.

## Implementation Plan

- [ ] Replace line 1547's single entry `("Signs: in/surf/out", "red/lime/dodgerblue")` with three separate legend entries: `("TSDF: inside", "red")`, `("TSDF: surface", "lime")`, `("TSDF: outside", "dodgerblue")`.
- [ ] Adjust the legend `size` parameter (currently `(0.18, 0.28)`) to accommodate the two additional entries — increase the height proportionally (e.g., `(0.18, 0.36)`).

## Verification Criteria
- `python scripts/visualize_grasp_debug.py` runs without ValueError
- Legend displays three separate TSDF sign entries with correct colors

## Potential Risks and Mitigations
1. **Legend too tall for viewport**: The legend grows from 10 to 12 entries. The height increase is modest (~28% taller). Mitigation: if it overflows, the `loc="upper left"` anchor keeps it visible; user can also resize the window.

## Alternative Approaches
1. **Use a single representative color**: Replace with `("TSDF signs", "red")` — simpler but loses the color legend information.
2. **Remove the entry entirely**: The scalar bar on the TSDF plot already documents the color mapping — reduces legend clutter but removes at-a-glance reference.
