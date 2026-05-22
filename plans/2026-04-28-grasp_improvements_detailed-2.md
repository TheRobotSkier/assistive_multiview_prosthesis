# Three Improvements to Grasp Preshaping — Detailed Implementation Plan

## Objective

1. Show the raw number of active collision/contact points in the debug visualizer
2. Make grasp score weights configurable from `config.rs`
3. Add a multi-finger diversity check to prevent single-finger grasps from passing validation

---

## Part 1: Configurable Grasp Weights (config.rs + planner.rs)

### Task 1.1 — Add weight constants to `config.rs`

In `src/config.rs`, after line 23 (`pub const FIXED_COV_V`), add:

```rust
// Grasp scoring weights
pub const GRASP_WEIGHT_PROBABILITY: f64 = 1.0;
pub const GRASP_WEIGHT_ALIGNMENT: f64 = 1.0;
pub const GRASP_WEIGHT_FORCE_CLOSURE: f64 = 1.0;
pub const GRASP_WEIGHT_CONTACT_COUNT: f64 = 1.5;
```

These are the exact same values currently hardcoded in `planner.rs:42-47`.

### Task 1.2 — Update `GraspWeights::default()` in `planner.rs`

In `src/planner.rs`, change the `GraspWeights::default()` impl (lines 40-48) from:

```rust
impl Default for GraspWeights {
    fn default() -> Self {
        Self {
            w_probability: 1.0,
            w_alignment: 1.0,
            w_force_closure: 1.0,
            w_contact_count: 1.5,
        }
    }
}
```

to:

```rust
impl Default for GraspWeights {
    fn default() -> Self {
        Self {
            w_probability: config::GRASP_WEIGHT_PROBABILITY,
            w_alignment: config::GRASP_WEIGHT_ALIGNMENT,
            w_force_closure: config::GRASP_WEIGHT_FORCE_CLOSURE,
            w_contact_count: config::GRASP_WEIGHT_CONTACT_COUNT,
        }
    }
}
```

---

## Part 2: Export Raw Contact Count (planner.rs + debug_export.rs + c_api.rs + visualize_grasp_debug.py)

### Task 2.1 — Add `active_contact_count` to `GraspScoreResult` in `planner.rs`

In `src/planner.rs`, change `GraspScoreResult` (lines 6-13) from:

```rust
pub struct GraspScoreResult {
    pub closure_amount: f64,
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub contact_count_score: f64,
    pub found_collision: bool,
}
```

to:

```rust
pub struct GraspScoreResult {
    pub closure_amount: f64,
    pub alignment_score: f64,
    pub force_closure_score: f64,
    pub contact_count_score: f64,
    pub active_contact_count: usize,
    pub found_collision: bool,
}
```

### Task 2.2 — Populate `active_contact_count` in all `GraspScoreResult` constructors in `planner.rs`

There are 3 places where `GraspScoreResult` is constructed in `score_grasp` (lines 276-331):

**No collision case** (line 285-291): Add `active_contact_count: 0,`

**Collision at sample 0 case** (handled by `None`, no change needed)

**Collision found case** (lines 323-329): Add `active_contact_count: active.len(),`

### Task 2.3 — Add `active_contact_count` to `ScoredGraspExport` in `debug_export.rs`

In `src/debug_export.rs`, add to the struct (after line 26, before `found_collision`):

```rust
    pub active_contact_count: usize,
```

### Task 2.4 — Expand scored_grasps from 26 to 27 columns in `debug_export.rs`

Update the column count comment (line 62) and the serialization block (lines 193-230).

Change the column comment to say 27 columns, and add the new column. The serialization loop becomes:

```rust
    // --- scored_grasps (f64, M*27 flat) ---
    // Columns per row (27 total):
    //   [0]  sample_index
    //   [1]  grasp_type (1=cyl, 2=pinch, 3=lat)
    //   [2]  closure_amount
    //   [3]  alignment_score
    //   [4]  force_closure_score
    //   [5]  contact_count_score
    //   [6]  active_contact_count
    //   [7]  found_collision (0.0 or 1.0)
    //   [8]  combined_score
    //   [9]  sample_probability
    //   [10]  wrist_rotation (radians)
    //   [11..27]  pose_se3 row-major 4x4
```

In the serialization block, change `let total = m * 26;` to `let total = m * 27;`, update the shape, and add after the `contact_count_score` push:

```rust
    writer.push(&(g.active_contact_count as f64))?;
```

The column order shifts: `found_collision` moves from index 6 to 7, `combined_score` from 7 to 8, etc.

### Task 2.5 — Wire `active_contact_count` through `c_api.rs`

In `src/c_api.rs`, in the debug export construction (around line 383-395), add:

```rust
    active_contact_count: sg.result.active_contact_count,
```

Also update the `GraspScoreResult` construction in the `None` branch (line 199-205) to include `active_contact_count: 0,`.

### Task 2.6 — Update Python visualizer (`scripts/visualize_grasp_debug.py`)

**In `load_dump`** (around line 118-174): Add 27-column format handling. The 27-col format becomes:

```python
    if raw_len > 0:
        if raw_len % 27 == 0:
            row_len = 27
        elif raw_len % 26 == 0:
            row_len = 26
        elif raw_len % 25 == 0:
            row_len = 25
        else:
            row_len = 24
```

For `row_len == 27`, add:

```python
    if row_len == 27:
        grasps = {
            "sample_index": grasps_raw[:, 0].astype(int),
            "grasp_type": grasps_raw[:, 1].astype(int),
            "closure": grasps_raw[:, 2],
            "alignment": grasps_raw[:, 3],
            "force_closure": grasps_raw[:, 4],
            "contact_count_score": grasps_raw[:, 5],
            "active_contact_count": grasps_raw[:, 6].astype(int),
            "found_collision": grasps_raw[:, 7] > 0.5,
            "combined": grasps_raw[:, 8],
            "probability": grasps_raw[:, 9],
            "wrist_rotation": grasps_raw[:, 10],
            "pose_4x4": grasps_raw[:, 11:27].reshape(-1, 4, 4),
        }
```

For older formats (26/25/24), add `"active_contact_count": np.zeros(len(grasps_raw), dtype=int)` as a fallback field.

**In `print_summary`** (around line 299-306): Add `active_contact_count` to the best grasp display:

```python
    print(f"    contacts={grasps['active_contact_count'][best_idx]}"
          f"  closure={grasps['closure'][best_idx]:.4f}"
          f"  alignment={grasps['alignment'][best_idx]:.4f}"
          f"  force_closure={grasps['force_closure'][best_idx]:.4f}"
          f"  combined={grasps['combined'][best_idx]:.4f}"
          f"  prob={grasps['probability'][best_idx]:.4f}")
```

---

## Part 3: Multi-Finger Diversity Check (lut_helper.rs + planner.rs)

### Task 3.1 — Add `finger_group` method to `Contact` in `lut_helper.rs`

Add a `FingerGroup` enum and a `finger_group()` method to `Contact`. In `src/lut_helper.rs`, after the `Contact` enum (after line 185), add:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FingerGroup {
    Thumb,
    Index,
    Middle,
    Ring,
    Little,
    Palm,
}

impl Contact {
    pub fn finger_group(self) -> FingerGroup {
        match self {
            Contact::ThumbAddPip | Contact::ThumbAddDip | Contact::ThumbAddTip
            | Contact::ThumbAbdPip | Contact::ThumbAbdDip | Contact::ThumbAbdTip => FingerGroup::Thumb,
            Contact::IndexPip | Contact::IndexPipSide | Contact::IndexMcp
            | Contact::IndexMcpSide | Contact::IndexDip | Contact::IndexDipSide
            | Contact::IndexTip | Contact::IndexTipSide => FingerGroup::Index,
            Contact::MiddleMcp | Contact::MiddlePip | Contact::MiddleDip
            | Contact::MiddleTip => FingerGroup::Middle,
            Contact::RingDip | Contact::RingPip | Contact::RingTip => FingerGroup::Ring,
            Contact::LittleDip | Contact::LittlePip | Contact::LittleTip => FingerGroup::Little,
            Contact::PalmProxUlna | Contact::PalmProxRadi
            | Contact::PalmDistUlna | Contact::PalmDistRadi => FingerGroup::Palm,
        }
    }
}
```

### Task 3.2 — Add `min_fingers` to `GraspSpec` and update grasp specs in `planner.rs`

In `src/planner.rs`, add a field to `GraspSpec` (after line 73):

```rust
    /// Minimum number of distinct finger groups that must have active contacts.
    min_fingers: usize,
```

Update the import to include `FingerGroup`:
```rust
use crate::lut_helper::{Contact, FingerGroup, FingerLUT};
```

Update each spec constructor:

- `cylindrical_spec()`: Add `min_fingers: 2,` (thumb + at least one other finger)
- `pinch_spec()`: Add `min_fingers: 2,` (thumb + index)
- `lateral_spec()`: Add `min_fingers: 2,` (thumb + index side)

### Task 3.3 — Add finger diversity check in `score_grasp`

In `src/planner.rs`, in the `score_grasp` function, after computing `active` contacts and before computing the score (around lines 309-322), add a finger diversity check.

Change `find_active_contacts` to also return which contacts are active:

```rust
fn find_active_contacts(
    lut: &FingerLUT,
    tsdf: &Tsdf,
    base: &Matrix4<f64>,
    score_contacts: &[Contact],
    lo_ctrl: f64,
    hi_ctrl: f64,
    collision_tol: f32,
) -> Vec<(Contact, ActiveContact)> {
    let mut active = Vec::new();
    let threshold = collision_tol * 2.0;

    for &contact in score_contacts {
        let p_hi = pos_at_control(lut, contact, hi_ctrl, base);
        let dist = tsdf.get_distance(p_hi.x, p_hi.y, p_hi.z);
        if dist >= threshold {
            continue;
        }

        let normal = tsdf.get_surface_normal(p_hi.x, p_hi.y, p_hi.z);
        let p_lo = pos_at_control(lut, contact, lo_ctrl, base);

        let fd = Vector3::new(
            p_hi.x as f64 - p_lo.x as f64,
            p_hi.y as f64 - p_lo.y as f64,
            p_hi.z as f64 - p_lo.z as f64,
        );
        let norm = fd.norm();
        let force_dir = if norm > 1e-10 {
            fd / norm
        } else {
            Vector3::zeros()
        };

        active.push((contact, ActiveContact {
            surface_normal: normal,
            force_direction: force_dir,
        }));
    }

    active
}
```

Then in `score_grasp`, after the `find_active_contacts` call, add the finger diversity check:

```rust
            let active_with_contacts = find_active_contacts(
                lut,
                tsdf,
                base_transform,
                &spec.score_contacts,
                lo,
                hi,
                collision_tol,
            );

            // Check finger diversity — reject grasps where contacts are
            // concentrated on too few fingers.
            let mut finger_set = std::collections::HashSet::new();
            for &(contact, _) in &active_with_contacts {
                finger_set.insert(contact.finger_group());
            }
            if finger_set.len() < spec.min_fingers {
                return None;
            }

            let active: Vec<_> = active_with_contacts.into_iter().map(|(_, ac)| ac).collect();
            let contact_count_score = if active.is_empty() {
                0.0
            } else {
                (active.len() as f64 / spec.min_contacts as f64).min(1.0)
            };
```

Note: `use std::collections::HashSet;` needs to be added at the top of `planner.rs`, or use `use std::collections::HashSet;` inline. Since `HashSet` is from std, just add it to the imports.

Also need to add `use std::collections::HashSet;` at the top of `planner.rs`.

---

## Verification

After all changes:

1. Run `cargo build` to verify Rust compilation
2. Run `cargo test` to verify existing tests pass
3. Run the debug dump example (`cargo run --example debug_dump`) to generate a new npz
4. Run the visualizer to confirm contact count shows in the summary
5. Verify backward compatibility by loading an old 26-column npz
