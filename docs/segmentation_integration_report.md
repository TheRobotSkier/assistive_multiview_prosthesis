# Segmentation Integration Report

## Scope

Checked and tightened the segmentation bridge contract used by the automatic
collision-click path.

## Interface

Inputs:

- `/segmentation/input_cloud` from `/fused_pointcloud`
- `/segmentation/click_positive` from collision prediction or RViz click relay
- `/segmentation/click_negative` for manual negative seeds

Output:

- `/segmentation/object_cloud`

The bridge sends XYZ/RGB arrays plus positive/negative click lists to:

```text
POST /segment
```

on the configured inference server.

## Fix

- Output headers now use deep copies, so publishing a segmented cloud does not
  mutate the stored source-cloud header.
- The bridge now rejects inference masks whose length does not match the input
  cloud point count.

## Tests

Passed locally:

```bash
bash scripts/test_segmentation_contracts.sh
```
