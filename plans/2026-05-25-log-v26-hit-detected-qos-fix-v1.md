# Fix QoS Mismatch on `/twist_propagation/hit_detected`

## Objective

Fix the DURABILITY QoS incompatibility between the twist propagation publisher and the pipeline manager subscriber on the `/twist_propagation/hit_detected` topic. This is the root cause of the hand/wrist not moving after preshaping in log v26.

## Root Cause

The pipeline manager subscribes with `QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)` at `pipeline_manager_node.py:238`, but the twist propagation publisher was created with default QoS (VOLATILE durability) at `twist_propagation_node.py:720-721`. ROS 2 silently drops all messages when QoS policies are incompatible.

## Implementation Plan

- [ ] Fix 1. Update `_hit_detected_pub` publisher QoS to use `TRANSIENT_LOCAL` durability in `twist_propagation_node.py:720-721`
  - Change `self.create_publisher(Bool, "/twist_propagation/hit_detected", 10)` to use `QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)` instead of bare `10`
  - The `QoSProfile` and `DurabilityPolicy` imports already exist at line 64

## Verification Criteria

- [ ] No "incompatible QoS" warnings in log for `/twist_propagation/hit_detected`
- [ ] Pipeline manager receives hit detection events and transitions state
- [ ] Hand/wrist movement is triggered after preshaping

## Potential Risks and Mitigations

1. **Late-joining subscribers may miss messages**
   Mitigation: TRANSIENT_LOCAL durability ensures late-joining subscribers receive the last published value, which is the desired behavior for hit detection state

## Alternative Approaches

1. Change the pipeline manager subscription to VOLATILE: This would work but would lose the late-joiner guarantee that TRANSIENT_LOCAL provides
