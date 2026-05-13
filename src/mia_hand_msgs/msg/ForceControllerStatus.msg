# Force Controller Status — published by the force_controller node.
#
# Provides real-time feedback on force regulation state for monitoring
# and for triggering pipeline state transitions (GRASPING -> HOLDING).

bool active                      # True when force regulation is engaged (GRASPING/HOLDING)
float64[3] current_normal_forces # Latest normal force per finger [thumb, index, mrl] (raw sensor units)
float64[3] current_tangential_forces # Latest tangential force per finger [thumb, index, mrl] (raw sensor units)
float64[3] force_errors          # Error (target - actual) per finger [thumb, index, mrl]
bool force_stable                # True when all fingers within tolerance for stability_window seconds
bool slip_detected               # True when tangential force rate-of-change exceeds threshold
string state                     # Human-readable: "IDLE", "APPROACHING", "GRASPING", "HOLDING", "RELEASING"
