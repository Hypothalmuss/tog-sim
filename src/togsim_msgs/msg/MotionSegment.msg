uint8 PTP_JOINT=0                  # joint-space point-to-point (joint_target used)
uint8 PTP_CART=1                   # Cartesian target, full stop at the end
uint8 VIA_CART=2                   # Cartesian via point, fly-through with via_speed_mps along approach axis
uint8 TRACK_CART=3                 # Cartesian target expressed in a moving frame (tracked_frame), retargeted every tick

uint8 type
float64[] joint_target
geometry_msgs/PoseStamped pose     # for *_CART: TCP pose (frame_id = base or tracked_frame)
float32 tilt_deg
float32 via_speed_mps              # VIA_CART only: velocity magnitude carried through the via point
string tracked_frame               # TRACK_CART only
float32 dwell_s
float32 velocity_scale             # 0..1 (multiplied with global speed override), 0 = use 1.0
