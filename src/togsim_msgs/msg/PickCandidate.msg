# One segmented product instance that the robot could pick.
std_msgs/Header header
uint32 id                          # instance id (stable within a frame)
string class_name                  # e.g. "bar", "carton"
float32 score                      # detector confidence 0..1
geometry_msgs/PoseStamped grasp_pose   # TCP target in base frame, z-axis = approach direction
float32 tilt_deg                   # required 5th-axis tilt (0 = vertical grasp)
float32 yaw_deg                    # product principal axis yaw in base frame
float32 suction_margin_mm          # distance-transform max minus cup radius (<0 => cup overhang)
geometry_msgs/Vector3 centre_offset   # product centre minus grasp point, world frame (placement compensates it)
float32 height_m                   # top surface height above belt
float32 mask_area_px
bool occluded                      # something lies on top of it
geometry_msgs/Vector3 belt_velocity    # belt velocity at observation (for ETA extrapolation)
builtin_interfaces/Time observed   # image stamp the candidate was computed from
