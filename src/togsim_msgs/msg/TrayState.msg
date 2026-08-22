# Tray detection + pocket occupancy grid.
std_msgs/Header header
uint32 tray_id
geometry_msgs/PoseStamped pose     # tray origin (pocket (0,0) corner frame) in base frame
geometry_msgs/Vector3 velocity     # estimated tray velocity (belt) m/s
uint8 rows
uint8 cols
float32 pitch_x
float32 pitch_y
bool[] occupied                    # row-major, rows*cols
float32[] confidence
bool tracked                       # false if estimate is stale / lost
string model                       # tray model name (trays.yaml key), e.g. tray_2x4 / tray_bar_2x3
