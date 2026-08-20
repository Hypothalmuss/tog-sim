uint8 OK=0
uint8 NO_SEAL=1
uint8 DROPPED=2
uint8 NO_CANDIDATE=3
uint8 NO_POCKET=4
uint8 UNREACHABLE=5
uint8 ABORT=6
uint8 MISSED=7

std_msgs/Header header
uint32 cycle_id
uint8 result
string product_class
uint32 tray_id
uint8 pocket_index
float32 t_pick_s
float32 t_transfer_s
float32 t_place_s
float32 cycle_s
float32 placement_error_mm         # from ground truth, sim only (-1 if unknown)
