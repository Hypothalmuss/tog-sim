uint8 IDLE=0
uint8 HOMING=1
uint8 RUNNING=2
uint8 PAUSED=3
uint8 STOPPING=4
uint8 ERROR=5

uint8 MODE_AUTO=0
uint8 MODE_MANUAL=1
uint8 MODE_DRY_RUN=2

std_msgs/Header header
uint8 state
uint8 mode
float32 speed_override             # 0..1
string recipe
string[] active_alarms
