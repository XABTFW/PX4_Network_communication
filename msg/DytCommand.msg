uint64 timestamp  # time since system start (microseconds)

uint8 CMD_NONE = 0
uint8 CMD_AUTO_LOCK = 1
uint8 CMD_STOP_TRACK = 2
uint8 CMD_RETRIGGER = 3
uint8 CMD_NOFOLLOW = 4
uint8 CMD_CENTER = 5
uint8 CMD_YAW_FOLLOW = 6
uint8 CMD_ELECT_LOCK = 7
uint8 CMD_ELECT_UNLOCK = 8
uint8 CMD_LOCK_VIEW = 9
uint8 CMD_CENTER_GIMBAL = 10

uint8 command
int16 param_x
int16 param_y
uint8 param3
int8 zoom_rate
