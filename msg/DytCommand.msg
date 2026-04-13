uint64 timestamp  # time since system start (microseconds)

uint8 CMD_NONE = 0
uint8 CMD_AUTO_LOCK = 1
uint8 CMD_STOP_TRACK = 2
uint8 CMD_RETRIGGER = 3

uint8 command
int16 param_x
int16 param_y
uint8 param3
int8 zoom_rate
