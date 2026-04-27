uint64 timestamp  # time since system start (microseconds)

uint8 CMD_NONE = 0       # no command / idle placeholder
uint8 CMD_AUTO_LOCK = 1  # request the payload to auto-lock a target
uint8 CMD_STOP_TRACK = 2 # request the payload to stop tracking
uint8 CMD_RETRIGGER = 3  # request a stop-then-retrigger sequence
uint8 CMD_CENTER_GIMBAL = 4 # point the gimbal to frame azimuth/pitch zero

uint8 command   # command type, uses CMD_* enum
int16 param_x   # protocol-specific X parameter, commonly used as image-space X/offset command
int16 param_y   # protocol-specific Y parameter, commonly used as image-space Y/offset command
uint8 param3    # protocol-specific auxiliary byte parameter
int8 zoom_rate  # protocol-specific zoom rate/setpoint command
