uint64 timestamp  # time since system start (microseconds)

uint8 ORB_QUEUE_LENGTH = 8

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
uint8 CMD_SET_FRAME_ANGLE = 11
uint8 CMD_SEARCH_RATE = 12
uint8 CMD_SEND_OWNSHIP_STATE = 13
uint8 CMD_GEO_TRACK = 14
uint8 CMD_GEO_TRACK_EXIT = 15

uint8 command
int16 param_x
int16 param_y
uint8 param3
int8 zoom_rate

# Used by CMD_SEND_OWNSHIP_STATE and CMD_GEO_TRACK.
float64 lat
float64 lon
float32 alt
float32 rel_alt
float32 roll_rad
float32 pitch_rad
float32 yaw_rad
float32 airspeed_m_s
float32 groundspeed_m_s
