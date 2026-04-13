# CAN Test command from GCS
uint64 timestamp        # time since system start (microseconds)

uint8 command           # 1=send, 2=set_mode
uint32 can_id           # CAN frame ID (11-bit standard)
uint8 dlc               # Data length (0-8)
uint8[8] data           # CAN frame data
