# CAN Test status to GCS
uint64 timestamp        # time since system start (microseconds)

uint32 can_id           # Received CAN frame ID
uint8 dlc               # Data length
uint8[8] data           # CAN frame data
uint64 rx_count         # Total received frames
uint64 tx_count         # Total transmitted frames
