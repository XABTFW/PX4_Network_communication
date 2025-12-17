uint64 timestamp  # time since system start (microseconds)

uint32 mavid      #mavid
float64 lat       # leader position (deg * 1e7)
float64 lon       # leader position (deg * 1e7)
float64 alt       #leader  altitude (m)
float64 vx	  #leader velocity of vx
float64 vy	  #leader velocity of vy
float64 vz	  #leader velocity of vz
float64 yaw	  #leader yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
float64 yawspeed  #leader yawspeed (only for multirotors, in rad/s)
uint32 land       #ture if leader landing
uint8 at_target   # 1 if aircraft has reached target position (for collision avoidance priority)
