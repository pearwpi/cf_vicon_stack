uint16 SUPERVISOR_INFO_CAN_BE_ARMED = 1
uint16 SUPERVISOR_INFO_IS_ARMED     = 2
uint16 SUPERVISOR_INFO_AUTO_ARM     = 4
uint16 SUPERVISOR_INFO_CAN_FLY      = 8
uint16 SUPERVISOR_INFO_IS_FLYING    = 16
uint16 SUPERVISOR_INFO_IS_TUMBLED   = 32
uint16 SUPERVISOR_INFO_IS_LOCKED    = 64

uint8 PM_STATE_BATTERY   = 0
uint8 PM_STATE_CHARGING  = 1
uint8 PM_STATE_CHARGED   = 2
uint8 PM_STATE_LOW_POWER = 3
uint8 PM_STATE_SHUTDOWN  = 4

std_msgs/Header header
uint16 supervisor_info   # Bitfield, see SUPERVISOR_INFO for individual bits
float32 battery_voltage  # internal battery voltage [V]
uint8 pm_state           # See PM_STATE_* for potential values
uint8 rssi               # latest radio signal strength indicator [dBm]
uint32 num_rx_broadcast
uint32 num_tx_broadcast
uint32 num_rx_unicast
uint32 num_tx_unicast
uint16 latency_unicast   # latency (in ms) for unicast packets
