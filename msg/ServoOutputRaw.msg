Header header

# Timestamp (microseconds since system boot)
uint32 time_usec

# Value of servo outputs 1-8, in microseconds
uint16 servo1_raw
uint16 servo2_raw
uint16 servo3_raw
uint16 servo4_raw
uint16 servo5_raw
uint16 servo6_raw
uint16 servo7_raw
uint16 servo8_raw

# Servo output port (set of 8 outputs = 1 port)
# Most MAVs will just use one, but this allows to encode more than 8 servos
uint8 port
