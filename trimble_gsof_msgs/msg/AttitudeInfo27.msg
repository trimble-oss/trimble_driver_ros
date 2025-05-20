std_msgs/Header header

uint32 gps_time_of_week
uint8 flags
uint8 number_space_vehicles
uint8 calculation_mode
EulerAngle attitude
float64 master_slave_range_meters
float32 pdop
AttitudeVariance variance

uint8 CALC_MODE_NONE = 0
uint8 CALC_MODE_AUTONOMOUS = 1
uint8 CALC_MODE_RTK_FLOAT = 2
uint8 CALC_MODE_RTK_FIX = 3
uint8 CALC_MODE_DGPS = 4