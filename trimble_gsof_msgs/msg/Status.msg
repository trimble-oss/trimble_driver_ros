uint8 imu_alignment

uint8 GPS_ONLY        = 0
uint8 COARSE_LEVELING = 1
uint8 DEGRADED        = 2
uint8 ALIGNED         = 3
uint8 FULL_NAV        = 4
uint8 IMU_UNKNOWN     = 5

uint8 gnss

uint8 FIX_NOT_AVAILABLE          = 0
uint8 GNSS_SPS_MODE              = 1
uint8 DIFFERENTIAL_GPS_SPS       = 2
uint8 GPS_PPS_MODE               = 3
uint8 FIXED_RTK_MODE             = 4
uint8 FLOAT_RTK                  = 5
uint8 DIRECT_GEOREFERENCING_MODE = 6
uint8 GNSS_UNKNOWN               = 7
