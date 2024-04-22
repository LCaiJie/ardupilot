# include file to reduce flash by including less GPS drivers

define AP_GPS_BACKEND_DEFAULT_ENABLED 0
define AP_GPS_UBLOX_ENABLED 1
undef HAL_MSP_GPS_ENABLED
define HAL_MSP_GPS_ENABLED HAL_MSP_SENSORS_ENABLED
