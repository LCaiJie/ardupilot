# this include file is used to remove features which will never be
#  wanted on any low-flash board in our standard builds.  It is to be
#  included by other minimize_*.inc files and not generally used
#  otherwise.

# disable emitting nice strings when activating RC channel aux functions:
define AP_RC_CHANNEL_AUX_FUNCTION_STRINGS_ENABLED 0

# low-end boards aren't expected to be used in environments where
# things like satellite communications are required:
define HAL_HIGH_LATENCY2_ENABLED 0

# disable use of onboard FFT library:
define HAL_GYROFFT_ENABLED 0

# several notify backends are rare and not supported by default on smaller boards:


# smaller boards lose some GPS support
define AP_GPS_BACKEND_DEFAULT_ENABLED 0
define AP_GPS_UBLOX_ENABLED 1

# no moving baseline support:
define GPS_MOVING_BASELINE 0

# No LTM telemetry on minimized boards:
define AP_LTM_TELEM_ENABLED 0

# various structures increase the flash size when using >16 servos:
define NUM_SERVO_CHANNELS 16

# prune out some odd camera backends:
# no SLCAN on these boards (use can-over-mavlink if required)


# restricted battery backends:
define AP_BATTERY_BACKEND_DEFAULT_ENABLED 0
define AP_BATTERY_ANALOG_ENABLED 1
define AP_BATTERY_WATT_MAX_ENABLED 0
define AP_BATTERY_SUM_ENABLED 1
define AP_BATTERY_SYNTHETIC_CURRENT_ENABLED 1
define AP_BATTERY_SMBUS_ENABLED 1
define AP_BATTERY_SMBUS_GENERIC_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_NEODESIGN_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_SUI_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_MAXELL_ENABLED AP_BATTERY_SMBUS_ENABLED
define AP_BATTERY_SMBUS_ROTOYE_ENABLED AP_BATTERY_SMBUS_ENABLED

# don't probe for external Barometers:
define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 0

# no wind compensation code:
define HAL_BARO_WIND_COMP_ENABLED 0


# no Notify Display support:


# remove support for killing IMUs at runtime - a developer feature:
define AP_INERTIALSENSOR_KILL_IMU_ENABLED 0

# shortened names in @SYS/taskinfo.txt
define AP_SCHEDULER_EXTENDED_TASKINFO_ENABLED 0

define AP_LANDINGGEAR_ENABLED APM_BUILD_COPTER_OR_HELI

# Plane-specific defines; these defines are only used in the Plane
#  directory, but are seen across the entire codebase:
define OFFBOARD_GUIDED 0
define QAUTOTUNE_ENABLED 0

# Copter-specific defines; these defines are only used in the Copter
#  directory, but are seen across the entire codebase:
define MODE_ZIGZAG_ENABLED 0
define AC_NAV_GUIDED 0
define AC_OAPATHPLANNER_ENABLED 0
define MODE_FOLLOW_ENABLED 0
define MODE_GUIDED_NOGPS_ENABLED 0
define MODE_SYSTEMID_ENABLED 0
define WEATHERVANE_ENABLED 0
define MODE_AUTOROTATE_ENABLED 0


# don't need payload place mission items on these boards, it's very niche:
define AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED 0
# don't need the payload place flight behaviour either:
define AC_PAYLOAD_PLACE_ENABLED 0
