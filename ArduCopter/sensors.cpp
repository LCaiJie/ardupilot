#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;
}

void Copter::init_rangefinder(void)
{
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
    // downward facing rangefinder
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;

    // upward facing rangefinder
    rangefinder_up_state.enabled = false;
    rangefinder_up_state.alt_healthy = false;
    rangefinder_up_state.alt_cm = 0;
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return (rangefinder_up_state.enabled && rangefinder_up_state.alt_healthy);
}

// update rangefinder based terrain offset
// terrain offset is the terrain's height above the EKF origin
void Copter::update_rangefinder_terrain_offset()
{
    float terrain_offset_cm = rangefinder_state.inertial_alt_cm - rangefinder_state.alt_cm_glitch_protected;
    rangefinder_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    terrain_offset_cm = rangefinder_up_state.inertial_alt_cm + rangefinder_up_state.alt_cm_glitch_protected;
    rangefinder_up_state.terrain_offset_cm += (terrain_offset_cm - rangefinder_up_state.terrain_offset_cm) * (copter.G_Dt / MAX(copter.g2.surftrak_tc, copter.G_Dt));

    if (rangefinder_state.alt_healthy || (AP_HAL::millis() - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS)) {
        wp_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#if MODE_CIRCLE_ENABLED
        circle_nav->set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.terrain_offset_cm);
#endif
    }
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret) const
{
    if (!rangefinder_alt_ok()) {
        return false;
    }
    ret = rangefinder_state.alt_cm_filt.get();
    float inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    ret += inertial_alt_cm - rangefinder_state.inertial_alt_cm;
    return true;
}
