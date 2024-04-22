#include "AP_Mission_config.h"

#if AP_MISSION_ENABLED

#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

#if AP_RC_CHANNEL_ENABLED
bool AP_Mission::start_command_do_aux_function(const AP_Mission::Mission_Command& cmd)
{
    const RC_Channel::AUX_FUNC function = (RC_Channel::AUX_FUNC)cmd.content.auxfunction.function;
    const RC_Channel::AuxSwitchPos pos = (RC_Channel::AuxSwitchPos)cmd.content.auxfunction.switchpos;

    // sanity check the switch position.  Could map from the mavlink
    // enumeration if we were really keen
    switch (pos) {
    case RC_Channel::AuxSwitchPos::HIGH:
    case RC_Channel::AuxSwitchPos::MIDDLE:
    case RC_Channel::AuxSwitchPos::LOW:
        break;
    default:
        return false;
    }
    rc().run_aux_function(function, pos, RC_Channel::AuxFuncTriggerSource::MISSION);
    return true;
}
#endif  // AP_RC_CHANNEL_ENABLED


bool AP_Mission::start_command_parachute(const AP_Mission::Mission_Command& cmd)
{
    return false;
}

bool AP_Mission::command_do_set_repeat_dist(const AP_Mission::Mission_Command& cmd)
{
    _repeat_dist = cmd.p1;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Resume repeat dist set to %u m",_repeat_dist);
    return true;
}

bool AP_Mission::start_command_do_sprayer(const AP_Mission::Mission_Command& cmd)
{
    return false;
}

bool AP_Mission::start_command_do_scripting(const AP_Mission::Mission_Command& cmd)
{
    return false;
}

bool AP_Mission::start_command_do_gimbal_manager_pitchyaw(const AP_Mission::Mission_Command& cmd)
{

    // if we got this far then message is not handled
    return false;
}

#endif  // AP_MISSION_ENABLED
