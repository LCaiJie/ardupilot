#include "AP_Mission_config.h"

#if AP_MISSION_ENABLED

#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
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

#if AP_GRIPPER_ENABLED
bool AP_Mission::start_command_do_gripper(const AP_Mission::Mission_Command& cmd)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return false;
    }

    // Note: we ignore the gripper num parameter because we only
    // support one gripper
    switch (cmd.content.gripper.action) {
    case GRIPPER_ACTION_RELEASE:
        gripper->release();
        // Log_Write_Event(DATA_GRIPPER_RELEASE);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper Released");
        return true;
    case GRIPPER_ACTION_GRAB:
        gripper->grab();
        // Log_Write_Event(DATA_GRIPPER_GRAB);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gripper Grabbed");
        return true;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled gripper case");
#endif
        return false;
    }
}
#endif  // AP_GRIPPER_ENABLED

#if AP_SERVORELAYEVENTS_ENABLED
bool AP_Mission::start_command_do_servorelayevents(const AP_Mission::Mission_Command& cmd)
{
    AP_ServoRelayEvents *sre = AP::servorelayevents();
    if (sre == nullptr) {
        return false;
    }

    switch (cmd.id) {
    case MAV_CMD_DO_SET_SERVO:
        return sre->do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);

#if AP_RELAY_ENABLED
    case MAV_CMD_DO_SET_RELAY:
        return sre->do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
#endif

    case MAV_CMD_DO_REPEAT_SERVO:
        return sre->do_repeat_servo(cmd.content.repeat_servo.channel,
                                    cmd.content.repeat_servo.pwm,
                                    cmd.content.repeat_servo.repeat_count,
                                    cmd.content.repeat_servo.cycle_time * 1000.0f);

#if AP_RELAY_ENABLED
    case MAV_CMD_DO_REPEAT_RELAY:
        return sre->do_repeat_relay(cmd.content.repeat_relay.num,
                                    cmd.content.repeat_relay.repeat_count,
                                    cmd.content.repeat_relay.cycle_time * 1000.0f);
#endif

    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled servo/relay case");
#endif
        return false;
    }
}
#endif  // AP_SERVORELAYEVENTS_ENABLED

bool AP_Mission::start_command_parachute(const AP_Mission::Mission_Command& cmd)
{
#if HAL_PARACHUTE_ENABLED
    AP_Parachute *parachute = AP::parachute();
    if (parachute == nullptr) {
        return false;
    }

    switch (cmd.p1) {
    case PARACHUTE_DISABLE:
        parachute->enabled(false);
        break;
    case PARACHUTE_ENABLE:
        parachute->enabled(true);
        break;
    case PARACHUTE_RELEASE:
        parachute->release();
        break;
    default:
        // do nothing
        return false;
    }

    return true;
#else
    return false;
#endif // HAL_PARACHUTE_ENABLED
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
