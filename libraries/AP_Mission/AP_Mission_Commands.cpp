#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_Parachute/AP_Parachute.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

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
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper Released");
        return true;
    case GRIPPER_ACTION_GRAB:
        gripper->grab();
        // Log_Write_Event(DATA_GRIPPER_GRAB);
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper Grabbed");
        return true;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled gripper case");
#endif
        return false;
    }
}

bool AP_Mission::start_command_do_servorelayevents(const AP_Mission::Mission_Command& cmd)
{
    AP_ServoRelayEvents *sre = AP::servorelayevents();
    if (sre == nullptr) {
        return false;
    }

    switch (cmd.id) {
    case MAV_CMD_DO_SET_SERVO:
        sre->do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        return true;

    case MAV_CMD_DO_SET_RELAY:
        sre->do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        return true;

    case MAV_CMD_DO_REPEAT_SERVO:
        sre->do_repeat_servo(cmd.content.repeat_servo.channel,
                             cmd.content.repeat_servo.pwm,
                             cmd.content.repeat_servo.repeat_count,
                             cmd.content.repeat_servo.cycle_time * 1000.0f);
        return true;

    case MAV_CMD_DO_REPEAT_RELAY:
        sre->do_repeat_relay(cmd.content.repeat_relay.num,
                             cmd.content.repeat_relay.repeat_count,
                             cmd.content.repeat_relay.cycle_time * 1000.0f);
        return true;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Unhandled servo/relay case");
#endif
        return false;
    }
}

bool AP_Mission::start_command_parachute(const AP_Mission::Mission_Command& cmd)
{
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
}
