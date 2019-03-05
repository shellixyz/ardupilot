#include "AP_Mission.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>

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
