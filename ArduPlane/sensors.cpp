#include "Plane.h"
#include <AP_RSSI/AP_RSSI.h>

/*
  calibrate compass
*/
void Plane::compass_cal_update() {
    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }
}

/*
    Accel calibration
*/
void Plane::accel_cal_update() {
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

/*
  ask airspeed sensor for a new value
 */
void Plane::read_airspeed(void)
{
    airspeed.update(should_log(MASK_LOG_IMU));

    // we calculate airspeed errors (and thus target_airspeed_cm) even
    // when airspeed is disabled as TECS may be using synthetic
    // airspeed for a quadplane transition
    calc_airspeed_errors();
    
    // update smoothed airspeed estimate
    float aspeed;
    if (ahrs.airspeed_estimate(&aspeed)) {
        smoothed_airspeed = smoothed_airspeed * 0.8f + aspeed * 0.2f;
    }
}
