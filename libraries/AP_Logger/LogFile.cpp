#include <stdlib.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

#include "AP_Logger.h"
#include "LoggerMessageWriter.h"

extern const AP_HAL::HAL& hal;


/*
  write a structure format to the log - should be in frontend
 */
void AP_Logger_Backend::Fill_Format(const struct LogStructure *s, struct log_Format &pkt)
{
}

/*
  Pack a LogStructure packet into a structure suitable to go to the logfile:
 */
void AP_Logger_Backend::Fill_Format_Units(const struct LogStructure *s, struct log_Format_Units &pkt)
{
}

/*
  write a structure format to the log
 */
bool AP_Logger_Backend::Write_Format(const struct LogStructure *s)
{
    return true;
}

/*
  write a unit definition
 */
bool AP_Logger_Backend::Write_Unit(const struct UnitStructure *s)
{
    return true;
}

/*
  write a unit-multiplier definition
 */
bool AP_Logger_Backend::Write_Multiplier(const struct MultiplierStructure *s)
{
    return true;
}

/*
  write the units for a format to the log
 */
bool AP_Logger_Backend::Write_Format_Units(const struct LogStructure *s)
{
    return true;
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const char *name, float value)
{
    return true;
}

/*
  write a parameter to the log
 */
bool AP_Logger_Backend::Write_Parameter(const AP_Param *ap,
                                            const AP_Param::ParamToken &token,
                                            enum ap_var_type type)
{
    return true;
}

// Write an GPS packet
void AP_Logger::Write_GPS(uint8_t i, uint64_t time_us)
{
}


// Write an RCIN packet
void AP_Logger::Write_RCIN(void)
{
}

// Write an SERVO packet
void AP_Logger::Write_RCOUT(void)
{
}

// Write an RSSI packet
void AP_Logger::Write_RSSI(AP_RSSI &rssi)
{
}

void AP_Logger::Write_Baro_instance(uint64_t time_us, uint8_t baro_instance, enum LogMessages type)
{
}

// Write a BARO packet
void AP_Logger::Write_Baro(uint64_t time_us)
{
}

void AP_Logger::Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
}

// Write an raw accel/gyro data packet
void AP_Logger::Write_IMU()
{
}

// Write an accel/gyro delta time data packet
void AP_Logger::Write_IMUDT_instance(const uint64_t time_us, const uint8_t imu_instance, const enum LogMessages type)
{
}

void AP_Logger::Write_IMUDT(uint64_t time_us, uint8_t imu_mask)
{
}

void AP_Logger::Write_Vibration()
{
}

bool AP_Logger_Backend::Write_Mission_Cmd(const AP_Mission &mission,
                                              const AP_Mission::Mission_Command &cmd)
{
    return true;
}

void AP_Logger_Backend::Write_EntireMission()
{
}

// Write a text message to the log
bool AP_Logger_Backend::Write_Message(const char *message)
{
    return true;
}

void AP_Logger::Write_Power(void)
{
}

// Write an AHRS2 packet
void AP_Logger::Write_AHRS2(AP_AHRS &ahrs)
{
}

// Write a POS packet
void AP_Logger::Write_POS(AP_AHRS &ahrs)
{
}

#if AP_AHRS_NAVEKF_AVAILABLE
void AP_Logger::Write_EKF(AP_AHRS_NavEKF &ahrs)
{
}


/*
  write an EKF timing message
 */
void AP_Logger::Write_EKF_Timing(const char *name, uint64_t time_us, const struct ekf_timing &timing)
{
}

void AP_Logger::Write_EKF2(AP_AHRS_NavEKF &ahrs)
{
}

#endif

// Write an attitude packet
void AP_Logger::Write_Attitude(AP_AHRS &ahrs, const Vector3f &targets)
{
}

// Write an attitude packet
void AP_Logger::Write_AttitudeView(AP_AHRS_View &ahrs, const Vector3f &targets)
{
}

void AP_Logger::Write_Current_instance(const uint64_t time_us,
                                                 const uint8_t battery_instance,
                                                 const enum LogMessages type,
                                                 const enum LogMessages celltype)
{
}

// Write an Current data packet
void AP_Logger::Write_Current()
{
}

void AP_Logger::Write_Compass_instance(const uint64_t time_us, const uint8_t mag_instance, const enum LogMessages type)
{
}

// Write a Compass packet
void AP_Logger::Write_Compass(uint64_t time_us)
{
}

// Write a mode packet.
bool AP_Logger_Backend::Write_Mode(uint8_t mode, uint8_t reason)
{
    return true;
}

// Write ESC status messages
//   id starts from 0
//   rpm is eRPM (rpm * 100)
//   voltage is in centi-volts
//   current is in centi-amps
//   temperature is in centi-degrees Celsius
//   current_tot is in centi-amp hours
void AP_Logger::Write_ESC(uint8_t id, uint64_t time_us, int32_t rpm, uint16_t voltage, uint16_t current, int16_t temperature, uint16_t current_tot)
{
}

// Write a Yaw PID packet
void AP_Logger::Write_PID(uint8_t msg_type, const PID_Info &info)
{
}

void AP_Logger::Write_Origin(uint8_t origin_type, const Location &loc)
{
}

// Write a rate packet
void AP_Logger::Write_Rate(const AP_AHRS_View *ahrs,
                                     const AP_Motors &motors,
                                     const AC_AttitudeControl &attitude_control,
                                     const AC_PosControl &pos_control)
{
}

// Write AOA and SSA
void AP_Logger::Write_AOA_SSA(AP_AHRS &ahrs)
{
}

void AP_Logger::Write_SRTL(bool active, uint16_t num_points, uint16_t max_points, uint8_t action, const Vector3f& breadcrumb)
{
}
