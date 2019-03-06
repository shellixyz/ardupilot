#include "AP_Logger.h"

#include "AP_Logger_Backend.h"

//#include "AP_Logger_File.h"
//#include "AP_Logger_SITL.h"
//#include "AP_Logger_DataFlash.h"
//#include "AP_Logger_MAVLink.h"
#include <GCS_MAVLink/GCS.h>

AP_Logger *AP_Logger::_singleton;

extern const AP_HAL::HAL& hal;

#ifndef HAL_LOGGING_FILE_BUFSIZE
#define HAL_LOGGING_FILE_BUFSIZE  16
#endif 

#ifndef HAL_LOGGING_MAV_BUFSIZE
#define HAL_LOGGING_MAV_BUFSIZE  8
#endif 

#ifndef HAL_LOGGING_BACKENDS_DEFAULT
# ifdef HAL_LOGGING_DATAFLASH
#  define HAL_LOGGING_BACKENDS_DEFAULT DATAFLASH_BACKEND_BLOCK
# else
#  define HAL_LOGGING_BACKENDS_DEFAULT DATAFLASH_BACKEND_FILE
# endif
#endif

const AP_Param::GroupInfo AP_Logger::var_info[] = {
    // @Param: _BACKEND_TYPE
    // @DisplayName: AP_Logger Backend Storage type
    // @Description: Bitmap of what Logger backend types to enable. Block-based logging is available on SITL and boards with dataflash chips. Multiple backends can be selected.
    // @Bitmask: 0:File,1:MAVLink,2:Block
    // @User: Standard
    AP_GROUPINFO("_BACKEND_TYPE",  0, AP_Logger, _params.backend_types,       HAL_LOGGING_BACKENDS_DEFAULT),

    // @Param: _FILE_BUFSIZE
    // @DisplayName: Maximum AP_Logger File Backend buffer size (in kilobytes)
    // @Description: The AP_Logger_File backend uses a buffer to store data before writing to the block device.  Raising this value may reduce "gaps" in your SD card logging.  This buffer size may be reduced depending on available memory.  PixHawk requires at least 4 kilobytes.  Maximum value available here is 64 kilobytes.
    // @User: Standard
    AP_GROUPINFO("_FILE_BUFSIZE",  1, AP_Logger, _params.file_bufsize,       HAL_LOGGING_FILE_BUFSIZE),

    // @Param: _DISARMED
    // @DisplayName: Enable logging while disarmed
    // @Description: If LOG_DISARMED is set to 1 then logging will be enabled while disarmed. This can make for very large logfiles but can help a lot when tracking down startup issues
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_DISARMED",  2, AP_Logger, _params.log_disarmed,       0),

    // @Param: _REPLAY
    // @DisplayName: Enable logging of information needed for Replay
    // @Description: If LOG_REPLAY is set to 1 then the EKF2 state estimator will log detailed information needed for diagnosing problems with the Kalman filter. It is suggested that you also raise LOG_FILE_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_REPLAY",  3, AP_Logger, _params.log_replay,       0),

    // @Param: _FILE_DSRMROT
    // @DisplayName: Stop logging to current file on disarm
    // @Description: When set, the current log file is closed when the vehicle is disarmed.  If LOG_DISARMED is set then a fresh log will be opened.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("_FILE_DSRMROT",  4, AP_Logger, _params.file_disarm_rot,       0),

    // @Param: _MAV_BUFSIZE
    // @DisplayName: Maximum AP_Logger MAVLink Backend buffer size
    // @Description: Maximum amount of memory to allocate to AP_Logger-over-mavlink
    // @User: Advanced
    // @Units: kB
    AP_GROUPINFO("_MAV_BUFSIZE",  5, AP_Logger, _params.mav_bufsize,       HAL_LOGGING_MAV_BUFSIZE),

    AP_GROUPEND
};

#define streq(x, y) (!strcmp(x, y))

AP_Logger::AP_Logger(const AP_Int32 &log_bitmask)
    : _log_bitmask(log_bitmask)
{
}

void AP_Logger::Init(const struct LogStructure *structures, uint8_t num_types)
{
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>

#define DEBUG_LOG_STRUCTURES 0

extern const AP_HAL::HAL& hal;
#define Debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)

/// return the number of commas present in string
static uint8_t count_commas(const char *string)
{
    return 0;
}

/// return a unit name given its ID
const char* AP_Logger::unit_name(const uint8_t unit_id)
{
    return NULL;
}

/// return a multiplier value given its ID
double AP_Logger::multiplier_name(const uint8_t multiplier_id)
{
    return 1.0f;
}

/// pretty-print field information from a log structure
void AP_Logger::dump_structure_field(const struct LogStructure *logstructure, const char *label, const uint8_t fieldnum)
{
}

/// pretty-print log structures
/// @note structures MUST be well-formed
void AP_Logger::dump_structures(const struct LogStructure *logstructures, const uint8_t num_types)
{
}

bool AP_Logger::validate_structure(const struct LogStructure *logstructure, const int16_t offset)
{
    return true;
}

void AP_Logger::validate_structures(const struct LogStructure *logstructures, const uint8_t num_types)
{
    return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

const struct LogStructure *AP_Logger::structure(uint16_t num) const
{
    return &_structures[num];
}

bool AP_Logger::logging_present() const
{
    return false;
}

bool AP_Logger::logging_enabled() const
{
    return false;
}
bool AP_Logger::logging_failed() const
{
    return false;
}

void AP_Logger::Write_MessageF(const char *fmt, ...)
{
}

void AP_Logger::backend_starting_new_log(const AP_Logger_Backend *backend)
{
}

bool AP_Logger::should_log(const uint32_t mask) const
{
    return false;
}

const struct UnitStructure *AP_Logger::unit(uint16_t num) const
{
    return &_units[num];
}

const struct MultiplierStructure *AP_Logger::multiplier(uint16_t num) const
{
    return &log_Multipliers[num];
}

#define FOR_EACH_BACKEND(methodcall)              \
    do {                                          \
        for (uint8_t i=0; i<_next_backend; i++) { \
            backends[i]->methodcall;              \
        }                                         \
    } while (0)

void AP_Logger::PrepForArming()
{
}

void AP_Logger::setVehicle_Startup_Writer(vehicle_startup_message_Writer writer)
{
}

void AP_Logger::set_vehicle_armed(const bool armed_state)
{

}


// start functions pass straight through to backend:
void AP_Logger::WriteBlock(const void *pBuffer, uint16_t size) {
}

void AP_Logger::WriteCriticalBlock(const void *pBuffer, uint16_t size) {
}

void AP_Logger::WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) {
}

// change me to "DoTimeConsumingPreparations"?
void AP_Logger::EraseAll() {
}
// change me to "LoggingAvailable"?
bool AP_Logger::CardInserted(void) {
    return false;
}

void AP_Logger::Prep() {
}

void AP_Logger::StopLogging()
{
}

uint16_t AP_Logger::find_last_log() const {
        return 0;
}
void AP_Logger::get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) {
}
void AP_Logger::get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) {
}
int16_t AP_Logger::get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) {
        return 0;
}
uint16_t AP_Logger::get_num_logs(void) {
        return 0;
}

/* we're started if any of the backends are started */
bool AP_Logger::logging_started(void) {
    return true;
}

void AP_Logger::handle_mavlink_msg(GCS_MAVLINK &link, mavlink_message_t* msg)
{
}

void AP_Logger::periodic_tasks() {
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    // currently only AP_Logger_File support this:
void AP_Logger::flush(void) {
}
#endif


void AP_Logger::Write_EntireMission()
{
}

void AP_Logger::Write_Message(const char *message)
{
}

void AP_Logger::Write_Mode(uint8_t mode, uint8_t reason)
{
}

void AP_Logger::Write_Parameter(const char *name, float value)
{
}

void AP_Logger::Write_Mission_Cmd(const AP_Mission &mission,
                                            const AP_Mission::Mission_Command &cmd)
{
}

void AP_Logger::Write_RallyPoint(uint8_t total,
                                 uint8_t sequence,
                                 const RallyLocation &rally_point)
{
}

uint32_t AP_Logger::num_dropped() const
{
        return 0;
}


// end functions pass straight through to backend

void AP_Logger::internal_error() const {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_HAL::panic("Internal AP_Logger error");
#endif
}

/* Write support */
void AP_Logger::Write(const char *name, const char *labels, const char *fmt, ...)
{
}

void AP_Logger::Write(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, ...)
{
}

void AP_Logger::WriteV(const char *name, const char *labels, const char *units, const char *mults, const char *fmt, va_list arg_list)
{
}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void AP_Logger::assert_same_fmt_for_name(const AP_Logger::log_write_fmt *f,
                                               const char *name,
                                               const char *labels,
                                               const char *units,
                                               const char *mults,
                                               const char *fmt) const
{
}
#endif

AP_Logger::log_write_fmt *AP_Logger::msg_fmt_for_name(const char *name, const char *labels, const char *units, const char *mults, const char *fmt)
{
        return nullptr;
}

const struct LogStructure *AP_Logger::structure_for_msg_type(const uint8_t msg_type)
{
    return nullptr;
}

const struct AP_Logger::log_write_fmt *AP_Logger::log_write_fmt_for_msg_type(const uint8_t msg_type) const
{
    return nullptr;
}


// returns true if the msg_type is already taken
bool AP_Logger::msg_type_in_use(const uint8_t msg_type) const
{
    return false;
}

// find a free message type
int16_t AP_Logger::find_free_msg_type() const
{
    return -1;
}

/*
 * It is assumed that logstruct's char* variables are valid strings of
 * maximum lengths for those fields (given in LogStructure.h e.g. LS_NAME_SIZE)
 */
bool AP_Logger::fill_log_write_logstructure(struct LogStructure &logstruct, const uint8_t msg_type) const
{
    return false;
}

/* calculate the length of output of a format string.  Note that this
 * returns an int16_t; if it returns -1 then an error has occurred.
 * This was mechanically converted from init_field_types in
 * Tools/Replay/MsgHandler.cpp */
int16_t AP_Logger::Write_calc_msg_len(const char *fmt) const
{
            return -1;
}

/* End of Write support */

#undef FOR_EACH_BACKEND

// Write information about a series of IMU readings to log:
bool AP_Logger::Write_ISBH(const uint16_t seqno,
                                     const AP_InertialSensor::IMU_SENSOR_TYPE sensor_type,
                                     const uint8_t sensor_instance,
                                     const uint16_t mult,
                                     const uint16_t sample_count,
                                     const uint64_t sample_us,
                                     const float sample_rate_hz)
{
        return false;
}


// Write a series of IMU readings to log:
bool AP_Logger::Write_ISBD(const uint16_t isb_seqno,
                                     const uint16_t seqno,
                                     const int16_t x[32],
                                     const int16_t y[32],
                                     const int16_t z[32])
{
 return false;
}

// Wrote an event packet
void AP_Logger::Write_Event(Log_Event id)
{
}

namespace AP {

AP_Logger &logger()
{
    return *AP_Logger::get_singleton();
}

};
