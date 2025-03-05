// include/types.h
#ifndef QUADCOPTER_CUSTOM_TYPES_H
#define QUADCOPTER_CUSTOM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// Geographic Coordinate
typedef struct {
    double latitude;       // Latitude in decimal degrees
    double longitude;      // Longitude in decimal degrees
    float altitude;        // Altitude in meters
} geo_coordinate_t;

// 3D Vector
typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

// Quaternion Representation
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

// Flight Mode Enumeration
typedef enum {
    FLIGHT_MODE_MANUAL,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_ALTITUDE_HOLD,
    FLIGHT_MODE_POSITION_HOLD,
    FLIGHT_MODE_RETURN_TO_HOME,
    FLIGHT_MODE_MISSION,
    FLIGHT_MODE_FAILSAFE
} flight_mode_t;

// Sensor Health Status
typedef struct {
    bool mpu6050_ok;
    bool gps_ok;
    bool radio_ok;
    bool battery_ok;
} sensor_health_t;

// Power System Status
typedef struct {
    float battery_voltage;
    float battery_current;
    float battery_remaining_capacity;
    bool is_low_voltage;
    bool is_critical_voltage;
} power_status_t;

// Navigation Waypoint
typedef struct {
    geo_coordinate_t coordinate;
    float heading;
    float loiter_time;
    bool is_current;
} waypoint_t;

// Control Surface Inputs
typedef struct {
    float throttle;    // 0 to 1
    float roll;        // -1 to 1
    float pitch;       // -1 to 1
    float yaw;         // -1 to 1
} control_input_t;

// Motor Output
typedef struct {
    uint16_t motor_speeds[4];  // PWM values for each motor
} motor_output_t;

// Error Handling
typedef enum {
    ERROR_NONE,
    ERROR_SENSOR_FAILURE,
    ERROR_COMMUNICATION,
    ERROR_LOW_BATTERY,
    ERROR_GPS_LOST,
    ERROR_RADIO_SIGNAL_LOST,
    ERROR_CALIBRATION_REQUIRED
} system_error_t;

// Telemetry Packet
typedef struct {
    uint32_t timestamp;
    geo_coordinate_t position;
    vector3_t velocity;
    quaternion_t orientation;
    flight_mode_t current_mode;
    sensor_health_t sensor_status;
    power_status_t power_status;
    system_error_t current_error;
} telemetry_packet_t;

// Configuration Validation Function
bool validate_types(void);

#endif // QUADCOPTER_CUSTOM_TYPES_H