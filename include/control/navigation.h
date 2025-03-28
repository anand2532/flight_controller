// #ifndef QUADCOPTER_NAVIGATION_H
// #define QUADCOPTER_NAVIGATION_H

// #include <stdint.h>
// #include <stdbool.h>
// #include <esp_err.h>
// #include "types.h"

// // GPS and Position Constants
// #define MAX_WAYPOINTS 20
// #define EARTH_RADIUS 6371000.0f  // Earth radius in meters

// // Navigation Modes
// typedef enum {
//     NAV_MODE_MANUAL,
//     NAV_MODE_HOLD_POSITION,
//     NAV_MODE_RETURN_TO_HOME,
//     NAV_MODE_WAYPOINT_MISSION,
//     NAV_MODE_ORBIT,
//     NAV_MODE_FOLLOW_ME
// } navigation_mode_t;

// // Geographic Coordinate Structure
// typedef struct {
//     double latitude;   // Latitude in decimal degrees
//     double longitude;  // Longitude in decimal degrees
//     float altitude;    // Altitude in meters
// } geo_location_t;

// // // Waypoint Structure
// // typedef struct {
// //     geo_location_t location;
// //     float acceptance_radius;  // Acceptance radius in meters
// //     uint16_t hold_time;       // Time to hold at waypoint (seconds)
// // } waypoint_t;

// // Navigation State
// typedef struct {
//     geo_location_t current_position;
//     geo_location_t home_location;
//     geo_location_t target_location;
    
//     float ground_speed;       // Current ground speed (m/s)
//     float heading;            // Current heading (degrees)
//     float desired_heading;    // Desired heading (degrees)
    
//     navigation_mode_t current_mode;
// } navigation_state_t;

// // Navigation Configuration
// typedef struct {
//     float max_horizontal_speed;  // Maximum horizontal speed (m/s)
//     float max_vertical_speed;    // Maximum vertical speed (m/s)
//     float navigation_tolerance;  // Navigation tolerance (meters)
//     float return_home_altitude;  // Altitude for return to home (meters)
// } navigation_config_t;

// // Mission Plan Structure
// typedef struct {
//     waypoint_t waypoints[MAX_WAYPOINTS];
//     uint8_t waypoint_count;
//     uint8_t current_waypoint_index;
// } mission_plan_t;

// // Navigation System Context
// typedef struct {
//     navigation_state_t state;
//     navigation_config_t config;
//     mission_plan_t mission;
    
//     // Flags and internal state
//     bool mission_completed;
//     bool position_valid;
// } navigation_system_t;

// /**
//  * @brief Initialize navigation system
//  * @param nav Navigation system context
//  * @param config Navigation configuration
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_init(
//     navigation_system_t *nav, 
//     const navigation_config_t *config
// );

// /**
//  * @brief Update navigation system
//  * @param nav Navigation system context
//  * @param current_position Current GPS position
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_update(
//     navigation_system_t *nav, 
//     const geo_location_t *current_position
// );

// /**
//  * @brief Set navigation mode
//  * @param nav Navigation system context
//  * @param mode New navigation mode
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_set_mode(
//     navigation_system_t *nav, 
//     navigation_mode_t mode
// );

// /**
//  * @brief Load mission plan
//  * @param nav Navigation system context
//  * @param mission Mission plan to load
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_load_mission(
//     navigation_system_t *nav, 
//     const mission_plan_t *mission
// );

// /**
//  * @brief Compute distance and bearing between two coordinates
//  * @param from Starting location
//  * @param to Destination location
//  * @param distance Pointer to store distance
//  * @param bearing Pointer to store bearing
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_compute_distance_bearing(
//     const geo_location_t *from,
//     const geo_location_t *to,
//     float *distance,
//     float *bearing
// );

// /**
//  * @brief Set home location
//  * @param nav Navigation system context
//  * @param home Home location coordinates
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_set_home_location(
//     navigation_system_t *nav, 
//     const geo_location_t *home
// );

// /**
//  * @brief Get current navigation target
//  * @param nav Navigation system context
//  * @param target Pointer to store target location
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_get_current_target(
//     const navigation_system_t *nav, 
//     geo_location_t *target
// );

// /**
//  * @brief Perform return to home procedure
//  * @param nav Navigation system context
//  * @return ESP_OK on success, error code on failure
//  */
// esp_err_t navigation_return_to_home(
//     navigation_system_t *nav
// );

// /**
//  * @brief Check if current mission is completed
//  * @param nav Navigation system context
//  * @return true if mission is completed, false otherwise
//  */
// bool navigation_is_mission_complete(
//     const navigation_system_t *nav
// );

// #endif // QUADCOPTER_NAVIGATION_H

#ifndef QUADCOPTER_NAVIGATION_H
#define QUADCOPTER_NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "types.h"  // Make sure to include this, as it defines waypoint_t

// GPS and Position Constants
#define MAX_WAYPOINTS 20
#define EARTH_RADIUS 6371000.0f  // Earth radius in meters

// Navigation Modes
typedef enum {
    NAV_MODE_MANUAL,
    NAV_MODE_HOLD_POSITION,
    NAV_MODE_RETURN_TO_HOME,
    NAV_MODE_WAYPOINT_MISSION,
    NAV_MODE_ORBIT,
    NAV_MODE_FOLLOW_ME
} navigation_mode_t;

// Navigation State
typedef struct {
    geo_location_t current_position;
    geo_location_t home_location;
    geo_location_t target_location;
    
    float ground_speed;       // Current ground speed (m/s)
    float heading;            // Current heading (degrees)
    float desired_heading;    // Desired heading (degrees)
    
    navigation_mode_t current_mode;
} navigation_state_t;

// Navigation Configuration
typedef struct {
    float max_horizontal_speed;  // Maximum horizontal speed (m/s)
    float max_vertical_speed;    // Maximum vertical speed (m/s)
    float navigation_tolerance;  // Navigation tolerance (meters)
    float return_home_altitude;  // Altitude for return to home (meters)
} navigation_config_t;

// Mission Plan Structure
typedef struct {
    waypoint_t waypoints[MAX_WAYPOINTS]; // Uses definition from types.h
    uint8_t waypoint_count;
    uint8_t current_waypoint_index;
} mission_plan_t;

// Navigation System Context
typedef struct {
    navigation_state_t state;
    navigation_config_t config;
    mission_plan_t mission;
    
    // Flags and internal state
    bool mission_completed;
    bool position_valid;
} navigation_system_t;

esp_err_t navigation_init(navigation_system_t *nav, const navigation_config_t *config);
esp_err_t navigation_update(navigation_system_t *nav, const geo_location_t *current_position);
esp_err_t navigation_set_mode(navigation_system_t *nav, navigation_mode_t mode);
esp_err_t navigation_load_mission(navigation_system_t *nav, const mission_plan_t *mission);
esp_err_t navigation_compute_distance_bearing(const geo_location_t *from, const geo_location_t *to, float *distance, float *bearing);
esp_err_t navigation_set_home_location(navigation_system_t *nav, const geo_location_t *home);
esp_err_t navigation_get_current_target(const navigation_system_t *nav, geo_location_t *target);
esp_err_t navigation_return_to_home(navigation_system_t *nav);
bool navigation_is_mission_complete(const navigation_system_t *nav);

#endif // QUADCOPTER_NAVIGATION_H
