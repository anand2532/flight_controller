// #include <math.h>
// #include <string.h>
// #include <esp_log.h>

// #include "control/navigation.h"
// #include "config.h"

// #define TAG "QUADCOPTER_NAVIGATION"

// // Convert degrees to radians
// #define DEG_TO_RAD(deg) ((deg) * (float)M_PI / 180.0f)
// #define RAD_TO_DEG(rad) ((rad) * 180.0f / (float)M_PI)

// // Default navigation configuration
// static const navigation_config_t DEFAULT_NAVIGATION_CONFIG = {
//     .max_horizontal_speed = 10.0f,    // 10 m/s
//     .max_vertical_speed = 5.0f,       // 5 m/s
//     .navigation_tolerance = 2.0f,     // 2 meters tolerance
//     .return_home_altitude = 30.0f     // 30 meters RTH altitude
// };

// // Internal helper functions
// static esp_err_t process_waypoint_mission(navigation_system_t *nav);
// static esp_err_t handle_position_hold(navigation_system_t *nav);
// static esp_err_t handle_return_to_home(navigation_system_t *nav);

// esp_err_t navigation_init(
//     navigation_system_t *nav, 
//     const navigation_config_t *config
// ) {
//     if (nav == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Use provided config or default
//     const navigation_config_t *use_config = 
//         config ? config : &DEFAULT_NAVIGATION_CONFIG;

//     // Clear navigation system
//     memset(nav, 0, sizeof(navigation_system_t));

//     // Copy configuration
//     memcpy(&nav->config, use_config, sizeof(navigation_config_t));

//     // Set initial mode
//     nav->state.current_mode = NAV_MODE_MANUAL;

//     // Initialize position as invalid
//     nav->position_valid = false;

//     ESP_LOGI(TAG, "Navigation system initialized");
//     return ESP_OK;
// }

// esp_err_t navigation_update(
//     navigation_system_t *nav, 
//     const geo_location_t *current_position
// ) {
//     if (nav == NULL || current_position == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Update current position
//     memcpy(&nav->state.current_position, current_position, sizeof(geo_location_t));
//     nav->position_valid = true;

//     // Process based on current navigation mode
//     switch (nav->state.current_mode) {
//         case NAV_MODE_WAYPOINT_MISSION:
//             return process_waypoint_mission(nav);

//         case NAV_MODE_HOLD_POSITION:
//             return handle_position_hold(nav);

//         case NAV_MODE_RETURN_TO_HOME:
//             return handle_return_to_home(nav);

//         case NAV_MODE_MANUAL:
//         default:
//             // No automatic navigation
//             return ESP_OK;
//     }
// }

// esp_err_t navigation_set_mode(
//     navigation_system_t *nav, 
//     navigation_mode_t mode
// ) {
//     if (nav == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Validate mode transition
//     switch (mode) {
//         case NAV_MODE_WAYPOINT_MISSION:
//             // Ensure mission is loaded
//             if (nav->mission.waypoint_count == 0) {
//                 ESP_LOGE(TAG, "No mission loaded");
//                 return ESP_ERR_INVALID_STATE;
//             }
//             break;

//         case NAV_MODE_RETURN_TO_HOME:
//             // Ensure home location is set
//             if (nav->state.home_location.latitude == 0 && 
//                 nav->state.home_location.longitude == 0) {
//                 ESP_LOGE(TAG, "Home location not set");
//                 return ESP_ERR_INVALID_STATE;
//             }
//             break;

//         default:
//             break;
//     }

//     // Set new mode
//     nav->state.current_mode = mode;
    
//     // Reset mission-related flags
//     nav->mission_completed = false;
//     nav->mission.current_waypoint_index = 0;

//     ESP_LOGI(TAG, "Navigation mode changed to %d", mode);
//     return ESP_OK;
// }

// esp_err_t navigation_load_mission(
//     navigation_system_t *nav, 
//     const mission_plan_t *mission
// ) {
//     if (nav == NULL || mission == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Validate mission
//     if (mission->waypoint_count > MAX_WAYPOINTS) {
//         ESP_LOGE(TAG, "Too many waypoints");
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Copy mission plan
//     memcpy(&nav->mission, mission, sizeof(mission_plan_t));

//     // Reset mission state
//     nav->mission_completed = false;
//     nav->mission.current_waypoint_index = 0;

//     ESP_LOGI(TAG, "Mission loaded with %d waypoints", mission->waypoint_count);
//     return ESP_OK;
// }

// esp_err_t navigation_compute_distance_bearing(
//     const geo_location_t *from,
//     const geo_location_t *to,
//     float *distance,
//     float *bearing
// ) {
//     if (from == NULL || to == NULL || distance == NULL || bearing == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Convert to radians
//     float lat1 = DEG_TO_RAD(from->latitude);
//     float lon1 = DEG_TO_RAD(from->longitude);
//     float lat2 = DEG_TO_RAD(to->latitude);
//     float lon2 = DEG_TO_RAD(to->longitude);

//     // Haversine formula for distance
//     float dlat = lat2 - lat1;
//     float dlon = lon2 - lon1;

//     float a = sinf(dlat/2) * sinf(dlat/2) +
//               cosf(lat1) * cosf(lat2) *
//               sinf(dlon/2) * sinf(dlon/2);
//     float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    
//     *distance = EARTH_RADIUS * c;

//     // Bearing calculation
//     float y = sinf(dlon) * cosf(lat2);
//     float x = cosf(lat1) * sinf(lat2) - 
//               sinf(lat1) * cosf(lat2) * cosf(dlon);
    
//     *bearing = RAD_TO_DEG(atan2f(y, x));

//     // Normalize bearing to 0-360 degrees
//     if (*bearing < 0) {
//         *bearing += 360.0f;
//     }

//     return ESP_OK;
// }

// esp_err_t navigation_set_home_location(
//     navigation_system_t *nav, 
//     const geo_location_t *home
// ) {
//     if (nav == NULL || home == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Copy home location
//     memcpy(&nav->state.home_location, home, sizeof(geo_location_t));

//     ESP_LOGI(TAG, "Home location set: Lat=%.6f, Lon=%.6f", 
//              home->latitude, home->longitude);

//     return ESP_OK;
// }

// esp_err_t navigation_get_current_target(
//     const navigation_system_t *nav, 
//     geo_location_t *target
// ) {
//     if (nav == NULL || target == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Copy current target location
//     memcpy(target, &nav->state.target_location, sizeof(geo_location_t));

//     return ESP_OK;
// }

// esp_err_t navigation_return_to_home(
//     navigation_system_t *nav
// ) {
//     if (nav == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     // Set navigation mode to return to home
//     return navigation_set_mode(nav, NAV_MODE_RETURN_TO_HOME);
// }

// bool navigation_is_mission_complete(
//     const navigation_system_t *nav
// ) {
//     if (nav == NULL) {
//         return false;
//     }

//     return nav->mission_completed;
// }

// // Waypoint mission processing
// static esp_err_t process_waypoint_mission(navigation_system_t *nav) {
//     if (!nav->position_valid) {
//         return ESP_ERR_INVALID_STATE;
//     }

//     // Get current waypoint
//     waypoint_t *current_wp = &nav->mission.waypoints[nav->mission.current_waypoint_index];

//     // Compute distance and bearing to current waypoint
//     float distance, bearing;
//     esp_err_t ret = navigation_compute_distance_bearing(
//         &nav->state.current_position, 
//         &current_wp->location, 
//         &distance, 
//         &bearing
//     );

//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Update navigation state
//     nav->state.desired_heading = bearing;
//     nav->state.target_location = current_wp->location;

//     // Check if waypoint reached
//     if (distance <= current_wp->acceptance_radius) {
//         // Move to next waypoint or complete mission
//         nav->mission.current_waypoint_index++;

//         if (nav->mission.current_waypoint_index >= nav->mission.waypoint_count) {
//             nav->mission_completed = true;
//             ESP_LOGI(TAG, "Mission completed");
//             return navigation_set_mode(nav, NAV_MODE_HOLD_POSITION);
//         }
//     }

//     return ESP_OK;
// }

// // Position hold processing
// static esp_err_t handle_position_hold(navigation_system_t *nav) {
//     if (!nav->position_valid) {
//         return ESP_ERR_INVALID_STATE;
//     }

//     // Set target as current position
//     nav->state.target_location = nav->state.current_position;
    
//     // Maintain current position and altitude
//     return ESP_OK;
// }

// // Return to home processing
// static esp_err_t handle_return_to_home(navigation_system_t *nav) {
//     if (!nav->position_valid) {
//         return ESP_ERR_INVALID_STATE;
//     }

//     // Compute distance and bearing to home
//     float distance, bearing;
//     esp_err_t ret = navigation_compute_distance_bearing(
//         &nav->state.current_position, 
//         &nav->state.home_location, 
//         &distance, 
//         &bearing
//     );

//     if (ret != ESP_OK) {
//         return ret;
//     }

//     // Update navigation state
//     nav->state.desired_heading = bearing;
//     nav->state.target_location = nav->state.home_location;

//     // Adjust altitude to return home altitude
//     nav->state.target_location.altitude = nav->config.return_home_altitude;

//     // Check if home reached
//     if (distance <= nav->config.navigation_tolerance) {
//         ESP_LOGI(TAG, "Returned to home");
//         return navigation_set_mode(nav, NAV_MODE_HOLD_POSITION);
//     }

//     return ESP_OK;
// }


#include <math.h>
#include <string.h>
#include <esp_log.h>
#include "control/navigation.h"
#include "config.h"

#define TAG "QUADCOPTER_NAVIGATION"

// Convert degrees to radians
#define DEG_TO_RAD(deg) ((deg) * (float)M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / (float)M_PI)

// Default navigation configuration
static const navigation_config_t DEFAULT_NAVIGATION_CONFIG = {
    .max_horizontal_speed = 10.0f,
    .max_vertical_speed = 5.0f,
    .navigation_tolerance = 2.0f,
    .return_home_altitude = 30.0f
};

// Internal helper function declarations
static esp_err_t process_waypoint_mission(navigation_system_t *nav);
static esp_err_t handle_position_hold(navigation_system_t *nav);
static esp_err_t handle_return_to_home(navigation_system_t *nav);

esp_err_t navigation_init(navigation_system_t *nav, const navigation_config_t *config) {
    if (nav == NULL) return ESP_ERR_INVALID_ARG;

    const navigation_config_t *use_config = config ? config : &DEFAULT_NAVIGATION_CONFIG;

    memset(nav, 0, sizeof(navigation_system_t));
    memcpy(&nav->config, use_config, sizeof(navigation_config_t));

    nav->state.current_mode = NAV_MODE_MANUAL;
    nav->position_valid = false;

    ESP_LOGI(TAG, "Navigation system initialized");
    return ESP_OK;
}

esp_err_t navigation_update(navigation_system_t *nav, const geo_location_t *current_position) {
    if (nav == NULL || current_position == NULL) return ESP_ERR_INVALID_ARG;

    memcpy(&nav->state.current_position, current_position, sizeof(geo_location_t));
    nav->position_valid = true;

    switch (nav->state.current_mode) {
        case NAV_MODE_WAYPOINT_MISSION:
            return process_waypoint_mission(nav);
        case NAV_MODE_HOLD_POSITION:
            return handle_position_hold(nav);
        case NAV_MODE_RETURN_TO_HOME:
            return handle_return_to_home(nav);
        case NAV_MODE_MANUAL:
        default:
            return ESP_OK;
    }
}

esp_err_t navigation_compute_distance_bearing(const geo_location_t *from, const geo_location_t *to, float *distance, float *bearing) {
    if (from == NULL || to == NULL || distance == NULL || bearing == NULL) return ESP_ERR_INVALID_ARG;

    float lat1 = DEG_TO_RAD(from->latitude);
    float lon1 = DEG_TO_RAD(from->longitude);
    float lat2 = DEG_TO_RAD(to->latitude);
    float lon2 = DEG_TO_RAD(to->longitude);

    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;

    float a = sinf(dlat/2) * sinf(dlat/2) + cosf(lat1) * cosf(lat2) * sinf(dlon/2) * sinf(dlon/2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));

    *distance = EARTH_RADIUS * c;
    *bearing = RAD_TO_DEG(atan2f(sinf(dlon) * cosf(lat2), cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon)));

    if (*bearing < 0) *bearing += 360.0f;

    return ESP_OK;
}

static esp_err_t process_waypoint_mission(navigation_system_t *nav) {
    if (!nav->position_valid) return ESP_ERR_INVALID_STATE;

    waypoint_t *current_wp = &nav->mission.waypoints[nav->mission.current_waypoint_index];

    float distance, bearing;
    esp_err_t ret = navigation_compute_distance_bearing(&nav->state.current_position, &current_wp->coordinate, &distance, &bearing);

    if (ret != ESP_OK) return ret;

    nav->state.desired_heading = bearing;
    nav->state.target_location = current_wp->coordinate;

    if (distance <= current_wp->acceptance_radius) {
        nav->mission.current_waypoint_index++;

        if (nav->mission.current_waypoint_index >= nav->mission.waypoint_count) {
            nav->mission_completed = true;
            ESP_LOGI(TAG, "Mission completed");
            return navigation_set_mode(nav, NAV_MODE_HOLD_POSITION);
        }
    }

    return ESP_OK;
}

static esp_err_t handle_position_hold(navigation_system_t *nav) {
    if (!nav->position_valid) return ESP_ERR_INVALID_STATE;

    nav->state.target_location = nav->state.current_position;
    return ESP_OK;
}

static esp_err_t handle_return_to_home(navigation_system_t *nav) {
    if (!nav->position_valid) return ESP_ERR_INVALID_STATE;

    nav->state.target_location = nav->state.home_location;
    nav->state.target_location.altitude = nav->config.return_home_altitude;
    
    return ESP_OK;
}

bool navigation_is_mission_complete(const navigation_system_t *nav) {
    return nav ? nav->mission_completed : false;
}
