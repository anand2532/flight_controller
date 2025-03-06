// include/utils/math_utils.h
#ifndef QUADCOPTER_MATH_UTILS_H
#define QUADCOPTER_MATH_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Mathematical constants
#define M_PI_F        3.14159265358979323846f
#define DEG_TO_RAD(x) ((x) * (M_PI_F / 180.0f))
#define RAD_TO_DEG(x) ((x) * (180.0f / M_PI_F))

// 3D Vector structure
typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

// Quaternion structure
typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

// Matrix 3x3 structure
typedef struct {
    float m[3][3];
} matrix3x3_t;

/**
 * @brief Compute dot product of two 3D vectors
 * @param a First vector
 * @param b Second vector
 * @return Dot product
 */
float vec3_dot_product(const vec3_t *a, const vec3_t *b);

/**
 * @brief Compute cross product of two 3D vectors
 * @param a First vector
 * @param b Second vector
 * @param result Resulting vector
 */
void vec3_cross_product(const vec3_t *a, const vec3_t *b, vec3_t *result);

/**
 * @brief Normalize a 3D vector
 * @param vector Vector to normalize
 * @return Normalized vector length
 */
float vec3_normalize(vec3_t *vector);

/**
 * @brief Compute vector magnitude
 * @param vector Input vector
 * @return Vector magnitude
 */
float vec3_magnitude(const vec3_t *vector);

/**
 * @brief Convert Euler angles to quaternion
 * @param roll Roll angle (radians)
 * @param pitch Pitch angle (radians)
 * @param yaw Yaw angle (radians)
 * @param result Resulting quaternion
 */
void euler_to_quaternion(
    float roll, 
    float pitch, 
    float yaw, 
    quaternion_t *result
);

/**
 * @brief Convert quaternion to Euler angles
 * @param q Input quaternion
 * @param roll Pointer to store roll angle (radians)
 * @param pitch Pointer to store pitch angle (radians)
 * @param yaw Pointer to store yaw angle (radians)
 */
void quaternion_to_euler(
    const quaternion_t *q, 
    float *roll, 
    float *pitch, 
    float *yaw
);

/**
 * @brief Rotate a vector by a quaternion
 * @param v Vector to rotate
 * @param q Rotation quaternion
 * @param result Rotated vector
 */
void quaternion_rotate_vector(
    const vec3_t *v, 
    const quaternion_t *q, 
    vec3_t *result
);

/**
 * @brief Compute quaternion multiplication
 * @param a First quaternion
 * @param b Second quaternion
 * @param result Resulting quaternion
 */
void quaternion_multiply(
    const quaternion_t *a, 
    const quaternion_t *b, 
    quaternion_t *result
);

/**
 * @brief Linear interpolation between two values
 * @param a Start value
 * @param b End value
 * @param t Interpolation factor (0.0 to 1.0)
 * @return Interpolated value
 */
float lerp(float a, float b, float t);

/**
 * @brief Constrain a value between min and max
 * @param value Input value
 * @param min Minimum value
 * @param max Maximum value
 * @return Constrained value
 */
float constrain(float value, float min, float max);

/**
 * @brief Low-pass filter implementation
 * @param current_value Current input value
 * @param prev_filtered_value Previous filtered value
 * @param alpha Smoothing factor (0.0 to 1.0)
 * @return Filtered value
 */
float low_pass_filter(
    float current_value, 
    float prev_filtered_value, 
    float alpha
);

/**
 * @brief Moving average filter
 * @param buffer Circular buffer of values
 * @param buffer_size Size of the buffer
 * @param new_value New value to add
 * @param current_index Pointer to current buffer index
 * @return Computed moving average
 */
float moving_average_filter(
    float *buffer, 
    size_t buffer_size, 
    float new_value, 
    size_t *current_index
);

/**
 * @brief Compute great circle distance between two points
 * @param lat1 Latitude of first point (degrees)
 * @param lon1 Longitude of first point (degrees)
 * @param lat2 Latitude of second point (degrees)
 * @param lon2 Longitude of second point (degrees)
 * @return Distance in meters
 */
float great_circle_distance(
    float lat1, 
    float lon1, 
    float lat2, 
    float lon2
);

/**
 * @brief Compute initial bearing between two points
 * @param lat1 Latitude of first point (degrees)
 * @param lon1 Longitude of first point (degrees)
 * @param lat2 Latitude of second point (degrees)
 * @param lon2 Longitude of second point (degrees)
 * @return Initial bearing in degrees
 */
float initial_bearing(
    float lat1, 
    float lon1, 
    float lat2, 
    float lon2
);

#endif // QUADCOPTER_MATH_UTILS_H
