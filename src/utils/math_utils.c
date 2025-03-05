// src/utils/math_utils.c
#include <math.h>
#include <string.h>
#include "utils/math_utils.h"

// Earth's radius in meters
#define EARTH_RADIUS 6371000.0f

float vec3_dot_product(const vec3_t *a, const vec3_t *b) {
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vec3_cross_product(const vec3_t *a, const vec3_t *b, vec3_t *result) {
    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
}

float vec3_normalize(vec3_t *vector) {
    float mag = vec3_magnitude(vector);
    
    if (mag > 0.0f) {
        vector->x /= mag;
        vector->y /= mag;
        vector->z /= mag;
    }
    
    return mag;
}

float vec3_magnitude(const vec3_t *vector) {
    return sqrtf(
        vector->x * vector->x + 
        vector->y * vector->y + 
        vector->z * vector->z
    );
}

void euler_to_quaternion(
    float roll, 
    float pitch, 
    float yaw, 
    quaternion_t *result
) {
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    result->w = cr * cp * cy + sr * sp * sy;
    result->x = sr * cp * cy - cr * sp * sy;
    result->y = cr * sp * cy + sr * cp * sy;
    result->z = cr * cp * sy - sr * sp * cy;
}

void quaternion_to_euler(
    const quaternion_t *q, 
    float *roll, 
    float *pitch, 
    float *yaw
) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI_F / 2.0f, sinp);
    } else {
        *pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void quaternion_rotate_vector(
    const vec3_t *v, 
    const quaternion_t *q, 
    vec3_t *result
) {
    quaternion_t v_quat = {0, v->x, v->y, v->z};
    quaternion_t q_conj = {q->w, -q->x, -q->y, -q->z};
    quaternion_t rotated_q;

    // Rotation: q * v * q^-1
    quaternion_t temp;
    quaternion_multiply(q, &v_quat, &temp);
    quaternion_multiply(&temp, &q_conj, &rotated_q);

    result->x = rotated_q.x;
    result->y = rotated_q.y;
    result->z = rotated_q.z;
}

void quaternion_multiply(
    const quaternion_t *a, 
    const quaternion_t *b, 
    quaternion_t *result
) {
    result->w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    result->x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    result->y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    result->z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
}

float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float low_pass_filter(
    float current_value, 
    float prev_filtered_value, 
    float alpha
) {
    // Constrain alpha between 0 and 1
    alpha = constrain(alpha, 0.0f, 1.0f);
    
    return alpha * current_value + (1.0f - alpha) * prev_filtered_value;
}

float moving_average_filter(
    float *buffer, 
    size_t buffer_size, 
    float new_value, 
    size_t *current_index
) {
    // Add new value to circular buffer
    buffer[*current_index] = new_value;
    
    // Update index with wraparound
    *current_index = (*current_index + 1) % buffer_size;
    
    // Compute average
    float sum = 0.0f;
    for (size_t i = 0; i < buffer_size; i++) {
        sum += buffer[i];
    }
    
    return sum / buffer_size;
}

float great_circle_distance(
    float lat1, 
    float lon1, 
    float lat2, 
    float lon2
) {
    // Convert to radians
    float lat1_rad = DEG_TO_RAD(lat1);
    float lon1_rad = DEG_TO_RAD(lon1);
    float lat2_rad = DEG_TO_RAD(lat2);
    float lon2_rad = DEG_TO_RAD(lon2);

    // Haversine formula
    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;

    float a = sinf(dlat/2) * sinf(dlat/2) +
              cosf(lat1_rad) * cosf(lat2_rad) *
              sinf(dlon/2) * sinf(dlon/2);
    
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    return EARTH_RADIUS * c;
}

float initial_bearing(
    float lat1, 
    float lon1, 
    float lat2, 
    float lon2
) {
    // Convert to radians
    float lat1_rad = DEG_TO_RAD(lat1);
    float lon1_rad = DEG_TO_RAD(lon1);
    float lat2_rad = DEG_TO_RAD(lat2);
    float lon2_rad = DEG_TO_RAD(lon2);

    float dlon = lon2_rad - lon1_rad;

    float y = sinf(dlon) * cosf(lat2_rad);
    float x = cosf(lat1_rad) * sinf(lat2_rad) - 
              sinf(lat1_rad) * cosf(lat2_rad) * cosf(dlon);
    
    float bearing = atan2f(y, x);
    
    // Convert to degrees and normalize to 0-360
    bearing = RAD_TO_DEG(bearing);
    if (bearing < 0.0f) {
        bearing += 360.0f;
    }

    return bearing;
}