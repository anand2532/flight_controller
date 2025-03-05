// test/test_math_utils.c
#include <unity.h>
#include <math.h>
#include "utils/math_utils.h"

// Helper function to compare float values with tolerance
void assert_float_equal(float expected, float actual, float tolerance) {
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
}

// Test vector dot product
void test_vec3_dot_product(void) {
    vec3_t a = {1.0f, 2.0f, 3.0f};
    vec3_t b = {4.0f, 5.0f, 6.0f};
    
    float result = vec3_dot_product(&a, &b);
    TEST_ASSERT_EQUAL_FLOAT(32.0f, result);
}

// Test vector cross product
void test_vec3_cross_product(void) {
    vec3_t a = {1.0f, 2.0f, 3.0f};
    vec3_t b = {4.0f, 5.0f, 6.0f};
    vec3_t result;
    
    vec3_cross_product(&a, &b, &result);
    
    TEST_ASSERT_EQUAL_FLOAT(-3.0f, result.x);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, result.y);
    TEST_ASSERT_EQUAL_FLOAT(-3.0f, result.z);
}

// Test vector normalization
void test_vec3_normalize(void) {
    vec3_t vector = {3.0f, 4.0f, 0.0f};
    float magnitude = vec3_normalize(&vector);
    
    // Check magnitude
    TEST_ASSERT_EQUAL_FLOAT(5.0f, magnitude);
    
    // Check normalized vector
    assert_float_equal(3.0f/5.0f, vector.x, 0.001f);
    assert_float_equal(4.0f/5.0f, vector.y, 0.001f);
    assert_float_equal(0.0f, vector.z, 0.001f);
}

// Test vector magnitude
void test_vec3_magnitude(void) {
    vec3_t vector = {3.0f, 4.0f, 0.0f};
    float mag = vec3_magnitude(&vector);
    
    TEST_ASSERT_EQUAL_FLOAT(5.0f, mag);
}

// Test Euler to Quaternion conversion
void test_euler_to_quaternion(void) {
    quaternion_t q;
    euler_to_quaternion(DEG_TO_RAD(30.0f), DEG_TO_RAD(45.0f), DEG_TO_RAD(60.0f), &q);
    
    // Quaternion magnitude should be close to 1
    float magnitude = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    assert_float_equal(1.0f, magnitude, 0.001f);
}

// Test Quaternion to Euler conversion
void test_quaternion_to_euler(void) {
    quaternion_t q;
    float roll, pitch, yaw;
    
    // Create a quaternion from known Euler angles
    euler_to_quaternion(DEG_TO_RAD(30.0f), DEG_TO_RAD(45.0f), DEG_TO_RAD(60.0f), &q);
    
    // Convert back to Euler
    quaternion_to_euler(&q, &roll, &pitch, &yaw);
    
    // Check converted angles (within tolerance)
    assert_float_equal(30.0f, RAD_TO_DEG(roll), 1.0f);
    assert_float_equal(45.0f, RAD_TO_DEG(pitch), 1.0f);
    assert_float_equal(60.0f, RAD_TO_DEG(yaw), 1.0f);
}

// Test quaternion vector rotation
void test_quaternion_rotate_vector(void) {
    vec3_t v = {1.0f, 0.0f, 0.0f};
    quaternion_t q;
    vec3_t result;
    
    // Rotate 90 degrees around Z axis
    euler_to_quaternion(0.0f, 0.0f, DEG_TO_RAD(90.0f), &q);
    
    quaternion_rotate_vector(&v, &q, &result);
    
    assert_float_equal(0.0f, result.x, 0.001f);
    assert_float_equal(1.0f, result.y, 0.001f);
    assert_float_equal(0.0f, result.z, 0.001f);
}

// Test linear interpolation
void test_lerp(void) {
    float result1 = lerp(0.0f, 10.0f, 0.5f);
    float result2 = lerp(0.0f, 10.0f, 0.0f);
    float result3 = lerp(0.0f, 10.0f, 1.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(5.0f, result1);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result2);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, result3);
}

// Test value constraining
void test_constrain(void) {
    float result1 = constrain(5.0f, 0.0f, 10.0f);
    float result2 = constrain(-5.0f, 0.0f, 10.0f);
    float result3 = constrain(15.0f, 0.0f, 10.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(5.0f, result1);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result2);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, result3);
}

// Test low-pass filter
void test_low_pass_filter(void) {
    float prev_filtered = 0.0f;
    float current = 10.0f;
    float alpha = 0.1f;
    
    float filtered = low_pass_filter(current, prev_filtered, alpha);
    
    // Result should be between previous and current value
    TEST_ASSERT_TRUE(filtered > prev_filtered);
    TEST_ASSERT_TRUE(filtered < current);
}

// Test moving average filter
void test_moving_average_filter(void) {
    float buffer[5] = {0};
    size_t index = 0;
    
    // Add some values
    float avg1 = moving_average_filter(buffer, 5, 10.0f, &index);
    float avg2 = moving_average_filter(buffer, 5, 20.0f, &index);
    float avg3 = moving_average_filter(buffer, 5, 30.0f, &index);
    
    // Check average calculation
    assert_float_equal(10.0f, avg1, 0.001f);
    assert_float_equal(15.0f, avg2, 0.001f);
    assert_float_equal(20.0f, avg3, 0.001f);
}

// Test great circle distance
void test_great_circle_distance(void) {
    // Known distance between two points
    float distance = great_circle_distance(
        40.7128f,  // New York Latitude
        -74.0060f, // New York Longitude
        34.0522f,  // Los Angeles Latitude
        -118.2437f // Los Angeles Longitude
    );
    
    // Approximate distance between New York and Los Angeles
    assert_float_equal(3935000.0f, distance, 100000.0f);
}

// Test initial bearing
void test_initial_bearing(void) {
    float bearing = initial_bearing(
        40.7128f,  // New York Latitude
        -74.0060f, // New York Longitude
        34.0522f,  // Los Angeles Latitude
        -118.2437f // Los Angeles Longitude
    );
    
    // Bearing should be around 292 degrees (west-northwest)
    assert_float_equal(292.0f, bearing, 5.0f);
}

// Collect all tests
void run_math_utils_tests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_vec3_dot_product);
    RUN_TEST(test_vec3_cross_product);
    RUN_TEST(test_vec3_normalize);
    RUN_TEST(test_vec3_magnitude);
    RUN_TEST(test_euler_to_quaternion);
    RUN_TEST(test_quaternion_to_euler);
    RUN_TEST(test_quaternion_rotate_vector);
    RUN_TEST(test_lerp);
    RUN_TEST(test_constrain);
    RUN_TEST(test_low_pass_filter);
    RUN_TEST(test_moving_average_filter);
    RUN_TEST(test_great_circle_distance);
    RUN_TEST(test_initial_bearing);
    UNITY_END();
}

// Optional main for standalone testing
#ifdef STANDALONE_TEST
int main(void) {
    run_math_utils_tests();
    return 0;
}
#endif