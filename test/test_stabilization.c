// test/test_stabilization.c
#include <unity.h>
#include <math.h>
#include "control/stabilization.h"
#include "drivers/mpu6050.h"
#include "drivers/radio_receiver.h"

// Helper function to compare float values with tolerance
void assert_float_equal(float expected, float actual, float tolerance) {
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
}

// Test stabilization system initialization
void test_stabilization_init(void) {
    stabilization_system_t stabilization;
    stabilization_config_t config = {
        .mode = STABILIZE_MODE_ANGLE,
        .max_angle_setpoint = 45.0f,
        .max_rate_setpoint = 250.0f,
        .acro_rate_multiplier = 1.0f
    };

    esp_err_t init_result = stabilization_init(&stabilization, &config);
    TEST_ASSERT_EQUAL(ESP_OK, init_result);

    // Verify configuration
    TEST_ASSERT_EQUAL(STABILIZE_MODE_ANGLE, stabilization.config.mode);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, stabilization.config.max_angle_setpoint);
}

// Test orientation computation
void test_compute_orientation(void) {
    // Simulate sensor data
    mpu6050_data_t sensor_data = {
        .scaled = {
            .accel_x = 0.0f,
            .accel_y = 0.0f,
            .accel_z = 9.81f,  // Gravity
            .gyro_x = 0.0f,
            .gyro_y = 0.0f,
            .gyro_z = 0.0f
        }
    };

    quadcopter_orientation_t orientation;
    esp_err_t result = compute_quadcopter_orientation(&sensor_data, &orientation);

    TEST_ASSERT_EQUAL(ESP_OK, result);

    // Level orientation should be close to 0
    assert_float_equal(0.0f, orientation.roll, 0.1f);
    assert_float_equal(0.0f, orientation.pitch, 0.1f);
}

// Test stabilization with different flight modes
void test_stabilization_modes(void) {
    stabilization_system_t stabilization;
    stabilization_config_t config = {
        .mode = STABILIZE_MODE_ANGLE,
        .max_angle_setpoint = 45.0f,
        .max_rate_setpoint = 250.0f
    };
    stabilization_init(&stabilization, &config);

    // Simulate receiver input and sensor data
    mpu6050_data_t sensor_data = {
        .scaled = {
            .accel_x = 0.0f,
            .accel_y = 0.0f,
            .accel_z = 9.81f,
            .gyro_x = 0.0f,
            .gyro_y = 0.0f,
            .gyro_z = 0.0f
        }
    };

    receiver_input_t receiver_input = {
        .normalized_channels = {0},
        .current_flight_mode = FLIGHT_MODE_STABILIZE
    };
    receiver_input.normalized_channels[ROLL_CHANNEL] = 0.5f;  // 50% roll
    receiver_input.normalized_channels[PITCH_CHANNEL] = 0.0f;
    receiver_input.normalized_channels[YAW_CHANNEL] = 0.0f;
    receiver_input.normalized_channels[THROTTLE_CHANNEL] = 0.5f;

    motor_outputs_t motor_outputs;
    esp_err_t result = stabilize_quadcopter(&sensor_data, &receiver_input, &motor_outputs);

    TEST_ASSERT_EQUAL(ESP_OK, result);

    // Verify motor outputs exist and are within expected range
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(motor_outputs.speeds[i] >= 1000);
        TEST_ASSERT_TRUE(motor_outputs.speeds[i] <= 2000);
    }
}

// Test safety monitoring
void test_stabilization_safety_monitor(void) {
    stabilization_system_t stabilization;
    stabilization_init(&stabilization, NULL);

    struct {
        bool excessive_tilt;
        bool rapid_rotation;
        bool orientation_loss;
    } safety_status;

    // Simulate an extreme orientation scenario
    stabilization.current_orientation.roll = 60.0f;   // Excessive tilt
    stabilization.current_orientation.pitch = 0.0f;
    stabilization.current_orientation.roll_rate = 500.0f;  // Rapid rotation

    esp_err_t result = stabilization_safety_monitor(&stabilization, &safety_status);

    TEST_ASSERT_EQUAL(ESP_OK, result);
    TEST_ASSERT_TRUE(safety_status.excessive_tilt);
    TEST_ASSERT_TRUE(safety_status.rapid_rotation);
    TEST_ASSERT_TRUE(safety_status.orientation_loss);
}

// Test stabilization reset
void test_stabilization_reset(void) {
    stabilization_system_t stabilization;
    stabilization_init(&stabilization, NULL);

    // Modify some internal state
    stabilization.current_orientation.roll = 45.0f;
    stabilization.is_armed = true;

    // Reset stabilization
    esp_err_t result = stabilization_reset(&stabilization);

    TEST_ASSERT_EQUAL(ESP_OK, result);

    // Verify reset
    TEST_ASSERT_EQUAL_FLOAT(0.0f, stabilization.current_orientation.roll);
    TEST_ASSERT_FALSE(stabilization.is_armed);
}

// Collect all tests
void run_stabilization_tests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_stabilization_init);
    RUN_TEST(test_compute_orientation);
    RUN_TEST(test_stabilization_modes);
    RUN_TEST(test_stabilization_safety_monitor);
    RUN_TEST(test_stabilization_reset);
    UNITY_END();
}

// Optional main for standalone testing
#ifdef STANDALONE_TEST
int main(void) {
    run_stabilization_tests();
    return 0;
}
#endif