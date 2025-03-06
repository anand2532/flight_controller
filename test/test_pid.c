// test/test_pid.c
#include <unity.h>
#include <math.h>
#include "control/pid.h"

// Helper function to compare float values with tolerance
void assert_float_equal(float expected, float actual, float tolerance) {
    TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
}

// Test initialization of PID controller
void test_pid_init(void) {
    pid_controller_t pid;
    pid_params_t params = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f,
        .max_output = 100.0f,
        .min_output = -100.0f
    };

    // Test initialization with custom parameters
    bool init_result = pid_init(&pid, PID_ROLL, &params, 0.01f);
    TEST_ASSERT_TRUE(init_result);

    // Verify parameters are set correctly
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.params.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pid.params.ki);
    TEST_ASSERT_EQUAL_FLOAT(0.01f, pid.params.kd);
    TEST_ASSERT_TRUE(pid.is_enabled);
}

// Test PID computation
void test_pid_compute(void) {
    pid_controller_t pid;
    pid_params_t params = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f,
        .max_output = 100.0f,
        .min_output = -100.0f
    };

    pid_init(&pid, PID_ROLL, &params, 0.01f);

    // Scenario 1: No error
    float output1 = pid_compute(&pid, 0.0f, 0.0f);
    assert_float_equal(0.0f, output1, 0.001f);

    // Scenario 2: Proportional term
    float output2 = pid_compute(&pid, 10.0f, 0.0f);
    assert_float_equal(10.0f, output2, 0.001f);

    // Scenario 3: Integral term accumulation
    float output3_1 = pid_compute(&pid, 10.0f, 0.0f);
    float output3_2 = pid_compute(&pid, 10.0f, 0.0f);
    TEST_ASSERT_TRUE(output3_2 > output3_1);

    // Scenario 4: Derivative term
    float output4_1 = pid_compute(&pid, 10.0f, 0.0f);
    float output4_2 = pid_compute(&pid, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(output4_2 < output4_1);
}

// Test PID output limits
void test_pid_output_limits(void) {
    pid_controller_t pid;
    pid_params_t params = {
        .kp = 10.0f,
        .ki = 1.0f,
        .kd = 0.1f,
        .max_output = 50.0f,
        .min_output = -50.0f
    };

    pid_init(&pid, PID_ROLL, &params, 0.01f);

    // Large error to test output limits
    float output = pid_compute(&pid, 10.0f, 0.0f);
    TEST_ASSERT_TRUE(output <= 50.0f);
    TEST_ASSERT_TRUE(output >= -50.0f);
}

// Test PID reset functionality
void test_pid_reset(void) {
    pid_controller_t pid;
    pid_params_t params = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f
    };

    pid_init(&pid, PID_ROLL, &params, 0.01f);

    // Compute some outputs to modify internal state
    pid_compute(&pid, 10.0f, 0.0f);
    pid_compute(&pid, 10.0f, 0.0f);

    // Reset PID
    pid_reset(&pid);

    // Check that state has been reset
    pid_state_t state;
    pid_get_state(&pid, &state);
    
    assert_float_equal(0.0f, state.integral, 0.001f);
    assert_float_equal(0.0f, state.last_error, 0.001f);
    assert_float_equal(0.0f, state.output, 0.001f);
}

// Test PID tuning
void test_pid_tune(void) {
    pid_controller_t pid;
    pid_params_t initial_params = {
        .kp = 1.0f,
        .ki = 0.1f,
        .kd = 0.01f
    };

    pid_init(&pid, PID_ROLL, &initial_params, 0.01f);

    // New tuning parameters
    pid_params_t new_params = {
        .kp = 2.0f,
        .ki = 0.2f,
        .kd = 0.02f
    };

    // Tune PID
    pid_tune(&pid, &new_params);

    // Verify parameters have been updated
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.params.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, pid.params.ki);
    TEST_ASSERT_EQUAL_FLOAT(0.02f, pid.params.kd);
}

// Collect all tests
void run_pid_tests(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_init);
    RUN_TEST(test_pid_compute);
    RUN_TEST(test_pid_output_limits);
    RUN_TEST(test_pid_reset);
    RUN_TEST(test_pid_tune);
    UNITY_END();
}

// Optional main for standalone testing
#ifdef STANDALONE_TEST
int main(void) {
    run_pid_tests();
    return 0;
}
#endif