// include/config.h
#ifndef QUADCOPTER_GLOBAL_CONFIG_H
#define QUADCOPTER_GLOBAL_CONFIG_H

// System Configuration
#define QUADCOPTER_FIRMWARE_VERSION    "1.0.0"
#define QUADCOPTER_HARDWARE_VERSION    "1.0"

// Flight Control Parameters
#define FLIGHT_CONTROL_LOOP_RATE_HZ    400     // Control loop frequency
#define FLIGHT_CONTROL_PERIOD_MS       (1000 / FLIGHT_CONTROL_LOOP_RATE_HZ)

// Hardware Pin Configurations
// I2C Configuration
#define I2C_MASTER_SCL_PIN             GPIO_NUM_22
#define I2C_MASTER_SDA_PIN             GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ             400000

// MPU6050 Sensor Configuration
#define MPU6050_I2C_ADDRESS            0x68
#define MPU6050_INT_PIN                GPIO_NUM_19

// Motor Control Configuration
#define MOTOR_PWM_FREQUENCY_HZ         400
#define MOTOR_PIN_1                    GPIO_NUM_12
#define MOTOR_PIN_2                    GPIO_NUM_13
#define MOTOR_PIN_3                    GPIO_NUM_14
#define MOTOR_PIN_4                    GPIO_NUM_15

// Radio Receiver Configuration
#define RECEIVER_PPM_PIN               GPIO_NUM_18

// Battery Monitoring Configuration
#define BATTERY_VOLTAGE_PIN            GPIO_NUM_36
#define BATTERY_CURRENT_PIN            GPIO_NUM_37

// Safety Limits
#define MAX_ANGLE_SETPOINT             45.0f   // Maximum allowed angle (degrees)
#define MAX_ROTATION_RATE              250.0f  // Maximum rotation rate (degrees/sec)

// Battery Configuration
#define BATTERY_LOW_VOLTAGE            3.7f    // Low battery threshold (V)
#define BATTERY_CRITICAL_VOLTAGE       3.5f    // Critical battery threshold (V)
#define BATTERY_CELL_COUNT             4       // Number of battery cells

// Logging Configuration
#define ENABLE_DEBUG_LOGGING           1
#define LOG_BUFFER_SIZE                100     // Number of log entries to store
#define LOG_FILE_PATH                  "/sdcard/quadcopter_log.txt"

// Quadcopter Frame Types
typedef enum {
    FRAME_TYPE_X,
    FRAME_TYPE_PLUS,
    FRAME_TYPE_V
} quadcopter_frame_type_t;

// Default Configuration Structures
typedef struct {
    quadcopter_frame_type_t frame_type;
    float max_horizontal_speed;
    float max_vertical_speed;
    float max_tilt_angle;
} airframe_config_t;

typedef struct {
    float kp_roll;
    float ki_roll;
    float kd_roll;
    float kp_pitch;
    float ki_pitch;
    float kd_pitch;
    float kp_yaw;
    float ki_yaw;
    float kd_yaw;
} pid_gains_config_t;

// Default Configurations
extern const airframe_config_t DEFAULT_AIRFRAME_CONFIG;
extern const pid_gains_config_t DEFAULT_PID_GAINS;

// Function to initialize default configurations
void config_init(void);

// Function to load saved configuration
esp_err_t config_load(void);

// Function to save current configuration
esp_err_t config_save(void);

#endif // QUADCOPTER_GLOBAL_CONFIG_H