# Quadcopter Hardware Connections Guide

## Overview
This document provides a comprehensive guide to the hardware connections for our ESP32-based quadcopter flight controller, detailing the interconnections between various components, pin mappings, and electrical considerations.

## System Block Diagram

```
                     +-------------------+
                     |   ESP32 DevKit    |
                     |   Microcontroller |
                     +--------+----------+
                              |
         +-------------------+-------------------+
         |                   |                   |
+--------v--------+  +--------v--------+  +--------v--------+
|   MPU6050       |  |   Radio         |  |   Battery       |
|   IMU Sensor    |  |   Receiver      |  |   Monitor       |
+--------+--------+  +--------+--------+  +--------+--------+
         |                   |                   |
         |                   |                   |
+--------v--------+  +--------v--------+  +--------v--------+
|   I2C Interface |  |   PWM Input     |  |   ADC Input     |
|   (SCL/SDA)     |  |   (PPM Signal)  |  |   (Voltage/     |
|                 |  |                 |  |    Current)     |
+-----------------+  +-----------------+  +-----------------+
         |                   |                   |
         |                   |                   |
+--------v--------+  +--------v--------+  +--------v--------+
|   Motor         |  |   ESC           |  |   Power         |
|   Controllers   |  |   (Electronic   |  |   Distribution  |
|                 |  |    Speed Ctrl)  |  |    Board        |
+-----------------+  +-----------------+  +-----------------+
         |                   |                   |
         +-------------------+-------------------+
                     Quadcopter Frame
```

## Detailed Pin Mapping

### ESP32 Pin Configuration

| Component          | ESP32 Pin   | Function               | Notes                         |
|--------------------|-------------|------------------------|-------------------------------|
| MPU6050 (IMU)      | GPIO_NUM_21 | I2C SDA                | Primary sensor communication  |
| MPU6050 (IMU)      | GPIO_NUM_22 | I2C SCL                | Primary sensor communication  |
| Radio Receiver     | GPIO_NUM_18 | PPM Signal Input       | RC receiver input channel     |
| Motor 1            | GPIO_NUM_12 | PWM Output             | Front-Right Motor             |
| Motor 2            | GPIO_NUM_13 | PWM Output             | Rear-Right Motor              |
| Motor 3            | GPIO_NUM_14 | PWM Output             | Rear-Left Motor               |
| Motor 4            | GPIO_NUM_15 | PWM Output             | Front-Left Motor              |
| Battery Voltage    | GPIO_NUM_36 | ADC Input              | Battery voltage sensing       |
| Battery Current    | GPIO_NUM_37 | ADC Input              | Battery current monitoring    |
| RSSI (Optional)    | GPIO_NUM_39 | ADC Input              | Radio signal strength         |

## Power System

### Battery Specifications
- Type: LiPo (Lithium Polymer)
- Cell Count: 4S (14.8V Nominal)
- Capacity: 5000mAh
- Discharge Rate: 50C
- Connector: XT60

### Power Distribution
1. Main Battery Input
   - Connects to Power Distribution Board (PDB)
   - Provides power to:
     - ESCs
     - Flight Controller
     - Onboard electronics

2. Voltage Regulation
   - 5V BEC (Battery Elimination Circuit)
     - Supplies power to RC Receiver
     - Powers ESP32 via USB or dedicated 5V input

## Sensor Connections

### MPU6050 Inertial Measurement Unit (IMU)
- Communication Protocol: I2C
- Address: 0x68
- Connection:
  - VCC: 3.3V
  - GND: Ground
  - SDA: GPIO_NUM_21
  - SCL: GPIO_NUM_22

### Radio Receiver
- Protocol: PPM (Pulse Position Modulation)
- Input Pin: GPIO_NUM_18
- Supported Channels: 8
- Voltage: 3.3V-5V

## Motor and ESC Configuration

### Electronic Speed Controllers (ESCs)
- Type: 30A Brushless ESC
- Input Voltage: 2-4S LiPo
- PWM Frequency: 400 Hz
- Signal Voltage: 3.3V

### Motor Arrangement (X Configuration)
```
    Motor 1 (Front-Right)
          \   /
           \ /
   Motor 4  X  Motor 2
           / \
          /   \
    Motor 3 (Rear-Left)
```

## Wiring Recommendations

1. Use high-quality, flexible silicone-insulated wire
2. Minimum wire gauge for power: 12-14 AWG
3. Signal wires: 20-22 AWG
4. Use connectors with high current rating
5. Implement robust strain relief
6. Add decoupling capacitors near power inputs

## Safety Considerations

1. Install low-voltage cutoff in firmware
2. Use XT60 connectors with anti-spark mechanism
3. Implement hardware and software emergency stop
4. Use vibration-damping mounting for electronics
5. Install prop guards for testing and learning phases

## Troubleshooting Checklist

- [ ] Verify all connections are secure
- [ ] Check for short circuits
- [ ] Confirm correct voltage at each component
- [ ] Verify signal integrity
- [ ] Test continuity of power traces
- [ ] Validate ground connections

## Firmware Configuration Alignment

Ensure `config.h` matches physical hardware:
```c
#define MOTOR_PIN_1    GPIO_NUM_12  // Front-Right
#define MOTOR_PIN_2    GPIO_NUM_13  // Rear-Right
#define MOTOR_PIN_3    GPIO_NUM_14  // Rear-Left
#define MOTOR_PIN_4    GPIO_NUM_15  // Front-Left

#define I2C_MASTER_SDA_PIN  GPIO_NUM_21
#define I2C_MASTER_SCL_PIN  GPIO_NUM_22
```

## Version History
- v1.0: Initial hardware connection specification
- v1.1: Updated power system details
- v1.2: Added safety considerations

## Appendix: Component Datasheets
- [Link to ESP32 Datasheet]
- [Link to MPU6050 Datasheet]
- [Link to ESC Specification]

**Note:** Always refer to specific component datasheets for precise electrical characteristics.