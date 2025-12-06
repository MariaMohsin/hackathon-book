---
title: Sensors Overview
sidebar_position: 1
---

# Sensors Overview

## What is a Robot Sensor?

A robot sensor is a device that detects and responds to physical stimuli like light, sound, temperature, pressure, or motion. Sensors are the robot's "perceptual system" - they gather information about the environment and the robot's state.

## Why Sensors Matter

Robots cannot perform tasks without knowing:
- **Where am I?** (Localization)
- **What's around me?** (Environment perception)
- **Am I moving correctly?** (Motion feedback)
- **What should I do next?** (Task-specific data)

## Types of Sensors

### Proprioceptive Sensors
Measure the robot's internal state and position:

- **Encoders**: Measure joint rotation angles
- **IMU (Inertial Measurement Unit)**: Measures acceleration and orientation
- **Force/Torque Sensors**: Measure forces at joints
- **Position Sensors**: Track end-effector location

### Exteroceptive Sensors
Measure the external environment:

- **Camera**: Visual perception (RGB or depth)
- **Lidar**: 3D laser scanning
- **Ultrasonic**: Distance measurement
- **Infrared**: Temperature and proximity
- **Tactile**: Touch and pressure sensing

## Sensor Specifications

When choosing a sensor, consider:

| Specification | Description |
|---|---|
| **Range** | Minimum and maximum detection distance |
| **Resolution** | Smallest detectable change |
| **Accuracy** | How close measurement is to true value |
| **Latency** | Delay between sensing and output |
| **Noise** | Random measurement variations |
| **Field of View** | Angular range of detection |

## Sensor Fusion

Combining multiple sensors improves reliability:

```python
# Example: Sensor fusion with Kalman filter
from filterpy.kalman import KalmanFilter

def fuse_sensors(encoder_data, imu_data):
    # Combines position and motion estimates
    kf = KalmanFilter(dim_x=2, dim_z=2)
    # Configuration and filtering...
    return fused_state
```

## Common Sensor Issues

### Noise
- **Problem**: Electrical interference causes unstable readings
- **Solution**: Filter with moving average or Kalman filter

### Drift
- **Problem**: Sensor reading slowly changes over time
- **Solution**: Periodic recalibration

### Latency
- **Problem**: Delayed sensor data causes control lag
- **Solution**: Predict robot state between readings

## Sensor Calibration

All sensors need calibration:

1. **Zero Calibration**: Set baseline at known state
2. **Scale Calibration**: Map sensor range to physical units
3. **Non-linearity Calibration**: Correct response curves

## Next Steps

Learn about specific sensor types in [Vision Sensors](vision-sensors) and [Motion Sensors](motion-sensors), then explore how to [Integrate Sensors](sensor-integration) into your robot.
