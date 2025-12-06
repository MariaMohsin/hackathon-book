---
title: Motion Sensors
sidebar_position: 3
---

# Motion Sensors

## Overview

Motion sensors measure the robot's movement, velocity, acceleration, and orientation. They are essential for robot control, navigation, and balancing.

## Encoders

### What Are Encoders?

Encoders measure angular position and velocity of rotating shafts. They are critical for:
- **Joint Position**: Know exact angle of each joint
- **Velocity Control**: Track speed of motors
- **Odometry**: Estimate distance traveled

### Types of Encoders

#### Incremental Encoders
- Detect changes from a reference point
- Output: pulses as shaft rotates
- Resolution: Lines per revolution (LPR)
- Example: 2048 LPR encoder

```python
# Read encoder data via GPIO
import RPi.GPIO as GPIO

ENCODER_PIN = 17
count = 0

def encoder_callback(channel):
    global count
    count += 1

GPIO.setup(ENCODER_PIN, GPIO.IN)
GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING, callback=encoder_callback)

# Calculate distance: distance = count * (wheel_circumference / PPR)
```

#### Absolute Encoders
- Know exact position without reference
- Multiturn encoders track unlimited rotations
- Output: Digital value (CANopen, SSI, etc.)
- More expensive but more reliable

### Calculating Robot Motion from Encoders

```python
class OdometryCalculator:
    def __init__(self, wheel_radius, wheel_distance):
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        self.left_count = 0
        self.right_count = 0
        self.x = 0
        self.y = 0
        self.theta = 0
    
    def update_odometry(self, left_counts, right_counts, dt):
        # Calculate distances traveled
        left_dist = (left_counts * 2 * 3.14159 * self.wheel_radius) / 2048
        right_dist = (right_counts * 2 * 3.14159 * self.wheel_radius) / 2048
        
        # Calculate linear and angular velocity
        linear = (left_dist + right_dist) / 2
        angular = (right_dist - left_dist) / self.wheel_distance
        
        # Update position
        self.theta += angular
        self.x += linear * 3.14159 / 180 * self.theta
        self.y += linear * 3.14159 / 180 * self.theta
        
        return self.x, self.y, self.theta
```

## IMU (Inertial Measurement Unit)

### What is an IMU?

An IMU combines three sensors:
- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity (rotation rate)
- **Magnetometer**: Measures magnetic field (compass)

### IMU Data

```
Accelerometer output: [ax, ay, az] in m/s²
Gyroscope output: [gx, gy, gz] in rad/s
Magnetometer output: [mx, my, mz] in Tesla
```

### Applications

- **Robot Orientation**: Determine which way is up
- **Impact Detection**: Detect collisions
- **Vibration Analysis**: Monitor motor health
- **Balance Control**: Stabilize bipedal robots

### Example: Using MPU-6050 IMU

```python
import smbus
import math
import time

class MPU6050:
    def __init__(self, address=0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.bus.write_byte_data(self.address, 0x6B, 0)
    
    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        value = (high << 8) | low
        if value > 32768:
            value = value - 65536
        return value
    
    def get_accel(self):
        x = self.read_raw_data(0x3B) / 16384.0
        y = self.read_raw_data(0x3D) / 16384.0
        z = self.read_raw_data(0x3F) / 16384.0
        return x, y, z
    
    def get_gyro(self):
        x = self.read_raw_data(0x43) / 131.0
        y = self.read_raw_data(0x45) / 131.0
        z = self.read_raw_data(0x47) / 131.0
        return x, y, z

# Usage
imu = MPU6050()
while True:
    accel = imu.get_accel()
    gyro = imu.get_gyro()
    print(f"Accel: {accel}, Gyro: {gyro}")
    time.sleep(0.1)
```

## Force/Torque Sensors

### Applications
- **Gripper Control**: Measure grip force to avoid crushing objects
- **Compliance**: Detect interaction forces
- **Impedance Control**: React to external forces intelligently

### Example: Monitoring Gripper Force

```python
def monitor_grip_force(force_sensor, target_force, max_force):
    """Maintain consistent grip force"""
    current_force = force_sensor.read()
    
    if current_force < target_force:
        # Increase grip
        return "increase"
    elif current_force > max_force:
        # Decrease grip (too much force)
        return "decrease"
    else:
        # Hold current grip
        return "hold"
```

## Position Sensors

### Types
- **Potentiometers**: Variable resistor tracking position
- **Linear Potentiometers**: Measure linear displacement
- **Draw-wire Sensors**: Measure distance as cable extends

### Applications
- Track end-effector position
- Monitor joint limits
- Feedback for position control

## Sensor Fusion for Better Estimates

Combining multiple motion sensors improves accuracy:

```python
from filterpy.kalman import KalmanFilter
import numpy as np

class MotionEstimator:
    def __init__(self):
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([[0.], [0.], [0.], [0.]])  # [x, y, vx, vy]
        self.kf.F = np.array([[1., 0., 1., 0.],
                              [0., 1., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])  # State transition
        self.kf.H = np.array([[1., 0., 0., 0.],
                              [0., 1., 0., 0.]])  # Measurement function
        self.kf.R = np.eye(2) * 0.1  # Measurement noise
        self.kf.Q = np.eye(4) * 0.001  # Process noise
    
    def update(self, encoder_position, imu_accel):
        """Fuse encoder and IMU data"""
        self.kf.predict()
        self.kf.update(encoder_position)
        return self.kf.x.flatten()
```

## Best Practices

1. **Sensor Placement**: Mount sensors where vibration is minimal
2. **Calibration**: Zero calibrate before each use
3. **Filtering**: Apply low-pass filters to reduce noise
4. **Synchronization**: Timestamp all sensor data
5. **Redundancy**: Use multiple sensors for critical measurements

## Troubleshooting

| Problem | Cause | Solution |
|---|---|---|
| Encoder reading drifts | Slipping wheels | Check tire traction |
| IMU gives random values | I2C communication issue | Check wiring and bus clock |
| Force sensor noisy | Electrical interference | Shield cables, add capacitors |

## Next Steps

Learn how to [Integrate Sensors](sensor-integration) into a complete system.
