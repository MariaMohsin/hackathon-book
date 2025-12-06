---
title: Actuators
sidebar_position: 4
---

# Actuators

## What is an Actuator?

An actuator is a device that converts electrical energy into mechanical motion. Actuators are the robot's "muscles" that enable movement and interaction with the environment.

## Types of Actuators

### DC Motors

#### Characteristics
- Simple, cost-effective
- Wide range of sizes and power ratings
- Direct proportional relationship between voltage and speed
- Produce significant torque

#### Control

```python
import RPi.GPIO as GPIO

class DCMotor:
    def __init__(self, pin_forward, pin_backward, pwm_pin, frequency=1000):
        self.pin_f = pin_forward
        self.pin_b = pin_backward
        self.pwm_pin = pwm_pin
        
        GPIO.setup(pin_forward, GPIO.OUT)
        GPIO.setup(pin_backward, GPIO.OUT)
        GPIO.setup(pwm_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(pwm_pin, frequency)
        self.pwm.start(0)
    
    def forward(self, speed):
        """Speed: 0-100 percent"""
        GPIO.output(self.pin_f, GPIO.HIGH)
        GPIO.output(self.pin_b, GPIO.LOW)
        self.pwm.ChangeDutyCycle(speed)
    
    def backward(self, speed):
        GPIO.output(self.pin_f, GPIO.LOW)
        GPIO.output(self.pin_b, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(speed)
    
    def stop(self):
        self.pwm.ChangeDutyCycle(0)
```

### Servo Motors

#### Characteristics
- Closed-loop control with feedback
- Limited rotation (typically 180°)
- High precision positioning
- Ideal for precise joint angles

#### Control

```python
import RPi.GPIO as GPIO
import time

class ServoMotor:
    def __init__(self, pin, min_angle=0, max_angle=180):
        self.pin = pin
        self.min_angle = min_angle
        self.max_angle = max_angle
        
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 50)  # 50 Hz for servo
        self.pwm.start(0)
    
    def set_angle(self, angle):
        """Set servo to specific angle"""
        # Duty cycle: 5% = 0°, 10% = 180°
        duty_cycle = 5 + (angle / 18)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.01)
    
    def sweep(self):
        for angle in range(self.min_angle, self.max_angle):
            self.set_angle(angle)
            time.sleep(0.01)
```

### Stepper Motors

#### Characteristics
- Precise position control without feedback
- High holding torque
- Discrete steps (no smooth motion unless microstepping)
- Good for accurate positioning

#### Advantages
- Know exact position (no encoders needed)
- Can hold position with power
- Predictable motion

#### Applications
- 3D printers and CNC machines
- Robotic arms requiring precise positioning
- Camera pan/tilt mechanisms

### Brushless Motors (BLDC)

#### Characteristics
- More efficient than brushed DC motors
- Longer lifespan
- Higher speed capability
- Require electronic speed controller (ESC)

#### Applications
- Quadrotor drones
- High-speed robot wheels
- High-performance robotic actuators

### Pneumatic Actuators

#### Characteristics
- Powered by compressed air
- Simple, lightweight design
- Fail-safe (collapse when air pressure lost)
- Clean operation (no electrical hazards)

#### Disadvantages
- Require air compressor
- Less precise than electric motors
- Compressor adds size/weight

### Hydraulic Actuators

#### Characteristics
- Highest power-to-weight ratio
- Precise force control
- Expensive, complex
- Commonly used in industrial robots

## Motor Control with ROS 2

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        self.motor1 = DCMotor(17, 27, 22)
        self.motor2 = DCMotor(23, 24, 25)
        
        # Subscribe to motor commands
        self.create_subscription(Float32, 'motor1/command', self.motor1_callback, 10)
        self.create_subscription(Float32, 'motor2/command', self.motor2_callback, 10)
    
    def motor1_callback(self, msg):
        speed = msg.data  # -100 to 100
        if speed > 0:
            self.motor1.forward(speed)
        elif speed < 0:
            self.motor1.backward(-speed)
        else:
            self.motor1.stop()
    
    def motor2_callback(self, msg):
        # Similar to motor1
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Power Requirements

### Motor Current Calculation

```python
# P = V × I (power = voltage × current)
# I = P / V (current = power / voltage)

def calculate_motor_current(power_watts, voltage):
    """Calculate required current"""
    return power_watts / voltage

# Example: 50W motor at 12V
current = calculate_motor_current(50, 12)  # 4.17 Amps
```

### Power Supply Selection

- Choose PSU with 20-30% headroom above peak current
- Separate power supplies for logic and motors recommended
- High-current motors need thicker wires (check AWG rating)

## Motor Selection

When choosing a motor, consider:

| Parameter | Description |
|---|---|
| **Torque** | Rotational force needed (N-m) |
| **Speed** | RPM required |
| **Voltage** | Operating voltage |
| **Current** | Maximum current draw |
| **Efficiency** | Percentage of electrical power converted to mechanical |
| **Weight** | Important for mobile robots |
| **Cost** | Budget constraints |

## Gearboxes and Reducers

### Purpose
- Increase torque while reducing speed
- Trade-off: reduced speed for increased force

### Gear Ratio Calculation

```python
def gear_ratio_effect(input_torque, input_speed, gear_ratio, efficiency=0.95):
    """Calculate output torque and speed"""
    output_torque = input_torque * gear_ratio * efficiency
    output_speed = input_speed / gear_ratio
    return output_torque, output_speed

# Example: 1:50 gearbox
output_torque, output_speed = gear_ratio_effect(1, 3000, 50)
print(f"Output: {output_torque} N·m at {output_speed} RPM")
```

## Next Steps

Learn about [Sensor Integration](sensor-integration) to combine sensors and actuators into a complete control system.
