---
title: Sensor Integration
sidebar_position: 5
---

# Sensor Integration

## Overview

Sensor integration combines multiple sensors into a unified system that provides accurate, reliable perception for robot control and decision-making.

## Integration Architecture

### Hardware Layer
```
Sensors ──→ Microcontroller/SBC ──→ Main Computer
            (Polling, interrupts)    (ROS 2 nodes)
```

### Communication Protocols

#### I2C (Inter-Integrated Circuit)
- Simple, uses only 2 wires (SDA, SCL)
- Multiple sensors on same bus
- Slower speed (100-400 kHz typical)

```python
import smbus

bus = smbus.SMBus(1)  # Bus 1 for RPi
address = 0x68

# Write to sensor
bus.write_byte_data(address, 0x6B, 0)

# Read from sensor
data = bus.read_byte_data(address, 0x3B)
```

#### SPI (Serial Peripheral Interface)
- Faster than I2C (up to 10 MHz)
- Requires separate chip select (CS) line per sensor
- More wires needed

```python
import spidev

spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0
spi.max_speed_hz = 1000000

# Transfer data
data = spi.xfer2([0x00, 0x00])
```

#### Serial (UART)
- Simple point-to-point communication
- Widely supported
- Limited to one device per port

```python
import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)
data = ser.readline()
```

#### CAN Bus
- Robust, used in vehicles and industrial robotics
- Good for real-time distributed systems

## Multi-Sensor Data Processing Pipeline

```python
class RobotPerceptionSystem:
    def __init__(self):
        self.camera = CameraModule()
        self.imu = IMUModule()
        self.encoders = EncoderModule()
        self.lidar = LidarModule()
        
        self.last_readings = {}
        self.fused_state = None
    
    def acquire_sensor_data(self):
        """Collect data from all sensors"""
        readings = {
            'camera': self.camera.read(),
            'imu': self.imu.read(),
            'encoders': self.encoders.read(),
            'lidar': self.lidar.read(),
            'timestamp': time.time()
        }
        return readings
    
    def preprocess(self, readings):
        """Filter and normalize sensor data"""
        processed = {}
        
        # Apply noise filters
        processed['imu'] = self.filter_imu(readings['imu'])
        processed['encoders'] = self.filter_encoders(readings['encoders'])
        
        # Undistort camera image
        processed['camera'] = self.undistort_image(readings['camera'])
        
        return processed
    
    def fuse_data(self, processed):
        """Combine sensor data for better estimates"""
        # Use Kalman filter to fuse encoder and IMU
        position = self.kalman_filter(
            processed['encoders'],
            processed['imu']
        )
        
        # Detect objects with camera + lidar
        objects = self.detect_objects(
            processed['camera'],
            processed['lidar']
        )
        
        return {
            'position': position,
            'objects': objects,
            'raw_sensors': processed
        }
    
    def filter_imu(self, imu_data):
        """Apply complementary filter"""
        # Combines accelerometer and gyroscope
        alpha = 0.98
        filtered = (alpha * self.last_readings.get('gyro', imu_data['gyro']) +
                   (1 - alpha) * imu_data['accel'])
        return filtered
    
    def detect_objects(self, image, lidar):
        """Combine vision and lidar for 3D object detection"""
        # Run object detection on image
        boxes = self.object_detector.predict(image)
        
        # Associate with lidar points
        objects_3d = []
        for box in boxes:
            # Find lidar points within image box projection
            depth = self.get_lidar_depth(box, lidar)
            obj = {
                'class': box.class_name,
                'bbox_2d': box,
                'distance': depth
            }
            objects_3d.append(obj)
        
        return objects_3d

def main():
    perception = RobotPerceptionSystem()
    
    while True:
        # 1. Acquire data from all sensors
        raw_data = perception.acquire_sensor_data()
        
        # 2. Preprocess (filter, normalize)
        processed = perception.preprocess(raw_data)
        
        # 3. Fuse sensor data
        fused = perception.fuse_data(processed)
        
        # 4. Use fused data for control decision
        publish_robot_state(fused)
```

## ROS 2 Integration

### Publishing Sensor Data

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from geometry_msgs.msg import Pose

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        
        self.camera_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Timer callback at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_sensors)
    
    def publish_sensors(self):
        # Publish camera image
        image_msg = self.read_camera()
        self.camera_pub.publish(image_msg)
        
        # Publish IMU data
        imu_msg = self.read_imu()
        self.imu_pub.publish(imu_msg)
        
        # Publish lidar
        scan_msg = self.read_lidar()
        self.lidar_pub.publish(scan_msg)
```

### Subscribing to Sensor Data

```python
class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        self.create_subscription(Image, 'camera/image_raw', 
                                self.image_callback, 10)
        self.create_subscription(Imu, 'imu/data', 
                                self.imu_callback, 10)
    
    def image_callback(self, msg):
        # Process camera image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # ... perform object detection, etc.
    
    def imu_callback(self, msg):
        # Process IMU data
        acc_x = msg.linear_acceleration.x
        # ... perform calculations
```

## Sensor Synchronization

Important for multi-sensor fusion:

```python
import message_filters

class SensorSynchronizer(Node):
    def __init__(self):
        super().__init__('sensor_sync')
        
        # Subscribe to multiple topics
        image_sub = message_filters.Subscriber(self, Image, 'camera/image_raw')
        imu_sub = message_filters.Subscriber(self, Imu, 'imu/data')
        
        # Synchronize messages (approximate time synchronization)
        ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, imu_sub],
            queue_size=10,
            slop=0.1  # Allow 100ms time difference
        )
        
        ts.registerCallback(self.synchronized_callback)
    
    def synchronized_callback(self, image_msg, imu_msg):
        # Both messages arrived at approximately the same time
        self.process_fused_data(image_msg, imu_msg)
```

## Common Integration Challenges

### Challenge 1: Different Sensor Rates
- **Problem**: Sensors update at different frequencies
- **Solution**: Use buffering and interpolation

```python
class SensorBuffer:
    def __init__(self, max_size=100):
        self.buffer = collections.deque(maxlen=max_size)
    
    def add(self, timestamp, data):
        self.buffer.append((timestamp, data))
    
    def interpolate_at(self, target_time):
        """Get interpolated value at target time"""
        # Find two surrounding samples
        # Linearly interpolate between them
        pass
```

### Challenge 2: Sensor Latency
- **Problem**: Delayed sensor readings cause control lag
- **Solution**: Predict sensor data into the future

```python
def predict_state(current_state, velocity, dt):
    """Predict where robot will be in dt seconds"""
    predicted = current_state + velocity * dt
    return predicted
```

### Challenge 3: Sensor Noise
- **Solution**: Apply appropriate filters

```python
class LowPassFilter:
    def __init__(self, cutoff_frequency, sampling_rate):
        self.alpha = 1 / (1 + 2*3.14159*cutoff_frequency/sampling_rate)
        self.last_value = 0
    
    def filter(self, new_value):
        self.last_value = self.alpha * new_value + (1-self.alpha)*self.last_value
        return self.last_value
```

## Next Steps

Now that sensors are integrated, you can build complete robotic systems with autonomous control.
