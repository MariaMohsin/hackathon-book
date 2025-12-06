---
title: Vision Sensors
sidebar_position: 2
---

# Vision Sensors

## Overview

Vision sensors provide the most information-rich perception capability for robots. They capture visual data that can be processed for object detection, navigation, and manipulation tasks.

## RGB Cameras

### How They Work
RGB cameras capture three color channels (Red, Green, Blue) at each pixel location.

### Specifications
- **Resolution**: 640x480, 1280x720, 1920x1080, or higher
- **Frame Rate**: 30 FPS, 60 FPS, or higher for fast motion
- **Field of View**: Wide (greater than 90 degrees) for panoramic, normal (~50 degrees), or narrow (less than 30 degrees) for detail

### Applications
- Object detection and recognition
- Visual odometry for navigation
- Quality inspection
- Gesture recognition

### Example: Capturing Images with OpenCV

```python
import cv2

# Open default camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    
    if ret:
        # Process frame
        cv2.imshow('Camera Feed', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()
```

## Depth Cameras

### How They Work
Depth cameras measure distance to objects by:
- **Time-of-Flight (ToF)**: Measure light travel time
- **Structured Light**: Project patterns and analyze reflections
- **Stereo Vision**: Compare images from two cameras

### Popular Depth Cameras
- Intel RealSense D435
- Microsoft Kinect
- Luxonis OAK-D
- Basler depth cameras

### Point Cloud Data

Depth cameras produce point clouds - 3D coordinates of visible surfaces:

```python
import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud("cloud.ply")

# Visualize
o3d.visualization.draw_geometries([pcd])

# Filter outliers
cl, ind = pcd.remove_statistical_outlier(
    nb_neighbors=20, std_ratio=2.0
)
```

### Applications
- 3D object detection and grasping
- 3D mapping and reconstruction
- Obstacle avoidance
- Human pose estimation

## Thermal Cameras

### Characteristics
- Detect heat radiation (IR spectrum)
- Work in darkness
- Useful for predictive maintenance
- Good for search and rescue

### Limitations
- Lower resolution than RGB
- Affected by reflective surfaces
- Cannot see through glass

## Event Cameras

### How They Work
- Record pixel-level brightness changes instead of frames
- Very low latency (microseconds)
- High dynamic range
- Sparse, event-based output

### Advantages
- Extreme sensitivity to motion
- Works in high-speed scenarios
- Low power consumption

### Disadvantages
- Difficult to process (specialized algorithms)
- Limited availability and cost
- Smaller field of view than RGB

## Best Practices for Vision Sensors

### Camera Mounting
- Mount at appropriate height and angle for task
- Avoid backlighting
- Protect from dust and moisture
- Secure firmly to prevent vibration

### Lighting Conditions
- Ensure adequate illumination for RGB cameras
- Avoid specular reflections on glossy surfaces
- Use infrared LEDs with depth cameras for better performance
- Consider different lighting for different tasks

### Image Processing Pipeline

```python
def process_camera_image(frame):
    # 1. Capture image
    # 2. Undistort (correct lens distortion)
    undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
    
    # 3. Convert color space if needed
    hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)
    
    # 4. Apply filters
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
    
    # 5. Detect features
    edges = cv2.Canny(blurred, 100, 200)
    
    # 6. Post-process
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    result = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
    return result
```

## Camera Calibration

Every camera needs calibration to correct lens distortion:

```python
import cv2
import numpy as np

# Detect checkerboard corners in calibration images
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Calculate camera matrix
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)
```

## Integration with ROS 2

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        
        self.timer = self.create_timer(0.033, self.capture_and_publish)
    
    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Next Steps

Learn about [Motion Sensors](motion-sensors) and then explore [Sensor Integration](sensor-integration).
