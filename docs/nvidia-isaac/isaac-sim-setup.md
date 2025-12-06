---
title: Isaac Sim Setup
sidebar_position: 2
---

# Isaac Sim Setup

## Installation

### Step 1: Download Isaac Sim

Visit [NVIDIA Developer Website](https://developer.nvidia.com/isaac) and download Isaac Sim for your platform.

### Step 2: System Preparation

```bash
# Update system packages
sudo apt-get update
sudo apt-get upgrade

# Install NVIDIA CUDA Toolkit (if not already installed)
sudo apt-get install nvidia-cuda-toolkit

# Verify GPU
nvidia-smi
```

### Step 3: Install Isaac Sim

```bash
# Extract downloaded archive
tar -xzf isaac_sim_*.tar.gz

# Navigate to directory
cd isaac_sim

# Install dependencies
./install_dependencies.sh

# Run Isaac Sim
./isaac-sim.sh
```

## First Launch

### Launching Isaac Sim

```bash
cd ~/.local/share/ov/pkg/isaac_sim-*
./isaac-sim.sh
```

### Understanding the Interface

The main window includes:
- **Viewport**: 3D scene visualization
- **Stage**: Hierarchy of objects and actors
- **Property Panel**: Object properties and settings
- **Console**: Python output and debugging

## Creating Your First Scene

### Step 1: Add a Ground Plane

1. In the **Stage** panel, right-click → Create → Mesh → Ground Plane
2. Set size to 100x100 meters

### Step 2: Add a Robot

```python
# Using Isaac Sim API
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World

simulation_app = SimulationApp({"headless": False})
world = World(stage_units_in_meters=1.0)

# Load a robot from URDF
robot = world.scene.add(
    Articulation(
        prim_path="/World/robot",
        usd_path="model://path_to_robot.usd"
    )
)

simulation_app.app.update()
world.step(render=True)
```

### Step 3: Add Objects to Interact With

```python
# Add a cube
from omni.isaac.core.objects import DynamicCuboid

cube = world.scene.add(
    DynamicCuboid(
        name="cube",
        prim_path="/World/cube",
        position=[0.5, 0, 0.5],
        scale=[0.1, 0.1, 0.1]
    )
)
```

## Physics Configuration

### Setting Physics Parameters

```python
from omni.isaac.core.physics_context import PhysicsContext

# Create physics context
physics_context = PhysicsContext(
    physics_dt=1/500.0,  # 500 Hz physics simulation
    stage_units_in_meters=1.0
)

# Set gravity
physics_context.set_gravity(-9.81)

# Set default material properties
physics_context.set_default_material_properties(
    static_friction=0.5,
    dynamic_friction=0.5,
    restitution=0.1
)
```

## Camera Configuration

### Adding and Configuring Cameras

```python
from omni.isaac.core.objects.camera import Camera

camera = Camera(
    prim_path="/World/camera",
    position=[1, 1, 1],
    rotation=[0, 0, 0]
)

# Set camera parameters
camera.set_focal_length(50.0)
camera.set_clipping_range(0.1, 1000.0)
camera.set_resolution([1920, 1080])
```

### Capturing Images

```python
from omni.isaac.core import World
import cv2
import numpy as np

world = World()

# Render and get image
world.step(render=True)
rgb_image = camera.get_rgb()

# Save image
cv2.imwrite('capture.png', rgb_image)
```

## Sensor Simulation

### Adding a Lidar Sensor

```python
from omni.isaac.sensor import Lidar

lidar = Lidar(
    prim_path="/World/robot/lidar",
    translation=[0, 0, 0.5],
    rotation=[0, 0, 0],
    frequency=10.0,
    resolution=0.1,
    max_range=100.0
)

# Get lidar data
point_cloud = lidar.get_point_cloud()
```

### Adding an IMU Sensor

```python
from omni.isaac.sensor import Imu

imu = Imu(
    prim_path="/World/robot/imu",
    translation=[0, 0, 0],
    rotation=[0, 0, 0]
)

# Get IMU readings
lin_acc, ang_vel = imu.get_linear_acceleration_and_angular_velocity()
```

## ROS 2 Integration

### Connecting to ROS 2

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class IsaacSimROS2Bridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        self.lidar_publisher = self.create_publisher(
            PointCloud2, 'lidar/points', 10
        )
        
        self.timer = self.create_timer(0.1, self.publish_lidar_data)
    
    def publish_lidar_data(self):
        point_cloud = lidar.get_point_cloud()
        # Convert to ROS 2 PointCloud2 message
        msg = self.convert_to_pointcloud2(point_cloud)
        self.lidar_publisher.publish(msg)
```

### Running Simulation with ROS 2

```bash
# Terminal 1: Start Isaac Sim
cd isaac_sim
./isaac-sim.sh

# Terminal 2: Start ROS 2 nodes
ros2 launch isaac_sim_bridge bridge.launch.py
```

## Rendering and Visualization

### Real-time Rendering Options

```python
# Enable ray tracing
from omni.isaac.core.prims.xform_prim import XformPrim

# Adjust rendering quality
app.set_config({
    "rendering": {
        "enable_path_tracing": True,
        "samples_per_pixel": 256,
    }
})
```

### Taking Screenshots

```python
from omni.kit.capture import capture

# Capture screenshot
capture.take_screenshot(
    filepath="/tmp/screenshot.png",
    camera=camera
)
```

## Troubleshooting

| Issue | Solution |
|---|---|
| **Isaac Sim won't start** | Check NVIDIA driver version, ensure CUDA is installed |
| **Low simulation performance** | Reduce physics resolution, disable ray tracing |
| **Missing URDF support** | Import via USD bridge, check path references |
| **ROS 2 connection issues** | Verify ROS_DOMAIN_ID, check network connectivity |

## Next Steps

Learn about [Digital Twins](digital-twins) to create accurate virtual copies of real robots.
