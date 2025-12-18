# Physical AI & Humanoid Robotics Book


# API Documentation

## Base URL
`http://localhost:8000/api` (or your deployed URL)

## Endpoints

### Chat Endpoint
`POST /api/chat`

#### Request
```json
{
  "question": "string",
  "selected_text": "string or null",
  "session_id": "string or null"
}
```

#### Response
```json
{
  "answer": "string",
  "mode": "rag | selection-only",
  "citations": [
    {
      "source_id": "string",
      "excerpt": "string",
      "page_reference": "number or null",
      "section": "string or null",
      "score": "number or null"
    }
  ],
  "session_id": "string",
  "timestamp": "ISO date string"
}
```

#### Example
```bash
curl -X POST "http://localhost:8000/api/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main concept discussed in chapter 3?",
    "selected_text": null,
    "session_id": "session-123"
  }'
```

### Index Endpoint
`POST /api/index`

#### Request
```json
{
  "book_content": "string",
  "book_metadata": {
    "title": "string",
    "author": "string",
    "section": "string",
    "page_reference": "number",
    "chapter": "string"
  }
}
```

#### Response
```json
{
  "success": "boolean",
  "chunks_indexed": "number",
  "message": "string"
}
```

#### Example
```bash
curl -X POST "http://localhost:8000/api/index" \
  -H "Content-Type: application/json" \
  -d '{
    "book_content": "# Chapter 1\nThe main topic is... # Chapter 2\n...",
    "book_metadata": {
      "title": "My Technical Book",
      "author": "Author Name"
    }
  }'
```

### Health Check Endpoints

#### Basic Health
`GET /api/health`

#### Extended Health
`GET /api/health/extended`

#### Readiness
`GET /api/health/readiness`

## Authentication
The API currently does not require authentication but implements rate limiting to prevent abuse.

## Rate Limits
- 50 requests per hour per IP address
- Requests exceeding the limit will receive a 429 status code

---
title: Integrating Robots in Simulation
sidebar_position: 2
---

# Integrating Robots in Simulation

## Overview

Integrating robot models into your simulation environment is crucial for developing and testing robotics applications. This guide covers how to work with robot models in both Gazebo and Unity.

## Robot Description Format

### URDF (Unified Robot Description Format)

URDF is an XML format for describing a robot's kinematics and dynamics:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.2"/>
      </geometry>
    </visual>
  </link>
  
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0" effort="100" velocity="1"/>
  </joint>
</robot>
```

### SDRF (Semantic Robot Description Format)

SRDF adds semantic information to URDF:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <group name="base">
    <joint name="wheel_joint"/>
  </group>
</robot>
```

## Setting Up a Robot in Gazebo

### Step 1: Create a Package
```bash
ros2 pkg create my_robot_description
```

### Step 2: Add Robot Files
```
my_robot_description/
├── urdf/
│   └── my_robot.urdf.xacro
├── meshes/
│   ├── visual/
│   └── collision/
└── launch/
    └── gazebo.launch.py
```

### Step 3: Create Launch File
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-topic', 'robot_description'],
            output='screen'
        )
    ])
```

## Setting Up a Robot in Unity

### Step 1: Import Robot Model
1. Export URDF to Unity-compatible format
2. Import FBX or OBJ meshes
3. Create prefab components

### Step 2: Add Physics
- Add Rigidbody components
- Create colliders for each link
- Configure constraints for joints

### Step 3: Add ROS 2 Publishers/Subscribers
```csharp
using ROS2;
using geometry_msgs.msg;

public class RobotController : MonoBehaviour
{
    private ROS2UnityComponent ros2;
    private IPublisher<Twist> cmdVelPublisher;
    
    void Start()
    {
        ros2 = GetComponent<ROS2UnityComponent>();
        cmdVelPublisher = ros2.CreatePublisher<Twist>("cmd_vel");
    }
}
```

## Sensor Integration

### Adding a Camera Sensor

**Gazebo:**
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
</sensor>
```

**Unity:**
```csharp
var camera = GetComponent<Camera>();
// Configure ROS 2 publisher for images
```

### Adding a Lidar Sensor

**Gazebo:**
```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
    </range>
  </ray>
</sensor>
```

## Common Issues and Solutions

### Issue: Model Not Loading
- Check file paths are correct
- Verify URDF syntax is valid
- Ensure all mesh files exist

### Issue: Physics Not Working
- Verify inertial properties are set
- Check collision geometry is defined
- Adjust physics timestep if needed

### Issue: ROS 2 Communication Not Working
- Verify ROS_DOMAIN_ID matches
- Check topic names match
- Monitor message flow with ros2 topic echo

## Best Practices

- Keep robot models modular and reusable
- Use Xacro macros to reduce URDF duplication
- Test sensors individually before full integration
- Document your model structure
- Version control all robot description files

## Next Steps

Explore [ROS 2 documentation](../ros2/understanding-ros2) to better understand how to control your integrated robots.

---
title: Simulation Basics
sidebar_position: 1
---

# Simulation Basics

## What is Robotic Simulation?

Robotic simulation is the practice of creating a virtual representation of a robot and its environment. It allows you to test and develop robotics software without requiring physical hardware.

## Why Use Simulation?

- **Cost-effective**: No expensive hardware damage
- **Safe**: Test dangerous scenarios without risk
- **Fast iteration**: Quickly prototype and test ideas
- **Reproducibility**: Identical scenarios every time
- **Scale testing**: Test multiple scenarios in parallel

## Popular Simulation Environments

### Gazebo

Gazebo is one of the most popular open-source robot simulators. It provides:

- **Physics simulation**: Accurate rigid body dynamics
- **Sensor simulation**: Simulate cameras, lidar, and other sensors
- **Multi-robot simulation**: Support for multiple robots in one environment
- **Plugins**: Extensible through plugins
- **ROS 2 integration**: Native ROS 2 support

### Unity

Unity is a game engine that can be used for robotics simulation with excellent:

- **Graphics quality**: High-fidelity visualization
- **Real-time performance**: Optimized rendering
- **Cross-platform**: Deploy on various platforms
- **Simulation realism**: Can simulate complex environments

## Key Concepts

### World Files
Worlds define the simulation environment, including ground planes, obstacles, and lighting.

### Models
Models represent robots and objects in the simulation. They include:
- Visual geometry (how they look)
- Collision geometry (for physics)
- Inertial properties (mass, center of gravity)
- Sensors and actuators

### Physics Engine
The physics engine simulates realistic motion and collisions.

### Sensors
Sensors in simulation provide virtual measurements:
- Camera images
- Lidar point clouds
- IMU data
- Odometry

## Getting Started with Gazebo

### Installation
```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### Running a Basic Simulation
```bash
gazebo
```

### Loading a World
```bash
gazebo empty.world
```

## Getting Started with Unity

### Installation
1. Download Unity Hub
2. Install Unity Editor (2021 LTS or newer)
3. Install Unity Robotics packages

### Creating a New Robotics Scene
1. Create a new 3D scene
2. Add simulation components
3. Configure ROS 2 connection

## Best Practices

- Start with simple environments and gradually add complexity
- Validate simulation results against reality
- Use appropriate physics timesteps for accuracy
- Document your simulation setup
- Version control your simulation files

## Common Simulation Scenarios

### Pick and Place
Simulate a robot arm picking up objects and placing them elsewhere.

### Navigation
Test autonomous navigation algorithms in simulated environments.

### Manipulation
Practice complex manipulation tasks in a safe environment.

## Next Steps

Learn about [Integrating Robots in Simulation](integrating-robots) to start working with robot models.

# Welcome to the Physical AI & Humanoid Robotics Book

This is an introduction to the book.

---
title: Digital Twins
sidebar_position: 3
---

# Digital Twins

## What is a Digital Twin?

A digital twin is a virtual replica of a physical robot that mirrors its structure, behavior, and capabilities. It enables you to:

- **Simulate** robot behavior before deployment
- **Train** AI models safely
- **Troubleshoot** issues without hardware
- **Optimize** designs and algorithms

## Creating a Digital Twin

### Step 1: Model the Physical Robot

Export your robot design in URDF or CAD format:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="aluminum">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.2"/>
    </inertial>
  </link>
  
  <!-- Wheel links -->
  <link name="wheel_fl">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### Step 2: Import into Isaac Sim

```python
from omni.isaac.core import World
from omni.isaac.core.robots import Robot

world = World()

# Load robot from URDF
robot = Robot(
    prim_path="/World/my_robot",
    usd_path="path_to_robot.usd"  # Convert URDF to USD first
)

# Add to world
world.scene.add(robot)
```

### Step 3: Configure Sensors

Add all sensors from the physical robot:

```python
from omni.isaac.sensor import Lidar, Imu

# Match physical robot's sensor configuration
lidar = Lidar(
    prim_path="/World/my_robot/lidar",
    translation=[0, 0, 0.3],
    frequency=10.0,
    max_range=25.0
)

imu = Imu(
    prim_path="/World/my_robot/imu",
    translation=[0, 0, 0.15]
)
```

### Step 4: Simulate Control Commands

```python
class DigitalTwinController:
    def __init__(self, robot):
        self.robot = robot
        self.joint_positions = {}
    
    def set_joint_target(self, joint_name, position):
        """Set target position for a joint"""
        joint = self.robot.get_joint(joint_name)
        joint.set_target_position(position)
    
    def set_wheel_velocity(self, left_vel, right_vel):
        """Set wheel velocities (for differential drive)"""
        self.robot.get_joint("wheel_left").set_target_velocity(left_vel)
        self.robot.get_joint("wheel_right").set_target_velocity(right_vel)
    
    def get_joint_state(self, joint_name):
        """Get current joint state"""
        joint = self.robot.get_joint(joint_name)
        return {
            'position': joint.get_position(),
            'velocity': joint.get_velocity(),
            'effort': joint.get_effort()
        }
```

## Physics Tuning for Realism

### Matching Real Robot Behavior

```python
# 1. Measure physical robot properties
# - Mass, moment of inertia
# - Friction coefficients
# - Motor torque/speed limits

# 2. Configure in simulation
physics_context = PhysicsContext()

# Set material properties to match real materials
rubber_material = {
    'static_friction': 0.7,
    'dynamic_friction': 0.5,
    'restitution': 0.1
}

# Apply to wheels
wheel = world.scene.get_object("wheel_fl")
wheel.get_prim().GetAttribute("physxMaterial:staticFriction").Set(0.7)
```

### Friction and Traction

```python
def calibrate_friction(digital_twin, physical_robot):
    """Adjust simulation friction to match real robot"""
    
    # Test: push robot and measure deceleration
    velocities = []
    
    for t in range(100):
        digital_twin.step(render=False)
        vel = digital_twin.get_linear_velocity()
        velocities.append(vel)
    
    # Calculate deceleration in simulation
    sim_decel = (velocities[0] - velocities[-1]) / len(velocities)
    
    # Compare with physical robot deceleration
    # Adjust friction until they match
```

## Synthetic Data Generation

### Training Data from Simulation

```python
import cv2
import numpy as np
from pathlib import Path

class SyntheticDataGenerator:
    def __init__(self, world, output_dir):
        self.world = world
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
    
    def generate_dataset(self, num_samples=1000):
        """Generate labeled dataset for training"""
        
        images = []
        labels = []
        
        for i in range(num_samples):
            # Randomize scene
            self.randomize_objects()
            self.randomize_lighting()
            
            # Capture image
            self.world.step(render=True)
            image = self.world.camera.get_rgb()
            
            # Generate labels
            detections = self.detect_objects_in_image(image)
            
            # Save
            cv2.imwrite(f"{self.output_dir}/image_{i:06d}.png", image)
            self.save_labels(f"{self.output_dir}/label_{i:06d}.json", detections)
            
            images.append(image)
            labels.append(detections)
        
        return images, labels
    
    def randomize_objects(self):
        """Random object placement and orientation"""
        import random
        
        for obj in self.world.scene.get_all_objects():
            obj.set_position([
                random.uniform(-1, 1),
                random.uniform(-1, 1),
                random.uniform(0.5, 2)
            ])
    
    def randomize_lighting(self):
        """Vary lighting conditions"""
        import random
        
        light = self.world.scene.get_light("main_light")
        light.set_intensity(random.uniform(0.5, 2.0))
```

## Validation: Sim-to-Real Transfer

### Testing Algorithms in Simulation First

```python
class SimToRealValidator:
    def __init__(self, sim_world, real_robot):
        self.sim_world = sim_world
        self.real_robot = real_robot
    
    def test_algorithm(self, algorithm, test_scenarios):
        """Test algorithm in both simulation and reality"""
        
        sim_results = []
        real_results = []
        
        for scenario in test_scenarios:
            # Test in simulation
            self.sim_world.reset_to_scenario(scenario)
            sim_result = algorithm.run(self.sim_world)
            sim_results.append(sim_result)
            
            # Test on real robot
            self.real_robot.reset_to_scenario(scenario)
            real_result = algorithm.run(self.real_robot)
            real_results.append(real_result)
        
        # Compare results
        correlation = self.compute_correlation(sim_results, real_results)
        return correlation
    
    def compute_correlation(self, sim_results, real_results):
        """Calculate how well simulation matches reality"""
        import numpy as np
        return np.corrcoef(sim_results, real_results)[0, 1]
```

## Common Challenges

### Challenge 1: Physics Don't Match
- **Problem**: Simulation behaves differently from real robot
- **Solution**: Carefully tune mass, friction, and actuator limits

### Challenge 2: Sensor Simulation Inaccurate
- **Problem**: Simulated sensor readings don't match real sensors
- **Solution**: Add noise and latency to match real sensor characteristics

```python
def add_realistic_noise(sensor_reading):
    """Add noise to match real sensor"""
    import numpy as np
    
    # Add Gaussian noise
    noise = np.random.normal(0, sensor_noise_std)
    
    # Add latency (simulate delayed reading)
    delay = np.random.choice([0, 1, 2])  # 0-2 frames delayed
    
    return sensor_reading + noise - delay
```

### Challenge 3: Computational Cost
- **Problem**: Simulation too slow for real-time testing
- **Solution**: Use GPU acceleration, reduce simulation resolution

## Best Practices

1. **Start Simple**: Build minimal digital twin first
2. **Validate Incrementally**: Compare simulation to real robot step-by-step
3. **Use Domain Randomization**: Vary parameters to improve real-world robustness
4. **Monitor Discrepancies**: Track differences between sim and reality
5. **Iterative Refinement**: Continuously improve digital twin accuracy

## Next Steps

You can now use digital twins to train AI algorithms and create advanced robotics applications.


---
title: Isaac Overview
sidebar_position: 1
---

# NVIDIA Isaac Overview

## What is NVIDIA Isaac?

NVIDIA Isaac is a comprehensive robotics platform and simulation environment powered by NVIDIA's advanced AI and GPU acceleration. It enables developers to create, simulate, and deploy intelligent robots at scale.

## Isaac Components

### Isaac Sim
Advanced physics-based simulation with:
- **Accurate Physics**: Powered by PhysX engine
- **Photorealistic Rendering**: Real-time ray tracing
- **AI-Powered Scene Creation**: Synthetic data generation
- **ROS 2 Integration**: Native support for ROS 2 workflows

### Isaac Manipulator
Pre-configured packages for robotic arms:
- Trajectory planning
- Collision avoidance
- Grasp planning
- Teleoperation

### Isaac Perception
Computer vision and sensor simulation:
- Synthetic data for training
- Camera simulation
- Lidar point cloud generation
- Depth sensor emulation

### Isaac Learn
ML training framework:
- TensorFlow and PyTorch support
- Transfer learning
- Reinforcement learning
- Training on simulated data

## Why Isaac for Robotics?

### Advantages
1. **Accelerated Simulation**: GPU-powered physics runs 10x-100x faster than CPU-based simulators
2. **Synthetic Data**: Generate unlimited training data safely
3. **Digital Twins**: Create accurate virtual copies of real robots
4. **Cloud Deployment**: Deploy simulations and training on NVIDIA cloud
5. **AI Integration**: Seamless integration with NVIDIA's AI frameworks

### Use Cases
- **Development**: Develop algorithms faster
- **Testing**: Validate code before hardware deployment
- **Training**: Train ML models with synthetic data
- **Deployment**: Run inference on edge devices with NVIDIA Jetson

## Isaac Sim vs Gazebo

| Feature | Isaac Sim | Gazebo |
|---|---|---|
| **Physics Engine** | PhysX (GPU) | ODE, Bullet (CPU) |
| **Rendering** | Real-time ray tracing | OpenGL |
| **Speed** | Very fast (GPU) | Moderate (CPU) |
| **Ease of Use** | Visual interface | Configuration files |
| **Learning Curve** | Moderate | Steep |
| **Cost** | Commercial (free tier) | Open source |
| **AI Integration** | Excellent | Good (via external tools) |

## System Requirements

### Minimum Specifications
- **GPU**: NVIDIA RTX or RTX Ada (20GB+ VRAM recommended)
- **CPU**: 8-core modern processor
- **RAM**: 32GB minimum
- **Storage**: 100GB SSD
- **OS**: Linux (Ubuntu 22.04) or Windows

### Recommended Specifications
- **GPU**: NVIDIA RTX 4090 or RTX Ada
- **CPU**: 16+ core processor
- **RAM**: 64GB+
- **Storage**: NVMe SSD with 500GB+

## Next Steps

Learn about [Setting Up Isaac Sim](isaac-sim-setup) to start building intelligent robot simulations.


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


---
title: Practical Exercises
sidebar_position: 3
---

# Practical Exercises

## Exercise 1: Create Your First Node

### Objective
Create a simple ROS 2 node that publishes messages every second.

### Steps
1. Create a new ROS 2 package
2. Create a Python node that publishes to a topic
3. Create a subscriber node to receive the messages
4. Run both nodes and observe the communication

### Solution Outline
```python
# publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello {self.count}'
        self.publisher_.publish(msg)
        self.count += 1
```

## Exercise 2: Service Communication

### Objective
Implement a simple service that adds two numbers.

### Steps
1. Define a service interface
2. Create a service server
3. Create a service client
4. Test the communication

## Exercise 3: Parameter Server

### Objective
Learn to use ROS 2 parameters for configuration.

### Steps
1. Create a node that declares parameters
2. Access parameters during node operation
3. Modify parameters dynamically
4. Observe parameter changes in real-time

## Exercise 4: Multi-Node System

### Objective
Create a system with multiple nodes communicating through topics and services.

### Steps
1. Create a sensor node that publishes data
2. Create a processing node that subscribes and processes
3. Create a control node that sends commands
4. Integrate everything into a coordinated system

## Tips for Success

- Start with simple examples and gradually increase complexity
- Use `rqt` tools to visualize the node graph
- Monitor messages with `ros2 topic echo`
- Debug using `ros2 node list` and `ros2 topic list`

## Common Issues and Solutions

### Issue: Node not connecting
- Check node names and topic names for typos
- Ensure all nodes are using the same ROS domain ID
- Verify network connectivity between nodes

### Issue: Messages not received
- Check subscription QoS settings
- Verify message types match
- Use `ros2 topic pub` to manually test topics

## Next Steps

Apply these exercises in the [Simulating with Gazebo & Unity](../gazebo-unity/simulation-basics) section.: ROS 2

Apply your ROS 2 knowledge with hands-on coding challenges.

---
title: Understanding ROS 2
sidebar_position: 1
---

# Understanding ROS 2

## Overview

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools and libraries that help you build robot applications across a wide variety of robotic platforms.

## Key Concepts

### Nodes
Nodes are processes that perform computation. ROS 2 graphs are composed of multiple nodes working in concert.

### Topics
Topics are named buses over which nodes exchange messages. Topics are one of the main ways that data moves around a ROS 2 system.

### Services
Services allow nodes to send a request and receive a response.

### Actions
Actions are a form of asynchronous RPC that allow a node to invoke an action on another node and receive feedback and a result.

## Why ROS 2?

- **Distributed**: Runs on multiple computers and devices
- **Language agnostic**: Supports Python, C++, and other languages
- **Middleware agnostic**: Works with different communication middleware
- **Real-time capable**: Can be used in real-time systems
- **Security**: Built-in security features

## Getting Started

To begin working with ROS 2, you'll need to install it on your system and understand the basic concepts of nodes, topics, and services.

## Next Steps

Learn about [Working with Nodes](working-with-nodes) to understand how to create and manage ROS 2 nodes in your robotics applications.

---
title: Working with Nodes
sidebar_position: 2
---

# Working with Nodes

## What is a Node?

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes form the core building blocks of any ROS 2 application.

## Creating a Node

### Python Example

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node created!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Example

```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_node");
  RCLCPP_INFO(node->get_logger(), "Node created!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## Node Lifecycle

1. **Creation**: Instantiate the node object
2. **Initialization**: Set up publishers, subscribers, and services
3. **Spin**: Enter the main loop processing callbacks
4. **Shutdown**: Clean up resources

## Communication Patterns

### Publishers and Subscribers

Nodes can communicate through topics using publishers and subscribers.

### Service Servers and Clients

For request-response patterns, use services.

### Actions

For long-running tasks with feedback, use actions.

## Best Practices

- Keep nodes focused on a single responsibility
- Use meaningful node names
- Log important events and errors
- Handle shutdown gracefully

## Next Steps

Learn about [Practical Exercises](practical-exercises) to apply your node knowledge.

Learn how to create, manage, and communicate with ROS 2 nodes.

# RAG Chatbot for Technical Book - User Guide

## Overview
This RAG (Retrieval-Augmented Generation) chatbot allows you to interact with technical book content using natural language queries. The system retrieves relevant information from the book and generates grounded responses with citations.

## Getting Started

### Prerequisites
- Python 3.8+
- API keys for:
  - Cohere (for embeddings and text generation)
  - Qdrant Cloud (for vector storage)
  - Neon Postgres (for metadata storage)

### Setup
1. Clone the repository
2. Create a virtual environment: `python -m venv venv`
3. Activate it: `source venv/bin/activate` (Linux/Mac) or `venv\Scripts\activate` (Windows)
4. Install dependencies: `pip install -r requirements.txt`
5. Create a `.env` file with your API keys based on `.env.example`
6. Start the server: `uvicorn app.main:app --reload --port 8000`

## Usage

### Chat with the RAG Model
Send a `POST` request to `/api/chat` with your question:

```json
{
  "question": "What is the main concept in chapter 3?",
  "selected_text": null,
  "session_id": "session-123"
}
```

### Use Selection-Only Mode
If you want to ask about specific text, provide it in the `selected_text` field:

```json
{
  "question": "Explain this concept",
  "selected_text": "The concept of retrieval-augmented generation is...",
  "session_id": "session-123"
}
```

### Index Your Book Content
To add your book content to the RAG system, send a `POST` request to `/api/index`:

```json
{
  "book_content": "# Chapter 1\nThe main topic is...",
  "book_metadata": {
    "title": "My Technical Book",
    "author": "Author Name"
  }
}
```

## Features

### RAG Mode
- When `selected_text` is `null`, the system searches the indexed book content
- Returns answers grounded in the book with citations
- Shows source information for fact-checking

### Selection-Only Mode
- When `selected_text` contains text, the system only uses that text
- Provides focused answers to questions about specific passages
- Does not perform retrieval from the larger book content

### Citations
- All answers include source citations
- Citations contain excerpts, page references, and section information
- Sources are traceable and auditable

## Best Practices

### For Better Results
- Ask specific, clear questions
- Use the selection-only mode for detailed analysis of specific passages
- Provide context in your questions (e.g., "In chapter 3, what does the author mean by...")

### Performance Tips
- The system processes books in chunks to maintain context
- Very long questions might be trimmed for optimal performance
- Consider breaking complex questions into simpler components

## Troubleshooting

### Common Issues
- **"Answer not found in book"**: The requested information might not be in the indexed content
- **Rate limit exceeded**: You've exceeded the request limit per hour
- **API connection errors**: Check your API keys and network connection

### Error Messages
- The system provides clear error messages when something goes wrong
- Check the logs for more detailed information about errors

## API Limits
- Rate limiting: 50 requests per hour per IP
- If you need higher limits, contact your system administrator

## Security
- All API keys should be stored in environment variables
- The system does not store your questions permanently unless configured to do so
- All external API calls are secured with HTTPS

---
title: Deploying VLA on Robots
sidebar_position: 3
---

# Deploying Vision-Language-Action Models on Robots

## Deployment Pipeline

```
Trained Model → Optimization → Edge Device → Real Robot
     ↓              ↓               ↓           ↓
  PyTorch    Quantization    Jetson/CPU   Execute Actions
```

## Model Optimization

### 1. Convert to ONNX

```python
import torch
import torch.onnx

def export_to_onnx(model, example_image, example_text):
    """Export PyTorch model to ONNX format"""
    
    torch.onnx.export(
        model,
        (example_image, example_text),
        "vla_model.onnx",
        opset_version=11,
        do_constant_folding=True,
        input_names=['image', 'text'],
        output_names=['actions'],
        dynamic_axes={
            'image': {0: 'batch_size'},
            'text': {0: 'batch_size'},
            'actions': {0: 'batch_size'}
        }
    )
    
    return "vla_model.onnx"
```

### 2. Quantization

```python
import onnx
import onnxruntime as rt
from onnxruntime.quantization import quantize_dynamic

def quantize_onnx_model(onnx_path):
    """Quantize ONNX model for faster inference"""
    
    quantized_path = "vla_model_quantized.onnx"
    
    quantize_dynamic(
        onnx_path,
        quantized_path,
        weight_type=QuantType.QUInt8
    )
    
    # Verify quantized model
    model = onnx.load(quantized_path)
    onnx.checker.check_model(model)
    
    return quantized_path
```

### 3. Model Size Comparison

```python
import os

def compare_model_sizes(fp32_path, quantized_path):
    """Compare model sizes"""
    
    fp32_size = os.path.getsize(fp32_path) / (1024**2)  # MB
    quantized_size = os.path.getsize(quantized_path) / (1024**2)
    
    print(f"FP32 Model: {fp32_size:.1f} MB")
    print(f"Quantized Model: {quantized_size:.1f} MB")
    print(f"Reduction: {(1 - quantized_size/fp32_size)*100:.1f}%")
    
    return {
        'original': fp32_size,
        'quantized': quantized_size,
        'reduction_percent': (1 - quantized_size/fp32_size)*100
    }
```

## Edge Device Setup

### NVIDIA Jetson Setup

```bash
# Install JetPack
sudo apt-get update
sudo apt-get install nvidia-jetpack

# Verify GPU
nvidia-smi

# Install TensorRT for optimized inference
sudo apt-get install python3-tensorrt

# Install ROS 2
sudo apt-get install ros-humble-ros-core
```

### Running Inference on Jetson

```python
import tensorrt as trt
import numpy as np

class JetsonVLAInference:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.INFO)
        
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        
        self.context = self.engine.create_execution_context()
    
    def infer(self, image, instruction):
        """Run inference on Jetson"""
        
        # Prepare input
        image_tensor = np.ascontiguousarray(image)
        instruction_tensor = np.ascontiguousarray(instruction)
        
        # Allocate output
        output = np.empty(self.get_output_shape(), dtype=np.float32)
        
        # Execute
        self.context.execute_v2([
            image_tensor.data_ptr(),
            instruction_tensor.data_ptr(),
            output.data_ptr()
        ])
        
        return output
    
    def get_output_shape(self):
        return (1, 5, 7)  # [batch, num_waypoints, action_dim]
```

## Real-Time Inference

### ROS 2 Node for VLA Inference

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np

class VLAInferenceNode(Node):
    def __init__(self, model_path):
        super().__init__('vla_inference')
        
        # Load model
        self.model = self.load_model(model_path)
        self.bridge = CvBridge()
        
        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.instruction_sub = self.create_subscription(
            String, 'vla/instruction', self.instruction_callback, 10
        )
        self.action_pub = self.create_publisher(
            Float32MultiArray, 'vla/actions', 10
        )
        
        self.current_image = None
        self.current_instruction = None
        
        # Timer for inference
        self.timer = self.create_timer(0.1, self.run_inference)
    
    def load_model(self, model_path):
        """Load VLA model with your framework"""
        import onnxruntime as rt
        session = rt.InferenceSession(model_path)
        return session
    
    def image_callback(self, msg):
        """Receive camera image"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def instruction_callback(self, msg):
        """Receive natural language instruction"""
        self.current_instruction = msg.data
        self.get_logger().info(f"Instruction: {self.current_instruction}")
    
    def run_inference(self):
        """Run VLA inference when both image and instruction available"""
        
        if self.current_image is None or self.current_instruction is None:
            return
        
        try:
            # Preprocess image
            input_image = self.preprocess_image(self.current_image)
            
            # Run inference
            start_time = self.get_clock().now()
            
            output = self.model.run(
                None,
                {
                    'image': input_image,
                    'text': np.array([self.current_instruction])
                }
            )
            
            inference_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            
            # Extract action sequence
            actions = output[0]
            
            # Publish actions
            action_msg = Float32MultiArray()
            action_msg.data = actions.flatten().tolist()
            self.action_pub.publish(action_msg)
            
            self.get_logger().info(
                f"Inference time: {inference_time*1000:.1f}ms, "
                f"Actions: {actions.shape}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
    
    def preprocess_image(self, image):
        """Prepare image for model input"""
        
        # Resize to model input size
        resized = cv2.resize(image, (224, 224))
        
        # Normalize
        normalized = resized.astype(np.float32) / 255.0
        
        # Add batch dimension
        batched = np.expand_dims(normalized, axis=0)
        
        return batched

def main(args=None):
    rclpy.init(args=args)
    node = VLAInferenceNode("vla_model_optimized.onnx")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Action Execution

### Converting Model Output to Robot Commands

```python
class ActionExecutor:
    def __init__(self, robot):
        self.robot = robot
    
    def execute_vla_actions(self, action_sequence, instruction):
        """Execute sequence of actions from VLA model"""
        
        self.get_logger().info(f"Executing: {instruction}")
        
        for idx, waypoint_action in enumerate(action_sequence):
            # Waypoint is [joint1, joint2, ..., joint6, gripper]
            joint_angles = waypoint_action[:6]
            gripper_state = waypoint_action[6]
            
            self.execute_waypoint(joint_angles, gripper_state)
            
            # Check for early termination
            if self.check_success_condition(instruction):
                self.get_logger().info("Task completed successfully!")
                return True
            
            # Wait between waypoints
            time.sleep(0.5)
        
        return False
    
    def execute_waypoint(self, joint_angles, gripper_state):
        """Move to single waypoint"""
        
        # Send to ROS 2 controller
        trajectory_msg = FollowJointTrajectoryGoal()
        trajectory_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0.0] * len(joint_angles)
        point.time_from_start = Duration(sec=1, nanosec=0).to_msg()
        
        trajectory_msg.trajectory.points.append(point)
        
        # Send trajectory
        self.action_client.send_goal(trajectory_msg)
        
        # Control gripper
        if gripper_state > 0.5:
            self.robot.gripper.close()
        else:
            self.robot.gripper.open()
    
    def check_success_condition(self, instruction):
        """Check if task completed successfully"""
        
        # Example: for "pick up" tasks, check if object is griped
        if "pick" in instruction.lower():
            return self.robot.gripper.is_gripping()
        
        # Add more task-specific checks
        return False
```

## Safety and Error Handling

### Safety Constraints

```python
class SafeVLAExecutor:
    def __init__(self, robot, joint_limits, velocity_limits):
        self.robot = robot
        self.joint_limits = joint_limits
        self.velocity_limits = velocity_limits
    
    def validate_action(self, action):
        """Check action safety before execution"""
        
        joint_angles = action[:6]
        
        # Check joint limits
        for i, angle in enumerate(joint_angles):
            if angle < self.joint_limits[i]['min'] or angle > self.joint_limits[i]['max']:
                self.get_logger().warn(
                    f"Joint {i} angle {angle} exceeds limits. Clamping."
                )
                joint_angles[i] = np.clip(
                    angle,
                    self.joint_limits[i]['min'],
                    self.joint_limits[i]['max']
                )
        
        return joint_angles
    
    def execute_with_safety(self, action_sequence):
        """Execute with safety checks"""
        
        for action in action_sequence:
            # Validate action
            safe_action = self.validate_action(action)
            
            # Check for obstacles
            if self.check_collision_risk(safe_action):
                self.get_logger().error("Collision detected! Stopping.")
                self.robot.stop()
                return False
            
            # Execute
            self.robot.execute_action(safe_action)
    
    def check_collision_risk(self, action):
        """Use collision detection to verify safety"""
        
        # Move to action in simulation
        self.collision_simulator.predict_state(action)
        
        # Check for collisions
        collisions = self.collision_detector.get_collisions()
        
        return len(collisions) > 0
```

## Monitoring and Logging

```python
class VLAMonitor:
    def __init__(self):
        self.metrics = {
            'inference_time': [],
            'success_count': 0,
            'failure_count': 0,
            'instructions_executed': []
        }
    
    def log_execution(self, instruction, success, inference_time):
        """Log VLA execution"""
        
        self.metrics['inference_time'].append(inference_time)
        self.metrics['instructions_executed'].append(instruction)
        
        if success:
            self.metrics['success_count'] += 1
        else:
            self.metrics['failure_count'] += 1
        
        # Calculate statistics
        avg_inference = np.mean(self.metrics['inference_time'])
        success_rate = (self.metrics['success_count'] / 
                       (self.metrics['success_count'] + self.metrics['failure_count']))
        
        print(f"Success Rate: {success_rate*100:.1f}%")
        print(f"Avg Inference: {avg_inference*1000:.1f}ms")
```

## Next Steps

Explore real-world applications and advanced deployment scenarios in your specific robotics domain.


---
title: Training VLA Models
sidebar_position: 2
---

# Training Vision-Language-Action Models

## Data Collection

### Teleoperation
Collect robot demonstrations through human control:

```python
class TeleoperationDataCollector:
    def __init__(self, robot):
        self.robot = robot
        self.data = []
    
    def collect_episode(self, instruction):
        """Collect one demonstration episode"""
        
        episode = {
            'instruction': instruction,
            'frames': [],
            'actions': [],
            'rewards': []
        }
        
        print(f"Collecting data for: {instruction}")
        print("Use gamepad to teleoperate robot. Press START when done.")
        
        while not self.is_episode_done():
            # Capture frame
            frame = self.robot.camera.get_rgb()
            episode['frames'].append(frame)
            
            # Get human action
            action = self.gamepad_controller.get_action()
            episode['actions'].append(action)
            
            # Execute action on robot
            self.robot.execute_action(action)
            
            # Get reward (success signal)
            reward = self.evaluate_success(instruction)
            episode['rewards'].append(reward)
        
        self.data.append(episode)
        return episode
    
    def save_dataset(self, path):
        """Save collected data to disk"""
        import json
        with open(path, 'w') as f:
            json.dump(self.data, f)
```

### Synthetic Data
Generate data from simulation:

```python
class SyntheticDataGenerator:
    def __init__(self, sim_world):
        self.sim_world = sim_world
        self.data = []
    
    def generate_trajectories(self, num_episodes=1000):
        """Generate random trajectories in simulation"""
        
        for episode_idx in range(num_episodes):
            # Randomize scene
            self.sim_world.randomize_objects()
            instruction = self._generate_instruction()
            
            # Generate trajectory
            trajectory = self._generate_trajectory_for_instruction(instruction)
            
            self.data.append({
                'instruction': instruction,
                'trajectory': trajectory,
                'sim_data': True
            })
        
        return self.data
    
    def _generate_trajectory_for_instruction(self, instruction):
        """Use rule-based policy to generate trajectory"""
        if "pick up" in instruction.lower():
            return self._pick_and_place_trajectory()
        elif "place" in instruction.lower():
            return self._placement_trajectory()
        # ... more rules
```

## Dataset Structure

```
dataset/
├── instructions/
│   ├── pick_cup.txt
│   ├── open_drawer.txt
│   └── ...
├── episodes/
│   ├── episode_0001/
│   │   ├── frames/
│   │   │   ├── frame_0000.jpg
│   │   │   ├── frame_0001.jpg
│   │   │   └── ...
│   │   ├── actions.json      # Joint angles, gripper state
│   │   ├── metadata.json      # Task info, success signal
│   │   └── instruction.txt    # Natural language instruction
│   ├── episode_0002/
│   └── ...
└── metadata.json              # Dataset statistics
```

## Model Architecture

### Vision Encoder

```python
import torch
import torch.nn as nn
from torchvision import models

class VisionEncoder(nn.Module):
    def __init__(self, output_dim=256):
        super().__init__()
        
        # Use pre-trained ResNet-50
        self.backbone = models.resnet50(pretrained=True)
        
        # Remove classification head
        self.backbone = nn.Sequential(*list(self.backbone.children())[:-1])
        
        # Add projection head
        self.projection = nn.Linear(2048, output_dim)
    
    def forward(self, x):
        features = self.backbone(x)
        features = features.squeeze(-1).squeeze(-1)
        embeddings = self.projection(features)
        return embeddings
```

### Language Encoder

```python
from transformers import AutoTokenizer, AutoModel

class LanguageEncoder(nn.Module):
    def __init__(self, model_name="bert-base-uncased", output_dim=256):
        super().__init__()
        
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModel.from_pretrained(model_name)
        
        # Projection to match vision embedding dimension
        self.projection = nn.Linear(768, output_dim)
    
    def forward(self, text):
        tokens = self.tokenizer(text, return_tensors='pt', padding=True)
        outputs = self.model(**tokens)
        
        # Use [CLS] token representation
        cls_embedding = outputs.last_hidden_state[:, 0, :]
        embeddings = self.projection(cls_embedding)
        return embeddings
```

### Multi-Modal Fusion

```python
class MultiModalFusion(nn.Module):
    def __init__(self, embedding_dim=256, num_heads=4):
        super().__init__()
        
        self.cross_attention = nn.MultiheadAttention(
            embedding_dim, num_heads, batch_first=True
        )
    
    def forward(self, visual_embeds, language_embeds):
        # Cross-attention between vision and language
        fused, _ = self.cross_attention(
            visual_embeds,
            language_embeds,
            language_embeds
        )
        return fused
```

### Action Decoder

```python
class ActionDecoder(nn.Module):
    def __init__(self, input_dim=256, action_dim=7, num_waypoints=5):
        super().__init__()
        
        self.action_dim = action_dim  # 6 joints + 1 gripper
        self.num_waypoints = num_waypoints
        
        self.layers = nn.Sequential(
            nn.Linear(input_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, num_waypoints * action_dim)
        )
    
    def forward(self, fused_embedding):
        action_sequence = self.layers(fused_embedding)
        
        # Reshape to [num_waypoints, action_dim]
        actions = action_sequence.reshape(-1, self.num_waypoints, self.action_dim)
        
        return actions
```

## Training

### Full VLA Model

```python
import torch.optim as optim

class VLATrainer:
    def __init__(self, model, learning_rate=1e-4):
        self.model = model
        self.optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()
    
    def train_epoch(self, train_loader):
        self.model.train()
        total_loss = 0
        
        for batch in train_loader:
            images, instructions, actions = batch
            
            # Forward pass
            predicted_actions = self.model(images, instructions)
            
            # Compute loss
            loss = self.criterion(predicted_actions, actions)
            
            # Backward pass
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
            
            total_loss += loss.item()
        
        return total_loss / len(train_loader)
    
    def train(self, train_loader, val_loader, epochs=100):
        best_val_loss = float('inf')
        
        for epoch in range(epochs):
            train_loss = self.train_epoch(train_loader)
            val_loss = self.validate(val_loader)
            
            print(f"Epoch {epoch}: Train Loss={train_loss:.4f}, Val Loss={val_loss:.4f}")
            
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                self.save_checkpoint(f"best_model.pth")
    
    def validate(self, val_loader):
        self.model.eval()
        total_loss = 0
        
        with torch.no_grad():
            for batch in val_loader:
                images, instructions, actions = batch
                predicted_actions = self.model(images, instructions)
                loss = self.criterion(predicted_actions, actions)
                total_loss += loss.item()
        
        return total_loss / len(val_loader)
```

## Evaluating VLA Models

### Success Metrics

```python
class VLAEvaluator:
    def __init__(self, robot):
        self.robot = robot
    
    def evaluate_success(self, instruction, predicted_actions, max_steps=100):
        """Execute predicted actions and check if task succeeded"""
        
        success = False
        steps = 0
        
        for action in predicted_actions:
            self.robot.execute_action(action)
            steps += 1
            
            # Check success condition
            success = self._check_success_condition(instruction)
            if success:
                break
            
            if steps > max_steps:
                break
        
        return {
            'success': success,
            'steps': steps,
            'efficiency': steps / max_steps
        }
    
    def evaluate_dataset(self, test_episodes):
        """Evaluate on entire test set"""
        
        results = []
        success_rate = 0
        
        for instruction, actions in test_episodes:
            result = self.evaluate_success(instruction, actions)
            results.append(result)
            success_rate += result['success']
        
        success_rate /= len(test_episodes)
        
        return {
            'success_rate': success_rate,
            'results': results
        }
```

## Optimization and Deployment

### Model Quantization

```python
import torch.quantization

def quantize_model(model):
    """Convert to lower precision for faster inference"""
    
    # Prepare model
    model.qconfig = torch.quantization.get_default_qat_qconfig('fbgemm')
    torch.quantization.prepare_qat(model, inplace=True)
    
    # Convert to quantized model
    torch.quantization.convert(model, inplace=True)
    
    return model
```

### Knowledge Distillation

```python
def distill_to_smaller_model(teacher_model, teacher_output):
    """Train smaller student model to mimic teacher"""
    
    student_model = VLAModel(smaller=True)
    teacher_model.eval()
    
    for images, instructions in train_loader:
        # Get teacher predictions
        with torch.no_grad():
            teacher_actions = teacher_model(images, instructions)
        
        # Train student to match teacher
        student_actions = student_model(images, instructions)
        loss = nn.KLDivLoss()(student_actions, teacher_actions)
        loss.backward()
```

## Next Steps

Learn about [Deploying VLA on Robots](deploying-vla) to run your trained models on real hardware.


---
title: VLA Fundamentals
sidebar_position: 1
---

# Vision-Language-Action (VLA) Fundamentals

## What is VLA?

Vision-Language-Action (VLA) is an AI paradigm that combines:
- **Vision**: Computer vision and image understanding
- **Language**: Natural language processing and LLMs
- **Action**: Robot control and manipulation

VLA systems can understand visual scenes, interpret natural language instructions, and execute corresponding robot actions.

## The VLA Pipeline

```
Visual Input (Camera) ──→ Vision Encoder ──→ Multi-Modal Fusion ──→ Action Decoder ──→ Robot Command
    ↓                          ↓                     ↓                    ↓
Text Input (Instruction) ──→ Language Encoder ──────────────────────────────────
```

## VLA Advantages

### 1. Intuitive Instruction Format
```
User: "Pick up the red cube and place it on the table"

VLA System understands:
- Visual: Where is the red cube?
- Language: What is the task?
- Action: What arm/gripper commands?
```

### 2. Few-Shot Learning
Train with few examples:

```python
# With traditional robotics: requires 1000s of examples
# With VLA: can learn from 10-50 demonstrations
demonstrations = [
    {
        'image': camera_frame,
        'instruction': "Pick up the block",
        'actions': [arm_position, gripper_command]
    },
    # ... more examples
]
```

### 3. Generalization
```python
# Trained on:
# - Red cube, wooden table
# - Blue sphere, metal table
# - Green block, plastic table

# Can generalize to:
# - Yellow cylinder, marble table (never seen before)
```

## VLA Model Architectures

### Vision Transformer (ViT)
- Divides image into patches
- Applies transformer attention
- Excellent for image understanding

### Language Model (LLM)
- GPT-style autoregressive models
- Claude, GPT-4, Llama
- Generates action sequences

### Multi-Modal Fusion
```python
class VLAModel:
    def __init__(self):
        self.vision_encoder = VisionTransformer()  # Image → embeddings
        self.language_encoder = LanguageModel()    # Text → embeddings
        self.fusion_layer = CrossAttention()       # Combine modalities
        self.action_decoder = ActionDecoder()      # Embeddings → robot actions
    
    def forward(self, image, instruction):
        # Encode visual and language inputs
        visual_features = self.vision_encoder(image)
        language_features = self.language_encoder(instruction)
        
        # Fuse multi-modal information
        fused = self.fusion_layer(visual_features, language_features)
        
        # Generate robot actions
        actions = self.action_decoder(fused)
        
        return actions
```

## VLA Action Output

### Joint-Space Actions
```python
# Direct joint commands
actions = {
    'shoulder_pan': 0.5,      # radians
    'shoulder_lift': 1.2,
    'elbow': -0.3,
    'wrist_1': 0.0,
    'wrist_2': 0.0,
    'wrist_3': 0.0,
    'gripper': 0.8            # 0=open, 1=closed
}
```

### Task-Space Actions
```python
# End-effector target
actions = {
    'position': [0.5, 0.0, 0.5],  # x, y, z in meters
    'orientation': [0, 0, 1, 0],  # quaternion
    'gripper': 0.8
}
```

### Waypoint Sequences
```python
# Multi-step action sequence
actions = [
    {'position': [0.5, 0.0, 0.3], 'gripper': 1.0},  # Approach
    {'position': [0.5, 0.0, 0.0], 'gripper': 1.0},  # Lower
    {'position': [0.5, 0.0, 0.0], 'gripper': 0.0},  # Release
    {'position': [0.5, 0.0, 0.3], 'gripper': 0.0}   # Retreat
]
```

## Popular VLA Models

### RT-1 (Robotics Transformer-1)
- Google's foundational VLA model
- Trained on 130,000+ robot episodes
- 50-60% success rate on unseen tasks

### RT-2 (Robotics Transformer-2)
- Improved with visual reasoning
- 50% improvement over RT-1
- Better generalization to new objects

### Open-Source Alternatives
- **MOSAIC**: Multi-task robotics model
- **Flamingo**: Vision-language model by DeepMind
- **CLIP**: OpenAI's vision-language matching

## VLA vs Traditional Robotics

| Aspect | Traditional | VLA |
|---|---|---|
| **Programming** | Code imperative steps | Describe in language |
| **Generalization** | Limited to trained scenarios | Generalizes to new scenarios |
| **Learning** | Manual feature engineering | End-to-end learning |
| **Flexibility** | Task-specific | Multi-task capable |
| **Development Time** | Weeks/months | Days/weeks |

## Current Limitations

### 1. Computational Requirements
```python
# VLA models are large
model_size = 1.2  # Billion parameters
inference_time = 0.5  # seconds per action
```

### 2. Real-Time Performance
- RT-1/RT-2: ~500ms per action
- Requires optimization for real-time control

### 3. Sim-to-Real Gap
- Training in simulation
- Deploying on real robots
- Domain randomization helps but not perfect

## Use Cases

### 1. Pick and Place
```
Instruction: "Pick the coffee cup and put it in the dishwasher"
VLA understands: object location, target location, gripper actions
```

### 2. Manipulation Tasks
```
Instruction: "Open the drawer and retrieve the keys"
VLA understands: sequential actions, gripper control
```

### 3. Assembly Tasks
```
Instruction: "Assemble the chair: attach leg A to frame, add screws"
VLA understands: multi-step assembly process
```

### 4. Mobile Manipulation
```
Instruction: "Navigate to the kitchen and place items on the counter"
VLA understands: navigation + manipulation
```

## Next Steps

Learn about [Training VLA Models](training-vla-models) to build your own vision-language-action system.

