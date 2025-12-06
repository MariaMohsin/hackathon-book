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