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
