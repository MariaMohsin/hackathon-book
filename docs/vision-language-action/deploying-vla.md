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
