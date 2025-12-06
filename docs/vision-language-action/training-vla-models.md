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
