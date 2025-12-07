# Exercises: Vision-Language-Action (VLA) Integration for Humanoid Robotics

## Exercise 1: Voice Processing Implementation

### Objective
Implement a voice processing system using OpenAI Whisper that converts speech to text for humanoid robot commands.

### Requirements
1. Create a ROS 2 node for voice processing with OpenAI Whisper integration
2. Implement audio input handling from microphone or ROS topic
3. Process speech-to-text conversion with error handling
4. Publish transcribed text to ROS topic for further processing
5. Include confidence scoring for transcriptions

### Implementation Steps
1. Set up OpenAI Whisper API integration with proper authentication
2. Implement audio input handling using PyAudio
3. Create ROS 2 publisher for transcribed text
4. Add error handling for API failures and audio issues
5. Implement confidence scoring for transcription quality
6. Test with various audio inputs and noise conditions
7. Validate real-time performance requirements

### Validation
- Verify Whisper API integration works correctly
- Confirm audio input is properly captured and processed
- Validate transcription accuracy under different conditions
- Test error handling for API failures
- Measure real-time performance against requirements

## Exercise 2: Cognitive Planning System

### Objective
Create a cognitive planning system that converts natural language commands into multi-step action plans.

### Requirements
1. Parse natural language commands into structured tasks
2. Generate multi-step action plans with dependencies
3. Optimize plans for efficiency and safety
4. Handle plan execution monitoring and adaptation
5. Implement error recovery strategies

### Implementation Steps
1. Create natural language parser for command understanding
2. Implement task decomposition into primitive actions
3. Design plan optimization algorithms
4. Add dependency tracking between plan steps
5. Implement execution monitoring and adaptation
6. Create error recovery mechanisms
7. Test with various command types and complexities
8. Validate plan safety and feasibility

### Validation
- Verify command parsing accuracy for various inputs
- Confirm multi-step plans are executable
- Test plan optimization effectiveness
- Validate error recovery mechanisms
- Check execution monitoring functionality

## Exercise 3: Voice-to-Action Pipeline

### Objective
Develop a complete pipeline connecting voice processing to action execution.

### Requirements
1. Integrate voice processing with cognitive planning
2. Create intent extraction from transcribed commands
3. Map intents to appropriate action types
4. Handle context awareness for command interpretation
5. Implement feedback mechanisms for users

### Implementation Steps
1. Connect voice processing output to planning system
2. Implement intent classification algorithms
3. Create mapping between intents and actions
4. Add context awareness for command disambiguation
5. Implement user feedback mechanisms
6. Test pipeline with various command types
7. Validate end-to-end performance
8. Optimize for real-time operation

### Validation
- Verify end-to-end voice-to-action functionality
- Confirm intent classification accuracy
- Test context awareness improvements
- Validate real-time performance
- Check user feedback mechanisms

## Exercise 4: Object Recognition Integration

### Objective
Integrate computer vision object recognition with the VLA system.

### Requirements
1. Implement real-time object detection using computer vision
2. Estimate 3D positions of detected objects
3. Integrate with Isaac Sim synthetic sensors
4. Create object tracking for manipulation planning
5. Publish detection results for action planning

### Implementation Steps
1. Set up computer vision pipeline with camera input
2. Implement object detection using pre-trained models
3. Add 3D position estimation from camera parameters
4. Integrate with Isaac Sim sensor data
5. Create object tracking for manipulation
6. Publish detection results to ROS topics
7. Test with various objects and lighting conditions
8. Optimize for real-time performance

### Validation
- Verify object detection accuracy
- Confirm 3D position estimation precision
- Test integration with Isaac Sim
- Validate real-time performance
- Check robustness to lighting changes

## Exercise 5: Multi-Step Action Execution

### Objective
Implement a framework for executing complex multi-step actions with coordination.

### Requirements
1. Execute sequences of navigation, manipulation, and gesture actions
2. Handle dependencies between action steps
3. Implement error handling and recovery
4. Monitor execution progress and status
5. Provide feedback during execution

### Implementation Steps
1. Create action execution framework
2. Implement navigation action execution
3. Add manipulation action execution
4. Include gesture action execution
5. Implement dependency handling
6. Add error handling and recovery
7. Create progress monitoring
8. Add execution feedback mechanisms
9. Test with complex multi-step tasks

### Validation
- Verify execution of various action types
- Confirm dependency handling works correctly
- Test error recovery mechanisms
- Validate progress monitoring
- Check feedback mechanisms

## Exercise 6: LLM Integration for Task Planning

### Objective
Integrate Large Language Models for advanced task planning and natural language understanding.

### Requirements
1. Connect LLM for complex task decomposition
2. Implement context-aware command understanding
3. Generate detailed action plans from natural language
4. Handle multi-modal input (text + vision)
5. Provide explanations for plan decisions

### Implementation Steps
1. Set up LLM API integration (e.g., OpenAI GPT)
2. Create prompt engineering for task planning
3. Implement context management
4. Add multi-modal input handling
5. Create plan explanation mechanisms
6. Test with complex natural language commands
7. Validate plan quality and safety
8. Optimize for response time

### Validation
- Verify LLM integration works correctly
- Confirm complex task decomposition
- Test context-aware understanding
- Validate plan safety and feasibility
- Measure response time performance

## Exercise 7: Task Execution Logging

### Objective
Implement comprehensive logging for task execution monitoring and analysis.

### Requirements
1. Log all task execution events with timestamps
2. Record performance metrics and success rates
3. Capture error conditions and recovery attempts
4. Store robot state during execution
5. Provide analysis tools for log data

### Implementation Steps
1. Design logging data structure and format
2. Implement real-time logging for task events
3. Add performance metric tracking
4. Create error condition logging
5. Implement robot state capture
6. Add log rotation and management
7. Create analysis and visualization tools
8. Test logging with various execution scenarios

### Validation
- Verify all events are properly logged
- Confirm performance metrics accuracy
- Test error condition logging
- Validate log rotation functionality
- Check analysis tool effectiveness

## Exercise 8: Capstone Autonomous Demo

### Objective
Create a comprehensive demonstration integrating all VLA components.

### Requirements
1. Integrate all VLA components into a cohesive system
2. Demonstrate voice command processing and execution
3. Show object recognition and manipulation
4. Execute multi-step tasks with LLM assistance
5. Provide comprehensive feedback and monitoring

### Implementation Steps
1. Integrate all VLA components
2. Create demonstration scenarios
3. Implement demo orchestration
4. Add user interaction mechanisms
5. Create performance monitoring dashboard
6. Test complete system integration
7. Validate demonstration scenarios
8. Optimize overall system performance

### Validation
- Verify complete system integration
- Test all demonstration scenarios
- Confirm component interactions
- Validate performance metrics
- Check user experience quality

## Exercise 9: Isaac Sim Integration

### Objective
Integrate the complete VLA system with Isaac Sim for realistic simulation.

### Requirements
1. Connect VLA system to Isaac Sim synthetic sensors
2. Validate perception in simulated environments
3. Test navigation in Isaac Sim worlds
4. Verify manipulation in physics simulation
5. Evaluate system performance in simulation

### Implementation Steps
1. Configure Isaac Sim sensor outputs
2. Connect perception system to simulated sensors
3. Set up navigation in simulated environments
4. Implement manipulation in physics simulation
5. Create test scenarios in Isaac Sim
6. Validate perception accuracy
7. Test system robustness in simulation
8. Optimize for simulation-real transfer

### Validation
- Verify sensor integration works correctly
- Confirm perception accuracy in simulation
- Test navigation performance in various scenarios
- Validate manipulation in physics simulation
- Check simulation-real transfer capabilities

## Exercise 10: Performance Optimization

### Objective
Optimize the complete VLA system for real-time performance and efficiency.

### Requirements
1. Profile system performance bottlenecks
2. Optimize computational efficiency
3. Reduce latency in critical paths
4. Optimize memory usage and resource management
5. Validate performance improvements without degrading accuracy

### Implementation Steps
1. Profile current system performance
2. Identify computational bottlenecks
3. Apply optimization techniques (algorithmic, parallelization)
4. Optimize critical execution paths
5. Implement resource management
6. Test optimized system performance
7. Validate accuracy is maintained
8. Document performance improvements

### Validation
- Confirm performance improvements achieved
- Verify accuracy is not compromised
- Test system stability under load
- Validate real-time constraints are met
- Document optimization results

## Solutions

### Exercise 1 Solution

Voice Processing Node Implementation:
```python
#!/usr/bin/env python3
"""
Voice Processing Node for Humanoid Robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import pyaudio
import wave
import threading
import queue
import time
import openai
import os
from typing import Optional


class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        # Parameters
        self.declare_parameter('whisper_model', 'whisper-1')
        self.declare_parameter('audio_chunk_size', 1024)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('record_seconds', 5)

        self.whisper_model = self.get_parameter('whisper_model').value
        self.chunk_size = self.get_parameter('audio_chunk_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.record_seconds = self.get_parameter('record_seconds').value

        # OpenAI API key
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        if not self.openai_api_key:
            self.get_logger().warn('OPENAI_API_KEY environment variable not set')

        # Audio processing setup
        self.audio_queue = queue.Queue()
        self.recording = False
        self.audio = None
        self.stream = None

        # Publishers
        self.transcript_pub = self.create_publisher(String, '/voice/transcript', 10)
        self.command_pub = self.create_publisher(String, '/voice/command', 10)

        # Timer for audio recording
        self.recording_timer = self.create_timer(0.1, self.recording_callback)

        self.get_logger().info('Voice Processor initialized')

    def recording_callback(self):
        if not self.recording:
            self.start_recording()

    def start_recording(self):
        try:
            self.recording = True
            self.audio = pyaudio.PyAudio()

            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )

            self.get_logger().info('Started audio recording')

            record_thread = threading.Thread(target=self.record_audio)
            record_thread.daemon = True
            record_thread.start()

        except Exception as e:
            self.get_logger().error(f'Error starting audio recording: {str(e)}')
            self.recording = False

    def record_audio(self):
        try:
            frames = []

            for _ in range(0, int(self.sample_rate / self.chunk_size * self.record_seconds)):
                if not self.recording:
                    break
                data = self.stream.read(self.chunk_size)
                frames.append(data)

            self.stop_recording()
            self.process_audio(frames)

        except Exception as e:
            self.get_logger().error(f'Error recording audio: {str(e)}')
            self.recording = False

    def stop_recording(self):
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            if self.audio:
                self.audio.terminate()
            self.recording = False
            self.get_logger().info('Stopped audio recording')
        except Exception as e:
            self.get_logger().error(f'Error stopping audio recording: {str(e)}')

    def process_audio(self, frames: list):
        try:
            import io
            wav_buffer = io.BytesIO()

            with wave.open(wav_buffer, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(pyaudio.get_sample_size(pyaudio.paInt16))
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(frames))

            wav_data = wav_buffer.getvalue()
            temp_filename = '/tmp/temp_audio.wav'
            with open(temp_filename, 'wb') as f:
                f.write(wav_data)

            transcript = self.transcribe_audio(temp_filename)

            if transcript:
                self.publish_transcript(transcript)

            if os.path.exists(temp_filename):
                os.remove(temp_filename)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')

    def transcribe_audio(self, audio_file_path: str) -> Optional[str]:
        if not self.openai_api_key:
            self.get_logger().error('OpenAI API key not set')
            return None

        try:
            openai.api_key = self.openai_api_key

            with open(audio_file_path, 'rb') as audio_file:
                transcript_response = openai.Audio.transcribe(
                    model=self.whisper_model,
                    file=audio_file
                )

            transcript = transcript_response.get('text', '').strip()
            self.get_logger().info(f'Transcribed: {transcript}')

            return transcript

        except Exception as e:
            self.get_logger().error(f'Error transcribing audio: {str(e)}')
            return None

    def publish_transcript(self, transcript: str):
        try:
            transcript_msg = String()
            transcript_msg.data = transcript
            self.transcript_pub.publish(transcript_msg)

            command_msg = String()
            command_msg.data = transcript
            self.command_pub.publish(command_msg)

            self.get_logger().info(f'Published transcript: {transcript}')

        except Exception as e:
            self.get_logger().error(f'Error publishing transcript: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    voice_processor = VoiceProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        voice_processor.get_logger().info('Shutting down Voice Processor')
    finally:
        if voice_processor.recording:
            voice_processor.stop_recording()
        voice_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Exercise 2 Solution

Cognitive Planning System:
```python
#!/usr/bin/env python3
"""
Cognitive Planning System for Humanoid Robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from move_base_msgs.action import MoveBase
from humanoid_control_interfaces.action import NavigateToPose, ManipulateObject
from typing import Dict, List, Optional, Tuple
import json
import re
from dataclasses import dataclass
from enum import Enum


class TaskType(Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    GESTURE = "gesture"
    SPEECH = "speech"
    WAIT = "wait"


@dataclass
class TaskStep:
    id: str
    task_type: TaskType
    parameters: Dict[str, any]
    priority: int = 1
    dependencies: List[str] = None
    timeout: float = 30.0

    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []


class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')

        # Parameters
        self.declare_parameter('plan_timeout', 60.0)
        self.declare_parameter('max_plan_steps', 20)
        self.declare_parameter('enable_plan_optimization', True)

        self.plan_timeout = self.get_parameter('plan_timeout').value
        self.max_plan_steps = self.get_parameter('max_plan_steps').value
        self.enable_plan_optimization = self.get_parameter('enable_plan_optimization').value

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/high_level_command',
            self.command_callback,
            10
        )

        # Publishers
        self.plan_pub = self.create_publisher(String, '/action_plan', 10)
        self.plan_status_pub = self.create_publisher(Bool, '/plan_active', 10)

        # Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulate_object_client = ActionClient(self, ManipulateObject, 'manipulate_object')

        # State variables
        self.current_plan = []
        self.current_step_index = 0
        self.plan_active = False
        self.plan_history = []

        self.get_logger().info('Cognitive Planner initialized')

    def command_callback(self, msg: String):
        try:
            command_text = msg.data.lower().strip()
            self.get_logger().info(f'Received high-level command: {command_text}')

            plan = self.generate_plan_from_command(command_text)

            if plan:
                plan_msg = String()
                plan_msg.data = json.dumps([{
                    'id': step.id,
                    'task_type': step.task_type.value,
                    'parameters': step.parameters,
                    'priority': step.priority,
                    'dependencies': step.dependencies,
                    'timeout': step.timeout
                } for step in plan])
                self.plan_pub.publish(plan_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def generate_plan_from_command(self, command: str) -> Optional[List[TaskStep]]:
        try:
            # Parse the command to understand the task
            parsed_tasks = self.parse_command(command)

            if not parsed_tasks:
                return None

            # Create task steps
            plan = []
            step_id = 1

            for task in parsed_tasks:
                if task['type'] == 'navigation':
                    nav_step = TaskStep(
                        id=f'nav_{step_id}',
                        task_type=TaskType.NAVIGATION,
                        parameters={
                            'destination': task['destination'],
                            'x': task.get('x', 0.0),
                            'y': task.get('y', 0.0),
                            'theta': task.get('theta', 0.0)
                        }
                    )
                    plan.append(nav_step)
                    step_id += 1

                elif task['type'] == 'manipulation':
                    manip_step = TaskStep(
                        id=f'manip_{step_id}',
                        task_type=TaskType.MANIPULATION,
                        parameters={
                            'action': task['action'],
                            'object': task.get('object', 'unknown'),
                            'pose': task.get('pose', {})
                        }
                    )
                    plan.append(manip_step)
                    step_id += 1

                elif task['type'] == 'gesture':
                    gesture_step = TaskStep(
                        id=f'gesture_{step_id}',
                        task_type=TaskType.GESTURE,
                        parameters={
                            'gesture_type': task['gesture'],
                            'duration': task.get('duration', 2.0)
                        }
                    )
                    plan.append(gesture_step)
                    step_id += 1

            # Optimize plan if enabled
            if self.enable_plan_optimization:
                plan = self.optimize_plan(plan)

            return plan

        except Exception as e:
            self.get_logger().error(f'Error generating plan from command: {str(e)}')
            return None

    def parse_command(self, command: str) -> List[Dict]:
        tasks = []

        # Navigation commands
        nav_patterns = [
            (r'move to (.+)', 'navigation'),
            (r'go to (.+)', 'navigation'),
            (r'navigate to (.+)', 'navigation'),
            (r'walk to (.+)', 'navigation'),
        ]

        for pattern, task_type in nav_patterns:
            match = re.search(pattern, command)
            if match:
                destination = match.group(1).strip()
                tasks.append({
                    'type': task_type,
                    'destination': destination
                })

        # Manipulation commands
        manip_patterns = [
            (r'pick up (.+)', 'manipulation'),
            (r'grab (.+)', 'manipulation'),
            (r'take (.+)', 'manipulation'),
            (r'lift (.+)', 'manipulation'),
            (r'put down (.+)', 'manipulation'),
            (r'drop (.+)', 'manipulation'),
        ]

        for pattern, task_type in manip_patterns:
            match = re.search(pattern, command)
            if match:
                obj = match.group(1).strip()
                tasks.append({
                    'type': task_type,
                    'action': 'pick_up' if any(x in pattern for x in ['pick up', 'grab', 'take', 'lift']) else 'put_down',
                    'object': obj
                })

        # Gesture commands
        gesture_patterns = [
            (r'wave', 'gesture'),
            (r'wave hello', 'gesture'),
            (r'nod', 'gesture'),
        ]

        for pattern, task_type in gesture_patterns:
            if re.search(pattern, command):
                tasks.append({
                    'type': task_type,
                    'gesture': pattern
                })

        return tasks

    def optimize_plan(self, plan: List[TaskStep]) -> List[TaskStep]:
        try:
            # Simple optimization: group navigation tasks together
            nav_steps = [step for step in plan if step.task_type == TaskType.NAVIGATION]
            other_steps = [step for step in plan if step.task_type != TaskType.NAVIGATION]

            # Reorder: navigation first, then other tasks
            optimized_plan = nav_steps + other_steps

            # Update dependencies if needed
            for i, step in enumerate(optimized_plan):
                if i > 0 and not step.dependencies:
                    step.dependencies = [optimized_plan[i-1].id]

            return optimized_plan

        except Exception as e:
            self.get_logger().error(f'Error optimizing plan: {str(e)}')
            return plan


def main(args=None):
    rclpy.init(args=args)
    cognitive_planner = CognitivePlanner()

    try:
        rclpy.spin(cognitive_planner)
    except KeyboardInterrupt:
        cognitive_planner.get_logger().info('Shutting down Cognitive Planner')
    finally:
        cognitive_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Assessment Rubric

### Technical Implementation (60%)
- Proper voice processing implementation with Whisper (10%)
- Effective cognitive planning system (10%)
- Robust voice-to-action pipeline (10%)
- Accurate object recognition integration (10%)
- Comprehensive multi-step execution framework (10%)
- Successful LLM integration (10%)

### Code Quality (25%)
- Clean, well-documented code (10%)
- Proper error handling and validation (10%)
- Following ROS 2 and Isaac Sim best practices (5%)

### Integration and Testing (15%)
- Successful integration of all VLA components (10%)
- Comprehensive testing and validation (5%)

## Learning Objectives Assessment

After completing these exercises, students should be able to:
- Implement voice processing systems with OpenAI Whisper
- Design cognitive planning systems for multi-step tasks
- Create voice-to-action pipelines connecting language to actions
- Integrate object recognition with robot perception
- Execute complex multi-step action plans
- Integrate LLMs for advanced task planning
- Implement comprehensive task execution logging
- Build integrated VLA demonstrations
- Optimize VLA systems for performance