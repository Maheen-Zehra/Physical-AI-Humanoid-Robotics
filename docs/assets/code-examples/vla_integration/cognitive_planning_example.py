#!/usr/bin/env python3
"""
Cognitive Planning Example for Humanoid Robot

This example demonstrates the cognitive planning system that creates
multi-step action plans based on high-level commands for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from move_base_msgs.action import MoveBase
from humanoid_control_interfaces.action import NavigateToPose, ManipulateObject
from typing import Dict, List, Optional, Tuple, Any
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
    """Represents a single step in a multi-step task plan."""
    id: str
    task_type: TaskType
    parameters: Dict[str, Any]
    priority: int = 1
    dependencies: List[str] = None
    timeout: float = 30.0

    def __post_init__(self):
        if self.dependencies is None:
            self.dependencies = []


class CognitivePlanningExample(Node):
    """
    Example cognitive planning system that creates multi-step action plans.
    """

    def __init__(self):
        super().__init__('cognitive_planning_example')

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

        self.get_logger().info('Cognitive Planning Example initialized')

    def command_callback(self, msg: String):
        """
        Process high-level commands and generate action plans.
        """
        try:
            command_text = msg.data.lower().strip()
            self.get_logger().info(f'Received high-level command: {command_text}')

            # Generate plan from command
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
        """
        Generate a multi-step plan from a natural language command.
        """
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
        """
        Parse a natural language command into structured tasks.
        """
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
        """
        Optimize the plan by reordering steps for efficiency.
        """
        try:
            # Simple optimization: group navigation tasks together
            nav_steps = [step for step in plan if step.task_type == TaskType.NAVIGATION]
            other_steps = [step for step in plan if step.task_type != TaskType.NAVIGATION]

            # Reorder: navigation first, then other tasks
            optimized_plan = nav_steps + other_steps

            # Update dependencies if needed
            for i, step in enumerate(optimized_plan):
                if i > 0 and not step.dependencies:
                    # Add dependency on previous step
                    step.dependencies = [optimized_plan[i-1].id]

            return optimized_plan

        except Exception as e:
            self.get_logger().error(f'Error optimizing plan: {str(e)}')
            return plan


def main(args=None):
    rclpy.init(args=args)
    cognitive_planning_example = CognitivePlanningExample()

    try:
        rclpy.spin(cognitive_planning_example)
    except KeyboardInterrupt:
        cognitive_planning_example.get_logger().info('Shutting down Cognitive Planning Example')
    finally:
        cognitive_planning_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()