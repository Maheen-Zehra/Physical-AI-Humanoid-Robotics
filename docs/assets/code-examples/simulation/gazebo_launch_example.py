from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='humanoid_world.world',
        description='Choose one of the world files from `/gazebo_worlds/models`'
    )

    # Get the launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Create launch description
    ld = LaunchDescription()

    # Add argument declaration
    ld.add_action(world_file_arg)

    # Launch Gazebo with the interaction world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': PathJoinSubstitution([get_package_share_directory('gazebo_worlds'),
                                          'models', LaunchConfiguration('world')])
        }.items()
    )

    # Spawn the humanoid model
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_humanoid',
            '-file', PathJoinSubstitution([get_package_share_directory('humanoid_control'),
                                          'models', 'simple_humanoid.urdf']),
            '-x', '0', '-y', '0', '-z', '1'
        ],
        output='screen'
    )

    # Add all actions to launch description
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    return ld