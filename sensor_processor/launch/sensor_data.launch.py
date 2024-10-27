from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_processor',  # Replace with the actual package name
            executable='accel',          # Replace with the actual executable name for the accelerometer node
            name='accel_node',
            output='screen',
        ),
        Node(
            package='sensor_processor',  # Replace with the actual package name
            executable='gyro',           # Replace with the actual executable name for the gyro node
            name='gyro_node',
            output='screen',
        ),
        Node(
            package='sensor_processor',  # Replace with the actual package name
            executable='obstacle',       # Replace with the actual executable name for the obstacle node
            name='obstacle_node',
            output='screen',
        ),
        Node(
            package='sensor_processor',  # Replace with the actual package name
            executable='process',        # Replace with the actual executable name for the processing node
            name='process_node',
            output='screen',
        ),
    ])

