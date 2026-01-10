from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de seguimiento de línea con control PID
        Node(
            package='puzzlebot',
            executable='image_viewer_node',
            name='image_viewer',
            output='screen'
        ),

        # Nodo de detección de señales con YOLO
        Node(
            package='puzzlebot',
            executable='ml_yolo',
            name='ml_yolo',
            output='screen'
        ),

        # Nodo de detección de semáforo por color
        Node(
            package='puzzlebot',
            executable='traffic_light_node',
            name='traffic_light_detector',
            output='screen'
        ),

        # Nodo maestro de control jerárquico y FSM
        Node(
            package='puzzlebot',
            executable='master_node',
            name='master_node',
            output='screen'
        )
    ])