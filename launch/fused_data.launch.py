from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sensor_fusion_pkg",
            executable="fused_data",
            name="fused_data",
            output="screen",  #this means that we are printing the node logs directly to the terminal 
                              #instead of 'log' which writes to a file, which is useful for production, but annoying for development.
            parameters=[{     #we are overriding the default values declared in the code here. we can tune alpha here without touching code using 
                              # ros2 launch sensor_fusion_pkg fused_data.launch.py alpha:=0.95
                              
                "alpha": 0.98,   # Complementary filter coefficient
                "max_dt": 1.0,   # Max allowed time gap between messages (s)
            }],
        )
    ])
