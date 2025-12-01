from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "do_our_best",
                default_value="True",
                description="What you should do",
                choices=["True", "False"],
            ),
            DeclareLaunchArgument(
                "ideal_depth_m",
                default_value="1.3",
                description="The ideal depth we dive too",
            ),
            DeclareLaunchArgument(
                "mission",
                default_value="best_mission",
                description="Which Mission File you want to run",
                choices=["alpha", "bravo", "bravo2", "charlie", "charlie2", "delta", "best_mission"],
            ),
            Node(
                package="ht_core",
                executable="mission_node",
                name="mission_node",
                output="screen",
                parameters=[
                    {"ideal_depth_m": LaunchConfiguration("ideal_depth_m")},
                    {"do_our_best": LaunchConfiguration("do_our_best")},
                    {"mission": LaunchConfiguration("mission")},

                ],
            ),
            Node(
                package="ht_core",
                executable="depth_node",
                name="depth_node",
                output="screen",
                parameters=[
                ],
            ),
            Node(
                package="my_gyro_pkg",
                executable="fog_node",
                name="fog_node",
                output="screen",
                parameters=[
                ],
            ),
            
        ]
    )
