from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["Giskard", "BB8", "Daneel", "Lander", "C3PO"]

    for robot_name in robot_names:
        robot_news_node = Node(
            package="my_py_pkg",
            executable="robot_news_station",
            remappings=[("__node", "robot_news_station_" + robot_name.lower())],
            parameters=[{"robot_name": robot_name}],
        )

        ld.add_action(robot_news_node)

    smartphone_node = Node(package="my_cpp_pkg", executable="rhobot")

    ld.add_action(smartphone_node)

    return ld
