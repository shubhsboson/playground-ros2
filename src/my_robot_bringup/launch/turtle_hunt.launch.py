from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(package="turtlesim", executable="turtlesim_node")
    turtle_spawner_node = Node(
        package="turtlesim_catchem_all", executable="turtle_spawner"
    )
    turtle_controller_node = Node(
        package="turtlesim_catchem_all", executable="turtle_controller"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)

    return ld
