"""
Handles spawning random turtles, publishing their name and co-ordinates to
topic '/alive_turtles', hosts a service '/catch_turtle' to kill them.
"""

import rclpy
from functools import partial
from random import Random
from rclpy.node import Node

from turtlesim.srv import Spawn, Kill

from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawner(Node):

    # Constants
    PARAM_ONE = "spawn_frequency"
    PARAM_TWO = "turtle_name_prefix"

    def __init__(self):
        super().__init__("turtle_spawner")

        # Parameters
        self.declare_parameter(self.PARAM_ONE, 0.5)
        self.declare_parameter(self.PARAM_TWO, "murtle")

        # Class Variables
        self.spawn_frequency = self.get_parameter(self.PARAM_ONE).value
        self.turtle_name_prefix = self.get_parameter(self.PARAM_TWO).value
        self.get_logger().debug(
            f"Starting node /{self.get_name()} with parameters {self.PARAM_ONE}:{self.spawn_frequency} {self.PARAM_TWO}:{self.turtle_name_prefix}"
        )
        self.alive_turtles = TurtleArray()
        self.random = Random()

        # Topics/Services/Clients
        self.alive_turtle_publisher = self.create_publisher(
            TurtleArray, "alive_turtles", 10
        )
        self.turtle_catcher_service = self.create_service(
            CatchTurtle, "catch_turtle", self.catch_turtle_callback
        )
        self.turtle_spawner_client = self.create_client(Spawn, "spawn")
        self.turtle_killer_client = self.create_client(Kill, "kill")

        # Timers
        self.create_timer(1 / self.spawn_frequency, self.spawn_turtles)
        self.create_timer(1.0, self.publish_alive_turtles)

        # Start with one turtle already
        self.spawn_turtles()

    def spawn_turtles(self):
        while not self.turtle_spawner_client.wait_for_service(1.0):
            self.get_logger().info("Waiting for server /spawn ...")

        request = Spawn.Request()
        turtle = Turtle()
        request.x = turtle.x = self.__random_coordinate()
        request.y = turtle.y = self.__random_coordinate()
        request.theta = turtle.theta = self.__random_coordinate()
        request.name = turtle.name = (
            self.turtle_name_prefix + "_" + self.__random_six_digits()
        )

        self.get_logger().info(f"Calling service '/spawn' with request {str(request)}")
        future = self.turtle_spawner_client.call_async(request)
        future.add_done_callback(
            partial(self.callback_spawn_turtle_client, request=request, turtle=turtle)
        )

    def publish_alive_turtles(self):
        self.get_logger().info(f"Publishing alive turtles to /alive_turtles ...")
        self.alive_turtle_publisher.publish(self.alive_turtles)

    def callback_spawn_turtle_client(self, future, request, turtle):
        try:
            response = future.result()
            self.get_logger().info(f"Spwaning {response.name} successful.")
            self.alive_turtles.alive_turtles.append(turtle)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def catch_turtle_callback(self, request, response):
        while not self.turtle_killer_client.wait_for_service(1.0):
            self.get_logger().info("Waiting for server /kill ...")

        kill_request = Kill.Request()
        kill_request.name = request.name

        self.get_logger().info(f"Killing turtle {request.name} ...")
        self.turtle_killer_client.call_async(kill_request)
        self.__forget_turtle(request.name)

        return response

    def __forget_turtle(self, name):
        len_alive_turtles = len(self.alive_turtles.alive_turtles)
        for i, turtle in enumerate(self.alive_turtles.alive_turtles):
            if turtle.name == name:
                self.alive_turtles.alive_turtles.pop(i)

    def __random_coordinate(self):
        return float(self.random.randint(1, 10))

    def __random_angle(self):
        return float(self.random.randint(0, 360))

    def __random_six_digits(self):
        return str(self.random.randint(100000, 999999))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
