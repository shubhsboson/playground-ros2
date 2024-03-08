import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        self.alive_turtles = []
        self.current_pose = Pose()

        self.client_catch_turtle = self.create_client(CatchTurtle, "catch_turtle")
        self.pub_topic_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "turtle1/pose", self.callback_current_pose, 10)
        self.create_subscription(
            TurtleArray, "/alive_turtles", self.callback_sub_topic_alive_turtles, 10
        )

        self.create_timer(0.1, self.hunt_turtles)

    def hunt_turtles(self):
        if not self.alive_turtles == []:
            target = self.alive_turtles[0]
            hunter = self.current_pose
            msg = Twist()
            msg.linear.x = (
                abs(math.sqrt((target.x - hunter.x) ** 2 + (target.y - hunter.y) ** 2))
                * 6
            )
            msg.angular.z = (
                math.atan2(target.y - hunter.y, target.x - hunter.x) - hunter.theta
            ) * 12
            self.pub_topic_vel.publish(msg)

            if self.are_close(hunter, target):
                request = CatchTurtle.Request()
                request.name = target.name
                self.client_catch_turtle.call_async(request)
                self.get_logger().info(f"Caught {target.name}")
                self.alive_turtles.pop(0)
        else:
            self.get_logger().info("No turtle alive to hunt.")

    def are_close(self, turtle1, turtle2):
        error_radius = 0.2
        return (
            abs(turtle1.x - turtle2.x) < error_radius
            and abs(turtle1.y - turtle2.y) < error_radius
        )
        # return True

    def callback_sub_topic_alive_turtles(self, msg):
        self.alive_turtles = msg.alive_turtles

    def callback_current_pose(self, msg):
        self.current_pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
