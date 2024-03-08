import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLedStatePanel
from my_robot_interfaces.msg import LedStateArray


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")

        self.declare_parameter("panel_state", [False, False, False])

        self.led_state_publisher = self.create_publisher(
            LedStateArray, "led_panel_state", 10
        )
        self.set_led_srv = self.create_service(
            SetLedStatePanel, "set_led", self.set_led_state
        )
        self.led_panel = self.get_parameter("panel_state").value
        self.timer_ = self.create_timer(4, self.publish_led_panel_state)
        self.get_logger().info("Led Panel Node has started.")

    def set_led_state(self, request, response):
        try:
            self.led_panel[request.led_number - 1] = request.state
            response.success = True
            self.publish_led_panel_state()
        except IndexError as ex:
            response.success = False

        return response

    def publish_led_panel_state(self):
        msg = LedStateArray()
        msg.led_states = self.led_panel
        self.led_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
