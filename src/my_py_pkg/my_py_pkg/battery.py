import rclpy
from rclpy.node import Node
from time import sleep
from functools import partial

from my_robot_interfaces.srv import SetLedStatePanel


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")

        self.get_logger().info("Setting battery state to 'full' ...")
        self.set_led_state(3, False)
        sleep(4)
        self.get_logger().info("Setting battery state to 'empty' ...")
        self.set_led_state(3, True)
        sleep(6)

    def set_led_state(self, led_number, state):
        led_panel_client = self.create_client(SetLedStatePanel, "set_led")
        while not led_panel_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service'/set_led' ...")

        request = SetLedStatePanel.Request()
        request.led_number = led_number
        request.state = state

        future = led_panel_client.call_async(request)
        future.add_done_callback(partial(self.callback_set_led_state))

    def callback_set_led_state(self, future):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
