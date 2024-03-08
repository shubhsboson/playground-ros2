#!/usr/bin/env python3

import rclpy
from time import sleep
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import SetBool


class ResetCounterClientNode(Node):  
    def __init__(self):
        super().__init__("reset_counter_client")
        self.call_reset_counter_server(True)
        sleep(20)
        self.call_reset_counter_server(False)

    def call_reset_counter_server(self, flag):
        client = self.create_client(SetBool, "reset_counter")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server /reset_counter")

        request = SetBool.Request()
        request.data = flag

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_reset_counter))

    def callback_reset_counter(self, future):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))
            self.get_logger().info(str(response.message))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterClientNode()  
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
