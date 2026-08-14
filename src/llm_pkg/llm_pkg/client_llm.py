import sys
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import Llm


class LlmClientAsync(Node):
    def __init__(self):
        super().__init__("llm_client_async")
        self.cli = self.create_client(Llm, "llm")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = Llm.Request()

    def send_request(self, user_input: str):
        self.req.user_input = user_input
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    if len(sys.argv) < 2:
        raise SystemExit("Usage: ros2 run llm_pkg client_llm '<your instruction>'")

    user_input = " ".join(sys.argv[1:])
    llm_client = LlmClientAsync()
    future = llm_client.send_request(user_input)
    rclpy.spin_until_future_complete(llm_client, future)

    result = future.result()
    if result is None:
        llm_client.get_logger().error("No response from service.")
    else:
        llm_client.get_logger().info(str(result.pairs))
         
    llm_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
