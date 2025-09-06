#!/usr/bin/env python3
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from my_robot_interfaces.action._count_until import CountUntil_GetResult_Response
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from my_robot_interfaces.action import CountUntil
from typing import cast


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")

    def send_goal(self, target_number: int, period: float) -> None:
        # wait for the server
        self.count_until_client_.wait_for_server()

        # create goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # send goal
        self.get_logger().info("Sending goal")
        self.count_until_client_.send_goal_async(goal).add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future: Future) -> None:
        self.goal_handle_: ClientGoalHandle = cast(ClientGoalHandle, future.result())
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            result: Future = self.goal_handle_.get_result_async()
            result.add_done_callback(self.goal_result_callback)

        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future: Future) -> None:
        goal_result: CountUntil_GetResult_Response = cast(
            CountUntil_GetResult_Response, future.result()
        )
        reached_number = goal_result.result.reached_number
        self.get_logger().info(f"Result: {reached_number}")


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(-6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
