#!/usr/bin/env python3
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from my_robot_interfaces.action._count_until import (
    CountUntil_GetResult_Response,
    CountUntil_FeedbackMessage,
)
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from my_robot_interfaces.action import CountUntil
from typing import cast, Optional


class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
        self.goal_handle_: Optional[ClientGoalHandle] = None

    def send_goal(self, target_number: int, period: float) -> None:
        # wait for the server
        self.count_until_client_.wait_for_server()

        # create goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # send goal
        self.get_logger().info("Sending goal")
        self.count_until_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

        # Send a cancel request 2 seconds later
        self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self) -> None:
        self.get_logger().info("Send a cancel request")
        self.timer_.cancel()
        if self.goal_handle_:
            self.goal_handle_.cancel_goal_async()

    def goal_feedback_callback(self, feedback_msg: CountUntil_FeedbackMessage) -> None:
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f"Got feedback {number}")

    def goal_response_callback(self, future: Future) -> None:
        self.goal_handle_ = cast(ClientGoalHandle, future.result())
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
        status = goal_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(f"Goal aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled")
        reached_number = goal_result.result.reached_number
        self.get_logger().info(f"Result: {reached_number}")


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
