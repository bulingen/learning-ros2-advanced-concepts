#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from typing import Optional, cast
from my_robot_interfaces.action._count_until import CountUntil_Goal


class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.count_until_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.goal_lock_ = threading.Lock()
        self.current_goal_handle_: Optional[ServerGoalHandle] = None

        self.get_logger().info("Action server has been started")

    def goal_callback(self, goal_request: CountUntil.Goal) -> GoalResponse:
        self.get_logger().info("Received a goal")

        # Policy: refuse new goal if current goal still active
        # make sure two threads aren't accessing the handle simultaneously
        # with self.goal_lock_:
        #     if self.current_goal_handle_ and self.current_goal_handle_.is_active:
        #         self.get_logger().info("A goal is already active, rejecting new goal")
        #         return GoalResponse.REJECT

        # validate the goal request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT

        # Policy: preempt existing goal when receiving new goal
        with self.goal_lock_:
            if self.current_goal_handle_ and self.current_goal_handle_.is_active:
                self.get_logger().info("Aborting current goal and accepting new goal")
                self.current_goal_handle_.abort()

        self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> CountUntil.Result:
        # make sure two threads aren't accessing the handle simultaneously
        with self.goal_lock_:
            # set global handle for access elsewhere
            self.current_goal_handle_ = goal_handle

        # get request from goal
        request = cast(CountUntil_Goal, goal_handle.request)
        target_number = request.target_number
        period = request.period

        # execute the action
        self.get_logger().info("Executing the goal")
        counter = 0
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # once done, set goal final state
        goal_handle.succeed()
        # send result

        result.reached_number = counter
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
