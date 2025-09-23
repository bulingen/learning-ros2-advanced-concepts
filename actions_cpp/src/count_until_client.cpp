#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;
class CountUntilClientNode : public rclcpp::Node
{
public:
    CountUntilClientNode() : Node("count_until_client")
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int target_number, double period)
    {
        // Wait for action server
        RCLCPP_INFO(this->get_logger(), "Waiting for server");
        count_until_client_->wait_for_action_server();

        // create goal
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        // add callbacks
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback, this, _1);

        // send goal
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        count_until_client_->async_send_goal(goal, options);
    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;

    // callback to receive the result once the goal is done
    void goal_result_callback(const CountUntilGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeded");
        }
        else if (status == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_WARN(this->get_logger(), "Aborted");
        }
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
    }

    // callback to know if the goal was accepted or rejected
    void goal_response_callback(const CountUntilGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal got rejected");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal was accepted");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->send_goal(6, 1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
