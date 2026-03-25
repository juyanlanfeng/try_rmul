#include "decision/game_start.hpp"

GameStart::GameStart(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    gamestart_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/game_start", 10, std::bind(&GameStart::GameStart_callback, this, std::placeholders::_1));
}

void GameStart::GameStart_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    setOutput("update_start", msg->data);
    RCLCPP_INFO(node_->get_logger(), "游戏开始");
}

BT::PortsList GameStart::providedPorts()
{
    return {
        BT::InputPort<bool>("get_start"),
        BT::OutputPort<bool>("update_start")
    };
}

BT::NodeStatus GameStart::tick()
{
    bool game_start;
    getInput("get_start", game_start);
    if (game_start) {
        return BT::NodeStatus::SUCCESS;
    }
    else {
        return BT::NodeStatus::FAILURE;
    }
}