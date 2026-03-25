#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class GameStart : public BT::ConditionNode
{
public:
    GameStart(const std::string& condition_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void GameStart_callback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gamestart_sub_;
};