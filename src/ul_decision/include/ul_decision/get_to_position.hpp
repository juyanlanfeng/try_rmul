#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include <atomic>

class GetToPosition : public BT::StatefulActionNode
{
public:
    GetToPosition(const std::string& action_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    static int ticks; // 模拟耗时计数
    rclcpp::Node::SharedPtr node_;
    std::atomic<double> target_pos_x_, target_pos_y_;
};