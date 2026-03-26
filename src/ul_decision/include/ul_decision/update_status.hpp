#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include <atomic>
#include <vector>

class UpdateStatus : public BT::SyncActionNode
{
public:
    UpdateStatus(const std::string& action_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    void cur_hp_callback(std_msgs::msg::Float32 hp_msg);
    void ctlarea_status_callback(std_msgs::msg::Int16 ctl_msg);
    void enermy_observed_callback(std_msgs::msg::Bool enermy_msg);
    void enermy_onhighland_callback(std_msgs::msg::Bool highland_msg);

    std::vector<double> add_blood_, pos_1_, pos_2_, pos_3_, pos_4_, pos_5_, attack_pos_, idle_pos_;
    double min_hp_, max_hp_;
    int temp_ctlarea_status;
    bool temp_enermy_onhighland;

    std::atomic<bool> hp_status_;
    std::atomic<int> ctlarea_status_;
    std::atomic<bool> enermy_observed_;
    std::atomic<bool> enermy_onhighland_;
    std::atomic<double> cur_hp_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cur_hp_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ctlarea_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enermy_observed_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enermy_onhighland_sub_;
};