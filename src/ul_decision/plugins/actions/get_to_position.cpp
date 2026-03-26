#include "ul_decision/get_to_position.hpp"
#include "ul_decision/public_data.hpp"

int GetToPosition::ticks = 0;

GetToPosition::GetToPosition(const std::string& action_name, const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(action_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    at_home = true;  // 初始假设在家
}

BT::PortsList GetToPosition::providedPorts()
{
    return { BT::InputPort<std::vector<double>>("target_position") };
}

BT::NodeStatus GetToPosition::onStart()
{
    std::vector<double> temp_pos;
    getInput("target_position", temp_pos);

    target_pos_x_.store(temp_pos[0]);
    target_pos_y_.store(temp_pos[1]);
    RCLCPP_INFO(node_->get_logger(), "开始前往(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetToPosition::onRunning()
{
    if (ticks % 50 == 0) {
        RCLCPP_INFO(node_->get_logger(), "正在前往(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());
    }
    ticks++;

    if (ticks <= 200) {
        return BT::NodeStatus::RUNNING;
    } else {
        RCLCPP_INFO(node_->get_logger(), "已经到达(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());
        // 根据目标位置更新全局 at_home
        if (target_pos_x_.load() == 0.0 && target_pos_y_.load() == 0.0) {
            at_home = true;
        } else if (target_pos_x_.load() == 1.0 && target_pos_y_.load() == 1.0) {
            at_home = false;
        }
        ticks = 0;
        return BT::NodeStatus::SUCCESS;
    }
}

void GetToPosition::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "前往(x:%f y:%f)的进程被打断", target_pos_x_.load(), target_pos_y_.load());
    ticks = 0;
}