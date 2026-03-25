#include "decision/check_status.hpp"

CheckStatus::CheckStatus(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf) {}

BT::PortsList CheckStatus::providedPorts()
{
    return { BT::InputPort<bool>("get_status") };
}

BT::NodeStatus CheckStatus::tick()
{
    bool status;
    getInput("get_status", status);
    return status ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
