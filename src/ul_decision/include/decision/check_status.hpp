#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>

class CheckStatus : public BT::ConditionNode
{
public:
    CheckStatus(const std::string& condition_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};