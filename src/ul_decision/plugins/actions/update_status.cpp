#include "ul_decision/update_status.hpp"
#include "ul_decision/public_data.hpp"

UpdateStatus::UpdateStatus(const std::string& action_name, const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(action_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    node_->declare_parameter<std::vector<double>>("add_blood", {0.0, 0.0});
    node_->declare_parameter<std::vector<double>>("pos1", {1.0, 1.0});
    node_->declare_parameter<std::vector<double>>("pos2", {2.0, 2.0});
    node_->declare_parameter<std::vector<double>>("pos3", {3.0, 3.0});
    node_->declare_parameter<std::vector<double>>("pos4", {4.0, 4.0});
    node_->declare_parameter<std::vector<double>>("pos5", {5.0, 5.0});
    node_->declare_parameter<std::vector<double>>("attack_pos", {9.0, 9.0});
    node_->declare_parameter<std::vector<double>>("idle_pos", {10.0, 10.0});

    node_->declare_parameter<double>("max_blood", 400.0);
    node_->declare_parameter<double>("min_blood", 200.0);
    node_->declare_parameter<int>("ctlarea_status", 3);
    node_->declare_parameter<bool>("enermy_onhighland", false);

    add_blood_ = node_->get_parameter("add_blood").as_double_array();
    pos_1_ = node_->get_parameter("pos1").as_double_array();
    pos_2_ = node_->get_parameter("pos2").as_double_array();
    pos_3_ = node_->get_parameter("pos3").as_double_array();
    pos_4_ = node_->get_parameter("pos4").as_double_array();
    pos_5_ = node_->get_parameter("pos5").as_double_array();
    attack_pos_ = node_->get_parameter("attack_pos").as_double_array();
    idle_pos_ = node_->get_parameter("idle_pos").as_double_array();

    max_hp_ = node_->get_parameter("max_blood").as_double();
    min_hp_ = node_->get_parameter("min_blood").as_double();
    temp_ctlarea_status = node_->get_parameter("ctlarea_status").as_int();
    temp_enermy_onhighland = node_->get_parameter("enermy_onhighland").as_bool();

    cur_hp_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/cur_hp", 10, std::bind(&UpdateStatus::cur_hp_callback, this, std::placeholders::_1));
    ctlarea_status_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
        "/ctlarea_status", 10, std::bind(&UpdateStatus::ctlarea_status_callback, this, std::placeholders::_1));
    enermy_observed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/enermy_observed", 10, std::bind(&UpdateStatus::enermy_observed_callback, this, std::placeholders::_1));
    enermy_onhighland_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/enermy_onhighland", 10, std::bind(&UpdateStatus::enermy_onhighland_callback, this, std::placeholders::_1));

    cur_hp_.store(max_hp_); // 默认满血
    ctlarea_status_.store(temp_ctlarea_status);
    enermy_observed_.store(false);
    enermy_onhighland_.store(temp_enermy_onhighland);
}

void UpdateStatus::cur_hp_callback(std_msgs::msg::Float32 hp_msg)
{
    cur_hp_.store(hp_msg.data);
    RCLCPP_INFO(node_->get_logger(), "生命值更改成功");
}

void UpdateStatus::ctlarea_status_callback(std_msgs::msg::Int16 ctl_msg)
{
    ctlarea_status_.store(ctl_msg.data);
    RCLCPP_INFO(node_->get_logger(), "控制区状态更改成功");
}

void UpdateStatus::enermy_observed_callback(std_msgs::msg::Bool enermy_msg)
{
    enermy_observed_.store(enermy_msg.data);
    RCLCPP_INFO(node_->get_logger(), "敌人状态更改成功");
}

void UpdateStatus::enermy_onhighland_callback(std_msgs::msg::Bool highland_msg)
{
    enermy_onhighland_.store(highland_msg.data);
    RCLCPP_INFO(node_->get_logger(), "高地状态更改成功");
}

BT::PortsList UpdateStatus::providedPorts()
{
    return {
        BT::OutputPort<bool>("hp_status"),
        BT::OutputPort<std::string>("ctlarea_status"),
        BT::OutputPort<bool>("enermy_observed"),
        BT::OutputPort<bool>("enermy_onhighland"),
        BT::OutputPort<bool>("at_home"),
        BT::OutputPort<std::vector<double>>("add_blood"),
        BT::OutputPort<std::vector<double>>("pos_1"),
        BT::OutputPort<std::vector<double>>("pos_2"),
        BT::OutputPort<std::vector<double>>("pos_3"),
        BT::OutputPort<std::vector<double>>("pos_4"),
        BT::OutputPort<std::vector<double>>("pos_5"),
        BT::OutputPort<std::vector<double>>("attack_pos"),
        BT::OutputPort<std::vector<double>>("idle_pos")
    };
}

BT::NodeStatus UpdateStatus::tick()
{
    setOutput("add_blood", add_blood_);
    setOutput("pos_1", pos_1_);
    setOutput("pos_2", pos_2_);
    setOutput("pos_3", pos_3_);
    setOutput("pos_4", pos_4_);
    setOutput("pos_5", pos_5_);
    setOutput("attack_pos", attack_pos_);
    setOutput("idle_pos", idle_pos_);

    switch (ctlarea_status_.load()) {
        case 1:
            setOutput("ctlarea_status", "enermy_control");
            break;
        case 2:
            setOutput("ctlarea_status", "myteam_control");
            break;
        case 3:
            setOutput("ctlarea_status", "none_control");
            break;
        default:
            setOutput("ctlarea_status", "default");
            break;
    }

    setOutput("hp_status", cur_hp_.load() <= min_hp_);
    setOutput("enermy_observed", enermy_observed_.load());
    setOutput("enermy_onhighland", enermy_onhighland_.load());

    // 使用全局变量 at_home
    if (at_home) {
        cur_hp_.store(max_hp_);
        setOutput("at_home", true);
    } else {
        setOutput("at_home", false);
    }

    return BT::NodeStatus::SUCCESS;
}