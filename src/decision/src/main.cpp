#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include <atomic> // 用于线程安全的标志
#include <vector>

std::vector<double> current_position = {-1.0, -1.0};

class GameStart : public BT::ConditionNode
{
public:
    GameStart(const std::string& condition_name, const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        // 创建订阅者，订阅 /game_start 话题
        gamestart_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/game_start", 10, std::bind(&GameStart::GameStart_callback, this, std::placeholders::_1));
    }

    void GameStart_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        setOutput("update_start", msg->data);
        RCLCPP_INFO(node_->get_logger(), "游戏开始");
    }

    static BT::PortsList providedPorts()
        {
            return
            {
                BT::InputPort<bool>("get_start"),
                BT::OutputPort<bool>("update_start")
            };
        }

    BT::NodeStatus tick()
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
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gamestart_sub_; // 游戏是否开始
};


class UpdateStatus : public BT::SyncActionNode
{
public:
    UpdateStatus(const std::string& action_name, const BT::NodeConfiguration& conf)
      : BT::SyncActionNode(action_name, conf) 
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        node_->declare_parameter<std::vector<double>>("add_blood", {0.0, 0.0});
        node_->declare_parameter<std::vector<double>>("pos1", {1.0, 1.0});
        node_->declare_parameter<std::vector<double>>("pos2", {2.0, 2.0});
        node_->declare_parameter<std::vector<double>>("pos3", {3.0, 3.0});
        node_->declare_parameter<std::vector<double>>("pos4", {4.0, 4.0});
        node_->declare_parameter<std::vector<double>>("pos5", {5.0, 5.0});
        node_->declare_parameter<std::vector<double>>("attack_pos", {0.0, 0.0});
        node_->declare_parameter<std::vector<double>>("idle_pos", {0.0, 0.0});

        node_->declare_parameter<double>("max_blood", 400.0);
        node_->declare_parameter<double>("min_blood", 200.0);

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

        cur_hp_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/cur_hp", 10, std::bind(&UpdateStatus::cur_hp_callback, this, std::placeholders::_1));
        ctlarea_status_sub_ = node_->create_subscription<std_msgs::msg::Int16>(
            "/ctlarea_status", 10, std::bind(&UpdateStatus::ctlarea_status_callback, this, std::placeholders::_1));
        enermy_observed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/enermy_observed", 10, std::bind(&UpdateStatus::enermy_observed_callback, this, std::placeholders::_1));
        enermy_onhighland_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/enermy_onhighland", 10, std::bind(&UpdateStatus::enermy_onhighland_callback, this, std::placeholders::_1));
    }

    void cur_hp_callback(std_msgs::msg::Float32 hp_msg) {
        cur_hp_.store(hp_msg.data);
        RCLCPP_INFO(node_->get_logger(), "生命值更改成功");
    }

    void ctlarea_status_callback(std_msgs::msg::Int16 ctl_msg) {
        ctlarea_status_.store(ctl_msg.data);
        RCLCPP_INFO(node_->get_logger(), "控制区状态更改成功");
    }
    
    void enermy_observed_callback(std_msgs::msg::Bool enermy_msg) {
        enermy_observed_.store(enermy_msg.data);
        RCLCPP_INFO(node_->get_logger(), "敌人状态更改成功");
    }

    void enermy_onhighland_callback(std_msgs::msg::Bool highland_msg) {
        enermy_onhighland_.store(highland_msg.data);
        RCLCPP_INFO(node_->get_logger(), "高地状态更改成功");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<bool>("hp_status"),
            BT::OutputPort<std::string>("ctlarea_status"),
            BT::OutputPort<bool>("enermy_observed"),
            BT::OutputPort<bool>("enermy_onhighland"),
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

    BT::NodeStatus tick() override
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
        
        if (cur_hp_.load() <= this->min_hp_) {
            setOutput("hp_status", true);
        }
        else {
            setOutput("hp_status", false);
        }

        if (enermy_observed_.load()) {
            setOutput("enermy_observed", true);
        }
        else {
            setOutput("enermy_observed", false);
        }

        if (enermy_onhighland_.load()) {
            setOutput("enermy_onhighland", true);
        }
        else {
            setOutput("enermy_onhighland", false);
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    std::vector<double> add_blood_, pos_1_, pos_2_, pos_3_, pos_4_, pos_5_, attack_pos_, idle_pos_;
    double min_hp_, max_hp_;
    std::atomic<bool> hp_status_;
    std::atomic<int> ctlarea_status_;
    std::atomic<bool> enermy_observed_;
    std::atomic<bool> enermy_onhighland_;
    std::atomic<double> cur_hp_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cur_hp_sub_; // 生命值
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr ctlarea_status_sub_; // 控制区状态
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enermy_observed_sub_; // 是否发现敌人
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enermy_onhighland_sub_; // 高地上是否有敌人
};


class CheckStatus : public BT::ConditionNode
{
public:
    CheckStatus(const std::string& condition_name, const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf) {}

    static BT::PortsList providedPorts()
        {
            return
            {
                BT::InputPort<bool>("get_status"),
            };
        }

    BT::NodeStatus tick()
    {
        bool status;
        getInput("get_status", status);
        if (status) {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return BT::NodeStatus::FAILURE;
        }
    }
};


class GetToPosition : public BT::StatefulActionNode
{
public:
    GetToPosition(const std::string& action_name, const BT::NodeConfiguration& conf)
      : BT::StatefulActionNode(action_name, conf) {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::vector<double>>("target_position"),
            BT::InputPort<std::vector<double>>("get_curpos"),
            BT::OutputPort<std::vector<double>>("update_curpos")
        }; 
    }

    BT::NodeStatus onStart() override
    {
        std::vector<double> temp_pos;
        getInput("target_position", temp_pos);

        target_pos_x_.store(temp_pos[0]);
        target_pos_y_.store(temp_pos[1]);
        RCLCPP_INFO(node_->get_logger(), "开始前往(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());
        
        current_position[0] = -1.0;
        current_position[1] = -1.0;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // 判断当前位置
        if (target_pos_x_.load() == current_position[0] && target_pos_y_.load() == current_position[1]) {
            RCLCPP_INFO(node_->get_logger(), "已经到达(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());
        }
        // 每50次tick输出一次，避免刷屏
        if (ticks % 50 == 0) {
            RCLCPP_INFO(node_->get_logger(), "正在前往(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());
        }
        ticks++;
        if (ticks <= 200) {
            return BT::NodeStatus::RUNNING;
        }
        else {
            ticks = 0;
            RCLCPP_INFO(node_->get_logger(), "已经到达(x:%f y:%f)", target_pos_x_.load(), target_pos_y_.load());
            current_position[0] = target_pos_x_.load();
            current_position[1] = target_pos_y_.load();
            return BT::NodeStatus::SUCCESS;
        }
    }

    void onHalted() override
    {
        RCLCPP_INFO(node_->get_logger(), "前往(x:%f y:%f)的进程被打断", target_pos_x_.load(), target_pos_y_.load());
    }
private:
    static int ticks; // 模拟耗时前往目标点
    rclcpp::Node::SharedPtr node_;

    std::atomic<double> target_pos_x_, target_pos_y_;
};
int GetToPosition::ticks = 0; // 类外初始化ticks为0


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_node");

    BT::BehaviorTreeFactory factory; // 创建行为树工厂
    factory.registerNodeType<GameStart>("GameStart"); // 注册自定义节点
    factory.registerNodeType<UpdateStatus>("UpdateStatus");
    factory.registerNodeType<CheckStatus>("CheckStatus");
    factory.registerNodeType<GetToPosition>("GetToPosition");

    BT::Blackboard::Ptr blackboard = BT::Blackboard::create(); // 创建黑板
    blackboard->set("node", node); // 在blackboard上添加ros2节点给bt节点使用
    blackboard->set("game_start", false);

    rclcpp::executors::MultiThreadedExecutor executor; // 多线程执行器：MultiThreadedExecutor，允许 ROS2 回调在多线程中并发执行
    executor.add_node(node); // 将节点添加到执行器中，以便处理该节点的订阅、服务、定时器等回调

    rclcpp::Rate loop_rate(50); // 50Hz运行行为树

    std::string path = "/home/bluemaple/try_rmul/src/decision/param/decision_tree.xml";
    auto tree = factory.createTreeFromFile(path, blackboard);
    RCLCPP_INFO(node->get_logger(), "决策树启动");

    while (rclcpp::ok())
    {
        executor.spin_some(); // 非阻塞地处理所有当前可用的 ROS 回调。这确保 ROS 消息能被及时接收，同时不会阻塞后续的行为树 tick
        tree.tickRoot(); // tick 开始
        loop_rate.sleep(); // 本次tick周期结束，休眠以维持循环频率
    }
    rclcpp::shutdown();
    return 0;
}
