#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include <atomic> // 用于线程安全的标志
#include <mutex>
#include <string>
#include <behaviortree_cpp_v3/action_node.h>

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
        cur_hp_.store(400.0); // 初始状态为满血
        pre_hp_.store(400.0);
        teammate_status_.store(false); // 开始时控制区没有队友
        game_process_.store(true); // 默认上半场
        enermy_approach_.store(false); // 默认没有敌人

        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        cur_hp_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/cur_hp", 10, std::bind(&UpdateStatus::cur_hp_callback, this, std::placeholders::_1));
        teammate_at_center_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/teammate_at_center", 10, std::bind(&UpdateStatus::Teammate_callback, this, std::placeholders::_1));
        teammate_hp_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/center_teammate_hp", 10, std::bind(&UpdateStatus::Teammate_hp_callback, this, std::placeholders::_1));
        game_process_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/game_process", 10, std::bind(&UpdateStatus::game_process_callback, this, std::placeholders::_1));
        enermy_approach_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/enermy_approach", 10, std::bind(&UpdateStatus::enermy_approach_callback, this, std::placeholders::_1));
    }

    void cur_hp_callback(const std_msgs::msg::Float32::SharedPtr hp) {
        pre_hp_.store(cur_hp_.load());
        cur_hp_.store(hp->data);
        RCLCPP_INFO(node_->get_logger(), "hp更改成功");
    }

    void Teammate_callback(const std_msgs::msg::Bool::SharedPtr teammate) {
        teammate_status_.store(teammate->data);
        RCLCPP_INFO(node_->get_logger(), "队友状态更改成功");
    }

    void Teammate_hp_callback(const std_msgs::msg::Float32::SharedPtr mate_hp) {
        mate_cur_hp_.store(mate_hp->data);
        RCLCPP_INFO(node_->get_logger(), "控制区队友hp更改成功");
    }

    void game_process_callback(const std_msgs::msg::Float32::SharedPtr process) {
        if (process->data == 0.0) {
            game_process_.store(true);
        }
        else {
            game_process_.store(false);
        }
        RCLCPP_INFO(node_->get_logger(), "比赛进程更改成功");
    }

    void enermy_approach_callback(std_msgs::msg::Bool::SharedPtr enermy) {
        enermy_approach_.store(enermy->data);
        RCLCPP_INFO(node_->get_logger(), "敌人状态更改成功");
    }

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<std::string>("add_blood"),
            BT::OutputPort<std::string>("control_area"),
            BT::OutputPort<std::string>("protect_pos"),
            BT::OutputPort<std::string>("attack_pos"),
            BT::OutputPort<std::string>("idle_area"),
            BT::OutputPort<bool>("hp_status"),
            BT::OutputPort<bool>("teammate_status"),
            BT::OutputPort<bool>("mate_hp_status"),
            BT::OutputPort<bool>("is_attacked"),
            BT::OutputPort<bool>("game_process"),
            BT::OutputPort<bool>("enermy_approach")
        };
    }

    BT::NodeStatus tick() override
    {

        if (game_process_.load()) {
            this->danger_hp_ = 200.0;
            this->mate_danger_hp_ = 150.0;
        }
        else {
            this->danger_hp_ = 100.0;
            this->mate_danger_hp_ = 100.0;
        }

        setOutput("add_blood", "add_blood");
        setOutput("control_area", "control_area");
        setOutput("protect_pos", "protect_pos");
        setOutput("attack_pos", "attack_pos");
        setOutput("idle_area", "idle_area");

        if (cur_hp_.load() <= this->danger_hp_) {
            setOutput("hp_status", true);
        }
        else {
            setOutput("hp_status", false);
        }

        if (teammate_status_.load()) {
            setOutput("teammate_status", false);
        }
        else {
            setOutput("teammate_status", true);
        }

        if (mate_cur_hp_.load() <= this->mate_danger_hp_) {
            setOutput("mate_hp_status", true);
        }
        else {
            setOutput("mate_hp_status", false);
        }

        if (cur_hp_.load() - pre_hp_.load() < 0) {
            setOutput("is_attacked", true);
        }
        else {
            setOutput("is_attacked", false);
        }

        if (game_process_.load()) {
            setOutput("game_process", true);
        }
        else {
            setOutput("game_process", false);
        }

        if (enermy_approach_.load()) {
            setOutput("enermy_approach", true);
        }
        else {
            setOutput("enermy_approach", false);
        }

        return BT::NodeStatus::SUCCESS;
    }

private:
    double danger_hp_, mate_danger_hp_;

    rclcpp::Node::SharedPtr node_;
    std::atomic<double> cur_hp_;
    std::atomic<double> pre_hp_; // 用于检查是否扣血
    std::atomic<bool> teammate_status_;
    std::atomic<double> mate_cur_hp_;
    std::atomic<bool> game_process_;
    std::atomic<bool> enermy_approach_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cur_hp_sub_; // 生命值
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teammate_at_center_sub_; // 是否有队友在控制区(自己不算)
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr teammate_hp_sub_; // 占领控制区的队友hp
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr game_process_sub_; // 上半场 0 / 下半场 1
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enermy_approach_sub_; // 敌人是否接近
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
            BT::InputPort<std::string>("go_position")
        }; 
    }

    BT::NodeStatus onStart() override
    {
        std::string temp_pos;
        getInput("go_position", temp_pos);
        {
            std::lock_guard<std::mutex> lock(target_mutex);
            target_position = temp_pos;
        }
        RCLCPP_INFO(node_->get_logger(), "开始前往%s", target_position.c_str());
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // 每50次tick输出一次，避免刷屏
        if (ticks % 50 == 0) {
            std::string temp_pos;
            {
                std::lock_guard<std::mutex> lock(target_mutex);
                temp_pos = target_position;
            }
            RCLCPP_INFO(node_->get_logger(), "正在前往%s", temp_pos.c_str());
        }
        ticks++;
        if (ticks <= 200) {
            return BT::NodeStatus::RUNNING;
        }
        else {
            ticks = 0;
            return BT::NodeStatus::SUCCESS;
        }
    }

    void onHalted() override
    {
        std::string temp_pos;
        {
            std::lock_guard<std::mutex> lock(target_mutex);
            temp_pos = target_position;
        }
        std::cout << "前往" << temp_pos << "的进程被打断" << std::endl; // 待定
    }
private:
    static int ticks; // 模拟耗时前往目标点
    rclcpp::Node::SharedPtr node_;

    std::string target_position;          // 改为普通 std::string
    mutable std::mutex target_mutex;       // 互斥锁，mutable 允许在 const 成员函数中加锁
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
