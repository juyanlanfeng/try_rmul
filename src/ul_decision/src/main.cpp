#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include <behaviortree_cpp_v3/action_node.h>

#include "decision/game_start.hpp"
#include "decision/update_status.hpp"
#include "decision/check_status.hpp"
#include "decision/get_to_position.hpp"
#include "decision/public_data.hpp"

#include <atomic> // 用于线程安全的标志
#include <vector>

bool at_home;   // 全局变量定义

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

    std::string path = "/home/bluemaple/learn_projects/try_rmul/src/ul_decision/param/decision_tree.xml";
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
