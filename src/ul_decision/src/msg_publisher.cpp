#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include <string.h>
#include <chrono>

using namespace std::chrono_literals;

class PublishNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cur_hp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teammate_at_center_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr teammate_hp_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    double frequency_ = 0.5;

    int times = 0;
public:
    PublishNode() : Node("sensor_node") {
        // 创建发布者
        start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/game_start", 10);
        cur_hp_pub_ = this->create_publisher<std_msgs::msg::Float32>("/cur_hp", 10);
        teammate_at_center_sub_ = this->create_publisher<std_msgs::msg::Bool>("/teammate_at_center", 10);
        teammate_hp_sub_ = this->create_publisher<std_msgs::msg::Float32>("/center_teammate_hp", 10);

        // 创建定时器
        auto timer_period = std::chrono::duration<double>(1.0 / this->frequency_);
        timer_ = this->create_wall_timer(timer_period, std::bind(&PublishNode::timer_callback, this));
        
    }

    void timer_callback() {
        std_msgs::msg::Bool msg_start;
        std_msgs::msg::Float32 msg_hp;
        std_msgs::msg::Bool msg_mate_center;
        std_msgs::msg::Float32 msg_mate_hp;

        times++;

        if (times >= 2) {
            msg_start.data = true;
        }
        else {
            msg_start.data = false;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublishNode>();
    RCLCPP_INFO(node->get_logger(), "Sensor_node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
