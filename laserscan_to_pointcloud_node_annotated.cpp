/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Eurotec, Netherlands
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * 作者：Rein Appeldoorn
 */

#include "pointcloud_to_laserscan/laserscan_to_pointcloud_node.hpp" // 引入节点类的头文件，包含 LaserScanToPointCloudNode 类的定义
#include <chrono> // 引入标准库头文件：chrono 用于时间操作
#include <functional> // functional 用于函数对象和绑定
#include <limits> // limits 用于数值类型的极限值
#include <memory> // memory 用于智能指针 
#include <string> // string 用于字符串处理
#include <thread> // thread 用于多线程支持
#include <utility> // utility 用于通用工具函数

#include "sensor_msgs/point_cloud2_iterator.hpp" // 引入点云迭代器工具，用于遍历和修改点云数据
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" // 引入 TF2 传感器消息转换工具，用于坐标系转换
#include "tf2_ros/create_timer_ros.h" // 引入 TF2 ROS 定时器创建接口

// 定义命名空间，将代码组织在 pointcloud_to_laserscan 下
namespace pointcloud_to_laserscan
{

// =================================================== 构造函数 =====================================================================
LaserScanToPointCloudNode::LaserScanToPointCloudNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("laserscan_to_pointcloud", options)  // 调用基类构造函数，设置节点名称
{
  target_frame_ = this->declare_parameter("target_frame", ""); // 从参数服务器获取目标坐标系名称，默认为空字符串
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01); // 从参数服务器获取坐标变换的容差值（秒），默认 0.01 秒
  // TODO(hidmic)：根据执行器实际可达到的并发级别调整默认输入队列大小
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency())); // 从参数服务器获取输入队列大小，默认值为硬件并发线程数

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS()); // 创建点云发布器，发布到 "cloud" 话题，使用传感器数据的 QoS 配置

  using std::placeholders::_1;
  // TF过滤的目的：激光雷达数据更新的速度与TF发布的速度不同，如果直接进行TF变换可能会出现这样的情况：
  // 激光雷达的数据已经到达了，需要进行TF变换，将点云转移至目标坐标系；但此时TF还未发布，查询不到TF，导致崩溃
  // TF过滤的作用就是，自动同步两个消息的时间，并且确保 数据 + 对应时刻的TF变换 全部就绪后才会触发callback，而不是普通订阅者一接收到数据就触发TF
  // 这也是为什么指定了点云目标坐标系才需要TF过滤，如果没有指定目标坐标系，就不会有TF变换，也就不需要过滤
  // 如果指定了点云目标坐标系，需要通过 TF 变换来过滤消息
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); // 创建 TF2 缓冲区，用于存储坐标变换数据
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface()); // 创建 TF2 ROS 定时器接口，用于在 ROS2 环境中管理定时器
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_); // 创建 TF2 变换监听器，用于监听坐标变换
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface()); // 创建消息过滤器，用于在消息到达时检查 TF 变换是否可用
    message_filter_->registerCallback(
      std::bind(
        &LaserScanToPointCloudNode::scanCallback, this, _1)); // 注册回调函数，当消息过滤器触发时调用 scanCallback
  } else {  // 否则设置直接订阅，不进行 TF 过滤
    // 注册直接回调函数
    sub_.registerCallback(std::bind(&LaserScanToPointCloudNode::scanCallback, this, _1));
  }

  // 启动订阅监听器线程，用于动态管理订阅
  subscription_listener_thread_ = std::thread(
    std::bind(&LaserScanToPointCloudNode::subscriptionListenerThreadLoop, this));
}

// =================================================================================================================================

// =================================================== 析构函数 =====================================================================
LaserScanToPointCloudNode::~LaserScanToPointCloudNode()
{
  alive_.store(true); // 设置存活标志为 true（注意：这里可能是 bug，应该是 false）
  subscription_listener_thread_.join(); // 等待订阅监听器线程结束
}

// ==================================================================================================================================

// =========================================== 订阅监听器线程的主循环函数 =============================================================
void LaserScanToPointCloudNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context(); // 获取节点的上下文对象，用于检查 ROS2 是否运行

  const std::chrono::milliseconds timeout(100); // 设置等待图变化的超时时间为 100 毫秒

  // 当 ROS2 正常运行且节点存活时持续循环
  while (rclcpp::ok(context) && alive_.load()) { 
    int subscription_count = pub_->get_subscription_count() +
      pub_->get_intra_process_subscription_count(); // 检查这个节点的发布者有多少个订阅者
    // 如果有订阅者
    if (subscription_count > 0) {
      // 如果当前没有订阅2D点云
      if (!sub_.getSubscriber()) {
        // 记录日志：有点云订阅者，启动2D点云订阅
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to pointcloud, starting laserscan subscriber");

        rclcpp::SensorDataQoS qos; // 创建传感器数据的 QoS 配置
        qos.keep_last(input_queue_size_); // 设置 QoS 保持最后 N 个消息

        sub_.subscribe(this, "scan_in", qos.get_rmw_qos_profile()); // 订阅 "scan_in" 话题
      }
    } else if (sub_.getSubscriber()) {  // 如果没有订阅者订阅节点输出的消息，但是节点还在订阅2D点云消息
      // 记录日志：没有点云订阅者，关闭2D点云订阅
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to pointcloud, shutting down laserscan subscriber");
      sub_.unsubscribe(); // 取消订阅
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event(); // 获取图事件对象，用于等待节点图的变化
    this->wait_for_graph_change(event, timeout); // 等待图变化，超时时间为 100 毫秒
  }
  // 如果ros或者节点死了，while循环退出
  sub_.unsubscribe(); // 线程退出前取消订阅
}

// ==================================================================================================================================

// =============================================== 激光雷达扫描消息的回调函数 =========================================================
void LaserScanToPointCloudNode::scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
  auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>(); // 创建新的3D点云消息对象

  // 投影器的功能：就是将极坐标转换为直角坐标
  projector_.projectLaser(*scan_msg, *cloud_msg); // 使用投影器将激光雷达扫描数据转换为点云

  // 如果需要转换坐标系且当前坐标系与目标坐标系不同，则将3D点云变换到目标坐标系
  if (!target_frame_.empty() && cloud_msg->header.frame_id != target_frame_) {
    try {
      *cloud_msg = tf2_->transform(*cloud_msg, target_frame_, tf2::durationFromSec(tolerance_)); // 使用 TF2 将点云转换到目标坐标系
    } catch (tf2::TransformException & ex) { // 变换失败，报错并返回
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }
  // 发布转换后的点云消息
  pub_->publish(std::move(cloud_msg));
}

// ==================================================================================================================================

}  // namespace pointcloud_to_laserscan

// 引入 ROS2 组件注册宏
#include "rclcpp_components/register_node_macro.hpp"

// 注册 LaserScanToPointCloudNode 为 ROS2 组件节点
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::LaserScanToPointCloudNode)
