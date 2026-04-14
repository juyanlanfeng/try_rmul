/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * 作者：Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"
#include <chrono>
#include <functional> // functional 用于函数对象和绑定
#include <limits> // limits 用于数值类型的极限值
#include <memory>
#include <string>
#include <thread>
#include <utility> // utility 用于通用工具函数 

#include "sensor_msgs/point_cloud2_iterator.hpp" // 引入点云迭代器工具，用于遍历点云数据
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" // 引入 TF2 传感器消息转换工具，用于坐标系转换
#include "tf2_ros/create_timer_ros.h" // 引入 TF2 ROS 定时器创建接口

namespace pointcloud_to_laserscan // 定义命名空间，将代码组织在 pointcloud_to_laserscan 下
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", ""); // 从参数服务器获取目标坐标系名称，默认为空字符串
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01); // 从参数服务器获取坐标变换的容差值（秒），默认 0.01 秒
  // TODO(hidmic)：根据执行器实际可达到的并发级别调整默认输入队列大小
  input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency())); // 从参数服务器获取输入队列大小，默认值为硬件并发线程数
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min()); // 从参数服务器获取点云有效高度的最小值，默认为负无穷
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max()); // 从参数服务器获取点云有效高度的最大值，默认为正无穷
  angle_min_ = this->declare_parameter("angle_min", -M_PI); // 从参数服务器获取激光扫描的最小角度（弧度），默认 -π
  angle_max_ = this->declare_parameter("angle_max", M_PI); // 从参数服务器获取激光扫描的最大角度（弧度），默认 π
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0); // 从参数服务器获取角度增量（弧度），默认 1 度（π/180）
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0); // 从参数服务器获取扫描时间间隔（秒），默认 1/30 秒
  range_min_ = this->declare_parameter("range_min", 0.0); // 从参数服务器获取最小测量范围（米），默认 0.0
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max()); // 从参数服务器获取最大测量范围（米），默认为正无穷
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0); // 从参数服务器获取无穷大替换值的容差，默认 1.0
  use_inf_ = this->declare_parameter("use_inf", true); // 从参数服务器获取是否使用无穷大表示无效测量，默认 true

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS()); // 创建激光扫描发布器，发布到 "scan" 话题，使用传感器数据的 QoS 配置

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
    tf2_->setCreateTimerInterface(timer_interface); // 创建 TF2 变换监听器，用于监听坐标变换
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface()); // 用于解决传感器频率与TF发布频率不符导致查询不到TF的问题，filter可以自动处理所有的消息
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1)); // 注册回调函数，当消息过滤器触发时调用 cloudCallback
  } else {
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1)); // 否则设置直接订阅点云数据，回调函数还是 cloudCallback
  }

  // 启动订阅监听器线程，用于动态管理订阅
  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false); // 设置存活标志为 false，通知监听线程退出
  subscription_listener_thread_.join(); // 等待订阅监听器线程结束
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context(); // 获取节点的上下文对象，用于检查 ROS2 是否运行
  const std::chrono::milliseconds timeout(100); // 设置等待图变化的超时时间为 100 毫秒
  while (rclcpp::ok(context) && alive_.load()) { // 当 ROS2 正常运行且节点存活时持续循环
    int subscription_count = pub_->get_subscription_count() + pub_->get_intra_process_subscription_count(); // 获取LaserScan的订阅者数量（包括进程内订阅）
    // 如果有LaserScan的订阅者
    if (subscription_count > 0) {
      // 如果当前没有订阅3D点云
      if (!sub_.getSubscriber()) {
        // 记录日志：有LaserScan订阅者，启动3D点云订阅
        RCLCPP_INFO(
          this->get_logger(),
          "Got a subscriber to laserscan, starting pointcloud subscriber");
        rclcpp::SensorDataQoS qos; // 创建传感器数据的 QoS 配置
        qos.keep_last(input_queue_size_); // 设置 QoS 保持最后 N 个消息
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile()); // 订阅 "cloud_in" 话题
      }
    } else if (sub_.getSubscriber()) {  // 如果没有LaserScan订阅者但有激活的3D点云订阅者
      // 记录日志：没有LaserScan订阅者，关闭3D点云订阅
      RCLCPP_INFO(
        this->get_logger(),
        "No subscribers to laserscan, shutting down pointcloud subscriber");
      sub_.unsubscribe(); // 取消3D点云订阅
    }
    rclcpp::Event::SharedPtr event = this->get_graph_event(); // 获取图事件对象，用于等待节点图的变化
    this->wait_for_graph_change(event, timeout); // 等待图变化，超时时间为 100 毫秒
  }
  // 如果ROS或者节点死了，那么循环结束
  sub_.unsubscribe(); // 线程退出前取消订阅
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>(); // 要输出的二维扫描结果
  scan_msg->header = cloud_msg->header; // 复制输入点云消息的头信息（时间戳、坐标系等）到激光扫描消息

  // 如果指定了目标坐标系，设置激光扫描消息的坐标系
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_; // 设置激光扫描的最小角度（弧度）
  scan_msg->angle_max = angle_max_; // 设置激光扫描的最大角度（弧度）
  scan_msg->angle_increment = angle_increment_; // 设置激光扫描的角度增量（弧度）
  scan_msg->time_increment = 0.0; // 设置时间增量为 0.0（单线激光雷达不需要时间增量）？
  scan_msg->scan_time = scan_time_; // 设置扫描时间间隔
  scan_msg->range_min = range_min_; // 设置最小测量范围
  scan_msg->range_max = range_max_; // 设置最大测量范围

  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment); // 计算激光扫描的射线数量（角度范围内的射线数）

  // 初始化：先用无效数据填充二维结果
  if (use_inf_) { // 判断配置中是否使用无穷大表示无效测量
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity()); // 使用无穷大填充所有射线
  } else { 
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_); // 使用最大范围 + 容差填充所有射线
  }

  // 如果需要坐标系转换且当前坐标系与目标坐标系不同(如果指定了目标坐标系，检查目标坐标系与原来点云的坐标系是否相同，没有指定目标坐标系直接由原坐标系复制得出)
  // 那么先将点云变换到目标坐标系下再处理
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(); // 创建新的点云消息用于存储转换结果
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_)); // 使用 TF2 将点云转换到目标坐标系
      cloud_msg = cloud; // 更新 cloud_msg 指针指向转换后的点云
    } catch (tf2::TransformException & ex) { // 如果变换失败，记录错误日志并返回
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  // 遍历点云中的每个点，使用迭代器访问 x, y, z 坐标
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
    iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    // 如果点的坐标包含 NaN（无效值），跳过该点
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    // 如果点的 Z 坐标超出高度范围，跳过该点
    if (*iter_z > max_height_ || *iter_z < min_height_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
      continue;
    }

    // 计算点到原点的水平距离（X-Y 平面上的距离）
    double range = hypot(*iter_x, *iter_y);
    // 如果距离小于最小测量范围，跳过该点
    if (range < range_min_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    // 如果距离大于最大测量范围，跳过该点
    if (range > range_max_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    // 计算点的角度（相对于 X 轴的方位角）
    double angle = atan2(*iter_y, *iter_x);
    // 如果角度超出激光扫描的角度范围，跳过该点
    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
      continue;
    }

    // 计算该点对应的激光扫描射线索引（总角度/角度步长）
    int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
    // 如果新计算的距离小于当前射线存储的距离，更新该射线的距离值（初始化的时候每个点的距离要么是无穷大，要么是最大测量距离，计算出的距离必须小于初始化的值才有效）
    // 计算结果不符合要求则这个点视为无效测量点
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
    }
  }
  // 发布生成的激光扫描消息
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

// 引入 ROS2 组件注册宏
#include "rclcpp_components/register_node_macro.hpp"

// 注册 PointCloudToLaserScanNode 为 ROS2 组件节点
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
