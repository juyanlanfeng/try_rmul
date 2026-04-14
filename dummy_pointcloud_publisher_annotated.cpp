/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Open Source Robotics Foundation, Inc.
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
 *     with the distribution.
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

#include <memory>
#include <random>


#include "rclcpp/rclcpp.hpp" // 引入 ROS2 核心头文件：rclcpp 用于节点创建和管理
#include "sensor_msgs/msg/point_cloud2.hpp" // 引入传感器消息类型：PointCloud2 用于点云数据
#include "sensor_msgs/point_cloud2_iterator.hpp" // 引入点云迭代器工具，用于遍历和修改点云数据

// 程序主函数，ROS2 节点的入口点
int main(int argc, char * argv[])
{
  // 初始化 ROS2 系统，解析命令行参数
  rclcpp::init(argc, argv);

  // 创建一个名为 "dummy_pointcloud_publisher" 的 ROS2 节点
  // 使用智能指针管理节点生命周期
  auto node = std::make_shared<rclcpp::Node>("dummy_pointcloud_publisher");
  // 创建一个发布器，用于发布 PointCloud2 类型的消息到 "cloud" 话题
  // rclcpp::SensorDataQoS() 设置适合传感器数据的 QoS 策略
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());

  // 创建一个空的 PointCloud2 消息对象，用于存储生成的点云数据
  sensor_msgs::msg::PointCloud2 dummy_cloud;
  // 创建点云修改器，用于设置点云的字段和大小
  sensor_msgs::PointCloud2Modifier modifier(dummy_cloud);
  // 设置点云包含 3 个字段：x, y, z，每个字段都是 32 位浮点数
  modifier.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  // 设置点云的大小（点数），从参数服务器获取，默认值为 100
  modifier.resize(node->declare_parameter("cloud_size", 100));
  // 创建随机数生成器，种子从参数服务器获取，默认值为 0
  std::mt19937 gen(node->declare_parameter("cloud_seed", 0));
  // 定义点云的分布范围，从参数服务器获取，默认值为 10.0
  double extent = node->declare_parameter("cloud_extent", 10.0);
  // 创建均匀分布对象，用于生成 -extent/2 到 extent/2 之间的随机浮点数
  std::uniform_real_distribution<float> distribution(-extent / 2, extent / 2);
  // 创建点云迭代器，分别指向 x, y, z 字段
  sensor_msgs::PointCloud2Iterator<float> it_x(dummy_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(dummy_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(dummy_cloud, "z");
  // 遍历点云中的每个点，使用迭代器访问和修改数据
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    // 为每个点的 x 坐标赋值为随机数
    *it_x = distribution(gen);
    // 为每个点的 y 坐标赋值为随机数
    *it_y = distribution(gen);
    // 为每个点的 z 坐标赋值为随机数
    *it_z = distribution(gen);
  }
  // 设置点云消息的坐标系 ID，从参数服务器获取，默认值为空字符串
  dummy_cloud.header.frame_id = node->declare_parameter("cloud_frame_id", "");

  // 创建单线程执行器，用于管理节点的回调和事件
  rclcpp::executors::SingleThreadedExecutor executor;
  // 将节点添加到执行器中
  executor.add_node(node);
  // 创建速率控制器，设置为 1.0 Hz（每秒 1 次）
  rclcpp::Rate rate(1.0);
  // 主循环，当 ROS2 系统正常运行时持续执行
  while (rclcpp::ok()) {
    // 更新点云消息的时间戳为当前时间
    dummy_cloud.header.stamp = node->get_clock()->now();
    // 发布点云消息
    pub->publish(dummy_cloud);
    // 处理节点的回调事件
    executor.spin_some();
    // 按照设定的速率休眠，控制发布频率
    rate.sleep();
  }

  // 关闭 ROS2 系统
  rclcpp::shutdown();
  // 程序正常退出
  return 0;
}
