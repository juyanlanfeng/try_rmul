/*
 * uart_node.cpp - UART串口通信节点（带详细注释版）
 * 
 * 此文件为原 uart_node.cpp 的带注释版本，保留了所有原始代码和注释，
 * 并添加了详细的逐行解释说明。
 * 
 * 功能概述：
 * 1. 作为ROS系统与嵌入式控制器（STM32）之间的串口通信桥梁
 * 2. 接收来自视觉处理节点的目标检测数据（VisionSendData）
 * 3. 发送控制指令到嵌入式控制器（云台角度、射击控制、底盘速度等）
 * 4. 接收嵌入式控制器发来的游戏状态、传感器数据、机器人状态等（VisionRecvData）
 * 5. 处理TF变换，发布云台坐标系到机器人基座坐标系的变换
 * 6. 支持分布式处理模式（多相机系统）
 * 7. 实现心跳检测、错误恢复和重连机制
 * 
 * 消息接口：
 * - 输入：vision_send_data (hnurm_interfaces/msg/VisionSendData) - 视觉节点发送的目标数据
 * - 输入：/decision/vision_send_data (hnurm_interfaces/msg/VisionSendData) - 决策节点发送的控制数据
 * - 输入：/cmd_vel_remap (geometry_msgs/msg/Twist) - 导航系统发送的速度指令
 * - 输入：/decision/spin_control (std_msgs/msg/Float32) - 小陀螺控制指令
 * - 输入：/decision/enable_180_scan (std_msgs/msg/Bool) - 启用180度扫描控制
 * - 输入：/decision/scan_center_angle (std_msgs/msg/Float32) - 扫描中心角度
 * - 输入：/decision/back_target_state (std_msgs/msg/Bool) - 后视目标状态
 * - 输入：/back_target (std_msgs/msg/Float32) - 回传目标指令
 * - 输入：/is_in_special_area (std_msgs/msg/Bool) - 是否在特殊区域标志
 * - 输出：vision_recv_data (hnurm_interfaces/msg/VisionRecvData) - 接收到的嵌入式系统数据
 * 
 * 串口通信协议：
 * - 自定义二进制协议，包含帧头、数据长度、CRC校验、命令ID、数据载荷和帧尾
 * - 协议头：0xA5
 * - 数据编码：将VisionSendData结构体编码为二进制流
 * - 数据解码：将二进制流解码为VisionRecvData结构体
 * 
 * 系统架构：
 * - 主线程：ROS回调处理、消息发布
 * - UART线程：串口数据收发、协议编解码、错误处理
 * - 支持多相机分布式处理：通过control_id字段区分不同相机数据
 */

#include "hnurm_uart/uart_node.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <angles/angles.h> // 角度处理库：提供角度归一化等工具函数

#include <filesystem> // 文件系统库：用于检查串口设备文件是否存在

using namespace std::chrono_literals; // 使用chrono字面量，简化时间单位书写（如1s表示1秒）

namespace hnurm
{
    void UartNode::run()
    {
        
        while (!std::filesystem::exists("/dev/serial/by-id/")) // 检查串口设备是否存在，确认串口正确连接后再执行别的
        {
            RCLCPP_WARN(logger, "Waiting for /dev/serial/by-id/ to be created");
            std::this_thread::sleep_for(1s);  // 每秒检查一次
        }

        recv_topic_ = this->declare_parameter("recv_topic", "vision_recv_data");  // 接收数据发布话题
        send_topic_ = this->declare_parameter("send_topic", "vision_send_data");  // 发送数据订阅话题

        // 控制ID相关参数：用于多相机系统数据分发
        use_control_id_ = this->declare_parameter("use_control_id", false);  // 是否使用control_id过滤
        control_id_ = static_cast<float>(this->declare_parameter("control_id", 1.0f));  // 将 declare_parameter 返回的值显式转换为 float 类型

        serial_codec_ = new SerialCodec(shared_from_this()); // 创建串口编解码器对象，负责串口通信和协议处理
        
        // 创建回调组：用于管理回调函数的执行顺序和并发
        callback_group1_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Reentrant：可重入，允许同一组的回调函数并行执行
        callback_group2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // MutuallyExclusive：互斥，同一组的回调函数串行执行

        // 分布式处理相关参数：用于多机器人或多相机系统
        // 分布式处理：
        /*
        通过命名空间（namespace）和数据标识字段（control_id）来区分不同物理设备的数据流，
        从而实现一个串口节点同时为多个上层视觉/决策节点服务，
        或在不同相机视角间切换控制权
        */
        use_distribution_ = this->declare_parameter("use_distribution", false);  // 是否启用分布式处理
        master_ns_ = this->declare_parameter("master_ns", "main");               // 主节点命名空间
        slave_ns_ = this->declare_parameter("slave_ns", "right");                // 从节点命名空间
        decision_send_topic_ = this->declare_parameter("decision_send_topic", "/decision/vision_send_data");  // 决策发送话题
        back_target_ = this->declare_parameter("back_target", "/back_target");  // 回传目标话题

        // 各种控制话题参数
        spin_control_topic_ = this->declare_parameter("spin_control_topic", "/decision/spin_control");           // 小陀螺控制
        scan_center_angle_topic_ = this->declare_parameter("scan_center_angle_topic", "/decision/scan_center_angle");  // 扫描中心角度
        back_target_state_topic_ = this->declare_parameter("back_target_state_topic", "/decision/back_target_state");  // 后视目标状态
        enable_scan_control_topic_ = this->declare_parameter("enable_scan_control_topic", "/decision/enable_180_scan"); // 启用扫描控制

        auto twist_topic = this->declare_parameter("twist_topic", "/cmd_vel_remap"); // 速度指令话题：接收导航系统的速度控制指令

        if (use_distribution_) // 如果启用分布式处理
        {
            master_pub_ = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
                recv_topic_, rclcpp::SensorDataQoS()); // 创建主发布者：发布接收到的数据
            // 默认的主线程不使用命名空间
            slave_pub = create_publisher<hnurm_interfaces::msg::VisionRecvData>(
                slave_ns_ + "/" + recv_topic_, rclcpp::SensorDataQoS()); // 创建从发布者：在从节点命名空间下发布数据
            // 从发布者的话题前会添加“slave_ns_/”，从而区分不同的数据流
            RCLCPP_INFO_STREAM(logger, "using distribution");
            RCLCPP_INFO_STREAM(logger, "master ns: " << master_ns_ << ", slave ns: " << slave_ns_);
        }
        else
        {
            master_pub_ = create_publisher<hnurm_interfaces::msg::VisionRecvData>(recv_topic_, rclcpp::SensorDataQoS());
            RCLCPP_INFO_STREAM(logger, "not using distribution"); // 非分布式模式：只创建一个发布者
        }

        // 配置订阅选项
        auto sub_option = rclcpp::SubscriptionOptions(); // 空的默认设置
        sub_option.callback_group = callback_group2_; // 设置：带有sub_option的回调函数，放在callback_group2_中执行

        // 订阅视觉发送数据：来自视觉处理节点的目标检测结果
        sub_ = create_subscription<hnurm_interfaces::msg::VisionSendData>(
            send_topic_,  // 话题名称
            rclcpp::SensorDataQoS(),  // QoS策略：传感器数据质量
            std::bind(&UartNode::sub_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );

        // 订阅决策发送数据：来自决策节点的控制指令
        decision_sub_ = create_subscription<hnurm_interfaces::msg::VisionSendData>(
            decision_send_topic_,  // 决策话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::decision_sub_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );

        // 订阅速度指令：来自导航系统的底盘速度控制
        sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
            twist_topic,  // 速度话题
            rclcpp::ServicesQoS(),  // QoS策略：服务级别质量
            std::bind(&UartNode::sub_twist_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );

        // 订阅回传目标指令：特殊控制指令（如666.0触发回传）
        back_target_sub_ = create_subscription<std_msgs::msg::Float32>(
            back_target_,  // 回传目标话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::back_target_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );
        
        // 订阅是否在特殊区域标志：影响发送策略
        in_special_area_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/is_in_special_area",  // 特殊区域话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::special_area_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );
        
        // 订阅小陀螺控制指令：控制机器人旋转模式
        spin_control_sub_ = create_subscription<std_msgs::msg::Float32>(
            spin_control_topic_,  // 小陀螺控制话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::spin_control_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );
        
        // 订阅扫描中心角度：用于180度扫描模式
        scan_center_angle_sub_ = create_subscription<std_msgs::msg::Float32>(
            scan_center_angle_topic_,  // 扫描中心角度话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::scan_center_angle_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );
        
        // 订阅后视目标状态：控制后视摄像头目标跟踪
        back_target_state_sub_ = create_subscription<std_msgs::msg::Bool>(
            back_target_state_topic_,  // 后视目标状态话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::back_target_state_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );
        
        // 订阅启用扫描控制标志：控制180度扫描模式开关
        enable_scan_control_sub_ = create_subscription<std_msgs::msg::Bool>(
            enable_scan_control_topic_,  // 启用扫描控制话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&UartNode::enable_scan_control_callback, shared_from_this(), std::placeholders::_1),  // 回调函数
            sub_option  // 放在callback_group2_中执行
        );

        // TF参数：用于云台坐标系的发布
        target_frame_ = this->declare_parameter("target_frame", "base_footprint");  // 目标坐标系（机器人基座）
        timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);      // 时间戳偏移量（补偿延迟）

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this()); // 创建TF广播器：动态发布云台坐标系变换
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this()); // 创建静态TF广播器：用于发布固定坐标系变换（如需要）

        set_mode_client_ = create_client<hnurm_interfaces::srv::SetMode>("/armor_detector/set_mode"); // 创建服务客户端：用于更新视觉检测模式（自瞄红/蓝、大小符等）

        heartbeat_publisher_ = fyt::HeartBeatPublisher::create(this); // 创建心跳检测发布者：监控节点健康状态

        // 创建独立线程处理串口通信，避免阻塞ROS主线程
        uart_thread_ = std::thread([this]() {
            // 线程主循环：持续处理串口数据
            while(rclcpp::ok() && !stop_flag_)  // 检查ROS系统状态和停止标志
            {
                timer_callback();  // 调用串口数据处理回调
            }
            RCLCPP_WARN(logger, "uart thread exit"); 
        });  // 线程退出日志
    }

// 处理特殊的回传指令（如666.0触发紧急回传）
    void UartNode::back_target_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 检查是否为有效的回传指令（666.0是特殊指令）
        if (msg->data != 666.0)
        {
            // 创建回传数据包
            hnurm_interfaces::msg::VisionSendData send_data;
            // 设置特殊速度值（2000表示特殊指令）
            send_data.vel_x = 2000.0;
            send_data.vel_y = 2000.0;
            send_data.vel_yaw = 2000.0;
            send_data.control_id = 50.0;  // 特殊control_id标识回传指令
            
            // 发送回传数据到串口
            if (serial_codec_->send_data(send_data))
                RCLCPP_INFO(logger, "send back target data");
        }
    }

// 处理来自决策节点的控制指令
    void UartNode::decision_sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        // 设置特殊速度值（2000表示决策指令，不需要底盘移动）
        msg->vel_x = 2000.0;
        msg->vel_y = 2000.0;
        msg->vel_yaw = 2000.0;
        msg->spin_ctrl = spin_control_value_;  // 添加小陀螺控制值
        
        // 缓存手势数据，用于后续的速度指令发送
        std::lock_guard<std::mutex> lock(decision_cache_mutex_);
        cached_gesture_ = msg->gesture.data;
        has_cached_gesture_ = true;
    }

// 更新机器人是否在特殊区域的标志
    void UartNode::special_area_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_in_special_area = msg->data;
    }

    /*
    hnurm_interfaces::msg::VisionSendData

    std_msgs/Header header

    TargetState target_state
    TargetType target_type
    Gesture gesture

    float32 pitch
    float32 yaw
    float32 pitch_diff
    float32 yaw_diff
    float32 target_distance
    float32 original_yout

    float32 vel_x
    float32 vel_y
    float32 vel_yaw

    # only compared to zero to determine the controller
    # control_id > 0 : right
    # control_id < 0 : left
    float32 control_id
    float32 spin_ctrl  # 1 顺 2 逆 3 stop
    */

    void UartNode::sub_callback(hnurm_interfaces::msg::VisionSendData::SharedPtr msg)
    {
        // PC通过串口向单片机发送消息时，是将云台角度，底盘速度等所有控制数据打包发出的，那么比如说如果只想改变云台的角度，不影响底盘的速度，就和下位机约定一个“无效数值”
        // 这个2000就是无效数值，下位机收到2000后，会直接忽略这一部分指令，从而保持相应状态不变
        msg->vel_x = 2000.0;
        msg->vel_y = 2000.0;
        msg->vel_yaw = 2000.0;
        msg->spin_ctrl = spin_control_value_;  // 添加小陀螺控制值
        // msg->spin_ctrl = 4;  // 测试值（已注释）
        
        denormalizeAngle(*msg); // 角度去归一化处理：将视觉坐标系的角度转换为云台实际角度
        
        // 调试日志（已注释）
        // RCLCPP_INFO(logger, "target state: %d, target type: %d, gesture: %d, pitch: %f, yaw: %f", msg->target_state.data, msg->target_type.data, msg->gesture.data, msg->pitch, msg->yaw);
        
        // 如果在特殊区域，则不发送视觉数据（避免干扰）
        if (is_in_special_area)
            return;
            
        // 发送数据到串口
        if (serial_codec_->send_data(*msg))
            ;  // 发送成功（空语句）
        // RCLCPP_INFO(logger, "send data");  // 调试日志（已注释）
    }

// 处理来自导航系统的底盘速度控制指令
    void UartNode::sub_twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 创建发送数据包
        hnurm_interfaces::msg::VisionSendData send_data;

        // 从缓存中获取手势数据（如果有的话）
        {
            std::lock_guard<std::mutex> lock(decision_cache_mutex_);
            if (has_cached_gesture_)
            {
                send_data.gesture.data = cached_gesture_;
            }
        }

        // 设置速度指令：来自导航系统的线速度和角速度
        send_data.vel_x = static_cast<float>(msg->linear.x);
        send_data.vel_y = static_cast<float>(msg->linear.y);
        send_data.spin_ctrl = spin_control_value_; // 发送小陀螺控制值
        // send_data.control_id = 25.0;  //测试（已注释）
        
        // 根据当前状态选择不同的控制策略
        if (is_in_special_area)
        {
            // 特殊区域模式：使用特定control_id和速度控制
            send_data.control_id = 25.0;
            send_data.vel_yaw = static_cast<float>(msg->linear.z);  // 使用z轴作为偏航角速度
        }
        else if (enable_scan_control_)
        {
            // 扫描控制模式：使用特定control_id和扫描中心角度
            send_data.control_id = 25.0;
            send_data.vel_yaw = static_cast<float>(scan_center_angle_);
        }
        
        // 发送数据到串口
        if (serial_codec_->send_data(send_data))
            ;  // 发送成功（空语句）
        // 调试日志（已注释）
        // RCLCPP_DEBUG(logger, "send fused data, gesture=%u, spin_ctrl=%f", send_data.gesture.data, send_data.spin_ctrl);
        // RCLCPP_INFO(logger, "send data");
    }

// 保存小陀螺控制值，用于后续数据发送
    void UartNode::spin_control_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 保存小陀螺控制值，在 sub_twist_callback 中统一发送给电控
        spin_control_value_ = msg->data;
    }

// 处理180度扫描模式的启用/禁用
    void UartNode::enable_scan_control_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 处理启用扫描控制的数据
        enable_scan_control_ = msg->data;
        // 根据需要将enable_scan_control转换为适当的格式并发送到串口
    }

// 设置180度扫描模式的中心角度
    void UartNode::scan_center_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 处理扫描中心角度数据
        scan_center_angle_ = msg->data;
        // 根据需要将scan_center_angle转换为适当的格式并发送到串口
    }

// 处理后视摄像头目标跟踪状态
    void UartNode::back_target_state_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 处理后视目标状态数据
        back_target_state_ = msg->data;
    }

// ============================================================
// timer_callback()：串口数据处理回调函数（UART线程主循环）
// 在独立线程中运行，负责接收和处理串口数据
// ============================================================
    int counter = 0;  // 调试计数器（已注释）
    void UartNode::timer_callback()
    {
        // TF变换消息
        geometry_msgs::msg::TransformStamped static_transform;
        geometry_msgs::msg::TransformStamped transformStamped;
        tf2::Quaternion q;  // 四元数，用于表示旋转
        
        // 接收数据缓冲区
        hnurm_interfaces::msg::VisionRecvData recv_data;
        
        // 尝试从串口获取数据（非阻塞，超时返回）
        if (serial_codec_->try_get_recv_data_for(recv_data))
        {
            // early return：如果自方颜色未设置，忽略此消息
            if (recv_data.self_color.data == hnurm_interfaces::msg::SelfColor::COLOR_NONE)
            {
                RCLCPP_WARN(logger, "self color not set, ignoring this msg");
                return;
            }

            // 坐标系对齐：将接收到的角度进行归一化处理
            // 嵌入式系统发送的角度可能是0-360度，需要归一化到-180~180度
            current_yaw_ = recv_data.yaw;  // 保存原始角度
            recv_data.yaw = angles::normalize_angle(recv_data.yaw * M_PI / 180.0) * 180.0 / M_PI;

            // 检测模式变化并调用服务：根据接收到的颜色和工作模式更新视觉检测模式
            checkAndUpdateMode(recv_data);

            // 分布式数据处理逻辑
            // if(use_distribution_)  // 简化判断（已注释）
            if (use_distribution_ && use_control_id_)
            {
                // use control id to distribute data：使用control_id分发数据到不同节点
                if (recv_data.control_id < 0)
                {
                    // 负control_id：发送到主节点
                    recv_data.header.stamp = now();      // 设置时间戳
                    recv_data.header.frame_id = "serial"; // 设置坐标系
                    master_pub_->publish(recv_data);     // 发布到主话题
                }
                else if (recv_data.control_id > 0)
                {
                    // 正control_id：发送到从节点
                    recv_data.header.stamp = now();
                    recv_data.header.frame_id = "serial";
                    slave_pub->publish(recv_data);       // 发布到从话题
                }
                else
                {
                    // control_id为0：非法值，忽略后续消息
                    RCLCPP_WARN_ONCE(logger, "control_id is illegal[0], ignoring further msg");
                }
            }
            else if (use_control_id_ && (control_id_ != recv_data.control_id))
            {
                // 控制ID过滤模式：如果control_id不匹配，忽略此消息
                return;
            }
            else
            {
                // 标准处理模式：发布数据并发布TF变换
                recv_data.header.stamp = now();
                recv_data.header.frame_id = "serial";
                
                // 调试模式：如果游戏未开始，设置默认值（测试用）
                if (recv_data.game_progress == 0.0 && recv_data.remain_time == 0.0 && recv_data.center_ctrl == 0.0)
                {
                    recv_data.current_hp = 400.0;
                    recv_data.allow_fire_amount = 750.0;
                }
                
                // 发布接收到的数据
                master_pub_->publish(recv_data);
                
                // 调试日志（已注释）
                // RCLCPP_INFO(this->get_logger(),"hp:%f ,enrmy outpost hp:%f,allow fire:%f",recv_data.current_hp,recv_data.current_enemy_outpost_hp,recv_data.allow_fire_amount);
                
                // 发布 tf 变换：云台坐标系到机器人基座坐标系的变换
                recv_data.pitch = -recv_data.pitch;  // 符号反转（坐标系定义差异）
                
                // 设置TF变换消息
                transformStamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                transformStamped.header.frame_id = target_frame_;  // 父坐标系：机器人基座
                transformStamped.child_frame_id = "gimbal_link";   // 子坐标系：云台
                transformStamped.transform.translation.x = 0.0;    // 平移：无偏移（云台在基座中心）
                transformStamped.transform.translation.y = 0.0;
                transformStamped.transform.translation.z = 0.0;

                // 使用接收到的 roll, pitch, yaw 值创建四元数
                // pitch 已经在前面取反：recv_data.pitch = -recv_data.pitch;
                auto roll_rad = recv_data.roll * M_PI / 180.0;    // 转换为弧度
                auto pitch_rad = recv_data.pitch * M_PI / 180.0;
                auto yaw_rad = recv_data.yaw * M_PI / 180.0;

                // 设置四元数（滚转-俯仰-偏航顺序）
                q.setRPY(roll_rad, pitch_rad, yaw_rad);
                transformStamped.transform.rotation = tf2::toMsg(q);
                
                // 发布TF变换
                tf_broadcaster_->sendTransform(transformStamped);
            }
            // 调试日志（已注释）
            // RCLCPP_WARN(logger, "publish to master, game_progress=%f, remaining_time=%f", recv_data.game_progress, recv_data.remain_time);
            // RCLCPP_INFO(logger, "recv data: %f, %f, %f", recv_data.pitch, recv_data.yaw, recv_data.roll);
        }
        else
        {
            // 串口数据接收失败处理
            if (error_cnt_++ > 100)  // 错误计数超过阈值
            {
                // 启动重连线程
                std::thread([this]()
                            { re_launch(); })  // 调用重连函数
                    .detach();  // 分离线程，异步执行
            }
            // 短暂休眠，避免CPU占用过高
            std::this_thread::sleep_for(10ms);
            RCLCPP_WARN(logger, "no data received from serial for %d times", error_cnt_);
        }
    }

// ============================================================
// re_launch()：串口重连函数
// 在串口通信失败时重新初始化串口连接
// ============================================================
    void UartNode::re_launch()
    {
        // 停止标志，通知UART线程退出
        stop_flag_ = true;
        uart_thread_.join();  // 等待UART线程结束
        stop_flag_ = false;   // 重置停止标志

        // 检查串口设备目录是否存在，等待系统创建
        while (!std::filesystem::exists("/dev/serial/by-id/"))
        {
            RCLCPP_WARN(logger, "Waiting for /dev/serial/by-id/ to be created");
            std::this_thread::sleep_for(1s);
        }

        // 重置错误计数
        error_cnt_ = 0;
        // 重新初始化串口端口
        serial_codec_->init_port();
        
        // 重新启动UART线程
        uart_thread_ = std::thread([this]()
                                   {
        while(rclcpp::ok() && !stop_flag_)
        {
            timer_callback();
        }
        RCLCPP_WARN(logger, "uart thread exit"); });
    }

// ============================================================
// checkAndUpdateMode()：检测并更新视觉模式函数
// 根据接收到的自方颜色和工作模式，更新视觉检测节点的模式
// ============================================================
    void UartNode::checkAndUpdateMode(const hnurm_interfaces::msg::VisionRecvData &recv_data)
    {
        // 提取自方颜色和工作模式
        uint8_t self_color = recv_data.self_color.data;
        uint8_t work_mode = recv_data.work_mode.data;
        // RCLCPP_INFO(logger, "Received self_color: %u, work_mode: %u", self_color, work_mode);  // 调试日志（已注释）
        
        uint8_t my_mode{0};  // 视觉检测模式编码

        // 模式编码逻辑：
        // 0:自瞄红, 1:自瞄蓝, 2:小符红, 3:小符蓝, 4:大符红, 5:大符蓝
        if (self_color == hnurm_interfaces::msg::SelfColor::BLUE)
        {
            // 自方为蓝色：检测红色目标
            if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_AIM)
            {
                my_mode = 0; // 自瞄红
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_SRUNE)
            {
                my_mode = 2; // 小符红
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_BRUNE)
            {
                my_mode = 4; // 大符红
            }
            else
            {
                my_mode = 0; // 默认值
            }
        }
        else if (self_color == hnurm_interfaces::msg::SelfColor::RED)
        {
            // 自方为红色：检测蓝色目标
            if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_AIM)
            {
                my_mode = 1; // 自瞄蓝
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_SRUNE)
            {
                my_mode = 3; // 小符蓝
            }
            else if (work_mode == hnurm_interfaces::msg::WorkMode::AUTO_BRUNE)
            {
                my_mode = 5; // 大符蓝
            }
            else
            {
                my_mode = 1; // 默认值
            }
        }
        else
        {
            my_mode = 0; // 默认值（颜色未知）
        }

        // 检测模式变化并调用服务
        if (my_mode != last_mode_)
        {
            // 等待服务可用
            if (!set_mode_client_->wait_for_service(1s))
            {
                RCLCPP_WARN(logger, "Service /armor_detector/set_mode not available, waiting...");
                return;
            }

            // 创建服务请求
            auto request = std::make_shared<hnurm_interfaces::srv::SetMode::Request>();
            request->mode = my_mode;

            // 异步调用服务
            set_mode_client_->async_send_request(
                request,
                [this, mode = my_mode](
                    rclcpp::Client<hnurm_interfaces::srv::SetMode>::SharedFuture future)
                {
                    try
                    {
                        auto response = future.get();
                        if (response->success)
                        {
                            this->last_mode_ = mode;  // 更新最后模式
                            RCLCPP_INFO(this->logger, "Updated mode to %d: %s", mode, response->message.c_str());
                        }
                        else
                        {
                            RCLCPP_ERROR(
                                this->logger, "Failed to update mode: %s (will retry)", response->message.c_str());
                        }
                    }
                    catch (const std::exception &e)
                    {
                        RCLCPP_ERROR(this->logger, "Service call exception: %s (will retry)", e.what());
                    }
                });
        }
    }

    // current_yaw_是从下位机获取的云台角度，它可能是一个累积值，比如750°（已经转了两圈多）
    // send_data.yaw是直接从视觉话题那里订阅来的，是“单圈归一化”的，其范围是(-180, +180), 比如30°（意思是在当前这一圈内，云台应该指向30°方向
    // 如果直接把30°发给下位机，下位机会以为要瞬间从750°跳回30°，导致它逆时针狂转两圈，也就是角度环绕问题
    void UartNode::denormalizeAngle(hnurm_interfaces::msg::VisionSendData &send_data)
    {
        double send_yaw = send_data.yaw;  // 视觉坐标系下的目标偏航角（归一化到-180~180）
        double recv_yaw = angles::normalize_angle(current_yaw_ * M_PI / 180.0) * 180.0 / M_PI;  // 将current_yaw_映射到[-pi, +pi]区间内

        // 计算圈数：计算current_yaw_中包含“几圈”，也就是几个360
        int circle_count = std::floor((current_yaw_ + 180.0) / 360.0); // 向下取整，这个+180似乎是一个习惯问题？

        // 计算两个角度差值
        double diff = send_yaw - recv_yaw;

        // 给目标角度加上正确的圈数，解决角度环绕问题
        if (std::abs(diff) <= 180.0)
        {
            send_data.yaw = send_yaw + circle_count * 360.0; // 角度差在180度内，直接使用当前圈数
        }
        else if (diff > 0)
        {
            send_data.yaw = send_yaw + (circle_count - 1) * 360.0; // 角度差大于180度，需要减小一圈
        }
        else
        {
            send_data.yaw = send_yaw + (circle_count + 1) * 360.0; // 角度差小于-180度，需要增加一圈
        }

        // 调试日志（已注释）
        // RCLCPP_INFO(this->logger, "send_yaw: %f, recv_yaw: %f, send_data.yaw: %f, current_yaw_: %f", send_yaw, recv_yaw, send_data.yaw,  current_yaw_);
    }

} // namespace hnurm 结束
