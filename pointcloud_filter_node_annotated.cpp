#include "pointcloud_filter/pointcloud_filter_node.hpp"

#include <pcl_conversions/pcl_conversions.h>  // ROS消息与PCL点云之间的转换
#include <pcl/common/transforms.h>            // 点云变换
#include <pcl/filters/conditional_removal.h>  // 条件滤波
#include <pcl/filters/radius_outlier_removal.h> // 半径离群点移除
#include <pcl/filters/passthrough.h>          // 直通滤波

namespace hnurm {

    PointCloudNode::PointCloudNode(const rclcpp::NodeOptions& options)
        : Node("PointCloudNode", options)  // 调用基类构造函数，设置节点名为"PointCloudNode"
    {
        // 节点启动日志
        RCLCPP_INFO(get_logger(), "PointCloudNode is running");

        // 声明并初始化参数（从ROS参数服务器读取或使用默认值）
        // 参数说明：
        lidar_topic_        = this->declare_parameter("lidar_topic", "segmentation/obstacle");     // 输入点云话题，默认来自地面分割的障碍物点云
        odom_topic_         = this->declare_parameter("odom_topic", "/Odometry");                  // 里程计话题，用于获取传感器高度
        sensor_height_      = this->declare_parameter("sensor_height", 0.31);                      // 传感器安装高度（米）
        output_lidar_topic_ = this->declare_parameter("output_lidar_topic_", "/pointcloud");       // 输出点云话题
        output_laser_topic_ = this->declare_parameter("output_laser_topic_", "/ld_laserscan");     // 输出激光扫描话题
        base_frame_         = this->declare_parameter("base_frame", "base_footprint");             // 机器人基座坐标系
        lidar_frame_        = this->declare_parameter("lidar_frame", "lidar_link");                // 激光雷达坐标系
        laser_topic_        = this->declare_parameter("laser_topic", "/laser/scan");               // 输入激光扫描话题
        laser_frame_        = this->declare_parameter("laser_frame", "laser_link");                // 激光扫描坐标系
        robot_radius_       = this->declare_parameter("robot_radius", 0.5);                        // 机器人半径（米），用于过滤本体点
        filter_topic_       = this->declare_parameter("filter_topic", "/special_areas");           // 特殊区域订阅话题

        // 初始化TF监听器和缓冲区
        // TF用于坐标变换，特别是将特殊区域从map坐标系变换到传感器坐标系
        tf_buffer_          = std::make_unique<tf2_ros::Buffer>(get_clock());   // TF缓冲区，存储变换数据
        tf_listener_        = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);  // TF监听器，自动订阅TF话题

        // 定义可靠传输的 QoS 配置（用于激光扫描发布）
        // 可靠传输确保消息不丢失，适用于重要的传感器数据
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));  // 保持最后10条消息
        qos_profile.reliable();  // 明确设置为可靠传输，确保消息不丢失

        // 订阅点云数据
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_,  // 话题名称
            rclcpp::SensorDataQoS(),  // QoS策略：适合传感器数据
            std::bind(&PointCloudNode::pointcloud_callback, this, std::placeholders::_1)
        );

        // 订阅输入激光扫描数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_,  // 激光话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&PointCloudNode::laser_callback, this, std::placeholders::_1)  // 回调函数
        );

        // 订阅里程计数据（用于获取当前传感器高度）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,  // 里程计话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&PointCloudNode::odom_callback, this, std::placeholders::_1)  // 回调函数
        );

        // 订阅特殊区域消息（用于区域过滤）
        spa_sub_ = this->create_subscription<hnurm_interfaces::msg::SpecialArea>(
            filter_topic_,  // 特殊区域话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&PointCloudNode::spa_callback, this, std::placeholders::_1)  // 回调函数
        );

        // 订阅是否在特殊区域的标志（已弃用，保留接口兼容性）
        // 旧方法：通过布尔标志控制过滤，新方法直接使用SpecialArea消息
        in_special_area_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/is_in_special_area",  // 布尔标志话题
            rclcpp::SensorDataQoS(),  // 传感器数据QoS
            std::bind(&PointCloudNode::is_in_SpecialArea_callback, this, std::placeholders::_1)  // 回调函数
        );

        // 发布过滤后的3D点云数据
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_lidar_topic_,  // 输出点云话题
            rclcpp::SensorDataQoS()  // 传感器数据QoS
        );

        // 发布过滤后的2D激光扫描数据
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            output_laser_topic_,  // 输出激光话题
            qos_profile  // 使用可靠传输QoS
        );

        // 发布测试用激光数据（调试用）
        test_laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/test_laser",  // 测试激光话题
            qos_profile  // 可靠传输QoS
        );

        // 发布变换后的特殊区域（已转换到传感器坐标系，供调试或下游节点使用）
        tfed_spa_pub_ = this->create_publisher<hnurm_interfaces::msg::SpecialArea>(
            "/transformed_special_area",  // 变换后特殊区域话题
            rclcpp::SensorDataQoS()  // 传感器数据QoS
        );

    }

    void PointCloudNode::is_in_SpecialArea_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // use_filter = msg->data;     //old method,do not use
        // 旧方法：通过布尔标志控制过滤开关
        // 新方法：收到SpecialArea消息时自动启用过滤
        // 此回调函数被保留以保持接口兼容性，但实际不执行任何操作
    }

    void PointCloudNode::spa_callback(const hnurm_interfaces::msg::SpecialArea::SharedPtr msg)
    {
        current_spa_ = *msg; // 保存最新接收到的特殊区域
        use_filter = true;  // 接收到特殊区域后启用过滤模式
    }

    void PointCloudNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 从里程计中提取当前传感器高度（Z坐标）
        // 用于未来可能的Z方向过滤扩展
        current_sensor_height = msg->pose.pose.position.z;
    }

// 转换特殊区域的坐标系
    hnurm_interfaces::msg::SpecialArea PointCloudNode::transform_SpecialArea(const geometry_msgs::msg::TransformStamped& transform, const hnurm_interfaces::msg::SpecialArea& spa)
    {

        hnurm_interfaces::msg::SpecialArea transformed_spa_ = spa; // 创建变换后的特殊区域副本
        transformed_spa_.points.clear();  //clear pre points - 清空之前变换的点，准备填入新的数据

        // 创建Eigen变换矩阵（3D等距变换）
        Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();

        //translation matrix - 平移部分
        transform_matrix.translation() = Eigen::Vector3d(
            transform.transform.translation.x, // x:直接取TF中的x距离
            transform.transform.translation.y, // y:直接取TF中的y距离
            0.0  // Z方向设为0，因为SpecialArea是2D多边形
            // transform.transform.translation.z
        ); // 将平移变换填入变换矩阵

        // rotation matrix - 旋转部分
        Eigen::Quaterniond rotation(
            transform.transform.rotation.w,  // 四元数w分量
            transform.transform.rotation.x,  // 四元数x分量
            transform.transform.rotation.y,  // 四元数y分量
            transform.transform.rotation.z   // 四元数z分量
            // 四个分量都取自TF
        );
        transform_matrix.rotate(rotation.normalized());  // 将旋转变换填入变换矩阵
        
        //SpecialArea.points type : std::vector<ZoneEndPoint2D> polygon is closed
        for (auto sp : spa.points) // 遍历特殊区域的所有点，逐个进行坐标变换
        {
            Eigen::Vector3d point_3d(sp.x, sp.y, 0.0); // 将2D点扩展为3D点（Z=0）
            Eigen::Vector3d transformed_3d = transform_matrix * point_3d; // 应用变换矩阵：新坐标 = 变换矩阵 * 旧坐标
            hnurm_interfaces::msg::ZoneEndPoint2D transformed_point; // 创建变换后的2D点
            transformed_point.x = transformed_3d.x();  // 提取变换后的X坐标
            transformed_point.y = transformed_3d.y();  // 提取变换后的Y坐标
            transformed_spa_.points.push_back(transformed_point); // 添加到变换后的点列表
        }
        return transformed_spa_; // 返回变换后的特殊区域
    }

// 射线法判断点是否在多边形内（改进版本）
// 算法原理：从点向右发射水平射线，统计与多边形边的交点数量
// 奇数次相交点在多边形内，偶数次相交点在多边形外
    bool PointCloudNode::is_in_SpecialArea(float x, float y, const hnurm_interfaces::msg::SpecialArea& area) {
        bool inside = false;  // 默认点在特殊区域外
        const size_t n = area.points.size();  // 多边形顶点数量
        if (n < 3) return false;  // 少于3个点无法构成多边形，直接返回false
        // 遍历多边形的每条边（从顶点i到顶点j）
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const float xi = area.points[i].x, yi = area.points[i].y;  // 当前边终点坐标
            const float xj = area.points[j].x, yj = area.points[j].y;  // 当前边起点坐标
            
            // 检查点的Y坐标是否在边的Y坐标范围内（射线与边可能相交的条件）
            const bool y_in_range = (yi > y) != (yj > y);
            // y_in_range
            // true: yi 和 yj 一个大于 y 一个小于 y ，y 位于 yi 和 yj 之间
            // false: yi 和 yj要么都在 y 上面，要么都在 y 下面
            if (y_in_range) { // 只有该点的 y 坐标位于这条边两个顶点之间，该点向右发射射线才有可能与这条边相交
                const float dx = xj - xi, dy = yj - yi;  // 边的X和Y方向增量
                if (dy == 0) continue; // avoid divide zero 如果这条边水平，这个点还位于 yi 和 yj 之间，说明这个点在多边形的这条边上，不算区域内
                // 计算射线与边的交点X坐标
                const float t = (y - yi) / dy;  // 参数t：点到顶点1的y距离占顶点一和顶点二之间y距离的比值
                const float x_intersect = xi + t * dx;  // 顶点1 x 坐标 + 比值t * 顶点一和顶点二之间总的x距离
                if (x <= x_intersect) { // 如果点的X坐标小于等于交点X坐标（点在交点左侧），则向右的射线与边相交
                    inside = !inside;  // 切换内外状态（奇数次相交->内，偶数次相交->外）
                }
            }
        }
        return inside;  // 返回最终判断结果
    }

// 激光雷达数据回调函数
// 处理激光扫描数据，过滤机器人半径内和特殊区域内的点
    void PointCloudNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        hnurm_interfaces::msg::SpecialArea temp_tfed_spa_;  // 临时存储变换后的特殊区域
        if (use_filter)  // 如果启用了特殊区域过滤
        {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    msg->header.frame_id,  // target_frame: 点云坐标系
                    "map",                  // source_frame: 地图坐标系
                    rclcpp::Time(0),       // 使用最新可用变换（时间戳为0表示最新）
                    rclcpp::Duration::from_seconds(0.1)  // 超时时间：0.1秒
                ); // 查询map->点云坐标系的TF
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "tf failed: %s", ex.what());
                return; // TF查询失败，输出警告并跳过此帧的过滤
            }
            temp_tfed_spa_ = transform_SpecialArea(transform, current_spa_); // 将特殊区域从map坐标系变换到激光坐标系
        }

        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);  // 创建过滤后的扫描副本
        //debug test - 调试用（已注释）
        sensor_msgs::msg::LaserScan test_;
        // test_.header = msg->header;

        // 调整过滤后扫描的距离数组大小与原始一致
        const size_t num_points = msg->ranges.size();  // 激光扫描点数
        filtered_scan->ranges.resize(msg->ranges.size());  // 确保大小一致

        //angle pre-calculation - 预计算每个激光束的角度
        std::vector<float> angles(num_points);
        for (size_t i = 0; i < num_points; ++i) {
            angles[i] = msg->angle_min + i * msg->angle_increment; // 每个激光束的角度 = 起始角度 + 索引 * 角度增量
        }

        // #pragma omp parallel for - 并行化注释（可启用OpenMP并行计算）
        // 遍历所有激光束进行过滤
        for (size_t i = 0; i < num_points; ++i) {
            const float range = msg->ranges[i];  // 当前激光束的距离值
            // 检查距离值是否有效（不是无穷大且在最大范围内）
            if (!std::isfinite(range) || range > msg->range_max) { // 这里似乎多了一个 ！
                filtered_scan->ranges[i] = range;  // 无效值保持原样
                continue;  // 跳过后续处理
            }

            // calculate current point in the laser_link - 计算当前点在激光坐标系中的坐标
            // 极坐标转直角坐标：x = r * cos(θ), y = r * sin(θ)
            const float x = range * std::cos(angles[i]);
            const float y = range * std::sin(angles[i]);

            // 计算到车体中心的水平距离（欧几里得距离）
            const float distance = std::hypot(x, y); // 为什么不直接用range?

            //filter datas in special area - 在特殊区域内过滤数据

            if (use_filter)  // 如果启用了特殊区域过滤
            {
                // 判断点是否在特殊区域内或距离小于机器人半径
                if (is_in_SpecialArea(x, y, temp_tfed_spa_) || distance <= robot_radius_)
                {
                    // test_.ranges.push_back(filtered_scan->ranges[i]); - 调试用
                    filtered_scan->ranges[i] = std::numeric_limits<float>::infinity(); // remove - 设置为无穷大表示移除
                }
                else filtered_scan->ranges[i] = range;  // keep - 保留原始值
            }
            else if (distance <= robot_radius_) {  // 不考虑特殊区域
                filtered_scan->ranges[i] = std::numeric_limits<float>::infinity(); // remove - 设置为无穷大
            }
            else filtered_scan->ranges[i] = range;  // keep - 保留原始值
        }
        // 4. 发布过滤后的数据
        filtered_scan->header.frame_id = laser_frame_;  // 更新坐标系为激光坐标系
        laser_pub_->publish(*filtered_scan);  // 发布过滤后的激光扫描
        // test_laser_pub_->publish(test_); - 调试发布（已注释）
    }


// ============================================================
// 创建空点云函数（已注释，保留接口）
// ============================================================
    // sensor_msgs::msg::PointCloud2 PointCloudNode::create_empty_pointcloud(const sensor_msgs::msg::PointCloud2& msg )
    // {
    //     sensor_msgs::msg::PointCloud2 cloud;
    //     cloud.header =msg.heade;
    //     // cloud.data.clear();
    //     return cloud;
    // }

// 点云数据回调函数
// 处理点云数据，过滤机器人半径内和特殊区域内的点
    void PointCloudNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        hnurm_interfaces::msg::SpecialArea temp_tfed_spa_;  // 临时存储变换后的特殊区域
        if (use_filter)  // 如果启用了特殊区域过滤
        {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_->lookupTransform(
                    msg->header.frame_id,  // target_frame: 点云数据坐标系
                    "map",                  // source_frame: 地图坐标系
                    rclcpp::Time(0),       // 使用最新可用变换
                    rclcpp::Duration::from_seconds(0.1)  // 超时时间：0.1秒
                ); // 查询map->点云的TF
            }
            catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "tf failed: %s", ex.what());
                return; // TF查询失败，输出警告并跳过此帧的过滤
            }
            temp_tfed_spa_ = transform_SpecialArea(transform, current_spa_); // 将特殊区域从map坐标系变换到点云坐标系
            // transform:包含变换数据（平移，旋转）current_spa_:待变换的特殊区域
            tfed_spa_pub_->publish(temp_tfed_spa_); // 发布变换后的特殊区域
        }
        // 将ROS PointCloud2消息转换为PCL点云格式以便处理
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        //过滤车体半径内的点
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud; // 创建空白点云
        // 遍历原始点云中的每个点
        for (const auto& point : cloud) {
            // 计算点到车体中心的水平距离（忽略Z轴）
            const float distance = std::hypot(point.x, point.y);
            // 根据过滤模式进行处理
            if (use_filter)  // 特殊区域过滤模式
            {
                // 如果点在特殊区域内或距离小于机器人半径，则过滤掉
                if (is_in_SpecialArea(point.x, point.y, temp_tfed_spa_) || distance < robot_radius_) {
                    continue;  // 跳过此点，不添加到过滤后点云
                }
                else filtered_cloud.push_back(point);  // 否则保留此点
            }
            else if (distance > robot_radius_) {  // 不考虑特殊区域
                // 旧的高度过滤逻辑（已注释）
                // if (point.z<sensor_height_/2.0)
                // {
                //   continue;
                // }
                // else filtered_cloud.push_back(point);  //似乎没啥效果，先不做
                filtered_cloud.push_back(point);  // 距离大于机器人半径，保留此点
                // 距离小于机器人半径的会被忽略
            }
        }

        //发布过滤后的点云 - 将PCL点云转换回ROS消息
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(filtered_cloud, filtered_msg);
        filtered_msg.header.stamp = msg->header.stamp;  // 保持原始时间戳
        // filtered_msg.header.frame_id = base_frame_;  // 坐标系已转换到车体 - 旧代码（已注释）
        filtered_msg.header.frame_id = lidar_frame_;  // 使用激光雷达坐标系
        pointcloud_pub_->publish(filtered_msg);  // 发布过滤后的点云
    }

}  // namespace hnurm 结束
