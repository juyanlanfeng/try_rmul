## 消息定义

### `sensor_msgs/msg/LaserScan.msg`定义

```txt
std_msgs/Header header # 包括1.扫描时间戳：通常为第一束激光发射时间或扫描中点时间（取决于驱动）2.激光雷达自身坐标系：如 laser_link、front_laser
float32 angle_min
float32 angle_max
float32 angle_increment # 相邻波束的角度间隔（弧度）
float32 time_increment # 相邻波束发射的时间间隔（秒）
float32 scan_time # 完整两帧扫描之间的时间间隔（秒）
float32 range_min
float32 range_max
float32[] ranges # 距离数组
float32[] intensities # 反射强度数组（可选）
```

#### 解释

`LaserScan`消息相当于一个数组，数组中存储每一个被测量到的点到原点的距离。相邻两条线之间的夹角是固定不变的，
所以只需要根据索引就可以计算任意一条线到起始位置(第一条线)的夹角，即`angle = angle_min + n * angle_increment`

可以理解为用**极坐标**表示的点云，即通过`角度+距离`表示一个点的位置

### `sensor_msgs/msg/PointCloud2`定义

```txt
Header header # 时间戳和坐标系
uint32 height # 点云高度（行数）
uint32 width # 点云宽度（列数）
sensor_msgs/PointField[] fields # 定义每个点的数据结构
bool    is_bigendian # 是否大端字节序
uint32  point_step # 单个点的字节大小
uint32  row_step # 单行数据的字节大小(比如point_step × width)
bytes   data
bool    is_dense # 是否没有无效点(比如返回NaN的点)
```

#### 解释

`PointCloud2`同样可以理解为一个数组，但这个数组中存储了每一个测量点的x、y、z坐标，通过迭代器访问每一个点的坐标

可以理解为用**直角坐标系**表示的点云，即通过`x、y、z坐标`表示一个点的位置

## 转换过程

1. 创建一个空的2D点云，用**无穷大**或者**设定的最大测量距离+容差**填充所有的线
2. 遍历3D点云中每一个点的x、y、z坐标
3. 根据设定的测量距离和角度范围去除3D点云中不符合要求的点
4. "忽略"3D点云中每个点的z坐标，仅考虑x、y坐标(相当于将每个空间中的点投影到xoy平面)
5. 根据x、y坐标计算该点的角度，再由角度得出这个点在2D点云中对应的索引
6. 计算投影点到原点的距离，填充到对应的位置。如果有多个点对应同一个索引，则取最小距离