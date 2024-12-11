#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

def lidar_callback(scan_data):
    """
    回调函数：过滤激光雷达数据，只保留非 inf 且非 0 的有效数据
    """
    # 初始化新的 ranges 和 intensities
    filtered_ranges = []
    filtered_intensities = []

    # 获取参数
    angle_min = scan_data.angle_min  # 起始角度（弧度）
    angle_increment = scan_data.angle_increment  # 每次扫描的角度增量（弧度）

    # 设置需要的角度范围（-30° 到 30°，转换为弧度）
    min_angle = math.radians(-60)  # -0.5236 弧度
    max_angle = math.radians(60)   # 0.5236 弧度

    # 遍历 ranges 数据
    for i in range(len(scan_data.ranges)):
        current_angle = angle_min + i * angle_increment

        # 判断是否在目标角度范围内
        if min_angle <= current_angle <= max_angle:
            # 只保留非 inf 且非 0 的数据
            value = scan_data.ranges[i]
            if value != float('inf') and value > 0:  # 删除 inf 和 0
                filtered_ranges.append(value)
                filtered_intensities.append(scan_data.intensities[i])

    # 如果没有有效数据，直接返回空的 LaserScan 消息
    if not filtered_ranges:
        rospy.logwarn("No valid data in the specified range.")
        return

    # 创建新的 LaserScan 消息
    filtered_scan = LaserScan()
    filtered_scan.header = scan_data.header
    filtered_scan.angle_min = min_angle
    filtered_scan.angle_max = max_angle
    filtered_scan.angle_increment = angle_increment
    filtered_scan.time_increment = scan_data.time_increment
    filtered_scan.scan_time = scan_data.scan_time
    filtered_scan.range_min = scan_data.range_min
    filtered_scan.range_max = scan_data.range_max
    filtered_scan.ranges = filtered_ranges
    filtered_scan.intensities = filtered_intensities

    # 发布新的话题
    lidar_pub.publish(filtered_scan)

# 初始化 ROS 节点
rospy.init_node('lidar_filter_node')

# 发布新话题 /scan1
lidar_pub = rospy.Publisher('/scan1', LaserScan, queue_size=10)

# 订阅原始激光雷达数据 /scan
rospy.Subscriber('/scan', LaserScan, lidar_callback)

# 保持节点运行
rospy.spin()


