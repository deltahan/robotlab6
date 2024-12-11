#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class HuskyHighlevelController:
    def __init__(self):
        rospy.init_node('husky_highlevel_controller', anonymous=True)
        self.p_ang = 1.0  # 转向控制比例增益
        self.fixed_velocity = 0.3  # 固定前进速度

        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.msg = Twist()

        # 目标位置和距离
        self.target_distance = None
        self.target_angle = None

        self.goal_reached = False  # 是否到达目标位置

    def set_vel(self, vel, dof):
        """
        设置速度。
        :param vel: 速度值
        :param dof: 自由度 ('forward' 或 'ang')
        """
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def publish_velocity(self):
        """
        发布速度命令。
        """
        self.vel_pub.publish(self.msg)

    def laser_callback(self, msg):
        """
        激光雷达回调函数。
        确定目标角度后，固定目标角度，仅根据该角度的距离值动态更新目标距离。
        """
        if not self.goal_reached and self.target_angle is None:
            # 过滤有效的雷达数据
            valid_ranges = [r for r in msg.ranges if r > 0]
            valid_indices = [i for i, r in enumerate(msg.ranges) if r > 0]

            if not valid_ranges or not valid_indices:
                rospy.logwarn("No valid points in scan data.")
                return

            # 检测突变点
            mutation_points = self.find_mutation_points(valid_ranges)

            if mutation_points:
                # 选择最近的突变点作为目标
                nearest_mutation_index = mutation_points[0]
                self.target_distance = valid_ranges[nearest_mutation_index]
                self.target_angle = msg.angle_min + valid_indices[nearest_mutation_index] * msg.angle_increment

                rospy.loginfo(f"Target fixed: Angle={math.degrees(self.target_angle):.2f}°, Distance={self.target_distance:.2f}m")
        elif self.target_angle is not None and not self.goal_reached:
            # 根据目标角度更新距离
            target_index = int((self.target_angle - msg.angle_min) / msg.angle_increment)
            if 0 <= target_index < len(msg.ranges):
                self.target_distance = msg.ranges[target_index]

                # 忽略无效数据
                if self.target_distance <= 0:
                    rospy.logwarn("Invalid range value at target angle.")
                    return

                # 前进控制逻辑
                self.move_to_target()

    def find_mutation_points(self, ranges):
        """
        查找突变点。
        :param ranges: 激光雷达距离数组
        :return: 突变点索引列表
        """
        mutation_points = []
        for i in range(1, len(ranges) - 1):
            if abs(ranges[i] - ranges[i - 1]) > 1:  # 差值是否大于1米
                mutation_points.append(i)
        return mutation_points

    def move_to_target(self):
        """
        朝目标点移动，直到距离为 0.17m 停止。
        """
        rate = rospy.Rate(10)  # 控制循环频率
        if self.target_distance > 0.17:
            # 调整方向
            self.set_vel(-self.p_ang * self.target_angle, "ang")
            # 前进
            self.set_vel(self.fixed_velocity, "forward")
            self.publish_velocity()

            rospy.loginfo(f"Moving to target: Angle={math.degrees(self.target_angle):.2f}°, Remaining Distance={self.target_distance:.2f}m")
        else:
            # 停止移动
            self.set_vel(0.0, "forward")
            self.set_vel(0.0, "ang")
            self.publish_velocity()
            rospy.loginfo("Reached the target. Stopping.")
            self.goal_reached = True


if __name__ == '__main__':
    controller = HuskyHighlevelController()
    rospy.spin()

