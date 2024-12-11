#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class CombinedController:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('combined_controller', anonymous=True)

        # 定义 Action 客户端
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # 激光雷达和速度控制
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("/scan1", LaserScan, self.laser_callback)

        # 初始化参数
        self.msg = Twist()
        self.goal_reached = False
        self.target_distance = None
        self.target_angle = None
        self.p_ang = 1.0
        self.fixed_velocity = 0.3

    def send_goal(self, x, y, w):
        """
        发送导航目标点到 move_base
        """
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        rospy.loginfo(f"Sending goal: x={x}, y={y}, w={w}")
        self.client.send_goal(goal)

        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal reached!")
            return self.client.get_result()

    def laser_callback(self, msg):
        """
        激光雷达回调处理
        """
        valid_ranges = [r for r in msg.ranges if r > 0]
        valid_indices = [i for i, r in enumerate(msg.ranges) if r > 0]

        # 找到突变点
        mutation_points = self.find_mutation_points(valid_ranges)

        if mutation_points and not self.goal_reached:
            # 转向角度和移动距离
            target_angle, target_distance = self.calculate_target_angle_and_distance(
                mutation_points, valid_indices, msg
            )
            self.target_angle = target_angle
            self.target_distance = target_distance
            self.adjust_heading(target_angle)
            self.adjust_speed(target_distance - 0.15)
            self.publish_velocity()

            if self.target_distance <= 0.17:
                self.stop_robot()
                rospy.loginfo(f"已到达目标位置，目标距离：{self.target_distance}m")
                self.goal_reached = True

    def find_mutation_points(self, ranges):
        """
        检测突变点
        """
        mutation_points = []
        for i in range(1, len(ranges) - 1):
            if abs(ranges[i] - ranges[i - 1]) > 1:  # 突变点差值大于 1 米
                mutation_points.append(i)
        return mutation_points

    def calculate_target_angle_and_distance(self, mutation_points, valid_indices, msg):
        """
        计算目标角度和距离
        """
        angles = []
        distances = []

        for i in range(1, len(mutation_points)):
            idx1 = mutation_points[i - 1]
            idx2 = mutation_points[i]
            avg_angle = (msg.angle_min + idx1 * msg.angle_increment + msg.angle_min + idx2 * msg.angle_increment) / 2
            avg_distance = (msg.ranges[valid_indices[idx1]] + msg.ranges[valid_indices[idx2]]) / 2
            angles.append(avg_angle)
            distances.append(avg_distance)

        if angles and distances:
            return angles[0], distances[0]
        else:
            return 0.0, 0.0

    def adjust_speed(self, dist):
        """
        调整速度
        """
        vel = 0.0 if dist <= 0.17 else self.fixed_velocity
        self.set_vel(vel, "forward")

    def adjust_heading(self, ang):
        """
        调整方向
        """
        self.set_vel(-ang, "ang")

    def set_vel(self, vel, dof):
        """
        设置速度
        """
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def publish_velocity(self):
        """
        发布速度
        """
        self.vel_pub.publish(self.msg)

    def stop_robot(self):
        """
        停止机器人
        """
        self.set_vel(0.0, "forward")
        self.set_vel(0.0, "ang")
        self.publish_velocity()

    def execute(self):
        """
        执行主逻辑
        """
        # Step 1: 导航到目标点
        self.send_goal(0.485, 1.62, 1)

        # Step 2: 激光雷达控制
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.goal_reached:
            rate.sleep()

        # Step 3: 停留 2 秒
        self.stop_robot()
        rospy.loginfo("Robot stopped. Waiting for 2 seconds...")
        rospy.sleep(2)


if __name__ == '__main__':
    try:
        controller = CombinedController()
        controller.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

