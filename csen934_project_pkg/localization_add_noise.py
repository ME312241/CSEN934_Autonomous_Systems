"""Adds zero-mean Gaussian noise to the ground-truth odometry.

Subscribes : /Car_Odom_GT    (nav_msgs/Odometry, zero covariance)
Publishes  : /Car_Odom_Noisy (nav_msgs/Odometry, covariance updated)

Noise (1 sigma), per the milestone table:
    x, y          0.3 m
    yaw           1 deg
    linear speed  0.3 m/s   (applied to twist.linear.x, body frame)
    angular speed 1 deg/s   (applied to twist.angular.z)

Covariance layout (6x6 row-major, order x y z roll pitch yaw):
    pose.covariance[0], [7], [35]  += sigma_xy^2, sigma_xy^2, sigma_yaw^2
    twist.covariance[0], [35]      += sigma_v^2, sigma_w^2
"""

import math

from csen934_project_pkg.utils import (euler_to_quaternion,
                                       quaternion_to_euler, wrap_angle)
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node


class LocalizationAddNoiseNode(Node):

    def __init__(self):
        super().__init__('Localization_Add_Noise_Node')
        self.declare_parameter('sigma_xy', 0.3)            # m
        self.declare_parameter('sigma_yaw_deg', 1.0)       # deg
        self.declare_parameter('sigma_v', 0.3)             # m/s
        self.declare_parameter('sigma_w_deg', 1.0)         # deg/s
        self.declare_parameter('seed', -1)                 # <0 => random

        seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.s_xy = float(self.get_parameter('sigma_xy').value)
        self.s_yaw = math.radians(float(self.get_parameter('sigma_yaw_deg').value))
        self.s_v = float(self.get_parameter('sigma_v').value)
        self.s_w = math.radians(float(self.get_parameter('sigma_w_deg').value))

        self.pub = self.create_publisher(Odometry, '/Car_Odom_Noisy', 10)
        self.sub = self.create_subscription(Odometry, '/Car_Odom_GT', self.callback, 10)

    def callback(self, gt):
        out = Odometry()
        out.header = gt.header              # same stamp => exact GT matching later
        out.child_frame_id = gt.child_frame_id
        out.pose.pose.position.x = gt.pose.pose.position.x + self.rng.normal(0.0, self.s_xy)
        out.pose.pose.position.y = gt.pose.pose.position.y + self.rng.normal(0.0, self.s_xy)
        out.pose.pose.position.z = gt.pose.pose.position.z

        q = gt.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        yaw = wrap_angle(yaw + self.rng.normal(0.0, self.s_yaw))
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        out.twist.twist = gt.twist.twist
        out.twist.twist.linear.x = gt.twist.twist.linear.x + self.rng.normal(0.0, self.s_v)
        out.twist.twist.angular.z = gt.twist.twist.angular.z + self.rng.normal(0.0, self.s_w)

        pc = list(gt.pose.covariance)
        pc[0] += self.s_xy ** 2
        pc[7] += self.s_xy ** 2
        pc[35] += self.s_yaw ** 2
        out.pose.covariance = pc

        tc = list(gt.twist.covariance)
        tc[0] += self.s_v ** 2
        tc[35] += self.s_w ** 2
        out.twist.covariance = tc

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationAddNoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
