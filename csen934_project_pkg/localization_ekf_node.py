"""Localization_EKF_Node: fuses noisy odometry with the Ackermann command.

Subscribes : /Car_Odom_Noisy (nav_msgs/Odometry)       -> measurement update
             /Car_Ackrm      (ackermann_msgs/AckermannDrive) -> control input
Publishes  : /Car_Odom_Filtered (nav_msgs/Odometry)
             optional TF odom -> base_link (param publish_tf, default false,
             because Tesla_Node already publishes the ground-truth TF)

The filter itself lives in ekf_core.AckermannEKF (see that file for the model).
Time step dt is taken from message header stamps (simulation time), so the
filter is correct whether or not this node runs with use_sim_time.
"""

from ackermann_msgs.msg import AckermannDrive
from csen934_project_pkg.ekf_core import AckermannEKF
from csen934_project_pkg.utils import (euler_to_quaternion,
                                       quaternion_to_euler, stamp_to_sec)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# state index -> (covariance block, 6x6 index) for x, y, yaw | v, w
_POSE_IDX = [0, 1, 5]
_TWIST_IDX = [0, 5]


class LocalizationEKFNode(Node):

    def __init__(self):
        super().__init__('Localization_EKF_Node')
        self.declare_parameter('wheelbase', 2.94)               # m (Webots TeslaModel3)
        self.declare_parameter('velocity_time_constant', 1.0)   # s
        self.declare_parameter('process_noise', [0.02, 0.02, 0.0005, 2.0, 0.05])
        self.declare_parameter('max_dt', 1.0)                   # s, reset above this
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.ekf = AckermannEKF(
            wheelbase=self.get_parameter('wheelbase').value,
            tau=self.get_parameter('velocity_time_constant').value,
            q_diag=list(self.get_parameter('process_noise').value))
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.u = None
        self.t_prev = None

        self.pub = self.create_publisher(Odometry, '/Car_Odom_Filtered', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(AckermannDrive, '/Car_Ackrm', self.control_cb, 10)
        self.create_subscription(Odometry, '/Car_Odom_Noisy', self.odom_cb, 10)

    def control_cb(self, msg):
        self.u = (float(msg.speed), float(msg.steering_angle))

    @staticmethod
    def _measurement(msg):
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        z = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                      msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        pc = np.asarray(msg.pose.covariance).reshape(6, 6)
        tc = np.asarray(msg.twist.covariance).reshape(6, 6)
        R = np.zeros((5, 5))
        R[:3, :3] = pc[np.ix_(_POSE_IDX, _POSE_IDX)]
        R[3:, 3:] = tc[np.ix_(_TWIST_IDX, _TWIST_IDX)]
        return z, R, roll, pitch

    def odom_cb(self, msg):
        z, R, roll, pitch = self._measurement(msg)
        t = stamp_to_sec(msg.header.stamp)

        # Duplicate stamp (e.g. repeated sim step): skip, so /Car_Odom_Filtered stays
        # strictly increasing in time. Cartographer aborts on non-increasing odometry.
        if self.t_prev is not None and t == self.t_prev:
            return

        if not self.ekf.initialized or self.t_prev is None or \
                not (0.0 <= t - self.t_prev <= self.max_dt):
            if self.ekf.initialized:
                self.get_logger().warn('Time jump of %.3f s, re-initialising EKF'
                                       % (t - self.t_prev))
            self.ekf.initialize(z, R)
        else:
            self.ekf.predict(t - self.t_prev, self.u)
            self.ekf.update(z, R)
        self.t_prev = t
        self.publish(msg, roll, pitch)

    def publish(self, src, roll, pitch):
        x, y, yaw, v, w = self.ekf.x
        P = self.ekf.P
        out = Odometry()
        out.header.stamp = src.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.position.z = src.pose.pose.position.z
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = float(v)
        out.twist.twist.angular.z = float(w)

        pc = np.zeros((6, 6))
        pc[np.ix_(_POSE_IDX, _POSE_IDX)] = P[:3, :3]
        tc = np.zeros((6, 6))
        tc[np.ix_(_TWIST_IDX, _TWIST_IDX)] = P[3:, 3:]
        out.pose.covariance = pc.flatten().tolist()
        out.twist.covariance = tc.flatten().tolist()
        self.pub.publish(out)

        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header = out.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEKFNode()
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
