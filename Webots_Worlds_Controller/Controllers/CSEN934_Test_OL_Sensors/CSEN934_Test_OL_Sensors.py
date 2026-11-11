"""CSEN934 Milestone 01 - Webots controller of the car, hosting the ROS 2 node Tesla_Node.

Milestone interface
    Publishes  /Car_Odom_GT  nav_msgs/Odometry
               pose  : position from GPS, orientation from InertialUnit (roll, pitch, yaw)
               twist : linear velocity from GPS speed vector (rotated into the body frame),
                       angular velocity from Gyro; zero covariance (ground truth);
               published every simulation step (basicTimeStep)
    Subscribes /Car_Ackrm    ackermann_msgs/AckermannDrive
               speed [m/s] -> Car.setCruisingSpeed [km/h], steering_angle [rad]

Additional outputs used by the EKF / SLAM parts of the milestone
    /clock             rosgraph_msgs/Clock      simulation time
    /tf                odom -> base_link        ground truth (param publish_tf)
    /tf_static         base_link -> lidar_link  measured from the world via the supervisor API
    /Car_Imu           sensor_msgs/Imu          frame lidar_link (IMU co-located with the lidar)
    /Car_Lidar_Points  sensor_msgs/PointCloud2  x y z intensity ring time, 10 Hz

Frames: odom = Webots world (ENU); base_link = GPS = car origin (rear-axle centre),
x forward, y left, z up.
"""

import math

from ackermann_msgs.msg import AckermannDrive
from Define_Sensors import CarSensors
from geometry_msgs.msg import TransformStamped
from Helping_Functions import (euler_to_quaternion, lidar_to_points, make_pointcloud2,
                               rotation_matrix_to_euler, sim_time_to_stamp,
                               world_to_body, wrap_angle)
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    from vehicle import Car as Vehicle      # Car adds getWheelbase() etc.
except ImportError:                          # pragma: no cover
    from vehicle import Driver as Vehicle


class TeslaNode(Node):

    def __init__(self, car, timestep):
        super().__init__('Tesla_Node')
        self.car = car
        self.dt = timestep / 1000.0

        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('steering_sign', 1.0)         # +1: positive = left (REP 103)
        # used only if the supervisor API cannot measure the lidar mount
        self.declare_parameter('lidar_mount_xyz', [1.11, 0.0, 1.51])

        self.sensors = CarSensors(car, timestep)
        self.get_logger().info('Sensors: ' + self.sensors.describe())
        missing = self.sensors.missing_required()
        if missing:
            self.get_logger().error('Missing required sensors %s: /Car_Odom_GT disabled' % missing)
        try:
            self.get_logger().info('Wheelbase from Webots: %.3f m (set it in '
                                   'config/localization.yaml)' % car.getWheelbase())
        except AttributeError:
            pass

        self.odom_pub = self.create_publisher(Odometry, '/Car_Odom_GT', 10)
        self.imu_pub = self.create_publisher(Imu, '/Car_Imu', 50)
        self.cloud_pub = self.create_publisher(PointCloud2, '/Car_Lidar_Points', 5)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        self.create_subscription(AckermannDrive, '/Car_Ackrm', self.ackermann_cb, 10)

        self.prev_pos = None
        self.prev_yaw = None
        self.next_cloud_t = 0.0
        self.publish_lidar_mount()

    # ------------------------------------------------------------ static TF
    def publish_lidar_mount(self):
        if self.sensors.lidar is None:
            return
        T = self.sensors.mount_relative_to_gps('lidar')
        tf = TransformStamped()
        tf.header.frame_id = self.get_parameter('base_frame').value
        tf.child_frame_id = self.get_parameter('lidar_frame').value
        if T is not None:
            xyz = T[:3, 3]
            q = euler_to_quaternion(*rotation_matrix_to_euler(T[:3, :3]))
            src = 'measured (supervisor)'
        else:
            xyz = self.get_parameter('lidar_mount_xyz').value
            q = (0.0, 0.0, 0.0, 1.0)
            src = 'parameter lidar_mount_xyz'
        tf.transform.translation.x, tf.transform.translation.y, \
            tf.transform.translation.z = (float(v) for v in xyz)
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.static_tf_pub.sendTransform(tf)
        self.get_logger().info('base_link -> lidar_link = (%.3f, %.3f, %.3f) m, %s'
                               % (*(float(v) for v in xyz), src))

    # ------------------------------------------------------------- control
    def ackermann_cb(self, msg):
        self.car.setCruisingSpeed(float(msg.speed) * 3.6)       # m/s -> km/h
        sign = float(self.get_parameter('steering_sign').value)
        self.car.setSteeringAngle(sign * float(msg.steering_angle))

    # ------------------------------------------------------------ each step
    def step(self):
        sec, nsec = sim_time_to_stamp(self.car.getTime())
        if self.get_parameter('publish_clock').value:
            c = Clock()
            c.clock.sec, c.clock.nanosec = sec, nsec
            self.clock_pub.publish(c)

        s = self.sensors
        if s.missing_required():
            return
        pos = np.array(s.gps.getValues(), dtype=float)
        roll, pitch, yaw = s.imu.getRollPitchYaw()
        if not np.all(np.isfinite(pos)) or not all(map(math.isfinite, (roll, pitch, yaw))):
            return                                             # sensors not ready yet

        v_body = world_to_body(self.world_velocity(pos), roll, pitch, yaw)
        w_body = self.angular_velocity(yaw)
        self.prev_pos, self.prev_yaw = pos, yaw
        q = euler_to_quaternion(roll, pitch, yaw)

        od = Odometry()
        od.header.stamp.sec, od.header.stamp.nanosec = sec, nsec
        od.header.frame_id = self.get_parameter('odom_frame').value
        od.child_frame_id = self.get_parameter('base_frame').value
        p, o = od.pose.pose.position, od.pose.pose.orientation
        p.x, p.y, p.z = (float(v) for v in pos)
        o.x, o.y, o.z, o.w = q
        lin, ang = od.twist.twist.linear, od.twist.twist.angular
        lin.x, lin.y, lin.z = (float(v) for v in v_body)
        ang.x, ang.y, ang.z = (float(v) for v in w_body)
        self.odom_pub.publish(od)                              # covariance stays zero

        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header = od.header
            tf.child_frame_id = od.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y, \
                tf.transform.translation.z = p.x, p.y, p.z
            tf.transform.rotation = o
            self.tf_pub.sendTransform(tf)

        self.publish_imu(od.header.stamp, q, w_body)

        t = self.car.getTime()
        if s.lidar is not None and t + 1e-9 >= self.next_cloud_t:
            self.next_cloud_t = t + self.sensors.lidar_period_ms / 1000.0 - 1e-6
            x, y, z, ring, rng = lidar_to_points(s.lidar)
            self.cloud_pub.publish(make_pointcloud2(
                od.header.stamp, self.get_parameter('lidar_frame').value,
                x, y, z, ring, rng, s.lidar.getMinRange(), s.lidar.getMaxRange()))

    def world_velocity(self, pos):
        try:
            v = np.array(self.sensors.gps.getSpeedVector(), dtype=float)  # Webots >= R2023a
            if np.all(np.isfinite(v)):
                return v
        except AttributeError:
            pass
        return np.zeros(3) if self.prev_pos is None else (pos - self.prev_pos) / self.dt

    def angular_velocity(self, yaw):
        if self.sensors.gyro is not None:
            return np.array(self.sensors.gyro.getValues(), dtype=float)
        dyaw = 0.0 if self.prev_yaw is None else wrap_angle(yaw - self.prev_yaw)
        return np.array([0.0, 0.0, dyaw / self.dt])

    def publish_imu(self, stamp, q, w_body):
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = self.get_parameter('lidar_frame').value
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (
            float(v) for v in w_body)
        acc = self.sensors.accelerometer
        if acc is not None:
            a = acc.getValues()               # specific force: +9.81 on z when level
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (
                float(v) for v in a)
        else:
            m.linear_acceleration_covariance[0] = -1.0
        for cov in (m.orientation_covariance, m.angular_velocity_covariance):
            cov[0] = cov[4] = cov[8] = 1e-6
        self.imu_pub.publish(m)


def main():
    car = Vehicle()
    timestep = int(car.getBasicTimeStep())
    rclpy.init(args=None)
    node = TeslaNode(car, timestep)
    try:
        while car.step() != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.step()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
