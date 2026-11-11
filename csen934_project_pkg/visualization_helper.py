"""Visualization_Helper_Node: makes the odometry streams plottable / viewable.

rqt_plot cannot plot yaw from a quaternion, so for each odometry stream this
node republishes a geometry_msgs/Pose2D (x, y, theta[rad]):
    /Car_Pose2D_GT, /Car_Pose2D_Noisy, /Car_Pose2D_Filtered
plus absolute errors against ground truth (matched on identical stamps):
    /Car_Error_Noisy, /Car_Error_Filtered          (Pose2D of |error|)
and nav_msgs/Path trajectories for RViz:
    /Car_Path_GT, /Car_Path_Noisy, /Car_Path_Filtered
    /Slam_Path   (from TF slam_frame -> base_frame, when a SLAM node runs)
Running RMSE (x, y, yaw) of noisy and filtered vs GT is logged periodically.

Optional: align_frame != '' publishes a static TF odom -> align_frame equal
to the first GT position, so a SLAM system that starts at (0,0,0) (LIO-SAM)
is drawn on top of the ground truth.
"""

from collections import OrderedDict
import math

from csen934_project_pkg.utils import (euler_to_quaternion, stamp_to_ns,
                                       wrap_angle, yaw_from_msg_quaternion)
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformException,
                     TransformListener)

STREAMS = {'GT': '/Car_Odom_GT', 'Noisy': '/Car_Odom_Noisy', 'Filtered': '/Car_Odom_Filtered'}


class VisualizationHelperNode(Node):

    def __init__(self):
        super().__init__('Visualization_Helper_Node')
        self.declare_parameter('path_min_distance', 0.5)    # m between path poses
        self.declare_parameter('path_max_length', 20000)
        self.declare_parameter('rmse_log_period', 5.0)      # s
        self.declare_parameter('slam_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('slam_path_rate', 5.0)       # Hz
        self.declare_parameter('align_frame', '')
        self.declare_parameter('align_yaw', False)

        self.min_d = float(self.get_parameter('path_min_distance').value)
        self.max_len = int(self.get_parameter('path_max_length').value)

        self.pose_pub, self.path_pub, self.paths = {}, {}, {}
        for name, topic in STREAMS.items():
            self.pose_pub[name] = self.create_publisher(Pose2D, f'/Car_Pose2D_{name}', 10)
            self.path_pub[name] = self.create_publisher(Path, f'/Car_Path_{name}', 10)
            self.paths[name] = Path()
            self.create_subscription(Odometry, topic,
                                     lambda m, n=name: self.odom_cb(n, m), 50)
        self.err_pub = {n: self.create_publisher(Pose2D, f'/Car_Error_{n}', 10)
                        for n in ('Noisy', 'Filtered')}

        self.gt_cache = OrderedDict()                 # stamp_ns -> (x, y, yaw)
        self.sq = {n: [0.0, 0.0, 0.0, 0] for n in ('Noisy', 'Filtered')}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.slam_path = Path()
        self.slam_pub = self.create_publisher(Path, '/Slam_Path', 10)
        self.create_timer(1.0 / float(self.get_parameter('slam_path_rate').value),
                          self.slam_path_cb)
        self.create_timer(float(self.get_parameter('rmse_log_period').value), self.log_rmse)

        self.static_tf = StaticTransformBroadcaster(self)
        self.aligned = False
        self.last_gt_ns = None

    # ---------------------------------------------------------------- reset
    def reset_on_time_jump(self):
        """Webots reset/reload: sim time went backwards, drop all history."""
        self.get_logger().warn('Simulation time jumped backwards: clearing paths, '
                               'metrics and TF buffer')
        self.tf_buffer.clear()
        for path in list(self.paths.values()) + [self.slam_path]:
            path.poses.clear()
        self.gt_cache.clear()
        self.sq = {n: [0.0, 0.0, 0.0, 0] for n in self.sq}
        self.aligned = False

    # ---------------------------------------------------------------- odom
    def odom_cb(self, name, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_msg_quaternion(msg.pose.pose.orientation)
        self.pose_pub[name].publish(Pose2D(x=p.x, y=p.y, theta=yaw))
        self._append(self.paths[name], self.path_pub[name], msg.header, msg.pose.pose)

        key = stamp_to_ns(msg.header.stamp)
        if name == 'GT':
            if self.last_gt_ns is not None and key < self.last_gt_ns - 500_000_000:
                self.reset_on_time_jump()
            self.last_gt_ns = key
            self.gt_cache[key] = (p.x, p.y, yaw)
            while len(self.gt_cache) > 5000:
                self.gt_cache.popitem(last=False)
            if not self.aligned and self.get_parameter('align_frame').value:
                self._publish_alignment(msg, yaw)
        elif key in self.gt_cache:
            gx, gy, gyaw = self.gt_cache[key]
            ex, ey, eyaw = p.x - gx, p.y - gy, wrap_angle(yaw - gyaw)
            self.err_pub[name].publish(Pose2D(x=abs(ex), y=abs(ey), theta=abs(eyaw)))
            s = self.sq[name]
            s[0] += ex * ex
            s[1] += ey * ey
            s[2] += eyaw * eyaw
            s[3] += 1

    def _append(self, path, pub, header, pose):
        if path.poses:
            last = path.poses[-1].pose.position
            if math.hypot(pose.position.x - last.x, pose.position.y - last.y) < self.min_d:
                return
        ps = PoseStamped()
        ps.header = header
        ps.pose = pose
        path.header = header
        path.poses.append(ps)
        if len(path.poses) > self.max_len:
            path.poses.pop(0)
        pub.publish(path)

    # ---------------------------------------------------------------- SLAM path
    def slam_path_cb(self):
        target = self.get_parameter('slam_frame').value
        base = self.get_parameter('base_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(target, base, Time(),
                                                 timeout=Duration(seconds=0.0))
        except TransformException:
            return
        ps = PoseStamped()
        ps.header = tf.header
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        self._append(self.slam_path, self.slam_pub, tf.header, ps.pose)

    # ---------------------------------------------------------------- alignment
    def _publish_alignment(self, gt, yaw):
        t = TransformStamped()
        t.header.stamp = gt.header.stamp
        t.header.frame_id = gt.header.frame_id or 'odom'
        t.child_frame_id = self.get_parameter('align_frame').value
        t.transform.translation.x = gt.pose.pose.position.x
        t.transform.translation.y = gt.pose.pose.position.y
        t.transform.translation.z = gt.pose.pose.position.z
        q = euler_to_quaternion(0.0, 0.0, yaw if self.get_parameter('align_yaw').value else 0.0)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.static_tf.sendTransform(t)
        self.aligned = True
        self.get_logger().info('Published alignment TF %s -> %s'
                               % (t.header.frame_id, t.child_frame_id))

    # ---------------------------------------------------------------- metrics
    def log_rmse(self):
        parts = []
        for n, (sx, sy, syaw, k) in self.sq.items():
            if k:
                parts.append('%s RMSE x=%.3f m y=%.3f m yaw=%.3f deg (n=%d)' % (
                    n, math.sqrt(sx / k), math.sqrt(sy / k),
                    math.degrees(math.sqrt(syaw / k)), k))
        if parts:
            self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationHelperNode()
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
