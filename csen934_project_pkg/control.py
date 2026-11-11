"""Control node: publishes a constant AckermannDrive command on /Car_Ackrm.

Milestone spec: velocity = 10 m/s, steering = 0 rad (steady straight-line
motion). Both are exposed as parameters whose defaults are the spec values.
"""

from ackermann_msgs.msg import AckermannDrive
import rclpy
from rclpy.node import Node


class ControlNode(Node):

    def __init__(self):
        super().__init__('Control_Node')
        self.declare_parameter('velocity', 10.0)        # m/s
        self.declare_parameter('steering_angle', 0.0)   # rad
        self.declare_parameter('publish_rate', 20.0)    # Hz

        self.pub = self.create_publisher(AckermannDrive, '/Car_Ackrm', 10)
        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(
            'Publishing /Car_Ackrm: speed=%.2f m/s, steering=%.3f rad' % (
                self.get_parameter('velocity').value,
                self.get_parameter('steering_angle').value))

    def publish_command(self):
        msg = AckermannDrive()
        msg.speed = float(self.get_parameter('velocity').value)
        msg.steering_angle = float(self.get_parameter('steering_angle').value)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: stop the car on exit so it does not keep cruising.
        try:
            node.pub.publish(AckermannDrive())
        except Exception:  # context may already be shut down by SIGINT
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
