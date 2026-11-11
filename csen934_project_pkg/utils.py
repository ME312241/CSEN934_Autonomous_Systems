"""Small, dependency-free math helpers shared by the nodes (ZYX / REP-103)."""

import math


def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def euler_to_quaternion(roll, pitch, yaw):
    """Return (x, y, z, w) for intrinsic ZYX roll/pitch/yaw in radians."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_euler(x, y, z, w):
    """Return (roll, pitch, yaw) in radians from a unit quaternion."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, s) if abs(s) >= 1.0 else math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def yaw_from_msg_quaternion(q):
    """Yaw from a geometry_msgs/Quaternion-like object."""
    return quaternion_to_euler(q.x, q.y, q.z, q.w)[2]


def stamp_to_sec(stamp):
    """builtin_interfaces/Time -> float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_to_ns(stamp):
    """builtin_interfaces/Time -> integer nanoseconds (exact, hashable)."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec
