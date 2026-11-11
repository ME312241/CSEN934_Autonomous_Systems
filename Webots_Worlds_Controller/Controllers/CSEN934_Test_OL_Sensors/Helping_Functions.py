"""Helper math / message functions for the CSEN934 Webots controller.

Conventions: ENU world, FLU body (x forward, y left, z up), intrinsic ZYX
roll/pitch/yaw in radians, quaternions as (x, y, z, w) like ROS.
"""

import array
import ctypes
import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


# --------------------------------------------------------------------- angles
def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def euler_to_quaternion(roll, pitch, yaw):
    """Roll/pitch/yaw [rad] -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quaternion_to_euler(x, y, z, w):
    """Quaternion (x, y, z, w) -> roll, pitch, yaw [rad]."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, s) if abs(s) >= 1.0 else math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def euler_to_rotation_matrix(roll, pitch, yaw):
    """Body -> world rotation matrix R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def rotation_matrix_to_euler(R):
    """Inverse of euler_to_rotation_matrix."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def world_to_body(vec_world, roll, pitch, yaw):
    """Express a world-frame vector in the body frame."""
    return euler_to_rotation_matrix(roll, pitch, yaw).T @ np.asarray(vec_world, dtype=float)


# ----------------------------------------------------------------------- time
def sim_time_to_stamp(t):
    """Webots simulation time [s] -> (sec, nanosec) for builtin_interfaces/Time."""
    sec = int(t)
    return sec, int(round((t - sec) * 1e9)) % 1_000_000_000


# ---------------------------------------------------------------------- lidar
# Point layout compatible with LIO-SAM "velodyne" (fields are matched by name).
CLOUD_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24})
CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
    PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1),
]


# Webots WbLidarPoint struct: float x, y, z; int layer_id; float time
WB_POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                           ('layer', '<i4'), ('time', '<f4')])


def _webots_point_cloud(lidar):
    """Raw WbLidarPoint array straight from the C API (the Python wrapper's
    getPointCloud() builds one object per point, which is too slow at 10 Hz)."""
    from controller.wb import wb                       # Webots internal ctypes binding
    n = int(wb.wb_lidar_get_number_of_points(lidar._tag))
    ptr = wb.wb_lidar_get_point_cloud(lidar._tag)
    addr = ctypes.cast(ptr, ctypes.c_void_p).value
    if not n or not addr:
        raise RuntimeError('point cloud not available')
    return np.frombuffer(ctypes.string_at(addr, n * WB_POINT_DTYPE.itemsize),
                         dtype=WB_POINT_DTYPE)


def lidar_to_points(lidar):
    """Read a Webots lidar -> x, y, z, ring, range arrays in the lidar frame.

    Prefers Webots' own point cloud (exact axis conventions); falls back to
    rebuilding the points from the range image.
    """
    layers = lidar.getNumberOfLayers()
    try:
        wb_pts = _webots_point_cloud(lidar)
        x, y, z = (wb_pts[k].astype(np.float32) for k in ('x', 'y', 'z'))
        ring = (layers - 1 - wb_pts['layer']).clip(0, None)      # 0 = lowest beam
        return x, y, z, ring, np.sqrt(x * x + y * y + z * z)
    except Exception:
        return range_image_to_points(lidar.getRangeImage(), layers,
                                     lidar.getHorizontalResolution(),
                                     lidar.getFov(), lidar.getVerticalFov())


def range_image_to_points(ranges, layers, hres, fov, vfov):
    """Webots range image (top layer first, left to right) -> x, y, z, ring arrays.

    Ring 0 is the lowest beam (Velodyne convention).
    """
    r = np.asarray(ranges, dtype=np.float32).reshape(layers, hres)
    full = abs(fov - 2.0 * math.pi) < 1e-3
    step = fov / hres if full else fov / max(hres - 1, 1)
    az = fov / 2.0 - np.arange(hres) * step                       # +left ... -right
    if layers > 1:
        el = vfov / 2.0 - np.arange(layers) * vfov / (layers - 1)  # top ... bottom
    else:
        el = np.zeros(1)
    ce = np.cos(el)[:, None]
    x = r * ce * np.cos(az)[None, :]
    y = r * ce * np.sin(az)[None, :]
    z = r * np.sin(el)[:, None] * np.ones((1, hres))
    ring = np.repeat((layers - 1 - np.arange(layers))[:, None], hres, axis=1)
    return x.ravel(), y.ravel(), z.ravel(), ring.ravel(), r.ravel()


def make_pointcloud2(stamp, frame_id, x, y, z, ring, rng, min_range, max_range):
    """Build a dense sensor_msgs/PointCloud2 (invalid / out-of-range points removed)."""
    ok = np.isfinite(rng) & (rng > min_range) & (rng < max_range * 0.999)
    n = int(ok.sum())
    pts = np.zeros(n, dtype=CLOUD_DTYPE)
    pts['x'], pts['y'], pts['z'] = x[ok], y[ok], z[ok]
    pts['intensity'] = rng[ok]
    pts['ring'] = ring[ok].astype(np.uint16)
    pts['time'] = 0.0               # "fixed" Webots lidar: whole scan at one instant

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, n
    msg.fields = CLOUD_FIELDS
    msg.is_bigendian = False
    msg.point_step = CLOUD_DTYPE.itemsize
    msg.row_step = msg.point_step * n
    msg.is_dense = True             # LIO-SAM refuses non-dense clouds
    msg.data = array.array('B', pts.tobytes())
    return msg
