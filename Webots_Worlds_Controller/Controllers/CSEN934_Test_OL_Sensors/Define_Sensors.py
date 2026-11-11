"""Sensor definitions for the CSEN934 car (TeslaModel3 in Worlds/city.wbt).

Device names below match city.wbt. If a name is missing, the first device of
the right type is used instead, so the controller still works on other cars.

    Slot            Device           Name             Used for
    sensorsSlotCenter GPS            "gps"            position, world velocity  (= base_link)
    sensorsSlotTop  InertialUnit     "inertial unit"  roll / pitch / yaw
    sensorsSlotTop  Gyro             "gyro"           body angular velocity
    sensorsSlotTop  Accelerometer    "accelerometer"  specific force (LIO-SAM IMU)
    sensorsSlotTop  Lidar            "lidar"          16 x 1800, 360 deg, 10 Hz (SLAM)
"""

from controller import Node
import numpy as np

SENSOR_NAMES = {
    'gps': 'gps',
    'imu': 'inertial unit',
    'gyro': 'gyro',
    'accelerometer': 'accelerometer',
    'lidar': 'lidar',
}
SENSOR_TYPES = {
    'gps': Node.GPS,
    'imu': Node.INERTIAL_UNIT,
    'gyro': Node.GYRO,
    'accelerometer': Node.ACCELEROMETER,
    'lidar': Node.LIDAR,
}


def _find(robot, name, node_type):
    by_type = None
    for i in range(robot.getNumberOfDevices()):
        dev = robot.getDeviceByIndex(i)
        if dev.getName() == name:
            return dev
        if by_type is None and dev.getNodeType() == node_type:
            by_type = dev
    return by_type


class CarSensors:
    """Finds and enables all sensors of the car."""

    def __init__(self, robot, timestep, lidar_period_ms=100):
        self.robot = robot
        self.timestep = timestep
        # lidar rendering is expensive: sample it at ~10 Hz, a multiple of the step
        self.lidar_period_ms = max(timestep, int(round(lidar_period_ms / timestep)) * timestep)
        self.devices = {}
        for key, name in SENSOR_NAMES.items():
            dev = _find(robot, name, SENSOR_TYPES[key])
            if dev is not None:
                dev.enable(self.lidar_period_ms if key == 'lidar' else timestep)
            self.devices[key] = dev
        if self.lidar is not None:
            self.lidar.enablePointCloud()

    gps = property(lambda self: self.devices['gps'])
    imu = property(lambda self: self.devices['imu'])
    gyro = property(lambda self: self.devices['gyro'])
    accelerometer = property(lambda self: self.devices['accelerometer'])
    lidar = property(lambda self: self.devices['lidar'])

    def missing_required(self):
        return [k for k in ('gps', 'imu') if self.devices[k] is None]

    def describe(self):
        found = ', '.join('%s=%s' % (k, d.getName() if d else None)
                          for k, d in self.devices.items())
        if self.lidar is None:
            return found
        lid = self.lidar
        return found + (
            ' | lidar layers=%d horizontal_res=%d fov=%.1f deg vfov=%.1f deg '
            'range=[%.1f, %.1f] m' % (
                lid.getNumberOfLayers(), lid.getHorizontalResolution(),
                np.degrees(lid.getFov()), np.degrees(lid.getVerticalFov()),
                lid.getMinRange(), lid.getMaxRange()))

    def mount_relative_to_gps(self, key):
        """4x4 pose of a sensor in the GPS (base_link) frame, or None.

        Needs the robot's `supervisor` field TRUE (set in city.wbt) and Webots >= R2023a.
        """
        dev, gps = self.devices.get(key), self.gps
        if dev is None or gps is None:
            return None
        try:
            node = self.robot.getFromDevice(dev)
            ref = self.robot.getFromDevice(gps)
            return np.array(node.getPose(ref), dtype=float).reshape(4, 4)
        except Exception:
            return None
