from glob import glob
import os

from setuptools import setup

package_name = 'csen934_project_pkg'

# Each node is registered twice: the plain name (ROS 2 convention) and the
# ".py" name used verbatim in the milestone's Testing section, so both
# `ros2 run csen934_project_pkg control` and
# `ros2 run csen934_project_pkg control.py` work.
_nodes = {
    'control': 'csen934_project_pkg.control:main',
    'localization_add_noise': 'csen934_project_pkg.localization_add_noise:main',
    'localization_ekf_node': 'csen934_project_pkg.localization_ekf_node:main',
    'visualization_helper': 'csen934_project_pkg.visualization_helper:main',
    'analyze_bag': 'csen934_project_pkg.analyze_bag:main',
}
console_scripts = []
for exe, target in _nodes.items():
    console_scripts.append(f'{exe} = {target}')
    console_scripts.append(f'{exe}.py = {target}')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='altac9275928@gmail.com',
    description='CSEN934 Milestone 01: odometry, control, noise, EKF and SLAM.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': console_scripts},
)
