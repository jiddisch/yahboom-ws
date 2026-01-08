from setuptools import setup
import os
from glob import glob

package_name = 'yahboomcar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
      
        (os.path.join('share','yahboomcar_description','rviz'),glob(os.path.join('rviz','*.rviz*'))),
        (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
        'Mcnamu_driver_M1	= yahboomcar_bringup.Mcnamu_driver_M1:main',
        'calibrate_angular_M1 = yahboomcar_bringup.calibrate_angular_M1:main',
        'calibrate_linear_M1 = yahboomcar_bringup.calibrate_linear_M1:main',
        'calibrate_akm_angle = yahboomcar_bringup.calibrate_akm_angle:main',
        'patrol_M1 = yahboomcar_bringup.patrol_M1:main',
        'M1_descri_Mcnamu_driver = yahboomcar_bringup.Mcnamu_driver_M1_descri:main',
        ],
    },
)
