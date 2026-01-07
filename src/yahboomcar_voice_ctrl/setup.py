from setuptools import setup, find_packages
import os
from glob import glob
package_name = 'yahboomcar_voice_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  
    #packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'config'),glob(os.path.join('config','*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='13377528435@sina.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
        'colorTracker = yahboomcar_voice_ctrl.colorTracker:main',
        'qrTracker = yahboomcar_voice_ctrl.qrTracker:main',
        'monoTracker = yahboomcar_voice_ctrl.monoTracker:main',
        'faceTracker = yahboomcar_voice_ctrl.faceTracker:main',
        'apriltagTracker = yahboomcar_voice_ctrl.apriltagTracker:main',
        'gestureTracker = yahboomcar_voice_ctrl.gestureTracker:main',
        'poseTracker = yahboomcar_voice_ctrl.poseTracker:main',
        'colorFollow = yahboomcar_voice_ctrl.colorFollow:main',
        'faceFollow = yahboomcar_voice_ctrl.faceFollow:main',
        'qrFollow = yahboomcar_voice_ctrl.qrFollow:main',
        'monoFollow = yahboomcar_voice_ctrl.monoFollow:main',
        'gestureFollow = yahboomcar_voice_ctrl.gestureFollow:main',
        'poseFollow = yahboomcar_voice_ctrl.poseFollow:main',
        'apriltagFollow = yahboomcar_voice_ctrl.apriltagFollow:main',
        'follow_line = yahboomcar_voice_ctrl.follow_line:main',
        'colorSelect = yahboomcar_voice_ctrl.colorSelect:main',
        ],
    },
)
