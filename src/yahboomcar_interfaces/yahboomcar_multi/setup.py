from setuptools import setup
import os
from glob import glob
package_name = 'yahboomcar_multi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch*'))),
        (os.path.join('share',package_name,'param'),glob(os.path.join('param','*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
        'yahboomcar_A1_ctrl = yahboomcar_multi.yahboom_A1_joy:main',
        'yahboomcar_M1_ctrl = yahboomcar_multi.yahboom_M1_joy:main',
        'get_follower_point = yahboomcar_multi.get_follower_point:main',
        'pub_follower_goal = yahboomcar_multi.pub_follower_goal:main'
        ],
    },
)
