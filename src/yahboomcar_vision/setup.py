from setuptools import find_packages, setup

package_name = 'yahboomcar_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_qrcode = yahboomcar_vision.create_qrcode:main',
            'parse_qrcode = yahboomcar_vision.parse_qrcode:main',
            'detect_pose = yahboomcar_vision.detect_pose:main',
            'detect_object = yahboomcar_vision.detect_object:main',
            'simple_ar = yahboomcar_vision.simple_ar:main',
            'astra_rgb_image = yahboomcar_vision.astra_rgb_image:main',
            'astra_depth_image = yahboomcar_vision.astra_depth_image:main',
            'pub_image = yahboomcar_vision.pub_image:main',
        ],
    },
)
