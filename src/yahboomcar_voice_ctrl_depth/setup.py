from setuptools import find_packages, setup

package_name = 'yahboomcar_voice_ctrl_depth'

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
    maintainer='root',
    maintainer_email='977566262@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_get_dist = yahboomcar_voice_ctrl_depth.voice_get_dist:main',
            'colorSelect = yahboomcar_voice_ctrl_depth.colorSelect:main',
            'voice_face_follow = yahboomcar_voice_ctrl_depth.follow.voice_face_follow:main',
            'voice_colorTracker = yahboomcar_voice_ctrl_depth.follow.voice_colorTracker:main',
            'voice_apriltagFollow = yahboomcar_voice_ctrl_depth.follow.voice_apriltagFollow:main',
            'voice_qrFollow = yahboomcar_voice_ctrl_depth.follow.voice_qrFollow:main',
            'voice_follow_line = yahboomcar_voice_ctrl_depth.follow.voice_follow_line:main',
            'voice_gestureFollow = yahboomcar_voice_ctrl_depth.mediapipe.voice_gestureFollow:main',
            'voice_poseFollow = yahboomcar_voice_ctrl_depth.mediapipe.voice_poseFollow:main',
            'voice_KCF_Tracker = yahboomcar_voice_ctrl_depth.kcf.voice_KCF_Tracker:main',
        ],
    },
)
