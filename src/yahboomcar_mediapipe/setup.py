from setuptools import find_packages, setup

package_name = 'yahboomcar_mediapipe'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '01_HandDetector = yahboomcar_mediapipe.01_HandDetector:main',
            '02_PoseDetector = yahboomcar_mediapipe.02_PoseDetector:main',
            '03_Holistic = yahboomcar_mediapipe.03_Holistic:main',
            '04_FaceMesh = yahboomcar_mediapipe.04_FaceMesh:main',
            '05_FaceDetection = yahboomcar_mediapipe.05_FaceDetection:main',
            '06_FaceLandmarks = yahboomcar_mediapipe.06_FaceLandmarks:main',
            '07_Objectron = yahboomcar_mediapipe.07_Objectron:main',
            '08_VirtualPaint = yahboomcar_mediapipe.08_VirtualPaint:main',
            '09_HandCtrl = yahboomcar_mediapipe.09_HandCtrl:main',
            '10_GestureRecognition = yahboomcar_mediapipe.10_GestureRecognition:main',
            '11_FindHand = yahboomcar_mediapipe.11_FindHand:main',
            '12_FingerTrajectory = yahboomcar_mediapipe.12_FingerTrajectory:main',
        ],
    },
)
