from setuptools import find_packages, setup

package_name = 'face_detect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'face_detect': ['haarcascade_frontalface_default.xml']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@lubancat.local',
    description='YOLOv8-Face RKNN inference for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_detect_node = face_detect.face_detect_node:main',
            'face_viz_node   = face_detect.face_viz_node:main',
        ],
    },
)