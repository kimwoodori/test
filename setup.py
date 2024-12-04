from setuptools import setup

package_name = 'yorizori'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yorizori_launch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Yorizori Robot Arm Control and Cooking Automation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_publisher_node = yorizori.command_publisher_node:main',
            'robot_control_node = yorizori.robot_control_node:main',
            'esp32_control_node = yorizori.esp32_control_node:main',
            'suction_cup_node = yorizori.suction_cup_node:main',  # 흡입컵 노드 추가
        ],
    },
)
