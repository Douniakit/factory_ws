from setuptools import setup

package_name = 'robot_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dounia',
    maintainer_email='your@email.com',
    description='Robot kinematics package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_monitor = robot_kinematics.joint_state_monitor:main',
            'kinematics_service = robot_kinematics.kinematics_service_node:main',
        ],
    },
)
