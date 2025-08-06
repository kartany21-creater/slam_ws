from setuptools import setup

package_name = 'ydlidar_slam_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_with_toolbox.launch.py']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Launch file for YDLIDAR TG15 and SLAM Toolbox',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

