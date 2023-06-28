from setuptools import setup

package_name = 'turtlebot_utilities'

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
    maintainer='rajaonsl',
    maintainer_email='lois.rajaonson@grenoble-inp.fr',
    description='Various utility nodes for turtlebot3',
    license=' MIT License   |   Copyright (c) 2023 rajaonsl',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_plot_node = turtlebot_utilites.sensor_plotter_node:main",
            "range_logger_node = turtlebot_utilities.range_logger_node:main"
        ],
    },
)
