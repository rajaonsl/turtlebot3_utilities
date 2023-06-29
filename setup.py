from setuptools import setup

package_name = 'turtlebot3_utilities'

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
    maintainer_email='lois.rajaonson@grenoble-inp.pro',
    description='Various utility nodes for turtlebot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_plot_node = turtlebot3_utilities.sensor_plotter_node:main",
            "range_logger_node = turtlebot3_utilities.range_logger_node:main"
        ],
    },
)
