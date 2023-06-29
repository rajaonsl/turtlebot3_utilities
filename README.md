# turtlebot3_utilities
 Various utility classes and ROS2 nodes for the turtlebot3 (and other range sensors)

*Author: L. Rajaonson | Supervisors: S. Gay, I. Prodan | Internship at LCIS (Valence, FR). 2023*
# Sensor Plotter
Easily visualize LiDAR measurements on a canvas. 

- `sensor_plotter_node` is a ROS2 node which listens on the default turtlebot3 topic `/scan`, and plots the measurements.

- `sensor_plotter` defines the underlying class (*SensorPlotter*) that is used for plotting measurements. It can be imported and used separately in your own programs to plot measurements.

# Range Logger
Save range measurements in a text file, and read them back. 

- The `range_logger_node` is a ROS2 node that listens to a turtlebot3's range sensor output, and writes measurements in a text file.

- `range_log_reader` defines a class (*RangeReader*) that you can import in your program to read a specific file.

# Usage

## Prerequisites

Plotting requires the standard Python library `tkinter`, which already comes with most Python installations (check with `python -m tkinter`)

Using the ROS nodes requires a working installation of ROS2 (preferably `humble` or `foxy`). If you don't have that, check the [wiki](https://github.com/rajaonsl/turtlebot3_utilities/wiki).

## Building ROS2 package (to run nodes)

Clone the repository into the source folder (src) of your desired ROS workspace. Then simply build the package using colcon:

    my_workspace$ colcon build --packages-select turtlebot3_utilities --symlink-install

Don't forget to source your workspace:

    my_workspace$ source ~/EXAMPLE/REPLACE_THIS_PATH/my_workspace/install/setup.bash

## Installing as a Python package (to import and reuse)

Without ROS, you can still import and reuse the classes defined here, install the repository as a package, using pip:

    turtlebot3_utilities$ pip install -e .
