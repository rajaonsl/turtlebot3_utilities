"""
NOT A ROS2 NODE
Class used for reading ranges from a log file

Ranges can be read one by one using the get_next method, or all at once
by accessing the 'ranges' attribute directly
===============================================================================
Example use:

from turtlebot3_utilites.range_log_reader import RangeReader


reader = RangeReader('/home/ubuntu/my_ranges.txt")

while reader.index < len(reader.ranges):
    range_data = reader.get_next()

    # ...do things with the data here

"""

# @TODO: make the class iterable

PATH = '/home/ubuntu/share/ranges.txt'
class RangeReader:
    """ A class used to read range measurement logs """
    def __init__(self, path: str = PATH):
        self.ranges = [] # All the measurements in one place
        self.index = 0

        with open(path, 'r', encoding='utf-8') as file:
            lines = file.readlines()
            for line in lines:
                if line == '\n':
                    continue
                ranges = [float(x) for x in line.split()]
                self.ranges.append(ranges)

    def get_next(self) -> list:
        """ Get the next measurement. If no more measurements, loops back to the first one"""
        if self.index >= len(self.ranges):
            self.index = 0 # loop back
        ranges = self.ranges[self.index]
        self.index += 1
        return ranges
