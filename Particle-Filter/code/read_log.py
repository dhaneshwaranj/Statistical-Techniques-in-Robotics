import math


class LogParser:
    """
    Gets all data in CENTIMETERS only and theta [0, 2pi]
    """

    def __init__(self, file_path):
        """
        :param file_path: File path for the log file
        """

        self.log_file = open(file_path, "r")
        self.laser_reading = None
        self.laser_center = None
        self.odom_reading = None
        self.time = None

    def parse_line(self, line):
        """
        :param line: One line in the log file
        :return: Parsed and cleaned data
        """

        if line[0] is 'L':
            line_list = line.split()
            self.odom_reading = line_list[1:4]
            # Convert angle range to [0, 2pi]
            self.odom_reading[-1] = self.correct_orientation(self.odom_reading[-1])
            self.laser_center = line_list[4:7]
            # Convert angle range to [0, 2pi]
            self.laser_center[-1] = self.correct_orientation(self.laser_center[-1])
            self.laser_reading = line_list[7:-1]
            self.time = line_list[-1]
            return ['L', self.odom_reading, self.laser_center, self.laser_reading,
                    self.time]

        elif line[0] is 'O':
            line_list = line.split()
            self.odom_reading = line_list[1:-1]
            # Convert angle range to [0, 2pi]
            self.odom_reading[-1] = self.correct_orientation(self.odom_reading[-1])
            self.time = line_list[-1]
            return ['O', self.odom_reading, self.time]

    def correct_orientation(self, angle):
        """
        Corrects the range to [0, 2pi]
        :param angle: Angle in [-pi,pi]
        :return: Angle in [0, 2pi]
        """
        return (2 * math.pi + angle) % (2 * math.pi)

if __name__ == "__main__":
    log_parser = LogParser("../data/log/robotdata1.log")
    for l in log_parser.log_file:
        log_parser.parse_file(line=l)
