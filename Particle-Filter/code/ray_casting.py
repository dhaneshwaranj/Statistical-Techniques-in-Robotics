import numpy as np
import math
from read_map import MapImage
import gc
import time


class RayCasting:
    """
    Class for doing Ray Casting
    """

    def __init__(self, Map=None, threshold=0.6):
        """
        :param Map: Map of the world
        :param threshold: Threshold for the obstacle
        """
        self.steps = range(0, int(Map.shape[0]*(2**0.5))*10, 9)
        self.z_star_table = -np.ones([Map.shape[0], Map.shape[1], 120], dtype=np.int16)
        self.Map = Map
        self.threshold = threshold
        print "Creating distance table."
        self.compute_z_star_table()
        print "Done creatinng distance table."

    def compute_z_star_table(self):
        """
        Computes the Z* value for each pose of the robot
        """

        for y in range(self.Map.shape[0]): # for every y
            print y

            for x in range(self.Map.shape[1]): # for every x

                if self.Map[y][x] >= self.threshold: # checking for open space

                    for pose_theta in range(0, 360, 3): # for pose theta steps

                        self.assign_z_star(y, x, pose_theta)

    def assign_z_star(self, y, x, pose_theta):
        """
        Stores the z_star value in the corresponding entry in the table

        :param y: y location
        :param x: x location
        :param pose_theta: theta orientation

        """

        for dist in self.steps:  # stepping through the line

            theta = math.radians(pose_theta)
            # (x_new, y_new) is the point on the line
            y_new = int(y - dist * math.sin(theta) // 10)
            x_new = int(x + dist * math.cos(theta) // 10)

            # if it's within the map
            if (0 <= y_new < self.Map.shape[0]) and (0 <= x_new < self.Map.shape[1]):

                # if it's not an unknown area
                if self.Map[y_new][x_new] >= 0:

                    # if it is an obstacle
                    if self.Map[y_new][x_new] <= self.threshold:
                        # set the distance travelled so far as the z_start value
                        self.z_star_table[y][x][pose_theta // 3] = dist
                        break
            else:
                self.z_star_table[y][x][pose_theta // 3] = dist
                break

    def query_z_star(self, y, x, theta, phi):
        """
        :param x: x location
        :param y: y location
        :param theta: theta orientation
        :param phi: range sensor degree
        :return: Z* value of that range sensor
        """

        return self.z_star_table[y][x][(theta-90+phi)//3]


if __name__ == '__main__':
    GlobalMap = MapImage().map
    threshold = 0.75
    gc.collect()
    start_time = time.time()
    z_star = RayCasting(Map=GlobalMap, threshold=threshold)
    print "Time elapsed : ", time.time() - start_time
