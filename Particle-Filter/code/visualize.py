import ray_casting
import cv2
import numpy as np
import os
from read_map import MapImage


class Visualize:
    """
    Class to visualize the particles
    """

    def __init__(self):
        self.Map = MapImage().map
        self.img = cv2.cvtColor(np.float32(self.Map), cv2.COLOR_GRAY2BGR)

        if os.path.isfile("distance_table.txt"):
            # Load distance table if already present
            self.distance_table = np.fromfile("distance_table.txt", dtype=np.int16).reshape([800, 800, 120])

        else:
            # Create distance table
            self.distance_table = ray_casting.RayCasting(self.Map).z_star_table
            self.distance_table.tofile("distance_table.txt")

    def visualize_particle(self, pose):
        """
        Draw the particle on the image wit ha circle.
        :param pose: Pose is in image coordinates (800, 800, [0,2pi]).
        """
        cv2.circle(self.img, (int(pose[1]), int(pose[0])), 1, (0, 0, 255), 0)

    def refresh_image(self):
        """
        Sets the image to be just the map image.
        """
        self.img = cv2.cvtColor(np.float32(self.Map), cv2.COLOR_GRAY2BGR)


if __name__ == '__main__':
    v = Visualize()
    v.visualize_particle([10, 600])
    cv2.imshow('map', v.img)
    cv2.waitKey(0)
