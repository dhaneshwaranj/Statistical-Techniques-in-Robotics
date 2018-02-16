import numpy as np
import math
from read_map import MapImage


class MeasurementModel:
    """
    Class that implements measurement model of the sensor
    """

    def __init__(self, Map=None, distance_table=None):

        #print 'Setting up measurement model'
        self.z_hit = 60                                       # mixture coeff for gaussian
        self.z_short = 30                                     # mixture coeff for short
        self.z_max = 5                                       # misture coeff for max reading
        self.z_rand = 15                                      # misture coeff for random noise
        self.Map = Map
        self.var = 250                                           # variance of the gaussian
        self.lamb = 1                                           # lamda parameter for short
        self.max_range = 3000
        self.epsilon = 10                                        # epsilon for max range reading
        self.distance_table = distance_table

    def prob_hit(self, z, z_star):
        """
        Returns the probability of the reading coming from the true obstacle
        :param z: Reading
        :param z_star: True Reading obtained using ray casting
        :return: Probability of the reading coming from the true obstacle
        """
        if 0 < z <= self.max_range:
            gauss = lambda x: np.exp(-np.power(x - z_star, 2.) / (2 * self.var ** 2))\
                              / ((2 * math.pi) ** 0.5 * self.var)
            return gauss(z)

        else:
            return 0

    def prob_short(self, z, z_star):
        """
        Returns the probability of the reading coming from a random obstacle in front of the robot
        :param z: Reading
        :param z_star: True Reading obtained using ray casting
        :return: Probability of the reading coming from a random obstacle in front of the robot
        """
        if 0 < z <= z_star:
            short = lambda x: self.lamb * np.exp(-self.lamb * z)
            return short(z)

        else:
            return 0

    def prob_max(self, z):
        """
        Returns the probability of the reading coming from max range

        :param z: Reading
        :return: Probability of the reading coming from max range
        """

        if self.max_range-self.epsilon <= z <= self.max_range+self.epsilon:
            return 1/(2*self.epsilon)

        else:
            return 0

    def prob_rand(self, z):
        """
        Returns the probability of the reading coming from a random measurement

        :param z: Reading
        :return: Probability of the reading coming from a random measurement
        """

        if 0 < z <= self.max_range:
            return 1/self.max_range

        else:
            return 0

    def correct_orientation(self, angle):
        """
        Corrects the range to [0, 2pi]
        :param angle: Angle in [-pi,pi]
        :return: Angle in [0, 2pi]
        """
        return (360 + angle) % 360

    def measurement_probability(self, z, x):
        """
        Returns the probability of the reading fromm the current location

        :param z: Reading
        :param x: Position, in range (800, 800, [0,2pi])
        :return: Probability of the reading fromm the current location
        """
        # convert to readable angles
        pose = x[:]
        pose[2] = self.correct_orientation(int(math.degrees(pose[2])))

        # check for the position of particle in map if it is on obstacle
        if self.Map[pose[0]][pose[1]] < 0:
            return 10**-10

        q = 1
        for phi in range(0, 180, 10): # Take every 10th laser reading
            z_star = self.distance_table[pose[0]][pose[1]][(pose[2]-90+phi)//3]
            p = self.z_hit * self.prob_hit(z[phi], z_star) + self.z_short * self.prob_short(z[phi], z_star) + \
                self.z_max * self.prob_max(z[phi]) + self.z_rand * self.prob_rand(z[phi])
            q = q*math.exp(p)

        return math.log(q)


if __name__ == "__main__":
    GlobalMap = MapImage().map
    print len(GlobalMap)
    measurement_model = MeasurementModel(GlobalMap)
