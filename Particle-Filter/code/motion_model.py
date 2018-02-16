import math
import random


class MotionModel:
    """
    Class that implements the odometry motion model of the robot
    """

    def __init__(self, distribution):
        """
        Implements motion model on the particles
        """
        self.alpha = [0.1, 0.6, 0.6, 0.1]
        self.prev_odom = [0, 0, 0]
        self.distribution = distribution

    def correct_orientation(self, angle):
        """
        Corrects the range to [0, 2pi]
        :param angle: Angle in [-pi,pi]
        :return: Angle in [0, 2pi]
        """
        return (2 * math.pi + angle) % (2 * math.pi)

    def run_odometry_model(self, odom_reading, prev_pose):
        """
        Propagates the particle based on the Odometry data
        :param odom_reading: Current Odometer Reading in (8000, 8000, [-pi,pi])
        :param prev_pose: Previous Pose/State of the Robot
        :return: New Pose/State of the Robot
        """
        odom = [odom_reading[0]//10, odom_reading[1]//10,
                self.correct_orientation(odom_reading[2])]

        # Tweaking because y is upside down
        del_rot1 = math.atan2(self.prev_odom[0]-odom[0],
                              odom[1]-self.prev_odom[1]) - self.prev_odom[2]
        del_rot1 = self.correct_orientation(del_rot1)

        del_trans = math.sqrt((self.prev_odom[0]-odom[0])**2 +
                              (odom[1]-self.prev_odom[1])**2)

        del_rot2 = odom[2] - self.prev_odom[2] - del_rot1
        del_rot2 = self.correct_orientation(del_rot2)

        del_rot1_hat = del_rot1 - self.sample(self.alpha[0] * del_rot1 ** 2 +
                                              self.alpha[1] * del_trans ** 2)
        del_rot1_hat = self.correct_orientation(del_rot1_hat)

        del_trans_hat = del_trans - self.sample(self.alpha[2] * del_trans ** 2 +
                                                self.alpha[3] * del_rot1 ** 2 +
                                                self.alpha[3] * del_rot2 ** 2)

        del_rot2_hat = del_rot2 - self.sample(self.alpha[0] * del_rot2 ** 2 +
                                              self.alpha[1] * del_trans ** 2)
        del_rot2_hat = self.correct_orientation(del_rot2_hat)

        y = prev_pose[0] - del_trans_hat * \
                           math.sin(prev_pose[2] + del_rot1_hat)
        y = 799 * (y > 799) + 0 * (y < 0) + y * (0 <= y <= 799)

        x = prev_pose[1] + del_trans_hat * \
                           math.cos(prev_pose[2] + del_rot1_hat)
        x = 799 * (x > 799) + 0 * (x < 0) + x * (0 <= x <= 799)

        theta = prev_pose[2] + del_rot1_hat + del_rot2_hat
        theta = self.correct_orientation(theta)

        return [y, x, theta]

    def sample(self, var):
        """
        Samples from a distribution
        :param var: Variance
        :return: Returns a sample from the distribution
        """

        sd = var**0.5
        if self.distribution is "Normal":
            summation = 0.0
            for i in range(12):
                summation += random.uniform(-sd, sd)
            return summation/2

        elif self.distribution is "Triangular":
            return (random.uniform(-sd, sd) * random.uniform(-sd, sd)) * (6 ** 0.5) / 2


if __name__ == "__main__":
    model = MotionModel("Normal")
