import math
import random
import motion_model
import read_log
import visualize
import cv2
import copy
import measurement_model

log_file_path = "../data/log/robotdata1.log"
NUM_PARTICLES = 5000


class Particle:
    """
    Class for the each particle. A particle is an object of this class
    """

    def __init__(self, pose, weight, Map, distance_table):

        # Pose is in Image co-ordinates (800, 800, [0,2pi])
        self.pose = pose
        self.weight = weight
        self.motion_model = motion_model.MotionModel("Normal")
        self.measurement_model = measurement_model.MeasurementModel(Map, distance_table)

    def apply_motion_model(self, odom_reading):
        """
        Applies the motion model on the particle
        :param odom_reading: New Odometer reading in (8000, 8000, [-pi,pi])
        """

        self.pose = self.motion_model.run_odometry_model(odom_reading, self.pose)

    def update_with_measurement(self, laser_center, laser_reading):
        """
        Calculates weight of each particle using the measurement from the laser
        :param laser_reading: 180 readings from the laser
        :param laser_center: In terms of mage range (800, 800, [0,2pi])
        """

        self.weight = self.measurement_model.measurement_probability(laser_reading,
                                                                     laser_center)


class ParticleFilter:
    """
    The main class that runs the Particle Filter. It has the algorithm and integrates
    everything together.
    """

    def __init__(self, log_file, num_particles):

        print "Initializing Particle Filter"

        self.visualize = visualize.Visualize()
        self.Map = self.visualize.Map
        self.particles = []
        self.log_data = read_log.LogParser(log_file)
        self.num_particles = num_particles
        self.distance_table = self.visualize.distance_table

    def create_particles(self):
        """
        Initial particle sampling
        """

        print "Initializing Particle Set"

        for i in range(self.num_particles):
            flag = True

            while flag:
                y = random.randint(0, 799)
                x = random.randint(0, 799)
                if self.Map[y][x] > 0.9:
                    flag = False

            theta = random.uniform(0, 2*math.pi)
            pose = [y, x, theta]

            self.visualize.visualize_particle(pose)
            self.particles.append(Particle(pose, 1.0/self.num_particles,
                                           self.Map, self.distance_table))
        cv2.waitKey(0)
        print "Done initializing particles."

    def run_particle_filter(self):
        """
        Run till we run out of Log Data
        """

        for line in self.log_data.log_file:
            data = self.log_data.parse_line(line)

            if data[0] == 'O':
                odom_reading = data[1]
                for i in range(self.num_particles):
                    self.particles[i].apply_motion_model(odom_reading)

            elif data[0] == 'L':
                odom_reading = data[1]
                laser_center = data[2]
                laser_reading = data[3]
                for i in range(self.num_particles):
                    self.particles[i].apply_motion_model(odom_reading)

                    pose = self.particles[i].pose[:]
                    laser_center = [int(pose[0] - (25 * math.sin(pose[2])) // 10),
                                    int(pose[1] - (25 * math.cos(pose[2])) // 10),
                                    pose[2]]
                    self.particles[i].update_with_measurement(laser_center, laser_reading)

                self.normalize_weights()
                self.resample()
                self.reset_weights()

            self.visualize_particles()

    def normalize_weights(self):
        """
        Normalize weights for each particle
        """

        normalizer = 0.0
        for i in range(self.num_particles):
            normalizer += self.particles[i].weight

        for i in range(self.num_particles):
            self.particles[i].weight /= normalizer

    def resample(self):
        """
        Resample particles based on the measurement seen. Implements the low
        variance sampler.
        """

        new_particle_set = []
        r = random.uniform(0, 1.0/self.num_particles)
        c = self.particles[0].weight
        i = 0

        for m in range(self.num_particles):
            u = r + m*1.0/self.num_particles
            while u > c:
                i = i + 1
                c = c + self.particles[i].weight
            new_particle_set.append(copy.copy(self.particles[i]))

        self.particles = new_particle_set

    def reset_weights(self):
        """
        Resets weights of all the particles evenly.
        """

        for i in range(self.num_particles):
            self.particles[i].weight = 1.0/self.num_particles

    def visualize_particles(self):
        """
        Refreshes the image with the new set of particles
        """

        self.visualize.refresh_image()

        for i in range(self.num_particles):
            self.visualize.visualize_particle(self.particles[i].pose)

        cv2.imshow('Image', self.visualize.img)
        cv2.waitKey(1)


if __name__ == "__main__":

    particle_filter = ParticleFilter(log_file_path, NUM_PARTICLES)
    particle_filter.create_particles()
    particle_filter.run_particle_filter()
