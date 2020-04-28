#!/usr/bin/env python

import math as ma
import os
import re
import random
import motion_model
import read_log_data
import visuvalize
import cv2
import copy
import measurement_model
import multiprocessing
import numpy as np
import operator
import os

# default constants for the program

NUM_PARTICLES = 10
# for motion model
#alpha = [.005, .0008, .008, .001]
#alpha = [.005, .0005, .0005, .005]
alpha = [.005, .0005, .0005, 1]
# map path file
map_file_path = "../data/map/wean.dat"

# log data path
log_path = "../data/log/robotdata1.log"

# distance table rotation step (1,2,3 etc) while creating distacle table
rotation_step = 5#8
step_search = 0.5

# saving a video
fourcc = cv2.cv.CV_FOURCC(*'MJPG')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (800,800))

# image path
path = '../images'

class Particle:

    def __init__(self, state, coef, w, global_map, table, rotation_step):

        # initialize pose
        self.pose = state

        # weight of the particle
        self.weight = w

        # initialize the motion model and mesurement model
        self.motion_model = motion_model.MotionModel(coef, [0, 0, 0], 'normal')

        # measeurement model
        self.measurement_model = measurement_model.MeasurementModel(
            global_map, table, rotation_step)

    def propagateParticle(self, curr_odom):
        # motion model progates for each particle
        # add sampler with map
        self.pose = copy.copy(
            self.motion_model.odomModel(
                curr_odom[:], self.pose[:]))
        return

    def updateParticle(self, curr_laser, z_required):
        # sensor model updates the weight, z_test is the actual range array for
        # each pose of the particle
        self.weight = copy.copy(
            self.measurement_model.measurement_probability(
                curr_laser[:], self.pose[:], z_required))
        return


class ParticleFilter:

    def __init__(self, map_file, num, alpha, log_file_path, r_step, s_search):

        print 'Intializing particle filter'
        # TODO: load the map first
        self.visuvalize = visuvalize.Visuvalize(map_file, r_step, s_search)
        self.rotation_step = int(r_step)

        # initialize set of particles
        self.particles = []

        # read logs and based on the data propogate or update
        self.d = read_log_data.DataReader(log_file_path)

        # number of particles
        self.num = num

        # alphas for motion  model
        self.alpha = alpha

        # initial odom
        self.initial_odom = np.array([0, 0, 0])

        # check if the robot has moved to do resmapling
        self.isMoved = False
        self.start = True
        self.laser_count = 0

        self.z_test = []

        # read distance table
        self.z_required = np.load('distance_table.npy', mmap_mode='r')

        # generate random particles
        self.generate_particles = False

        # laser data
        self.ll = []

        #img count
        self.img_count = 0

        # for using vectorization
        self.particles_weight = np.ones(self.num) * 1.0 / self.num
        self.particles_pose = np.ndarray(shape = (self.num,3), dtype=float)

        # initialize the motion model and mesurement model
        self.motion_model = motion_model.MotionModel(alpha, [0, 0, 0], 'normal')

        # measeurement model
        self.measurement_model = measurement_model.MeasurementModel(
            self.visuvalize.global_map, self.visuvalize.distance_table, self.rotation_step)

    def convertImgPosesToDistance(self, pose):

        # converts the image coordinates to distances

        self.pose_orig = [pose[0] * 10.0, pose[1] * 10.0, pose[2]]
        return

    def propagateParticles(self, curr_odom):
        # motion model progates for each particle
        # add sampler with map
        self.particles_pose = np.copy(self.motion_model.odomModel(curr_odom[:], self.particles_pose))
        return

    def updateParticles(self, curr_laser, z_required):
        # sensor model updates the weight, z_test is the actual range array for
        # each pose of the particle
        self.particles_weight = np.copy(self.measurement_model.measurement_probability(curr_laser[:], self.particles_weight, z_required))
        return

    def moveAndUpdate(self, i, log_odom, log_laser):

        self.propagateParticles(log_odom)
        self.updateParticles(log_laser, self.z_required)
    #def moveAndUpdate(self, i, log_odom, log_laser):

    #    # perform both propagate and update

    #    self.particles[i].propagateParticle(log_odom)
    #    self.particles[i].updateParticle(log_laser, self.z_required)
    #    return

    def createInitialParticles(self):
        # initialze by creating n particles
        # generate samples only in free space based on the map

        print 'Creating initial set of particles'

        for i in range(self.num):
            pose = [0, 0, 0]
            #if len(self.particles) < self.num:
            #  #while not (self.visuvalize.global_map[int(pose[0]), int(
            #  #        pose[1])] == 1 and 385 < pose[0] < 410 and 385 < pose[1] < 410) and len(self.particles) < self.num:
            #  while not (self.visuvalize.global_map[int(pose[0]), int(
            #          pose[1])] == 1 and 400 < pose[0] < 440 and 490 < pose[1] < 530) and len(self.particles) < self.num:
            #  #while not (self.visuvalize.global_map[int(pose[0]), int(
            #  #        pose[1])] == 1 ):
            #      pose[0] = random.uniform(0, 800)
            #      pose[1] = random.uniform(0, 800)
            #      pose[2] = random.uniform(-ma.pi, ma.pi)
            #      #print 'initial_pose:',pose
            #else:

            while not (self.visuvalize.global_map[int(pose[0]), int(
                    pose[1])] == 1 ):
                pose[0] = random.uniform(0, 800)
                pose[1] = random.uniform(0, 800)
                pose[2] = random.uniform(-ma.pi, ma.pi)
                  #print 'initial_pose:',pose

            # visuvalize the pose
            self.visuvalize.visuvalizeParticle(pose[:])

            # convert them to meters and radians
            self.convertImgPosesToDistance(pose)

            # store each particle with pose, weight and number
            # Add desired coefficients
            #self.particles.append(
            #    Particle(
            #        self.pose_orig[:],
            #        self.alpha,
            #        1.0 / self.num,
            #        self.visuvalize.global_map,
            #        self.visuvalize.distance_table,
            #        self.rotation_step))
            self.particles_pose[i] = self.pose_orig[:]
        cv2.waitKey(1)

    def runParticleFilter(self):

        print 'Running particle filter'

        for line in self.d.my_file:

            log_data = self.d.parseFile(line)

            # if data is odom propagate
            if log_data[0] == 'O':

                print log_data[1]
                log_odom = log_data[1]

                if self.start:
                    self.start = False
                    self.initial_odom = log_odom

                # calculate difference to check if robot has moved
                diff = np.array(log_odom) - self.initial_odom
                #print ma.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

                for i in range(self.num):
                    pass
                    #self.particles[i].propagateParticle(log_odom)

            # if data is laser the use measurement model
            elif log_data[0] == 'L':
                # alternatively cpuld use number of laser measurements to check
                # for
                self.laser_count = self.laser_count + 1

                # if threads are used
                #threads = []

                # decoding log data
                log_odom = log_data[2]
                log_laser = log_data[1]
                log_rob_laser = log_data[3]
                print log_odom
                self.ll = log_laser

                if self.start:
                    self.start = False
                    self.initial_odom = log_odom

                # to check if the robot has moved
                diff = np.array(log_odom) - self.initial_odom
                if (ma.sqrt(diff[0] * diff[0] + diff[1]
                            * diff[1]) > 26 or diff[2] > 0.182):
                    self.isMoved = True
                    self.laser_count = 0

                for i in range(self.num):
                    # do both upadte and propagate one after the other
                    self.moveAndUpdate(i, log_odom, log_laser)

                    # if threads are used

                    #thread = multiprocessing.Process(target = self.moveAndUpdate, args = (i, log_odom, log_laser))
                    # thread.start()
                    # thread.join()
                    # threads.append(thread)
                if self.isMoved:
                    print 'resampling'
                    # renormalize the weights before using it for resampling
                    self.calculateSumOfWeights()
                    # resample step add for loop
                    self.resampleParticleFilter(log_odom)
                    #self.resampleMultinominal()
                    self.resetWeight()

            # visuvalize the particles
            self.visuvalizeParticles()
            self.resetWeight()

    def calculateSumOfWeights(self):
        #self.w_dict = {}
        # loop over all the samples to calculate
        #pf = copy.copy(self.particles)
        #w_sum = 0.0
        #for i in range(self.num):
        #    w_sum = w_sum + copy.copy(pf[i].weight)
        #    #self.w_dict[i] = copy.copy(pf[i].weight)
        ## normalize these weights
        #for i in range(self.num):
        #    temp = copy.copy(pf[i].weight / w_sum)
        #    self.particles[i].weight = temp
        ## for testing
        #print 'sum', w_sum
        #if w_sum < 55:
        #    self.generate_particles = True

        # if useing vectorization
        w_sum = np.sum(self.particles_weight)
        self.particles_weight = self.particles_weight / w_sum

    def generateRandomParticles(self):
        # find index with largest weight
        max_value = max(self.w_dict.values())  # maximum value
        max_keys = [k for k, v in self.w_dict.items() if v == max_value]

        # generate 500 particles around the max value
        for i in max_keys:
            pf = copy.copy(self.particles)
            pose = pf[i].pose[:]
            for j in range(0, 100):
                x = random.uniform(pose[0] - 150, pose[0] + 150)
                y = random.uniform(pose[1] - 150, pose[1] + 150)
                th = random.uniform(pose[2] - 0.5, pose[2] + 0.5)
                self.particles.append(Particle([x,
                                                y,
                                                th],
                                               pf[i].motion_model.alpha[:],
                                               pf[i].weight,
                                               self.visuvalize.global_map,
                                               self.visuvalize.distance_table,
                                               self.rotation_step))

    def resetWeight(self):

        # reset all the weights to 1 / N
        #for i in range(self.num):
        #    self.particles[i].weight = 1.0 / self.num

        # for vectorization use different method
        self.particles_weight = np.ones(self.num) * 1.0 / self.num

    def visuvalizeParticles(self):

        # refresh the image
        self.visuvalize.refreshImage()
        w_dict = {}

        for i in range(self.num):
            w_dict[i] = copy.copy(self.particles[i].weight)
            # to display the actual range readings

            #ref_angle = ma.pi / 2.0
            #ang = 0
            ##print 'z_test',self.z_test, pose_new
            # for j in self.z_test:
            #    x = pose_new[0] + j / 10 * ma.cos(pose_new[2] + ma.radians(-90 + ang))
            #    y = pose_new[1] + j / 10 * ma.sin(pose_new[2] + ma.radians(-90 + ang))
            #    ang = ang + self.rotation_step
            #    #print 'angle=', ma.degrees(pose_new[2] + ma.radians(-90 + ang))
            #    #self.visuvalize.visuvalizeLaserDots([int(x),int(y),ang])
            #    #self.visuvalize.visuvalizeLaser([int(x),int(y),ang], pose_new)i
            #    ang1=0

            # to display the lidar readings
        max_value = max(w_dict.values())  # maximum value
        max_keys = [k for k, v in w_dict.items() if v == max_value]
        max_key = max_keys[0]

        pose_new = self.particles[max_key].pose[:]
        pose_new[0] = int(pose_new[0] / 10)
        pose_new[1] = int(pose_new[1] / 10)
        ang1 = 0
        for j in self.ll:
            x = (pose_new[0] + j / 10 * ma.cos(pose_new[2] + ma.radians(-90 + ang1)))
            #pose_new[0] = int(pose_new[0] / 10)
            y = (pose_new[1] + j / 10 * ma.sin(pose_new[2] + ma.radians(-90 + ang1)))
            #pose_new[1] = int(pose_new[1] / 10)
            ang1 = ang1 + 1
            #print 'angle=', ma.degrees(pose_new[2] + ma.radians(-90 + ang))
            self.visuvalize.visuvalizeLaserData([int(x),int(y),ang1])
            self.visuvalize.visuvalizeLaser([int(x),int(y),ang1], pose_new)

        for i in range(self.num):

            # displaying particles

            pose_new = copy.copy(self.particles[i].pose)
            pose_new[0] = pose_new[0] / 10
            pose_new[1] = pose_new[1] / 10
            x = (pose_new[0] + 10 * ma.cos(pose_new[2]))
            #pose_new[0] = int(pose_new[0] / 10)
            y = (pose_new[1] + 10 * ma.sin(pose_new[2]))
            self.visuvalize.visuvalizeOrientation([int(pose_new[0]), int(pose_new[1])], [int(x), int(y)])
            self.visuvalize.visuvalizeParticle(pose_new)

        cv2.imshow('image', self.visuvalize.img)
        cv2.imwrite(os.path.join(path , 'log4_.jpg'+ str(self.img_count + 1)) + '.png', self.visuvalize.img)
        self.img_count = self.img_count + 1
        out.write(copy.copy(np.uint8(self.visuvalize.img)))
        cv2.waitKey(1)

        return

    def resampleMultinominal(self):
        # after updating reweight each of them based on measurement model
        # resampling step using multinomianal distribution
        # create a copy of old set
        pf = copy.copy(self.particles)

        # store the weight in an array
        weight_dict = {}
        weight = []

        for i in range(self.num):
            weight.append(copy.copy(pf[i].weight))
            weight_dict[i] = int(0)
        # pick the indices based on weight
        index = np.random.multinomial(self.num, weight)
        # create a new set
        pf_new = []
        print weight_dict

        # maintain the sixe of particles
        particle_count = 0
        total = 0
        # iterate over all the indices
        for i in (index):
            weight_dict[i] = weight_dict[i] + 1
            if weight_dict[i] > 300:
                # perturb
                new_particle = copy.copy(pf[i])
                pose = self.calculatePerturbedPose(new_particle.pose)
                pf_new.append(copy.copy(new_particle))
            else:
                pf_new.append(copy.copy(pf[i]))

            #while count > 0:
            #    if count == 1:
            #        pf_new.append(copy.copy(pf[i]))
            #        total = total + 1
            #    elif count < 3 and particle_count < 1200: # Stop duplicating if max reached
            #        # Need to add copies to new list, not just identical references!
            #        new_particle = copy.copy(pf[idx])
            #        pose = self.calculatePerturbedPose(new_particle.pose)
            #        pf_new.append(copy.copy(new_particle))
            #        total = total + 1
            #    count -= 1
        # copy the old set to  new
        #self.num = total
        #print self.num, total
        self.particles = copy.copy(pf_new)

    def calculatePerturbedPose(self,pose):
        x = random.uniform(pose[0] - 5, pose[0] + 5)
        y = random.uniform(pose[1] - 5, pose[1] + 5)
        th = random.uniform(pose[2] - 0.015, pose[2] + 0.015)

        return [x,y,th]

    def resampleParticleFilter(self, log_odom):
        # after updating reweight each of them based on measurement model
        # resampling step using low variance sampler
        # create a copy of old set
        pf_pose = np.copy(self.particles_pose)
        pf_weight = np.copy(self.particles_weight)

        # create a new set
        pf_new = np.ndarray(shape(self.num,3), dtype=float)

        # create a random number bet 0 and 1/M
        r = random.uniform(0, 1.0 / self.num)
        c = copy.copy(pf_weight[0])
        i = 0
        #pf_pose = copy.copy(pf[i].pose)
        #count = 0
        #for p in range(self.num):
        #    print 'weight',pf[p].weight,'pose',pf[p].pose
        for m in range(self.num):
            # draw i with prob proportional to weight
            u = r + m * 1.0 / self.num
            while c < u:
                i = i + 1
                c = c + copy.copy(pf_weight[i])
            #pf_new.append(Particle(pf[i].pose[:], pf[i].motion_model.alpha[:], pf[i].weight, self.visuvalize.global_map, self.visuvalize.distance_table))
            pf_new[m] = np.copy(pf_pose[i])

        #w_dict = {}

        #for i in range(self.num):
        #    w_dict[i] = copy.copy(self.particles[i].weight)

        #sorted_w = sorted(w_dict.items(), key=operator.itemgetter(1))
        #weight = 0
        #count = 0
        #pf_modified = []
        #for i in range(self.num):
        #    print sorted_w[1,1]
        #    if ma.fabs(weight - sorted_w[i]) < 10**-15 and count <200:
        #        count = count + 1
        #        pf_modified.append(copy.copy(pf_new[i]))

        #    elif count > 200:
        #        print 'perturbing'
        #        pf_modified[i].pose = self.calculatePerturbedPose(pf_modified[i].pose)
        #        pf_modified.append(copy.copy(pf_new[i]))

        #    elif weight != sorted_w[i]:
        #        print 'I am here'
        #        count = 0
        #        weight = sorted_w[i]
        #        pf_modified.append(copy.copy(pf_new[i]))

        #self.particles = copy.copy(pf_new)
        self.particles_pose = np.copy(pf_new)
        self.initial_odom = np.array(log_odom)
        self.isMoved = False

        #for p in range(self.num):
        #    print 'afer','weight',self.particles[p].weight,'pose',self.particles[p].pose
        #if self.generate_particles:
        #    print 'Generating random particles'
        #    #self.createInitialParticles()
        #    #self.generateRandomParticles()
        #    self.generate_particles = False


if __name__ == "__main__":

    # TODO: get all the required coefficients and parameters

    # Create a particle filter
    pf = ParticleFilter(
        map_file_path,
        NUM_PARTICLES,
        alpha,
        log_path,
        rotation_step,
        step_search)

    # initialize a set of particles
    pf.createInitialParticles()

    # run the particle filter
    pf.runParticleFilter()
    out.release()
    cv2.waitKey(0)
