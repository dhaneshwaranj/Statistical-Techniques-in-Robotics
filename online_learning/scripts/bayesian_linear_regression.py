#!/usr/bin/env python

import numpy as np

class BayesianRegression:

    def __init__(self, m, n, l = 0.5):

        # keep track of updated weight
        # m = number of features
        # n = number classes
        self.weight = np.ones((1, m))

        # learning rate of the process(lets keep it constant for now)
        self.sigma_sq = 1

        # precision matrix initialization
        self.P = 1 / l * np.identity(m)

        # now initialize the J matrix
        self.J = np.dot(self.weight, self.P)

        # dict for lable predictor
        self.lable = {0: 1004, 1: 1200, 2: 1100, 3: 1103, 4: 1400}

    def getLoss(self, x, y):

        # get squared loss
        y_new = self.vectorY(y)

        loss = 0.5 * np.square((np.dot((self.weight), x) - y_new))
        return loss

    def update(self, x, y):

        # update the weight for each step

        # convert y to a vector of lables
        y_vect = self.vectorY(y)

        # now update the weights
        x = np.reshape(x, (int(len(x)), 1))

        # clacultae P
        self. P += 1 / self.sigma_sq * np.dot(x,x.transpose())

        # calculate J
        self.J += 1 / self.sigma_sq * y_vect * x.transpose()

        self.weight = np.dot(self.J, np.linalg.inv(self.P))

        #print self.weight

    def vectorY(self, y):

        # append zeros for all the lables and set the lable observed to 1
        #y_vect = np.zeros((1, 1))
        #y_vect[int(y), 0] = 1
        #print y
        if y == 0:
            return -1
        elif y == 1:
            return 1

    def linearPredictor(self, x):

        # predict the lable by getting the max of of wT*x
        prediction = np.dot(self.weight, x)
        if prediction > 0:
            return self.lable[1]
        else:
            return self.lable[0]

