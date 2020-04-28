#!/usr/bin/env python

import numpy as np

class GradientDescent:

    def __init__(self, m, n):

        # keep track of updated weight
        # m = number of features
        # n = number classes
        self.n = n
        self.m = m
        self.weight = np.ones((n, m))

        # learning rate of the process(lets keep it constant for now)
        self.alpha = .003

        # dict for lable predictor
        self.lable = {0: 1004, 1: 1200, 2: 1100, 3: 1103, 4: 1400}

    def getLoss(self, x, y):

        # get squared loss
        y_new = self.oneVsAllY(y)

        loss = 0.5 * np.square((np.dot((self.weight), x) - y_new))
        return loss

    def update(self, x, y):

        # update the weight for each step

        if isinstance(y, int):
            # convert y to a vector of lables
            y_vect = self.oneVsAllY(y)

            # now update the weights
            x = np.reshape(x, (int(len(x)), 1))

            self.weight = self.weight - self.alpha * np.dot((np.dot(self.weight, x) - y_vect), np.transpose(x))
            #print self.weight

        else:
            count = 0
            weight = np.zeros((self.n,self.m))

            for i,j in zip(x,y):

                # convert y to a vector of lables
                y_vect = self.oneVsAllY(j)

                # now update the weights
                x_new = np.reshape(i, (int(len(i)), 1))

                # calculate average of weights
                weight = weight + np.dot((np.dot((self.weight), x_new) - y_vect), np.transpose(x_new))

                count += 1

            weight = weight / count

            # update weight
            self.weight = self.weight - self.alpha * weight

    def oneVsAllY(self, y):

        # append zeros for all the lables and set the lable observed to 1
        y_vect = np.zeros((self.weight.shape[0], 1))
        y_vect[int(y), 0] = 1

        return y_vect

    def linearPredictor(self, x):

        # predict the lable by getting the max of of wT*x
        y = np.argmax(np.dot(self.weight, x))
        return self.lable[int(y)]
