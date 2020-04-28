#!/usr/bin/env python

import numpy as np

class Softmax:

    def __init__(self, m, n):

        # keep track of updated weight
        # m = number of features
        # n = number classes
        self.weight = np.ones((n, m))

        # learning rate of the process(lets keep it constant for now)
        self.alpha = 0.003

        # dict for lable predictor
        self.lable = {0: 1004, 1: 1200, 2: 1100, 3: 1103, 4: 1400}

        # initialize probs
        self.probs = np.zeros((5,1))

    def getLoss(self, x, y):

        correct_logprob = -np.log(self.probs[y])
        #print correct_logprob, self.probs[y]
        return correct_logprob

    def update(self, x, y):

        # update the weight for each step

        # now update the weights
        x = np.reshape(x, (int(len(x)), 1))

        # either do backpropagation or w*e^(alpha*log(P(yt)))
        # find gradient scores
        dscores = self.probs
        dscores[y] -= 1

        # backpropagate
        dw = np.dot(dscores, x.T)

        self.weight = self.weight - self.alpha * dw

    def computeProb(self, x):

        x = np.reshape(x, (int(len(x)), 1))

        # find appropriate scores
        score = np.dot(self.weight, x)

        # now take exponents and normalize
        exp_score = np.exp(score)

        # normalize to get probabilities
        self.probs = exp_score / np.sum(exp_score)
        #print self.probs


    def linearPredictor(self, x):

        # predict the lable by getting the max of of wT*x

        # calculate probabilities
        self.computeProb(x)
        y = np.argmax(self.probs)

        return self.lable[int(y)]

