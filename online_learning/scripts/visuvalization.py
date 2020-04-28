#!/usr/bin/env python

import numpy as np
import vtk
import gradient_descent as gd
import softmax_classifier as sm
import bayesian_linear_regression as blr

data_path_tr = '../data/oakland_part3_am_rf.node_features'
data_path_te = '../data/oakland_part3_an_rf.node_features'

class Visuvalization:

    def __init__(self, file_name, zMin=-10.0, zMax=10.0, max_num_points=1e6):
        # setting up visuvalization params
        print 'Setting up visuvalization params'

        # read the file
        self.dataFile = open(file_name, "r")

        # set maximum number of points to be used
        self.maxNumPoints = max_num_points

        # initialize a poly data
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()


        # The mapper is responsible for pushing the geometry into the graphics
        # library. It may also do color mapping, if scalars or other
        # attributes are defined.
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInput(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)

        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

        # Setup the colors array
        self.colors = vtk.vtkUnsignedCharArray()
        self.colors.SetNumberOfComponents(3)
        self.colors.SetName("colors")

        # set up color dicts
        r = [255, 0, 0]
        g = [0, 255, 0]
        br = [244, 164, 96]
        y = [0, 255, 255]
        p = [255, 255, 0]

        # match labels to corresponding color
        self.colorDict = {1004: g, 1200: br, 1100: y, 1103: r, 1400: p}

        # dict for feeatures
        self.feature_dict = {1004: 0, 1200: 1, 1100:2, 1103:3, 1400:4}

        print 'Done Setting up visuvalization'

    def clearPoints(self):
        # Create the geometry of a point (the coordinate)
        self.points = vtk.vtkPoints()

        # Create the topology of the point (a vertex)
        self.vertices = vtk.vtkCellArray()

        # set the geometer and topology of poly data
        self.vtkPolyData.SetPoints(self.points)
        self.vtkPolyData.SetVerts(self.vertices)

    def addPoint(self, color, pos):
        # Add the next point color
        self.colors.InsertNextTupleValue(color)

        if self.points.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.points.InsertNextPoint(pos[:])
            self.vertices.InsertNextCell(1)
            self.vertices.InsertCellPoint(pointId)

        else:
            r = np.random.randint(0, self.maxNumPoints)
            self.points.SetPoint(r, point[:])

        self.vertices.Modified()
        self.points.Modified()

    def visuvalize(self, pos, y):

        # input args : x: pos and y: labels
        if isinstance(y, int):
            self.addPoint(self.colorDict[y], pos)

        else:
            for i, j in zip(pos, y):
                # add point to point cloud
                # only if we want to disply raw data
                self.addPoint(self.colorDict[int(j)], i)

        self.vtkPolyData.GetPointData().SetScalars(self.colors)


    def readFile(self):

        # iterate through each line
        read_line = False

        # create a numpy arrays to store read data with lables, features and position
        self.data_pos = []
        self.data_lable = []
        self.data_features = []

        for i,line in enumerate(self.dataFile):

            # remove white sapce and new line
            line = line.rstrip()

            if line:
                if line[0] == '#':
                    # start reading line from now
                    continue

            else:
                continue

            # read line
            # split the line
            line_split = line.split()

            # read x y z
            pos = [float(j) for j in (line_split[0:3])]
            self.data_pos.append(np.asarray(pos))

            # read lable
            lable = int(line_split[4])
            self.data_lable.append(np.asarray(lable))

            # read feature
            feature = [float(j) for j in (line_split[5:])]
            self.data_features.append(np.asarray(feature))


        # test visuvalization
        #self.visuvalize(self.data_pos, self.data_lable)

    def iterateOverData(self, gradient, n = 5, test = False, batch = False, class_list = []):

        print 'iterating ', n, 'times with test = ',test, 'batch = ', batch
        # performance parameters
        count_loss = 0
        loss = []

        # train in mini-batch for better accuracy
        x_vect = []
        y_vect = []
        # iterate over recorded data
        if not test:
            num = n
        else:
            num = 1

        for no in range(num):
            print 'iteration = ', no
            for i in np.random.choice(range(len(self.data_lable)), len(self.data_lable), False):
            #for x, y in zip(self.data_features, self.data_lable):
                # extract features from indices
                y = self.feature_dict[int(self.data_lable[i])]
                if class_list:
                    if y not in class_list:
                        continue

                x = self.data_features[i]
                # predict using linear predictor
                predicted_y = gradient.linearPredictor(x)
                #print gradient.getLoss(x, self.feature_dict[int(y)])
                if predicted_y == self.data_lable[i]:
                    count_loss = count_loss + 1

                loss.append(gradient.getLoss(x, y))

                # update the weights only if training
                if not test:
                    if not batch:
                        gradient.update(x, y)
                    else:
                        x_vect.append(x)
                        y_vect.append(y)
                        if i % 10 == 0 and i != 0:

                            # update weights mini-batch wise
                            gradient.update(x_vect, y_vect)

                            # reset the arrays
                            del x_vect[:]
                            del y_vect[:]

                if test:
                    # add point to visuvalize
                    self.visuvalize(self.data_pos[i], predicted_y)

            # print out results
            print 'mean loss', np.mean(loss)
            print 'percentage predicted right = ', float(count_loss) / len(self.data_lable)

            # reset loss and count
            count_loss = 0
            del loss[:]

    def runGradientDescent(self, data_path_tr, data_path_te, batch = False):

        print "Running Gradient descent on squared loss"
        # set the data path to test
        self.dataFile = open(data_path_tr, "r")
        self.readFile()

        # first run the learning data then run the test data
        # TODO: for now hard coded the shape
        gradient = gd.GradientDescent(self.data_features[0].shape[0], 5)

        self.iterateOverData(gradient, 7, False, False, [])

        # set the data path to test
        self.dataFile = open(data_path_te, "r")
        self.readFile()

        self.iterateOverData(gradient, 1, True, False, [])

    def runSoftMax(self, data_path_tr, data_path_te):

        print "Running Softmax classifier"
        # set the data path to test
        self.dataFile = open(data_path_tr, "r")
        self.readFile()

        # first run the learning data then run the test data
        # TODO: for now hard coded the shape
        gradient = sm.Softmax(self.data_features[0].shape[0], 5)

        self.iterateOverData(gradient, 40, False, False, [])

        # set the data path to test
        self.dataFile = open(data_path_te, "r")
        self.readFile()

        self.iterateOverData(gradient, 1, True, False, [])

    def runBayesianRegression(self, data_path_tr, data_path_te):

        # set the data path to test
        print "Running Bayseian Regression"
        self.dataFile = open(data_path_tr, "r")
        self.readFile()

        # first run the learning data then run the test data
        # TODO: for now hard coded the shape
        gradient = blr.BayesianRegression(self.data_features[0].shape[0], 5)

        self.iterateOverData(gradient, 1, False, False, [0, 1])

        # set the data path to test
        self.dataFile = open(data_path_te, "r")
        self.readFile()

        self.iterateOverData(gradient, 1, True, False, [0, 1])

if __name__ == "__main__":

    v = Visuvalization(data_path_te)
    #v.readFile()

    algo = int(raw_input('Use 0: sq loss gd, 1: baysien regre, 2: softmax : '))

    if algo == 0:
        # run gradient descent
        v.runGradientDescent(data_path_tr, data_path_te)

    elif algo == 2:
        # run softmax classifer
        v.runSoftMax(data_path_tr, data_path_te)

    elif algo == 1:

        # bayesian linear regression
        v.runBayesianRegression(data_path_tr, data_path_te)

    else:
        v.readFile()
        v.visuvalize(v.data_pos, v.data_lable)
        print "Not a valid algo. Displaying ground truth. Quiting Rerun"

    # Renderer
    renderer = vtk.vtkRenderer()
    renderer.AddActor(v.vtkActor)
    renderer.SetBackground(.1, .1, .1)
    renderer.ResetCamera()

    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # Begin Interaction
    renderWindow.Render()
    renderWindowInteractor.Start()
