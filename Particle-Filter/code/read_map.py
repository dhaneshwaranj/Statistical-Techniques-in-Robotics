import numpy as np
import matplotlib.image as img
import cv2


class MapImage:
    """
    Class to read the map and store it
    """

    def __init__(self):
        self.map = None
        print "Reading map"

        with open("..\data\map\wean.dat", "r") as f:
            im = '' + str(f.readline())
            s, x = ' '.join(im.split()).split(" ")

            im = '' + str(f.readline())
            s, y = ' '.join(im.split()).split(" ")

            im = '' + str(f.readline())
            s, res = ' '.join(im.split()).split(" ")

            im = '' + str(f.readline())
            s, autoX = ' '.join(im.split()).split(" ")

            im = '' + str(f.readline())
            s, autoY = ' '.join(im.split()).split(" ")

            im = f.readline()

            im = '' + str(f.readline())

            s, xSize, ySize = im.split(" ")

            xSize = int(xSize)
            ySize = int(ySize)

            i = 0
            arr = np.zeros([xSize, ySize])

            for i in range(xSize):
                arr[i] = [float(val) for val in f.readline().split() if val >= 0]

            arr = np.rot90(arr)

            self.map = arr

            print "Done reading map."
    def store_map(self):
        self.map = cv2.cvtColor(np.float32(self.map), cv2.COLOR_GRAY2BGR)
        img.imsave("map_py.jpg", self.map, 0, 1)


if __name__ == '__main__':
    MapImage().store_map()
