
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

class environment:

    def __init__(self, length=10, width=10, delta = 1):
        self.length = length
        self.width = width
        self.delta = delta
        self.map = [[' ' for x in range(self.width)] for y in range(self.length)]
        self.features = []

    
    def addFeature(self, featureCoords):
        #Features should be list of 2d coords
        for coordPair in featureCoords:
            x = coordPair[0]
            y = coordPair[1]
            self.map[y][x] = 'x'
        return
    
    def print(self):
        for row in self.map:
            print(row)
        print('\n')

    def plot(self):
        fig, ax = plt.subplots()
        ax.scatter(self.delta, self.delta)
        #ax.set_ticks(range(0,self.length, self.delta))
        ax.Axis.set_data_interval(0,self.length)
        ax.set_xlabel(r'X', fontsize=15)
        ax.set_ylabel(r'Y', fontsize=15)
        ax.set_title('Environment')
        ax.grid(True)
        fig.tight_layout()
        plt.show()
    
world = environment()
world.print()
feature = [[1,ii] for ii in range(world.length-1)]
world.addFeature(feature)
world.print()