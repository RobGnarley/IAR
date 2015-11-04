import numpy
import matplotlib.pyplot as plt

width = 31 # measured in holes
length = 56 

resolution = 2

grid = numpy.zeros([31, 56])


grid[0:30, 40:45] = 1

print grid

plt.plot(grid)
plt.show()
