import numpy
import matplotlib.pyplot as plt

width = 30 # measured in holes equivalent to inches
length = 52 

resolution = 2

grid = numpy.zeros([30, 52])

grid[11:15,0:5] = 1 # Gray blocks on far left
grid[7:12, 4:5] = 1 # Gray block connected to above

grid[5:7,7:9] = 1 # circular block on top left

grid[20:25,0:3] = 1 # bottom left gray block

grid[29,5:10] = 1 # light block against wall on bottom left

grid[16:18,10:12] = 1 # Circular block lower left

grid[-5:,17] = 1
grid[-1, 16] = 1 # Light L-block on bottom left

grid[-7:-5,16:18] = 1 # Circular block at end of L-block



print grid

plt.imshow(grid,interpolation='nearest')
plt.show()
