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

grid[5:10,17] = 1
grid[5,16] = 1 # top left L block

grid[13:18,23:28] = 1 # big central block
grid[9:13,23:26] = 1 # block connected to central block

grid[-4:-3, -12:-7] =1 # bottom right gray blocks
grid[-8:-3, -7] = 1

grid[-12:-10, -6:-4] = 1 # bottom right circle

grid[-14:-12,-16:-12] = 1 # group of circles
grid[-16:-14,-15:-13] = 1 

grid[0:3,-20:-15] = 1 # Gray blocks top right
grid[3:8, -16] = 1
grid[7,-17] = 1 # While L top right

dim_y = 140
dim_x = 77
inc_y = 140/52.0
inc_x = 77/30.0
#print inc_x, inc_y
#plt.imshow(grid, cmap='Greys', interpolation='nearest')
#plt.show()
