import numpy as np
from collections import namedtuple
from mapping import grid as occupancy_grid
import math

Pose = namedtuple('Pose', 'x, y, theta')
WeightedSample('WeightedSample', 'weight, pose')

class ParticleFilter():
    num_particles = 200
    def __init__(self):
        self.samples = [Pose(25+np.random.randn(), 25+np.random.randn(), 2*np.pi*np.random.rand()) for i in range(0, num_particles)]        
 
    def weight_samples(self, ir_sensors):
        int_poses = [Pose(int(p.x), int(p.y), p.theta) for p in self.samples]
        

        for pose in int_poses:
            
            pass # Basically the idea is that if we detect an object where there shouldnt be one
                 # or don't detect one where it should be then we should weight this sample lower

    
    def sample(self, weigted_samples):
        weights = np.array([sample.weight for sample in weighted_samples])
        weights = weights * 200 / np.sum(weights)
        cum_weights = weights.cumsum()
        index = np.searchsort(cum_weights, np.random.random())
        return weighted_samples[index].pose

    def resample(self, ir_sensors):
        weighted_samples = self.weight_samples(ir_sensors)
        new_samples = [self.sample(weighted_samples) for i in range(0,num_particles)]
        self.samples = new_samples

    def update_pos(self, distance, d_theta):
        new_poses = []
        for pose in self.samples:
            theta = pose.theta + d_theta
            x = pose.x + distance * math.cos(theta)
            y = pose.y + distance * math.sin(theta)
            
            new_poses.append(Pose(x, y, theta))

        self.samples = new_poses


