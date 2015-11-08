import numpy as np
from collections import namedtuple
from mapping import grid as occupancy_grid

Pose = namedtuple('Pose', 'x, y, theta')
WeightedSample('WeightedSample', 'weight, pose')

class ParticleFilter():
    num_particles = 200
    def __init__(self):
        self.x_samples = np.random.randn(num_particles) + 25
        self.y_samples = np.random.randn(num_particles) + 25
        self.theta_samples = np.random.rand(num_particles) * 2*np.pi
    
    def weight_samples(self, ir_sensors):
        poses = []
        int_poses = []
        for i in range(0, num_particles):
            poses.append(Pose(self.x_samples[i], self.y_samples[i], self.theta_samples[i]))
            int_poses.append(Pose(int(self.x_samples[i]), int(self.y_samples[i]), self.theta_samples[i]))

        for pose in int_poses:
            pass # Basically the idea is that if we detect an object where there shouldnt be one
                 # or don't detect one where it should be then we should weight this sample lower

    
    def sample(self, weigted_samples):
        weights = np.array([sample.weight for sample in weighted_samples])
        weights = weights * 200 / np.sum(weights)
        cum_weights = weights.cumsum()
        index = np.searchsort(cum_weights, np.random.random())
        return weighted_samples[index].pose
