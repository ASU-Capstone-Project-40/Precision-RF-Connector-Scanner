import numpy
import matplotlib.pyplot as plt

class UniformDist:
    def __init__(self, min, max):
        self.min = min
        self.max = max
    
    def sample(self, n=1):
        return numpy.random.default_rng().uniform(self.min, self.max, n)

class NormalDist:
    def __init__(self, mean, std):
        self.mean = mean
        self.std = std
    
    def sample(self, n=1):
        return numpy.random.default_rng().normal(self.mean, self.std, n)
    
class CombinedDist:
    def __init__(self, dists):
        self.dists = dists
    
    def sample(self, n=1):
        samples = numpy.array([d.sample(n) for d in self.dists])
        return numpy.sum(samples, axis=0)
    
class Dist3D:
    def __init__(self, x_dist, y_dist, z_dist):
        self.x_dist = x_dist
        self.y_dist = y_dist
        self.z_dist = z_dist

    def sample(self, n=1):
        points = []
        coordinates = numpy.array([self.x_dist.sample(n), self.y_dist.sample(n), self.z_dist.sample(n)]).T
        for coordinate in coordinates:
            point = []
            point.append(coordinate[0])
            point.append(coordinate[1])
            point.append(coordinate[2])
            points.append(point)
        return points
    
# Plotting functions

class MultiPlot:
    def __init__(self, nrows, ncols):
        self.fig, self.axs = plt.subplots(nrows, ncols, figsize=(10, 7))
        self.current = 0
        self.nrows = nrows
        self.ncols = ncols

    def next_subplot(self):
        row = self.current // self.ncols
        col = self.current % self.ncols
        self.current += 1
        return self.axs[row, col] if (self.nrows > 1 or self.ncols > 1) else self.axs


def plot_histogram(multiplot, data, bins=None, title=None):
    ax = multiplot.next_subplot()
    ax.hist(data, bins=bins, edgecolor='black', alpha=0.7)
    ax.set_xlabel('Value')
    ax.set_ylabel('Frequency')
    ax.set_title(title)
    ax.grid(True)

def plot_point_cloud(data, boundary):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    inside = []
    outside = []
    for point in data:
        if (numpy.hypot(point[0], point[1]) <= boundary['radius_xy'] and
            abs(point[2]) <= boundary['z_tolerance']):
            inside.append(point)
        else:
            outside.append(point)
    
    inside = numpy.array(inside)
    outside = numpy.array(outside)
    
    if inside.size > 0:
        ax.scatter(inside[:, 0], inside[:, 1], inside[:, 2], c='green', marker='o', label='Inside tolerance (' + str(100 * len(inside) / len(data))  + '%)')
    
    if outside.size > 0:
        ax.scatter(outside[:, 0], outside[:, 1], outside[:, 2], c='red', marker='o', label='Outside tolerance(' + str(100 * len(outside) / len(data))  + '%)')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.legend()
    
    plt.show()
