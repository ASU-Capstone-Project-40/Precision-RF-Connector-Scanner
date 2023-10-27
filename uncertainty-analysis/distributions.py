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
        return sum([d.sample(n) for d in self.dists])
    
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
