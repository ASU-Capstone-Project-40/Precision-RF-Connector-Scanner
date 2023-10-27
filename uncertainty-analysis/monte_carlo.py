import matplotlib.pyplot as plt
import distributions as dist
import constants


gripperError = dist.NormalDist(constants.GRIPPER_ERROR_MEAN, constants.GRIPPER_ERROR_STD)
cameraError = dist.UniformDist(constants.CAMERA_ERROR_MIN, constants.CAMERA_ERROR_MAX)
totalError = dist.CombinedDist([gripperError, cameraError])

multiplot = dist.MultiPlot(2, 3)

dist.plot_histogram(multiplot, gripperError.sample(10000), bins=100, title="Gripper Error")
dist.plot_histogram(multiplot, cameraError.sample(10000), bins=100, title="Camera Error")
dist.plot_histogram(multiplot, totalError.sample(10000), bins=100, title="Total Error")

plt.tight_layout()
plt.show()