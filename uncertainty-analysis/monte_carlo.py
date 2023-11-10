import matplotlib.pyplot as plt
import distributions as dist
import constants


encoderUncertaintyX = dist.UniformDist(constants.ENCODER_X_MIN, constants.ENCODER_X_MAX)
encoderUncertaintyY = dist.UniformDist(constants.ENCODER_Y_MIN, constants.ENCODER_Y_MAX)
encoderUncertaintyZ = dist.UniformDist(constants.ENCODER_Z_MIN, constants.ENCODER_Z_MAX)
encoderUncertainty = dist.Dist3D(encoderUncertaintyX, encoderUncertaintyY, encoderUncertaintyZ)

gripperUncertaintyX = dist.UniformDist(constants.GRIPPER_X_MIN, constants.GRIPPER_X_MAX)
gripperUncertaintyY = dist.UniformDist(constants.GRIPPER_Y_MIN, constants.GRIPPER_Y_MAX)
gripperUncertaintyZ = dist.UniformDist(constants.GRIPPER_Z_MIN, constants.GRIPPER_Z_MAX)
gripperUncertainty = dist.Dist3D(gripperUncertaintyX, gripperUncertaintyY, gripperUncertaintyZ)

imageRecognitionUncertaintyX = dist.NormalDist(constants.IMAGE_RECOGNITION_X_MEAN, constants.IMAGE_RECOGNITION_X_STD)
imageRecognitionUncertaintyY = dist.NormalDist(constants.IMAGE_RECOGNITION_Y_MEAN, constants.IMAGE_RECOGNITION_Y_STD)
imageRecognitionUncertaintyZ = dist.NormalDist(constants.IMAGE_RECOGNITION_Z_MEAN, constants.IMAGE_RECOGNITION_Z_STD)
imageRecognitionUncertainty = dist.Dist3D(imageRecognitionUncertaintyX, imageRecognitionUncertaintyY, imageRecognitionUncertaintyZ)

totalUncertainty = dist.CombinedDist([encoderUncertainty, gripperUncertainty, imageRecognitionUncertainty])

boundary = {
    "radius_xy": constants.XY_TOLERANCE,
    "z_tolerance": constants.Z_TOLERANCE,
}

dist.plot_point_cloud(totalUncertainty.sample(1000), boundary=boundary)

# multiplot = dist.MultiPlot(2, 3)

# # dist.plot_histogram(multiplot, totalUncertainty.sample(10000), bins=100, title="Total Uncertainty")

# plt.tight_layout()
# plt.show()