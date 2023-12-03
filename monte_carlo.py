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

cameraUncertaintyX = dist.UniformDist(constants.CAMERA_X_MIN, constants.CAMERA_X_MAX)
cameraUncertaintyY = dist.UniformDist(constants.CAMERA_Y_MIN, constants.CAMERA_Y_MAX)
cameraUncertaintyZ = dist.UniformDist(constants.CAMERA_Z_MIN, constants.CAMERA_Z_MAX)
cameraUncertainty = dist.Dist3D(cameraUncertaintyX, cameraUncertaintyY, cameraUncertaintyZ)


totalUncertainty = dist.CombinedDist(
    [encoderUncertainty, 
    gripperUncertainty, 
    cameraUncertainty, 
    ])

boundary = {
    "radius_xy": constants.XY_TOLERANCE,
    "z_tolerance": constants.Z_TOLERANCE,
}

dist.plot_point_cloud(totalUncertainty.sample(100000), boundary=boundary)

# multiplot = dist.MultiPlot(2, 3)

# # dist.plot_histogram(multiplot, totalUncertainty.sample(10000), bins=100, title="Total Uncertainty")

# plt.tight_layout()
# plt.show()