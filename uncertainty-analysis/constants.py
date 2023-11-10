# Position Tolerance
XY_TOLERANCE = 4.0 # radius of boundary on xy plane
Z_TOLERANCE = 5.0 # +/- height of boundary on z axis

# Encoder Uncertainty Constants
# Distribution: Uniform
ENCODER_X_MIN = -0.5
ENCODER_X_MAX = 0.5
ENCODER_Y_MIN = -0.5
ENCODER_Y_MAX = 0.5
ENCODER_Z_MIN = -0.5
ENCODER_Z_MAX = 0.5

# Gripper Uncertainty Constants
# Distribution: Normal
GRIPPER_X_MIN = -1.0
GRIPPER_X_MAX = 1.0
GRIPPER_Y_MIN = -1.0
GRIPPER_Y_MAX = 1.0
GRIPPER_Z_MIN = -3.0
GRIPPER_Z_MAX = 3.0

# Image Recognition Uncertainty Constants
# Distribution: Normal
IMAGE_RECOGNITION_X_MEAN = 0.0
IMAGE_RECOGNITION_X_STD = 2.0
IMAGE_RECOGNITION_Y_MEAN = 0.0
IMAGE_RECOGNITION_Y_STD = 2.0
IMAGE_RECOGNITION_Z_MEAN = 0.0
IMAGE_RECOGNITION_Z_STD = 2.0

# Camera Resolution Uncertainty Constants
# Distribution: Uniform
CAMERA_RESOLUTION_MIN = -0.5
CAMERA_RESOLUTION_MAX = 0.5

# Homing Uncertainty Constants
# Distribution: Normal
HOMING_MEAN = 2.0
HOMING_STD = 0.5

