# pi
PI = 3.14159265

# single robot dof
DOF = 2

# model directory
MODELS = '/usr/local/share/openrave-0.9/'

# lock robot template model
LOCK_ROBOT_TEMPLATE = MODELS + 'robots/lockRobotTemplate.robot.xml'

# temporary lock robot template model 
TEMPORARY_LOCK_ROBOT = '/media/sf_Share/Final/tempLockRobot.robot.xml'

# intance robot path
INSTANCE_ROBOT = MODELS + 'robots/ball.kinbody.xml'

#
# DEG_LIMIT1 = -180.0
# DEG_LIMIT2 = 180.0

# default of robot spherical raidus
RADIUS = 30.0
TRANS_ERR = RADIUS/10000

#
ISDISPLAY = True

# system status
LOCK = True
UNLOCK = False

# number of homogeneous robots
INSTANCE_NUM = 4

# maximum multimodal sampling iteration
MM_MAX_ITER = 30
MAX_SAMPLING_TIME = 10000

# experiment one field constraint
X_MIN1 = -10
X_MAX1 = -2.2
X_MIN2 = 10
X_MAX2 = 2.2
Y_MAX = 10
Y_MIN = -10
Z = 20.3

# cliff length
CLIFF = 4
