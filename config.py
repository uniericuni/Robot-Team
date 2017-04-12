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
TRANS_ERR = RADIUS/1000

#
ISDISPLAY = True

# system status
UNLOCK = 0
LOCK0 = 1
LOCKN = 2

# number of homogeneous robots
INSTANCE_NUM = 14

# maximum multimodal sampling iteration
MM_MAX_ITER = 50
MAX_SAMPLING_TIME = 10

# experiment one field constraint
STEP = 0.2          # margin
X_MIN1 = -10 + STEP
X_MAX1 = -2.2
X_MIN2 = 2.2
X_MAX2 = 10 - STEP
Y_MIN = -10 + STEP
Y_MAX = 10 - STEP
Z = 20.3

# cliff length
CLIFF = 4

# =========================
# ASTAR
# =========================

H_PRIOR = 100     # heuristic weighting (k times step cost)

FLOAT_MAX = 1e+100
FLOAT_MIN = 1e-100

MAX_ERR = STEP*2
MAX_ITER = 200/(STEP*STEP)
TIME_DELTA = 0.0001

pi = 3.1415926
x_step = STEP
y_step = STEP
r_step = pi/2
point_size = x_step * 50.0

# =========================
# ROTATION
# =========================
ROT_ERR = 0.1
