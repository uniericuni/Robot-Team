# pi
PI = 3.14159265

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
