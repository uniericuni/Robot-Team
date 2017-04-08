import numpy as np
import openravepy

from HomogeneousRobotTeam import *
from Graph import *
from config import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# step size
STEP = TRNAS_ERR

# samplers
class sampler:

    # define sampler type
     def __init__(self, mode,               # 0:LOCK | 1:UNLCOK,0_base | 2:UNLOCK,N_base
                        is_trans=False,     # transition sampler or not
                        pair=None):

        # initiate random seed
        self.random.seed(int(time.time()))
        self.mode = mode
        self.is_trans = is_trans
        self.log = Log(display=True)

        # initialize transition pair for transition sampling 
        if self.is_trans:
            if (pair==None) or (len(pair) is not 2):
                self.log.msg('Error', 'transition pair not assigned')
                return
            else:
                self.pair==pair
    
    # make sample
    # TODO
    def makeSample(self):

        # sample configuration within a mode
        if not self.is_trans:

            # sample the cliff and distributed space
            # TODO: behavior of sample point for non-trans mode1 and 2
            if self.mode==0:
                return self.easySample()
            elif self.mode==1:
                pass
            elif self.mode==2:
                pass
            else:
                self.log.msg('Error', 'no specified mode')
                return 

        # sample transition configuration, i.e. configuration within 2 modes
        else:
        
            # transition from LOCK to UNLOCK
            if self.pair[0]==0 or self.pari[1]==0:
                return [self.easySample()]*2
            
            # transition between base0 and baseN, i.e. within a bounding circle
            # TODO: even 2 within a circle, it is not gauranteed that it can be solve by ik solver, such as self collision, kinematic constraints, etc
            # TODO: IKSolver Test
            else:
                rtn_pair = [self.easySample(), self.easySample()]
                while np.linalg.norm(rtn_pair[0]-rtn_pair[1]) < RANGE:
                    rtn_pair = [self.easySample(), self.easySample()]
                return rtn_pair

    # single sample within work space:
    def easySample(self):
        if np.random.rand() >= 0.5:
            rtn = [ (X_MAX1-X_MIN1)*np.random.rand()+X_MIN1 ]
        else:
            rtn = [ (X_MAX2-X_MIN2)*np.random.rand()+X_MIN2 ]
        rtn.append( (Y_MAX-Y_MIN)*np.random.rand()+Y_MIN )
        rtn.append( Z )
        return rtn
        
    # return transition pair
    def getTransPair(self):
        return self.pair
