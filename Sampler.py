import numpy as np
import time
import openravepy

from HomogeneousRobotTeam import *
from config import *
from Log import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

# samplers
class Sampler:

    # define sampler type
    def __init__(self,  mode,               # 0:LOCK | 1:UNLCOK,0_base | 2:UNLOCK,N_base
                        is_trans=False,     # transition sampler or not
                        pair=None,          # transition pair
                        pair_range=None):   # transition constraint

        # initiate random seed
        np.random.seed(int(time.time()))
        self.mode = mode
        self.is_trans = is_trans
        self.log = Log(display=True)
        self.pair_range = pair_range

        # initialize transition pair for transition sampling 
        if self.is_trans:
            if (pair==None) or (len(pair) is not 2):
                self.log.msg('Error', 'transition pair not assigned')
                return
            else:
                self.pair = pair
    
    # make sample
    def makeSample(self):

        # sample configuration within a mode
        if not self.is_trans:

            # sample the cliff and distributed space
            # TODO: behavior of sample point for non-trans mode1 and 2
            if self.mode==0:
                return self.easySample()
            elif self.mode==1:
                return self.easySample()
            elif self.mode==2:
                return self.easySample()
            else:
                self.log.msg('Error', 'no specified mode')
                return 

        # sample transition configuration, i.e. configuration within 2 modes
        else:
        
            # transition from LOCK to UNLOCK
            if self.pair[0]==0 or self.pair[1]==0:
                return [self.easySample()]*2
            
            # transition between base0 and baseN, i.e. within a bounding circle
            # TODO: IKSolver Test
            # note: even 2 within a circle, it is not gauranteed that it can be solve by ik solver, such as self collision, kinematic constraints, etc
            else:
                rtn_pair = [self.easySample(), self.easySample()]
                while isRejected(rtn_pair[0], rtn_pair[1], mode=LOCK0):
                    rtn_pair = [self.easySample(), self.easySample()]
                    # print rtn_pair, np.linalg.norm(rtn_pair[0]-rtn_pair[1]), self.pair_range
                return rtn_pair

    # single sample within work space:
    def easySample(self):
        if np.random.rand() >= 0.5:
            rtn = [ (X_MAX1-X_MIN1)*np.random.rand()+X_MIN1 ]
        else:
            rtn = [ (X_MAX2-X_MIN2)*np.random.rand()+X_MIN2 ]
        rtn.append( (Y_MAX-Y_MIN)*np.random.rand()+Y_MIN )
        rtn.append( Z )
        return np.array(rtn)
        
    # return transition pair
    def getTransPair(self):
        return self.pair

# ====================================================================================
# isRejected(pos, pos_ref=None, mode=UNLOCK):
#   Reject samples that didn't meet the constraints.
# Inputs:
#   pos:     np.array, sample configuration.
#   pos_ref: np.array, metric reference for the sample configuration.
#   mode:    MACRO, mode for constraints.
# ====================================================================================
def isRejected(pos, pos_ref=None, mode=UNLOCK):

    # unlocked mode
    # env2
    '''
    if mode==UNLOCK:
        return False
    '''
    # env1
    if mode==UNLOCK:
        return  not ((pos[0]<X_MAX1 and pos[0]>X_MIN1 and pos[1]<Y_MAX and pos[1]>Y_MIN) or 
                     (pos[0]<X_MAX2 and pos[0]>X_MIN2 and pos[1]<Y_MAX and pos[1]>Y_MIN))


    # locked mode
    elif mode==LOCK0 or mode==LOCKN:
        return  np.linalg.norm(pos-pos_ref) > self.pair_range
