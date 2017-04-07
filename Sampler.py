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
    def makeSample():
        if self.is_trans:
            if self.mode==0:
                pass
            elif self.mode==1:
                pass
            elif self.mode==2:
                pass
            else:
                self.log.msg('Error', 'unrecognized mode')
                return 

    # return transition pair
    def getTransPair():
        return self.pair
