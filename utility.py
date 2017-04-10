import numpy as np
from config import *

def fourConnected(pos):
    rtn = []
    for i in [-1,1]:
        new_pos = pos+np.array([i*x_step,0,0])
        rtn.append(new_pos)
    for i in [-1,1]:
        new_pos = pos+np.array([0,i*y_step,0])
        rtn.append(new_pos)
    return rtn 

def eightConnected(pos):
    rtn = []
    for i in range(-1,2):
        for j in range(-1,2):
            if i==0 and j==0:
                continue
            new_pos = pos+np.array([i*x_step,j*y_step,0])
            rtn.append(new_pos)
    return rtn 

def eDist(pos1, pos2):
    pos = (pos1-pos2)
    pos[2] = pos[2]%(2*pi)
    return np.linalg.norm(pos)

def mDist(pos1, pos2):
    pos = np.abs(pos1-pos2)
    pos[2] = pos[2]%(2*pi)
    return np.sum(pos)

def getKey(pos):
    return str(pos)
