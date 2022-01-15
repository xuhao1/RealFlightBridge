import math
import numpy as np
from transformations import *

def float_constrain(v, min, max):
    if v < min:
        v = min
    if v > max:
        v = max
    return v


def wrap_pi(v):
    return (v + np.pi) % (2 * np.pi) - np.pi

def JoyEXP(v,exp):
    if math.fabs(v) < 0.001:
        return 0
    return math.pow(math.fabs(v),exp)* v / math.fabs(v)

def quaternion_rotate(q, v):
    mat = quaternion_matrix(q)[0:3,0:3]
    v = np.array([v])
    v = v.transpose()
    v = mat @ v
    v = v.transpose()[0]
    return v

def quaternion_inv_rotate(q, v):
    mat = quaternion_matrix(q)[0:3,0:3].transpose()
    v = np.array([v])
    v = v.transpose()
    v = mat @ v
    v = v.transpose()[0]
    return v