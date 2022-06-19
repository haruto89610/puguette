"""
credit: https://dspace.mit.edu/handle/1721.1/98270
"""

import numpy as np
from KinematicModel import *

# BEZIER

xPoints = np.array([-0.1, -0.15, -0.25, -0.25, -0.25, 0, 0, 0, 0.25, 0.25, 0.2, 0.1])
zPoints = np.array([0, 0, 0.15, 0.15, 0.15, 0.1, 0.1, 0.2, 0.2, 0.2, 0, 0])
# xPoints = np.divide(xPoints, 2) # make gait smaller
zPoints = np.subtract(zPoints, 0.4)


def f(n, k):
    return np.math.factorial(n)/(np.math.factorial(k) * np.math.factorial(n-k))

def b(t, k, point):
    n = 11
    return point * f(n, k) * np.power(t, k) * np.power(1-t, n-k)

def bezier(T):
    xCoord = sum([b(T,i,xPoints[i]) for i in range(0, 12)])
    zCoord = sum([b(T,i,zPoints[i]) for i in range(0, 12)])

    return xCoord, zCoord

# TURN

defaultAngle = np.arctan2(L/2, W/2)
bodyRadius = np.sqrt((L/2)**2 + (W/2)**2)

def turn(T, angle):
    xCoordTurn = bodyRadius * np.sin(defaultAngle - angle)
    yCoordTurn = bodyRadius * np.cos(defaultAngle - angle)

    xCoord = (bodyRadius * np.sin(defaultAngle - angle)) * T
    yCoord = (bodyRadius * np.cos(defaultAngle - angle)) * T

    __radius = np.sqrt( xCoordTurn**2 + yCoordTurn**2 )

    zCoordTurn = np.sqrt( __radius * T**2 )

    return xCoordTurn, yCoordTurn
