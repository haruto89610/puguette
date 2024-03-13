import numpy as np

# BEZIER

xPoints = np.array([-0.1, -0.15, -0.25, -0.25, -0.25, 0, 0, 0.25, 0.25, 0.25, 0.15, 0.1])
zPoints = np.array([0, 0, 0.15, 0.15, 0.15, 0.1, 0.1, 0.15, 0.15, 0.15, 0, 0])
ratio = 4

def bezierPointAdjustment(x, z, ratio, zTranslation): #Adjust size and translation of gait
    xPoints = np.divide(x, ratio)
    zPoints = np.subtract(np.divide(z, ratio), zTranslation)
    return xPoints, zPoints

xPoints, zPoints = bezierPointAdjustment(xPoints, zPoints, ratio, 0.4)

def f(n, k):
    return np.math.factorial(n)/(np.math.factorial(k) * np.math.factorial(n-k))

def b(t, k, point):
    n = 11
    return point * f(n, k) * np.power(t, k) * np.power(1-t, n-k)

def bezier(T):
    xCoord = sum([b(T, i, xPoints[i]) for i in range(0, 12)])
    zCoord = sum([b(T, i, zPoints[i]) for i in range(0, 12)])
    return xCoord, zCoord
