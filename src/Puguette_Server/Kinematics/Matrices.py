import numpy as np

def Rx(roll): #rotation matrix around x axis
    return np.matrix([[1, 0, 0, 0],
                      [0, np.cos(roll), -np.size(roll), 0],
                      [0, np.sin(roll), np.cos(roll), 0],
                      [0, 0, 0, 1]])

def Ry(pitch): #rotation matrix around y axis
    return np.matrix([[np.cos(pitch), 0, np.sin(pitch), 0],
                      [0, 1, 0, 0],
                      [-np.sin(pitch), 0, np.cos(pitch), 0],
                      [0, 0, 0, 1]])

def Rz(yaw): #rotation matrix around z axis
    return np.matrix([[np.cos(yaw), -np.sin(yaw), 0, 0],
                      [np.sin(yaw), np.cos(yaw), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

def Rxyz(roll, pitch, yaw):
    if roll != 0 or pitch != 0 or yaw != 0:
        return Rx(roll) * Ry(pitch) * Rz(yaw)
    else:
        return np.identity(4)

def RTmatrix(orientation, position): #rototranslation matrix
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    x0 = position[0]
    y0 = position[1]
    z0 = position[2]

    translation = np.matrix([[1, 0, 0, x0],
                             [0, 1, 0, y0],
                             [0, 0, 1, z0],
                             [0, 0, 0, 1]])
    rotation = Rxyz(roll, pitch, yaw)
    return rotation * translation

def transform(coord, rotation, translation): #vector to desired rotation and translation
    vector = np.array([[coord[0]], [coord[1]], [coord[2]], [1]])
    transform = RTmatrix(rotation, translation) * vector
    return np.array([transform[0, 0], transform[1, 0], transform[2, 0]])