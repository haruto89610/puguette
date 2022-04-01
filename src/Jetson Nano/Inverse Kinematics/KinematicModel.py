import numpy as np
import IK
import Matrices

L = 0.52
W = 0.15
coxa = 0.18
femur = 0.29
tibia = 0.31

def Kinematic_Model(coord, rotation, translation):
    oriToFR = np.array([ L/2, -W/2, 0])
    oriToFL = np.array([ L/2,  W/2, 0])
    oriToBR = np.array([-L/2, -W/2, 0])
    oriToBL = np.array([-L/2,  W/2, 0])

    oriToFRNew = Matrices.transform(oriToFR, rotation, translation)
    oriToFLNew = Matrices.transform(oriToFL, rotation, translation)
    oriToBRNew = Matrices.transform(oriToBR, rotation, translation)
    oriToBLNew = Matrices.transform(oriToBL, rotation, translation)

    FROffset = -oriToFRNew + oriToFR
    FLOffset = -oriToFLNew + oriToFL
    BROffset = -oriToBRNew + oriToBR
    BLOffset = -oriToBLNew + oriToBL

    FRCoord = np.array([coord[0,0], coord[0,1], coord[0,2]])
    FLCoord = np.array([coord[1,0], coord[1,1], coord[1,2]])
    BRCoord = np.array([coord[2,0], coord[2,1], coord[2,2]])
    BLCoord = np.array([coord[3,0], coord[3,1], coord[3,2]])

    FRCoordNew = FRCoord + FROffset
    FLCoordNew = FLCoord + FLOffset
    BRCoordNew = BRCoord + BROffset
    BLCoordNew = BLCoord + BLOffset

    FRAngles = IK.solve_R(FRCoordNew, coxa, femur, tibia)
    FLAngles = IK.solve_L(FLCoordNew, coxa, femur, tibia)
    BRAngles = IK.solve_R(BRCoordNew, coxa, femur, tibia)
    BLAngles = IK.solve_L(BLCoordNew, coxa, femur, tibia)

    FRAngles[1] = ((FRAngles[0] * 180/np.pi) * 1/360) * 10
    FRAngles[2] = (((180-(FRAngles[1] * 180/np.pi)) * 1/360) * 10) * 21/12
    # return allAnglesString
    return FRAngles[1], FRAngles[2]
    # return FRCoordNew, FLCoordNew, BRCoordNew, BLCoordNew

coords = np.array([[0, 0, -0.4],   #FR
                   [0,  0.18, -0.35],   #FL
                   [0, -0.18, -0.35],   #BR
                   [0,  0.18, -0.35]])  #BL

rotations = np.array([0,0,0])
translation = np.array([0,0.1,-0.1])

# print(Kinematic_Model(coords, rotations, translation))