import numpy as np
from . import Inverse_Kinematics
from . import Matrices

L = 0.52
W = 0.15
coxa = 0.11
femur = 0.205
tibia = 0.280

def Kinematic_Model(coord, rotation, translation):
    oriToFR = np.array([ L/2,  W/2, 0])
    oriToFL = np.array([ L/2, -W/2, 0])
    oriToBR = np.array([-L/2,  W/2, 0])
    oriToBL = np.array([-L/2, -W/2, 0])

    oriToFRNew = Matrices.transform(oriToFR, rotation, translation)
    oriToFLNew = Matrices.transform(oriToFL, rotation, translation)
    oriToBRNew = Matrices.transform(oriToBR, rotation, translation)
    oriToBLNew = Matrices.transform(oriToBL, rotation, translation)

    FROffset = -oriToFRNew + oriToFR
    FLOffset = -oriToFLNew + oriToFL
    BROffset = -oriToBRNew + oriToBR
    BLOffset = -oriToBLNew + oriToBL

    FRCoord = np.array([coord[0][0], coord[0][1], coord[0][2]])
    FLCoord = np.array([coord[1][0], coord[1][1], coord[1][2]])
    BRCoord = np.array([coord[2][0], coord[2][1], coord[2][2]])
    BLCoord = np.array([coord[3][0], coord[3][1], coord[3][2]])

    FRCoordNew = FRCoord + FROffset
    FLCoordNew = FLCoord + FLOffset
    BRCoordNew = BRCoord + BROffset
    BLCoordNew = BLCoord + BLOffset

    FRAngles = np.rad2deg(Inverse_Kinematics.solve_FR(FRCoordNew, coxa, femur, tibia))
    FLAngles = np.rad2deg(Inverse_Kinematics.solve_FL(FLCoordNew, coxa, femur, tibia))
    BRAngles = np.rad2deg(Inverse_Kinematics.solve_BR(BRCoordNew, coxa, femur, tibia)) # Leg orientation ><
    BLAngles = np.rad2deg(Inverse_Kinematics.solve_BL(BLCoordNew, coxa, femur, tibia))

    FRAngles[0] = (FRAngles[0])/36
    FRAngles[1] = FRAngles[1]/36
    FRAngles[2] = FRAngles[2]/36

    FLAngles[0] = (FLAngles[0])/36
    FLAngles[1] = -FLAngles[1]/36
    FLAngles[2] = -FLAngles[2]/36

    BRAngles[0] = (BRAngles[0])/36
    BRAngles[1] = -BRAngles[1]/36
    BRAngles[2] = -BRAngles[2]/36

    BLAngles[0] = (BLAngles[0])/36
    BLAngles[1] = BLAngles[1]/36
    BLAngles[2] = BLAngles[2]/36

    return np.array([[FRAngles], [FLAngles], [BRAngles], [BLAngles]])
