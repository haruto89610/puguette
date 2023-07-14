import numpy as np

def checkdomain(D):
    if D > 1 or D < -1:
#        print("OUT OF DOMAIN")
        if D > 1:
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D

"""
"z                          "
" |                         "
" |                         "
" |                         "
" |                         "
" |     / y                 "
" |    /                    "
" |   /                     "
" |  /                      "
" | /                       "
" |/____________________ x  "
"                           "
" theta - hip abductor      "
" alpha - upper leg         "
" gamma - lower leg         "
"                           "
" zero pos                  "
"  0---0                    "
"  |                        "
"  |                        "
"  0                        "
"  |                        "
"  |                        "
"  |                        "
"                           "
"""

def solve_FR(coord, coxa, femur, tibia):
    D = (coord[1]**2+coord[2]**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1-D**2), D)
    theta = -np.arctan2(coord[2], coord[1]) - np.arctan2(np.sqrt(coord[1]**2+coord[2]**2-coxa**2), -coxa)
    alpha = np.arctan2(-coord[0], np.sqrt(coord[1]**2+coord[2]**2-coxa**2)) - np.arctan2(tibia*np.sin(gamma), femur+tibia*np.cos(gamma))
    return np.array([theta, alpha, gamma])

def solve_FL(coord, coxa, femur, tibia):
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1-D**2), D)
    theta = -np.arctan2(coord[2], coord[1]) - np.arctan2(np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2), coxa)
    alpha = np.arctan2(-coord[0], np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2)) - np.arctan2(tibia*np.sin(gamma), femur+tibia*np.cos(gamma))
    return np.array([theta, alpha, gamma])

def solve_BR(coord, coxa, femur, tibia):
    D = (coord[1]**2+coord[2]**2-coxa**2+coord[0]**2-femur**2-tibia**2)/(2*tibia*femur)
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1-D**2), D)
    theta = -np.arctan2(coord[2], coord[1]) - np.arctan2(np.sqrt(coord[1]**2+coord[2]**2-coxa**2), -coxa)
    alpha = np.arctan2(-coord[0], np.sqrt(coord[1]**2+coord[2]**2-coxa**2)) - np.arctan2(tibia*np.sin(gamma), femur+tibia*np.cos(gamma))
    return np.array([-theta, alpha, gamma])

def solve_BL(coord, coxa, femur, tibia):
    D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1-D**2), D)
    theta = -np.arctan2(coord[2], coord[1]) - np.arctan2(np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2), coxa)
    alpha = np.arctan2(-coord[0], np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2)) - np.arctan2(tibia*np.sin(gamma), femur+tibia*np.cos(gamma))
    return np.array([-theta, alpha, gamma])
