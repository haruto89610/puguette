"""     files imported from ./Puguette/src/new     """

import pybullet as p
import time
import pybullet_data
import numpy as np
import threading
import Gaits
import IK
import Matrices
from XInput import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

position = p.addUserDebugParameter("position" , -0.10 , 0.10 , 0.)
yTrans = p.addUserDebugParameter("y" , -0.10 , 0.10 , 0.)
zTrans = p.addUserDebugParameter("z" , -0.10 , 0.10 , 0.)
rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
pitchId = p.addUserDebugParameter("pitch" , -np.pi/4 , np.pi/4 , 0.)
yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)

cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("Puguette.urdf", cubeStartPos, cubeStartOrientation,
                     # useMaximalCoordinates=1, ## New feature in Pybullet
                     flags=p.URDF_USE_INERTIA_FROM_FILE)

sphereRadius = 0.05
colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[sphereRadius/2, sphereRadius/2, sphereRadius/2])

mass = 500
visualShapeId = -1

link_Masses = [1]
linkCollisionShapeIndices = [colBoxId]
linkVisualShapeIndices = [-1]
linkPositions = [[0, 0, 0.11]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[0, 0, 1]]

for i in range(3):
    for j in range(3):
        for k in range(3):
            basePosition = [
                1 + i * 5 * sphereRadius, 1 + j * 5 * sphereRadius, 1 + k * 5 * sphereRadius + 1
            ]
            baseOrientation = [0, 0, 0, 1]
            if (k & 2):
                sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition,
                                              baseOrientation)
            else:
                sphereUid = p.createMultiBody(mass,
                                              colBoxId,
                                              visualShapeId,
                                              basePosition,
                                              baseOrientation,
                                              linkMasses=link_Masses,
                                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                                              linkVisualShapeIndices=linkVisualShapeIndices,
                                              linkPositions=linkPositions,
                                              linkOrientations=linkOrientations,
                                              linkInertialFramePositions=linkInertialFramePositions,
                                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                                              linkParentIndices=indices,
                                              linkJointTypes=jointTypes,
                                              linkJointAxis=axis)

coords = np.array([[0.00, -0.10, -0.40],   #FR x y z
                   [0.00,  0.10, -0.40],   #FL
                   [0.00, -0.10, -0.40],   #BR
                   [0.00,  0.10, -0.40]])  #BL

rotations = np.array([0.00, 0.00, 0.00]) #roll pitch yaw
translation = np.array([0.00, 0.00, 0.00])

T = 0.00

zTrans = -0.4

L = 0.52
W = 0.15
coxa = 0.186
femur = 0.29
tibia = 0.31

def write(motornum, pos):
    p.setJointMotorControl2(robotId, motornum, p.POSITION_CONTROL, targetPosition = pos, targetVelocity = 0.5, force = 30)

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

    FRCoord = np.array([coord[0][0], coord[0][1], coord[0][2]])
    FLCoord = np.array([coord[1][0], coord[1][1], coord[1][2]])
    BRCoord = np.array([coord[2][0], coord[2][1], coord[2][2]])
    BLCoord = np.array([coord[3][0], coord[3][1], coord[3][2]])

    FRCoordNew = FRCoord + FROffset
    FLCoordNew = FLCoord + FLOffset
    BRCoordNew = BRCoord + BROffset
    BLCoordNew = BLCoord + BLOffset

    FRAngles = np.rad2deg(IK.solve_R(FRCoordNew, coxa, femur, tibia))
    FLAngles = np.rad2deg(IK.solve_L(FLCoordNew, coxa, femur, tibia))
    BRAngles = np.rad2deg(IK.solve_R(BRCoordNew, coxa, femur, tibia))
    BLAngles = np.rad2deg(IK.solve_L(BLCoordNew, coxa, femur, tibia))

    FRAngles[0] = FRAngles[0]
    FLAngles[0] = -FLAngles[0]
    BRAngles[0] = BRAngles[0]
    BLAngles[0] = -BLAngles[0]

    allAnglesString = np.array([[FRAngles], [-FLAngles], [-BRAngles], [BLAngles]])

    return allAnglesString

# currentEventType = None

currentGait = 0

def syncRobotMove():
    T = 0.00

    coords = np.array([[0.00, -0.10, -0.40],   #FR x y z
                       [0.00,  0.10, -0.40],   #FL
                       [0.00, -0.10, -0.40],   #BR
                       [0.00,  0.10, -0.40]])  #BL

    rotations = np.array([0.00, 0.00, 0.00]) #roll pitch yaw
    translation = np.array([0.00, 0.00, 0.00])

    newCoords = coords
    newRotations = rotations
    newTranslation = translation
    # myEventType = currentEventType
    while True:
        # print(newCoords)
        #FR
        write(0, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[0][0][0]))
        write(1, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[0][0][1]))
        write(2, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[0][0][2]))
        #FL
        write(3, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[1][0][0]))
        write(4, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[1][0][1]))
        write(5, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[1][0][2]))
        #BR
        write(9, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[1][0][0]))
        write(10, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[2][0][1]))
        write(11, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[2][0][2]))
        #BL
        write(6, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[3][0][0]))
        write(7, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[3][0][1]))
        write(8, np.deg2rad(Kinematic_Model(newCoords, newRotations, newTranslation)[3][0][2]))

        # Update newCoords
        #if myEventType == currentEventType:
        if currentGait == 0:
            coords = np.array([[0.00, -0.10, -0.40],   #FR x y z
                               [0.00,  0.10, -0.40],   #FL
                               [0.00, -0.10, -0.40],   #BR
                               [0.00,  0.10, -0.40]])  #BL

            rotations = np.array([0.00, 0.00, 0.00]) #roll pitch yaw
            translation = np.array([0.00, 0.00, 0.00])

            newCoords = coords
            newRotations = rotations
            newTranslation = translation
        elif currentGait == 1:
            if T <= 0:
                T = 2
            elif T <= 1:
                coords[0][0], coords[0][2] = Gaits.bezier(T) #FR
                coords[0][2] = coords[0][2]

                coords[3][0], coords[3][2] = Gaits.bezier(1-T) #BL
                coords[3][2] = coords[3][2]

                coords[1][0], coords[1][2] = -(0.1 * (T) - 0.05), -0.4 #FL

                coords[2][0], coords[2][2] = 0.1 * (T) - 0.05, -0.4 #BR

                # translation[0] = 0.1*(T-0.5)
                T += speed
            elif T > 1 and T <= 2:
                coords[1][0], coords[1][2] = Gaits.bezier(T-1)
                coords[1][2] = coords[1][2]

                coords[2][0], coords[2][2] = Gaits.bezier(2-T) #BR
                coords[2][2] = coords[2][2]

                coords[0][0], coords[0][2] = -(1/15 * T - 1/10), -0.4 #FR

                coords[3][0], coords[3][2] = 1/15 * T - 1/10, -0.4

                # translation[0] = 0.1*(-T+1.5)
                T += speed
            elif T > 2:
                T -= 2
        elif currentGait == 2:
            rotations[2] = yaw
            rotations[1] = pitch
        elif currentGait == 5:
            newTranslation[2] = l_trigger_pos
        elif currentGait == 6:
            newTranslation[2] = r_trigger_pos
        elif currentGait == 7:
            newRotations[0] = a
        elif currentGait == 8:
            newRotations[0] = b


        # elif currentGait == 3:
        #     newTranslation[2] =
        # elif currentGait == 4:
        #     newTranslation[2] =

        newCoords = coords
        newRotations = rotations
        newTranslation = translation

robotController = threading.Thread(target=syncRobotMove)
robotController.start()

while True:
    for event in get_events():
        currentEventType = event
        if event.type == EVENT_BUTTON_PRESSED:
            if event.button == "A" and translation[2] <= 0.1:
                currentGait = 3
                coords[2] += 0.008

                print(translation[2])
                time.sleep(0.1)

            elif event.button == "B" and translation[2] >= -0.1:
                currentGait = 4
                coords[2] -= 0.008

                print(translation[2])
                time.sleep(0.1)

            elif event.button == "LEFT_SHOULDER":
                currentGait = 7
                a = -1
            elif event.button == "RIGHT_SHOULDER":
                currentGait = 8
                b = 1

        elif event.type == EVENT_STICK_MOVED:
            if event.stick == LEFT:
                currentGait = 1
                speed = get_thumb_values(get_state(event.user_index))[0][1] / 20
            if event.stick == RIGHT:
                currentGait = 2
                yaw = get_thumb_values(get_state(event.user_index))[1][0] / 2
                pitch = get_thumb_values(get_state(event.user_index))[1][1] / 2

        elif event.type == EVENT_TRIGGER_MOVED:
            if event.trigger == LEFT:
                currentGait = 5
                l_trigger_pos = -round(get_trigger_values(get_state(event.user_index))[0] / 10, 2)
            elif event.trigger == RIGHT:
                currentGait = 6
                r_trigger_pos = round(get_trigger_values(get_state(event.user_index))[1] / 10, 2)

        else:
            currentGait = 0

robotController.join()
