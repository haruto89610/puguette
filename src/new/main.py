from XInput import *
import numpy as np
import odrive
from odrive.enums import *

from Gaits import *
import KinematicModel
import IK

FR = odrive.find_any(serial_number='207D348C5748')
FL = odrive.find_any(serial_number='2074347E5748')
BR = odrive.find_any(serial_number='207D34885748')
BL = odrive.find_any(serial_number='207D34945748')

# frontHip = odrive.find_any(serial_number=)
# backHip = odrive.find_any(serial_number='206F37915753')

# https://stackoverflow.com/questions/8508799/bit-masking-in-python

zTrans = 0
T = 0.00

coords = np.array([[0.00, -0.18, -0.20],   #FR x y z
                   [0.00,  0.18, -0.20],   #FL
                   [0.00, -0.18, -0.20],   #BR
                   [0.00,  0.18, -0.20]])  #BL

rotations = np.array([0.00, 0.00, 0.00]) #roll pitch yaw
translation = np.array([0.00, 0.00, 0.00])

while True:
    for event in get_events():
        if event.type == EVENT_BUTTON_PRESSED:

            if event.button == "BACK":
                print('Shutting Motors Off...')

                FR.axis0.requested_state = AXIS_STATE_IDLE
                FR.axis1.requested_state = AXIS_STATE_IDLE

                FL.axis0.requested_state = AXIS_STATE_IDLE
                FL.axis1.requested_state = AXIS_STATE_IDLE

                BR.axis0.requested_state = AXIS_STATE_IDLE
                BR.axis1.requested_state = AXIS_STATE_IDLE
                #
                BL.axis0.requested_state = AXIS_STATE_IDLE
                BL.axis1.requested_state = AXIS_STATE_IDLE

                # backHip.axis0.requested_state = AXIS_STATE_IDLE
                # backHip.axis1.requested_state = AXIS_STATE_IDLE

                time.sleep(0.1)

            elif event.button == "START":
                print('Starting Motors...')

                FR.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                FR.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                FL.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                FL.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                BR.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                BR.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                #
                BL.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                BL.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                # backHip.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                # backHip.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

                time.sleep(0.1)

            elif event.button == "A" and zTrans <= 0.15:
                print("A")
                zTrans += 0.008
                # translation[2] += 0.008
                coords[0][2] += 0.008
                coords[1][2] += 0.008
                coords[2][2] += 0.008
                coords[3][2] += 0.008

                print(coords[0][2], zTrans)
                time.sleep(0.1)

            elif event.button == "B" and zTrans >= -0.15:
                print("B")
                zTrans -= 0.008
                # translation[2] -= 0.008
                coords[0][2] -= 0.008
                coords[1][2] -= 0.008
                coords[2][2] -= 0.008
                coords[3][2] -= 0.008

                print(coords[0][2], zTrans)
                time.sleep(0.1)

            while event.button == "X":
                print("Move in Place")

                coords = np.array([[0.00, -0.18, -0.15],   #FR x y z
                                   [0.00,  0.18, -0.20],   #FL
                                   [0.00, -0.18, -0.20],   #BR
                                   [0.00,  0.18, -0.15]])  #BL

                time.sleep(0.5)

                coords = np.array([[0.00, -0.18, -0.20],   #FR x y z
                                   [0.00,  0.18, -0.15],   #FL
                                   [0.00, -0.18, -0.15],   #BR
                                   [0.00,  0.18, -0.20]])  #BL

                time.sleep(0.5)

            else:
                break

        elif event.type == EVENT_STICK_MOVED:
            if event.stick == LEFT:
                speed = get_thumb_values(get_state(event.user_index))[0][1] / 20
                print("Move At Speed: " + str(speed))
                if T <= 0:
                    T = 2
                elif T <= 1:
                    coords[0][0], coords[0][2] = bezier(T) #FR
                    coords[0][2] = coords[0][2] + zTrans

                    coords[3][0], coords[3][2] = bezier(T) #BL
                    coords[3][2] = coords[3][2] + zTrans

                    coords[1][0], coords[1][2] = (-0.4/(2-T) + 0.3), -0.4 + zTrans #FL

                    coords[2][0], coords[2][2] = (-0.4/(2-T) + 0.3), -0.4 + zTrans #BR

                    FR.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[0][0][1]
                    FR.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[0][0][2]

                    FL.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[1][0][1]
                    FL.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[1][0][2]

                    BR.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[2][0][1]
                    BR.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[2][0][2]

                    BL.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[3][0][1]
                    BL.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[3][0][2]

                    time.sleep(0.001)
                    T += speed
                elif T > 1 and T <= 2:
                    coords[1][0], coords[1][2] = bezier(T-1)
                    coords[1][2] = coords[1][2] + zTrans

                    coords[2][0], coords[2][2] = bezier(T-1) #FR
                    coords[2][2] = coords[2][2] + zTrans

                    coords[0][0], coords[0][2] = 0.4/T - 0.3, -0.4 + zTrans #BR

                    coords[3][0], coords[3][2] = 0.4/T - 0.3, -0.4 + zTrans #BR

                    FR.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[0][0][1]
                    FR.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[0][0][2]

                    FL.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[1][0][1]
                    FL.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[1][0][2]

                    BR.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[2][0][1]
                    BR.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[2][0][2]

                    BL.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[3][0][1]
                    BL.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[3][0][2]

                    time.sleep(0.001)
                    T += speed
                elif T > 2:
                    T -= 2

            while event.stick == RIGHT:
                yaw = get_thumb_values(get_state(event.user_index))[1][0]
                pitch = get_thumb_values(get_state(event.user_index))[1][1]

                print("Rotation By: Yaw: " + str(yaw) + " Pitch: " + str(pitch) + " Roll: ")
                rotations[0] = pitch
                rotations[2] = yaw
                # backHip.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[2][0][0]
                # backHip.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[3][0][0]

                # FL.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[1][0][1]
                # FL.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords, rotations, translation)[1][0][2]

            else:
                break

        else:
            print("Waiting For Command...")

''' # FOR TESTING INVERSE KINEAMATICS
    events = get_events()
    for event in events:

        coords[0][0], coords[0][2] = get_thumb_values(get_state(event.user_index))[1][0]/8, -0.3 + get_thumb_values(get_state(event.user_index))[1][1]/8

        odrv0.axis0.controller.input_pos = KinematicModel.Kinematic_Model(coords,rotations,translation)[0][0][1]
        odrv0.axis1.controller.input_pos = KinematicModel.Kinematic_Model(coords,rotations,translation)[0][0][2]

        time.sleep(0.)
        print(coords[0], KinematicModel.Kinematic_Model(coords,rotations,translation)[0][0][1], KinematicModel.Kinematic_Model(coords,rotations,translation)[0][0][2])
'''
