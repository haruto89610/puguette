import sys
import odrive
import threading
import can
import struct
import socket
import time
import numpy as np
from odrive.enums import *
from enum import Enum

from Gaits import Walk
from Gaits.Walk import ratio, xPoints, zPoints
from Kinematics import Model

struct_format = ">6f14?"

can_front = can.interface.Bus(channel='can0', bustype='socketcan')
can_back = can.interface.Bus(channel='can1', bustype='socketcan')

class Commands(Enum): #ODrive CAN cmd_id
    EMERGENCY_STOP = 0x002
    MOTOR_ERROR = 0x003
    ENCODER_ERROR = 0x004
    AXIS_REQUESTED_STATE = 0x007
    ENCODER_COUNT = 0x00A
    INPUT_POS = 0x00C
    REBOOT = 0x016
    VBUS_VOLTAGE = 0x017
    CLEAR_ERRORS = 0x018
    LINEAR_COUNT = 0x019

class State(Enum): #ODrive CAN data
    IDLE = 1
    FULL_CALIBRATION_SEQUENCE = 3
    CLOSED_LOOP_CONTROL = 8

class Joint(Enum): #Axis ID
    FL_hip = 0
    FL_upper = 1
    FL_lower = 2
    FR_hip = 3
    FR_upper = 4
    FR_lower = 5
    BL_hip = 6
    BL_upper = 7
    BL_lower = 8
    BR_hip = 9
    BR_upper = 10
    BR_lower = 11

def odrive_can_cmd(node_id, cmd_id, data=[], format=''): #convert to arbitration ID
    data_frame = struct.pack(format, *data)
    msg = can.Message(arbitration_id=((node_id << 5) | cmd_id), data=data_frame, is_extended_id=False)
#    if node_id < 6:
    can_front.send(msg)
#    elif node_id >= 6:
    can_back.send(msg)

"""
def odrive_getPos(node_id):
    while(1):
        msg = can_bus.recv()
        if(msg.arbitration_id == (node_id << 5) + 0x9):
            data = struct.unpack('ff', msg.data)
            return data[0]
"""

controller_state_available = threading.Event()

def socket_server(): #set up server
    global controller_state
    HOST = "10.0.0.8"
    PORT = 65432
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            previous_controller_state = 0
            while True:
                data = conn.recv(4096)
                if len(data) == struct.calcsize(struct_format):
                    controller_state = struct.unpack(struct_format, data)
                    previous_controller_state = controller_state
                    controller_state_available.set()
                else:
                    controller_state = previous_controller_state
                    controller_state_available.set()


def main():
    T = 0.00
    trot_state = 0
    coords = np.array([[0.00, -0.06, -0.40],
                       [0.00,  0.06, -0.40],
                       [0.00, -0.06, -0.40],
                       [0.00,  0.06, -0.40]])
    rotations = np.array([0.00, 0.00, 0.00])
    translations = np.array([0.00, 0.00, 0.00])

    speed, angle, pitch, roll, yaw = 0.00, 0.00, 0.00, 0.00, 0.00
    while True:
        controller_state_available.wait()
        time.sleep(0.01)
        speed = controller_state[0]
        angle = controller_state[1]
        pitch = controller_state[2]
        roll = controller_state[3]
        yaw = controller_state[4] + controller_state[5]

        linear_count_check = 0

        for member in list(Joint):

#            print(f"received {T, controller_state[0], Model.Kinematic_Model(coords, rotations, translations)[int(member.value/3)][0][member.value%3]}")
            #-------------CONTROLLER STATES-------------#

            if controller_state[12] == True:
#                #FL
                odrive_can_cmd(Joint.FL_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.FL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.FL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
#                #FR
                odrive_can_cmd(Joint.FR_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.FR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.FR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
#                #BL
                odrive_can_cmd(Joint.BL_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.BL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.BL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
#                #BR
                odrive_can_cmd(Joint.BR_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.BR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                odrive_can_cmd(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.IDLE.value], 'h')
                print(member.value)
            elif controller_state[13] == True:
#                #FL
                odrive_can_cmd(Joint.FL_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.FL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.FL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
#                #FR
                odrive_can_cmd(Joint.FR_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.FR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.FR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
#                #BL
                odrive_can_cmd(Joint.BL_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.BL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.BL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
#                #BR
                odrive_can_cmd(Joint.BR_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.BR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                odrive_can_cmd(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.CLOSED_LOOP_CONTROL.value], 'h')
                print(member.value)
            elif controller_state[14] == True:
#                #FL
                odrive_can_cmd(Joint.FL_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.FL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.FL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
#                #FR
                odrive_can_cmd(Joint.FR_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.FR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.FR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
#                #BL
                odrive_can_cmd(Joint.BL_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.BL_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.BL_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
#                #BR
                odrive_can_cmd(Joint.BR_hip.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.BR_upper.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                odrive_can_cmd(Joint.BR_lower.value, Commands.AXIS_REQUESTED_STATE.value, [State.FULL_CALIBRATION_SEQUENCE.value], 'h')
                print(member.value)

            #----------------CALIBRATION----------------#
            elif controller_state[19] == True:
                print("hip calibration")
                odrive_can_cmd(Joint.FL_hip.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.FR_hip.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.BL_hip.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.BR_hip.value, Commands.LINEAR_COUNT.value, [0], 'h')
#                for i in range(3):
#                    #HIPS
#                    odrive_can_cmd(3*i, Commands.LINEAR_COUNT.value, [0], 'h')
#                    i+=1
            elif controller_state[17] == True:
                odrive_can_cmd(Joint.FL_upper.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.FR_upper.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.BL_upper.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.BR_upper.value, Commands.LINEAR_COUNT.value, [0], 'h')
                print("upper leg calibration")
#                for i in range(3):
#                    odrive_can_cmd(3*i+1, Commands.LINEAR_COUNT.value, [0], 'h')
#                    i+=1
            elif controller_state[16] == True:
                print("lower leg calibration")
                odrive_can_cmd(Joint.FL_lower.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.FR_lower.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.BL_lower.value, Commands.LINEAR_COUNT.value, [0], 'h')
                odrive_can_cmd(Joint.BR_lower.value, Commands.LINEAR_COUNT.value, [0], 'h')
#                for i in range(3):
#                    odrive_can_cmd(3*i+2, Commands.LINEAR_COUNT.value, [0], 'h')
#                    i+=1

            #-------------------GAITS-------------------#
            elif speed != 0 or angle != 0:
                if trot_state == 0:
                    coords[0][2], coords[3][2] = -0.33, -0.33
                    coords[1][2], coords[2][2] = -0.4, -0.4

                    coords[0][1], coords[1][1], coords[2][1], coords[3][1] = -0.06, 0.06, -0.06, 0.06
                    trot_state+=1
                    time.sleep(0.2)
                elif trot_state == 1:
                    coords[1][2], coords[2][2] = -0.33, -0.33
                    coords[0][2], coords[3][2] = -0.4, -0.4

                    coords[0][1], coords[1][1], coords[2][1], coords[3][1] = -0.06, 0.06, -0.06, 0.06
                    trot_state=0
                    time.sleep(0.2)
            elif pitch != 0 or roll != 0 or yaw != 0:
                rotations[0] = roll
                rotations[1] = pitch
                rotations[2] = yaw

            else:
                coords = np.array([[0.00, -0.06, -0.40],
                                   [0.00,  0.06, -0.40],
                                   [0.00, -0.06, -0.40],
                                   [0.00,  0.06, -0.40]])
                rotations = np.array([0.00, 0.00, 0.00])
                translations = np.array([0.00, 0.00, 0.00])
            print(coords)
#            #FL
            odrive_can_cmd(Joint.FL_hip.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[1][0][0], 0, 0], 'fhh')
            odrive_can_cmd(Joint.FL_upper.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[1][0][1], 0, 0], 'fhh')
            odrive_can_cmd(Joint.FL_lower.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[1][0][2], 0, 0], 'fhh')
#            #FR
            odrive_can_cmd(Joint.FR_hip.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[0][0][0], 0, 0], 'fhh')
            odrive_can_cmd(Joint.FR_upper.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[0][0][1], 0, 0], 'fhh')
            odrive_can_cmd(Joint.FR_lower.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[0][0][2], 0, 0], 'fhh')
#            #BL
            odrive_can_cmd(Joint.BL_hip.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[3][0][0], 0, 0], 'fhh')
            odrive_can_cmd(Joint.BL_upper.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[3][0][1], 0, 0], 'fhh')
            odrive_can_cmd(Joint.BL_lower.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[3][0][2], 0, 0], 'fhh')
#            #BR
            odrive_can_cmd(Joint.BR_hip.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[2][0][0], 0, 0], 'fhh')
            odrive_can_cmd(Joint.BR_upper.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[2][0][1], 0, 0], 'fhh')
            odrive_can_cmd(Joint.BR_lower.value, Commands.INPUT_POS.value, [Model.Kinematic_Model(coords, rotations, translations)[2][0][2], 0, 0], 'fhh')
#            print(Model.Kinematic_Model(coords, rotations, translations)[2][0][0])
        controller_state_available.clear()

main_thread = threading.Thread(target=main)
server_thread = threading.Thread(target=socket_server)
main_thread.start()
server_thread.start()
