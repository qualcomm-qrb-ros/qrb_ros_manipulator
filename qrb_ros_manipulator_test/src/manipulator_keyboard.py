# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import sys
import threading
import ast
import rclpy
from geometry_msgs.msg import Pose

from qrb_ros_manipulator_msgs.srv import ManipulatorGetControlMode
from qrb_ros_manipulator_msgs.srv import ManipulatorSetControlMode
from qrb_ros_manipulator_msgs.srv import ManipulatorMoveJointPose
from qrb_ros_manipulator_msgs.srv import ManipulatorGetJointPose
from qrb_ros_manipulator_msgs.srv import ManipulatorMoveTcpPose
from qrb_ros_manipulator_msgs.srv import ManipulatorGetTcpPose
from qrb_ros_manipulator_msgs.srv import ManipulatorTargetReachable
from qrb_ros_manipulator_msgs.srv import ManipulatorClawControl
from qrb_ros_manipulator_msgs.srv import ManipulatorClawGetStatus

# from qrb_ros_manipulator_msgs.msg import ManipulatorExceptionMsg


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses to control manipulator. It works best with a US keyboard layout.
---------------------------------------------------------------

1 : get arm control mode (0:offline,1:controlling, 2:teaching)
2 : enable controlling mode
3 : enable teaching mode
4 : arm offline
5 : check arm target reachable[0.128688,-0.337827,0.764262,2.728567,1.360365,1.503259]
6 : move arm tcp[0.133768,-0.269089,0.819442,-0.746853,1.545852,-1.979675]
7 : get arm tcp [x,y,z,rx,ry,rz]
8 : move arm joints [0,0,0,0,0,0]
9 : get arm joints [joint_1,joint_2,joint_3,joint_4,joint_5,joint_6]
q : set claw [50, 50]
w : get claw [force, amplitude]

CTRL-C to quit
"""

ControlModeBindings = {
    '1': (0, 'get arm mode'),
    '2': (1, 'enable controlling mode'),
    '3': (2, 'enable teaching mode'),
    '4': (3, 'enable offline mode'),
    '5': (4, 'check arm target reachable'),
    '6': (5, 'move arm tcp'),
    '7': (6, 'get arm tcp'),
    '8': (7, 'move joints'),
    '9': (8, 'get joints'),
    'q': (9, 'set claw'),
    'w': (10, 'get claw'),
}

ModeBindings = {
    0: 'arm offline',
    1: 'arm controlling',
    2: 'arm teaching',
}

target_reach = [0.128688, -0.337827, 0.764262, 0.0, 0.0, 0.0, 1.0]
target_move_tcp_1 = [-0.124019, -0.208449, 0.716402, 0.683256, 0.158236, 0.138761, 0.699191]
target_move_joint_1 = [-6.315111, -1.602435, 6.294690, -1.492755, 0.021572, 1.860143]


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def input_command():
    try:
        key = input("Please input command to control\n")
        list_info = None
        if key == "5":
            info = input("Please input list,such as: [0, 0, 0, 0.0, 0.0, 0.0, 1.0]\n")
        elif key == "6":
            info = input("Please input list,such as: [0, 0, 0, 0.0, 0.0, 0.0, 1.0]\n")
        elif key == "8":
            info = input("Please input list,such as: [0,0,0,0,0,0]\n")
        elif key == "q":
            info = input("Please input list,[force,amplitude]such as: [50, 50]\n")
        else:
            info = None
        if info:
            try:
                list_info = eval(info)
                for i in range(len(list_info)):
                    list_info[i] = float(list_info[i])
                # print("Input : {} \n".format(list_info))
            except ValueError:
                print("Input not list")
        return key, list_info
    except:
        return '\x03'


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('arm_control_keyboard')

    manipulator_id = 1  # lebai

    namespace = ''
    get_state_client = node.create_client(ManipulatorGetControlMode, '{}manipulator_get_control_mode'.format(namespace))
    control_mode_client = node.create_client(ManipulatorSetControlMode,
                                             '{}manipulator_set_control_mode'.format(namespace))

    move_tcp_client = node.create_client(ManipulatorMoveTcpPose, '{}manipulator_move_tcp_pose'.format(namespace))
    get_tcp_client = node.create_client(ManipulatorGetTcpPose, '{}manipulator_get_tcp_pose'.format(namespace))

    move_joint_client = node.create_client(ManipulatorMoveJointPose, '{}manipulator_move_joint_pose'.format(namespace))
    get_joint_client = node.create_client(ManipulatorGetJointPose, '{}manipulator_get_joint_pose'.format(namespace))

    target_reachable_client = node.create_client(ManipulatorTargetReachable,
                                                 '{}manipulator_target_reachable'.format(namespace))

    set_claw_client = node.create_client(ManipulatorClawControl, '{}manipulator_claw_control'.format(namespace))
    get_claw_status_client = node.create_client(ManipulatorClawGetStatus,
                                                '{}manipulator_claw_get_status'.format(namespace))

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    try:
        print(msg)
        while True:
            print("\n\n------The New round----")
            key = input_command()
            print("key is {} ".format(key))
            if not key:
                continue
            if key[0] in ControlModeBindings.keys():
                # get arm state
                if key[0] == '1':
                    print('get arm mode start... ', end='', flush=True)
                    if not get_state_client.wait_for_service(timeout_sec=3.0):
                        print('state service not available, please try again')
                        continue
                    req = ManipulatorGetControlMode.Request()
                    req.manipulator_id = manipulator_id
                    resp = get_state_client.call(req)
                    print('success, mode: %s' % ModeBindings[resp.result])
                    continue

                # set arm mode controlling
                if key[0] == '2':
                    if not control_mode_client.wait_for_service(timeout_sec=3.0):
                        print('set manipulator mode service not available, please try again')
                        continue
                    req = ManipulatorSetControlMode.Request()
                    req.manipulator_id = manipulator_id
                    req.mode = 1
                    resp = control_mode_client.call(req)
                    print("result =%d." % (resp.result))
                    continue

                # set arm mode teaching
                if key[0] == '3':
                    if not control_mode_client.wait_for_service(timeout_sec=3.0):
                        print('set manipulator mode service not available, please try again')
                        continue
                    req = ManipulatorSetControlMode.Request()
                    req.manipulator_id = manipulator_id
                    req.mode = 2
                    resp = control_mode_client.call(req)
                    print("result =%d." % (resp.result))
                    continue

                # set arm mode controlling
                if key[0] == '4':
                    if not control_mode_client.wait_for_service(timeout_sec=3.0):
                        print('set manipulator mode service not available, please try again')
                        continue
                    req = ManipulatorSetControlMode.Request()
                    req.manipulator_id = manipulator_id
                    req.mode = 0
                    resp = control_mode_client.call(req)
                    print("result =%d." % (resp.result))
                    continue

                # check arm target reachable
                if key[0] == '5':
                    if len(key[1]) != 7:
                        print("Input format is incorrect")
                        continue
                    if not target_reachable_client.wait_for_service(timeout_sec=3.0):
                        print('target_reachable service not available, please try again')
                        continue
                    req = ManipulatorTargetReachable.Request()
                    req.manipulator_id = manipulator_id

                    req.pose.position.x = key[1][0]
                    req.pose.position.y = key[1][1]
                    req.pose.position.z = key[1][2]
                    req.pose.orientation.x = key[1][3]
                    req.pose.orientation.y = key[1][4]
                    req.pose.orientation.z = key[1][5]
                    req.pose.orientation.w = key[1][6]
                    # print("req.pose.position is {}".format(
                    #     [req.pose.position.x, req.pose.position.y, req.pose.position.z, req.pose.orientation.x,
                    #      req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w]))

                    resp = target_reachable_client.call(req)
                    print("success : result=%d." % (resp.result))
                    continue

                # move arm tcp
                if key[0] == '6':
                    if len(key[1]) != 7:
                        print("Input format is incorrect")
                        continue
                    if not move_tcp_client.wait_for_service(timeout_sec=3.0):
                        print('move_tcp service not available, please try again')
                        continue

                    req = ManipulatorMoveTcpPose.Request()
                    req.manipulator_id = manipulator_id

                    req.pose.position.x = key[1][0]
                    req.pose.position.y = key[1][1]
                    req.pose.position.z = key[1][2]
                    req.pose.orientation.x = key[1][3]
                    req.pose.orientation.y = key[1][4]
                    req.pose.orientation.z = key[1][5]
                    req.pose.orientation.w = key[1][6]
                    # print("req.pose.position is {}".format(
                    #     [req.pose.position.x, req.pose.position.y, req.pose.position.z, req.pose.orientation.x,
                    #      req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w]))

                    req.speed = 1.5
                    req.acc = 1.0
                    req.time = 0.0
                    req.radius = 0.0
                    resp = move_tcp_client.call(req)
                    print("success result=%d." % (resp.result))
                    continue

                # get arm tcp
                if key[0] == '7':
                    if not get_tcp_client.wait_for_service(timeout_sec=3.0):
                        print('get tcp service not available, please try again')
                        continue
                    req = ManipulatorGetTcpPose.Request()
                    req.manipulator_id = manipulator_id
                    resp = get_tcp_client.call(req)
                    print(" result=%d." % (resp.result))
                    print("get target x=%f,y=%f,z=%f,rx=%f,ry=%f,rz=%f. rw=%f" % (resp.pose.position.x,
                                                                                  resp.pose.position.y,
                                                                                  resp.pose.position.z,
                                                                                  resp.pose.orientation.x,
                                                                                  resp.pose.orientation.y,
                                                                                  resp.pose.orientation.z,
                                                                                  resp.pose.orientation.w,))
                    continue

                # move arm joints pose
                if key[0] == '8':
                    if len(key[1]) != 6:
                        print("Input format is incorrect")
                        continue
                    if not move_joint_client.wait_for_service(timeout_sec=3.0):
                        print('move_joints service not available, please try again')
                        continue

                    req = ManipulatorMoveJointPose.Request()
                    req.manipulator_id = manipulator_id
                    req.joints_pose = key[1]
                    # print("req.joints_pose type is : {}".format(type(req.joints_pose)))
                    # print("req.joints_pose is : {}".format((req.joints_pose)))
                    req.joints_num = 6
                    req.speed = 1.5
                    req.acc = 1.0
                    req.time = 0.0
                    req.radius = 0.0
                    resp = move_joint_client.call(req)
                    print("result=%d." % (resp.result))
                    continue

                # get arm joints pose
                if key[0] == '9':
                    if not get_joint_client.wait_for_service(timeout_sec=3.0):
                        print('get joints service not available, please try again')
                        continue
                    req = ManipulatorGetJointPose.Request()
                    req.manipulator_id = manipulator_id
                    resp = get_joint_client.call(req)
                    print(" result=%d." % (resp.result))
                    print("get target p1=%f,p2=%f,p3=%f,p4=%f,p5=%f,p6=%f." % (
                        resp.joints_pose[0], resp.joints_pose[1], resp.joints_pose[2], resp.joints_pose[3],
                        resp.joints_pose[4], resp.joints_pose[5]))
                    continue

                # move claw
                if key[0] == 'q':
                    if len(key[1]) != 2:
                        print("Input format is incorrect")
                        continue
                    if not set_claw_client.wait_for_service(timeout_sec=3.0):
                        print('claw control service not available, please try again')
                        continue
                    req = ManipulatorClawControl.Request()
                    req.manipulator_id = manipulator_id
                    # print("Sent req.force is {}, req.amplitude is {}".format((key[1][0]), (key[1][1])))
                    req.force = [key[1][0]]
                    req.amplitude = [key[1][1]]
                    resp = set_claw_client.call(req)
                    print("result=%d." % (resp.result))
                    continue

                # get arm joints pose
                if key[0] == 'w':
                    if not get_claw_status_client.wait_for_service(timeout_sec=3.0):
                        print('get claw service not available, please try again')
                        continue
                    req = ManipulatorClawGetStatus.Request()
                    req.manipulator_id = manipulator_id
                    resp = get_claw_status_client.call(req)
                    print(" result=%d." % (resp.result))
                    print("get target force=%f,amplittude=%f,." % (resp.force[0], resp.amplitude[0]))
                    continue

                # exit
            elif key == '\x03':
                print("exit process.")
                break
            else:
                print("key not right. Won't do anything")

    except Exception as e:
        print(e)
    finally:
        try:
            rclpy.shutdown()
            spinner.join(50)
            restoreTerminalSettings(settings)
        except:
            pass


if __name__ == '__main__':
    main()
