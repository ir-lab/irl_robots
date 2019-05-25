#!/usr/bin/env python

import datetime
import numpy as np
import rospy
import subprocess
import sys
import time

from irl_robots.msg import ur5Control
from irl_robots.msg import ur5Joints
from irl_robots.msg import ur5Tool
from sensor_msgs.msg import Joy

import kdl_parser_py.urdf
import PyKDL as kdl

# ***** NOTE *****
# I modified the URDF to introduce a yaw offset to the world_joint of 0.7854.
# This aligns the UR5 with the orientation of the physical robot in our lab.

VERBOSE = False

# Joy message axes mappings for Dualshock 4 controller
LEFT_X = 0
LEFT_Y = 1
RIGHT_X = 3
RIGHT_Y = 4
DPAD_X = 6
DPAD_Y = 7
LEFT_TRIGGER = 2
RIGHT_TRIGGER = 5

# Buttons
X_BUTTON = 0
CIRCLE_BUTTON = 1
TRIANGLE_BUTTON = 2
SQUARE_BUTTON = 3
L1_BUTTON = 4
R1_BUTTON = 5
L2_BUTTON = 6
R2_BUTTON = 7
START = 9
SELECT = 8
L3_BUTTON = 11
R3_BUTTON = 12
PS_BUTTON = 10

# Axis ranges
STICK_MAX_VALUE = 1.0
STICK_MIN_VALUE = -1.0
TRIGGER_MAX_VALUE = 1.0
TRIGGER_MIN_VALUE = -1.0

# Gamepad thumbstick deadzone
STICK_DEADZONE = 0.15

CONTROL_FREQUENCY = 30
NUM_JOINTS = 6
MAX_CARTESIAN_ACCELERATION = 8.0
MIN_CARTESIAN_ACCELERATION = -8.0
MAX_CARTESIAN_VELOCITY = 5.0
MAX_ANGULAR_ACCELERATION = 13.0
MIN_ANGULAR_ACCELERATION = -8.0
MAX_ANGULAR_VELOCITY = 7.0
MAX_RADIUS_ACCELERATION = 8.0
MIN_RADIUS_ACCELERATION = -5.0
MAX_RADIUS_VELOCITY = 2.5

# CONTROL_MODE = 0 # X/Y Grid
CONTROL_MODE = 1 # Radial

ONE_HANDED = True

class UR5Teleop:
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.joint_state = None
        self.tool_state = None

        self.joint_command = ur5Control()
        self.joint_command.acceleration = 5.0
        self.joint_command.blend = 0
        self.joint_command.command = "speedj"
        self.joint_command.gain = 0
        self.joint_command.jointcontrol = True
        self.joint_command.lookahead = 0
        self.joint_command.time = 1.0 / CONTROL_FREQUENCY
        self.joint_command.velocity = 0

        # Desired control velocity in x, y, z.
        self.desired_angular_velocity = 0.0
        self.desired_radius_velocity = 0.0
        self.desired_cartesian_velocity = [0.0, 0.0, 0.0]
        self.desired_wrist1_velocity = 0.0
        self.desired_wrist2_velocity = 0.0
        self.desired_wrist3_velocity = 0.0

        self.current_angular_velocity = 0.0
        self.current_radius_velocity = 0.0
        self.current_cartesian_velocity = [0.0, 0.0, 0.0]

        self.ur5_kdl_tree = kdl_parser_py.urdf.treeFromFile("/home/local/ASUAD/jacampb1/Downloads/ur_description/urdf/ur5_robot.urdf")[1]
        self.ur5_transform = self.ur5_kdl_tree.getChain("world", "ee_link")
        self.kdl_fk = kdl.ChainFkSolverPos_recursive(self.ur5_transform)
        self.kdl_ik = kdl.ChainIkSolverPos_LMA(self.ur5_transform)
        self.kdl_initial = kdl.JntArray(NUM_JOINTS)
        self.kdl_joints = kdl.JntArray(NUM_JOINTS)
        self.kdl_effector = kdl.Frame()

        # subscribed Topic
        self.gamepad_sub = rospy.Subscriber("/joy", Joy, self.gamepad_callback,  queue_size = 1)
        self.ur5_joint_sub = rospy.Subscriber("/ur5/joints", ur5Joints, self.ur5_joint_callback, queue_size = 1)
        # self.ur5_tool_sub = rospy.Subscriber("/ur5/tool", ur5Tool, self.ur5_tool_callback, queue_size = 1)
        self.ur5_command_pub = rospy.Publisher("/ur5/control", ur5Control, queue_size = 1)



        # time.sleep(3)
        # self.go_to_initial()

    def go_to_initial(self):
        self.joint_command.values = [0.57988, -0.99270, 2.03353, -0.96921, 1.40923, 1.5766]

        rate = rospy.Rate(CONTROL_FREQUENCY)
        while not rospy.is_shutdown() and not np.allclose(self.joint_command.values, self.joint_state.positions, atol=1e-02):
            self.ur5_command_pub.publish(self.joint_command)

    def ur5_joint_callback(self, message):
        self.joint_state = message

    # def ur5_tool_callback(self, message):
    #     self.tool_state = message

    def publish_joint_command(self):
        if(self.joint_state is not None):
            for idx in range(NUM_JOINTS):
                self.kdl_joints[idx] = self.joint_state.positions[idx]

            self.kdl_fk.JntToCart(self.kdl_joints, self.kdl_effector)
            self.tool_state = self.kdl_effector.p

        if(self.tool_state is not None and self.joint_state is not None):
            if(CONTROL_MODE == 0):
                for idx in range(3):
                    self.tool_state[idx] += self.desired_cartesian_velocity[idx] / CONTROL_FREQUENCY
            if(CONTROL_MODE == 1):
                if(self.desired_angular_velocity - self.current_angular_velocity > MAX_ANGULAR_ACCELERATION / CONTROL_FREQUENCY):
                    self.current_angular_velocity += MAX_ANGULAR_ACCELERATION / CONTROL_FREQUENCY
                elif(self.desired_angular_velocity - self.current_angular_velocity < MIN_ANGULAR_ACCELERATION / CONTROL_FREQUENCY):
                    self.current_angular_velocity += MIN_ANGULAR_ACCELERATION / CONTROL_FREQUENCY
                else:
                    self.current_angular_velocity = self.desired_angular_velocity

                if(self.desired_radius_velocity - self.current_radius_velocity > MAX_RADIUS_ACCELERATION / CONTROL_FREQUENCY):
                    self.current_radius_velocity += MAX_RADIUS_ACCELERATION / CONTROL_FREQUENCY
                elif(self.desired_radius_velocity - self.current_radius_velocity < MIN_RADIUS_ACCELERATION / CONTROL_FREQUENCY):
                    self.current_radius_velocity += MIN_RADIUS_ACCELERATION / CONTROL_FREQUENCY
                else:
                    self.current_radius_velocity = self.desired_radius_velocity

                if(self.desired_cartesian_velocity[2] - self.current_cartesian_velocity[2] > MAX_CARTESIAN_ACCELERATION / CONTROL_FREQUENCY):
                    self.current_cartesian_velocity[2] += MAX_CARTESIAN_ACCELERATION / CONTROL_FREQUENCY
                elif(self.desired_cartesian_velocity[2] - self.current_cartesian_velocity[2] < MIN_CARTESIAN_ACCELERATION / CONTROL_FREQUENCY):
                    self.current_cartesian_velocity[2] += MIN_CARTESIAN_ACCELERATION / CONTROL_FREQUENCY
                else:
                    self.current_cartesian_velocity[2] = self.desired_cartesian_velocity[2]

                # Finish doing this and add min acceleration. So then we can have separate max/min.

                # Calculate current radius of arm.
                radius = np.hypot(self.tool_state[0], self.tool_state[1])
                # Calculate current angle from origin
                current_angle = np.arctan2(self.tool_state[1], self.tool_state[0])
                # Calculate angle to desired point
                desired_angle = current_angle + ((self.current_angular_velocity / CONTROL_FREQUENCY) / radius)
                # Calculate desired radius to desired point
                desired_radius = radius + (self.current_radius_velocity / CONTROL_FREQUENCY)
                # Compute desired point
                self.tool_state[0] = desired_radius * np.cos(desired_angle)
                self.tool_state[1] = desired_radius * np.sin(desired_angle)
                self.tool_state[2] += self.current_cartesian_velocity[2] / CONTROL_FREQUENCY

            self.kdl_effector.p = self.tool_state
            for idx in range(NUM_JOINTS):
                self.kdl_initial[idx] = self.joint_state.positions[idx]
            self.kdl_ik.CartToJnt(self.kdl_initial, self.kdl_effector, self.kdl_joints)

            print(self.kdl_joints)

            for idx in range(NUM_JOINTS):
                self.joint_command.values[idx] = self.kdl_joints[idx]

            if(not np.allclose(self.desired_wrist1_velocity, 0.0)):
                self.joint_command.values[3] = self.joint_state.positions[3] + (self.desired_wrist1_velocity / CONTROL_FREQUENCY)
            # if(not np.allclose(self.desired_wrist2_velocity, 0.0)):
            self.joint_command.values[4] = self.joint_state.positions[4] + (self.desired_wrist2_velocity / CONTROL_FREQUENCY)
            if(not np.allclose(self.desired_wrist3_velocity, 0.0)):
                self.joint_command.values[5] = self.joint_state.positions[5] + (self.desired_wrist3_velocity / CONTROL_FREQUENCY)

            self.ur5_command_pub.publish(self.joint_command)

    def gamepad_callback(self, message):
        if(self.tool_state is None and self.joint_state is not None):
            for idx in range(NUM_JOINTS):
                self.kdl_joints[idx] = self.joint_state.positions[idx]

            self.kdl_fk.JntToCart(self.kdl_joints, self.kdl_effector)
            self.tool_state = self.kdl_effector.p

        if(CONTROL_MODE == 0):
            self.desired_cartesian_velocity[0] = (message.axes[LEFT_X] / STICK_MAX_VALUE) * MAX_CARTESIAN_VELOCITY
            self.desired_cartesian_velocity[1] = -(message.axes[LEFT_Y] / STICK_MAX_VALUE) * MAX_CARTESIAN_VELOCITY
            self.desired_cartesian_velocity[2] = (message.buttons[L1_BUTTON] * MAX_CARTESIAN_VELOCITY) - (message.buttons[L2_BUTTON] * MAX_CARTESIAN_VELOCITY)
            self.desired_wrist1_velocity = (message.axes[RIGHT_Y] / STICK_MAX_VALUE) * MAX_CARTESIAN_VELOCITY
            self.desired_wrist2_velocity = (message.axes[RIGHT_X] / STICK_MAX_VALUE) * MAX_CARTESIAN_VELOCITY
        elif(CONTROL_MODE == 1):

            self.desired_angular_velocity = -(message.axes[LEFT_X] / STICK_MAX_VALUE) * MAX_ANGULAR_VELOCITY
            self.desired_radius_velocity = (message.axes[LEFT_Y] / STICK_MAX_VALUE) * MAX_RADIUS_VELOCITY

            self.desired_cartesian_velocity[2] = (message.buttons[L1_BUTTON] * MAX_CARTESIAN_VELOCITY) - (message.buttons[L2_BUTTON] * MAX_CARTESIAN_VELOCITY)
            self.desired_wrist1_velocity = -(message.axes[RIGHT_Y] / STICK_MAX_VALUE) * MAX_ANGULAR_VELOCITY
            self.desired_wrist2_velocity = (message.axes[RIGHT_X] / STICK_MAX_VALUE) * MAX_ANGULAR_VELOCITY
            self.desired_wrist3_velocity = (message.buttons[R1_BUTTON] * MAX_ANGULAR_VELOCITY) - (message.buttons[R2_BUTTON] * MAX_ANGULAR_VELOCITY)

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("ur5_teleop_node")

    talker = UR5Teleop()

    rate = rospy.Rate(CONTROL_FREQUENCY) # 10hz
    while not rospy.is_shutdown():
        talker.publish_joint_command()
        rate.sleep()
    #try:
    #    rospy.spin()
    #except KeyboardInterrupt:
    #    print "Shutting down ROS gamepad talker module"

if __name__ == '__main__':
    main(sys.argv)
