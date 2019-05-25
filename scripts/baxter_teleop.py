#!/usr/bin/env python

import numpy as np
import scipy.spatial
import rospy
import baxter_interface
import sys

from geometry_msgs.msg import PoseStamped

CONTROL_FREQUENCY = 20.0

SIDES = ["left", "right"]
LIMBS = ["hand", "lowerarm", "upperarm", "chest"]
FRAME_OF_REFERENCE = "chest"

TOPICS = [
    ("/vrpn_client_node/teleop_left_hand/pose", "left", "hand"),
    ("/vrpn_client_node/teleop_left_lowerarm/pose", "left", "lowerarm"),
    ("/vrpn_client_node/teleop_left_upperarm/pose", "left", "upperarm"),
    ("/vrpn_client_node/teleop_right_hand/pose", "right", "hand"),
    ("/vrpn_client_node/teleop_right_lowerarm/pose", "right", "lowerarm"),
    ("/vrpn_client_node/teleop_right_upperarm/pose", "right", "upperarm"),
    ("/vrpn_client_node/teleop_chest/pose", "all", "chest")
]

TRACKING_LIMITS = {
    "left" : {
        "elbow" : [0.6, 1.7],
        "wrist" : [0.8, 0.3],
        "shoulder_leftright" : [-4.0, -2.0],
        "shoulder_updown" : [1.0, 0.0]
    },
    "right" : {
        "elbow" : [0.6, 1.9],
        "wrist" : [0.5, 1.3],
        "shoulder_leftright" : [-0.9, 0.6],
        "shoulder_updown" : [0.8, 1.2]
    }
}

BAXTER_LIMITS = {
    "left" : {
        "elbow" : [-0.05, 1.93],
        "wrist" : [0.11, 1.14],
        "shoulder_leftright" : [-1.3, 0.8],
        "shoulder_updown" : [-0.3, 0.9]
    },
    "right" : {
        "elbow" : [-0.05, 1.93],
        "wrist" : [0.11, 1.14],
        "shoulder_leftright" : [-0.8, 1.3],
        "shoulder_updown" : [-0.3, 0.9]
    }
}

class BaxterTeleop:
    def __init__(self):
        # self.left_hand_message = None
        # self.left_lowerarm_message = None
        # self.left_upperarm_message = None
        # self.right_hand_message = None
        # self.right_lowerarm_message = None
        # self.right_upperarm_message = None

        self.tracking_messages = {}
        self.initialization_transformations = {}
        self.baxter_limbs = {}

        for side in SIDES:
            self.tracking_messages[side] = {}
            self.initialization_transformations[side] = {}
            self.baxter_limbs[side] = baxter_interface.Limb(side)

            for limb in LIMBS:
                self.tracking_messages[side][limb] = None
                self.initialization_transformations[side][limb] = None

        self.subscribers = []
        for topic_name, side, limb in TOPICS:
            self.subscribers.append(
                rospy.Subscriber(topic_name, PoseStamped, self.tracking_callback, (side, limb), queue_size = 1)
            )

        self.initialized = False

    def tracking_callback(self, message, params):
        if(params[0] == "all"):
            for side in SIDES:
                self.tracking_messages[side][params[1]] = message
        else:
            self.tracking_messages[params[0]][params[1]] = message

    def get_quaternions(self, side, transform = True):
        quats = {}

        for limb in LIMBS:
            if(self.tracking_messages[side][limb] is not None):
                quat = scipy.spatial.transform.Rotation.from_quat([
                    self.tracking_messages[side][limb].pose.orientation.x,
                    self.tracking_messages[side][limb].pose.orientation.y,
                    self.tracking_messages[side][limb].pose.orientation.z,
                    self.tracking_messages[side][limb].pose.orientation.w
                ])

                if(transform):
                    # if(self.initialization_transformations[side][limb] is None):
                    #     print("      Bad transform for: " + limb + " " + side)
                    quat = quat * self.initialization_transformations[side][limb]
            else:
                quat = None

            quats[limb] = quat

        return quats

    # At start up, have person hold arm straight.
    # Then we create a projection transforming each rotation to itself such that they match.
    def initialize(self):
        for side in SIDES:
            quats = self.get_quaternions(side, transform = False)

            for key, value in quats.iteritems():
                if(value is None):
                    print("        Missing value for: " + str(key) + " " + side)
                    self.initialized = False
                    return False

            for limb in LIMBS:
                if(quats[limb] is None):
                    self.initialized = False
                    return False

                if(limb == FRAME_OF_REFERENCE):
                    self.initialization_transformations[side][limb] = scipy.spatial.transform.Rotation.from_rotvec([0, 0, 0])
                else:
                    rot = quats[FRAME_OF_REFERENCE].inv() * quats[limb]
                    self.initialization_transformations[side][limb] = rot

        self.initialized = True
        return True

    def transform_range(self, value, side, joint):
        new_value = (value - TRACKING_LIMITS[side][joint][0]) * ((BAXTER_LIMITS[side][joint][1] - BAXTER_LIMITS[side][joint][0]) / (TRACKING_LIMITS[side][joint][1] - TRACKING_LIMITS[side][joint][0])) + BAXTER_LIMITS[side][joint][0]

        return new_value

    def valid_range(self, value, side, joint):
        return True
        if(value > BAXTER_LIMITS[side][joint][0] and value < BAXTER_LIMITS[side][joint[1]]):
            return True
        else:
            return False

    def publish_joint_command(self):
        for side in SIDES:
            current_angles = self.baxter_limbs[side].joint_angles()

            quats = self.get_quaternions(side, transform = False)

            for key, value in quats.iteritems():
                if(value is None):
                    continue

            # if(side == "left"):
            #     continue

            # ELBOW
            rot = quats["upperarm"].inv() * quats["lowerarm"]
            rot_quat = rot.as_quat()
            theta = np.linalg.norm(rot.as_rotvec())

            # if(side == "right"):
            #     print("                Theta: " + str(theta))

            # theta = 2.0 * np.arctan2(np.linalg.norm([rot_quat[0], rot_quat[1], rot_quat[2]]), rot_quat[3])
            # print("                Rotation: " + str(rot.as_rotvec()))
            # print("                Norm: " + str(np.linalg.norm(rot.as_rotvec())))
            # print("                Theta: " + str(theta))
            # print("                 " + str(current_angles["left_e1"]))

            control_theta = self.transform_range(theta, side, "elbow")
            # print("                Control Theta: " + str(control_theta))

            if(self.valid_range(control_theta, side, "elbow")):
                current_angles[side + "_e1"] = control_theta


            # HAND
            rot = quats["lowerarm"].inv() * quats["hand"]
            rot_quat = rot.as_quat()
            theta = np.linalg.norm(rot.as_rotvec())
            if(side == "left"):
                print("              " + side + "  Theta: " + str(theta))
            control_theta = self.transform_range(theta, side, "wrist")
            # print("                Control Theta: " + str(control_theta))

            if(self.valid_range(control_theta, side, "wrist")):
                current_angles[side + "_w1"] = control_theta


            # SHOULDER LEFT/RIGHT
            rot = quats["chest"].inv() * quats["upperarm"]
            rot_quat = rot.as_quat()
            theta = rot.as_rotvec()[1]
            # Left
            if(side == "left"):
                if(theta > 0):
                    theta = -2.6 - (3.0 - theta)

            control_theta = self.transform_range(theta, side, "shoulder_leftright")

            if(self.valid_range(control_theta, side, "shoulder_leftright")):
                current_angles[side + "_s0"] = control_theta


            #     theta *= -1
            #     theta -= 3.0
            # print("                Rotation: " + str(rot.as_rotvec()))


            # SHOULDER UP/DOWN
            # theta = rot.as_rotvec()[0]
            # theta = np.abs(theta)
            # # print("                Theta: " + str(rot.as_rotvec()))
            # if(side == "left"):
            #     control_theta = self.transform_range(theta, side, "shoulder_updown")
            #     # print("                Rotation: " + str(rot.as_rotvec()))
            #     # print("                Theta: " + str(theta))
            #     if(self.valid_range(control_theta, side, "shoulder_updown")):
            #         current_angles[side + "_s1"] = control_theta




            if(side == "left"):
                current_angles["left_e0"] = -1.3
                current_angles["left_w0"] = 0.16
                current_angles["left_s1"] = 0.37
            if(side == "right"):
                current_angles["right_e0"] = 1.86
                current_angles["right_w0"] = -0.40
                current_angles["right_s1"] = 0.22

            self.baxter_limbs[side].set_joint_positions(current_angles)

            # print("              " + side + "S1   " + str(current_angles[side + "_s1"]))
            # print("              " + side + "W1   " + str(current_angles[side + "_w1"]))
            # print("              " + side + "E0   " + str(current_angles[side + "_e0"]))
            # print("              " + side + "W0   " + str(current_angles[side + "_w0"]))



            # current_angles["right_s0"] = 0.16
            # current_angles["right_w1"] = 0.90
            # current_angles["right_e0"] = 1.48
            # current_angles["right_w0"] = -0.34
            # current_angles["right_s1"] = 0.28




            # print(current_angles)


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("baxter_teleop_node")

    talker = BaxterTeleop()

    rate = rospy.Rate(CONTROL_FREQUENCY)

    while not rospy.is_shutdown():
        if(not talker.initialized):
            if(talker.initialize()):
                print("Initialized!")
            else:
                print("Failed to initialize!")
        else:
            talker.publish_joint_command()

        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)
