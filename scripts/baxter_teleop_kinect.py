#!/usr/bin/env python

import numpy as np
import rospy
import baxter_interface
import sys

from kinect_listen_driver.msg import KinectData

CONTROL_FREQUENCY = 20.0
TIMEOUT = 1.0 / CONTROL_FREQUENCY

KINECT_SHOULDER_MIN = 0.0
KINECT_SHOULDER_MAX = 1.5

class BaxterTeleop:
    def __init__(self):
        self.kinect_message = None

        self.baxter_left_arm = baxter_interface.Limb("left")
        self.baxter_right_arm = baxter_interface.Limb("right")

        self.kinect_sub = rospy.Subscriber("/kinect_skeleton_tracking", KinectData, self.kinect_callback, queue_size = 1)

    def publish_joint_command(self):
        if(self.kinect_message is not None):

            # ------ RIGHT SIDE -------
            current_angles = self.baxter_right_arm.joint_angles()
            print("            Right: " + str(current_angles["right_s1"]))
            # Compute angle of elbow
            # Compute shoulder->elbow and elbow->hand vectors.
            shoulder_elbow = np.array(self.kinect_message.elbow_right) - np.array(self.kinect_message.shoulder_right)
            elbow_hand = np.array(self.kinect_message.elbow_right) - np.array(self.kinect_message.hand_right)
            # These two angles lie in a plane since they share a vertx, so compute the angle between them using the cosine method.
            dot_product = np.dot(shoulder_elbow, elbow_hand)
            right_elbow_theta = np.arccos(dot_product / (np.linalg.norm(shoulder_elbow) * np.linalg.norm(elbow_hand)))
            right_elbow_theta = np.pi - right_elbow_theta
            if(right_elbow_theta > 0.0 and right_elbow_theta < 2.4):
                current_angles["right_e1"] = right_elbow_theta
            print("                  Right elbow: " + str(right_elbow_theta))

            # # Calculate the rotation of the arm.
            # normal_plane_vector = np.cross(shoulder_elbow, elbow_hand)
            # # Calculate angle between the normal vector of shoulder->elbow->hand plane and the vector pointing straight "up".
            # up_vector = np.array([-1, 0, 0])
            # dot_product = np.dot(normal_plane_vector, up_vector)
            # right_elbow_rotation = np.arccos(dot_product / (np.linalg.norm(normal_plane_vector) * np.linalg.norm(up_vector)))
            # # right_elbow_rotation = np.pi - right_elbow_rotation + np.pi / 2.0
            # if(self.kinect_message.elbow_right[0] < self.kinect_message.hand_right[0]):
            #     right_elbow_rotation *= -1.0
            # right_elbow_rotation *= -1.0
            # right_elbow_rotation = right_elbow_rotation + np.pi - 0.5
            # if(right_elbow_rotation > -3.0 and right_elbow_rotation < 3.0):
            #     current_angles["right_e0"] = right_elbow_rotation
            # print("                  Right elbow rotation: " + str(right_elbow_rotation))

            # Compute angle of shoulder
            # Compute shoulder->shoulder and shoulder->elbow vectors.
            shoulder_shoulder = np.array(self.kinect_message.shoulder_right) - np.array(self.kinect_message.shoulder_left)
            shoulder_elbow = np.array(self.kinect_message.shoulder_right) - np.array(self.kinect_message.elbow_right)
            # These two angles lie in a plane since they share a vertx, so compute the angle between them using the cosine method.
            dot_product = np.dot(shoulder_shoulder, shoulder_elbow)
            right_shoulder_theta = np.arccos(dot_product / (np.linalg.norm(shoulder_shoulder) * np.linalg.norm(shoulder_elbow)))
            right_shoulder_theta = np.pi - right_shoulder_theta
            # If the elbow is higher than the shoulder, make the angle negative.
            if(self.kinect_message.elbow_right[1] > self.kinect_message.shoulder_right[1]):
                right_shoulder_theta *= -1.0
                right_shoulder_theta += 1.0
                print("           Flipped " + str(right_shoulder_theta))


            print("                  Right shoulder: " + str(right_shoulder_theta))
            if(right_shoulder_theta > -2.0 and right_shoulder_theta < 1.0):
                current_angles["right_s1"] = right_shoulder_theta

            # # Calculate the rotation of the shoulder.
            # normal_plane_vector = np.cross(shoulder_shoulder, shoulder_elbow)
            # # Calculate angle between the normal vector of shoulder->elbow->hand plane and the vector pointing straight "up".
            # up_vector = np.array([0, 0, 1])
            # dot_product = np.dot(normal_plane_vector, up_vector)
            # right_shoulder_rotation = np.arccos(dot_product / (np.linalg.norm(normal_plane_vector) * np.linalg.norm(up_vector)))
            # # right_elbow_rotation = np.pi - right_elbow_rotation + np.pi / 2.0
            # if(self.kinect_message.elbow_right[2] > self.kinect_message.shoulder_right[2]):
            #     right_shoulder_rotation *= -1.0
            # # right_shoulder_rotation *= -1.0
            # right_shoulder_rotation = right_shoulder_rotation - np.pi / 4.0
            # if(right_shoulder_rotation > -1.7 and right_shoulder_rotation < 1.7):
            #     current_angles["right_s0"] = right_shoulder_rotation
            # print("                  Right shoulder rotation: " + str(right_shoulder_rotation))
            current_angles["right_s0"] = 0.16
            current_angles["right_w1"] = 0.90
            current_angles["right_e0"] = 1.48
            current_angles["right_w0"] = -0.34
            current_angles["right_s1"] = 0.28


            # self.baxter_right_arm.move_to_joint_positions(current_angles, timeout = TIMEOUT)
            self.baxter_right_arm.set_joint_positions(current_angles)

            # print(current_angles)

            # ------ LEFT SIDE -------
            current_angles = self.baxter_left_arm.joint_angles()
            print("             Left: " + str(current_angles["left_s1"]))
            # Compute angle of elbow
            # Compute shoulder->elbow and elbow->hand vectors.
            shoulder_elbow = np.array(self.kinect_message.elbow_left) - np.array(self.kinect_message.shoulder_left)
            elbow_hand = np.array(self.kinect_message.elbow_left) - np.array(self.kinect_message.hand_left)
            # These two angles lie in a plane since they share a vertx, so compute the angle between them using the cosine method.
            dot_product = np.dot(shoulder_elbow, elbow_hand)
            left_elbow_theta = np.arccos(dot_product / (np.linalg.norm(shoulder_elbow) * np.linalg.norm(elbow_hand)))
            left_elbow_theta = np.pi - left_elbow_theta
            if(left_elbow_theta > 0.0 and left_elbow_theta < 2.4):
                current_angles["left_e1"] = left_elbow_theta
            print("                  Left elbow: " + str(left_elbow_theta))

            # # Calculate the rotation of the arm.
            # normal_plane_vector = np.cross(shoulder_elbow, elbow_hand)
            # # Calculate angle between the normal vector of shoulder->elbow->hand plane and the vector pointing straight "up".
            # up_vector = np.array([-1, 0, 0])
            # dot_product = np.dot(normal_plane_vector, up_vector)
            # left_elbow_rotation = np.arccos(dot_product / (np.linalg.norm(normal_plane_vector) * np.linalg.norm(up_vector)))
            # # left_elbow_rotation = np.pi - left_elbow_rotation + np.pi / 2.0
            # # if(self.kinect_message.elbow_left[0] > self.kinect_message.hand_left[0]):
            #     # left_elbow_rotation *= -1.0
            # # left_elbow_rotation *= -1.0
            # left_elbow_rotation = left_elbow_rotation - np.pi + 0.5
            # if(left_elbow_rotation > -3.0 and left_elbow_rotation < 3.0):
            #     current_angles["left_e0"] = left_elbow_rotation
            # print("                  Left elbow rotation: " + str(left_elbow_rotation))

            # Compute angle of shoulder
            # Compute shoulder->shoulder and shoulder->elbow vectors.
            shoulder_shoulder = np.array(self.kinect_message.shoulder_left) - np.array(self.kinect_message.shoulder_right)
            shoulder_elbow = np.array(self.kinect_message.shoulder_left) - np.array(self.kinect_message.elbow_left)
            # These two angles lie in a plane since they share a vertx, so compute the angle between them using the cosine method.
            dot_product = np.dot(shoulder_shoulder, shoulder_elbow)
            left_shoulder_theta = np.arccos(dot_product / (np.linalg.norm(shoulder_shoulder) * np.linalg.norm(shoulder_elbow)))
            left_shoulder_theta = np.pi - left_shoulder_theta
            # If the elbow is higher than the shoulder, make the angle negative.
            if(self.kinect_message.elbow_left[1] > self.kinect_message.shoulder_left[1]):
                left_shoulder_theta *= -1.0
                left_shoulder_theta += 0.3

            print("                  Left shoulder: " + str(left_shoulder_theta))
            if(left_shoulder_theta > -2.0 and left_shoulder_theta < 1.0):
                current_angles["left_s1"] = left_shoulder_theta

            # # Calculate the rotation of the shoulder.
            # normal_plane_vector = np.cross(shoulder_shoulder, shoulder_elbow)
            # # Calculate angle between the normal vector of shoulder->elbow->hand plane and the vector pointing straight "up".
            # up_vector = np.array([0, 0, -1])
            # dot_product = np.dot(normal_plane_vector, up_vector)
            # left_shoulder_rotation = np.arccos(dot_product / (np.linalg.norm(normal_plane_vector) * np.linalg.norm(up_vector)))
            # # left_elbow_rotation = np.pi - left_elbow_rotation + np.pi / 2.0
            # # if(self.kinect_message.elbow_left[2] < self.kinect_message.shoulder_left[2]):
            #     # left_shoulder_rotation *= -1.0
            # # left_shoulder_rotation *= -1.0
            # # left_shoulder_rotation = left_shoulder_rotation - np.pi / 4.0
            # if(left_shoulder_rotation > -1.7 and left_shoulder_rotation < 1.7):
            #     current_angles["left_s0"] = left_shoulder_rotation
            # print("                  Left shoulder rotation: " + str(left_shoulder_rotation))
            #
            # triple = np.dot(shoulder_shoulder, np.cross(shoulder_elbow, normal_plane_vector))
            # print("                  Triple: " + str(triple))
            current_angles["left_s0"] = -0.23
            current_angles["left_w1"] = 0.90
            current_angles["left_e0"] = -1.48
            current_angles["left_w0"] = 0.3
            current_angles["left_s1"] = -0.04

            # self.baxter_left_arm.move_to_joint_positions(current_angles, timeout = TIMEOUT)
            self.baxter_left_arm.set_joint_positions(current_angles)



    def kinect_callback(self, message):
        self.kinect_message = message


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("baxter_teleop_node")

    talker = BaxterTeleop()

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
