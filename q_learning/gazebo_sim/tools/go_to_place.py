#!/usr/bin/env python
# -----------------------------------
# send car to the position user entered, save the images
# Author: Tao Chen
# Date: 2016.10.16
# -----------------------------------
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
import os

import numbers
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import yaml
import time
import threading


class Goer:
    def __init__(self):
        self.br = CvBridge()
        self.cameraOne_img = None
        self.cameraTwo_img = None
        self.cameraThree_img = None

        self.car_pos = np.zeros(3)
        self.car_ori = np.zeros(4)

        self.current_path = os.getcwd()

        self.img_dir = os.path.join(self.current_path, 'Go_images')
        self.front_cam_dir = os.path.join(self.img_dir, 'front')
        self.right_cam_dir = os.path.join(self.img_dir, 'right')
        self.back_cam_dir = os.path.join(self.img_dir, 'back')
        self.left_cam_dir = os.path.join(self.img_dir, 'left')
        self.merged_dir = os.path.join(self.img_dir, 'merged')
        self.car_pos_dir = os.path.join(self.img_dir, 'car_pos')

        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)

        if not os.path.exists(self.front_cam_dir):
            os.makedirs(self.front_cam_dir)

        if not os.path.exists(self.right_cam_dir):
            os.makedirs(self.right_cam_dir)

        if not os.path.exists(self.back_cam_dir):
            os.makedirs(self.back_cam_dir)

        if not os.path.exists(self.left_cam_dir):
            os.makedirs(self.left_cam_dir)

        if not os.path.exists(self.merged_dir):
            os.makedirs(self.merged_dir)

        if not os.path.exists(self.car_pos_dir):
            os.makedirs(self.car_pos_dir)

        self.x_min = -2.93
        self.x_max = 3.87
        self.y_min = -3.4
        self.y_max = 3.4
        self.x_prompt = 'Please enter the desired x coordinate (should be between ' + str(self.x_min) + ' and ' + str(
            self.x_max) + '):'
        self.y_prompt = 'Please enter the desired y coordinate (should be between ' + str(self.y_min) + ' and ' + str(
            self.y_max) + '):'
        self.angle_prompt = 'Please enter the desired angle (rad):'

        self.img_count = 0
        self.lock = threading.Lock()

        self.im_show = True
        if self.im_show:
            self.cameraOne_window = 'Front'
            self.cameraTwo_window = 'rightSide'
            self.cameraThree_window = 'Back'
            self.cameraFour_window = 'leftSide'
            self.car_link_name = 'ackermann_vehicle::base_link'
            cv2.namedWindow(self.cameraOne_window)
            cv2.namedWindow(self.cameraTwo_window)
            cv2.namedWindow(self.cameraThree_window)
            cv2.namedWindow(self.cameraFour_window)

        self.car_length = 0.6
        self.car_width = 0.38

        self.cameraOne_img = None
        self.cameraTwo_img = None
        self.cameraThree_img = None
        self.cameraFour_img = None

        self.cameraOne_sub = rospy.Subscriber("/ackermann_vehicle/multi/camera/basler/left/image_raw", Image,
                                              self.cameraOne_callback)
        self.cameraTwo_sub = rospy.Subscriber("/ackermann_vehicle/multi/camera/basler2/right/image_raw", Image,
                                              self.cameraTwo_callback)
        self.cameraThree_sub = rospy.Subscriber("/ackermann_vehicle/multi/camera/basler3/right/image_raw", Image,
                                                self.cameraThree_callback)
        self.cameraFour_sub = rospy.Subscriber("/ackermann_vehicle/multi/camera/basler4/left/image_raw", Image,
                                               self.cameraFour_callback)



    def set_car_pose(self, x, y, z, theta):

        try:
            rospy.wait_for_service('/ackermann_vehicle/gazebo/set_model_state')
            set_model_pose = rospy.ServiceProxy('/ackermann_vehicle/gazebo/set_model_state', SetModelState)
            state = ModelState()
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = np.sin(theta / 2)
            pose.orientation.w = np.cos(theta / 2)

            state.model_name = 'ackermann_vehicle'
            state.pose = pose
            resp = set_model_pose(state)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def cameraOne_callback(self, data):
        try:
            self.cameraOne_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraOne_window, self.cameraOne_img)

    def cameraTwo_callback(self, data):
        try:
            self.cameraTwo_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraTwo_window, self.cameraTwo_img)

    def cameraThree_callback(self, data):
        try:
            self.cameraThree_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraThree_window, self.cameraThree_img)

    def cameraFour_callback(self, data):
        try:
            self.cameraFour_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraFour_window, self.cameraFour_img)

    def go_to_place(self):
        x_inputed = False
        y_inputed = False
        theta_inputed = False
        while not x_inputed:
            x = raw_input(self.x_prompt)
            if x == '\x03' or x == '\x71':  # ctrl + c or 'q'
                sys.exit()
            try:
                x = eval(x)
                if not isinstance(x, numbers.Real):
                    print "x is not a number"
                    print "Please enter it again..."
                elif x < self.x_min or x > self.x_max:
                    print "x coordinate out of secure range!!"
                    print "Please enter it again..."
                else:
                    print "you entered x = ", x
                    x_inputed = True
            except NameError, e:
                print "You did not enter a number, please try again"

        while not y_inputed:
            y = raw_input(self.y_prompt)
            if y == '\x03' or y == '\x71':  # ctrl + c or 'q'
                sys.exit()
            try:
                y = eval(y)
                if not isinstance(y, numbers.Real):
                    print "y is not a number"
                    print "Please enter it again..."
                elif y < self.y_min or y > self.y_max:
                    print "y coordinate out of secure range!!"
                    print "Please enter it again..."
                else:
                    print "you entered y = ", y
                    y_inputed = True
            except NameError, e:
                print "You did not enter a number, please try again"

        z = 0.105
        while not theta_inputed:
            theta = raw_input(self.angle_prompt)
            if theta == '\x03' or theta == '\x71':  # ctrl + c or 'q'
                sys.exit()
            try:
                theta = eval(theta)
                if not isinstance(theta, numbers.Real):
                    print "theta is not a number"
                    print "Please enter it again..."
                else:
                    print "you entered theta = ", theta
                    theta_inputed = True
                    theta = theta % (2 * np.pi)
                    print "If converted into [0, 2pi) range, it's equal to: ",theta
            except NameError, e:
                print "You did not enter a number, please try again"

        self.set_car_pose(x, y, z, theta)
        self.save_pics()

    def save_pics(self):

        if self.cameraOne_img == None or self.cameraTwo_img == None or self.cameraThree_img == None or self.cameraFour_img == None:
            return

        # # save pictures in A, B, C, D, E
        time.sleep(2)
        self.lock.acquire()
        rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
        try:
            get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)
            resp = get_link_state('ackermann_vehicle::base_link', '')
            self.car_pos[0] = resp.link_state.pose.position.x
            self.car_pos[1] = resp.link_state.pose.position.y
            self.car_pos[2] = resp.link_state.pose.position.z
            self.car_ori[0] = resp.link_state.pose.orientation.x
            self.car_ori[1] = resp.link_state.pose.orientation.y
            self.car_ori[2] = resp.link_state.pose.orientation.z
            self.car_ori[3] = resp.link_state.pose.orientation.w
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        white_band = np.ones((self.cameraOne_img.shape[0], 20, 3)) * 255
        self.cameraAll = np.concatenate((self.cameraOne_img, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraTwo_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraThree_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraFour_img), axis=1)

        cameraOne_img_name = os.path.join(self.front_cam_dir, 'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        while os.path.exists(cameraOne_img_name):
            self.img_count += 1
            cameraOne_img_name = os.path.join(self.front_cam_dir, 'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        print 'saving image', self.img_count
        cameraTwo_img_name = os.path.join(self.right_cam_dir, 'R' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraThree_img_name = os.path.join(self.back_cam_dir, 'B' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraFour_img_name = os.path.join(self.left_cam_dir, 'L' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraAll_img_name = os.path.join(self.merged_dir, 'M' + str('{:06d}'.format(self.img_count)) + '.jpg')
        car_pose_name = os.path.join(self.car_pos_dir, 'car_pos' + str('{:06d}'.format(self.img_count)) + '.yaml')

        # # save pictures in continuous names without A, B, C, D, E
        # cameraOne_img_name = os.path.join(self.img_dir, str('{:05d}'.format(self.img_count)) + '.jpg')
        # while os.path.exists(cameraOne_img_name):
        #     self.img_count += 1
        #     cameraOne_img_name = os.path.join(self.img_dir, str('{:05d}'.format(self.img_count)) + '.jpg')
        # self.img_count += 1
        # cameraTwo_img_name = os.path.join(self.img_dir, str('{:05d}'.format(self.img_count)) + '.jpg')
        # self.img_count += 1
        # cameraThree_img_name = os.path.join(self.img_dir, str('{:05d}'.format(self.img_count)) + '.jpg')
        # self.img_count += 1
        # cameraFour_img_name = os.path.join(self.img_dir, str('{:05d}'.format(self.img_count)) + '.jpg')

        cv2.imwrite(cameraOne_img_name, self.cameraOne_img)
        cv2.imwrite(cameraTwo_img_name, self.cameraTwo_img)
        cv2.imwrite(cameraThree_img_name, self.cameraThree_img)
        cv2.imwrite(cameraFour_img_name, self.cameraFour_img)
        cv2.imwrite(cameraAll_img_name, self.cameraAll)

        # You can either use service or msg to get the car's pose, if you want to use
        # msg, then uncomment the definition of self.link_sub, and comment the following lines
        # otherwise, the following lines will use service to get the car's pose


        car_pose_stream = open(car_pose_name, "w")
        car_data = {'car_pos': self.car_pos.tolist(), 'car_ori': self.car_ori.tolist()}
        yaml.dump(car_data, car_pose_stream)
        self.lock.release()

        self.img_count += 1

        if self.img_count > 10000:
            sys.exit()


if __name__ == "__main__":
    rospy.init_node('go_to_place')
    cv2.startWindowThread()
    gg = Goer()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        gg.go_to_place()

