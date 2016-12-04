#!/usr/bin/env python
# -----------------------------------
# save synchronized images received from cameras mounted on the ackermann car, run this script in terminal
# Author: Tao Chen
# Date: 2016.10.16
# -----------------------------------

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import threading
import numpy as np
from gazebo_msgs.srv import GetLinkState
import yaml
import message_filters


from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from ackermann_msgs.msg import AckermannDriveStamped
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
import time
import rospkg

class saver:
    def __init__(self):
        self.br = CvBridge()
        self.cameraFront_img = None
        self.cameraRight_img = None
        self.cameraBack_img = None
        self.cameraLeft_img = None

        self.car_pos = np.zeros(3)
        self.car_ori = np.zeros(4)

        self.current_path = os.getcwd()

        self.img_dir = os.path.join(self.current_path, 'images_manual')
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

        self.img_count = 0
        self.car_pos = np.zeros(3)
        self.car_ori = np.zeros(4)

        self.im_show = False
        if self.im_show:
            self.cameraFront_window = 'Front'
            self.cameraRight_window = 'rightSide'
            self.cameraBack_window = 'Back'
            self.cameraLeft_window = 'leftSide'
            cv2.namedWindow(self.cameraFront_window)
            cv2.namedWindow(self.cameraRight_window)
            cv2.namedWindow(self.cameraBack_window)
            cv2.namedWindow(self.cameraLeft_window)



        self.cameraFront_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler/left/image_raw", Image)
        self.cameraRight_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler2/right/image_raw", Image)
        self.cameraBack_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler3/right/image_raw", Image)
        self.cameraLeft_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler4/left/image_raw", Image)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.cameraFront_sub, self.cameraRight_sub, self.cameraBack_sub, self.cameraLeft_sub], queue_size=10, slop=0.2)

        self.sync.registerCallback(self.sync_callback)
        self.lock = threading.Lock()



    def sync_callback(self, frontImage, rightImage, backImage, leftImage):
        print '....'
        self.lock.acquire()
        self.cameraFront_callback(frontImage)
        self.cameraRight_callback(rightImage)
        self.cameraBack_callback(backImage)
        self.cameraLeft_callback(leftImage)

        # You can either use service or msg to get the car's pose, if you want to use
        # msg, then uncomment the definition of self.link_sub, and comment the following lines
        # otherwise, the following lines will use service to get the car's pose
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
        self.lock.release()

    def cameraFront_callback(self, data):
        try:
            self.cameraFront_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraFront_window, self.cameraFront_img)

    def cameraRight_callback(self, data):
        try:
            self.cameraRight_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraRight_window, self.cameraRight_img)

    def cameraBack_callback(self, data):
        try:
            self.cameraBack_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraBack_window, self.cameraBack_img)

    def cameraLeft_callback(self, data):
        try:
            self.cameraLeft_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraLeft_window, self.cameraLeft_img)

    def link_callback(self, data):
        name_list = data.name
        car_link_index = name_list.index(self.car_link_name)
        self.lock.acquire()
        self.car_pos[0] = data.pose[car_link_index].position.x
        self.car_pos[1] = data.pose[car_link_index].position.y
        self.car_pos[2] = data.pose[car_link_index].position.z
        self.car_ori[0] = data.pose[car_link_index].orientation.x
        self.car_ori[1] = data.pose[car_link_index].orientation.y
        self.car_ori[2] = data.pose[car_link_index].orientation.z
        self.car_ori[3] = data.pose[car_link_index].orientation.w
        self.lock.release()

    def save_pics(self):
        if self.cameraFront_img == None or self.cameraRight_img == None or self.cameraBack_img == None:
            return

        white_band = np.ones((self.cameraFront_img.shape[0], 20, 3)) * 255
        self.cameraAll = np.concatenate((self.cameraFront_img, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraRight_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraBack_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraLeft_img), axis=1)

        cameraFront_img_name = os.path.join(self.front_cam_dir, 'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        while os.path.exists(cameraFront_img_name):
            self.img_count += 1
            cameraFront_img_name = os.path.join(self.front_cam_dir, 'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        print 'saving image', self.img_count
        cameraRight_img_name = os.path.join(self.right_cam_dir, 'R' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraBack_img_name = os.path.join(self.back_cam_dir, 'B' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraLeft_img_name = os.path.join(self.left_cam_dir, 'L' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraAll_img_name = os.path.join(self.merged_dir, 'M' + str('{:06d}'.format(self.img_count)) + '.jpg')
        car_pose_name = os.path.join(self.car_pos_dir, 'car_pos' + str('{:06d}'.format(self.img_count)) + '.yaml')

        cv2.imwrite(cameraFront_img_name, self.cameraFront_img)
        cv2.imwrite(cameraRight_img_name, self.cameraRight_img)
        cv2.imwrite(cameraBack_img_name, self.cameraBack_img)
        cv2.imwrite(cameraLeft_img_name, self.cameraLeft_img)
        cv2.imwrite(cameraAll_img_name, self.cameraAll)

        car_pose_stream = open(car_pose_name, "w")
        car_data = {'car_pos': self.car_pos.tolist(), 'car_ori': self.car_ori.tolist()}
        yaml.dump(car_data, car_pose_stream)

        self.img_count += 1

    def get_key(self):
        key = raw_input("save (s) or quit (q):")

        return key

    def do_save(self):

        key = self.get_key()
        if key == '\x03' or key == '\x71':  # ctrl + c or 'q'
            sys.exit()
        elif key == 's':
            self.save_pics()



if __name__ == "__main__":
    rospy.init_node('save_pics')
    print 'press \'s\' to save the image...'
    print 'press \'q\' to exit...'
    cv2.startWindowThread()
    ss = saver()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ss.do_save()


