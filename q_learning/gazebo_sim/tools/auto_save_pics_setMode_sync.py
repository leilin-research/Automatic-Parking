#!/usr/bin/env python
# -----------------------------------
# automatically save images seen from car's four cameras, synchronized
# Author: Tao Chen
# Date: 2016.10.16
# -----------------------------------
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
import os

import numpy as np
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
import yaml
import random
import time
import rospkg
import threading
import message_filters

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

        self.img_dir = os.path.join(self.current_path, 'images_away_from_terminal')
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
        self.lock = threading.Lock()

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

        self.reset()

        self.car_length = 0.6
        self.car_width = 0.38

        self.get_wall_boundary()
        self.get_fixed_car_boundary()
        self.set_car_move_boundary()


        self.cameraFront_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler/left/image_raw", Image)
        self.cameraRight_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler2/right/image_raw", Image)
        self.cameraBack_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler3/right/image_raw", Image)
        self.cameraLeft_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler4/left/image_raw", Image)
        self.motor_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.cameraFront_sub, self.cameraRight_sub, \
                                                                 self.cameraBack_sub, self.cameraLeft_sub],queue_size=15, slop=0.2)

        self.sync.registerCallback(self.sync_callback)
        # self.link_sub = rospy.Subscriber("/ackermann_vehicle/gazebo/link_states", LinkStates, self.link_callback)
        x, y, z, theta = self.generate_car_pose()
        self.set_car_pose(x, y, z, theta)
        time.sleep(2)

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


    def get_wall_boundary(self):
        self.wall_boundary = np.zeros(4)  # xmin, xmax, ymin, ymax
        try:
            rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
            get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)
            resp1 = get_link_state('wall_test1_0::Wall_0', '')
            self.wall_boundary[2] = resp1.link_state.pose.position.y

            resp2 = get_link_state('wall_test1_0::Wall_1', '')
            self.wall_boundary[1] = resp2.link_state.pose.position.x

            resp3 = get_link_state('wall_test1_0::Wall_2', '')
            self.wall_boundary[3] = resp3.link_state.pose.position.y

            resp4 = get_link_state('wall_test1_0::Wall_3', '')
            self.wall_boundary[0] = resp4.link_state.pose.position.x

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_fixed_car_boundary(self):
        self.cars_boundary = np.zeros((2, 4))  # xmin, xmax, ymin, ymax(Two cars)

        try:
            rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
            get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)

            resp1 = get_link_state('ack_c1::base_link', '')
            self.car1_center = np.zeros(3)
            self.car1_center[0] = resp1.link_state.pose.position.x
            self.car1_center[1] = resp1.link_state.pose.position.y
            self.car1_center[2] = resp1.link_state.pose.position.z
            self.cars_boundary[0][0] = self.car1_center[0] - self.car_length / 2.0
            self.cars_boundary[0][1] = self.car1_center[0] + self.car_length / 2.0
            self.cars_boundary[0][2] = self.car1_center[1] - self.car_width / 2.0
            self.cars_boundary[0][3] = self.car1_center[1] + self.car_width / 2.0

            resp2 = get_link_state('ack_c2::base_link', '')
            self.car2_center = np.zeros(3)
            self.car2_center[0] = resp2.link_state.pose.position.x
            self.car2_center[1] = resp2.link_state.pose.position.y
            self.car2_center[2] = resp2.link_state.pose.position.z
            self.cars_boundary[1][0] = self.car2_center[0] - self.car_length / 2.0
            self.cars_boundary[1][1] = self.car2_center[0] + self.car_length / 2.0
            self.cars_boundary[1][2] = self.car2_center[1] - self.car_width / 2.0
            self.cars_boundary[1][3] = self.car2_center[1] + self.car_width / 2.0

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def set_car_move_boundary(self):
        near_wall_tolerance = 0.8 * self.car_length
        near_fixed_car_tolerance = 0.7 * self.car_length

        self.car_move_boundary = np.zeros((2, 4))  # xmin, xmax, ymin, ymax (Outer Suqare, Innder boundary square)
        self.car_move_boundary[0][0] = self.wall_boundary[0] + near_wall_tolerance
        self.car_move_boundary[0][1] = self.wall_boundary[1] - near_wall_tolerance
        self.car_move_boundary[0][2] = self.wall_boundary[2] + near_wall_tolerance
        self.car_move_boundary[0][3] = self.wall_boundary[3] - near_wall_tolerance

        self.car_move_boundary[1][0] = self.cars_boundary[1][0] - near_fixed_car_tolerance
        self.car_move_boundary[1][1] = self.cars_boundary[0][1] + near_fixed_car_tolerance
        self.car_move_boundary[1][2] = self.cars_boundary[0][2] - near_fixed_car_tolerance
        self.car_move_boundary[1][3] = self.cars_boundary[0][3] + near_fixed_car_tolerance
        # print "self.car_move:",self.car_move_boundary

    def collision_check(self, x, y, z, theta):
        if x < self.car_move_boundary[0][0] or x > self.car_move_boundary[0][1] \
                or y < self.car_move_boundary[0][2] or y > self.car_move_boundary[0][3]:
            return True

        elif x > self.car_move_boundary[1][0] and x < self.car_move_boundary[1][1] \
                and y > self.car_move_boundary[1][2] and y < self.car_move_boundary[1][3]:
            return True

        # try:
        #
        #     # self.set_car_pose(x, y, z, theta)
        #     #
        #     # car_movement_tolerance = 0.05
        #     # rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
        #     # get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)
        #     # resp1 = get_link_state('ack_c1::base_link', '')
        #     # car1_center = np.zeros(3)
        #     # car1_center[0] = resp1.link_state.pose.position.x
        #     # car1_center[1] = resp1.link_state.pose.position.y
        #     # car1_center[2] = resp1.link_state.pose.position.z
        #     #
        #     # resp2 = get_link_state('ack_c2::base_link', '')
        #     # car2_center = np.zeros(3)
        #     # car2_center[0] = resp2.link_state.pose.position.x
        #     # car2_center[1] = resp2.link_state.pose.position.y
        #     # car2_center[2] = resp2.link_state.pose.position.z
        #     #
        #     # car1_movement = np.linalg.norm(self.car1_center - car1_center)
        #     # car2_movement = np.linalg.norm(self.car2_center - car2_center)
        #     #
        #     # if car1_movement > car_movement_tolerance or car2_movement > car_movement_tolerance:
        #     #     #self.reset()
        #     #     return True
        #
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s" % e

        return False

    def generate_car_pose(self):
        while True:
            x = random.uniform(self.car_move_boundary[0][0], self.car_move_boundary[0][1])
            y = random.uniform(self.car_move_boundary[0][2], self.car_move_boundary[0][3])
            z = 0.105
            theta = random.uniform(0, 2 * np.pi)
            if not self.collision_check(x, y, z, theta):
                break

        return [x, y, z, theta]

    def reset(self):
        # rospy.wait_for_service('/ackermann_vehicle/gazebo/set_model_state')
        # set_model_pose = rospy.ServiceProxy('/ackermann_vehicle/gazebo/set_model_state', SetModelState)
        # state = ModelState()
        # pose = Pose()
        #
        # # Fixed Car 1
        # pose.position.x = 0.7
        # pose.position.y = -2
        # pose.position.z = 0.1
        # pose.orientation.x = 0
        # pose.orientation.y = 0
        # pose.orientation.z = 0
        # pose.orientation.w = 1.0
        #
        # state.model_name = 'ack_c1'
        # state.pose = pose
        # resp1 = set_model_pose(state)
        #
        # # Fixed Car 2
        # pose.position.x = -0.7
        # pose.position.y = -2
        # pose.position.z = 0.1
        # pose.orientation.x = 0
        # pose.orientation.y = 0
        # pose.orientation.z = 0
        # pose.orientation.w = 1.0
        #
        # state.model_name = 'ack_c2'
        # state.pose = pose
        # resp2 = set_model_pose(state)
        #
        # # Agent Car
        # pose.position.x = 0.0
        # pose.position.y = 2.0
        # pose.position.z = 0.1
        # pose.orientation.x = 0
        # pose.orientation.y = 0
        # pose.orientation.z = -0.707
        # pose.orientation.w = 0.707
        #
        # state.model_name = 'ackermann_vehicle'
        # state.pose = pose
        # resp3 = set_model_pose(state)

        rospy.wait_for_service('/ackermann_vehicle/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/ackermann_vehicle/gazebo/reset_world', Empty)
        resp = reset_world()

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
            self.car_speed_pub(speed=0.0, steering_angle=0.0)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def car_speed_pub(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.motor_pub.publish(msg)

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
        # sys.stdout.write("\r")
        name_list = data.name
        car_link_index = name_list.index(self.car_link_name)
        # sys.stdout.write("\r")
        # print "car_link_index:",car_link_index
        # sys.stdout.write("\r")
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

        if self.cameraFront_img == None or self.cameraRight_img == None or self.cameraBack_img == None or self.cameraLeft_img == None:
            return

        x, y, z, theta = self.generate_car_pose()
        self.set_car_pose(x, y, z, theta)
        # # save pictures in A, B, C, D, E
        time.sleep(1)
        self.lock.acquire()

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
            cameraFront_img_name = os.path.join(self.front_cam_dir,'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        print 'saving image', self.img_count
        cameraRight_img_name = os.path.join(self.right_cam_dir, 'R' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraBack_img_name = os.path.join(self.back_cam_dir,'B' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraLeft_img_name = os.path.join(self.left_cam_dir, 'L' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraAll_img_name = os.path.join(self.merged_dir,'M' + str('{:06d}'.format(self.img_count)) + '.jpg')
        car_pose_name = os.path.join(self.car_pos_dir,'car_pos' + str('{:06d}'.format(self.img_count)) + '.yaml')


        cv2.imwrite(cameraFront_img_name, self.cameraFront_img)
        cv2.imwrite(cameraRight_img_name, self.cameraRight_img)
        cv2.imwrite(cameraBack_img_name, self.cameraBack_img)
        cv2.imwrite(cameraLeft_img_name, self.cameraLeft_img)
        cv2.imwrite(cameraAll_img_name, self.cameraAll)

        car_pose_stream = open(car_pose_name, "w")
        car_data = {'car_pos': self.car_pos.tolist(), 'car_ori': self.car_ori.tolist()}
        yaml.dump(car_data, car_pose_stream)
        self.lock.release()


        self.img_count += 1

        if self.img_count > 100000:
            sys.exit()


if __name__ == "__main__":
    rospy.init_node('save_pics_sync')
    cv2.startWindowThread()
    ss = saver()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ss.save_pics()

