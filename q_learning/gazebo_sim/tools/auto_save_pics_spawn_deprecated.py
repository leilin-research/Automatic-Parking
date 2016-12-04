#!/usr/bin/env python
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


class saver:
    def __init__(self):
        # /ackermann_vehicle/multi/camera/basler/left/image_raw
        # /ackermann_vehicle/multi/camera/basler2/right/image_raw
        # /ackermann_vehicle/multi/camera/basler3/right/image_raw
        # /ackermann_vehicle/multi/camera/basler4/left/image_raw

        self.br = CvBridge()
        self.cameraOne_img = None
        self.cameraTwo_img = None
        self.cameraThree_img = None

        self.car_pos = np.zeros(3)
        self.car_ori = np.zeros(4)

        self.current_path = os.getcwd()

        self.img_dir = os.path.join(self.current_path, 'auto_images')
        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)
        self.img_count = 0

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

        self.reset()

        self.car_length = 0.6
        self.car_width = 0.38

        self.get_wall_boundary()
        self.get_fixed_car_boundary()
        self.set_car_move_boundary()

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
        self.motor_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        # self.link_sub = rospy.Subscriber("/ackermann_vehicle/gazebo/link_states", LinkStates, self.link_callback)
        x, y, z, theta = self.generate_car_pose()
        self.set_car_pose(x, y, z, theta)
        time.sleep(2)

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
        near_wall_tolerance = 2 * self.car_length
        near_fixed_car_tolerance = 2 * self.car_length

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
        #     #     self.reset()
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
            # rospy.wait_for_service('/ackermann_vehicle/gazebo/set_model_state')
            # set_model_pose = rospy.ServiceProxy('/ackermann_vehicle/gazebo/set_model_state', SetModelState)
            # state = ModelState()
            # pose = Pose()
            # pose.position.x = x
            # pose.position.y = y
            # pose.position.z = z
            # pose.orientation.x = 0
            # pose.orientation.y = 0
            # pose.orientation.z = np.sin(theta / 2)
            # pose.orientation.w = np.cos(theta / 2)
            #
            # state.model_name = 'ackermann_vehicle'
            # state.pose = pose
            # resp = set_model_pose(state)
            rospy.wait_for_service('/ackermann_vehicle/gazebo/delete_model')
            delete_car = rospy.ServiceProxy('/ackermann_vehicle/gazebo/delete_model', DeleteModel)
            resp = delete_car('ackermann_vehicle')
            rospy.wait_for_service('/ackermann_vehicle/gazebo/spawn_urdf_model')
            spawn_car = rospy.ServiceProxy('/ackermann_vehicle/gazebo/spawn_urdf_model', SpawnModel)

            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('ackermann_vehicle_description')
            car_urdf_path = os.path.join(pkg_path, 'urdf','car_move.urdf')
            car_urdf = open(car_urdf_path,'r')
            car_urdf_content = car_urdf.read()

            # namespace = 'ackermann_vehicle/gazebo'
            namespace = ''

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = np.sin(theta / 2)
            pose.orientation.w = np.cos(theta / 2)
            resp = spawn_car('ackermann_vehicle',car_urdf_content, namespace, pose, '')

            self.car_speed_pub(speed=0.0, steering_angle=0.0)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def car_speed_pub(self, speed, steering_angle):
        print 'speed.................'
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self.motor_pub.publish(msg)

    def cameraOne_callback(self, data):
        print "one"
        try:
            self.cameraOne_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraOne_window, self.cameraOne_img)

    def cameraTwo_callback(self, data):
        print "two"
        try:
            self.cameraTwo_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraTwo_window, self.cameraTwo_img)

    def cameraThree_callback(self, data):
        print "three"
        try:
            self.cameraThree_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraThree_window, self.cameraThree_img)

    def cameraFour_callback(self, data):
        print "four"
        try:
            self.cameraFour_img = self.br.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.im_show:
            cv2.imshow(self.cameraFour_window, self.cameraFour_img)

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

        if self.cameraOne_img == None or self.cameraTwo_img == None or self.cameraThree_img == None or self.cameraFour_img == None:
            return

        x, y, z, theta = self.generate_car_pose()
        self.set_car_pose(x, y, z, theta)
        # # save pictures in A, B, C, D, E
        time.sleep(2)
        white_band = np.ones((self.cameraOne_img.shape[0], 20, 3)) * 255
        self.cameraAll = np.concatenate((self.cameraOne_img, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraTwo_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraThree_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraFour_img), axis=1)

        cameraOne_img_name = os.path.join(self.img_dir, 'A' + str('{:06d}'.format(self.img_count)) + '.jpg')
        while os.path.exists(cameraOne_img_name):
            self.img_count += 1
            cameraOne_img_name = os.path.join(self.img_dir, 'A' + str('{:06d}'.format(self.img_count)) + '.jpg')
        print 'saving image', self.img_count
        cameraTwo_img_name = os.path.join(self.img_dir, 'B' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraThree_img_name = os.path.join(self.img_dir, 'C' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraFour_img_name = os.path.join(self.img_dir, 'D' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraAll_img_name = os.path.join(self.img_dir, 'E' + str('{:06d}'.format(self.img_count)) + '.jpg')
        car_pose_name = os.path.join(self.img_dir, 'car_pos' + str('{:06d}'.format(self.img_count)) + '.yaml')

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

        car_pose_stream = open(car_pose_name, "w")
        car_data = {'car_pos': self.car_pos.tolist(), 'car_ori': self.car_ori.tolist()}
        yaml.dump(car_data, car_pose_stream)

        self.img_count += 1
        self.car_speed_pub(0.15,0)
        time.sleep(4)
        self.car_speed_pub(0.0,0)

        if self.img_count > 10000:
            sys.exit()


if __name__ == "__main__":
    rospy.init_node('save_pics')
    cv2.startWindowThread()
    ss = saver()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # raw_input('save ?')
        ss.save_pics()

