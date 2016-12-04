# -----------------------------------
# car parking simulation environment in Gazebo
# Author: Tao Chen
# Date: 2016.11.08
# -----------------------------------
import time
import random
import cv2
import rospy
import numpy as np
import threading
import message_filters
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkStates
import threading
from datetime import datetime
import tools
import math
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class Environment(object):
    """Environment within which all agents operate"""

    valid_actions = ['stop', 'forward', 'backward', 'left_45_forward', 'right_45_forward', 'left_45_backward', 'right_45_backward']
    step_length = 0.1
    speed = 0.1
    valid_actions_dict = {valid_actions[0]:np.array([0.0,0.0]), \
                          valid_actions[1]:np.array([speed,0.0]), \
                          valid_actions[2]:np.array([-speed,0.0]), \
                          valid_actions[3]:np.array([speed,0.785]), \
                          valid_actions[4]:np.array([speed,-0.785]), \
                          valid_actions[5]:np.array([-speed,0.785]), \
                          valid_actions[6]:np.array([-speed,-0.785])}  # np.array([speed, angle])


    def __init__(self):
        self.car_length = 0.55
        self.car_width = 0.3889
        self.car_diagonal_length = math.sqrt(self.car_width ** 2 + self.car_length ** 2)
        self.br = CvBridge()
        self.im_show = False
        self.cameraFront_img = None
        self.cameraRight_img = None
        self.cameraBack_img = None
        self.cameraLeft_img = None
        if self.im_show:
            self.cameraFront_window = 'Front'
            self.cameraRight_window = 'rightSide'
            self.cameraBack_window = 'Back'
            self.cameraLeft_window = 'leftSide'
            cv2.namedWindow(self.cameraFront_window)
            cv2.namedWindow(self.cameraRight_window)
            cv2.namedWindow(self.cameraBack_window)
            cv2.namedWindow(self.cameraLeft_window)

        self.lock = threading.Lock()

        self.wall = None
        self.wall_center = np.array([0.469, 0])
        self.wall_edge_length = 8.0
        self.wall_verts = self.get_rect_verts(self.wall_center, self.wall_edge_length, self.wall_edge_length, angle=0.0)
        self.car1_center = np.array([0.7, -2.0])
        self.car1_verts = self.get_rect_verts(self.car1_center, self.car_length, self.car_width, angle=0.0)

        self.car2_center = np.array([-0.7, -2.0])
        self.car2_verts = self.get_rect_verts(self.car2_center, self.car_length, self.car_width, angle=0.0)

        self.agent_speed = 0.0
        self.agent_dir = 0.0
        self.agent_pos = np.zeros(3)

        self.set_agent_start_region()

        self.reward_db = []

        self.cameraFront_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler/left/image_raw",
                                                          Image)
        self.cameraRight_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler2/right/image_raw",
                                                          Image)
        self.cameraBack_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler3/right/image_raw",
                                                         Image)
        self.cameraLeft_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler4/left/image_raw",
                                                         Image)

        self.sync = message_filters.ApproximateTimeSynchronizer([self.cameraFront_sub, self.cameraRight_sub, \
                                                                 self.cameraBack_sub, self.cameraLeft_sub], \
                                                                queue_size=1, slop=0.3)

        self.sync.registerCallback(self.sync_callback)
        self.agent_speed_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.link_sub = rospy.Subscriber("/ackermann_vehicle/gazebo/link_states", LinkStates, self.link_callback)
        time.sleep(1)


        self.agent_speed_pub_thread = threading.Thread(name="agent_speed", target=self.car_speed_pub)
        self.agent_speed_pub_thread.start()


        # self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)

    def get_rect_verts(self, center, length, width, angle):
        rotation_mtx = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        half_length = length / 2.0
        half_width = width / 2.0
        verts = np.array([[-half_length, half_width],   #top left
                          [half_length, half_width],    #top right
                          [half_length, -half_width],   #bottom right
                          [-half_length, -half_width]]) #bottom left
        verts_rot = np.dot(rotation_mtx, verts.T)
        verts_trans = verts_rot.T + center.reshape((1,2))
        return verts_trans.reshape((4,2))


    def reset(self):
        self.done = False
        self.t = 0

        # time.sleep(0.5)
        x, y, z, theta = self.generate_agent_pose()
        self.reset_world(x,y,theta)
        # self.set_agent_pose(x, y, z, theta)
        print '    agent starting pose:', x, y, theta

    def act(self, action):
        self.set_agent_velocity(self.valid_actions_dict[action])
        time.sleep(self.step_length / self.speed)
        self.set_agent_velocity(self.valid_actions_dict['stop'])


    def sense(self):
        return self.cameraFront_img, self.cameraRight_img, self.cameraBack_img, self.cameraLeft_img



    def set_agent_velocity(self, velocity):

        self.lock.acquire()
        self.agent_speed = 0.0
        self.agent_dir = velocity[1]
        self.lock.release()
        time.sleep(0.1)

        self.lock.acquire()
        self.agent_speed = velocity[0]
        self.agent_dir = velocity[1]
        self.lock.release()

    def fixed_car_movement_check(self):
        car_movement_tolerance = 0.02
        try:
            rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
            get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)
            resp1 = get_link_state('ack_c1::base_link', '')
            car1_center = np.zeros(2)
            car1_center[0] = resp1.link_state.pose.position.x
            car1_center[1] = resp1.link_state.pose.position.y

            resp2 = get_link_state('ack_c2::base_link', '')
            car2_center = np.zeros(2)
            car2_center[0] = resp2.link_state.pose.position.x
            car2_center[1] = resp2.link_state.pose.position.y


            car1_movement = np.linalg.norm(self.car1_center - car1_center)
            car2_movement = np.linalg.norm(self.car2_center - car2_center)
            if car1_movement > car_movement_tolerance or car2_movement > car_movement_tolerance:
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                print "Agent hit the cars"
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return False


    def reset_world(self,x,y,theta):
        try:
            rospy.wait_for_service('/ackermann_vehicle/gazebo/set_model_state')
            set_model_pose = rospy.ServiceProxy('/ackermann_vehicle/gazebo/set_model_state', SetModelState)
            state = ModelState()
            pose = Pose()

            # Fixed Car 1
            pose.position.x = 0.7
            pose.position.y = -2
            pose.position.z = 0.1
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1.0

            state.model_name = 'ack_c1'
            state.pose = pose
            resp1 = set_model_pose(state)

            # Fixed Car 2
            pose.position.x = -0.7
            pose.position.y = -2
            pose.position.z = 0.1
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1.0

            state.model_name = 'ack_c2'
            state.pose = pose
            resp2 = set_model_pose(state)

            # Agent Car
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.105
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = np.sin(theta / 2.0)
            pose.orientation.w = np.cos(theta / 2.0)

            state.model_name = 'ackermann_vehicle'
            state.pose = pose
            resp3 = set_model_pose(state)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e


    def car_speed_pub(self):
        try:
            while True:
                msg = AckermannDriveStamped()
                msg.drive.speed = self.agent_speed
                msg.drive.steering_angle = self.agent_dir
                self.agent_speed_pub.publish(msg)
        except rospy.ROSException, e:
            print 'ROS publish error: ',e



    def set_agent_start_region(self):
        self.agent_start_region = np.zeros(4)
        self.agent_start_region[0] = 0#self.wall_verts[0,0] #-0.2#-1.75
        self.agent_start_region[1] = 0#self.wall_verts[1,0] #0.2 # 1.75
        self.agent_start_region[2] = -1.2#self.wall_verts[-1,1] #-1.5#-3.15
        self.agent_start_region[3] = -1.2#self.wall_verts[0,1] #-1.0#-0.85

    def generate_agent_pose(self):
        random.seed(datetime.now())
        while True:
            x = random.uniform(self.agent_start_region[0], self.agent_start_region[1])
            y = random.uniform(self.agent_start_region[2], self.agent_start_region[3])
            theta = np.pi * 3 / 2
            # theta = random.uniform(0, 2 * np.pi)
            z = 0.105
            if x < self.car1_verts[1,0] and x > self.car2_verts[0,0] \
                and y < self.car1_verts[0,1] and y > self.car1_verts[-1,1]:
                continue
            elif self.collide_walls_with_pose(np.array([x, y ,theta])):
                continue
            elif self.collide_fixed_cars_with_pose(np.array([x, y ,theta])):
                continue
            else:
                break

        return np.array([x, y, z, theta])

    def link_callback(self, data):
        name_list = data.name
        car_link_index = name_list.index('ackermann_vehicle::base_link')
        self.lock.acquire()
        self.agent_pos[0] = data.pose[car_link_index].position.x
        self.agent_pos[1] = data.pose[car_link_index].position.y
        self.agent_pos[2] = data.pose[car_link_index].position.z
        # print data.pose[car_link_index].position.x, data.pose[car_link_index].position.y
        self.lock.release()

    def collide_fixed_cars_with_pose(self, pose):
        agent_center = pose[:2]
        verts = self.get_rect_verts(pose[:2], self.car_length, self.car_width, pose[2])
        agent_center_to_car1_center = np.linalg.norm(agent_center - self.car1_center)
        agent_center_to_car2_center = np.linalg.norm(agent_center - self.car2_center)
        car1_collision = False
        car2_collision = False
        if agent_center_to_car1_center > self.car_diagonal_length:
            # in this case, agent is not possible to collide with car1
            car1_collision = False
        else:
            car1_collision = tools.two_rects_intersect(verts, self.car1_verts)

        if not car1_collision:
            if agent_center_to_car2_center > self.car_diagonal_length:
                car2_collision = False
            else:
                car2_collision = tools.two_rects_intersect(verts, self.car2_verts)

        if car1_collision or car2_collision:
            return True
        else:
            return False

    def collide_walls(self):
        pose = self.agent_pos.copy()
        verts = self.get_rect_verts(pose[:2], self.car_length, self.car_width, pose[2])
        wall_collision = tools.two_rects_intersect(verts, self.wall_verts)
        out_of_wall = False
        if pose[0] > self.wall_verts[0, 0] and pose[0] < self.wall_verts[1, 0] \
                and pose[1] > self.wall_verts[2, 1] and pose[1] < self.wall_verts[1, 1]:
            out_of_wall = False
        else:
            out_of_wall = True
        return wall_collision or out_of_wall

    def collide_walls_with_pose(self, pose):
        verts = self.get_rect_verts(pose[:2], self.car_length, self.car_width, pose[2])
        wall_collision = tools.two_rects_intersect(verts, self.wall_verts)
        out_of_wall = False
        if pose[0] > self.wall_verts[0, 0] and pose[0] < self.wall_verts[1, 0] \
                and pose[1] > self.wall_verts[2, 1] and pose[1] < self.wall_verts[1, 1]:
            out_of_wall = False
        else:
            out_of_wall = True
        return wall_collision or out_of_wall



    def set_agent_pose(self, x, y, z, theta):

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
            self.car_speed_pub(np.zeros(2))
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def sync_callback(self, frontImage, rightImage, backImage, leftImage):
        self.lock.acquire()
        self.cameraFront_callback(frontImage)
        self.cameraRight_callback(rightImage)
        self.cameraBack_callback(backImage)
        self.cameraLeft_callback(leftImage)
        print '..'
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



    def clock_callback(self, data):

        # self.lock.acquire()
        secs = data.clock.secs
        nsecs = data.clock.nsecs
        self.cur_time = secs + nsecs / float(1000000000)
        # self.lock.release()

    def env_sleep(self, int_time):
        # time, unit (s)
        old_time = self.cur_time
        while self.cur_time - old_time < int_time:
            continue




class Agent(object):
    """Base class for all agents."""

    def __init__(self, env):
        self.env = env
        self.state = None


    def reset(self, destination=None):
        pass

    def update(self, t):
        pass

    def get_state(self):
        return self.state




