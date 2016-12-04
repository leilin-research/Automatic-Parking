# -----------------------------------
# car parking simulation environment in Gazebo
# Author: Tao Chen
# Date: 2016.11.12
# -----------------------------------
import time
import random
import cv2
import rospy
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkStates
import threading
import tools



class Environment(object):
    """Environment within which all agents operate"""

    valid_actions = ['forward', 'backward', 'left_45_forward', 'right_45_forward', 'left_45_backward',
                     'right_45_backward']
    step_length = 0.1
    speed = 0.1
    angle = 0.785
    valid_actions_dict = {valid_actions[0]: np.array([speed, 0.0]), \
                          valid_actions[1]: np.array([-speed, 0.0]), \
                          valid_actions[2]: np.array([speed, angle]), \
                          valid_actions[3]: np.array([speed, -angle]), \
                          valid_actions[4]: np.array([-speed, angle]), \
                          valid_actions[5]: np.array([-speed, -angle])}  # np.array([speed, angle])
    hard_time_limit = 200  # even if enforce_deadline is False, end trial when deadline reaches this value (to avoid deadlocks)


    def __init__(self):
        self.done = False
        self.enforce_deadline = False

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
        self.angle_blockwidth = np.pi / 8

        self.wall = None
        self.car_length = 0.55
        self.car_width = 0.3889
        self.car_diagonal_length = math.sqrt(self.car_width ** 2 + self.car_length ** 2)
        self.agent_pos = np.zeros(3)
        self.agent_ori = np.zeros(4)

        self.get_wall_boundary()
        self.get_fixed_car_boundary()
        self.set_agent_appro_move_boundary()
        self.get_terminal_pose()
        self.get_stage_one_terminal()
        self.get_stage_two_terminal()
        self.car1_verts = self.get_rect_verts(self.car1_center[:2], self.car_length, self.car_width, angle=0.0)
        self.car2_verts = self.get_rect_verts(self.car2_center[:2], self.car_length, self.car_width, angle=0.0)

        self.succ_times = 0
        self.num_hit_time_limit = 0
        self.num_out_of_time = 0
        self.hit_wall_times = 0
        self.hit_car_times = 0
        self.agent_speed = 0.0
        self.agent_dir = 0.0
        self.cur_time = 0.0
        self.has_finished_stage_two = False
        self.region_idx = 1  # 0:close to terminal, 1:near terminal, stage two , 2:far away from terminal, stage one
        self.to_terminal_idx = 0 # 0:bottom_left  1:bottom_right   2:top_right    3:top_left
        self.reward_db = []


        self.agent_speed_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.link_sub = rospy.Subscriber("/ackermann_vehicle/gazebo/link_states", LinkStates, self.link_callback)
        self.agent_speed_pub_thread = threading.Thread(name="agent_speed", target=self.car_speed_pub)
        self.agent_speed_pub_thread.start()

        time.sleep(1)

    def set_agent(self, agent, enforce_deadline=False):
        self.agent = agent
        self.enforce_deadline = enforce_deadline

    def create_agent(self, agent_class, *args, **kwargs):
        agent = agent_class(self, *args, **kwargs)
        return agent

    def reset(self):
        self.done = False
        self.t = 0

        x, y, z, theta = self.generate_agent_pose()
        self.reset_world(x,y,theta)
        self.to_terminal_idx = 0
        self.region_idx = 1
        self.has_finished_stage_two = False

        self.deadline = self.cal_deadline(x, y)
        print '   agent starting pose:', x, y, theta


    def get_deadline(self):
        return self.deadline

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

    def step(self):
        # Update agents
        self.agent.update()

        if self.done:
            return

        if self.agent is not None:
            if self.t >= self.hard_time_limit:
                print "Environment.step(): Primary agent hit hard time limit! Trial aborted."
                self.done = True
                self.num_hit_time_limit += 1

            elif self.enforce_deadline and self.t >= self.deadline:
                print "Environment.step(): Primary agent ran out of time! Trial aborted."
                self.done = True
                self.num_out_of_time += 1

            self.t += 1




    def sense(self):
        agent_pose = np.zeros(3) #[x, y, theta]
        agent_pose[0] = self.agent_pos[0]
        agent_pose[1] = self.agent_pos[1]
        print 'agent_pose:{:2f}, {:2f}'.format(agent_pose[0], agent_pose[1])
        agent_pose[2] = np.arctan2(self.agent_ori[2], self.agent_ori[3]) * 2
        agent_pose[2] = (agent_pose[2] + 2 * np.pi) % (2 * np.pi)
        if self.reach_stage_one_terminal(agent_pose) or self.has_finished_stage_two:
            grid_width = 0.1
            agent_pose[0] = np.floor((np.floor(agent_pose[0] / (grid_width / 2)) + 1) / 2) * grid_width
            agent_pose[1] = np.floor((np.floor(agent_pose[1] / (grid_width / 2)) + 1) / 2) * grid_width
            idx = np.floor(agent_pose[2] / (self.angle_blockwidth / 2))
            if idx % 2 == 0:
                idx = idx / 2
            else:
                idx = (idx + 1) / 2
            agent_pose[2] = idx % 16

            if self.reach_stage_two_terminal(agent_pose):
                self.has_finished_stage_two = True

            if not self.has_finished_stage_two:
                self.region_idx = 1
            else:
                self.region_idx = 0

        else:

            grid_width = 1.0
            agent_pose[0] = np.floor(agent_pose[0] / grid_width) * grid_width + 0.55 * grid_width
            agent_pose[1] = np.floor(agent_pose[1] / grid_width) * grid_width + 0.55 * grid_width
            idx = np.floor(agent_pose[2] / (self.angle_blockwidth / 2))
            if idx % 2 == 0:
                idx = idx / 2
            else:
                idx = (idx + 1) / 2
            agent_pose[2] = idx % 16

            self.region_idx = 2

        # agent_pose[2] represents the region the car's angle belongs to
        # [-11.25, 11.25) is region 0
        # [11.25, 33.75) is region 1
        # ...
        # [-33.75, -11.25) is region 15
        return agent_pose


    def act(self, agent, action):
        # self.car_speed_pub(self.valid_actions_dict[action])
        self.set_agent_velocity(self.valid_actions_dict[action])

        time.sleep(self.step_length / self.speed)
        self.set_agent_velocity(np.array([0,0]))
        reward = 0.0

        agent_pose = self.sense()
        if self.hit_wall_check(agent_pose):
            self.hit_wall_times += 1
            self.done = True
            reward = -20.0

        elif self.reach_terminal(agent_pose):
            if self.enforce_deadline:
                if self.t < self.hard_time_limit and self.t < self.deadline:
                    reward = 40.0
                    self.done = True
                    self.succ_times += 1
                    print '-----------------------------------------------------------------------------------------'
                    print '-----------------------------------------------------------------------------------------'
                    print '-----------------------------------------------------------------------------------------'
                    print '-----------------------------------------------------------------------------------------'
                    print "Environment.act(): Agent has reached destination!"
            else:
                if self.t < self.hard_time_limit:
                    reward = 40.0
                    self.done = True
                    self.succ_times += 1
                    print '-----------------------------------------------------------------------------------------'
                    print '-----------------------------------------------------------------------------------------'
                    print '-----------------------------------------------------------------------------------------'
                    print '-----------------------------------------------------------------------------------------'
                    print "Environment.act(): Agent has reached destination!"

        elif (agent.state, action) in self.reward_db:
            reward = 5.0
        elif self.fixed_car_movement_check():
            self.hit_car_times += 1
            self.done = True
            reward = -5.0
        return agent_pose, reward


    def set_agent_velocity(self, velocity):
        self.lock.acquire()
        self.agent_speed = 0.0
        self.agent_dir = velocity[1]
        self.lock.release()
        time.sleep(1.0)

        self.lock.acquire()
        self.agent_speed = velocity[0]
        self.agent_dir = velocity[1]
        self.lock.release()


    def cal_distance(self, pose_one, pose_two):
        distance = np.linalg.norm(pose_one - pose_two)
        return distance

    def cal_deadline(self, x, y):
        grid_width = 0.1
        dist = abs(x - self.terminal_xy[0]) / grid_width + abs(y - self.terminal_xy[1]) / grid_width
        dist = abs(x - self.terminal_xy[0]) / grid_width + abs(y - self.terminal_xy[1]) / grid_width
        deadline = max(int(dist) * 5,15)
        deadline = min(deadline, 150)
        return deadline

    def get_steps(self):
        return self.t


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
            print 'successful trials: ', self.succ_times
            print 'number of trials that hit the hard time limit: ', self.num_hit_time_limit
            print 'number of trials that ran out of time: ', self.num_out_of_time
            print 'number of trials that hit cars', self.hit_car_times
            print 'number of trials that hit walls', self.hit_wall_times


    def fixed_car_movement_check(self):
        car_movement_tolerance = 0.02
        try:
            rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
            get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)
            resp1 = get_link_state('ack_c1::base_link', '')
            car1_center = np.zeros(3)
            car1_center[0] = resp1.link_state.pose.position.x
            car1_center[1] = resp1.link_state.pose.position.y
            car1_center[2] = resp1.link_state.pose.position.z

            resp2 = get_link_state('ack_c2::base_link', '')
            car2_center = np.zeros(3)
            car2_center[0] = resp2.link_state.pose.position.x
            car2_center[1] = resp2.link_state.pose.position.y
            car2_center[2] = resp2.link_state.pose.position.z

            car1_movement = np.linalg.norm(self.car1_center - car1_center)
            car2_movement = np.linalg.norm(self.car2_center - car2_center)
            if car1_movement > car_movement_tolerance or car2_movement > car_movement_tolerance:
                print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                print "Agent hit the cars"
                return True
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return False

    def hit_wall_check(self, pose):
        # pose: [x, y, theta]
        near_wall_tolerance = 0.6 * self.car_length
        if pose[0] > self.wall_boundary[0] + near_wall_tolerance and pose[0] < self.wall_boundary[1] - near_wall_tolerance \
            and pose[1] > self.wall_boundary[2] + near_wall_tolerance and pose[1] < self.wall_boundary[3] - near_wall_tolerance:
            return False
        else:
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            print "Agent hit the wall"
            return True

    def get_terminal_pose(self):
        x_offset = 0.2
        y_offset = 0.2
        self.terminal_boundary = np.zeros(4)
        self.terminal_xy = np.zeros(2)

        self.terminal_xy[0] = 0.0
        self.terminal_xy[1] = -2.0

        self.terminal_boundary[0] = self.terminal_xy[0] - 1 * x_offset
        self.terminal_boundary[1] = self.terminal_xy[0] + 1 * x_offset
        self.terminal_boundary[2] = self.terminal_xy[1] - 1 * y_offset
        self.terminal_boundary[3] = self.terminal_xy[1] + 1 * y_offset

    def get_stage_two_terminal(self):
        x_offset = 0.049
        y_offset = 0.049
        self.stage_two_terminal_boundary = np.zeros((4, 4))
        self.stage_two_terminal_xy = np.zeros((4, 2))

        self.stage_two_terminal_xy[0][0] = 0.8
        self.stage_two_terminal_xy[0][1] = -1.4

        self.stage_two_terminal_xy[1][0] = -0.8
        self.stage_two_terminal_xy[1][1] = -1.4

        self.stage_two_terminal_xy[2][0] = -0.8
        self.stage_two_terminal_xy[2][1] = -2.6

        self.stage_two_terminal_xy[3][0] = 0.8
        self.stage_two_terminal_xy[3][1] = -2.6


        for i in range(4):
            self.stage_two_terminal_boundary[i][0] = self.stage_two_terminal_xy[i][0] - x_offset
            self.stage_two_terminal_boundary[i][1] = self.stage_two_terminal_xy[i][0] + x_offset
            self.stage_two_terminal_boundary[i][2] = self.stage_two_terminal_xy[i][1] - y_offset
            self.stage_two_terminal_boundary[i][3] = self.stage_two_terminal_xy[i][1] + y_offset

    def get_stage_one_terminal(self):

        self.stage_one_terminal_boundary = np.zeros(4)
        self.stage_one_terminal_xy = np.zeros(2)

        self.stage_one_terminal_xy[0] = 0.0
        self.stage_one_terminal_xy[1] = -2.0

        self.stage_one_terminal_boundary[0] = -1.75
        self.stage_one_terminal_boundary[1] = 1.75
        self.stage_one_terminal_boundary[2] = -3.15
        self.stage_one_terminal_boundary[3] =-0.85

    def within_range(self, value, min, max):
        if value >= min and value <= max:
            return True
        return False

    def reach_terminal(self, pose):
        if pose[0] > self.terminal_boundary[0] and pose[0] < self.terminal_boundary[1] \
                and pose[1] > self.terminal_boundary[2]  and pose[1] < self.terminal_boundary[3] \
                and (pose[2] == 0 or pose[2] == 8):
            return True
        return False

    def reach_stage_one_terminal(self, ori_pose):
        if ori_pose[0] > self.stage_one_terminal_boundary[0] and ori_pose[0] < self.stage_one_terminal_boundary[1] \
                and ori_pose[1] > self.stage_one_terminal_boundary[2]  and ori_pose[1] < self.stage_one_terminal_boundary[3]:
            return True
        return False

    def reach_stage_two_terminal(self, pose):
        x_offset = 0.0
        y_offset = 0.0
        if pose[0] >= self.stage_two_terminal_boundary[0][0] + x_offset and pose[0] <= self.stage_two_terminal_boundary[0][1] - x_offset \
                and pose[1] > self.stage_two_terminal_boundary[0][2] + y_offset and pose[1] < self.stage_two_terminal_boundary[0][
            3] - y_offset \
                and (pose[2] == 0):
            self.to_terminal_idx = 0
            return True

        elif pose[0] >= self.stage_two_terminal_boundary[1][0] + x_offset and pose[0] <= self.stage_two_terminal_boundary[1][1] - x_offset \
                and pose[1] >= self.stage_two_terminal_boundary[1][2] + y_offset and pose[1] <= self.stage_two_terminal_boundary[1][
            3] - y_offset \
                and (pose[2] == 8):
            self.to_terminal_idx = 1
            return True

        elif pose[0] >= self.stage_two_terminal_boundary[2][0] + x_offset and pose[0] <= self.stage_two_terminal_boundary[2][1] - x_offset \
                and pose[1] >= self.stage_two_terminal_boundary[2][2] + y_offset and pose[1] <= self.stage_two_terminal_boundary[2][
            3] - y_offset \
                and (pose[2] == 8):
            self.to_terminal_idx = 2
            return True

        elif pose[0] >= self.stage_two_terminal_boundary[3][0] + x_offset and pose[0] <= self.stage_two_terminal_boundary[3][1] - x_offset \
                and pose[1] >= self.stage_two_terminal_boundary[3][2] + y_offset and pose[1] <= self.stage_two_terminal_boundary[3][
            3] - y_offset \
                and (pose[2] == 0):
            self.to_terminal_idx = 3
            return True

        return False

    def generate_agent_pose(self):
        while True:
            x = random.uniform(self.agent_move_boundary[0][0], self.agent_move_boundary[0][1])
            y = random.uniform(self.agent_move_boundary[0][2], self.agent_move_boundary[0][3])
            z = 0.105
            # theta = np.pi
            theta = random.uniform(0, 2 * np.pi)
            if x < self.car1_verts[1,0] and x > self.car2_verts[0,0] \
                and y < self.car1_verts[0,1] and y > self.car1_verts[-1,1]:
                continue

            elif self.collide_fixed_cars_with_pose(np.array([x, y ,theta])):
                continue
            else:
                break
        return [x, y, z, theta]

    def collide_fixed_cars_with_pose(self, pose):
        agent_center = pose[:2]
        verts = self.get_rect_verts(pose[:2], self.car_length, self.car_width, pose[2])
        agent_center_to_car1_center = np.linalg.norm(agent_center - self.car1_center[:2])
        agent_center_to_car2_center = np.linalg.norm(agent_center - self.car2_center[:2])
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

    def set_agent_appro_move_boundary(self):

        self.agent_move_boundary = np.zeros((2, 4))  # xmin, xmax, ymin, ymax (Outer Suqare, Innder boundary square)


        self.agent_move_boundary[0][0] = self.wall_boundary[0] + self.car_length
        self.agent_move_boundary[0][1] = self.wall_boundary[1] - self.car_length
        self.agent_move_boundary[0][2] = self.wall_boundary[2] + self.car_length
        self.agent_move_boundary[0][3] = self.wall_boundary[3] - self.car_length

        self.agent_move_boundary[1][0] = 0
        self.agent_move_boundary[1][1] = 0
        self.agent_move_boundary[1][2] = 0
        self.agent_move_boundary[1][3] = 0



    def link_callback(self, data):
        name_list = data.name
        car_link_index = name_list.index('ackermann_vehicle::base_link')
        self.lock.acquire()
        self.agent_pos[0] = data.pose[car_link_index].position.x
        self.agent_pos[1] = data.pose[car_link_index].position.y
        self.agent_pos[2] = data.pose[car_link_index].position.z
        self.agent_ori[0] = data.pose[car_link_index].orientation.x
        self.agent_ori[1] = data.pose[car_link_index].orientation.y
        self.agent_ori[2] = data.pose[car_link_index].orientation.z
        self.agent_ori[3] = data.pose[car_link_index].orientation.w
        self.lock.release()













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




