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
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import LinkStates
import threading
from rosgraph_msgs.msg import Clock


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
        self.grid_width = 1
        self.angle_blockwidth = np.pi / 8

        self.wall = None
        self.car_length = 0.55
        self.car_width = 0.3889
        self.agent_pos = np.zeros(3)
        self.agent_ori = np.zeros(4)
        self.get_wall_boundary()
        self.get_fixed_car_boundary()
        self.set_agent_appro_move_boundary()
        self.get_terminal_pose()
        self.succ_times = 0
        self.num_hit_time_limit = 0
        self.num_out_of_time = 0
        self.hit_wall_times = 0
        self.hit_car_times = 0
        self.agent_speed = 0.0
        self.agent_dir = 0.0
        self.cur_time = 0.0

        self.reward_db = []

        # self.cameraFront_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler/left/image_raw",
        #                                                   Image)
        # self.cameraRight_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler2/right/image_raw",
        #                                                   Image)
        # self.cameraBack_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler3/right/image_raw",
        #                                                  Image)
        # self.cameraLeft_sub = message_filters.Subscriber("/ackermann_vehicle/multi/camera/basler4/left/image_raw",
        #                                                  Image)

        # self.sync = message_filters.ApproximateTimeSynchronizer([self.cameraFront_sub, self.cameraRight_sub, \
        #                                                          self.cameraBack_sub, self.cameraLeft_sub], \
        #                                                         queue_size=15, slop=0.2)
        #
        # self.sync.registerCallback(self.sync_callback)
        self.agent_speed_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.link_sub = rospy.Subscriber("/ackermann_vehicle/gazebo/link_states", LinkStates, self.link_callback)
        # time.sleep(1)


        self.agent_speed_pub_thread = threading.Thread(name="agent_speed", target=self.car_speed_pub)
        self.agent_speed_pub_thread.start()


        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)


    def set_agent(self, agent, enforce_deadline=False):
        self.agent = agent
        self.enforce_deadline = enforce_deadline

    def create_agent(self, agent_class, *args, **kwargs):
        agent = agent_class(self, *args, **kwargs)
        return agent

    def reset(self):
        self.done = False
        self.t = 0

        # time.sleep(0.5)
        x, y, z, theta = self.generate_agent_pose()
        self.reset_world(x,y,theta)

        self.deadline = self.cal_deadline(x, y)
        # self.set_agent_pose(x, y, z, theta)
        print 'agent pose:', x, y, theta


    def get_deadline(self):
        return self.deadline


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
        if agent_pose[0] > self.terminal_boundary[0] and agent_pose[0] < self.terminal_boundary[1] \
            and agent_pose[1] > self.terminal_boundary[2] and agent_pose[1] < self.terminal_boundary[3]:
            grid_width = 0.1
            agent_pose[0] = np.floor((np.floor(agent_pose[0] / (grid_width / 2)) + 1) / 2) * grid_width
            agent_pose[1] = np.floor((np.floor(agent_pose[1] / (grid_width / 2)) + 1) / 2) * grid_width

        else:
            agent_pose[0] = np.floor(agent_pose[0] / self.grid_width) * self.grid_width + 0.55 * self.grid_width
            agent_pose[1] = np.floor(agent_pose[1] / self.grid_width) * self.grid_width + 0.55 * self.grid_width
        idx = np.floor(agent_pose[2] / (self.angle_blockwidth / 2))
        if idx % 2 == 0:
            idx = idx / 2
        else:
            idx = (idx + 1) / 2
        agent_pose[2] = idx % 16
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
        cur_pose = agent_pose[:2]
        prev_pose = np.array([agent.state.x, agent.state.y])

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

        elif self.approaching_terminal(prev_pose, cur_pose):
            reward = 2.0

        return agent_pose, reward

    def approaching_terminal(self, prev_pose, current_pose):
        cur_dist = self.cal_distance(current_pose, self.terminal_xy)
        if cur_dist > 0.3:
            pre_dist = self.cal_distance(prev_pose, self.terminal_xy)
            if pre_dist > cur_dist:
                return True
        return False

    def set_agent_velocity(self, velocity):
        self.lock.acquire()
        self.agent_speed = 0.0
        self.agent_dir = velocity[1]
        self.lock.release()
        self.env_sleep(1.0)

        self.lock.acquire()
        self.agent_speed = velocity[0]
        self.agent_dir = velocity[1]
        self.lock.release()



    def is_going_away(self, prev_pose, current_pose):
        radius = 2 * self.car_length

        cur_dist = self.cal_distance(current_pose, self.terminal_xy)
        if cur_dist > radius:
            pre_dist = self.cal_distance(prev_pose, self.terminal_xy)
            if pre_dist < cur_dist:
                return True

        return False


    def cal_distance(self, pose_one, pose_two):
        distance = np.linalg.norm(pose_one - pose_two)
        return distance




    def cal_deadline(self, x, y):
        dist = abs(x - self.terminal_xy[0]) / self.grid_width + abs(y - self.terminal_xy[1]) / self.grid_width
        dist = abs(x - self.terminal_xy[0]) / self.grid_width + abs(y - self.terminal_xy[1]) / self.grid_width
        deadline = max(int(dist) * 5,15)
        deadline = min(deadline, 40)

        return deadline


    def get_steps(self):
        return self.t


    def reset_world(self,x,y,theta):
        # rospy.wait_for_service('/ackermann_vehicle/gazebo/reset_world')
        # reset_world = rospy.ServiceProxy('/ackermann_vehicle/gazebo/reset_world', Empty)
        # resp = reset_world()

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





    def appro_collision_check(self, x, y):
        if x < self.agent_move_boundary[0][0] or x > self.agent_move_boundary[0][1] \
                or y < self.agent_move_boundary[0][2] or y > self.agent_move_boundary[0][3]:
            return True

        elif x > self.agent_move_boundary[1][0] and x < self.agent_move_boundary[1][1] \
                and y > self.agent_move_boundary[1][2] and y < self.agent_move_boundary[1][3]:
            return True
        return False

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
        self.terminal_boundary = np.zeros(4)
        self.terminal_xy = np.zeros(2)

        self.terminal_xy[0] = 0.0
        self.terminal_xy[1] = -2.0



        self.terminal_boundary[0] = -1.75
        self.terminal_boundary[1] = 1.75
        self.terminal_boundary[2] = -3.15
        self.terminal_boundary[3] =-0.85

        print self.terminal_boundary

    def within_range(self, value, min, max):
        if value >= min and value <= max:
            return True
        return False

    def reach_terminal(self, pose):
        if pose[0] > self.terminal_boundary[0] and pose[0] < self.terminal_boundary[1] \
                and pose[1] > self.terminal_boundary[2]  and pose[1] < self.terminal_boundary[3]:
            return True
        return False





    def generate_agent_pose(self):
        while True:
            x = random.uniform(self.agent_move_boundary[0][0], self.agent_move_boundary[0][1])
            y = random.uniform(self.agent_move_boundary[0][2], self.agent_move_boundary[0][3])
            z = 0.105
            # theta = np.pi
            theta = random.uniform(0, 2 * np.pi)
            if not self.appro_collision_check(x, y):
                break
        return [x, y, z, theta]



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



    def sync_callback(self, frontImage, rightImage, backImage, leftImage):
        print '....'
        self.lock.acquire()
        self.cameraFront_callback(frontImage)
        self.cameraRight_callback(rightImage)
        self.cameraBack_callback(backImage)
        self.cameraLeft_callback(leftImage)

        white_band = np.ones((self.cameraFront_img.shape[0], 20, 3)) * 255
        self.cameraAll = np.concatenate((self.cameraFront_img, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraRight_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraBack_img), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, white_band), axis=1)
        self.cameraAll = np.concatenate((self.cameraAll, self.cameraLeft_img), axis=1)

        rospy.wait_for_service('/ackermann_vehicle/gazebo/get_link_state')
        try:
            get_link_state = rospy.ServiceProxy('/ackermann_vehicle/gazebo/get_link_state', GetLinkState)
            resp = get_link_state('ackermann_vehicle::base_link', '')
            self.agent_pos[0] = resp.link_state.pose.position.x
            self.agent_pos[1] = resp.link_state.pose.position.y
            self.agent_pos[2] = resp.link_state.pose.position.z
            self.agent_ori[0] = resp.link_state.pose.orientation.x
            self.agent_ori[1] = resp.link_state.pose.orientation.y
            self.agent_ori[2] = resp.link_state.pose.orientation.z
            self.agent_ori[3] = resp.link_state.pose.orientation.w
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
        car_link_index = name_list.index('ackermann_vehicle::base_link')
        self.lock.acquire()
        self.agent_pos[0] = data.pose[car_link_index].position.x
        self.agent_pos[1] = data.pose[car_link_index].position.y
        self.agent_pos[2] = data.pose[car_link_index].position.z
        self.agent_ori[0] = data.pose[car_link_index].orientation.x
        self.agent_ori[1] = data.pose[car_link_index].orientation.y
        self.agent_ori[2] = data.pose[car_link_index].orientation.z
        self.agent_ori[3] = data.pose[car_link_index].orientation.w
        # print data.pose[car_link_index].position.x, data.pose[car_link_index].position.y
        self.lock.release()


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




