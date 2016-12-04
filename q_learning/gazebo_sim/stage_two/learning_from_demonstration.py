# -----------------------------------
# Let the car (agent) learn from demonstration
# Author: Tao Chen
# Date: 2016.11.08
# -----------------------------------
import rospy
import numpy as np
import os
import sys, tty, termios
from collections import namedtuple
from environment import Environment
states = namedtuple('states','x y theta')
import cPickle
import re

class LfD(object):
    def __init__(self):
        self.grid_width = 0.1
        self.angle_blockwidth = np.pi / 8
        self.env = Environment()
        self.state_action = []
        self.tmp_state_action = []
        self.cur_state = None
        self.cur_action = None


    def set_action(self, action):
        self.env.set_agent_velocity(self.env.valid_actions_dict[action])
        self.env.env_sleep(self.env.step_length / self.env.speed)
        self.env.set_agent_velocity(self.env.valid_actions_dict['stop'])
        self.env.env_sleep(self.env.step_length / self.env.speed)

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


    def save_tmp_state_action(self, action):
        agent_pose = self.env.sense()
        self.cur_state = states(x=agent_pose[0], y=agent_pose[1], theta=agent_pose[2])
        self.cur_action = action
        if (self.cur_state, self.cur_action) not in self.tmp_state_action:
            self.tmp_state_action.append((self.cur_state, self.cur_action))

    def save_state_action(self):
        for item in self.tmp_state_action:
            if item not in self.state_action:
                self.state_action.append(item)



    def store_to_file(self):
        data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'LfD')

        if not os.path.exists(data_path):
            os.makedirs(data_path)
        files_lst = os.listdir(data_path)
        max_index = 0

        for filename in files_lst:
            fileindex_list = re.findall(r'\d+', filename)
            if not fileindex_list:
                continue
            fileindex = int(fileindex_list[0])
            if fileindex >= max_index:
                max_index = fileindex

        q_table_file = os.path.join(data_path, 'lfd' + str('{:07d}'.format(max_index + 1) + '.cpickle'))
        with open(q_table_file, 'wb') as f:
            cPickle.dump(self.state_action, f, protocol=cPickle.HIGHEST_PROTOCOL)
        print 'saving path to file done...'




    def read_key(self):
        while not rospy.is_shutdown():
            ch = self.getch()
            if ch == '\x1b':
                ch = self.getch()
                if ch == '[':
                    ch = self.getch()
                    if ch == 'A':
                        print 'forward'
                        self.save_tmp_state_action('forward')
                        self.set_action('forward')
                    elif ch == 'B':
                        print 'backward'
                        self.save_tmp_state_action('backward')
                        self.set_action('backward')
                    elif ch == 'C':
                        print 'right_45_backward'
                        self.save_tmp_state_action('right_45_backward')
                        self.set_action('right_45_backward')
                    elif ch == 'D':
                        print 'left_45_backward'
                        self.save_tmp_state_action('left_45_backward')
                        self.set_action('left_45_backward')
            elif ch == ' ':
                print 'stop'
                self.set_action('stop')
            elif ch == 'a':
                print 'left_45_forward'
                self.save_tmp_state_action('left_45_forward')
                self.set_action('left_45_forward')
            elif ch == 'd':
                print 'right_45_forward'
                self.save_tmp_state_action('right_45_forward')
                self.set_action('right_45_forward')
            elif ch == 'r':
                print 'reset'
                self.tmp_state_action[:] = []
                self.env.reset()
            elif ch == 's':
                print 'save the path you just ran'
                self.save_state_action()
            elif ch == 'z':
                print 'save paths to file'
                self.store_to_file()
            elif ch == 'c':
                print 'clear the previous paths you just ran'
                self.state_action[:] = []
                print 'you can start recording the path from scratch'
            elif ch == '\x03' or ch == '\x71':  # ctrl + c or 'q'
                rospy.signal_shutdown(ch)
                sys.exit()
            else:
                print ord(ch)


if __name__ == "__main__":
    rospy.init_node('LfD')
    LfD = LfD()
    rate = rospy.Rate(500)
    print '==================================================='
    print 'please press the following keys to control the car:'
    print '    A: left_45_forward'
    print '    D: right_45_forward'
    print '    Up arrow: forward'
    print '    Down arrow: backward'
    print '    Left arrow: left_45_backward'
    print '    Right arrow: right_45_backward'
    print '    R: reset'
    print '    Space: stop'
    print '    S: save path you just ran (This doesn\'t save path to file)'
    print '    C: clear saved paths'
    print '    Z: save paths to file'
    print '    Q: quit'
    print '==================================================='
    LfD.read_key()
