# -----------------------------------
# Let the car (agent) learn from demonstration
# Author: Tao Chen
# Date: 2016.11.08
# -----------------------------------
import rospy
import os
import sys, tty, termios
from environment import Environment
import re
import yaml
import cv2
import shutil

class LfD(object):
    def __init__(self):
        self.env = Environment()
        self.state_action = []
        self.tmp_state_action = []
        self.cur_state = None
        self.cur_action = None

        self.current_path = os.path.dirname(os.path.realpath(__file__))

        self.img_dir = os.path.join(self.current_path, 'middle_region')
        self.front_cam_dir = os.path.join(self.img_dir, 'front')
        self.right_cam_dir = os.path.join(self.img_dir, 'right')
        self.back_cam_dir = os.path.join(self.img_dir, 'back')
        self.left_cam_dir = os.path.join(self.img_dir, 'left')
        self.action_dir = os.path.join(self.img_dir, 'action')

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

        if not os.path.exists(self.action_dir):
            os.makedirs(self.action_dir)

        self.tmp_img_dir = os.path.join(self.current_path, 'tmp_middle_region')
        self.tmp_front_cam_dir = os.path.join(self.tmp_img_dir, 'front')
        self.tmp_right_cam_dir = os.path.join(self.tmp_img_dir, 'right')
        self.tmp_back_cam_dir = os.path.join(self.tmp_img_dir, 'back')
        self.tmp_left_cam_dir = os.path.join(self.tmp_img_dir, 'left')
        self.tmp_action_dir = os.path.join(self.tmp_img_dir, 'action')

        if not os.path.exists(self.tmp_img_dir):
            os.makedirs(self.tmp_img_dir)

        if not os.path.exists(self.tmp_front_cam_dir):
            os.makedirs(self.tmp_front_cam_dir)

        if not os.path.exists(self.tmp_right_cam_dir):
            os.makedirs(self.tmp_right_cam_dir)

        if not os.path.exists(self.tmp_back_cam_dir):
            os.makedirs(self.tmp_back_cam_dir)

        if not os.path.exists(self.tmp_left_cam_dir):
            os.makedirs(self.tmp_left_cam_dir)

        if not os.path.exists(self.tmp_action_dir):
            os.makedirs(self.tmp_action_dir)

        self.img_count = 0

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
        cameraFront_img_name = os.path.join(self.tmp_front_cam_dir, 'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        while os.path.exists(cameraFront_img_name):
            self.img_count += 1
            cameraFront_img_name = os.path.join(self.tmp_front_cam_dir, 'F' + str('{:06d}'.format(self.img_count)) + '.jpg')
        print 'saving image', self.img_count
        cameraRight_img_name = os.path.join(self.tmp_right_cam_dir, 'R' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraBack_img_name = os.path.join(self.tmp_back_cam_dir, 'B' + str('{:06d}'.format(self.img_count)) + '.jpg')
        cameraLeft_img_name = os.path.join(self.tmp_left_cam_dir, 'L' + str('{:06d}'.format(self.img_count)) + '.jpg')
        action_name = os.path.join(self.tmp_action_dir, 'action' + str('{:06d}'.format(self.img_count)) + '.yaml')
        self.img_count += 1

        front, right, back, left = self.env.sense()
        cv2.imwrite(cameraFront_img_name, front)
        cv2.imwrite(cameraRight_img_name, right)
        cv2.imwrite(cameraBack_img_name, back)
        cv2.imwrite(cameraLeft_img_name, left)
        with open(action_name, 'w') as f:
            action_data = {'act':self.env.valid_actions.index(action), 'act_name:':action}
            yaml.dump(action_data, f)

    def get_max_index(self, path):
        files_lst = os.listdir(path)
        max_index = -1
        for filename in files_lst:
            fileindex_list = re.findall(r'\d+', filename)
            if not fileindex_list:
                continue
            fileindex = int(fileindex_list[0])
            if fileindex >= max_index:
                max_index = fileindex
        return max_index

    def store_to_file(self):
        dest_count = self.get_max_index(self.front_cam_dir) + 1

        src_count = self.get_max_index(self.tmp_front_cam_dir) + 1
        foldername_list = os.listdir(self.tmp_img_dir)
        for i in range(src_count):
            for fld_name in foldername_list:
                src_folder_dir = os.path.join(self.tmp_img_dir, fld_name)
                dest_folder_dir = os.path.join(self.img_dir, fld_name)

                files_lst = os.listdir(src_folder_dir)
                first_filename = files_lst[0]
                fileindex_list = re.findall(r'\d+', files_lst[0])
                index_start = first_filename.index(fileindex_list[0])

                file_prefix = first_filename[:index_start]
                file_suffix = first_filename[index_start + len(fileindex_list[0]):]

                dest_file_name = file_prefix + str('{:06d}'.format(dest_count + i)) + file_suffix
                src_file_name = file_prefix + str('{:06d}'.format(i)) + file_suffix
                if not os.path.exists(os.path.join(src_folder_dir, src_file_name)):
                    continue
                shutil.move(os.path.join(src_folder_dir, src_file_name), os.path.join(dest_folder_dir, dest_file_name))

    def delete_tmp_files(self):
        folders = os.listdir(self.tmp_img_dir)
        for folder in folders:
            filelists = os.listdir(os.path.join(self.tmp_img_dir, folder))
            for file in filelists:
                os.remove(os.path.join(self.tmp_img_dir, folder, file))
        print 'deleting files done...'


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
                        self.env.act('forward')
                    elif ch == 'B':
                        print 'backward'
                        self.save_tmp_state_action('backward')
                        self.env.act('backward')
                    elif ch == 'C':
                        print 'right_45_backward'
                        self.save_tmp_state_action('right_45_backward')
                        self.env.act('right_45_backward')
                    elif ch == 'D':
                        print 'left_45_backward'
                        self.save_tmp_state_action('left_45_backward')
                        self.env.act('left_45_backward')
            elif ch == ' ':
                print 'stop'
                self.env.act('stop')
            elif ch == 'a':
                print 'left_45_forward'
                self.save_tmp_state_action('left_45_forward')
                self.env.act('left_45_forward')
            elif ch == 'd':
                print 'right_45_forward'
                self.save_tmp_state_action('right_45_forward')
                self.env.act('right_45_forward')
            elif ch == 'r':
                print 'reset'
                self.img_count = 0
                self.delete_tmp_files()
                self.env.reset()
            elif ch == 's':
                print 'save paths to file'
                self.save_tmp_state_action('stop')
                self.store_to_file()
            elif ch == 'c':
                print 'clear the previous paths you just ran'
                self.delete_tmp_files()
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
    print '    Q: quit'
    print '==================================================='
    LfD.read_key()
