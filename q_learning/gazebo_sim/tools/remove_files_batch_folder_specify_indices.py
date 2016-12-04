#!/usr/bin/env python
# -----------------------------------
# remove files that contain the specified indices in each subfolder in the given path
# Author: Tao Chen
# Date: 2016.10.16
# -----------------------------------

import os
import sys
import numbers
import re

def rm_riles(path, indices):
    if not os.path.exists(path):
        print 'path you entered did not exist.'
        return

    folder_lst = os.listdir(path)
    for foldername in folder_lst:
        folder_dir = os.path.join(path, foldername)
        files_lst = os.listdir(folder_dir)
        for filename in files_lst:
            fileindex_list = re.findall(r'\d+', filename)
            fileindex = int(fileindex_list[0])
            if fileindex in indices:
                try:
                    os.remove(os.path.join(folder_dir,filename))
                except OSError, e:
                    print 'remove file error'
                    print e

    print 'deleting files done...'



if __name__ == "__main__":
    dir = raw_input("please enter the full path of the directory: ")
    if dir == '\x03' or dir == '\x71':  # ctrl + c or 'q'
        sys.exit()


    index_inputed = False
    int_indices = []
    input_ok = True
    while not index_inputed:
        input = raw_input("please enter the indices of the file that you wanna delete: ")
        indices = input.split()
        for index in indices:
            if index == '\x03' or index == '\x71':  # ctrl + c or 'q'
                sys.exit()
            try:
                index = eval(index)
                if not isinstance(index, numbers.Integral):
                    print "indices contain non-integral number"
                    print "Please enter them again..."
                    input_ok = False
                    break
                else:
                    input_ok = True
                    int_indices.append(index)
            except NameError, e:
                print "You did not enter numbers, please try again"

        if input_ok:
            index_inputed = True

    rm_riles(dir, int_indices)